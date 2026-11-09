import unittest
from unittest import mock
import hashlib
import json
from pathlib import Path
import tempfile

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import (
    maximum_corner_speed, normal_loads, point_mass_view_of_planar,
)
from simplified_vehicle_models.solvers.envelope_driver import PedalPolicyContext
from simplified_vehicle_models.solvers.hybrid_setup import (build_hybrid_setup,
    HYBRID_EXECUTION_ALLOCATOR_ID, HYBRID_SETUP_VERSION)
from simplified_vehicle_models.solvers import forward_driver, hybrid_speed_plan
from simplified_vehicle_models.powertrain.energy_storage import StorageState
from simplified_vehicle_models.solvers.longitudinal_capability import CapabilityQuery
from simplified_vehicle_models.tires.unitire_simple import loaded_radius


class HybridSetupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.compiled = compile_formulation("models/formulations/iem24_murphy_real_14dof_simple.xml")
        cls.vehicle = cls.compiled.model.planar
        loads = normal_loads(point_mass_view_of_planar(cls.vehicle), 1.0, 0.0, 0.0)
        cls.radii = tuple(float(v) for v in loaded_radius(cls.vehicle.tire, loads))

    @staticmethod
    def source_hashes(*paths):
        return {str(p): hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in paths}

    def build(self, curvature, *, distance=None, initial=0.0, maximum=30.0, closed=False,
              storage_soc=None, wheel_radii=None, pedal_context=None, maximum_pedal_force=None,
              throttle_scale=2.5, lateral_margin=.9, brake_margin=.60, source_hashes=None):
        k = np.asarray(curvature, dtype=float)
        x = np.linspace(0., 20., len(k)) if distance is None else np.asarray(distance, dtype=float)
        force = float(self.vehicle.maximum_pedal_force)
        force = force if maximum_pedal_force is None else maximum_pedal_force
        context = pedal_context or PedalPolicyContext(1., 1., .02, force, .85, 1., 1.)
        soc = float(self.vehicle.drivetrain.powertrain.energy_storage.initial_state_of_charge if storage_soc is None else storage_soc)
        radii = self.radii if wheel_radii is None else wheel_radii
        hashes = self.source_hashes("models/formulations/iem24_murphy_real_14dof_simple.xml") if source_hashes is None else source_hashes
        return build_hybrid_setup(self.vehicle, distance_m=x, curvature_inv_m=k,
            maximum_speed_mps=maximum, initial_speed_mps=initial, storage_soc=soc,
            wheel_radii_m=radii, model_source_hashes=hashes,
            maximum_pedal_force_n=force, pedal_context=context, is_closed=closed,
            throttle_acceleration_scale=throttle_scale, lateral_margin=lateral_margin, brake_margin=brake_margin)

    def test_real_compiled_straight_setup_has_in_domain_plan(self):
        setup = self.build(np.zeros(5), maximum=20.)
        self.assertTrue(setup.plan.algorithm_version.startswith("hybrid-speed-plan-v"))
        self.assertEqual(HYBRID_EXECUTION_ALLOCATOR_ID, "independent-wheel-torque-v1")
        self.assertEqual(json.loads(setup.setup_identity_payload_json)["schema"], HYBRID_SETUP_VERSION)
        self.assertEqual(json.loads(setup.setup_identity_payload_json)["inputs"]["execution_allocator_identity"], HYBRID_EXECUTION_ALLOCATOR_ID)
        self.assertEqual(setup.policy.execution_allocator_identity, HYBRID_EXECUTION_ALLOCATOR_ID)
        self.assertEqual(setup.provider.speed_grid[0], 0.)
        self.assertEqual(setup.provider.soc_grid[1], self.vehicle.drivetrain.powertrain.energy_storage.initial_state_of_charge)
        self.assertEqual(setup.plan.speed_mps[0], 0.)
        self.assertTrue(np.all(setup.corner_speed_caps_mps >= setup.plan.speed_mps))
        soc = float(self.vehicle.drivetrain.powertrain.energy_storage.initial_state_of_charge)
        self.assertTrue(setup.provider.evaluate(CapabilityQuery(0., 0., soc)).in_domain)
        self.assertTrue(setup.provider.evaluate(CapabilityQuery(float(setup.plan.speed_mps[1]), 0., soc)).in_domain)
        self.assertTrue(setup.builder_query_log)
        self.assertTrue(all(row[3] for row in setup.builder_query_log))
        self.assertTrue(all(row[2] == self.vehicle.drivetrain.powertrain.energy_storage.initial_state_of_charge for row in setup.builder_query_log))
        self.assertGreater(len(setup.builder_query_log), 0)
        self.assertLessEqual(len(setup.provider.curvature_grid), 17)
        self.assertEqual(setup.provider.curvature_grid[0], min(setup.provider.curvature_grid))
        self.assertLessEqual(len(setup.provider.speed_grid), 17)

    def test_caps_are_pointwise_physical_caps_and_builder_runs_once(self):
        curvature = np.array([.005, -.03, .02, -.04, .003])
        expected = np.asarray([
            maximum_corner_speed(
                point_mass_view_of_planar(self.vehicle),
                .9 / max(abs(float(k)), 1. / 1.0e4),
                maximum_speed=20., storage_state=StorageState(
                    float(self.vehicle.drivetrain.powertrain.energy_storage.initial_state_of_charge)
                ),
            )
            for k in curvature
        ])
        with mock.patch.object(
            forward_driver, "build_speed_plan",
            side_effect=AssertionError("legacy reachability planner must not run"),
        ) as legacy, mock.patch.object(
            hybrid_speed_plan, "build_hybrid_speed_plan",
            wraps=hybrid_speed_plan.build_hybrid_speed_plan,
        ) as builder:
            setup = self.build(curvature, maximum=20.)
        legacy.assert_not_called()
        builder.assert_called_once()
        np.testing.assert_allclose(setup.physical_corner_speed_caps_mps, expected, rtol=0., atol=1e-8)
        self.assertTrue(np.all(setup.corner_speed_caps_mps <= expected + 1e-12))
        self.assertTrue(np.all(setup.corner_speed_caps_mps <= setup.capability_domain_speed_caps_mps + 1e-12))
        self.assertTrue(np.any(setup.capability_domain_speed_caps_mps < setup.physical_corner_speed_caps_mps))
        self.assertTrue(all(row[3] for row in setup.builder_query_log))
        self.assertAlmostEqual(setup.brake_predictor.domain[0], .005)

    def test_cap_arrays_are_bytes_backed_and_irreversibly_readonly(self):
        setup = self.build(np.array([-.04, -.03, -.02, -.01, -.005]), maximum=20.)
        for array in (setup.corner_speed_caps_mps, setup.physical_corner_speed_caps_mps,
                      setup.capability_domain_speed_caps_mps):
            self.assertFalse(array.flags.writeable)
            with self.assertRaises(ValueError):
                array[0] = -1.
            with self.assertRaises(ValueError):
                array.setflags(write=True)

    def test_setup_identity_repeatability_and_relevant_input_sensitivity(self):
        curvature = np.full(5, -.03)
        baseline = self.build(curvature, maximum=20.)
        repeat = self.build(curvature, maximum=20.)
        self.assertEqual(baseline.setup_identity, repeat.setup_identity)
        self.assertEqual(baseline.setup_identity_payload_json, repeat.setup_identity_payload_json)
        payload = json.loads(baseline.setup_identity_payload_json)
        self.assertEqual(payload["provider_identity"], baseline.provider.identity)
        self.assertEqual(payload["inputs"]["execution_allocator_identity"], HYBRID_EXECUTION_ALLOCATOR_ID)
        self.assertEqual(baseline.policy.provider_identity, baseline.provider.identity)
        self.assertEqual(baseline.policy.execution_allocator_identity, HYBRID_EXECUTION_ALLOCATOR_ID)
        self.assertEqual(payload["policy"]["slabs"], 30)
        self.assertEqual(payload["policy"]["max_extra_forecasts"], 8)
        self.assertEqual(payload["policy"]["bracket_resolution"], 1e-8)
        self.assertEqual(payload["policy"]["max_total_forecasts"], 11)
        self.assertEqual(payload["policy"]["max_total_transitions"], 330)
        self.assertIn("source_closure_hash", payload)
        changed = []
        changed.append(self.build(curvature, maximum=20., storage_soc=float(self.vehicle.drivetrain.powertrain.energy_storage.initial_state_of_charge)-.01).setup_identity)
        changed.append(self.build(curvature, maximum=20., wheel_radii=tuple(r*.999 for r in self.radii)).setup_identity)
        changed.append(self.build(curvature, maximum=20., maximum_pedal_force=float(self.vehicle.maximum_pedal_force)*.999).setup_identity)
        changed.append(self.build(curvature, maximum=20., throttle_scale=2.6).setup_identity)
        changed.append(self.build(curvature, maximum=21.).setup_identity)
        changed.append(self.build(curvature, maximum=20., brake_margin=.59).setup_identity)
        changed.append(self.build(curvature, maximum=20., lateral_margin=.89).setup_identity)
        changed.append(self.build(curvature, maximum=20., initial=.01).setup_identity)
        changed.append(self.build(curvature, maximum=20., closed=True).setup_identity)
        changed.append(self.build(curvature, maximum=20., pedal_context=PedalPolicyContext(1.1,1.,.02,float(self.vehicle.maximum_pedal_force),.85,1.,1.)).setup_identity)
        for identity in changed:
            self.assertNotEqual(identity, baseline.setup_identity)
    def test_source_hash_binds_actual_file_bytes(self):
        force = float(self.vehicle.maximum_pedal_force)
        context = PedalPolicyContext(1., 1., .02, force, .85, 1., 1.)
        with tempfile.TemporaryDirectory(dir=".") as tempdir:
            path = Path(tempdir) / "models/data/fixture.xml"
            path.parent.mkdir(parents=True)
            path.write_bytes(b"source-before")
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            path.write_bytes(b"source-after")
            from simplified_vehicle_models.solvers import longitudinal_capability
            source_dir=Path(tempdir)/"src/simplified_vehicle_models/solvers"
            source_dir.mkdir(parents=True)
            (source_dir/"fixture.py").write_bytes(b"fixture")
            old=longitudinal_capability.__file__
            try:
                longitudinal_capability.__file__=str(source_dir/"fixture.py")
                relative="models/data/fixture.xml"
                root=Path(tempdir)
                target=root/relative
                with self.assertRaisesRegex(ValueError, "model source hash mismatch"):
                    self.build(np.full(5, -.03), maximum=20., source_hashes={relative: digest},
                               pedal_context=context, maximum_pedal_force=force)
            finally:
                longitudinal_capability.__file__=old

    def test_real_sampled_signed_corner_plan_setup(self):
        setup = self.build(np.array([.005, .005, .005, .005, .005]), maximum=20.)
        self.assertEqual(setup.plan.speed_mps.shape, (5,))
        self.assertTrue(np.all(setup.plan.speed_mps <= setup.corner_speed_caps_mps + 1e-9))
        for speed, curvature in zip(setup.plan.speed_mps, setup.plan.curvature_inv_m):
            soc = float(self.vehicle.drivetrain.powertrain.energy_storage.initial_state_of_charge)
            self.assertTrue(setup.provider.evaluate(CapabilityQuery(float(speed), float(curvature), soc)).in_domain)

    def test_rejects_initial_state_above_physical_corner_cap(self):
        with self.assertRaisesRegex(ValueError, "initial speed exceeds valid capability-domain corner cap"):
            self.build(np.full(5, .1), initial=20., maximum=30.)

    def test_rejects_endpoint_soc(self):
        with self.assertRaisesRegex(ValueError, "interior storage SOC"):
            force = float(self.vehicle.maximum_pedal_force)
            build_hybrid_setup(self.vehicle, distance_m=np.linspace(0., 20., 5), curvature_inv_m=np.zeros(5),
                maximum_speed_mps=20., initial_speed_mps=0., storage_soc=0., wheel_radii_m=self.radii,
                model_source_hashes=self.source_hashes("models/formulations/iem24_murphy_real_14dof_simple.xml"), maximum_pedal_force_n=force,
                pedal_context=PedalPolicyContext(1., 1., .02, force, .85, 1., 1.))

    def test_source_hashes_are_required_and_rechecked_from_disk(self):
        with self.assertRaisesRegex(ValueError, "requires nonempty repository-rooted"):
            force = float(self.vehicle.maximum_pedal_force)
            build_hybrid_setup(self.vehicle, distance_m=np.linspace(0., 20., 5), curvature_inv_m=np.zeros(5),
                maximum_speed_mps=20., initial_speed_mps=0., storage_soc=float(self.vehicle.drivetrain.powertrain.energy_storage.initial_state_of_charge), wheel_radii_m=self.radii,
                model_source_hashes=None, maximum_pedal_force_n=force,
                pedal_context=PedalPolicyContext(1., 1., .02, force, .85, 1., 1.))
        hashes = self.source_hashes("models/formulations/iem24_murphy_real_14dof_simple.xml")
        hashes[next(iter(hashes))] = "0" * 64
        with self.assertRaisesRegex(ValueError, "model source hash mismatch"):
            force = float(self.vehicle.maximum_pedal_force)
            build_hybrid_setup(self.vehicle, distance_m=np.linspace(0., 20., 5), curvature_inv_m=np.zeros(5),
                maximum_speed_mps=20., initial_speed_mps=0., storage_soc=float(self.vehicle.drivetrain.powertrain.energy_storage.initial_state_of_charge), wheel_radii_m=self.radii,
                model_source_hashes=hashes, maximum_pedal_force_n=force,
                pedal_context=PedalPolicyContext(1., 1., .02, force, .85, 1., 1.))


if __name__ == "__main__":
    unittest.main()
