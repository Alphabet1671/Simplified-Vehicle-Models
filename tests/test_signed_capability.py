from pathlib import Path
import tempfile
import unittest

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import evaluate_equilibrium, point_mass_view_of_planar
from simplified_vehicle_models.formulations.signed_capability import (
    SignedTransientCapability, build_signed_transient_capability,
    load_signed_transient_capability, save_signed_transient_capability,
    signed_feasible_interval, symbolic_lookup,
)

ROOT = Path(__file__).resolve().parents[1]
FORMULATION = ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"


def _chart():
    return SignedTransientCapability(
        np.array([0., .5, 1.]), np.array([-.1, 0., .1]),
        np.array([[2., 2., 2.], [1., 1., 1.], [-2., -2., -2.]]),
        np.array([[-8., -8., -8.], [-7., -7., -7.], [-6., -6., -6.]]),
        np.array([15., 20., 15.]), .5, "fixture",
        {"scan_points": 101}, {"sampled": True},
    )


class SignedCapabilityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = point_mass_view_of_planar(compile_formulation(str(FORMULATION)).model)

    def test_live_signed_negative_upper_regressions(self):
        cases = ((33.33908018475074, 0., -0.05282909), (35.067452826, 0., -4.0512410))
        for speed, curvature, expected_upper in cases:
            with self.subTest(speed=speed):
                self.assertFalse(evaluate_equilibrium(self.vehicle, speed, curvature, 0.).is_feasible)
                interval = signed_feasible_interval(
                    lambda acceleration: evaluate_equilibrium(self.vehicle, speed, curvature, acceleration),
                    acceleration_min=-30., acceleration_max=10., scan_points=401, refinement_steps=30,
                )
                self.assertIsNotNone(interval)
                self.assertAlmostEqual(interval[1], expected_upper, delta=4.e-3)
                self.assertLess(interval[1], 0.)

    def test_live_curved_invalid_cell_is_excluded(self):
        curvature = -.0606381561585
        invalid = signed_feasible_interval(
            lambda acceleration: evaluate_equilibrium(self.vehicle, 17.495664306, curvature, acceleration),
            acceleration_min=-30., acceleration_max=10., scan_points=401, refinement_steps=24,
        )
        valid = signed_feasible_interval(
            lambda acceleration: evaluate_equilibrium(self.vehicle, 16.5, curvature, acceleration),
            acceleration_min=-30., acceleration_max=10., scan_points=401, refinement_steps=24,
        )
        self.assertIsNone(invalid)
        self.assertIsNotNone(valid)
        self.assertAlmostEqual(valid[0], -2.42649, delta=.01)
        self.assertAlmostEqual(valid[1], .47861, delta=.01)

    def test_real_builder_conservative_dense_validation(self):
        chart = build_signed_transient_capability(
            self.vehicle, curvature=np.array([-.01, 0., .01]),
            u=np.linspace(0., 1., 9), maximum_speed=35.067452826,
            scan_points=161, refinement_steps=20, speed_scan_points=61,
            interior_subdivisions=2, endpoint_buffer=.002, domain_backoff=.002,
            refinement_attempts=0,
        )
        self.assertGreater(chart.validation["construction_stencil_count"], 0)
        self.assertGreater(chart.validation["validation_stencil_count"], 0)
        self.assertLess(chart.limits(chart.domain_speed(0.), 0.)[1], 0.)
        # Independent stencil: includes full domain edges, u interiors and both
        # curvature-cell interiors; it is deliberately denser than construction.
        for coordinate in np.linspace(0., 1., 37):
            for curvature in np.linspace(-.01, .01, 19):
                speed = chart.speed_floor + coordinate*(chart.domain_speed(curvature)-chart.speed_floor)
                low, high = chart.limits(speed, curvature)
                self.assertTrue(evaluate_equilibrium(self.vehicle, speed, curvature, low).is_feasible)
                self.assertTrue(evaluate_equilibrium(self.vehicle, speed, curvature, high).is_feasible)

    def test_physical_normalized_query_and_no_extrapolation(self):
        chart = _chart()
        self.assertEqual(chart.domain_speed(0.), 20.)
        self.assertAlmostEqual(chart.normalized_speed(10.25, 0.), .5)
        self.assertEqual(chart.limits(10.25, 0.), (-7., 1.))
        with self.assertRaisesRegex(ValueError, "capability domain"):
            chart.limits(20.001, 0.)
        with self.assertRaisesRegex(ValueError, "capability domain"):
            chart.limits(.4, 0.)

    def test_exclusive_versioned_serialization_preserves_legacy_files(self):
        chart = _chart()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "signed.npz"
            save_signed_transient_capability(chart, path)
            loaded = load_signed_transient_capability(path)
            self.assertEqual(loaded.representation, "signed_transient_normalized_v1")
            np.testing.assert_allclose(loaded.domain_speed_values, chart.domain_speed_values)
            with self.assertRaises(FileExistsError):
                save_signed_transient_capability(chart, path)

    def test_numeric_symbolic_lookup_parity(self):
        try:
            import casadi as ca
        except ImportError:
            self.skipTest("CasADi unavailable")
        chart = _chart(); lookup = symbolic_lookup(chart, "parity")
        v, k = ca.MX.sym("v"), ca.MX.sym("k")
        lower, upper, domain = lookup(v, k)
        function = ca.Function("lookup", [v, k], [lower, upper, domain])
        numeric = chart.limits(10.25, .04)
        actual = function(10.25, .04)
        self.assertAlmostEqual(float(actual[0]), numeric[0], places=12)
        self.assertAlmostEqual(float(actual[1]), numeric[1], places=12)
        self.assertAlmostEqual(float(actual[2]), chart.domain_speed(.04), places=12)

    def test_disconnected_acceleration_sets_are_refused(self):
        with self.assertRaisesRegex(ValueError, "disconnected"):
            signed_feasible_interval(lambda a: (-4 < a < -3) or a > 2,
                                     acceleration_min=-5., acceleration_max=5., scan_points=101)

class LinearProgramSignedCapabilityTest(unittest.TestCase):
    def test_lp_fit_preserves_width_where_common_interval_would_shrink(self):
        """A spatially varying signed band is feasible for LP vertex fitting.

        A legacy all-probe common interval is only [-.5, 2], while the LP retains
        the four-unit local interval; this guards against silently routing the
        opt-in mode through the reviewed common-interval builder.
        """
        from unittest.mock import patch
        import simplified_vehicle_models.formulations.signed_capability as signed
        class Vehicle: pass
        def fake_equilibrium(vehicle, speed, curvature, acceleration):
            coordinate = (speed-.5)/9.5
            low = -2. + 1.5*coordinate + .25*curvature
            return type("State", (), {"is_feasible": low <= acceleration <= low + 4.})()
        with patch.object(signed, "maximum_vehicle_speed", return_value=10.), \
             patch.object(signed, "_speed_domain", return_value=10.), \
             patch.object(signed, "capability_hash", return_value="synthetic"), \
             patch.object(signed, "evaluate_equilibrium", side_effect=fake_equilibrium):
            chart = signed.build_signed_transient_capability(
                Vehicle(), interval_fit="linear_program", curvature=np.array([-.5, .5]),
                u=np.array([0., 1.]), maximum_speed=10., acceleration_range=(-5., 5.),
                scan_points=41, refinement_steps=4, speed_scan_points=3,
                endpoint_buffer=.1, domain_backoff=.0, minimum_interval_width=.02,
                construction_subdivisions=2, validation_grid=(5, 5),
                heldout_validation_grid=(4, 4), cut_plane_attempts=0)
        self.assertEqual(chart.build_settings["interval_fit"], "linear_program")
        self.assertEqual(chart.build_settings["vehicle_physics_hash"], "synthetic")
        self.assertNotIn("model_hash", chart.build_settings)
        self.assertGreater(float((chart.acceleration_max-chart.acceleration_min).min()), 3.7)
        self.assertEqual(chart.validation["heldout_endpoint_checks"], 32)

    def test_lp_mode_rejects_bad_options_and_default_stays_common(self):
        import simplified_vehicle_models.formulations.signed_capability as signed
        with self.assertRaisesRegex(ValueError, "interval_fit"):
            signed.build_signed_transient_capability(object(), interval_fit="unreviewed", curvature=np.array([0., .1]))
        self.assertEqual(signed.build_signed_transient_capability.__kwdefaults__["interval_fit"], "common_interval")

class LinearProgramValidationTest(unittest.TestCase):
    def test_lp_rejects_malformed_settings_before_physics(self):
        from unittest.mock import patch
        import simplified_vehicle_models.formulations.signed_capability as signed
        cases = (
            {"scan_points": 81.0}, {"refinement_steps": True},
            {"speed_scan_points": "25"}, {"construction_subdivisions": 2.0},
            {"endpoint_buffer": float("nan")}, {"domain_backoff": float("nan")},
            {"minimum_interval_width": float("nan")}, {"acceleration_range": (-1.,)},
            {"acceleration_range": (-1., float("nan"))}, {"acceleration_range": (1., -1.)},
        )
        base = dict(interval_fit="linear_program", curvature=np.array([-.1, .1]),
                    u=np.array([0., 1.]), validation_grid=(2, 2), heldout_validation_grid=(2, 2))
        for bad in cases:
            with self.subTest(bad=bad), patch.object(signed, "maximum_vehicle_speed") as mechanical:
                with self.assertRaisesRegex(ValueError, "invalid linear-program"):
                    signed.build_signed_transient_capability(object(), **(base | bad))
                mechanical.assert_not_called()


class SignedCapabilityCliTest(unittest.TestCase):
    def test_cli_rejects_existing_and_dangling_output_before_compile(self):
        from unittest.mock import patch
        from tools.model_data import build_signed_capability as cli
        with tempfile.TemporaryDirectory() as directory:
            for name, dangling in (("existing.npz", False), ("dangling.npz", True)):
                output = Path(directory) / name
                if dangling:
                    output.symlink_to(Path(directory) / "missing.npz")
                else:
                    output.touch()
                with self.subTest(name=name), patch.object(cli, "compile_formulation") as compile_:
                    with self.assertRaises(SystemExit) as exit_:
                        cli.main(["--formulation", str(FORMULATION), "--output", str(output)])
                    self.assertEqual(exit_.exception.code, 2)
                    compile_.assert_not_called()

    def test_cli_uses_only_mode_specific_builder_arguments(self):
        from unittest.mock import patch
        from tools.model_data import build_signed_capability as cli
        class Chart:
            representation = "signed_transient_normalized_v1"
            provenance_hash = "vehicle"
            build_settings = {}
            validation = {}
        with tempfile.TemporaryDirectory() as directory:
            for fit in ("linear_program", "common_interval"):
                output = Path(directory) / f"{fit}.npz"
                with self.subTest(fit=fit), patch.object(cli, "compile_formulation", return_value=type("X", (), {"model": object()})()), \
                     patch.object(cli, "point_mass_view_of_planar", return_value=object()), \
                     patch.object(cli, "build_signed_transient_capability", return_value=Chart()) as build, \
                     patch.object(cli, "save_signed_transient_capability"):
                    self.assertEqual(cli.main(["--formulation", str(FORMULATION), "--output", str(output), "--interval-fit", fit]), 0)
                    kwargs = build.call_args.kwargs
                    self.assertEqual(kwargs["interval_fit"], fit)
                    self.assertEqual("minimum_interval_width" in kwargs, fit == "linear_program")
                    self.assertEqual(build.return_value.build_settings["source_manifest"]["formulation_path"], str(FORMULATION.resolve()))
                    self.assertRegex(build.return_value.build_settings["source_manifest"]["formulation_sha256"], r"^[0-9a-f]{64}$")
