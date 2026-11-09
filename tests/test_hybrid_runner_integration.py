"""Focused caller-level contract tests for opt-in hybrid runner integration.

Lifecycle tests use bounded explicit settings and preserve production defaults.
"""
import contextlib
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np

from tools.run import run_iem24_forward_driver as runner
from simplified_vehicle_models.solvers.hybrid_setup import HYBRID_SETUP_VERSION, HYBRID_EXECUTION_ALLOCATOR_ID
from simplified_vehicle_models.solvers.hybrid_driver import HYBRID_POLICY_VERSION
from tools.run.run_iem24_forward_driver import (
    DEFAULT_TRACK, canonical_json, child, parse_args, policy_identity_settings,
    source_files, warm_start_matches,
)


class HybridRunnerIntegrationTests(unittest.TestCase):
    def test_mode_is_opt_in_and_defaults_remain_legacy(self):
        defaults = parse_args([])
        self.assertEqual(defaults.longitudinal_mode, "legacy")
        self.assertEqual(defaults.initial_speed, 0.)
        selected = parse_args(["--model", "14dof-simple", "--longitudinal-mode",
                               "hybrid-drive-brake-v1", "--initial-speed", "1"])
        self.assertEqual(selected.longitudinal_mode, "hybrid-drive-brake-v1")
        self.assertEqual(selected.initial_speed, 1.)
        from tools.sim_gui.settings import RunSettings, forward_driver_config
        from simplified_vehicle_models.solvers.hybrid_setup import HYBRID_SETUP_VERSION
        gui = RunSettings(solver_kind="forward-driver", forward_longitudinal_mode="hybrid-drive-brake-v1")
        cfg = forward_driver_config(gui)
        self.assertEqual(cfg["initial_speed_mps"], 6.0)
        self.assertEqual(gui.forward_maximum_speed, 60.0)
        self.assertEqual((cfg["hybrid_horizon_s"], cfg["hybrid_slabs"]), (3.0, 30))
        self.assertEqual(cfg["hybrid_setup_version"], HYBRID_SETUP_VERSION)
        self.assertEqual(policy_identity_settings(selected)["hybrid_setup_version"], HYBRID_SETUP_VERSION)

    def test_hybrid_rejects_invalid_and_zero_initial_speed(self):
        for value in ("-1", "nan", "inf", "0"):
            stderr = io.StringIO()
            with self.subTest(value=value), contextlib.redirect_stderr(stderr), self.assertRaises(SystemExit):
                parse_args(["--model", "14dof-simple", "--longitudinal-mode",
                            "hybrid-drive-brake-v1", "--initial-speed", value])
            expected = "rolling starts" if value == "0" else "initial-speed"
            self.assertIn(expected, stderr.getvalue())
        self.assertEqual(parse_args(["--initial-speed", "0"]).initial_speed, 0.)

    def test_hybrid_source_closure_is_opt_in(self):
        hybrid = {Path(p).resolve() for p in source_files(
            "14dof-simple", DEFAULT_TRACK, longitudinal_mode="hybrid-drive-brake-v1")}
        legacy = {Path(p).resolve() for p in source_files("14dof-simple", DEFAULT_TRACK)}
        for name in ("hybrid_setup.py", "hybrid_driver.py", "hybrid_speed_plan.py", "longitudinal_capability.py"):
            path = (Path("src/simplified_vehicle_models/solvers") / name).resolve()
            self.assertIn(path, hybrid)
            self.assertNotIn(path, legacy)

    def _prior_hybrid_attempt(self, args, identity, source_hashes, calibration_sha):
        settings = {"longitudinal_policy": "hybrid-drive-brake-v1",
            "longitudinal_mode": "hybrid-drive-brake-v1", "physical_brake_planning": False,
            "transient_corner_planning": False, "envelope_deceleration_margin": args.envelope_deceleration_margin,
            "envelope_speed_points": args.envelope_speed_points, "initial_speed": args.initial_speed,
            "maximum_speed": args.maximum_speed, "lateral_acceleration": args.lateral_acceleration,
            "pedal_travel_time": args.pedal_travel_time,
            "throttle_apply_rate": args.throttle_apply_rate, "throttle_release_rate": args.throttle_release_rate,
            "brake_apply_rate": args.brake_apply_rate, "brake_release_rate": args.brake_release_rate,
            "spacing": args.spacing, "preview_seconds": args.preview_seconds,
            "heading_gain": args.heading_gain, "lateral_gain": args.lateral_gain,
            "speed_gain": args.speed_gain, "lateral_margin": args.lateral_margin,
            "track": str(args.track)}
        return {"status": "Completed", "lap_completed": True, "source_hashes": source_hashes,
            "settings": settings, "pedal_map": {"semantic_sha256": calibration_sha},
            "longitudinal_policy": {"policy": "hybrid-drive-brake-v1", "settings_identity": identity,
                "settings_identity_sha256": hashlib.sha256(canonical_json(identity).encode()).hexdigest()}}

    def _attempt_warm_start(self, prior, args, attempt, identity):
        args.warm_start = attempt
        args.child_attempt = attempt.parent / (attempt.name + "-child")
        args.child_attempt.mkdir()
        with mock.patch.object(runner, "source_files", return_value=tuple(prior["source_paths"])), \
             mock.patch("simplified_vehicle_models.compiler.compile.compile_formulation",
                        side_effect=AssertionError("compile must not run")) as compile_model, \
             mock.patch.object(np, "loadtxt", side_effect=AssertionError("warm-start arrays must not be read")) as loadtxt, \
             contextlib.redirect_stdout(io.StringIO()):
            result = child(args)
        return result, compile_model, loadtxt, args.child_attempt

    def test_warm_start_rejects_mutated_actual_calibration_before_arrays(self):
        from simplified_vehicle_models.solvers.hybrid_setup import build_hybrid_setup
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.formulations.point_mass import normal_loads, point_mass_view_of_planar
        from simplified_vehicle_models.tires.unitire_simple import loaded_radius
        from simplified_vehicle_models.powertrain.energy_storage import StorageState
        from simplified_vehicle_models.solvers.envelope_driver import PedalPolicyContext

        compiled = compile_formulation("models/formulations/iem24_murphy_real_14dof_simple.xml")
        vehicle = compiled.model.planar
        radii = tuple(float(v) for v in loaded_radius(vehicle.tire,
            normal_loads(point_mass_view_of_planar(vehicle), 1.0, 0.0, 0.0)))
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as td:
            root = Path(td)
            calibration = root / "pedal-ratio.csv"
            original_bytes = Path("models/data/murphy_pedal_ratio.csv").read_bytes()
            calibration.write_bytes(original_bytes)
            x = np.linspace(0., 20., 5)
            config = parse_args(["--model", "14dof-simple", "--longitudinal-mode",
                "hybrid-drive-brake-v1", "--initial-speed", "1", "--track", str(DEFAULT_TRACK),
                "--warm-start", str(root / "prior"), "--pedal-travel-time", ".175",
                "--throttle-apply-rate", "4", "--throttle-release-rate", "6",
                "--brake-apply-rate", "5", "--brake-release-rate", "6"])
            config.child_attempt = None
            actual_sources = list(runner.model_inputs("14dof-simple", DEFAULT_TRACK).values())
            sources = list(source_files("14dof-simple", DEFAULT_TRACK,
                longitudinal_mode="hybrid-drive-brake-v1"))
            sources.extend(Path(p) for p in actual_sources if Path(p) not in sources)
            calibration_paths = [Path(p).resolve() for p in actual_sources if Path(p).name == "murphy_pedal_ratio.csv"]
            self.assertEqual(len(calibration_paths), 1)
            actual_calibration = calibration_paths[0]
            # Use the production-declared model dependency, but materialize an isolated
            # temporary copy so byte mutation cannot alter repository model data.
            calibration.write_bytes(actual_calibration.read_bytes())
            actual_sources = [calibration if Path(p).resolve() == actual_calibration else Path(p) for p in actual_sources]
            sources = [calibration if Path(p).resolve() == actual_calibration else Path(p) for p in sources]
            source_hashes = {str(Path(p).resolve()): hashlib.sha256(Path(p).read_bytes()).hexdigest()
                             for p in sources}
            expected_pre_mutation_closure = hashlib.sha256(canonical_json(source_hashes).encode()).hexdigest()
            compiled_vehicle = compile_formulation("models/formulations/iem24_murphy_real_14dof_simple.xml").model.planar
            repository=Path(runner.__file__).resolve().parents[2]
            setup_source_hashes = {Path(key).resolve().relative_to(repository).as_posix(): value
                                   for key, value in source_hashes.items()
                                   if Path(key).resolve() != calibration.resolve() and repository in Path(key).resolve().parents}
            setup_source_hashes[actual_calibration.relative_to(repository).as_posix()] = hashlib.sha256(actual_calibration.read_bytes()).hexdigest()
            setup = build_hybrid_setup(compiled_vehicle, distance_m=x, curvature_inv_m=np.zeros_like(x),
                maximum_speed_mps=config.maximum_speed, initial_speed_mps=config.initial_speed,
                storage_soc=float(StorageState(vehicle.drivetrain.powertrain.energy_storage.initial_state_of_charge).state_of_charge),
                wheel_radii_m=radii, model_source_hashes=setup_source_hashes,
                maximum_pedal_force_n=float(vehicle.maximum_pedal_force),
                pedal_context=PedalPolicyContext(6., 5., .02, float(vehicle.maximum_pedal_force), .85, 4., 6.),
                lateral_margin=.9, is_closed=False)
            identity = {"policy": HYBRID_POLICY_VERSION, "longitudinal_mode": "hybrid-drive-brake-v1",
                "setup_identity": setup.setup_identity, "setup_identity_payload": json.loads(setup.setup_identity_payload_json),
                "setup_version": HYBRID_SETUP_VERSION, "plan_version": setup.plan.algorithm_version,
                "provider_identity": setup.provider.identity, "allocator_identity": HYBRID_EXECUTION_ALLOCATOR_ID,
                "physical_actuator_rates": {"throttle_apply_rate": config.throttle_apply_rate,
                    "throttle_release_rate": config.throttle_release_rate, "brake_apply_rate": config.brake_apply_rate,
                    "brake_release_rate": config.brake_release_rate}, "lateral_margin": .9,
                "brake_margin": config.envelope_deceleration_margin,
                "source_closure_sha256": hashlib.sha256(canonical_json(setup_source_hashes).encode()).hexdigest()}
            prior_dir = root / "prior"
            prior_dir.mkdir()
            prior = self._prior_hybrid_attempt(config, identity, source_hashes, "no-pedal-map")
            prior["source_paths"] = sources
            serialized_prior = {key: value for key, value in prior.items() if key != "source_paths"}
            (prior_dir / "summary.json").write_text(json.dumps(serialized_prior))
            # Prior arrays exist and are valid-looking; the negative case must reject
            # the mutated actual calibration bytes before either one is touched.
            (prior_dir / "trace.csv").write_text("time,state\n0,0\n1,0.1\n")
            (prior_dir / "pedal.csv").write_text("time,pedal\n0,0\n1,0.2\n")
            # Positive control through the production warm-start load seam: the
            # same unmutated source/setup identity accepts and reads both arrays.
            trace, pedals = runner._load_warm_start_arrays(prior_dir, serialized_prior, source_hashes,
                config, "no-pedal-map", identity, type("Plant", (), {"state_size": 1})())
            self.assertEqual(trace.shape, (2, 2))
            self.assertEqual(pedals.shape, (2, 2))
            calibration.write_bytes(calibration.read_bytes() + b"\n# calibration-byte-mutation\n")
            # Simulate the stale prior source snapshot: the child's current closure
            # is now different and must fail before compile or saved-array access.
            config.physical_source_closure_sha256 = expected_pre_mutation_closure
            result, compile_model, loadtxt, child_attempt = self._attempt_warm_start(prior, config, prior_dir, identity)
            self.assertNotEqual(result, 0)
            compile_model.assert_not_called()
            loadtxt.assert_not_called()
            summary = json.loads((child_attempt / "summary.json").read_text())
            self.assertIn("warm-start source/calibration identity changed", summary.get("error", ""))

    def test_hybrid_warm_start_uses_actual_setup_identity(self):
        identity = {"policy": HYBRID_POLICY_VERSION, "longitudinal_mode": "hybrid-drive-brake-v1",
                    "setup_identity": "canonical-setup-identity", "setup_version": HYBRID_SETUP_VERSION,
                    "plan_version": "hybrid-speed-plan-v6-cap-bounded-trial-search",
                    "provider_identity": "provider", "allocator_identity": HYBRID_EXECUTION_ALLOCATOR_ID,
                    "physical_actuator_rates": {
                        "throttle_apply_rate": 4.0, "throttle_release_rate": 6.0,
                        "brake_apply_rate": 5.0, "brake_release_rate": 6.0},
                    "lateral_margin": .9, "brake_margin": .6, "source_closure_sha256": "source-closure"}
        args = parse_args(["--model", "14dof-simple", "--longitudinal-mode",
                           "hybrid-drive-brake-v1", "--initial-speed", "1"])
        hashes = {"track": "track-hash"}
        prior = self._prior_hybrid_attempt(args, identity, hashes, "no-pedal-map")
        self.assertTrue(warm_start_matches(prior, hashes, args, "no-pedal-map", identity))
        for changed in ({**identity, "setup_identity": "different-setup"},
            {**identity, "longitudinal_mode": "legacy"},
            {**identity, "source_closure_sha256": "changed-source"},
            {**identity, "provider_identity": "changed-calibration"}):
            with self.subTest(changed=changed):
                self.assertFalse(warm_start_matches(prior, hashes, args, "no-pedal-map", changed))

    def test_initial_speed_is_part_of_warm_start_identity(self):
        args = parse_args([])
        identity = policy_identity_settings(args)
        hashes = {"track": "track-hash"}
        settings = {"maximum_speed": args.maximum_speed, "lateral_acceleration": args.lateral_acceleration,
            "pedal_travel_time": args.pedal_travel_time, "spacing": args.spacing,
            "preview_seconds": args.preview_seconds, "heading_gain": args.heading_gain,
            "lateral_gain": args.lateral_gain, "speed_gain": args.speed_gain,
            "lateral_margin": args.lateral_margin, "track": str(args.track),
            "initial_speed": args.initial_speed, "longitudinal_policy": "braking-envelope",
            "envelope_deceleration_margin": args.envelope_deceleration_margin,
            "envelope_speed_points": args.envelope_speed_points}
        prior = {"status": "Completed", "lap_completed": True, "source_hashes": hashes,
            "settings": settings, "longitudinal_policy": {"policy": "braking-envelope",
            "deceleration_margin": args.envelope_deceleration_margin,
            "speed_query_points": args.envelope_speed_points,
            "settings_identity_sha256": hashlib.sha256(canonical_json(identity).encode()).hexdigest()}}
        self.assertTrue(warm_start_matches(prior, hashes, args, policy_settings_identity=identity))
        altered = {**prior, "settings": {**settings, "initial_speed": 1.}}
        self.assertFalse(warm_start_matches(altered, hashes, args, policy_settings_identity=identity))


if __name__ == "__main__":
    unittest.main()
