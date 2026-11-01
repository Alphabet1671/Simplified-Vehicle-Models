import json
import tempfile
import unittest
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import patch

import numpy as np

from tools.run import compare_continuous_driver_throttle as runner


class PlanAndMetricTests(unittest.TestCase):
    def test_left_weighted_metrics_exclude_last_sample_duration(self):
        metrics = runner.interval_weighted_metrics({"time": [0., 1., 3.],
            "throttle": [1., .5, 0.], "speed_deficit": [2., 1., 0.]})
        self.assertAlmostEqual(metrics["full_throttle_fraction"], 1 / 3)
        self.assertAlmostEqual(metrics["mean_speed_deficit"], 4 / 3)

    def test_nonincreasing_and_nonfinite_times_fail(self):
        for times in ([0., 1., 1.], [0., float("nan"), 2.]):
            with self.subTest(times=times), self.assertRaises(ValueError):
                runner.interval_weighted_metrics({"time": times, "throttle": [1., .5, 0.],
                                                  "speed_deficit": [2., 1., 0.]})

    def test_slice_starts_at_interpolated_six_and_retains_precursor(self):
        class Plan:
            distance = np.array([0., 10., 100., 200., 250.])
            speed = np.array([0., 4., 8., 10., 12.])
            curvature = np.zeros(5)
            acceleration = np.ones(5)
        with patch.object(runner, "build_speed_plan", return_value=Plan()):
            sliced = runner.make_plan(object())
        self.assertEqual(sliced.speed[0], 6.)
        self.assertAlmostEqual(sliced.crop_origin_m, 55.)
        self.assertEqual(sliced.distance[0], 0.)
        self.assertEqual(sliced.distance[-1], 150.)
        self.assertEqual(sliced.precursor_distance[-1], 250.)
        self.assertEqual(sliced.precursor_curvature.tolist(), [0.] * 5)
        self.assertEqual(sliced.precursor_acceleration.tolist(), [1.] * 5)

    def test_reconstructed_speed_plan_acceleration_must_match_frozen_values(self):
        class SpeedPlan:
            def __init__(self, distance, speed, curvature, is_closed):
                self.acceleration = np.array([0., 99.])
        with patch("simplified_vehicle_models.solvers.forward_driver.SpeedPlan", SpeedPlan):
            frozen = runner.freeze_plan_arrays([0., 1.], [6., 7.], [0., 0.], [1., 2.])
            with self.assertRaisesRegex(ValueError, "acceleration"):
                runner._speed_plan(frozen)

    def test_unavailable_crossing_or_length_fail_explicitly(self):
        class Plan:
            distance = np.array([0., 100., 250.])
            speed = np.array([0., 5., 6.])
            curvature = np.zeros(3)
            acceleration = np.ones(3)
        with patch.object(runner, "build_speed_plan", return_value=Plan()):
            with self.assertRaisesRegex(ValueError, "lacks 150 m"):
                runner.make_plan(object())


class ContractsTests(unittest.TestCase):
    def test_manifest_mismatch_fails(self):
        with self.assertRaisesRegex(ValueError, "manifest"):
            runner.require_matching_manifests({"model_hash": "a"}, {"model_hash": "b"})

    def test_noncanonical_configuration_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "fixed"):
            runner.prepare_comparison("unused", {"driver": {"speed_gain": 9.}})

    def test_existing_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            existing = Path(tmp) / "run"
            existing.mkdir()
            with self.assertRaises(FileExistsError):
                runner.create_output_directory(existing)
            self.assertTrue(existing.is_dir())

    def test_source_manifest_captures_hashes_revision_and_dirty_status(self):
        with patch.object(runner, "subprocess") as subprocess:
            subprocess.run.side_effect = [type("P", (), {"stdout": "abc123\n"})(),
                                         type("P", (), {"stdout": ""})()]
            with patch.object(runner, "SOURCE_PATHS", ()):
                result = runner.collect_source_manifest()
        self.assertEqual(result["revision"], "abc123")
        self.assertFalse(result["dirty"])
        self.assertTrue(result["hashes"])


class MatchedArmTests(unittest.TestCase):
    def test_trace_sink_uses_explicit_mapping_context_not_sample_identity(self):
        from simplified_vehicle_models.solvers.forward_driver import DriverCommand, StepSample
        class Plant:
            last_evaluation = None
            last_powertrain = None
            plan = type("Plan", (), {"sample": staticmethod(lambda distance: (0., 0., 0.))})()
            def observe(self, state):
                return SimpleNamespace(distance=0., speed=0., lateral_error=0., heading_error=0.)
        command = DriverCommand(.4, 0., .1)
        sample = StepSample(0, 0., np.zeros(11), np.zeros(11), command, None,
                            command, command)
        for configured, label in ((False, "identity_no_mapper"), (True, "physical_mapper")):
            sink = runner.TraceSink(Plant(), mapping_configured=configured)
            sink(sample)
            self.assertEqual(sink.rows[0]["mapped_stage"], label)
            self.assertEqual(sink.rows[0]["mapped_throttle"], .4)

    def _prepared(self, tempdir):
        path = Path(tempdir)
        arrays = runner.freeze_plan_arrays([0., 150.], [6., 6.], [0., 0.], [0., 0.])
        for name in ("distance", "speed", "curvature", "acceleration"):
            np.save(path / f"plan_{name}.npy", getattr(arrays, name), allow_pickle=False)
        for name, values in (("precursor_distance", [0., 150., 250.]), ("precursor_speed", [6., 6., 10.]),
                             ("precursor_curvature", [0., 0., 0.]), ("precursor_acceleration", [0., 0., 1.])):
            np.save(path / f"plan_{name}.npy", np.asarray(values), allow_pickle=False)
        config = runner._canonical_config("compiled identity")
        data = {**config, "config_digest": runner._config_digest(config),
                "plan_hash": arrays.digest,
                "plan_arrays_hashes": {name: __import__("hashlib").sha256(np.asarray(getattr(arrays, name), dtype="<f8").tobytes()).hexdigest()
                                        for name in ("distance", "speed", "curvature", "acceleration")},
                "sources": {"hash": "same"}, "crop_origin_m": 0.,
                "tire_law": "steady combined-slip UniTire with dynamic wheel/body states; not transient tire relaxation",
                "archived_ttc_lap_equivalence": False, "setup_seconds": 0.,
                "runtime": {"python": "test", "numpy": np.__version__, "scipy": "test"},
                "artifact_digest": "placeholder"}
        manifest_body = dict(data)
        manifest_body.pop("artifact_digest", None)
        data["artifact_digest"] = runner._artifact_digest(dict(manifest_body), {
            **{name: getattr(arrays, name) for name in ("distance", "speed", "curvature", "acceleration")},
            **{"precursor_" + name: np.load(path / f"plan_precursor_{name}.npy")
               for name in ("distance", "speed", "curvature", "acceleration")}})
        (path / "manifest.json").write_text(json.dumps(data))
        return arrays

    def test_each_arm_receives_identical_plan_and_solver_settings_mapper_only_differs(self):
        class Vehicle: wheelbase=1.; steering_maximum_angle=.5; steering_maximum_rate=1.; maximum_pedal_force=100.
        class Plant:
            state_size=12
            state_names=tuple(f"x{i}" for i in range(12))
            def __init__(self, vehicle, plan): self.plan=plan
            def initial_state(self, speed):
                value=np.zeros(12); value[3]=speed; value[10]=.8; return value
        class Mapper:
            def __call__(self, *args): return args[-1]
            def snapshot_diagnostics(self): return {"callback_queries": 3}
        calls=[]
        class ObservationPlant(Plant):
            last_evaluation = None
            last_powertrain = None
            def observe(self, state):
                return SimpleNamespace(distance=float(state[0]), speed=float(state[3]),
                                       lateral_error=0., heading_error=0.)
        def solve(plant, driver, initial, **kwargs):
            calls.append((plant, driver, initial.copy(), kwargs))
            for i, time_value in enumerate((0., 1.)):
                command = SimpleNamespace(throttle=.5, brake_force=0., steering_rate=0.)
                kwargs["step_sink"](SimpleNamespace(index=i, time=time_value, state=initial.copy(),
                    rates=np.zeros(12), command=command, requested_command=command,
                    mapped_command=(command if kwargs["command_mapper"] is not None else None), pedal=0.))
            return type("Result", (), {"time": np.array([0., 1.]), "wall_seconds": .1,
                                        "diagnostics": {}})()
        with patch.object(runner, "collect_source_manifest", return_value={"hash":"same"}), \
             patch.object(runner, "compile_selected_model", return_value=type("C", (), {
                 "model":Vehicle(), "manifest":type("M", (), {"as_text":lambda self: "compiled identity"})()})()), \
             patch.object(runner, "make_plant_class", return_value=ObservationPlant), \
             patch.object(runner, "make_driver", side_effect=lambda plan, vehicle, gains: type("D", (), gains)()), \
             patch.object(runner, "make_mapper", side_effect=lambda *args: Mapper()), \
             patch.object(runner, "solve_forward", side_effect=solve):
            with tempfile.TemporaryDirectory() as tempdir:
                self._prepared(tempdir)
                heuristic_result = runner.run_arm(tempdir, "heuristic")
                physical_result = runner.run_arm(tempdir, "physical_mapper")
                self.assertEqual(json.loads((Path(tempdir) / "heuristic.json").read_text()), heuristic_result)
                self.assertEqual(json.loads((Path(tempdir) / "physical_mapper.json").read_text()), physical_result)
                self.assertEqual(heuristic_result["accepted_samples"], 2)
                self.assertEqual(len(heuristic_result["telemetry"]), 2)
                self.assertEqual(heuristic_result["telemetry"][0]["requested_throttle"], .5)
                self.assertEqual(heuristic_result["telemetry"][0]["mapped_stage"], "identity_no_mapper")
                self.assertEqual(heuristic_result["telemetry"][0]["pedal_lagged_applied_throttle"], .5)
                self.assertEqual(physical_result["telemetry"][0]["mapped_stage"], "physical_mapper")
        self.assertEqual(len(calls), 2)
        first, second = calls
        self.assertIsNone(first[3]["command_mapper"])
        self.assertIsNotNone(second[3]["command_mapper"])
        for key in ("initial_pedal", "maximum_step", "rtol", "atol", "wall_seconds", "maximum_wall_seconds"):
            self.assertEqual(first[3][key], second[3][key])
        self.assertEqual(first[3]["initial_pedal"], 0.)
        self.assertEqual(first[2].tolist(), second[2].tolist())
        self.assertEqual(first[0].plan.speed.tolist(), second[0].plan.speed.tolist())
        self.assertEqual(first[1].speed_gain, second[1].speed_gain)
        self.assertEqual(first[1].heading_gain, second[1].heading_gain)
        self.assertEqual(first[1].lateral_gain, second[1].lateral_gain)
        self.assertEqual(first[3]["command_mapper"], None)
        self.assertEqual(len(first[3]["step_sink"].rows), 2)
        self.assertEqual(len(second[3]["step_sink"].rows), 2)

    def test_preexisting_arm_evidence_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as tempdir:
            evidence = Path(tempdir) / "heuristic.json"
            original = '{"status":"preserve"}'
            evidence.write_text(original)
            with self.assertRaisesRegex(FileExistsError, "already exists"):
                runner.run_arm(tempdir, "heuristic")
            self.assertEqual(evidence.read_text(), original)
            self.assertFalse((Path(tempdir) / "heuristic.claim.json").exists())

    def test_tampered_precursor_and_crop_metadata_are_rejected_before_solver(self):
        class Vehicle: wheelbase=1.; steering_maximum_angle=.5; steering_maximum_rate=1.; maximum_pedal_force=100.
        compiled = SimpleNamespace(model=Vehicle(), manifest=SimpleNamespace(as_text=lambda: "compiled identity"))
        with tempfile.TemporaryDirectory() as tempdir:
            self._prepared(tempdir)
            precursor_path = Path(tempdir) / "plan_precursor_speed.npy"
            np.save(precursor_path, np.array([6., 7., 10.]), allow_pickle=False)
            with patch.object(runner, "compile_selected_model", return_value=compiled), \
                 patch.object(runner, "collect_source_manifest", return_value={"hash": "same"}), \
                 patch.object(runner, "solve_forward") as solve:
                with self.assertRaisesRegex(ValueError, "crop|artifact digest"):
                    runner._run_arm(tempdir, "heuristic")
                solve.assert_not_called()
        with tempfile.TemporaryDirectory() as tempdir:
            self._prepared(tempdir)
            manifest_path = Path(tempdir) / "manifest.json"
            data = json.loads(manifest_path.read_text())
            data["crop_origin_m"] = 2.
            manifest_path.write_text(json.dumps(data))
            with patch.object(runner, "compile_selected_model", return_value=compiled), \
                 patch.object(runner, "solve_forward") as solve:
                with self.assertRaisesRegex(ValueError, "crop|artifact digest"):
                    runner._run_arm(tempdir, "heuristic")
                solve.assert_not_called()

    def test_mapper_diagnostic_snapshot_failure_preserves_accepted_prefix(self):
        class Vehicle: wheelbase=1.; steering_maximum_angle=.5; steering_maximum_rate=1.; maximum_pedal_force=100.
        class Plant:
            state_size=12
            state_names=tuple(f"x{i}" for i in range(12))
            last_evaluation = None
            last_powertrain = None
            def __init__(self, vehicle, plan): self.plan=plan
            def initial_state(self, speed):
                value=np.zeros(12); value[3]=speed; value[10]=.8; return value
            def observe(self, state):
                return SimpleNamespace(distance=0., speed=float(state[3]), lateral_error=0., heading_error=0.)
        class FailingSnapshotMapper:
            def __call__(self, time_value, state, observation, requested):
                return requested
            def snapshot_diagnostics(self):
                raise RuntimeError("snapshot boom")
        def solve(plant, driver, initial, **kwargs):
            command = SimpleNamespace(throttle=.4, brake_force=0., steering_rate=0.)
            for index, time_value in enumerate((0., 1.)):
                kwargs["step_sink"](SimpleNamespace(index=index, time=time_value,
                    state=initial.copy(), rates=np.zeros(12), command=command,
                    requested_command=command, mapped_command=command, pedal=0.))
            return SimpleNamespace(time=np.array([0., 1.]), wall_seconds=.1, diagnostics={})
        compiled = SimpleNamespace(model=Vehicle(), manifest=SimpleNamespace(as_text=lambda: "compiled identity"))
        with tempfile.TemporaryDirectory() as tempdir:
            self._prepared(tempdir)
            with patch.object(runner, "compile_selected_model", return_value=compiled), \
                 patch.object(runner, "collect_source_manifest", return_value={"hash": "same"}), \
                 patch.object(runner, "make_plant_class", return_value=Plant), \
                 patch.object(runner, "make_driver", return_value=object()), \
                 patch.object(runner, "make_mapper", return_value=FailingSnapshotMapper()), \
                 patch.object(runner, "solve_forward", side_effect=solve):
                result = runner.run_arm(tempdir, "physical_mapper")
            saved = json.loads((Path(tempdir) / "physical_mapper.json").read_text())
            self.assertEqual(saved["status"], "failed")
            self.assertEqual(saved["accepted_samples"], 2)
            self.assertEqual(len(saved["telemetry"]), 2)
            self.assertEqual(saved["mapper_diagnostics_error"], "RuntimeError: snapshot boom")
            self.assertEqual(saved["error"], "RuntimeError: snapshot boom")
            self.assertEqual(result["telemetry"], saved["telemetry"])

    def test_setup_failure_persists_valid_json_and_claim_blocks_retry(self):
        with tempfile.TemporaryDirectory() as tempdir:
            self._prepared(tempdir)
            with patch.object(runner, "compile_selected_model", side_effect=RuntimeError("compile boom")):
                with self.assertRaisesRegex(RuntimeError, "compile boom"):
                    runner.run_arm(tempdir, "heuristic")
            artifact = json.loads((Path(tempdir) / "heuristic.json").read_text())
            self.assertEqual(artifact["status"], "failed")
            self.assertEqual(artifact["prefix_kind"], "none; failure before solve sink was established")
            with self.assertRaises(FileExistsError):
                runner.run_arm(tempdir, "heuristic")

    def test_nonfinite_terminal_trace_is_serialized_without_losing_prefix(self):
        class Vehicle: wheelbase=1.; steering_maximum_angle=.5; steering_maximum_rate=1.; maximum_pedal_force=100.
        class Plant:
            state_size=12
            state_names=tuple(f"x{i}" for i in range(12))
            def __init__(self, vehicle, plan): self.plan=plan
            def initial_state(self, speed):
                value=np.zeros(12); value[3]=speed; value[10]=.8; return value
            def observe(self, state):
                return SimpleNamespace(distance=0., speed=float(state[3]), lateral_error=0., heading_error=0.)
            last_evaluation = None
            last_powertrain = None
        def solve(plant, driver, initial, **kwargs):
            command = SimpleNamespace(throttle=float("nan"), brake_force=0., steering_rate=0.)
            kwargs["step_sink"](SimpleNamespace(index=0, time=0., state=initial.copy(), rates=np.zeros(12),
                command=command, requested_command=command, mapped_command=command, pedal=0.))
            return SimpleNamespace(time=np.array([0.]), wall_seconds=.1, diagnostics={})
        compiled = SimpleNamespace(model=Vehicle(), manifest=SimpleNamespace(as_text=lambda: "compiled identity"))
        with tempfile.TemporaryDirectory() as tempdir:
            self._prepared(tempdir)
            with patch.object(runner, "compile_selected_model", return_value=compiled), \
                 patch.object(runner, "collect_source_manifest", return_value={"hash": "same"}), \
                 patch.object(runner, "make_plant_class", return_value=Plant), \
                 patch.object(runner, "make_driver", return_value=object()), \
                 patch.object(runner, "solve_forward", side_effect=solve):
                result = runner.run_arm(tempdir, "heuristic")
            saved = json.loads((Path(tempdir) / "heuristic.json").read_text())
            self.assertEqual(result["accepted_samples"], 1)
            self.assertEqual(saved["accepted_samples"], 1)
            self.assertEqual(saved["telemetry"][0]["requested_throttle"], None)

    def test_tampered_complete_configuration_is_rejected_before_solver(self):
        class Vehicle: wheelbase=1.; steering_maximum_angle=.5; steering_maximum_rate=1.; maximum_pedal_force=100.
        compiled = SimpleNamespace(model=Vehicle(), manifest=SimpleNamespace(as_text=lambda: "compiled identity"))
        with tempfile.TemporaryDirectory() as tempdir:
            self._prepared(tempdir)
            manifest_path = Path(tempdir) / "manifest.json"
            data = json.loads(manifest_path.read_text())
            data["solver"]["maximum_step"] = .5
            body = dict(data)
            body.pop("artifact_digest", None)
            arrays = {name: np.load(Path(tempdir) / f"plan_{name}.npy")
                      for name in ("distance", "speed", "curvature", "acceleration")}
            arrays.update({"precursor_" + name: np.load(Path(tempdir) / f"plan_precursor_{name}.npy")
                           for name in ("distance", "speed", "curvature", "acceleration")})
            data["artifact_digest"] = runner._artifact_digest(body, arrays)
            manifest_path.write_text(json.dumps(data))
            with patch.object(runner, "compile_selected_model", return_value=compiled), \
                 patch.object(runner, "collect_source_manifest", return_value={"hash": "same"}), \
                 patch.object(runner, "solve_forward") as solve:
                with self.assertRaisesRegex(ValueError, "configuration"):
                    runner._run_arm(tempdir, "heuristic")
                solve.assert_not_called()


if __name__ == "__main__":
    unittest.main()
