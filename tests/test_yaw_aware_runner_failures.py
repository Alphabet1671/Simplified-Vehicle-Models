"""Task 7: durable runner failure artifacts, strict JSON, exit codes, provenance."""
import json
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np

from tools.run import run_yaw_aware_forward_driver as runner


def _args(directory, **overrides):
    args = runner.parse_args(["--track", "models/tracks/mis_ev_endurance_2024_width.xml",
                              "--output-dir", str(Path(directory) / "run"),
                              "--yaw-speed-points", "3", "--yaw-ax-points", "3", "--yaw-ay-points", "3"])
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


class _Common:
    """Patch the expensive model/track/map build with small real objects."""

    def patches(self, plan_side_effect):
        from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
        v = np.array([0., 5., 10.]); ax = np.array([-2., 0., 2.]); ay = np.array([-3., 0., 3.])
        shape = (3, 3, 3)
        cap = YawCapabilityMap(v, ax, ay, np.full(shape, 9.), np.full(shape, 9.), np.ones(shape, bool),
                               1., "physical_no_tv_v1", "runner-test")
        track = SimpleNamespace(path_coordinates=object(), has_edges=True)
        return [
            mock.patch("simplified_vehicle_models.compiler.compile.compile_formulation",
                       return_value=SimpleNamespace(model=SimpleNamespace(yaw_inertia=1.))),
            mock.patch("simplified_vehicle_models.tracks.profile.load_track", return_value=track),
            mock.patch("simplified_vehicle_models.tracks.line_trajectory.ReferenceCurve"),
            mock.patch("simplified_vehicle_models.tracks.corridor.corridor_from_track"),
            mock.patch("simplified_vehicle_models.formulations.point_mass.point_mass_view_of_planar"),
            mock.patch.object(runner, "physical_map_speed_limit", return_value=10.),
            mock.patch("simplified_vehicle_models.formulations.yaw_capability.build_physical_yaw_map", return_value=cap),
            mock.patch("simplified_vehicle_models.planning.yaw_aware_line.optimize_yaw_aware_line",
                       side_effect=plan_side_effect),
        ]


class RunnerFailureTests(_Common, unittest.TestCase):
    def _run(self, plan_side_effect, **overrides):
        directory = tempfile.mkdtemp()
        args = _args(directory, skip_preflight=True, yaw_map_mode='limited-grid', **overrides)
        patches = self.patches(plan_side_effect)
        for p in patches: p.start()
        try:
            summary = runner.run(args)
        finally:
            for p in reversed(patches): p.stop()
        return Path(args.output_dir), summary

    def test_planning_failure_is_persisted_with_typed_causes_and_nonzero_exit(self):
        from simplified_vehicle_models.planning.yaw_aware_line import YawLineOptimizationError
        records = [{"seed": 0, "accepted": False, "reason": "budget_exceeded", "profile_status": "budget_exceeded",
                    "lap_time": float("nan")},
                   {"seed": 1, "accepted": False, "reason": "capability_domain", "profile_status": "no_path_at_resolution"}]
        output, summary = self._run(YawLineOptimizationError("all rejected", records))
        self.assertEqual(summary["status"], "failed")
        self.assertEqual(summary["failure_stage"], "planning")
        self.assertEqual(summary["causes"], ["budget_exceeded", "no_path_at_resolution"])
        self.assertFalse(summary["lap_completed"])
        for name in ("summary.json", "planning_failure.json", "inputs.json"):
            # strict JSON: NaN must have been converted, not emitted
            text = (output / name).read_text()
            json.loads(text, parse_constant=lambda c: self.fail(f"{name} contains {c}"))
        failure = json.loads((output / "planning_failure.json").read_text())
        self.assertIsNone(failure["candidate_records"][0]["lap_time"])
        self.assertFalse((output / "line.csv").exists())
        self.assertFalse((output / "run_record.npz").exists())
        self.assertEqual(runner.EXIT_CODES[summary["status"]], 4)

    def test_inputs_record_effective_profile_settings_and_map_hash(self):
        from simplified_vehicle_models.planning.yaw_aware_line import YawLineOptimizationError
        output, _ = self._run(YawLineOptimizationError("x", []), profile_wall_seconds=7.,
                              profile_max_candidates=99)
        inputs = json.loads((output / "inputs.json").read_text())
        self.assertEqual(inputs["profile_solver"], "coupled")
        self.assertEqual(inputs["profile_settings"]["max_wall_seconds"], 7.)
        self.assertEqual(inputs["profile_settings"]["max_candidates_per_station"], 99)
        self.assertEqual(len(inputs["map"]["array_sha256"]), 64)
        with np.load(output / "yaw_map.npz") as saved:
            self.assertEqual(saved["feasible_mask"].shape, (3, 3, 3))

    def test_main_returns_nonzero_for_failed_summary(self):
        with mock.patch.object(runner, "parse_args", return_value=SimpleNamespace()), \
             mock.patch.object(runner, "run", return_value={"status": "failed"}):
            self.assertEqual(runner.main([]), 4)
        with mock.patch.object(runner, "parse_args", return_value=SimpleNamespace()), \
             mock.patch.object(runner, "run", return_value={"status": "incomplete"}):
            self.assertEqual(runner.main([]), 3)
        with mock.patch.object(runner, "parse_args", return_value=SimpleNamespace()), \
             mock.patch.object(runner, "run", return_value={"status": "planned"}):
            self.assertEqual(runner.main([]), 0)

    def test_parser_rejects_invalid_profile_budgets(self):
        import contextlib, io
        for flag, value in (("--profile-wall-seconds", "0"), ("--profile-max-candidates", "0"),
                            ("--profile-max-refinements", "-1")):
            with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
                runner.parse_args(["--track", "t.xml", "--output-dir", "o", flag, value])

    def test_strict_json_helper_nulls_nonfinite_and_arrays(self):
        doc = runner._jsonable({"a": np.array([1., np.inf]), "b": float("nan"), "c": np.float64(2.)})
        self.assertEqual(doc, {"a": [1., None], "b": None, "c": 2.})
        json.dumps(doc, allow_nan=False)


class RecordProvenanceRoundTripTests(unittest.TestCase):
    def test_yaw_profile_provenance_survives_save_and_load(self):
        from simplified_vehicle_models.results.run_record import RunRecorder, load_run_record
        args = SimpleNamespace(model="planar-7dof", track=Path("models/tracks/mis_ev_endurance_2024_width.xml"),
                               maximum_speed=None, lateral_margin=.15, control_spacing=60., planning_grid_spacing=.05,
                               planner_iterations=1, planner_passes=1, method="BDF", rtol=1e-6, maximum_step=.05,
                               duration=1., wall_seconds=1., line_csv_sha256="ab" * 32,
                               yaw_profile_provenance={"solver": "coupled", "map_array_sha256": "cd" * 32,
                                                       "resolution_converged": False, "budget_reason": None})
        capability = SimpleNamespace(source_kind="physical_no_tv_v1", source_identity="id", yaw_inertia=1.,
                                     speed=np.array([0., 1.]), longitudinal_acceleration=np.array([-1., 1.]),
                                     lateral_acceleration=np.array([-1., 1.]))
        plan = SimpleNamespace(status="single_pass", settings=None, shape_metrics={}, profile=SimpleNamespace(lap_time=1.))
        result = SimpleNamespace(wall_seconds=.1, time=np.array([0., 1.]), lap_completed=False)
        metadata = runner.record_metadata(args, SimpleNamespace(), SimpleNamespace(name="car"), capability, plan, result)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "rec.npz"
            recorder = RunRecorder(path, state_names=("x",))
            recorder.append(0., 0, [0.]); recorder.append(1., 1, [1.])
            recorder.close(metadata)
            loaded = load_run_record(path, verify_sources=False)
        manifest = loaded.manifest
        self.assertEqual(manifest.model["yaw_profile"]["solver"], "coupled")
        self.assertEqual(manifest.model["yaw_profile"]["map_array_sha256"], "cd" * 32)
        self.assertEqual(manifest.track["optimized_path_sha256"], "ab" * 32)


if __name__ == "__main__":
    unittest.main()
