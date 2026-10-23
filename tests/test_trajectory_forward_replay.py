"""Fast contract tests for bounded serialized-trajectory replay."""
import unittest
import json
import tempfile
from pathlib import Path
from types import SimpleNamespace

from tests import test_trajectory_artifact as artifact_tests
from tools.run.solve_minimum_time_line import _write_trajectory_metadata
from simplified_vehicle_models.solvers.trajectory_artifact import load_trajectory_artifact

import numpy as np

from simplified_vehicle_models.solvers.trajectory_forward_replay import (
    EndpointHoldReferenceConsumer, FrozenReplaySettings, replay_exact_trajectory,
)


class ReplayTests(unittest.TestCase):
    def make_loaded(self):
        helper = artifact_tests.TrajectoryArtifactTest()
        solution, _ = helper.make_solution(False)
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "trajectory.npz"
            _write_trajectory_metadata(path, solution)
            return load_trajectory_artifact(path)

    def test_settings_guard_types_and_ceiling(self):
        for kwargs in ({"duration_seconds": True}, {"preview_seconds": -1},
                       {"maximum_wall_seconds": 301}, {"rtol": np.inf}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                FrozenReplaySettings(**kwargs)

    def test_open_hold_explicit_and_closed_endpoint_preserved(self):
        class Ref:
            length = 10.
            is_closed = False
            distance = speed = curvature = acceleration = np.array([0., 10.])
            def sample(self, d):
                if d < 0 or d > 10:
                    raise ValueError("strict")
                return d, d + 1, d + 2
        ref = EndpointHoldReferenceConsumer(Ref())
        self.assertEqual(ref.sample(-2), (0., 1., 2.))
        self.assertEqual(ref.sample(12), (10., 11., 12.))
        self.assertEqual(ref.outside_query_count, 2)
        self.assertEqual(ref.maximum_outside_distance, 2)
        self.assertEqual(ref.first_outside_before, -2.)
        self.assertEqual(ref.first_outside_after, 12.)
        with self.assertRaises(ValueError): ref.sample(True)
        with self.assertRaises(ValueError): ref.sample(np.nan)
        class Closed(Ref):
            is_closed = True
            def sample(self, d): return (d, d, d)
        closed = EndpointHoldReferenceConsumer(Closed())
        self.assertEqual(closed.sample(10), (10, 10, 10))
        self.assertEqual(closed.outside_query_count, 0)

    def test_fake_replay_layout_metrics_and_fail_closed_flags(self):
        class Ref:
            length = 10.; is_closed = False
            distance = speed = curvature = acceleration = np.array([0., 10.])
            def sample(self, d): return (10., 0., 0.)
        loaded = self.make_loaded()
        from simplified_vehicle_models.solvers.trajectory_artifact import ExactTrajectoryDriverReference
        endpoint = ExactTrajectoryDriverReference(loaded).length
        vehicle = SimpleNamespace(steering_maximum_angle=.5, steering_maximum_rate=1.,
                                  maximum_pedal_force=100., wheelbase=2.)
        class Plant:
            state_names = ("s", "n", "heading_error", "v_x", "v_y")
            state_size = 5
            def __init__(self, vehicle, plan): self.plan = plan
            def initial_state(self, speed): return np.zeros(5)
        class Driver:
            def __init__(self, *a, **kw): pass
        result = SimpleNamespace(time=np.array([0., 1.]), state=np.array([
            [0., endpoint], [1., 2.], [0., .1], [10., 12.], [0., 0.]]),
            lap_completed=True, evaluations=4, wall_seconds=.2, message="fake finished",
            midpoint_sample_validation_complete=False)
        ev, report = replay_exact_trajectory(loaded, vehicle,
            plant_factory=Plant, driver_factory=Driver, forward_solver=lambda *a, **kw: result)
        self.assertTrue(report["completed"])
        self.assertFalse(report["accepted"])
        self.assertFalse(report["U3_qualified"])
        self.assertEqual(ev.state.shape, (5, 2))
        self.assertFalse(ev.state.flags.writeable)
        self.assertAlmostEqual(ev.metrics["max_abs_speed_error_mps"], 2.)
        self.assertFalse(ev.metrics["midpoint_sample_validation_complete"])
        self.assertEqual(ev.metrics["forward_message"], "fake finished")
        json.dumps({"report": report, "metrics": dict(ev.metrics)}, allow_nan=False)
        expected_time_delta = 1. - loaded.physical_time
        self.assertAlmostEqual(ev.metrics["time_vs_reference_seconds_diagnostic_only"], abs(expected_time_delta))
        self.assertAlmostEqual(ev.metrics["time_vs_reference_signed_seconds_diagnostic_only"], expected_time_delta)
        self.assertAlmostEqual(ev.metrics["time_vs_reference_relative_diagnostic_only"], abs(expected_time_delta) / loaded.physical_time)
        self.assertAlmostEqual(ev.metrics["time_difference_absolute_seconds"], abs(expected_time_delta))
        self.assertAlmostEqual(ev.metrics["time_difference_signed_relative"], expected_time_delta / loaded.physical_time)

    def test_closed_path_periodicity_excludes_progress_and_soc(self):
        helper = artifact_tests.TrajectoryArtifactTest()
        solution, _ = helper.make_solution(True)
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "closed.npz"
            _write_trajectory_metadata(path, solution)
            loaded = load_trajectory_artifact(path)
        from simplified_vehicle_models.solvers.trajectory_artifact import ExactTrajectoryDriverReference
        endpoint = ExactTrajectoryDriverReference(loaded).length
        vehicle = SimpleNamespace(steering_maximum_angle=.5, steering_maximum_rate=1.,
                                  maximum_pedal_force=100., wheelbase=2.)
        class Plant:
            state_names = ("s", "n", "heading_error", "v_x", "v_y", "soc")
            state_size = 6
            periodic_indices = (1, 2, 3, 4)
            periodic_tolerances = np.array([.001, .001, .01, .01])
            def __init__(self, v, p): pass
            def initial_state(self, speed): return np.zeros(6)
        class Driver:
            def __init__(self, *a, **kw): pass
        result = SimpleNamespace(time=np.array([0., 1.]), state=np.array([
            [0., endpoint], [0., .0005], [0., .0005], [12., 12.001],
            [0., 0.001], [.8, .2]]), lap_completed=True,
            midpoint_sample_validation_complete=True, message="closed fake")
        evidence, report = replay_exact_trajectory(loaded, vehicle, plant_factory=Plant,
            driver_factory=Driver, forward_solver=lambda *a, **kw: result)
        self.assertTrue(report["completed"])
        self.assertTrue(evidence.metrics["periodic_within_published_tolerances"])
        self.assertNotIn("soc", evidence.metrics["periodic_state_delta"])
        self.assertNotIn("s", evidence.metrics["periodic_state_delta"])

    def test_nonfinite_diagnostic_is_null_and_annotated_in_report(self):
        from simplified_vehicle_models.solvers.trajectory_artifact import ExactTrajectoryDriverReference
        loaded = self.make_loaded()
        endpoint = ExactTrajectoryDriverReference(loaded).length
        vehicle = SimpleNamespace(steering_maximum_angle=.5, steering_maximum_rate=1.,
                                  maximum_pedal_force=100., wheelbase=2.)
        class Plant:
            state_names = ("s", "n", "heading_error", "v_x", "v_y")
            def __init__(self, v, p): pass
            def initial_state(self, speed): return np.zeros(5)
        class Driver:
            def __init__(self, *a, **kw): pass
        result = SimpleNamespace(time=np.array([0., 1.]), state=np.array([
            [0., endpoint], [0., 0.], [0., 0.], [12., 12.], [0., 0.]]),
            lap_completed=True, message="nonfinite diagnostic",
            evaluations=np.nan, midpoint_sample_validation_complete=False)
        evidence, report = replay_exact_trajectory(loaded, vehicle, plant_factory=Plant,
            driver_factory=Driver, forward_solver=lambda *a, **kw: result)
        self.assertIsNone(evidence.metrics["evaluations"])
        self.assertIn("metrics.evaluations", report["nonfinite_metadata_fields"])
        json.dumps({"report": report, "metrics": dict(evidence.metrics)}, allow_nan=False)

    def test_numpy_bool_completion_remains_json_serializable(self):
        from simplified_vehicle_models.solvers.trajectory_artifact import ExactTrajectoryDriverReference
        loaded = self.make_loaded()
        endpoint = ExactTrajectoryDriverReference(loaded).length
        vehicle = SimpleNamespace(steering_maximum_angle=.5, steering_maximum_rate=1.,
                                  maximum_pedal_force=100., wheelbase=2.)
        class Plant:
            state_names = ("s", "n", "heading_error", "v_x", "v_y")
            def __init__(self, v, p): pass
            def initial_state(self, speed): return np.zeros(5)
        class Driver:
            def __init__(self, *a, **kw): pass
        result = SimpleNamespace(time=np.array([0., 1.]), state=np.array([
            [0., endpoint], [0., 0.], [0., 0.], [12., 12.], [0., 0.]]),
            lap_completed=np.bool_(True), evaluations=np.int64(3),
            wall_seconds=np.float64(.1), message="numpy typed")
        evidence, report = replay_exact_trajectory(loaded, vehicle, plant_factory=Plant,
            driver_factory=Driver, forward_solver=lambda *a, **kw: result)
        self.assertIs(report["completed"], True)
        json.dumps({"report": report, "metrics": dict(evidence.metrics)}, allow_nan=False)

    def test_periodic_empty_applicable_set_is_not_vacuous_pass(self):
        helper = artifact_tests.TrajectoryArtifactTest()
        solution, _ = helper.make_solution(True)
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "closed.npz"
            _write_trajectory_metadata(path, solution)
            loaded = load_trajectory_artifact(path)
        from simplified_vehicle_models.solvers.trajectory_artifact import ExactTrajectoryDriverReference
        endpoint = ExactTrajectoryDriverReference(loaded).length
        vehicle = SimpleNamespace(steering_maximum_angle=.5, steering_maximum_rate=1.,
                                  maximum_pedal_force=100., wheelbase=2.)
        class Plant:
            state_names = ("s", "n", "heading_error", "v_x", "v_y", "soc")
            state_size = 6
            periodic_indices = (0, 5)
            periodic_tolerances = np.array([.01, .01])
            def __init__(self, v, p): pass
            def initial_state(self, speed): return np.zeros(6)
        class Driver:
            def __init__(self, *a, **kw): pass
        result = SimpleNamespace(time=np.array([0., 1.]), state=np.array([
            [0., endpoint], [0., 0.], [0., 0.], [12., 12.], [0., 0.], [.8, .2]]),
            lap_completed=True, midpoint_sample_validation_complete=True)
        evidence, _ = replay_exact_trajectory(loaded, vehicle, plant_factory=Plant,
            driver_factory=Driver, forward_solver=lambda *a, **kw: result)
        self.assertIsNone(evidence.metrics["periodic_state_delta"])
        self.assertIsNone(evidence.metrics["periodic_within_published_tolerances"])

    def test_periodic_metadata_rejects_nonpositive_tolerance(self):
        helper = artifact_tests.TrajectoryArtifactTest()
        solution, _ = helper.make_solution(True)
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "closed.npz"
            _write_trajectory_metadata(path, solution)
            loaded = load_trajectory_artifact(path)
        from simplified_vehicle_models.solvers.trajectory_artifact import ExactTrajectoryDriverReference
        endpoint = ExactTrajectoryDriverReference(loaded).length
        vehicle = SimpleNamespace(steering_maximum_angle=.5, steering_maximum_rate=1.,
                                  maximum_pedal_force=100., wheelbase=2.)
        class Plant:
            state_names = ("s", "n", "heading_error", "v_x", "v_y", "soc")
            periodic_indices = (1, 2)
            periodic_tolerances = np.array([.01, 0.])
            def __init__(self, v, p): pass
            def initial_state(self, speed): return np.zeros(6)
        class Driver:
            def __init__(self, *a, **kw): pass
        result = SimpleNamespace(time=np.array([0., 1.]), state=np.array([
            [0., endpoint], [0., 0.], [0., 0.], [12., 12.], [0., 0.], [.8, .2]]),
            lap_completed=True)
        evidence, report = replay_exact_trajectory(loaded, vehicle, plant_factory=Plant,
            driver_factory=Driver, forward_solver=lambda *a, **kw: result)
        self.assertIsNone(evidence)
        self.assertIn("periodicity metadata", report["error"])

    def test_soft_budget_includes_setup_and_skips_forward(self):
        from unittest.mock import patch
        loaded = self.make_loaded()
        vehicle = SimpleNamespace(steering_maximum_angle=.5, steering_maximum_rate=1.,
                                  maximum_pedal_force=100., wheelbase=2.)
        called = []
        with patch("simplified_vehicle_models.solvers.trajectory_forward_replay.perf_counter",
                   side_effect=[0., 2., 2.]):
            evidence, report = replay_exact_trajectory(loaded, vehicle,
                settings=FrozenReplaySettings(maximum_wall_seconds=1.),
                forward_solver=lambda *a, **k: called.append(True))
        self.assertIsNone(evidence)
        self.assertEqual(called, [])
        self.assertEqual(report["phase"], "wall_budget_before_forward")

    def test_bad_forward_result_fails_closed(self):
        vehicle = SimpleNamespace(steering_maximum_angle=.5, steering_maximum_rate=1.,
                                  maximum_pedal_force=100., wheelbase=2.)
        class Plant:
            state_names = ("s", "n", "heading_error", "v_x", "v_y")
            def __init__(self, v, p): pass
            def initial_state(self, speed): return np.zeros(5)
        class Driver:
            def __init__(self, *a, **kw): pass
        loaded = self.make_loaded()
        bad = SimpleNamespace(time=np.array([0., 1.]), state=np.full((5, 2), np.nan), lap_completed=True)
        evidence, report = replay_exact_trajectory(loaded, vehicle, plant_factory=Plant,
            driver_factory=Driver, forward_solver=lambda *a, **kw: bad)
        self.assertIsNone(evidence)
        self.assertFalse(report["accepted"])
        self.assertIn("invalid", report["error"])


if __name__ == "__main__":
    unittest.main()
