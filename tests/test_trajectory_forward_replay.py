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

    def test_physical_mapper_opt_in_capture_failure_snapshot_and_legacy_kwargs(self):
        from unittest.mock import patch
        from simplified_vehicle_models.solvers.physical_longitudinal_mapper import PhysicalMapperSettings
        from simplified_vehicle_models.solvers.trajectory_artifact import ExactTrajectoryDriverReference

        loaded = self.make_loaded()
        endpoint = ExactTrajectoryDriverReference(loaded).length
        vehicle = SimpleNamespace(steering_maximum_angle=.5, steering_maximum_rate=1.,
                                  maximum_pedal_force=100., wheelbase=2.)
        names = ("s", "n", "heading_error", "v_x", "v_y", "yaw_rate",
                 "omega_fl", "omega_fr", "omega_rl", "omega_rr", "soc", "steering")
        class Plant:
            state_names = names
            def __init__(self, v, plan):
                self.plan = plan
                seen["plant_reference"] = plan
            def initial_state(self, speed): return np.zeros(len(names))
        class Driver:
            speed_gain = 2.
            brake_governor_margin = .85
            def __init__(self, *a, **kw): pass
        settings = PhysicalMapperSettings(speed_gain=2.)
        seen = {}
        class FakeMapper:
            def __init__(self, v, ref, *, state_names, settings, brake_governor_margin):
                seen.update(vehicle=v, reference=ref, state_names=state_names,
                            settings=settings, margin=brake_governor_margin)
                self.diag = {"callback_queries": np.int64(3), "accepted": False}
            def snapshot_diagnostics(self): return self.diag
        with patch("simplified_vehicle_models.solvers.trajectory_forward_replay.PhysicalLongitudinalMapper", FakeMapper):
            def fail_forward(*args, **kwargs):
                seen["kwargs"] = kwargs
                seen["mapper"] = kwargs["command_mapper"]
                raise RuntimeError("synthetic forward failure")
            evidence, report = replay_exact_trajectory(loaded, vehicle,
                plant_factory=Plant, driver_factory=Driver, forward_solver=fail_forward,
                physical_mapper_settings=settings)
        self.assertIsNone(evidence)
        self.assertEqual(seen["vehicle"], vehicle)
        self.assertIs(seen["reference"], seen["plant_reference"])
        self.assertEqual(seen["state_names"], names)
        self.assertIs(seen["settings"], settings)
        self.assertEqual(seen["margin"], .85)
        self.assertIs(seen["kwargs"]["command_mapper"], seen["mapper"])
        self.assertEqual(report["phase"], "forward")
        self.assertEqual(report["longitudinal_command_mapping"]["diagnostics"]["callback_queries"], 3)
        self.assertFalse(report["accepted"])
        json.dumps(report, allow_nan=False)

        legacy_kwargs = []
        class ShortPlant:
            state_names = ("s", "n", "heading_error", "v_x", "v_y")
            def __init__(self, v, plan): pass
            def initial_state(self, speed): return np.zeros(5)
        class LegacyDriver:
            def __init__(self, *a, **kw): pass
        legacy = SimpleNamespace(time=np.array([0., 1.]), state=np.array([
            [0., endpoint], [0., 0.], [0., 0.], [12., 12.], [0., 0.]]), lap_completed=True)
        _, legacy_report = replay_exact_trajectory(loaded, vehicle, plant_factory=ShortPlant,
            driver_factory=LegacyDriver,
            forward_solver=lambda *a, **kw: (legacy_kwargs.append(kw) or legacy))
        self.assertNotIn("command_mapper", legacy_kwargs[0])
        self.assertEqual(legacy_report["longitudinal_command_mapping"]["mode"], "legacy_acceleration_fraction")

    def test_mapper_snapshot_failure_preserves_forward_error_and_evidence_is_deeply_frozen(self):
        from unittest.mock import patch
        from simplified_vehicle_models.solvers.physical_longitudinal_mapper import PhysicalMapperSettings
        from simplified_vehicle_models.solvers.trajectory_artifact import ExactTrajectoryDriverReference

        loaded = self.make_loaded()
        endpoint = ExactTrajectoryDriverReference(loaded).length
        vehicle = SimpleNamespace(steering_maximum_angle=.5, steering_maximum_rate=1.,
                                  maximum_pedal_force=100., wheelbase=2.)
        names = ("s", "n", "heading_error", "v_x", "v_y", "yaw_rate",
                 "omega_fl", "omega_fr", "omega_rl", "omega_rr", "soc", "steering")
        class Plant:
            state_names = names
            def __init__(self, v, plan): pass
            def initial_state(self, speed): return np.zeros(len(names))
        class Driver:
            speed_gain = 2.
            brake_governor_margin = .85
            def __init__(self, *a, **kw): pass
        class FakeMapper:
            def __init__(self, *a, **kw): self.diag = {"accepted": False, "nested": {"items": [1, 2]}}
            def snapshot_diagnostics(self): raise RuntimeError("BADSNAP")
        with patch("simplified_vehicle_models.solvers.trajectory_forward_replay.PhysicalLongitudinalMapper", FakeMapper):
            _, report = replay_exact_trajectory(loaded, vehicle, plant_factory=Plant,
                driver_factory=Driver, physical_mapper_settings=PhysicalMapperSettings(),
                forward_solver=lambda *a, **k: (_ for _ in ()).throw(RuntimeError("SOLVERFAILED")))
        self.assertEqual(report["phase"], "forward")
        self.assertIn("SOLVERFAILED", report["error"])
        self.assertIn("BADSNAP", report["longitudinal_command_mapping"]["diagnostics_error"])
        self.assertIsNone(report["longitudinal_command_mapping"]["diagnostics"])
        json.dumps(report, allow_nan=False)

        result = SimpleNamespace(time=np.array([0., 1.]), state=np.array([
            [0., endpoint], [0., 0.], [0., 0.], [12., 12.], [0., 0.]]),
            lap_completed=True)
        class ShortPlant:
            state_names = ("s", "n", "heading_error", "v_x", "v_y")
            def __init__(self, v, plan): pass
            def initial_state(self, speed): return np.zeros(5)
        evidence, returned = replay_exact_trajectory(loaded, vehicle,
            plant_factory=ShortPlant, driver_factory=lambda *a, **k: object(),
            forward_solver=lambda *a, **k: result)
        with self.assertRaises(TypeError): evidence.report["longitudinal_command_mapping"]["mode"] = "changed"
        with self.assertRaises(TypeError): evidence.metrics["max_abs_speed_error_mps"] = 99.
        self.assertIsInstance(evidence.report["longitudinal_command_mapping"], type(evidence.report))
        returned["longitudinal_command_mapping"]["mode"] = "mutated-return"
        self.assertEqual(evidence.report["longitudinal_command_mapping"]["mode"], "legacy_acceleration_fraction")
        json.dumps(returned, allow_nan=False)

    def test_physical_mapper_bad_type_and_gain_mismatch_fail_closed(self):
        from simplified_vehicle_models.solvers.physical_longitudinal_mapper import PhysicalMapperSettings
        loaded = self.make_loaded()
        vehicle = SimpleNamespace(steering_maximum_angle=.5, steering_maximum_rate=1.,
                                  maximum_pedal_force=100., wheelbase=2.)
        calls = []
        _, bad = replay_exact_trajectory(loaded, vehicle, physical_mapper_settings=object(),
                                        forward_solver=lambda *a, **k: calls.append(True))
        self.assertIn("PhysicalMapperSettings", bad["error"])
        self.assertEqual(calls, [])
        class Plant:
            state_names = ("s", "n", "heading_error", "v_x", "v_y", "yaw_rate",
                           "omega_fl", "omega_fr", "omega_rl", "omega_rr", "soc", "steering")
            def __init__(self, v, p): pass
            def initial_state(self, speed): return np.zeros(12)
        class Driver:
            speed_gain = 3.
            brake_governor_margin = .85
            def __init__(self, *a, **kw): pass
        _, mismatch = replay_exact_trajectory(loaded, vehicle, plant_factory=Plant,
            driver_factory=Driver, physical_mapper_settings=PhysicalMapperSettings(),
            forward_solver=lambda *a, **k: calls.append(True))
        self.assertIn("speed_gain", mismatch["error"])
        self.assertEqual(calls, [])
        self.assertFalse(mismatch["accepted"])

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
