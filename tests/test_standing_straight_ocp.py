import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.solvers.standing_straight_ocp import (
    StandingStraightSettings,
    solve_standing_straight,
)


class StandingStraightOcpTests(unittest.TestCase):
    def test_constant_cap_solves_actual_time_optimum_and_is_immutable(self):
        for intervals in (4, 8):
            result, report = solve_standing_straight(
                object(), 75.3, settings=StandingStraightSettings(intervals=intervals),
                acceleration_limit=lambda vehicle, speed, curvature: 3.0)
            self.assertIsNotNone(result, report)
            self.assertFalse(report["accepted"])
            self.assertTrue(report["optimizer_converged"])
            self.assertTrue(report["heldout_sampling_complete"])
            self.assertTrue(report["sampled_feasible"])
            self.assertEqual(result.time[0], 0.0)
            self.assertEqual(result.distance[0], 0.0)
            self.assertEqual(result.speed[0], 0.0)
            self.assertTrue(np.all(np.diff(result.time) > 0))
            self.assertTrue(np.all(np.diff(result.distance) >= 0))
            self.assertTrue(np.all(np.diff(result.speed) >= 0))
            self.assertAlmostEqual(result.distance[-1], 75.3, delta=1e-7)
            self.assertLessEqual(abs(report["finish_distance_error_m"]), 1e-7)
            self.assertAlmostEqual(result.total_time, np.sqrt(2 * 75.3 / 3), delta=3e-5)
            for array in (result.time, result.distance, result.speed, result.acceleration):
                self.assertFalse(array.flags.writeable)
            self.assertTrue(result.fixed_straight_only)
            self.assertFalse(result.joint_line_optimized)
            self.assertFalse(result.physical_feasibility_certified)
            json.dumps(report, allow_nan=False)

    def test_acceleration_cap_can_increase_above_rest_cap_and_default_is_original(self):
        calls = []
        def cap(vehicle, speed, curvature):
            calls.append((speed, curvature))
            return 1.0 if speed < 2.0 else 5.0
        result, report = solve_standing_straight(
            object(), 20.0, settings=StandingStraightSettings(intervals=3), acceleration_limit=cap)
        self.assertIsNotNone(result, report)
        self.assertGreater(float(np.max(result.acceleration)), 1.0)
        self.assertTrue(all(k == 0 for _, k in calls))
        self.assertTrue(any(v == 0 for v, _ in calls))

        from simplified_vehicle_models.formulations import point_mass
        observed = []
        def original(vehicle, speed, curvature):
            observed.append((speed, curvature))
            return 3.0
        with patch.object(point_mass, "maximum_acceleration_limit", original):
            result, report = solve_standing_straight(object(), 10.0)
        self.assertIsNotNone(result, report)
        self.assertTrue(any(speed == 0 and curvature == 0 for speed, curvature in observed))
        self.assertTrue(any(speed > 0 and curvature == 0 for speed, curvature in observed))

    def test_invalid_inputs_and_settings_fail_before_callback(self):
        calls = []
        def cap(*args):
            calls.append(args); return 2.0
        for distance in (0.0, float("nan"), -1.0, True):
            result, report = solve_standing_straight(object(), distance, acceleration_limit=cap)
            self.assertIsNone(result); self.assertFalse(report["accepted"])
            json.dumps(report, allow_nan=False)
        for settings in (StandingStraightSettings(heldout_fractions=2),
                         StandingStraightSettings(tolerance=float("nan")),
                         StandingStraightSettings(intervals=True)):
            result, report = solve_standing_straight(object(), 10., settings=settings, acceleration_limit=cap)
            self.assertIsNone(result); self.assertFalse(report["accepted"])
        self.assertEqual(calls, [])

    def test_invalid_cap_query_and_slow_callback_budgets_never_accept(self):
        for value in (float("nan"), -1.0, 0.0):
            result, report = solve_standing_straight(object(), 10., acceleration_limit=lambda *_: value)
            self.assertIsNone(result); self.assertFalse(report["accepted"])
            json.dumps(report, allow_nan=False)
        result, report = solve_standing_straight(
            object(), 10., settings=StandingStraightSettings(maximum_limit_queries=1),
            acceleration_limit=lambda *_: 2.)
        self.assertIsNone(result); self.assertFalse(report["accepted"])
        self.assertIn("budget", report["failure"])
        ticks = iter([0., 0., 2.] + [2.] * 30)
        with patch("simplified_vehicle_models.solvers.standing_straight_ocp.time.monotonic", side_effect=lambda: next(ticks)):
            result, report = solve_standing_straight(
                object(), 10., settings=StandingStraightSettings(maximum_wall_seconds=1.),
                acceleration_limit=lambda *_: 2.)
        self.assertIsNone(result); self.assertFalse(report["optimizer_converged"])
        self.assertFalse(report["heldout_sampling_complete"])
        self.assertFalse(report["accepted"])

    def test_fake_success_invalid_negative_acceleration_and_interval_bounds_rejected(self):
        for badx in (np.array([1., 1., -1., 6.]), np.array([1e-8, 2., 2., 2.]),
                     np.array([1., 1., 2., 2.])):
            fake = SimpleNamespace(success=True, status=0, message="success", x=badx, nit=4)
            with patch("scipy.optimize.minimize", return_value=fake):
                result, report = solve_standing_straight(
                    object(), 10., settings=StandingStraightSettings(intervals=2),
                    acceleration_limit=lambda *_: 20.)
            self.assertIsNone(result, report)
            self.assertFalse(report["accepted"])

    def test_optimizer_failure_preserves_last_finite_iterate_and_exception(self):
        fake = SimpleNamespace(success=False, status=9, message="iteration limit", x=np.array([1., 1., 2., 2.]), nit=7)
        with patch("scipy.optimize.minimize", return_value=fake):
            result, report = solve_standing_straight(
                object(), 10., settings=StandingStraightSettings(intervals=2),
                acceleration_limit=lambda *_: 20.)
        self.assertIsNone(result)
        self.assertFalse(report["optimizer_converged"])
        self.assertEqual(report["iterations"], 7)
        self.assertEqual(len(report["last_finite_time_s"]), 3)
        self.assertIn("last_finite_finish_distance_error_m", report)

    def test_wall_budget_after_complete_report_serialization_returns_no_candidate(self):
        fake = SimpleNamespace(success=True, status=0, message="success", x=np.array([.5, .5, 20., 20.]), nit=1)
        module = __import__("simplified_vehicle_models.solvers.standing_straight_ocp", fromlist=["_jsonsafe"])
        real_dumps = json.dumps
        clock = [0.]
        serialization_completed = []
        def advance_after_success_report(*args, **kwargs):
            encoded = real_dumps(*args, **kwargs)
            if args and isinstance(args[0], dict) and args[0].get("optimizer_converged"):
                serialization_completed.append(True)
                clock[0] = 2.
            return encoded
        with patch("scipy.optimize.minimize", return_value=fake), \
             patch.object(module.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(module.json, "dumps", side_effect=advance_after_success_report):
            result, report = solve_standing_straight(
                object(), 10., settings=StandingStraightSettings(intervals=2, maximum_wall_seconds=1.),
                acceleration_limit=lambda *_: 20.)
        self.assertTrue(serialization_completed)
        self.assertIsNone(result)
        self.assertEqual(report["failure"], "budget_exhausted")
        self.assertTrue(report["solver_reported_success"])
        self.assertFalse(report["optimizer_converged"])
        self.assertFalse(report["heldout_sampling_complete"])
        self.assertFalse(report["accepted"])
        real_dumps(report, allow_nan=False)

    def test_wall_budget_is_checked_before_cached_limit_hit(self):
        calls = []
        def cap(vehicle, speed, curvature):
            calls.append((speed, curvature))
            return 20.
        settings = StandingStraightSettings(intervals=2, maximum_wall_seconds=1.)
        clock = [0.]
        def fake_minimize(fun, x0, **kwargs):
            # The rest-speed limit was queried before optimization and is cached.
            # Make a repeated rest-speed query cross the budget before it can hit cache.
            clock[0] = 2.
            kwargs["constraints"][1]["fun"](x0)
            raise AssertionError("expired cache query should have stopped optimization")
        module = __import__("simplified_vehicle_models.solvers.standing_straight_ocp", fromlist=["time"])
        with patch("scipy.optimize.minimize", side_effect=fake_minimize), \
             patch.object(module.time, "monotonic", side_effect=lambda: clock[0]):
            result, report = solve_standing_straight(object(), 10., settings=settings, acceleration_limit=cap)
        self.assertIsNone(result)
        self.assertEqual(report["failure"], "budget_exhausted")
        self.assertEqual(calls, [(0.0, 0.0)])
        self.assertEqual(report["limit_queries"], 1)
        self.assertFalse(report["optimizer_converged"])
        self.assertFalse(report["heldout_sampling_complete"])
        self.assertFalse(report["accepted"])

    def test_unsuccessful_optimizer_and_heldout_dip_never_accept(self):
        fake = SimpleNamespace(success=False, status=9, message="iteration limit", x=np.array([1., 1., 2., 2.]))
        with patch("scipy.optimize.minimize", return_value=fake):
            result, report = solve_standing_straight(object(), 10., acceleration_limit=lambda *_: 2.,
                                                     settings=StandingStraightSettings(intervals=2))
        self.assertIsNone(result); self.assertFalse(report["accepted"])

        def dip(vehicle, speed, curvature): return .01 if 2.49 < speed < 2.51 else 20.
        fake = SimpleNamespace(success=True, status=0, message="success", x=np.array([.5, .5, 20., 20.]))
        with patch("scipy.optimize.minimize", return_value=fake):
            result, report = solve_standing_straight(object(), 10., acceleration_limit=dip,
                                                     settings=StandingStraightSettings(intervals=2))
        self.assertIsNone(result)
        self.assertTrue(report["solver_reported_success"])
        self.assertFalse(report["accepted"])
        self.assertTrue(report["heldout_sampling_complete"])
        self.assertFalse(report["heldout_feasible"])


if __name__ == "__main__":
    unittest.main()
