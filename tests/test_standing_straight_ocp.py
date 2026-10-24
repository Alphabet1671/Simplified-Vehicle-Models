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
                object(), 75.3,
                settings=StandingStraightSettings(intervals=intervals),
                acceleration_limit=lambda vehicle, speed, curvature: 3.0,
            )
            self.assertIsNotNone(result, report)
            self.assertFalse(report["accepted"])
            self.assertTrue(report["optimizer_converged"])
            self.assertTrue(report["sampled_feasible"])
            self.assertEqual(result.time[0], 0.0)
            self.assertEqual(result.distance[0], 0.0)
            self.assertEqual(result.speed[0], 0.0)
            self.assertAlmostEqual(result.distance[-1], 75.3, delta=1e-7)
            self.assertAlmostEqual(result.total_time, np.sqrt(2 * 75.3 / 3), delta=3e-5)
            for array in (result.time, result.distance, result.speed, result.acceleration):
                self.assertFalse(array.flags.writeable)
            self.assertTrue(result.fixed_straight_only)
            self.assertFalse(result.joint_line_optimized)
            json.dumps(report, allow_nan=False)

    def test_acceleration_cap_can_increase_above_rest_cap(self):
        calls = []
        def cap(vehicle, speed, curvature):
            calls.append((speed, curvature))
            return 1.0 if speed < 2.0 else 5.0
        result, report = solve_standing_straight(
            object(), 20.0, settings=StandingStraightSettings(intervals=3), acceleration_limit=cap
        )
        self.assertIsNotNone(result, report)
        self.assertGreater(float(np.max(result.acceleration)), 1.0)
        self.assertTrue(all(k == 0 for _, k in calls))
        self.assertTrue(any(v == 0 for v, _ in calls))

    def test_invalid_inputs_fail_before_callback_and_are_json_safe(self):
        calls = []
        def cap(*args):
            calls.append(args)
            return 2.0
        for distance in (0.0, float("nan"), -1.0, True):
            result, report = solve_standing_straight(object(), distance, acceleration_limit=cap)
            self.assertIsNone(result)
            self.assertFalse(report["accepted"])
            json.dumps(report, allow_nan=False)
        self.assertEqual(calls, [])

    def test_invalid_cap_and_query_budget_never_accept(self):
        for value in (float("nan"), -1.0, 0.0):
            result, report = solve_standing_straight(
                object(), 10.0, acceleration_limit=lambda *_: value
            )
            self.assertIsNone(result)
            self.assertFalse(report["accepted"])
        result, report = solve_standing_straight(
            object(), 10.0,
            settings=StandingStraightSettings(maximum_limit_queries=1),
            acceleration_limit=lambda *_: 2.0,
        )
        self.assertIsNone(result)
        self.assertFalse(report["accepted"])
        self.assertIn("budget", report["failure"])

    def test_failure_status_and_heldout_dip_cannot_fake_success(self):
        result, report = solve_standing_straight(
            object(), 10.0, acceleration_limit=lambda *_: 2.0,
            settings=StandingStraightSettings(maximum_iterations=1),
        )
        self.assertIsNone(result)
        self.assertFalse(report["accepted"])

        def dip(vehicle, speed, curvature):
            return 0.01 if 2.49 < speed < 2.51 else 20.0
        # Successful optimizer trace: interval midpoints are 0, 5, 15, 20;
        # held-out quarter-cell samples hit the narrow dip at speed 2.5.
        fake = SimpleNamespace(success=True, status=0, message="success",
                               x=np.array([0.5, 0.5, 20.0, 20.0]))
        with patch("scipy.optimize.minimize", return_value=fake):
            result, report = solve_standing_straight(
                object(), 10.0, acceleration_limit=dip,
                settings=StandingStraightSettings(intervals=2),
            )
        self.assertIsNone(result)
        self.assertTrue(report["optimizer_converged"])
        self.assertFalse(report["accepted"])
        self.assertFalse(report["heldout_feasible"])


if __name__ == "__main__":
    unittest.main()
