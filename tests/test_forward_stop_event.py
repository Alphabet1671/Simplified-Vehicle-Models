"""Focused API contract tests for the optional forward speed-stop event."""
import unittest

import numpy as np

from simplified_vehicle_models.solvers.forward_driver import (
    DriverObservation, PreviewDriver, RateSystem, build_speed_plan, solve_forward,
)
from simplified_vehicle_models.solvers.braking_envelope import BrakingEnvelope
from simplified_vehicle_models.solvers.envelope_driver import EnvelopePolicy, EnvelopeDriverSettings


def _driver(end_distance=100.):
    distance = np.array([0., end_distance])
    plan = build_speed_plan(distance, np.zeros(2), open_course=True)
    policy = EnvelopePolicy(BrakingEnvelope(
        distance, np.zeros(2), np.full(2, 20.), np.full(2, 20.),
        np.full(1, 1.), np.zeros(2, dtype=int), False, 400., {"fixture": True}),
        EnvelopeDriverSettings())
    return PreviewDriver(plan, 1.5, .5, 2., 1000., envelope_policy=policy)


class _Plant:
    state_size = 3

    def __init__(self, speed_rate=-1., distance_rate=1.):
        self.speed_rate = speed_rate
        self.distance_rate = distance_rate

    def observe(self, state):
        return DriverObservation(float(state[1]), 0., 0., float(state[0]), 0., 0.)

    def evaluate(self, time, state, command):
        return RateSystem(np.array([self.speed_rate, self.distance_rate, 0.]))


class ForwardStopEventTests(unittest.TestCase):
    def test_none_preserves_path_end_and_result_contract(self):
        result = solve_forward(_Plant(), _driver(.5), np.array([1., 0., 7.]),
                               duration=2., maximum_step=.1, method="Radau")
        self.assertTrue(result.lap_completed)
        self.assertAlmostEqual(result.state[1, -1], .5, places=7)
        self.assertEqual(result.state.shape[0], 3)

    def test_speed_root_is_downward_full_state_and_one_observer_row(self):
        plant, rows = _Plant(speed_rate=-.05), []
        result = solve_forward(plant, _driver(), np.array([1., 0., 7.]), duration=20.,
                               maximum_step=.1, method="Radau", stop_speed_mps=.01,
                               step_sink=rows.append)
        self.assertFalse(result.lap_completed)
        self.assertAlmostEqual(result.state[0, -1], .01, places=8)
        self.assertAlmostEqual(result.state[1, -1], 19.8, places=7)
        self.assertEqual(result.state.shape[0], 3)
        self.assertEqual(result.state[2, -1], 7.)
        self.assertEqual(len(rows), result.time.size)
        self.assertEqual(rows[-1].index, len(rows) - 1)
        self.assertAlmostEqual(rows[-1].state[0], .01, places=8)
        self.assertTrue(np.all(np.diff(result.time) > 0.))

    def test_course_finish_before_speed_stop_marks_lap_completed(self):
        result = solve_forward(_Plant(speed_rate=-1., distance_rate=1.), _driver(.5),
                               np.array([1., 0., 7.]), duration=2., maximum_step=.1,
                               method="Radau", stop_speed_mps=.01)
        self.assertTrue(result.lap_completed)
        self.assertAlmostEqual(result.state[1, -1], .5, places=7)
        self.assertGreater(result.state[0, -1], .01)

    def test_invalid_thresholds_and_initial_threshold_rejected(self):
        for threshold in (0., -1., np.nan, np.inf, -np.inf, True, False, "0.1"):
            with self.subTest(threshold=threshold), self.assertRaises(ValueError):
                solve_forward(_Plant(), _driver(), np.array([1., 0., 7.]),
                              duration=.1, stop_speed_mps=threshold)
        for initial_speed in (.01, .005, 0.):
            with self.subTest(initial_speed=initial_speed), self.assertRaisesRegex(
                    ValueError, "initial body speed"):
                solve_forward(_Plant(), _driver(), np.array([initial_speed, 0., 7.]),
                              duration=.1, stop_speed_mps=.01)

    def test_observer_parity_and_observer_exclusivity(self):
        args = (_Plant(), _driver(), np.array([1., 0., 7.]))
        plain = solve_forward(*args, duration=.5, maximum_step=.1, method="Radau",
                              stop_speed_mps=.01)
        rows = []
        observed = solve_forward(*args, duration=.5, maximum_step=.1, method="Radau",
                                 stop_speed_mps=.01, policy_sink=rows.append)
        np.testing.assert_array_equal(observed.time, plain.time)
        np.testing.assert_array_equal(observed.state, plain.state)
        self.assertEqual(len(rows), observed.time.size)
        with self.assertRaisesRegex(ValueError, "alternative observers"):
            solve_forward(_Plant(), _driver(), np.array([1., 0., 7.]), duration=.1,
                          stop_speed_mps=.01, step_sink=lambda _: None,
                          policy_sink=lambda _: None)


if __name__ == "__main__":
    unittest.main()
