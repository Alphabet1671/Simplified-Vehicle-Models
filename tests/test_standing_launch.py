"""Focused tests for original-model time-domain launch integration."""
import unittest
from types import SimpleNamespace

import numpy as np

from simplified_vehicle_models.solvers.standing_launch import LaunchSettings, integrate_standing_launch
from simplified_vehicle_models.tracks.line_trajectory import StationMap


class FakeTrajectory:
    def __init__(self, end=100.0, curvature_fn=None, ratio=1.0):
        self.station_map = StationMap(np.array([0., end / ratio]), np.array([0., end]), False)
        self.curvature_fn = curvature_fn or (lambda s: .0)
    def evaluate(self, s):
        return SimpleNamespace(curvature=float(self.curvature_fn(s)))


class StandingLaunchTest(unittest.TestCase):
    def test_constant_acceleration_closed_form_and_course_remainder(self):
        result = integrate_standing_launch(None, FakeTrajectory(),
            settings=LaunchSettings(transition_speed=2., maximum_time=3.),
            acceleration_limit=lambda vehicle, v, k: 2.)
        self.assertAlmostEqual(result.transition_time, 1., places=8)
        self.assertAlmostEqual(result.transition_distance, 1., places=8)
        self.assertAlmostEqual(result.remaining_path_length, 99., places=8)
        self.assertEqual(result.original_path_length, 100.)
        self.assertTrue(result.launch_only)
        self.assertFalse(result.event_qualified)
        self.assertFalse(result.time.flags.writeable)
        self.assertFalse(np.any(np.diff(result.time) <= 0.))

    def test_nonconstant_acceleration_against_analytic_solution(self):
        a0 = 3.0
        result = integrate_standing_launch(None, FakeTrajectory(end=1000.),
            settings=LaunchSettings(transition_speed=2.),
            acceleration_limit=lambda vehicle, v, k: a0 / (1. + v))
        self.assertAlmostEqual(result.transition_time, (2 + 2.**2 / 2) / a0, places=7)
        self.assertAlmostEqual(result.transition_distance, (2.**2 / 2 + 2.**3 / 3) / a0, places=7)

    def test_nonunit_station_metric_maps_physical_distance(self):
        seen = []
        trajectory = FakeTrajectory(end=40., curvature_fn=lambda s: .02 + s * .001, ratio=2.)
        def acceleration(vehicle, v, k):
            seen.append(k)
            return 2.
        result = integrate_standing_launch(None, trajectory,
            settings=LaunchSettings(transition_speed=2.), acceleration_limit=acceleration)
        self.assertAlmostEqual(result.transition_distance, 1.)
        self.assertAlmostEqual(result.transition_reference_station, .5)
        self.assertTrue(seen)
        self.assertAlmostEqual(seen[-1], .0205)

    def test_validation_and_finish_first(self):
        with self.assertRaises(ValueError): LaunchSettings(transition_speed=True)
        with self.assertRaises(ValueError): integrate_standing_launch(None, FakeTrajectory(),
            settings=LaunchSettings(transition_speed=.5))
        with self.assertRaises(RuntimeError): integrate_standing_launch(None, FakeTrajectory(end=.1),
            settings=LaunchSettings(transition_speed=2.), acceleration_limit=lambda *_: 1.)
        with self.assertRaises(RuntimeError): integrate_standing_launch(None, FakeTrajectory(),
            settings=LaunchSettings(transition_speed=2., maximum_time=.1), acceleration_limit=lambda *_: 1.)
        with self.assertRaises(RuntimeError): integrate_standing_launch(None, FakeTrajectory(),
            settings=LaunchSettings(transition_speed=2.), acceleration_limit=lambda *_: 0.)


if __name__ == "__main__":
    unittest.main()
