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
        for bad in (True, np.bool_(False), np.nan, np.inf, -np.inf, "1", 1 + 0j):
            with self.subTest(value=bad), self.assertRaises(ValueError):
                LaunchSettings(transition_speed=bad)
        for bad in (True, np.nan, np.inf, "0.5", 0.5 + 0j):
            with self.subTest(floor=bad), self.assertRaises(ValueError):
                integrate_standing_launch(None, FakeTrajectory(), capability_speed_floor=bad,
                    settings=LaunchSettings(transition_speed=2.), acceleration_limit=lambda *_: 1.)
        with self.assertRaises(ValueError): integrate_standing_launch(None, FakeTrajectory(),
            settings=LaunchSettings(transition_speed=.5))
        with self.assertRaises(RuntimeError): integrate_standing_launch(None, FakeTrajectory(end=.1),
            settings=LaunchSettings(transition_speed=2.), acceleration_limit=lambda *_: 1.)
        with self.assertRaises(RuntimeError): integrate_standing_launch(None, FakeTrajectory(),
            settings=LaunchSettings(transition_speed=2., maximum_time=.1), acceleration_limit=lambda *_: 1.)
        for bad in (0., -1., np.nan, np.inf, -np.inf):
            with self.subTest(acceleration=bad), self.assertRaises(RuntimeError):
                integrate_standing_launch(None, FakeTrajectory(), settings=LaunchSettings(transition_speed=2.),
                    acceleration_limit=lambda *_, value=bad: value)
        for bad in (np.nan, np.inf, -np.inf):
            with self.subTest(curvature=bad), self.assertRaises(RuntimeError):
                integrate_standing_launch(None, FakeTrajectory(curvature_fn=lambda _s, value=bad: value),
                    settings=LaunchSettings(transition_speed=2.), acceleration_limit=lambda *_: 1.)

    def test_default_original_acceleration_backend_queries_v0_through_station_map(self):
        from unittest.mock import patch
        trajectory = FakeTrajectory(end=40., curvature_fn=lambda s: 0.03 + s * .001, ratio=2.)
        calls = []
        def backend(vehicle, speed, curvature):
            calls.append((vehicle, speed, curvature))
            return 2.
        vehicle = object()
        with patch("simplified_vehicle_models.formulations.point_mass.maximum_acceleration_limit", backend):
            result = integrate_standing_launch(vehicle, trajectory, settings=LaunchSettings(transition_speed=2.))
        self.assertIs(calls[0][0], vehicle)
        self.assertEqual(calls[0][1], 0.)
        self.assertAlmostEqual(calls[0][2], .03)
        self.assertAlmostEqual(result.transition_reference_station, .5)

    def test_launch_result_rejects_malformed_arrays(self):
        from simplified_vehicle_models.solvers.standing_launch import LaunchResult
        kwargs = dict(time=np.array([0., np.nan]), path_distance=np.array([0., 1.]),
            reference_station=np.array([0., 1.]), speed=np.array([0., 1.]),
            acceleration=np.array([1., 1.]), transition_time=1., transition_distance=1.,
            transition_reference_station=1., transition_speed=1., original_path_length=2.,
            remaining_path_length=1.)
        with self.assertRaises(ValueError):
            LaunchResult(**kwargs)


if __name__ == "__main__":
    unittest.main()
