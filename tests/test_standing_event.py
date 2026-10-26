"""Analytic tests for frozen-path standing-event composition."""
import unittest
from types import SimpleNamespace

import numpy as np

from simplified_vehicle_models.solvers.standing_launch import LaunchSettings
from simplified_vehicle_models.solvers.standing_event import FrozenPathRemainder, solve_frozen_standing_profile
from simplified_vehicle_models.tracks.line_trajectory import StationMap
from simplified_vehicle_models.solvers.capability_profile import CapabilityProfile


class Trajectory:
    def __init__(self, closed=False, ratio=1.):
        self.station_map = StationMap([10., 60.], [5., 105.], closed)
        self.coefficients = object()
    def evaluate(self, s):
        a = np.asarray(s, dtype=float)
        return SimpleNamespace(north=a, east=2*a, heading=a*.01, curvature=.001+a*0.)


class Surface:
    speed_floor = .5


class StandingEventTests(unittest.TestCase):
    def test_constant_acceleration_exact_remainder_and_mapping(self):
        trajectory = Trajectory()
        calls = []
        def profile(path, surface, **kwargs):
            calls.append((path, kwargs))
            self.assertEqual(kwargs["boundary"], "rolling")
            self.assertAlmostEqual(path.distance[-1], 99.)
            self.assertAlmostEqual(path.reference_distance[0], 10.5)
            self.assertAlmostEqual(path.reference_distance[-1], 60.)
            self.assertAlmostEqual(path.reference_distance[0], 10.5)
            self.assertAlmostEqual(kwargs["trajectory"].station_map.path_end, 99.)
            self.assertIs(kwargs["trajectory"].trajectory, trajectory)
            return CapabilityProfile(np.array([0., 99.]), np.array([2., 20.]), np.array([0., 9.]),
                                     9., True, "converged", 1, 0., np.array([path.reference_distance[0], path.reference_distance[-1]]),
                                     {"test": True})
        result, diag = solve_frozen_standing_profile(None, trajectory, Surface(),
            launch_settings=LaunchSettings(transition_speed=2.), acceleration_limit=lambda *_: 2.,
            profile_solver=profile)
        self.assertIsNotNone(result, diag)
        self.assertEqual(len(calls), 1)
        self.assertAlmostEqual(result.total_time, 10.)
        self.assertAlmostEqual(result.full_path_length, 100.)
        self.assertAlmostEqual(result.path_distance[0], 0.)
        self.assertTrue(np.all(np.diff(result.path_distance) > 0.))
        self.assertAlmostEqual(result.path_distance[-1], 100.)
        expected_ref = np.asarray(trajectory.station_map.to_reference(5. + result.path_distance))
        np.testing.assert_allclose(result.reference_station, expected_ref, rtol=0., atol=1e-8)
        self.assertAlmostEqual(result.speed[-1], 20.)
        self.assertAlmostEqual(result.total_time, np.sqrt(2*100/2), places=8)
        self.assertFalse(result.accepted)
        self.assertFalse(result.event_qualified)
        self.assertFalse(result.joint_line_optimized)
        for a in (result.path_distance, result.speed, result.reference_station):
            self.assertFalse(a.flags.writeable)

    def test_nonunit_metric_remainder_domain_and_closed_no_wrap(self):
        tr = Trajectory(closed=True)
        # The local remainder is explicitly open despite a periodic original map.
        launch = __import__("simplified_vehicle_models.solvers.standing_launch", fromlist=["integrate_standing_launch"]).integrate_standing_launch(
            None, tr, settings=LaunchSettings(transition_speed=2.),
            acceleration_limit=lambda *_: 2.)
        rem = FrozenPathRemainder(tr, launch)
        self.assertFalse(rem.station_map.is_closed)
        self.assertAlmostEqual(rem.station_map.to_reference(0.), launch.transition_reference_station)
        self.assertAlmostEqual(rem.station_map.to_path(launch.transition_reference_station), 0.)
        with self.assertRaises(ValueError): rem.station_map.to_reference(rem.path_length + 1e-4)
        path = rem.to_path(step=7.)
        self.assertAlmostEqual(path.distance[-1], 99.)
        self.assertFalse(path.is_closed)
        with self.assertRaises(ValueError):
            rem.to_path(step=0.)

    def test_total_time_independent_of_transition_speed(self):
        for transition in (1., 2., 3.):
            def analytic(path, surface, **kwargs):
                end_speed = np.sqrt(transition**2 + 4.*path.distance[-1])
                remainder_time = (end_speed-transition)/2.
                refs = np.array([path.reference_distance[0], path.reference_distance[-1]])
                return CapabilityProfile(np.array([0., path.distance[-1]]),
                    np.array([transition, end_speed]), np.array([0., remainder_time]),
                    remainder_time, True, "converged", 1, 0., refs)
            result, diag = solve_frozen_standing_profile(None, Trajectory(), Surface(),
                launch_settings=LaunchSettings(transition_speed=transition),
                acceleration_limit=lambda *_: 2., profile_solver=analytic)
            self.assertIsNotNone(result, diag)
            self.assertAlmostEqual(result.total_time, np.sqrt(100.), places=7)

    def test_bad_profile_rejected(self):
        for converged, speeds, end in ((False, [2., 3.], 99.), (True, [2., np.nan], 99.),
                                       (True, [3., 4.], 99.), (True, [2., 4.], 98.)):
            def fake(path, surface, **kwargs):
                return CapabilityProfile(np.array([0., end]), np.array(speeds), np.array([0., 1.]),
                    1., converged, "bad", 1, 0., np.array([path.reference_distance[0], path.reference_distance[-1]]))
            result, diag = solve_frozen_standing_profile(None, Trajectory(), Surface(),
                launch_settings=LaunchSettings(transition_speed=2.), acceleration_limit=lambda *_: 2.,
                profile_solver=fake)
            self.assertIsNone(result)
            self.assertEqual(diag["phase"], "remainder")
            self.assertFalse(diag["event_qualified"])

    def test_profile_elapsed_time_and_energy_consistency_required(self):
        cases = (([0., 2.], 2., [2., 20.]),
                 ([0., np.nan], 9., [2., 20.]),
                 ([0., -1.], 9., [2., 20.]),
                 ([0., 8.], 9., [2., 20.]),
                 ([0., 2., 9.], 9., [2., 8., 20.]),
                 ([0., 9.], 9., [2., 18.]))
        for elapsed, lap_time, speeds in cases:
            def fake(path, surface, **kwargs):
                return CapabilityProfile(np.array([0., 99.]), np.array(speeds), np.array(elapsed),
                    lap_time, True, "converged", 1, 0.,
                    np.array([path.reference_distance[0], path.reference_distance[-1]]))
            result, diag = solve_frozen_standing_profile(None, Trajectory(), Surface(),
                launch_settings=LaunchSettings(transition_speed=2.), acceleration_limit=lambda *_: 2.,
                profile_solver=fake)
            self.assertIsNone(result)
            self.assertEqual(diag["phase"], "remainder")

    def test_wrong_interior_reference_profile_rejected(self):
        def fake(path, surface, **kwargs):
            refs = np.array([path.reference_distance[0], 50., path.reference_distance[-1]])
            return CapabilityProfile(np.array([0., 20., path.distance[-1]]), np.array([2., 8., 15.]),
                                     np.array([0., 1., 2.]), 2., True, "converged", 1, 0., refs)
        result, diag = solve_frozen_standing_profile(None, Trajectory(), Surface(),
            launch_settings=LaunchSettings(transition_speed=2.), acceleration_limit=lambda *_: 2.,
            profile_solver=fake)
        self.assertIsNone(result)
        self.assertEqual(diag["phase"], "remainder")

    def test_inconsistent_launch_transition_data_rejected(self):
        from dataclasses import replace
        from simplified_vehicle_models.solvers.standing_launch import integrate_standing_launch
        trajectory = Trajectory()
        good = integrate_standing_launch(None, trajectory,
            settings=LaunchSettings(transition_speed=2.), acceleration_limit=lambda *_: 2.)
        mutations = (
            {"path_distance": np.r_[good.path_distance[:-1], good.path_distance[-1] + 1.]},
            {"speed": np.r_[good.speed[:-1], good.speed[-1] + 1.]},
            {"reference_station": np.r_[good.reference_station[:-1], good.reference_station[-1] + 1.]},
            {"transition_speed": True},
            {"transition_time": np.nan},
        )
        for changes in mutations:
            with self.subTest(changes=changes):
                with self.assertRaises(ValueError):
                    FrozenPathRemainder(trajectory, replace(good, **changes))

    def test_launch_failure_returns_no_result(self):
        result, diag = solve_frozen_standing_profile(None, Trajectory(), Surface(),
            launch_settings=LaunchSettings(transition_speed=2., maximum_time=.01),
            acceleration_limit=lambda *_: 2., profile_solver=lambda *a, **k: self.fail("called"))
        self.assertIsNone(result)
        self.assertEqual(diag["phase"], "launch")


if __name__ == "__main__":
    unittest.main()
