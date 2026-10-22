from dataclasses import dataclass
import math
import unittest

import numpy as np

from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface
from simplified_vehicle_models.formulations.signed_capability import SignedTransientCapability
from simplified_vehicle_models.solvers.capability_profile import capability_profile, matched_profile_replay
from simplified_vehicle_models.tracks.types import PathCoordinates


@dataclass
class Path:
    distance: object
    curvature: object
    is_closed: bool = False
    reference_distance: object = None


def surface(*, acceleration=2., braking=-4., ceiling=100.):
    return CapabilitySurface(np.array([.5, 100.]), np.array([0., 1.]),
        np.full((2, 2), acceleration), np.full((2, 2), braking),
        np.full(2, ceiling), "synthetic")


def rest(acceleration=2., braking=-4.):
    return lambda v, k: (braking, acceleration)


class CapabilityProfileTest(unittest.TestCase):
    def test_rolling_constant_acceleration_matches_analytic_time(self):
        path = Path(np.array([0., 10.]), np.zeros(2))
        actual = capability_profile(path, surface(acceleration=2.), boundary="rolling", start_speed=3., step=1.)
        expected = (math.sqrt(3.**2 + 2*2*10)-3)/2
        self.assertTrue(actual.converged); self.assertEqual(actual.status, "converged")
        self.assertAlmostEqual(actual.lap_time, expected, places=6)
        self.assertEqual(actual.speed[0], 3.)

    def test_standing_constant_acceleration_matches_analytic_time(self):
        path = Path(np.array([0., 10.]), np.zeros(2))
        actual = capability_profile(path, surface(acceleration=2.), boundary="standing", step=.25, rest_limits=rest(2.))
        self.assertTrue(actual.converged)
        self.assertAlmostEqual(actual.lap_time, math.sqrt(2*10/2), places=5)
        self.assertEqual(actual.speed[0], 0.)

    def test_braking_preview_reaches_future_corner_cap(self):
        path = Path(np.array([0., 15., 16., 30.]), np.array([0., 0., 1., 1.]))
        table = surface(acceleration=4., braking=-2.)
        table = CapabilitySurface(table.speed, table.curvature, table.acceleration_max,
                                  table.acceleration_min, np.array([100., 5.]), "synthetic")
        actual = capability_profile(path, table, boundary="rolling", start_speed=5., step=1.)
        self.assertTrue(actual.converged)
        self.assertLessEqual(actual.speed[np.searchsorted(actual.distance, 16.)], 5.)
        self.assertEqual(actual.speed[0], 5.) # initial remains exactly pinned
        # Braking begins before the curvature-node cap, rather than local clipping.
        self.assertTrue(np.any(np.diff(actual.speed[:16]**2) < 0.))

    def test_closed_flying_is_periodic_and_open_flying_rejected(self):
        closed = Path(np.array([0., 5., 10.]), np.zeros(3), True)
        result = capability_profile(closed, surface(), boundary="flying", step=1.)
        self.assertTrue(result.converged); self.assertEqual(result.speed[0], result.speed[-1])
        with self.assertRaisesRegex(ValueError, "closed"):
            capability_profile(Path(np.array([0., 10.]), np.zeros(2)), surface(), boundary="flying")

    def test_exact_start_and_impossible_high_start(self):
        path = Path(np.array([0., 4.]), np.zeros(2))
        result = capability_profile(path, surface(), boundary="rolling", start_speed=1.25, step=1.)
        self.assertEqual(result.speed[0], 1.25)
        with self.assertRaisesRegex(ValueError, "exceeds"):
            capability_profile(path, surface(ceiling=1.), boundary="rolling", start_speed=2.)

    def test_signed_floor_requires_rest_limits_and_queries_never_go_ood(self):
        chart = SignedTransientCapability(np.array([0., 1.]), np.array([0., 1.]),
            np.full((2, 2), 2.), np.full((2, 2), -3.), np.full(2, 20.), .5, "synthetic")
        path = Path(np.array([0., 3.]), np.zeros(2))
        with self.assertRaisesRegex(ValueError, "rest_limits"):
            capability_profile(path, chart, boundary="standing")
        seen = []
        def low(v, k):
            seen.append((v, k)); return -3., 2.
        result = capability_profile(path, chart, boundary="standing", step=.25, rest_limits=low)
        self.assertTrue(result.converged)
        self.assertTrue(all(0 <= v <= .5 for v, _ in seen))

    def test_transition_failure_is_explicit_status(self):
        path = Path(np.array([0., 1.]), np.zeros(2))
        result = capability_profile(path, surface(acceleration=-1.), boundary="rolling", start_speed=1.)
        self.assertFalse(result.converged)
        self.assertIn(result.status, ("transition-domain-failed", "dynamically-infeasible"))

    def test_nonunit_reference_station_is_preserved_for_replay(self):
        path = Path(np.array([0., 5.]), np.zeros(2), False, np.array([10., 30.]))
        result = capability_profile(path, surface(), boundary="rolling", start_speed=1., step=1.)
        np.testing.assert_allclose(result.reference_distance, np.linspace(10., 30., result.distance.size))
        replay = matched_profile_replay(object(), result)
        np.testing.assert_allclose(replay.reference_distance, result.reference_distance)

    def test_trajectory_uses_reference_mapping_not_geometry_refit(self):
        path = PathCoordinates(np.array([0., 2.]), np.array([0., 2.]), np.zeros(2),
                               np.zeros(2), np.zeros(2), False, np.array([10., 20.]))
        class Trajectory:
            def evaluate(self, stations):
                return type("Values", (), {"curvature": np.asarray(stations)/20.})()
        # curvature at path end is 1; strict legacy domain supports exactly [0,1].
        result = capability_profile(path, surface(), boundary="rolling", start_speed=1., step=1., trajectory=Trajectory())
        self.assertTrue(result.converged)


if __name__ == "__main__":
    unittest.main()
