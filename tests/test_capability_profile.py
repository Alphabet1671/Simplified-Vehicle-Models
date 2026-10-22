from dataclasses import dataclass
import math
from pathlib import Path as FilePath
import unittest

import numpy as np

from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface
from simplified_vehicle_models.formulations.signed_capability import SignedTransientCapability, load_signed_transient_capability
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

    def test_closed_flying_requires_duplicate_physical_xy(self):
        closed = PathCoordinates(np.array([0., 5., 10.]), np.array([0., 1., 0.]),
                                 np.array([0., 0., 0.]), np.zeros(3), np.zeros(3), True)
        result = capability_profile(closed, surface(), boundary="flying", step=1.)
        self.assertTrue(result.converged); self.assertEqual(result.speed[0], result.speed[-1])
        with self.assertRaisesRegex(ValueError, "closed"):
            capability_profile(Path(np.array([0., 10.]), np.zeros(2)), surface(), boundary="flying")
        malformed = PathCoordinates(np.array([0., 5., 10.]), np.array([0., 1., 2.]),
                                   np.zeros(3), np.zeros(3), np.zeros(3), True)
        with self.assertRaisesRegex(ValueError, "closed"):
            capability_profile(malformed, surface(), boundary="flying")

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

    def test_trajectory_uses_exact_station_map_not_path_table_interpolation(self):
        path = PathCoordinates(np.array([0., 2.]), np.array([0., 2.]), np.zeros(2),
                               np.zeros(2), np.zeros(2), False, np.array([10., 20.]))
        class Map:
            calls = []
            def to_reference(self, ell):
                self.calls.append(np.array(ell)); return 10. + 2.5*np.asarray(ell)**2
        class Trajectory:
            station_map = Map()
            def evaluate(self, stations):
                return type("Values", (), {"curvature": np.zeros_like(stations)})()
        result = capability_profile(path, surface(), boundary="rolling", start_speed=1., step=1., trajectory=Trajectory())
        self.assertTrue(result.converged)
        np.testing.assert_allclose(result.reference_distance, 10. + 2.5*result.distance**2)
        self.assertEqual(len(Trajectory.station_map.calls), 1)
        with self.assertRaisesRegex(ValueError, "station_map"):
            capability_profile(path, surface(), boundary="rolling", start_speed=1., step=1.,
                               trajectory=type("NoMap", (), {"evaluate": lambda self, s: None})())

    def test_signed_energy_midpoint_limits_bound_achieved_segments(self):
        # A speed-dependent signed upper bound distinguishes the energy midpoint
        # sqrt((vi^2+vj^2)/2) from the arithmetic speed midpoint.
        chart = SignedTransientCapability(np.array([0., 1.]), np.array([0., 1.]),
            np.array([[4., 4.], [0.5, .5]]), np.full((2, 2), -4.),
            np.full(2, 10.), .5, "synthetic")
        result = capability_profile(Path(np.array([0., 3.]), np.zeros(2)), chart,
                                    boundary="rolling", start_speed=1., step=.25)
        self.assertTrue(result.converged)
        for left, right in zip(result.speed[:-1], result.speed[1:]):
            vm = math.sqrt(.5*(left*left + right*right))
            acceleration = (right*right-left*left)/(2*.25)
            lower, upper = chart.limits(vm, 0.)
            self.assertGreaterEqual(acceleration, lower-1.e-7)
            self.assertLessEqual(acceleration, upper+1.e-7)

    def test_signed_affine_transition_escapes_high_speed_underdrive(self):
        # At high speed amax is negative, but the physically connected low-speed
        # branch has strong propulsion.  A min-only fixed point locks into the
        # high-speed branch; exact affine-cell roots must select the largest
        # reachable endpoint instead.
        chart = SignedTransientCapability(np.array([0., .5, 1.]), np.array([0., 1.]),
            np.array([[4., 4.], [2., 2.], [-2., -2.]]),
            np.full((3, 2), -5.), np.full(2, 20.), .5, "synthetic")
        result = capability_profile(Path(np.array([0., 1.]), np.zeros(2)), chart,
                                    boundary="rolling", start_speed=1., step=1.)
        self.assertTrue(result.converged)
        self.assertGreater(result.speed[-1], 2.5)
        a = (result.speed[-1]**2-1.)/2.
        _, amax = chart.limits(math.sqrt(.5*(1.+result.speed[-1]**2)), 0.)
        self.assertAlmostEqual(a, amax, places=6)

    def test_signed_affine_backward_braking_root_previews_corner(self):
        chart = SignedTransientCapability(np.array([0., 1.]), np.array([0., 1.]),
            np.full((2, 2), 4.), np.array([[-1., -1.], [-5., -5.]]),
            np.array([20., 3.]), .5, "synthetic")
        result = capability_profile(Path(np.array([0., 1., 2.]), np.array([0., 0., 1.])), chart,
                                    boundary="rolling", start_speed=2., step=1.)
        self.assertTrue(result.converged)
        self.assertGreater(result.speed[1], result.speed[2])
        a = (result.speed[2]**2-result.speed[1]**2)/2.
        amin, _ = chart.limits(math.sqrt(.5*(result.speed[1]**2+result.speed[2]**2)), .5)
        self.assertGreaterEqual(a, amin-1.e-7)
        self.assertLess(a, 0.)

    def test_signed_nonmonotone_capability_selects_connected_maximum(self):
        # The upper limit dips below zero then returns positive.  The requested
        # endpoint lies beyond that dip, but only the connected low-speed branch
        # is reachable and is selected without fixed-point seed dependence.
        chart = SignedTransientCapability(np.array([0., .5, 1.]), np.array([0., 1.]),
            np.array([[2., 2.], [-4., -4.], [2., 2.]]),
            np.full((3, 2), -5.), np.full(2, 20.), .5, "synthetic")
        result = capability_profile(Path(np.array([0., 3.]), np.zeros(2)), chart,
                                    boundary="rolling", start_speed=1., step=3.)
        self.assertTrue(result.converged)
        self.assertLess(result.speed[-1], 4.)
        a = (result.speed[-1]**2-1.)/6.
        _, amax = chart.limits(math.sqrt(.5*(1.+result.speed[-1]**2)), 0.)
        self.assertAlmostEqual(a, amax, places=6)

    def test_signed_negative_acceleration_branch_continues_from_current_state(self):
        chart = SignedTransientCapability(np.array([0., 1.]), np.array([0., 1.]),
            np.full((2, 2), -1.), np.full((2, 2), -3.), np.full(2, 5.), .5, "synthetic")
        result = capability_profile(Path(np.array([0., 1.]), np.zeros(2)), chart,
                                    boundary="rolling", start_speed=2., step=1.)
        self.assertTrue(result.converged)
        self.assertAlmostEqual(result.speed[-1], math.sqrt(2.), places=7)

    def test_signed_positive_minimum_branch_continues_from_current_state(self):
        chart = SignedTransientCapability(np.array([0., 1.]), np.array([0., 1.]),
            np.full((2, 2), 3.), np.full((2, 2), 1.), np.full(2, 5.), .5, "synthetic")
        result = capability_profile(Path(np.array([0., 1.]), np.zeros(2)), chart,
                                    boundary="rolling", start_speed=1., step=1.)
        self.assertTrue(result.converged)
        self.assertGreater(result.speed[-1], 1.)

    def test_signed_transition_refuses_disconnected_current_state_prefix(self):
        # The high-speed branch is feasible but cannot be reached continuously
        # from v=1: its amax at the current-state side first demands braking.
        chart = SignedTransientCapability(np.array([0., 1.]), np.array([0., 1.]),
            np.array([[-2., -2.], [4., 4.]]), np.full((2, 2), -5.),
            np.full(2, 3.), .5, "synthetic")
        result = capability_profile(Path(np.array([0., 10.]), np.zeros(2)), chart,
                                    boundary="rolling", start_speed=1., step=10.)
        self.assertFalse(result.converged)
        self.assertEqual(result.status, "transition-domain-failed")
        self.assertIn("connected current-state", result.diagnostics["detail"])

    def test_actual_signed_acceleration_profile_matches_ocp_near_discretization(self):
        root = FilePath(__file__).resolve().parents[1]
        solution_path = root / "runs/no_tv_reliability/sixcase_signed_coarse_20261022/acceleration/mesh-5_basis-35_centerline/solution.npz"
        chart_path = root / "runs/no_tv_reliability/signed_lp_fullrange_product_20261022.npz"
        if not solution_path.is_file() or not chart_path.is_file():
            self.skipTest("optional signed acceleration artifacts are not present in this checkout")
        solution = np.load(solution_path)
        chart = load_signed_transient_capability(chart_path)
        path = PathCoordinates(solution["distance"], solution["curvature"], np.zeros_like(solution["distance"]),
                               np.zeros_like(solution["distance"]), np.zeros_like(solution["distance"]), False)
        result = capability_profile(path, chart, boundary="rolling", start_speed=10., step=.25)
        self.assertTrue(result.converged)
        self.assertLess(abs(result.lap_time-4.508858), 2.e-4)
        self.assertGreater(result.speed[1], 10.)

    def test_rest_join_checks_interior_interpolated_curvature(self):
        chart = SignedTransientCapability(np.array([0., 1.]), np.array([0., 1.]),
            np.full((2, 2), 2.), np.full((2, 2), -3.), np.full(2, 20.), .5, "synthetic")
        path = Path(np.array([0., 1.]), np.array([0., 1.]))
        def discontinuous(v, k):
            return (-3., 3.) if .4 < k < .6 else (-3., 2.)
        # step forces a midpoint curvature query at k=.5.
        with self.assertRaisesRegex(ValueError, "continuously match"):
            capability_profile(path, chart, boundary="standing", step=.5, rest_limits=discontinuous)


if __name__ == "__main__":
    unittest.main()
