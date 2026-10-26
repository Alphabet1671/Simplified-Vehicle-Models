import math
import unittest

import numpy as np

from simplified_vehicle_models.solvers.capability_profile import _bracketed_energy_transition, capability_profile
from simplified_vehicle_models.solvers.live_standing_profile import LiveStandingSettings, solve_live_standing_profile
from simplified_vehicle_models.tracks.types import PathCoordinates


class OpenTrajectory:
    class Ref:
        is_closed = False
        source_station = np.array([0., 75.3])
    reference = Ref()
    class Map:
        path_start = 0.
        path_end = 75.3
        def to_reference(self, x): return np.asarray(x, dtype=float)
    station_map = Map()
    def to_path(self, source):
        s = np.asarray(source, dtype=float)
        return PathCoordinates(s, np.zeros_like(s), np.zeros_like(s), np.zeros_like(s), np.zeros_like(s), False, s)
    def evaluate(self, source):
        return type("Geometry", (), {"curvature": np.zeros_like(source, dtype=float)})()


def variable_limits(vehicle, speed, curvature):
    return -5., 4. - .01*speed*speed
def cap(vehicle, curvature): return 20.
def variable_actual_limits(vehicle, speed, curvature):
    from simplified_vehicle_models.formulations.point_mass import maximum_acceleration_limit, maximum_deceleration_limit
    return -maximum_deceleration_limit(vehicle, speed, curvature), maximum_acceleration_limit(vehicle, speed, curvature)


class BracketedEnergyTests(unittest.TestCase):
    def test_quadratic_acceleration_exact_discrete_energy_recurrence(self):
        ds, length = 10., 75.3
        profile, report = solve_live_standing_profile(object(), OpenTrajectory(),
            settings=LiveStandingSettings(step=ds, transition_method="bracketed_energy"),
            limit_evaluator=variable_limits, speed_cap_evaluator=cap)
        self.assertIsNotNone(profile, report)
        w = 0.
        for segment in np.diff(profile.distance):
            # W_next-W = 2 ds (4 - .01 * (W+W_next)/2)
            w = (w*(1-.01*segment)+8*segment)/(1+.01*segment)
        self.assertAlmostEqual(profile.speed[-1]**2, w, delta=1.e-4)
        self.assertEqual(report["transition_method"], "bracketed_energy")
        self.assertFalse(report["accepted"])
        self.assertFalse(report["physical_qualified"])
        self.assertEqual(report["root_assumption"], "sampled_non_decreasing_energy_residual_not_continuous_certificate")
        legacy, _ = solve_live_standing_profile(object(), OpenTrajectory(),
            settings=LiveStandingSettings(step=10., tolerance=1.e-7),
            limit_evaluator=variable_limits, speed_cap_evaluator=cap)
        self.assertLess(profile.lap_time, legacy.lap_time)
        self.assertGreater(profile.speed[-1], legacy.speed[-1])
        for left, right, ds_i in zip(profile.speed[:-1], profile.speed[1:], np.diff(profile.distance)):
            amid = math.sqrt(.5*(left*left+right*right))
            self.assertLessEqual((right*right-left*left)/(2*ds_i), variable_limits(None, amid, 0.)[1]+1.e-7)

    def test_default_and_explicit_legacy_are_bitwise_equal(self):
        kwargs = dict(limit_evaluator=variable_limits, speed_cap_evaluator=cap)
        legacy, _ = solve_live_standing_profile(object(), OpenTrajectory(), settings=LiveStandingSettings(step=10.), **kwargs)
        explicit, _ = solve_live_standing_profile(object(), OpenTrajectory(), settings=LiveStandingSettings(step=10., transition_method="legacy_fixed_point"), **kwargs)
        self.assertIsNotNone(legacy); self.assertIsNotNone(explicit)
        np.testing.assert_array_equal(legacy.speed, explicit.speed)
        np.testing.assert_array_equal(legacy.elapsed_time, explicit.elapsed_time)

    def test_constant_acceleration_and_backward_braking_midpoint_limits(self):
        from simplified_vehicle_models.solvers.capability_profile import capability_profile
        class Path:
            distance = np.array([0., 5., 10.])
            curvature = np.zeros(3)
            is_closed = False
        const = capability_profile(Path(), type("Surface", (), {
            "speed_floor": 0., "domain_speed": lambda self,k: 100.,
            "limits": lambda self,v,k: (-4.,2.)})(), boundary="standing", step=1.,
            rest_limits=lambda v,k: (-4.,2.), transition_method="bracketed_energy")
        self.assertTrue(const.converged, const.diagnostics)
        self.assertAlmostEqual(const.speed[-1], math.sqrt(40.), places=6)
        class Drop(Path):
            distance = np.array([0., 1., 2.])
            curvature = np.array([0., 0., 1.])
        table = type("Surface", (), {"speed_floor": 0.,
            "domain_speed": lambda self,k: 100. if k == 0. else 3.,
            "limits": lambda self,v,k: (-2.,4.)})()
        brake = capability_profile(Drop(), table, boundary="rolling", start_speed=2., step=1., transition_method="bracketed_energy")
        self.assertTrue(brake.converged, brake.diagnostics)
        self.assertLess(brake.speed[-2], 4.)
        for a,b,k in zip(brake.speed[:-1], brake.speed[1:], (.0,.5)):
            self.assertGreaterEqual((b*b-a*a)/2., -2.-1.e-7)

    def test_sampled_nonmonotonic_and_discontinuous_residuals_fail(self):
        def nonmono(v, k): return (-5., -4.+4.*v*v)
        with self.assertRaisesRegex(RuntimeError, "nonmonotone"):
            _bracketed_energy_transition(nonmono, vi=0., bound=4., curvature=0., ds=1., forward=True, tolerance=1.e-7)
        def jump(v, k): return (-5., 2. if v < .7071067811865476 else 0.)
        with self.assertRaises(RuntimeError):
            _bracketed_energy_transition(jump, vi=0., bound=4., curvature=0., ds=1., forward=True, tolerance=1.e-7)

    def test_invalid_limit_and_iteration_cap_fail(self):
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            _bracketed_energy_transition(lambda v,k: (float("nan"), 2.), vi=0., bound=2., curvature=0., ds=1., forward=True, tolerance=1.e-7)
        with self.assertRaisesRegex(ValueError, "non-boolean"):
            _bracketed_energy_transition(lambda v,k: (False, 2.), vi=0., bound=2., curvature=0., ds=1., forward=True, tolerance=1.e-7)
        with self.assertRaisesRegex(ValueError, "overflow"):
            _bracketed_energy_transition(lambda v,k: (-5., 2.), vi=1.e308, bound=1.e308, curvature=0., ds=1., forward=True, tolerance=1.e-7)
        with self.assertRaisesRegex(RuntimeError, "did not meet"):
            _bracketed_energy_transition(lambda v,k: (-5., 4.-.01*v*v), vi=0., bound=20., curvature=0., ds=10., forward=True, tolerance=1.e-12, maximum_iterations=1)

    def test_actual_iem24_coarse_transition_is_faster_but_unqualified(self):
        from pathlib import Path
        from simplified_vehicle_models.compiler.compile import compile_formulation
        root = Path(__file__).resolve().parents[1]
        model = compile_formulation(str(root / "models/formulations/iem24_murphy_real_point_mass.xml")).model
        legacy, _ = solve_live_standing_profile(model, OpenTrajectory(), settings=LiveStandingSettings(step=10.))
        result, report = solve_live_standing_profile(model, OpenTrajectory(),
            settings=LiveStandingSettings(step=10., transition_method="bracketed_energy"))
        self.assertIsNotNone(legacy); self.assertIsNotNone(result, report)
        self.assertLess(result.lap_time, legacy.lap_time)
        self.assertGreater(result.speed[-1], legacy.speed[-1])
        self.assertFalse(report["accepted"])
        self.assertFalse(report["physical_qualified"])
        for left, right, ds_i in zip(result.speed[:-1], result.speed[1:], np.diff(result.distance)):
            vm = math.sqrt(.5*(left*left+right*right))
            actual = (right*right-left*left)/(2*ds_i)
            lo, hi = variable_actual_limits(model, vm, 0.)
            self.assertGreaterEqual(actual, lo-1.e-7)
            self.assertLessEqual(actual, hi+1.e-7)

    def test_negative_curvature_is_passed_through_transition_queries(self):
        seen = []
        def limits(v, k):
            seen.append(k)
            return -5., 2.
        class Path:
            distance = np.array([0., 1.])
            curvature = np.array([-.2, -.2])
            is_closed = False
        surface = type("Surface", (), {"speed_floor": 0., "domain_speed": lambda self,k: 20.,
            "limits": lambda self,v,k: limits(v,k)})()
        profile = capability_profile(Path(), surface, boundary="rolling", start_speed=1.,
            step=1., transition_method="bracketed_energy")
        self.assertTrue(profile.converged, profile.diagnostics)
        self.assertIn(-.2, seen)

    def test_signed_chart_rejects_new_method(self):
        from simplified_vehicle_models.formulations.signed_capability import SignedTransientCapability
        chart = SignedTransientCapability(np.array([0., 1.]), np.array([0., 1.]), np.full((2,2),2.), np.full((2,2),-3.), np.full(2,20.), .5, "test")
        path = PathCoordinates(np.array([0., 1.]), np.zeros(2), np.zeros(2), np.zeros(2), np.zeros(2), False)
        with self.assertRaisesRegex(ValueError, "signed charts"):
            capability_profile(path, chart, boundary="rolling", start_speed=1., transition_method="bracketed_energy")


if __name__ == "__main__":
    unittest.main()
