import math
import unittest

import numpy as np

from pathlib import Path

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import (
    evaluate_equilibrium, maximum_acceleration_limit, maximum_deceleration_limit,
    point_mass_view_of_planar,
)
from simplified_vehicle_models.solvers.capability_profile import _bracketed_energy_transition, capability_profile
from simplified_vehicle_models.solvers.live_standing_profile import LiveStandingSettings, solve_live_standing_profile
from simplified_vehicle_models.tracks.types import PathCoordinates
from tests.test_bracketed_energy_profile import OpenTrajectory, cap, variable_limits


class EnergySegmentLimitTests(unittest.TestCase):
    def test_default_midpoint_is_bitwise_legacy_and_new_root_uses_actual_endpoint(self):
        seen = []
        def limits(v, k):
            seen.append(v)
            return -5., 4. - .01*v*v
        midpoint = _bracketed_energy_transition(limits, vi=0., bound=20., curvature=0.,
            ds=10., forward=True, tolerance=1.e-10)
        named = _bracketed_energy_transition(limits, vi=0., bound=20., curvature=0.,
            ds=10., forward=True, tolerance=1.e-10, segment_limit_sampling="midpoint")
        self.assertEqual(midpoint, named)
        seen.clear()
        sampled = _bracketed_energy_transition(limits, vi=0., bound=20., curvature=0.,
            ds=10., forward=True, tolerance=1.e-10,
            segment_limit_sampling="speed_endpoints_and_midpoint")
        expected_w = (0. + 8.*10.)/(1.+.02*10.)
        self.assertAlmostEqual(sampled*sampled, expected_w, delta=2.e-7)
        self.assertIn(0., seen)
        self.assertIn(sampled, seen)
        self.assertTrue(any(math.isclose(v, sampled/math.sqrt(2.), rel_tol=0., abs_tol=1.e-7) for v in seen))

    def test_sampled_profile_dense_fixture_and_wrapper_report_forwarding(self):
        profile, report = solve_live_standing_profile(object(), OpenTrajectory(),
            settings=LiveStandingSettings(step=10., transition_method="bracketed_energy",
                segment_limit_sampling="speed_endpoints_and_midpoint"),
            limit_evaluator=variable_limits, speed_cap_evaluator=cap)
        self.assertIsNotNone(profile, report)
        self.assertEqual(report["segment_limit_sampling"], "speed_endpoints_and_midpoint")
        self.assertFalse(report["speed_sampling_certificate"])
        self.assertFalse(report["physical_qualified"])
        for left, right, ds in zip(profile.speed[:-1], profile.speed[1:], np.diff(profile.distance)):
            acceleration = (right*right-left*left)/(2.*ds)
            for fraction in np.linspace(0., 1., 5):
                speed = math.sqrt((1.-fraction)*left*left+fraction*right*right)
                self.assertLessEqual(acceleration, variable_limits(None, speed, 0.)[1]+1.e-7)

    def test_sampling_is_opt_in_and_guarded(self):
        with self.assertRaises(ValueError):
            LiveStandingSettings(segment_limit_sampling="wat")
        with self.assertRaisesRegex(ValueError, "requires"):
            LiveStandingSettings(segment_limit_sampling="speed_endpoints_and_midpoint")
        with self.assertRaisesRegex(ValueError, "segment_limit_sampling"):
            _bracketed_energy_transition(lambda v,k: (-5., 2.), vi=0., bound=2., curvature=0.,
                ds=1., forward=True, tolerance=1.e-7, segment_limit_sampling="bad")

    def test_negative_k_backward_braking_root_uses_most_restrictive_minimum(self):
        seen = []
        def limits(v, k):
            seen.append((v, k))
            return -2.-.01*v*v, 4.
        previous = _bracketed_energy_transition(limits, vi=3., bound=10., curvature=-.2,
            ds=2., forward=False, tolerance=1.e-9,
            segment_limit_sampling="speed_endpoints_and_midpoint")
        expected_w = 3.**2 + 2.*2.*(2.+.01*3.**2)
        self.assertAlmostEqual(previous**2, expected_w, delta=1.e-6)
        self.assertLess(previous, 10.)
        speeds = [v for v, k in seen]
        self.assertIn(3., speeds)
        self.assertIn(previous, speeds)
        self.assertIn(-.2, [k for _, k in seen])
        acceleration = (3.**2-previous**2)/(2.*2.)
        for fraction in np.linspace(0., 1., 5):
            speed = math.sqrt((1.-fraction)*previous**2+fraction*3.**2)
            self.assertGreaterEqual(acceleration, limits(speed, -.2)[0]-1.e-8)

    def test_compiled_planar_pointmass_view_sampled_segments_are_originally_feasible(self):
        root = Path(__file__).resolve().parents[1]
        model = compile_formulation(str(root / "models/formulations/iem24_murphy_real_planar_7dof.xml")).model
        vehicle = point_mass_view_of_planar(model)
        class StraightPath:
            distance = np.array([0., 75.3])
            curvature = np.zeros(2)
            is_closed = False
        profile = capability_profile(StraightPath(), type("LiveSurface", (), {
            "speed_floor": 0., "domain_speed": lambda self,k: 20.,
            "limits": lambda self,v,k: (
                -maximum_deceleration_limit(vehicle, v, k),
                maximum_acceleration_limit(vehicle, v, k),
            )})(), boundary="standing", step=2., rest_limits=lambda v,k: (
                -maximum_deceleration_limit(vehicle, v, k),
                maximum_acceleration_limit(vehicle, v, k)),
            transition_method="bracketed_energy", segment_limit_sampling="speed_endpoints_and_midpoint")
        self.assertTrue(profile.converged, profile.diagnostics)
        for left, right, ds in zip(profile.speed[:-1], profile.speed[1:], np.diff(profile.distance)):
            acceleration = (right*right-left*left)/(2.*ds)
            for fraction in np.linspace(0., 1., 5):
                speed = math.sqrt((1.-fraction)*left*left+fraction*right*right)
                self.assertTrue(evaluate_equilibrium(vehicle, speed, 0., acceleration).is_feasible,
                    (speed, acceleration, evaluate_equilibrium(vehicle, speed, 0., acceleration)))

    def test_constant_limits_exactly_match_midpoint_mode(self):
        kwargs = dict(vi=0., bound=20., curvature=0., ds=5., forward=True, tolerance=1.e-8)
        midpoint = _bracketed_energy_transition(lambda v,k: (-4., 2.), **kwargs)
        sampled = _bracketed_energy_transition(lambda v,k: (-4., 2.),
            segment_limit_sampling="speed_endpoints_and_midpoint", **kwargs)
        self.assertEqual(midpoint, sampled)


if __name__ == "__main__":
    unittest.main()
