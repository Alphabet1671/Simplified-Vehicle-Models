import math
import unittest

import numpy as np

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

    def test_negative_k_backward_braking_endpoint_samples(self):
        seen = []
        def limits(v, k):
            seen.append((v, k))
            return -2.-.01*v*v, 4.
        value = _bracketed_energy_transition(limits, vi=10., bound=5., curvature=-.2,
            ds=1., forward=False, tolerance=1.e-8,
            segment_limit_sampling="speed_endpoints_and_midpoint")
        self.assertGreaterEqual(value, 5.)
        speeds = [v for v, k in seen]
        self.assertIn(10., speeds)
        self.assertIn(5., speeds)
        self.assertIn(-.2, [k for _, k in seen])

    def test_constant_limits_exactly_match_midpoint_mode(self):
        kwargs = dict(vi=0., bound=20., curvature=0., ds=5., forward=True, tolerance=1.e-8)
        midpoint = _bracketed_energy_transition(lambda v,k: (-4., 2.), **kwargs)
        sampled = _bracketed_energy_transition(lambda v,k: (-4., 2.),
            segment_limit_sampling="speed_endpoints_and_midpoint", **kwargs)
        self.assertEqual(midpoint, sampled)


if __name__ == "__main__":
    unittest.main()
