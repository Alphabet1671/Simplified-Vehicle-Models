import unittest

import numpy as np

from simplified_vehicle_models.solvers.steering_capability import steering_transition_speed_cap


class SteeringCapabilityTests(unittest.TestCase):
    def call_cap(self, x, k, *, rate=1.0, tau=0.08, max_angle=0.7,
                 wheelbase=2.5, original=None, closed=False):
        return steering_transition_speed_cap(x, k, wheelbase, max_angle, rate, tau,
                                             original_cap=original, is_closed=closed)

    def test_constant_curvature_is_unchanged(self):
        x = [0, 1, 3]
        result = self.call_cap(x, [0.1] * 3, original=[20, 18, 15])
        np.testing.assert_allclose(result, [20, 18, 15])

    def test_nonuniform_signed_segments_and_physical_bounds(self):
        x = [0, 1, 4]
        k = [0, 0.2, -0.1]
        L, tau, rate, limit = 2.0, 0.1, 0.5, 0.8
        result = self.call_cap(x, k, wheelbase=L, tau=tau, rate=rate,
                               max_angle=limit)
        delta = np.arctan(L * np.asarray(k))
        sec = np.abs(np.diff(delta) / np.diff(x))
        slopes = np.array([sec[0], max(sec), sec[1]])
        expected = np.minimum(rate / slopes,
                              (limit - np.abs(delta)) / (tau * slopes))
        np.testing.assert_allclose(result, expected)

    def test_rate_only_when_tau_zero(self):
        result = self.call_cap([0, 2], [0, 0.2], tau=0, rate=0.25)
        expected = 0.25 / abs(np.arctan(2.5 * 0.2) / 2)
        np.testing.assert_allclose(result, [expected, expected])

    def test_closed_seam_and_duplicate_endpoint(self):
        x = [0, 1, 3, 4]
        k = [0.0, 0.2, -0.1, 0.0]
        result = self.call_cap(x, k, tau=0, rate=0.5, closed=True)
        angles = np.arctan(2.5 * np.asarray(k))
        slope = np.abs(np.diff(angles) / np.diff(x))
        self.assertAlmostEqual(result[0], 0.5 / max(slope[0], slope[-1]))
        self.assertEqual(result[0], result[-1])
        self.assertFalse(result.flags.writeable)
        with self.assertRaisesRegex(ValueError, "original_cap"):
            self.call_cap(x, k, original=[10., 10., 10., 11.], closed=True)

    def test_infeasible_geometry_is_explicit(self):
        with self.assertRaisesRegex(ValueError, "nominal bicycle infeasible geometry.*full-vehicle feasibility unresolved"):
            self.call_cap([0, 1], [0, 1], max_angle=0.5)

    def test_invalid_inputs(self):
        for kwargs in ({"rate": 0}, {"tau": -1}, {"max_angle": 0}, {"wheelbase": float("nan")}):
            with self.subTest(kwargs=kwargs), self.assertRaises((TypeError, ValueError)):
                self.call_cap([0, 1], [0, 0.1], **kwargs)
        with self.assertRaises(ValueError):
            self.call_cap([0, 1], [0, float("inf")])
        with self.assertRaises(ValueError):
            self.call_cap([0, 0], [0, 0.1])
        with self.assertRaises(ValueError):
            self.call_cap([0, 1], [0, 0.1], original=[1])


if __name__ == "__main__":
    unittest.main()
