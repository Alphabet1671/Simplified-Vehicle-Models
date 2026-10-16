import unittest

import numpy as np

from simplified_vehicle_models.suspension.errors import DomainError
from simplified_vehicle_models.suspension.monotone_spline import MonotoneSpline


class MonotoneSplineTests(unittest.TestCase):
    def test_reproduces_knots_exactly(self):
        x = np.array([0.0, 0.1, 0.3, 0.7, 1.5])
        y = np.array([0.0, 40.0, 90.0, 120.0, 130.0])
        spline = MonotoneSpline(x, y)
        np.testing.assert_allclose(spline(x), y, rtol=0.0, atol=1e-12)

    def test_monotone_between_knots_on_random_data(self):
        rng = np.random.default_rng(20261016)
        for _ in range(1000):
            n = int(rng.integers(2, 14))
            x = np.cumsum(rng.uniform(0.01, 1.0, n))
            increments = rng.uniform(0.0, 5.0, n)
            # Mix of flat segments and sharp changes.
            increments[rng.random(n) < 0.15] = 0.0
            y = np.cumsum(increments)
            if rng.random() < 0.5:
                y = -y
            spline = MonotoneSpline(x, y)
            q = np.linspace(x[0], x[-1], 400)
            d = np.diff(spline(q))
            scale = max(1.0, float(np.max(np.abs(y))))
            if y[-1] >= y[0]:
                self.assertGreaterEqual(float(d.min()), -1e-12 * scale)
            else:
                self.assertLessEqual(float(d.max()), 1e-12 * scale)

    def test_derivative_continuous_at_knots(self):
        rng = np.random.default_rng(7)
        x = np.cumsum(rng.uniform(0.05, 0.5, 9))
        y = np.cumsum(rng.uniform(0.0, 3.0, 9))
        spline = MonotoneSpline(x, y)
        interior = x[1:-1]
        left = spline.derivative(interior, side="left")
        right = spline.derivative(interior, side="right")
        np.testing.assert_allclose(left, right, rtol=0.0, atol=1e-9)

    def test_derivative_matches_finite_difference(self):
        x = np.array([0.0, 0.2, 0.5, 1.0, 2.0])
        y = np.array([0.0, 10.0, 25.0, 35.0, 38.0])
        spline = MonotoneSpline(x, y)
        q = np.array([0.1, 0.35, 0.8, 1.5])
        eps = 1e-6
        fd = (spline(q + eps) - spline(q - eps)) / (2 * eps)
        np.testing.assert_allclose(spline.derivative(q), fd, rtol=1e-6)

    def test_out_of_domain_raises(self):
        spline = MonotoneSpline([0.0, 1.0, 2.0], [0.0, 1.0, 3.0])
        for q in (-1e-9, 2.0 + 1e-9, np.array([0.5, 3.0])):
            with self.assertRaises(DomainError):
                spline(q)
            with self.assertRaises(DomainError):
                spline.derivative(q)
        spline(np.array([0.0, 2.0]))  # endpoints are inside the domain

    def test_nan_query_raises(self):
        spline = MonotoneSpline([0.0, 1.0], [0.0, 1.0])
        with self.assertRaises(DomainError):
            spline(float("nan"))

    def test_invalid_knots_rejected(self):
        with self.assertRaises(ValueError):
            MonotoneSpline([0.0, 0.0, 1.0], [0.0, 1.0, 2.0])
        with self.assertRaises(ValueError):
            MonotoneSpline([0.0], [0.0])
        with self.assertRaises(ValueError):
            MonotoneSpline([0.0, 1.0], [0.0, np.nan])

    def test_two_knots_is_linear(self):
        spline = MonotoneSpline([1.0, 3.0], [2.0, 6.0])
        np.testing.assert_allclose(spline([1.0, 2.0, 3.0]), [2.0, 4.0, 6.0])
        np.testing.assert_allclose(spline.derivative([1.0, 2.0, 3.0]), 2.0)

    def test_domain_error_is_specific_value_error(self):
        self.assertTrue(issubclass(DomainError, ValueError))


if __name__ == "__main__":
    unittest.main()
