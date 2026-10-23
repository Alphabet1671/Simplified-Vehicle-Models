"""Exact rational checks for reference metric/curvature interval bounds."""
from __future__ import annotations

from dataclasses import FrozenInstanceError
from fractions import Fraction
import math
import unittest

import numpy as np

from simplified_vehicle_models.solvers.reference_geometry_bounds import (
    _sqrt_interval, bound_reference_geometry,
)
from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve
from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile


def track(station, north, east, closed=False):
    s, x, y = map(lambda a: np.asarray(a, dtype=float), (station, north, east))
    z = np.zeros_like(s)
    return TrackProfile(s, np.diff(s), np.ones_like(s), z, closed, "test",
                        PathCoordinates(s, x, y, z, z, closed))


def exact_spline_derivative(spline, q, order):
    """Independent exact Fraction derivative recurrence and Cox basis oracle."""
    t = [Fraction.from_float(float(v)) for v in spline.t]
    c = [Fraction.from_float(float(v)) for v in spline.c[:len(t)-spline.k-1]]
    p = spline.k
    for _ in range(order):
        c = [Fraction(p)*(c[i+1]-c[i])/(t[i+p+1]-t[i+1]) for i in range(len(c)-1)]
        t, p = t[1:-1], p-1

    def basis(i, degree):
        if degree == 0:
            if q == t[-degree-1]:
                return Fraction(int(t[i] < q <= t[i+1]))
            return Fraction(int(t[i] <= q < t[i+1]))
        left = Fraction(0) if t[i+degree] == t[i] else (q-t[i])/(t[i+degree]-t[i]) * basis(i, degree-1)
        right = Fraction(0) if t[i+degree+1] == t[i+1] else (t[i+degree+1]-q)/(t[i+degree+1]-t[i+1]) * basis(i+1, degree-1)
        return left+right
    return sum(c[i]*basis(i, p) for i in range(len(c)))


def assert_signed_sqrt_enclosure(test, interval, numerator, denominator_squared):
    """Prove interval encloses numerator/sqrt(denominator_squared) by squares."""
    lo, hi = interval
    if numerator >= 0:
        test.assertGreaterEqual(hi, 0)
        if lo > 0:
            test.assertLessEqual(lo*lo*denominator_squared, numerator*numerator)
        if hi > 0:
            test.assertGreaterEqual(hi*hi*denominator_squared, numerator*numerator)
    else:
        magnitude = -numerator
        test.assertLessEqual(lo, 0)
        if lo < 0:
            test.assertGreaterEqual(lo*lo*denominator_squared, magnitude*magnitude)
        if hi < 0:
            test.assertLessEqual(hi*hi*denominator_squared, magnitude*magnitude)


class ReferenceGeometryBoundsTests(unittest.TestCase):
    def test_integer_sqrt_bounds_exact_rational_inputs(self):
        cases = [Fraction(0), Fraction(1), Fraction(4), Fraction(2),
                 Fraction(1, 10**300), Fraction(10**300),
                 Fraction.from_float(float.fromhex('0x0.0000000000001p-1022'))]
        for q in cases:
            lo, hi = _sqrt_interval((q, q), 80)
            self.assertLessEqual(lo*lo, q)
            self.assertGreaterEqual(hi*hi, q)
            if lo == hi:
                self.assertEqual(lo*lo, q)
        self.assertEqual(_sqrt_interval((Fraction(9), Fraction(9)), 16), (Fraction(3), Fraction(3)))

    def test_point_curvature_bounds_against_independent_exact_oracle(self):
        s = np.array([0., 1., 2., 3.])
        ref = ReferenceCurve(track(s, s*s, 2*s+1))
        for station in (0., .5, 1.5, 3.):
            out = bound_reference_geometry(ref, (station, station), sqrt_bits=96)
            q = Fraction.from_float(station)
            dx, ddx, dddx = (exact_spline_derivative(ref._x, q, n) for n in (1, 2, 3))
            dy, ddy, dddy = (exact_spline_derivative(ref._y, q, n) for n in (1, 2, 3))
            metric_sq = dx*dx+dy*dy
            self.assertLessEqual(out.metric_interval[0]**2, metric_sq)
            self.assertGreaterEqual(out.metric_interval[1]**2, metric_sq)
            numerator = dx*ddy-dy*ddx
            assert_signed_sqrt_enclosure(self, out.curvature_interval, numerator, metric_sq**3)
            dot = dx*ddx+dy*ddy
            ks_numerator = (dx*dddy-dy*dddx)*metric_sq-3*numerator*dot
            assert_signed_sqrt_enclosure(self, out.curvature_s_interval, ks_numerator, metric_sq**5)
            self.assertEqual(out.panel_count, 1)
            self.assertFalse(out.whole_trajectory_certified)

    def test_wide_circle_or_parabola_query_contains_exact_point_probes(self):
        s = np.array([0., .5, 1.2, 2., 2.8, 3.5, 4.])
        ref = ReferenceCurve(track(s, s, s*s))
        out = bound_reference_geometry(ref, (.1, 3.9), maximum_panels=1000)
        self.assertGreaterEqual(out.panel_count, 1)
        for station in np.linspace(.1, 3.9, 29):
            q = Fraction.from_float(float(station))
            dx = exact_spline_derivative(ref._x, q, 1)
            ddx = exact_spline_derivative(ref._x, q, 2)
            dddx = exact_spline_derivative(ref._x, q, 3)
            dy = exact_spline_derivative(ref._y, q, 1)
            ddy = exact_spline_derivative(ref._y, q, 2)
            dddy = exact_spline_derivative(ref._y, q, 3)
            m = dx*dx+dy*dy
            self.assertLessEqual(out.metric_interval[0]**2, m)
            self.assertGreaterEqual(out.metric_interval[1]**2, m)
            n = dx*ddy-dy*ddx
            assert_signed_sqrt_enclosure(self, out.curvature_interval, n, m**3)
            p = (dx*dddy-dy*dddx)*m-3*n*(dx*ddx+dy*ddy)
            assert_signed_sqrt_enclosure(self, out.curvature_s_interval, p, m**5)
        with self.assertRaises((FrozenInstanceError, AttributeError, TypeError)):
            out.panel_count = 0

    def test_budget_precision_types_and_source_immutability(self):
        s = np.array([0., 1., 2., 3., 4., 5.])
        ref = ReferenceCurve(track(s, s+.2*s*s, s**3/7))
        before = tuple((v.t.copy(), v.c.copy()) for v in (ref._x, ref._y))
        for kw in ({"maximum_panels": True}, {"maximum_depth": -1}, {"sqrt_bits": 15},
                   {"sqrt_bits": 257}, {"maximum_panel_curvature_width": 0},
                   {"maximum_curvature_width": 0}):
            with self.assertRaises((TypeError, ValueError)):
                bound_reference_geometry(ref, (.25, 4.75), **kw)
        with self.assertRaises(RuntimeError):
            bound_reference_geometry(ref, (0., 5.), maximum_panels=1)
        with self.assertRaises((TypeError, ValueError)):
            bound_reference_geometry(ref, (Fraction(1, 3), Fraction(2, 3)))
        out = bound_reference_geometry(ref, (.25, 4.75))
        self.assertTrue(out.exact_rational_enclosure)
        for spline, (knots, coeffs) in zip((ref._x, ref._y), before):
            np.testing.assert_array_equal(spline.t, knots)
            np.testing.assert_array_equal(spline.c, coeffs)

    def test_periodic_full_domain_and_open_query_endpoints(self):
        s = np.linspace(0., 2*np.pi, 9)
        ref = ReferenceCurve(track(s, np.cos(s), np.sin(s), True), interpolation_periodic=True)
        out = bound_reference_geometry(ref, (0., float(2*np.pi)), maximum_panels=100)
        self.assertGreater(out.panel_count, 1)
        self.assertEqual(out.station_interval, (0., float(2*np.pi)))
        with self.assertRaises(ValueError):
            bound_reference_geometry(ref, (0., float(np.nextafter(2*np.pi, np.inf))))

    def test_width_is_per_panel_and_global_union_keeps_real_variation(self):
        s = np.array([0., .5, 1.2, 2., 2.8, 3.5, 4.])
        ref = ReferenceCurve(track(s, s, s*s))
        target = Fraction(1, 5)
        out = bound_reference_geometry(ref, (.1, 3.9), maximum_panels=1000,
                                       maximum_panel_curvature_width=target)
        self.assertEqual(out.maximum_panel_curvature_width, target)
        self.assertLessEqual(out.max_accepted_panel_curvature_width, target)
        # The global range contains actual curvature variation, not just
        # per-panel dependency width; it is therefore allowed to exceed target.
        self.assertGreater(out.curvature_interval[1]-out.curvature_interval[0], target)

    def test_requested_tight_width_fails_closed_when_budget_is_insufficient(self):
        s = np.array([0., 1., 2., 3., 4., 5.])
        ref = ReferenceCurve(track(s, s+.2*s*s, s**3/7))
        with self.assertRaises(RuntimeError):
            bound_reference_geometry(ref, (.25, 4.75), maximum_panels=8,
                                     maximum_depth=1, maximum_panel_curvature_width=Fraction(1, 10**12))


if __name__ == "__main__":
    unittest.main()
