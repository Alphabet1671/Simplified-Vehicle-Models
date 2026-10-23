"""Exact-rational checks for reference spline derivative intervals."""
from __future__ import annotations

from fractions import Fraction
import unittest

import numpy as np

from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve
from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile
from simplified_vehicle_models.solvers.reference_spline_bounds import bound_reference_derivatives


def track(station, north, east, closed=False):
    s, x, y = map(lambda a: np.asarray(a, dtype=float), (station, north, east))
    z = np.zeros_like(s)
    return TrackProfile(s, np.diff(s), np.ones_like(s), z, closed, "test",
                        PathCoordinates(s, x, y, z, z, closed))


def basis(i, p, u, knots, left_endpoint=False):
    if p == 0:
        hit = knots[i] < u <= knots[i + 1] if left_endpoint else knots[i] <= u < knots[i + 1]
        return Fraction(int(hit))
    a = Fraction(0) if knots[i+p] == knots[i] else (u-Fraction.from_float(float(knots[i])))/(Fraction.from_float(float(knots[i+p]))-Fraction.from_float(float(knots[i])))
    b = Fraction(0) if knots[i+p+1] == knots[i+1] else (Fraction.from_float(float(knots[i+p+1]))-u)/(Fraction.from_float(float(knots[i+p+1]))-Fraction.from_float(float(knots[i+1])))
    return a*basis(i,p-1,u,knots,left_endpoint)+b*basis(i+1,p-1,u,knots,left_endpoint)


def oracle(spline, u, order):
    knots = [Fraction.from_float(float(v)) for v in spline.t]
    c = [Fraction.from_float(float(v)) for v in spline.c[:len(knots)-spline.k-1]]
    p = spline.k
    for _ in range(order):
        c = [p*(c[i+1]-c[i])/(knots[i+p+1]-knots[i+1]) for i in range(len(c)-1)]
        knots = knots[1:-1]
        p -= 1
    left_endpoint = u == knots[-p-1]
    return sum(c[i]*basis(i,p,u,knots,left_endpoint) for i in range(len(c)))


class ReferenceSplineBoundsTests(unittest.TestCase):
    def setUp(self):
        s = np.array([0., 1., 2., 3., 4., 5.])
        self.reference = ReferenceCurve(track(s, s + .2*s**2, s**3/7))

    def test_point_intervals_match_independent_exact_basis_oracle(self):
        for spline, north in ((self.reference._x, True), (self.reference._y, False)):
            for q in [Fraction(0), Fraction(1, 2), Fraction(3, 2), Fraction(7, 2)]:
                out = bound_reference_derivatives(self.reference, (float(q), float(q)))
                bounds = out.north_derivatives if north else out.east_derivatives
                for order, interval in enumerate(bounds):
                    exact = oracle(spline, q, order)
                    self.assertEqual(interval, (exact, exact))

    def test_wide_interval_encloses_exact_rational_probes(self):
        out = bound_reference_derivatives(self.reference, (.25, 4.75))
        for q in [Fraction(i, 16) for i in range(4, 77)]:
            for spl, bounds in ((self.reference._x, out.north_derivatives), (self.reference._y, out.east_derivatives)):
                for order, (lo, hi) in enumerate(bounds):
                    exact = oracle(spl, q, order)
                    self.assertLessEqual(lo, exact)
                    self.assertGreaterEqual(hi, exact)
        self.assertTrue(out.exact_arithmetic)
        self.assertFalse(out.whole_trajectory_certified)
        self.assertEqual(out.span_count, 1)  # six-point not-a-knot fit is one polynomial span

    def test_degree_three_short_fit_and_exact_domain_end(self):
        s = np.array([0., 1., 2., 3.])
        ref = ReferenceCurve(track(s, s*s, 2*s + 1))
        for q in (Fraction(0), Fraction(3)):
            out = bound_reference_derivatives(ref, (float(q), float(q)))
            for spl, bounds in ((ref._x, out.north_derivatives), (ref._y, out.east_derivatives)):
                for order, pair in enumerate(bounds):
                    value = oracle(spl, q, order)
                    self.assertEqual(pair, (value, value))

    def test_span_budget_and_fail_closed_inputs(self):
        out = bound_reference_derivatives(self.reference, (.25, 4.75))
        with self.assertRaises(ValueError):
            bound_reference_derivatives(self.reference, (.25, 4.75), maximum_spans=out.span_count-1)
        for interval in ((-1., 1.), (1., 1., 2.), (True, 2.), (float('nan'), 2.),
                         (Fraction(5)+Fraction(1, 10**400), Fraction(5)+Fraction(1, 10**400)),
                         (Fraction(1, 3), Fraction(2, 3)),
                         (Fraction(1, 2), Fraction(1, 2)+Fraction(1, 10**400)),
                         (Fraction(3, 2), Fraction(1, 2))):
            with self.assertRaises((TypeError, ValueError)):
                bound_reference_derivatives(self.reference, interval)
        exact_fraction = bound_reference_derivatives(self.reference, (Fraction(1, 2), Fraction(1, 2)))
        self.assertEqual(exact_fraction.station_interval, (.5, .5))
        with self.assertRaises(ValueError):
            bound_reference_derivatives(self.reference, (0., 5.), maximum_spans=0)
        original = self.reference._x
        from scipy.interpolate import BSpline
        self.reference._x = BSpline(original.t.astype(np.float64), original.c.astype(np.float64), original.k)
        self.reference._x.t = original.t.astype(np.float32)
        with self.assertRaises(ValueError):
            bound_reference_derivatives(self.reference, (1., 2.))
        self.reference._x = original

    def test_nonuniform_multispan_budget_and_open_periodic_fit(self):
        s = np.array([0., .4, 1.1, 1.8, 2.9, 4.2, 5.1, 6.7, 8.])
        ref = ReferenceCurve(track(s, s + np.sin(s), np.cos(.7*s)))
        a, b = Fraction(0), Fraction(8)
        interior = sorted({Fraction.from_float(float(v)) for v in ref._x.t if a < Fraction.from_float(float(v)) < b})
        expected = len(interior) + 1
        out = bound_reference_derivatives(ref, (0., 8.), maximum_spans=expected)
        self.assertEqual(out.span_count, expected)
        with self.assertRaisesRegex(ValueError, "span budget"):
            bound_reference_derivatives(ref, (0., 8.), maximum_spans=expected-1)
        open_periodic = ReferenceCurve(track(s, np.cos(2*np.pi*s/8), np.sin(2*np.pi*s/8)), interpolation_periodic=True)
        end = Fraction.from_float(open_periodic.end)
        out = bound_reference_derivatives(open_periodic, (0., open_periodic.end))
        for q in (Fraction(0), end):
            point = bound_reference_derivatives(open_periodic, (float(q), float(q)))
            for spl, bounds in ((open_periodic._x, point.north_derivatives), (open_periodic._y, point.east_derivatives)):
                for order, pair in enumerate(bounds):
                    value = oracle(spl, q, order)
                    self.assertEqual(pair, (value, value))
        with self.assertRaises(ValueError):
            bound_reference_derivatives(open_periodic, (0., np.nextafter(open_periodic.end, np.inf)))

    def test_periodic_reference_seam_endpoint_and_no_source_mutation(self):
        s = np.linspace(0., 2*np.pi, 9)
        ref = ReferenceCurve(track(s, np.cos(s), np.sin(s), True), interpolation_periodic=True)
        before = tuple((spline.t.copy(), spline.c.copy()) for spline in (ref._x, ref._y))
        for q in (Fraction(0), Fraction.from_float(float(s[-1]))):
            out = bound_reference_derivatives(ref, (float(q), float(q)))
            for spl, bounds in ((ref._x, out.north_derivatives), (ref._y, out.east_derivatives)):
                for order, pair in enumerate(bounds):
                    value = oracle(spl, q, order)
                    self.assertEqual(pair, (value, value))
        for spline, (knots, coeffs) in zip((ref._x, ref._y), before):
            np.testing.assert_array_equal(spline.t, knots)
            np.testing.assert_array_equal(spline.c, coeffs)
        with self.assertRaises((AttributeError, TypeError)):
            out.span_count = 99
        class SubReference(ReferenceCurve):
            pass
        with self.assertRaises(TypeError):
            bound_reference_derivatives(SubReference.__new__(SubReference), (0., 1.))


if __name__ == "__main__":
    unittest.main()
