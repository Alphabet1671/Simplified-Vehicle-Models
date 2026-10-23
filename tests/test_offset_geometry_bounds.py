"""Focused proof-oriented tests for actual offset geometry enclosures."""
from __future__ import annotations

from fractions import Fraction
import math
import unittest

import numpy as np

from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile
from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve, OffsetTrajectory
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.solvers.offset_geometry_bounds import bound_offset_geometry
from simplified_vehicle_models.solvers.reference_spline_bounds import bound_reference_derivatives


def track(s, x, y, closed=False):
    s, x, y = (np.asarray(v, dtype=float) for v in (s, x, y))
    z = np.zeros_like(s)
    return TrackProfile(s, np.diff(s), np.ones_like(s), z, closed, "test",
                        PathCoordinates(s, x, y, z, z, closed))


def power(poly, station, order):
    """Independent exact power-sum evaluation from stored PP coefficients."""
    index = min(max(int(np.searchsorted(poly.x, station, side="right")-1), 0), len(poly.x)-2)
    q = Fraction.from_float(float(station)) - Fraction.from_float(float(poly.x[index]))
    c = poly.c[:, index]
    degree = len(c)-1
    return sum(Fraction.from_float(float(c[i])) * Fraction(math.factorial(degree-i), math.factorial(degree-i-order))
               * q**(degree-i-order) for i in range(len(c)) if degree-i >= order)


def _basis(i, p, u, knots):
    if p == 0:
        return Fraction(int(knots[i] <= u < knots[i+1]))
    a = Fraction(0) if knots[i+p] == knots[i] else (u-knots[i])/(knots[i+p]-knots[i])
    b = Fraction(0) if knots[i+p+1] == knots[i+1] else (knots[i+p+1]-u)/(knots[i+p+1]-knots[i+1])
    return a*_basis(i,p-1,u,knots)+b*_basis(i+1,p-1,u,knots)


def source_oracle(spline, station, order):
    knots=[Fraction.from_float(float(v)) for v in spline.t]
    coeff=[Fraction.from_float(float(v)) for v in spline.c[:len(knots)-spline.k-1]]
    degree=spline.k
    for _ in range(order):
        coeff=[degree*(coeff[i+1]-coeff[i])/(knots[i+degree+1]-knots[i+1]) for i in range(len(coeff)-1)]
        knots=knots[1:-1]; degree-=1
    u=Fraction.from_float(float(station))
    return sum(coeff[i]*_basis(i,degree,u,knots) for i in range(len(coeff)))


def oracle_at(traj, s):
    p = traj._offset_curve
    n, ns, nss = (power(p, s, order) for order in (0,1,2))
    xs=[source_oracle(traj.reference._x,s,o) for o in range(4)]
    ys=[source_oracle(traj.reference._y,s,o) for o in range(4)]
    dx,ddx,dddx=xs[1:]; dy,ddy,dddy=ys[1:]
    assert dy == 0 and ddy == 0 and dddy == 0 and dx > 0
    h=dx; hs=ddx; k=Fraction(0); ks=Fraction(0)
    fold=1-k*n
    A,B=h*fold,ns
    As=hs*fold-h*(ks*n+k*ns)
    metric2=A*A+B*B
    curvature_num=(h*k)*metric2+A*nss-B*As
    return n,fold,metric2,curvature_num


class OffsetGeometryBoundsTests(unittest.TestCase):
    def setUp(self):
        s = np.array([0., 1., 2., 3., 4., 5.])
        profile = track(s, s, np.zeros_like(s))
        ref = ReferenceCurve(profile)
        spline = OffsetSpline(profile, 1.)
        coeff = .01 * spline.control_distance**2
        self.trajectory = OffsetTrajectory(ref, spline, coeff)

    def test_nonzero_offset_point_and_wide_exact_power_oracle(self):
        t = self.trajectory
        out = bound_offset_geometry(t, (.5, 4.5), maximum_panels=4096)
        self.assertTrue(out.exact_rational_enclosure)
        self.assertFalse(out.physical_feasibility_certified)
        for sf in (.5, 1.25, 2.5, 3.75, 4.5):
            n, fold, metric2, knum = oracle_at(t, sf)
            self.assertLessEqual(out.offset_interval[0], n); self.assertGreaterEqual(out.offset_interval[1], n)
            self.assertLessEqual(out.fold_interval[0], fold); self.assertGreaterEqual(out.fold_interval[1], fold)
            mlo,mhi=out.metric_interval
            self.assertGreaterEqual(mlo, 0)
            self.assertLessEqual(mlo*mlo, metric2); self.assertGreaterEqual(mhi*mhi, metric2)
            klo,khi=out.curvature_interval
            self.assertGreaterEqual(klo, 0)
            self.assertLessEqual(klo*klo*metric2**3, knum*knum)
            self.assertGreaterEqual(khi*khi*metric2**3, knum*knum)
        self.assertNotEqual(out.curvature_interval, (Fraction(0), Fraction(0)))
        refined=bound_offset_geometry(t,(.5,4.5),maximum_panel_curvature_width=Fraction(1,2))
        self.assertLessEqual(refined.max_accepted_panel_curvature_width,Fraction(1,2))
        self.assertGreater(refined.panel_count,out.panel_count)
        # The global range remains a union of accepted panels, not a claimed physical pass.
        # Public floating evaluator is a falsification cross-check only, not proof.
        dense=np.linspace(.5,4.5,101)
        actual=t.evaluate(dense)
        for value, interval in zip(actual.curvature, [out.curvature_interval]*len(dense)):
            self.assertGreaterEqual(value, float(interval[0])-1e-12)
            self.assertLessEqual(value, float(interval[1])+1e-12)

    def test_zero_offset_and_budget_and_typed_failures(self):
        t = self.trajectory
        z = OffsetTrajectory(t.reference, t.spline, np.zeros(t.spline.control_count))
        out = bound_offset_geometry(z, (2.,2.))
        self.assertEqual(out.offset_interval, (Fraction(0),Fraction(0)))
        self.assertEqual(out.curvature_interval, (Fraction(0),Fraction(0)))
        with self.assertRaises(ValueError):
            bound_offset_geometry(t,(0.,5.),maximum_panels=1)
        with self.assertRaises((TypeError,ValueError)):
            bound_offset_geometry(t,(Fraction(1,3),1.))
        with self.assertRaises(ValueError):
            bound_offset_geometry(t,(-1.,1.))
        with self.assertRaises(ValueError):
            bound_offset_geometry(t,(0.,5.),sqrt_bits=15)
        original=t._offset_curve.c
        t._offset_curve.c=original.astype(np.float32)
        try:
            with self.assertRaises(ValueError):
                bound_offset_geometry(t,(0.,1.))
        finally:
            t._offset_curve.c=original

    def test_sources_and_stored_offset_coefficients_are_unchanged(self):
        t=self.trajectory
        arrays=(t.reference._x.c.copy(),t.reference._x.t.copy(),t._offset_curve.c.copy(),t._offset_curve.x.copy())
        result=bound_offset_geometry(t,(0.,5.))
        self.assertEqual(result.station_interval,(0.,5.))
        for actual, expected in zip((t.reference._x.c,t.reference._x.t,t._offset_curve.c,t._offset_curve.x),arrays):
            np.testing.assert_array_equal(actual,expected)


if __name__ == "__main__":
    unittest.main()
