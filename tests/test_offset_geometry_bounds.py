"""Focused proof-oriented tests for actual offset geometry enclosures."""
from __future__ import annotations

from fractions import Fraction
import math
from decimal import Decimal
import unittest

import numpy as np

from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile
from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve, OffsetTrajectory
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.solvers.offset_geometry_bounds import (
    HeadingStatus, LimitStatus, _atan_point, bound_offset_geometry,
)
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
        self.assertGreaterEqual(refined.panel_count, out.panel_count)
        for sf in (.5, 1.25, 2.5, 3.75, 4.5):
            n, fold, metric2, knum = oracle_at(t, sf)
            self.assertLessEqual(refined.offset_interval[0], n)
            self.assertGreaterEqual(refined.offset_interval[1], n)
            self.assertLessEqual(refined.fold_interval[0], fold)
            self.assertGreaterEqual(refined.fold_interval[1], fold)
            self.assertLessEqual(refined.metric_interval[0]**2, metric2)
            self.assertGreaterEqual(refined.metric_interval[1]**2, metric2)
            self.assertLessEqual(refined.curvature_interval[0]**2*metric2**3, knum*knum)
            self.assertGreaterEqual(refined.curvature_interval[1]**2*metric2**3, knum*knum)
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

    def test_curved_closed_periodic_seam_endpoint_and_domain_rejection(self):
        s=np.linspace(0.,2*np.pi,9)
        profile=track(s,np.cos(s),np.sin(s),closed=True)
        ref=ReferenceCurve(profile)
        spline=OffsetSpline(profile,2*np.pi/8)
        coeff=.08*np.sin(2*np.pi*np.arange(spline.control_count)/spline.control_count)
        trajectory=OffsetTrajectory(ref,spline,coeff)
        start=bound_offset_geometry(trajectory,(0.,0.))
        end=bound_offset_geometry(trajectory,(2*np.pi,2*np.pi))
        self.assertNotEqual(start.curvature_interval,(Fraction(0),Fraction(0)))
        for bound, station in ((start,0.),(end,2*np.pi)):
            value=trajectory.evaluate(station).curvature
            self.assertGreaterEqual(value,float(bound.curvature_interval[0])-1e-12)
            self.assertLessEqual(value,float(bound.curvature_interval[1])+1e-12)
        with self.assertRaises(ValueError):
            bound_offset_geometry(trajectory,(np.nextafter(0.,-np.inf),.1))
        with self.assertRaises(ValueError):
            bound_offset_geometry(trajectory,(0.,np.nextafter(2*np.pi,np.inf)))
        out=bound_offset_geometry(trajectory,(0.,2*np.pi))
        dense=np.linspace(0.,2*np.pi,101)
        values=trajectory.evaluate(dense)
        for value in values.curvature:
            self.assertGreaterEqual(value,float(out.curvature_interval[0])-1e-12)
            self.assertLessEqual(value,float(out.curvature_interval[1])+1e-12)

    def test_open_periodic_source_fit_keeps_open_offset_endpoint_coefficients(self):
        s=np.linspace(0.,2*np.pi,9)
        profile=track(s,np.cos(s),np.sin(s),closed=False)
        ref=ReferenceCurve(profile,interpolation_periodic=True)
        spline=OffsetSpline(profile,2*np.pi/8)
        coeff=np.linspace(-.04,.11,spline.control_count)
        self.assertNotEqual(coeff[0],coeff[-1])
        trajectory=OffsetTrajectory(ref,spline,coeff)
        self.assertFalse(trajectory.reference.is_closed)
        left=bound_offset_geometry(trajectory,(0.,0.))
        right=bound_offset_geometry(trajectory,(2*np.pi,2*np.pi))
        self.assertNotEqual(left.offset_interval,right.offset_interval)
        out=bound_offset_geometry(trajectory,(0.,2*np.pi))
        dense=np.linspace(0.,2*np.pi,101)
        values=trajectory.evaluate(dense)
        for value in values.curvature:
            self.assertGreaterEqual(value,float(out.curvature_interval[0])-1e-12)
            self.assertLessEqual(value,float(out.curvature_interval[1])+1e-12)
        with self.assertRaises(ValueError):
            bound_offset_geometry(trajectory,(0.,np.nextafter(2*np.pi,np.inf)))

    def test_exact_total_panel_budget_and_depth_failure(self):
        t=self.trajectory
        interval=(.25,.75)
        width=Fraction(1,100)
        unconstrained=bound_offset_geometry(t,interval,maximum_panels=256,
                                            maximum_panel_curvature_width=width)
        self.assertGreater(unconstrained.panel_count,1)
        exact=bound_offset_geometry(t,interval,maximum_panels=unconstrained.panel_count,
                                    maximum_panel_curvature_width=width)
        self.assertEqual(exact.panel_count,unconstrained.panel_count)
        with self.assertRaisesRegex(RuntimeError,"panel budget"):
            bound_offset_geometry(t,interval,maximum_panels=unconstrained.panel_count-1,
                                  maximum_panel_curvature_width=width)
        with self.assertRaises(RuntimeError):
            bound_offset_geometry(t,interval,maximum_depth=0,
                                  maximum_panel_curvature_width=width)

    def test_opt_in_heading_and_rate_enclose_straight_zero_offset(self):
        t = self.trajectory
        z = OffsetTrajectory(t.reference, t.spline, np.zeros(t.spline.control_count))
        out = bound_offset_geometry(z, (0., 5.), heading_fold_guard=Fraction(1, 2))
        self.assertEqual(out.heading_error_interval, (Fraction(0), Fraction(0)))
        self.assertEqual(out.heading_error_s_interval, (Fraction(0), Fraction(0)))

    def test_heading_requires_explicit_positive_fold_guard(self):
        with self.assertRaises(ValueError):
            bound_offset_geometry(self.trajectory, (0., 5.), heading_fold_guard=0.)

    def test_heading_unresolved_preserves_full_legacy_geometry_union(self):
        t = self.trajectory
        legacy = bound_offset_geometry(t, (0., 5.))
        folded = bound_offset_geometry(t, (0., 5.), heading_fold_guard=Fraction(2))
        for name in ("offset_interval", "fold_interval", "metric_interval", "curvature_interval"):
            with self.subTest(field=name):
                self.assertEqual(getattr(folded, name), getattr(legacy, name))
        self.assertIs(folded.heading_status, HeadingStatus.GUARD_VIOLATION)

    def test_heading_policy_representations_and_closed_guard_statuses(self):
        t = self.trajectory
        z = OffsetTrajectory(t.reference, t.spline, np.zeros(t.spline.control_count))
        for name, bad in (("heading_fold_guard", np.float64(.5)),
                          ("maximum_heading", Decimal("0.1")),
                          ("maximum_heading_rate", 1)):
            kwargs = {"heading_fold_guard": .5, name: bad}
            with self.subTest(name=name, bad=bad), self.assertRaises(TypeError):
                bound_offset_geometry(z, (0., 5.), **kwargs)
        # Fold is exactly one; gamma=1 is established while gamma>1 is a
        # candidate guard violation, not a physical failure.
        equal = bound_offset_geometry(z, (0., 5.), heading_fold_guard=Fraction(1))
        self.assertIs(equal.heading_status, HeadingStatus.ESTABLISHED)
        violated = bound_offset_geometry(z, (0., 5.), heading_fold_guard=Fraction(2))
        self.assertIs(violated.heading_status, HeadingStatus.GUARD_VIOLATION)
        self.assertIsNone(violated.heading_error_interval)

    def test_heading_legacy_caps_status_limits_and_unresolved(self):
        t = self.trajectory
        z = OffsetTrajectory(t.reference, t.spline, np.zeros(t.spline.control_count))
        legacy = bound_offset_geometry(z, (0., 5.))
        self.assertIsNone(legacy.heading_error_interval)
        self.assertIsNone(legacy.heading_error_s_interval)
        self.assertIs(legacy.heading_status, HeadingStatus.NOT_REQUESTED)
        with self.assertRaises(ValueError):
            bound_offset_geometry(z, (0., 5.), heading_fold_guard=Fraction(1,2), atan_terms=0)
        exhausted = bound_offset_geometry(z, (0., 5.), heading_fold_guard=Fraction(1,2), atan_work_cap=1)
        self.assertIs(exhausted.heading_status, HeadingStatus.UNRESOLVED_TRANSCENDENTAL_BUDGET)
        self.assertIsNone(exhausted.heading_error_interval)
        self.assertIsNone(exhausted.heading_error_s_interval)
        self.assertEqual(exhausted.atan_work_cap, 1)
        self.assertEqual(exhausted.atan_work_used, exhausted.atan_work_cap)
        self.assertEqual(exhausted.atan_reductions_used, 0)
        folded = bound_offset_geometry(z, (0., 5.), heading_fold_guard=Fraction(2))
        self.assertIs(folded.heading_status, HeadingStatus.GUARD_VIOLATION)
        self.assertIsNone(folded.heading_error_interval)
        self.assertIsNone(folded.heading_error_s_interval)
        self.assertIsNone(folded.heading_limit_status)
        self.assertIsNone(folded.heading_rate_limit_status)
        within = bound_offset_geometry(z, (0., 5.), heading_fold_guard=Fraction(1,2), maximum_heading=.1, maximum_heading_rate=.1)
        self.assertIs(within.heading_limit_status, LimitStatus.WITHIN_LIMIT)
        self.assertIs(within.heading_rate_limit_status, LimitStatus.WITHIN_LIMIT)
        self.assertEqual(within.maximum_heading_limit, Fraction.from_float(.1))
        # Nonzero derivative on a straight reference has exact psi_s=n_ss/(1+n_s^2).
        coeff = np.zeros(t.spline.control_count)
        coeff[0] = .1
        nonzero = OffsetTrajectory(t.reference, t.spline, coeff)
        violated = bound_offset_geometry(nonzero, (0., 5.), heading_fold_guard=Fraction(1,2), maximum_heading=1e-9, maximum_heading_rate=1e-9)
        self.assertIs(violated.heading_limit_status, LimitStatus.UNRESOLVED_STRADDLE)
        self.assertIs(violated.heading_rate_limit_status, LimitStatus.UNRESOLVED_STRADDLE)
        with self.assertRaisesRegex(ValueError, "initial union panels exceed panel budget"):
            bound_offset_geometry(t, (0., 5.), maximum_panels=1, heading_fold_guard=Fraction(1,2), maximum_heading=.1)

    def test_direct_rational_atan_containment_and_reduction(self):
        for x in (Fraction(0), Fraction(1,2), Fraction(1), Fraction(2)):
            work = [0, 10000]
            lo, hi = _atan_point(x, 80, 32768, 8, work, [0])
            self.assertLessEqual(lo, hi)
            if x == 0:
                self.assertEqual((lo, hi), (Fraction(0), Fraction(0)))
            elif x == 1:
                # Machin π interval encloses π; atan(1)=π/4.
                self.assertGreater(hi, lo)

    def test_sources_and_stored_offset_coefficients_are_unchanged(self):
        t=self.trajectory
        arrays=(t.reference._x.c.copy(),t.reference._x.t.copy(),t._offset_curve.c.copy(),t._offset_curve.x.copy())
        result=bound_offset_geometry(t,(0.,5.))
        self.assertEqual(result.station_interval,(0.,5.))
        for actual, expected in zip((t.reference._x.c,t.reference._x.t,t._offset_curve.c,t._offset_curve.x),arrays):
            np.testing.assert_array_equal(actual,expected)


if __name__ == "__main__":
    unittest.main()
