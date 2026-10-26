"""Focused tests for explicit polynomial/stored-chart interval composition."""
from fractions import Fraction
import json
import unittest
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.formulations.signed_capability import SignedTransientCapability
from simplified_vehicle_models.solvers.chart_element_certificate import (
    ChartElementSettings, check_chart_element, _float_hi, _float_lo,
)
from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile
from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve, OffsetTrajectory
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline


def trajectory():
    s = np.array([0., 1., 2., 3., 4.])
    z = np.zeros_like(s)
    profile = TrackProfile(s, np.diff(s), np.ones_like(s), z, False, "test",
                           PathCoordinates(s, s, z, z, z, False))
    ref = ReferenceCurve(profile)
    spline = OffsetSpline(profile, 1.)
    return OffsetTrajectory(ref, spline, np.zeros(spline.control_count))


def chart(*, floor=.5, domain=30., amin=-10., amax=10., curvature=(-1.,1.)):
    u = np.array([0., 1.])
    k = np.array(curvature)
    return SignedTransientCapability(u, k, np.full((2,2), amax), np.full((2,2), amin),
                                     np.full(2, domain), floor, "test")


class ChartElementCertificateTests(unittest.TestCase):
    def test_constant_safe_explicit_polynomial_proves_chart_only(self):
        t, c = trajectory(), chart()
        out = check_chart_element(t, c, (0., 4.), (100,100), (0,0,0))
        self.assertTrue(out["chart_constraints_proven"], out)
        self.assertEqual(out["status"], "chart_constraints_proven")
        self.assertFalse(out["accepted"])
        for key in ("physical_feasibility_certified", "original_vehicle_validated", "event_qualified",
                    "U3_qualified", "policy_approved", "finite_public_eval_not_certified"):
            self.assertFalse(out[key]) if key != "finite_public_eval_not_certified" else self.assertTrue(out[key])
        json.dumps(out, allow_nan=False)
        self.assertEqual(out["panels_evaluated"], 1)

    def test_explicit_acceleration_violation_is_not_infeasibility_claim(self):
        out = check_chart_element(trajectory(), chart(), (0., 4.), (100,180), (20,20,20),
                                  maximum_speed_rate_defect=100.)
        self.assertFalse(out["chart_constraints_proven"])
        self.assertEqual(out["status"], "not_proven")
        self.assertFalse(out["accepted"])
        self.assertIn("acceleration", out["reason"])

    def test_midpoint_lagrange_spike_is_not_missed_by_nodes(self):
        # F(0)=F(1)=0, F(1/2)=40; W endpoints satisfy exact Hermite integral.
        out = check_chart_element(trajectory(), chart(), (0., 1.), (100, 100+Fraction(40,3)),
                                  (0,40,0), maximum_speed_rate_defect=100.)
        self.assertFalse(out["chart_constraints_proven"])
        self.assertFalse(out["accepted"])

    def test_energy_control_dynamic_defect_is_not_suppressed(self):
        out = check_chart_element(trajectory(), chart(), (0.,1.), (100,100), (1,1,1),
                                  maximum_speed_rate_defect=0.)
        self.assertFalse(out["chart_constraints_proven"])
        self.assertIn("defect", out["reason"])
        self.assertFalse(out["accepted"])

    def test_floor_and_domain_violations_remain_not_proven(self):
        floor = check_chart_element(trajectory(), chart(floor=11.), (0.,1.), (100,100), (0,0,0))
        domain = check_chart_element(trajectory(), chart(domain=9.), (0.,1.), (100,100), (0,0,0))
        for out in (floor, domain):
            self.assertEqual(out["status"], "not_proven")
            self.assertFalse(out["accepted"])

    def test_fold_failure_and_geometry_budget_failure(self):
        t = trajectory()
        # The straight stored path's fold is one; demanding two cannot pass.
        fold = check_chart_element(t, chart(), (0.,1.), (100,100), (0,0,0), minimum_fold=2.)
        self.assertFalse(fold["chart_constraints_proven"])
        budget = check_chart_element(t, chart(), (0.,4.), (100,100), (0,0,0),
                                     settings=ChartElementSettings(maximum_geometry_panels=1,
                                                                   geometry_maximum_depth=0))
        self.assertFalse(budget["chart_constraints_proven"])
        self.assertFalse(budget["accepted"])
        self.assertTrue(budget["geometry_failures"])

    def test_rational_binary64_rectangle_conversion_is_outward(self):
        for q in (Fraction(1,3), Fraction.from_float(.5)+Fraction(1,10**20),
                  Fraction.from_float(.5)-Fraction(1,10**20)):
            self.assertLessEqual(Fraction.from_float(_float_lo(q)), q)
            self.assertGreaterEqual(Fraction.from_float(_float_hi(q)), q)
        self.assertEqual(_float_lo(Fraction(1,2)), .5)
        self.assertEqual(_float_hi(Fraction(1,2)), .5)

    def test_exact_fraction_parameter_and_budget_are_json_safe_false(self):
        exact = check_chart_element(trajectory(), chart(), (0.,1.), (100,100), (0,0,0),
                                    speed_domain_margin_mps=Fraction(1,3))
        self.assertTrue(exact["chart_constraints_proven"], exact)
        self.assertFalse(exact["accepted"])
        exhausted = check_chart_element(trajectory(), chart(), (0.,4.), (100,100), (0,0,0),
                     settings=ChartElementSettings(maximum_panels=1, maximum_depth=0,
                                                   maximum_geometry_panels=1, geometry_maximum_depth=0))
        self.assertEqual(exhausted["status"], "not_proven")
        self.assertFalse(exhausted["accepted"])
        json.dumps(exact, allow_nan=False)
        json.dumps(exhausted, allow_nan=False)

    def test_wall_deadline_returns_not_proven(self):
        ticks = iter((0., 1., 1., 1., 2.))
        with patch("simplified_vehicle_models.solvers.chart_element_certificate.time.monotonic",
                   side_effect=lambda: next(ticks)):
            out = check_chart_element(trajectory(), chart(), (0.,1.), (100,100), (0,0,0),
                                      settings=ChartElementSettings(maximum_wall_seconds=.5))
        self.assertEqual(out["status"], "not_proven")
        self.assertEqual(out["reason"], "wall_budget_exhausted")
        self.assertFalse(out["accepted"])

    def test_false_settings_and_boolean_source_endpoints_do_not_prove(self):
        invalid_settings = check_chart_element(trajectory(), chart(), (0.,1.), (100,100), (0,0,0), settings=False)
        boolean_source = check_chart_element(trajectory(), chart(), (False,1.), (100,100), (0,0,0))
        for out in (invalid_settings, boolean_source):
            self.assertEqual(out["status"], "not_proven")
            self.assertFalse(out["chart_constraints_proven"])
            self.assertFalse(out["accepted"])

    def test_source_arrays_unchanged(self):
        t, c = trajectory(), chart()
        before = [x.copy() for x in (c.u,c.curvature,c.acceleration_min,c.acceleration_max,c.domain_speed_values,
                                     t._offset_curve.c)]
        check_chart_element(t,c,(0.,4.),(100,100),(0,0,0))
        for old,new in zip(before,(c.u,c.curvature,c.acceleration_min,c.acceleration_max,c.domain_speed_values,
                                  t._offset_curve.c)):
            np.testing.assert_array_equal(old,new)


if __name__ == "__main__":
    unittest.main()
