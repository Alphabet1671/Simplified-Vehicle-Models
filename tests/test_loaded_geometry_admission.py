"""Focused tests for pure stored-fit normal-chart admission."""
from fractions import Fraction
import unittest
import numpy as np

from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile
from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve, OffsetTrajectory
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.solvers.loaded_geometry_admission import (
    AdmissionSettings, AdmissionStatus, OperationBudget, admit_loaded_geometry,
    offset_basis_domain_supported,
)


def make_reference(start=0.0, end=5.0):
    s=np.linspace(start,end,6); z=np.zeros_like(s)
    p=PathCoordinates(s,s,np.zeros_like(s),z,z,False)
    return ReferenceCurve(TrackProfile(s,np.diff(s),np.ones_like(s),z,False,"fixture",p))


def make_candidate(ref):
    profile=type("Profile", (), {"lap_length": ref.end-ref.start, "is_closed": False})()
    spline=OffsetSpline(profile,1.0)
    return OffsetTrajectory(ref,spline,np.zeros(spline.control_count))


class AdmissionTests(unittest.TestCase):
    def setUp(self):
        self.ref=make_reference()
        self.settings=AdmissionSettings(Fraction(1,2),Fraction(1,4),Fraction(1,4))

    def test_straight_zero_offset_admitted(self):
        out=admit_loaded_geometry(reference=self.ref,trajectory_or_none=make_candidate(self.ref),
            settings=self.settings,domain=(0.,5.),operation_budget=OperationBudget())
        self.assertEqual(out.status,AdmissionStatus.ADMITTED)
        self.assertLessEqual(out.reference.metric_interval[0], Fraction(1))
        self.assertGreaterEqual(out.reference.metric_interval[1], Fraction(1))
        self.assertEqual(out.candidate.fold_interval,(Fraction(1),Fraction(1)))
        self.assertEqual(out.candidate.heading_interval,(Fraction(0),Fraction(0)))
        self.assertEqual(out.candidate.rate_interval,(Fraction(0),Fraction(0)))
        self.assertEqual(out.claim,"stored_fit_geometry_only")
        self.assertFalse(out.physical_feasibility_certified)
        self.assertTrue(out.sampled_checks_not_continuous_certificate)

    def test_invalid_strict_types_and_unresolved_budget(self):
        with self.assertRaises(TypeError):
            AdmissionSettings(np.float64(.5),Fraction(1,4),Fraction(1,4))
        with self.assertRaises(TypeError):
            admit_loaded_geometry(reference=object(),trajectory_or_none=None,settings=self.settings,
                                  domain=(0.,1.),operation_budget=OperationBudget())
        out=admit_loaded_geometry(reference=self.ref,trajectory_or_none=make_candidate(self.ref),settings=self.settings,
            domain=(0.,5.),operation_budget=OperationBudget(maximum_panels=1,maximum_depth=0,sqrt_bits=16))
        self.assertNotEqual(out.status,AdmissionStatus.ADMITTED)

    def test_nonzero_origin_basis_check_is_non_mutating(self):
        ref=make_reference(1.,5.)
        profile=type("Profile", (), {"lap_length": 5.0, "is_closed": False})()
        spline=OffsetSpline(profile,1.0)
        self.assertFalse(offset_basis_domain_supported(ref,spline))
        self.assertEqual(tuple(spline.control_distance[[0,-1]]),(0.0,5.0))

    def test_heading_work_cap_propagates_unresolved(self):
        out=admit_loaded_geometry(reference=self.ref,trajectory_or_none=make_candidate(self.ref),
            settings=self.settings,domain=(0.,5.),
            operation_budget=OperationBudget(atan_terms=1,atan_work_cap=1))
        self.assertEqual(out.status,AdmissionStatus.UNRESOLVED)
        self.assertIn("unresolved", out.reason_codes[0])

    def test_same_reference_identity_required(self):
        with self.assertRaises(ValueError):
            admit_loaded_geometry(reference=self.ref,trajectory_or_none=make_candidate(make_reference()),
                settings=self.settings,domain=(0.,5.),operation_budget=OperationBudget())


if __name__ == "__main__":
    unittest.main()
