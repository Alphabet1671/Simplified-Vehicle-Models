import unittest
from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.formulations.yaw_map_domain import (
    ProbeResult, ProbeStatus, UnsupportedLateralBoundError, YawMapDomainSettings,
    discover_yaw_map_domain, unitire_peak_lateral_bound,
)
from simplified_vehicle_models.parameters.tire import UniTirePeakEnvelope


@dataclass
class Vehicle:
    total_mass: float
    tire: object


def vehicle(peaks=(100., 120., 110.)):
    return Vehicle(40., UniTirePeakEnvelope(
        vertical_loads=np.array([0., 500., 1000.]),
        longitudinal_peak_force=np.array([0., 100., 100.]),
        lateral_peak_force=np.array(peaks), effective_radius=.2, reference_speed=3.,
    ))


def evaluator(predicate):
    def evaluate(_speed, _ax, ay):
        return ProbeResult(ProbeStatus.FEASIBLE if predicate(ay) else ProbeStatus.INFEASIBLE)
    return evaluate


class YawMapDomainTest(unittest.TestCase):
    def test_model_derived_bound_exceeds_one_g_and_marks_table_saturation(self):
        bound = unitire_peak_lateral_bound(vehicle())
        self.assertEqual(bound.peak_lateral_force, 120.)
        self.assertEqual(bound.lateral_acceleration, 12.)
        self.assertGreater(bound.lateral_acceleration, 9.81)
        self.assertTrue(bound.tyre_load_table_saturates)

    def test_unsupported_tyre_does_not_substitute_g_cap(self):
        with self.assertRaises(UnsupportedLateralBoundError):
            unitire_peak_lateral_bound(Vehicle(40., object()))

    def test_signed_asymmetric_and_holdout_scan(self):
        domain = discover_yaw_map_domain(
            vehicle(), [0., 10.], [-1., 1.],
            evaluator(lambda ay: -5.0 <= ay <= 2.0),
            YawMapDomainSettings(scan_spacing=1., bracket_tolerance=.125,
                                 probe_budget=1000, speed_ax_audit_subdivisions=1),
        )
        self.assertEqual(domain.audited_speeds, (0., 5., 10.))
        self.assertEqual(domain.audited_longitudinal_accelerations, (-1., 0., 1.))
        slice_ = next(item for item in domain.slices if item.speed == 5. and item.longitudinal_acceleration == 0.)
        self.assertEqual(len(slice_.components), 2)
        negative, positive = slice_.components
        self.assertLessEqual(negative.lower_transition.width, .125)
        self.assertLessEqual(positive.upper_transition.width, .125)
        self.assertLess(negative.sampled_lower_ay, -1.)
        self.assertGreaterEqual(positive.sampled_upper_ay, 2.)
        self.assertTrue(domain.diagnostics.complete)

    def test_disconnected_sampled_components_are_preserved(self):
        domain = discover_yaw_map_domain(
            vehicle(), [5.], [0.], evaluator(lambda ay: 1. <= ay <= 2. or 5. <= ay <= 6.),
            YawMapDomainSettings(scan_spacing=1., bracket_tolerance=.25, probe_budget=1000),
        )
        components = domain.slices[0].components
        self.assertEqual([(item.sampled_lower_ay, item.sampled_upper_ay) for item in components], [(1., 2.), (5., 6.)])
        self.assertTrue(domain.scan_resolution_limited)

    def test_empty_slice_is_not_an_error(self):
        domain = discover_yaw_map_domain(
            vehicle(), [5.], [0.], evaluator(lambda _ay: False),
            YawMapDomainSettings(scan_spacing=1., bracket_tolerance=.25, probe_budget=1000),
        )
        self.assertEqual(domain.slices[0].components, ())
        self.assertTrue(domain.diagnostics.complete)

    def test_unresolved_lp_is_never_relabelled_infeasible(self):
        def unresolved(_speed, _ax, ay):
            if ay == 2.:
                return ProbeResult(ProbeStatus.UNRESOLVED, "HiGHS numerical failure")
            return ProbeResult(ProbeStatus.INFEASIBLE)
        domain = discover_yaw_map_domain(
            vehicle(), [5.], [0.], unresolved,
            YawMapDomainSettings(scan_spacing=1., bracket_tolerance=.25, probe_budget=1000),
        )
        self.assertFalse(domain.diagnostics.complete)
        self.assertIn((5., 0., 2., "HiGHS numerical failure"), domain.diagnostics.unresolved_probes)

    def test_probe_budget_reports_incomplete_without_claiming_infeasible(self):
        domain = discover_yaw_map_domain(
            vehicle(), [5.], [0.], evaluator(lambda _ay: False),
            YawMapDomainSettings(scan_spacing=1., bracket_tolerance=.25, probe_budget=2),
        )
        self.assertTrue(domain.diagnostics.budget_exhausted)
        self.assertFalse(domain.diagnostics.complete)
        self.assertGreater(len(domain.diagnostics.unresolved_probes), 0)

    def test_malformed_settings_rejected(self):
        for kwargs in (
            {"scan_spacing": 0.}, {"bracket_tolerance": 2., "scan_spacing": 1.},
            {"probe_budget": 0}, {"speed_ax_audit_subdivisions": -1},
            {"lateral_interpolation_loss_tolerance": -1.},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                YawMapDomainSettings(**kwargs)


if __name__ == "__main__":
    unittest.main()
