"""Active additive physical model compatibility for the setup-sweep CLI surface."""
import unittest

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from tools.run.sweep_vertical_setup import _damping_matrix, apply_overrides, valid_override_names, sweep_vertical_setup
from tools.reports.compare_iem24_four_way import suspension_description
from simplified_vehicle_models.suspension.layouts.parallel import STRUT_NAMES

FORMULATION = "models/formulations/iem24_murphy_real_14dof_simple.xml"


class PhysicalSweepConsumerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(FORMULATION).model

    def test_names_use_physical_rates_and_disabled_dampers_have_no_clicks(self):
        names = valid_override_names(self.vehicle)
        self.assertIn("rate_heave-front", names)
        self.assertIn("inertance_roll-rear", names)
        self.assertIn("preload_heave-front", names)
        self.assertNotIn("corner-fl.click_compression", names)
        self.assertIn("rate_corner-fl", names)
        self.assertIn("rate_corner-fr", names)
        self.assertIn("rate_corner-rl", names)
        self.assertIn("rate_corner-rr", names)

    def test_overrides_keep_physical_values_and_click_targeting(self):
        updated = apply_overrides(self.vehicle, {
            "rate_heave-front": 70000.0,
            "inertance_roll-front": 2.5,
            "heave-front.click_compression": 4,
        })
        network = updated.suspension.network
        front = network.struts[STRUT_NAMES.index("heave-front")]
        roll = network.struts[STRUT_NAMES.index("roll-front")]
        self.assertEqual(front.spring_rate, 70000.0)
        self.assertEqual(front.damper.setting.clicks["compression"], 4.0)
        self.assertEqual(roll.inertance, 2.5)
        self.assertNotEqual(self.vehicle.suspension.network.struts[4].spring_rate, 70000.0)

    def test_zero_corner_spring_rate_can_be_overridden_physically(self):
        updated = apply_overrides(self.vehicle, {"rate_corner-fl": 12000.0})
        corner = updated.suspension.network.struts[STRUT_NAMES.index("corner-fl")]
        self.assertEqual(corner.spring_rate, 12000.0)
        self.assertEqual(self.vehicle.suspension.network.struts[0].spring_rate, 0.0)

    def test_preload_on_zero_spring_remains_invalid(self):
        from dataclasses import replace
        corner = self.vehicle.suspension.network.struts[STRUT_NAMES.index("corner-fl")]
        with self.assertRaisesRegex(ValueError, "nonzero preload requires a nonzero spring rate"):
            replace(corner, preload=10.0)

    def test_damping_matrix_scales_physical_damper_linearly(self):
        from dataclasses import replace
        network = self.vehicle.suspension.network
        scales = np.array([0.0, 0.25, 0.5, 0.0, 0.75, 0.4, 0.0, 1.5])
        struts = tuple(replace(strut, damper_scale=float(scale))
                       for strut, scale in zip(network.struts, scales))
        vehicle = replace(self.vehicle, suspension=replace(
            self.vehicle.suspension, network=replace(network, struts=struts)))
        slopes = np.array([11., 13., 17., 19., 23., 29., 31., 37.])
        A = vehicle.suspension.network.jacobian
        expected = A.T @ np.diag(slopes * scales) @ A
        actual = _damping_matrix(vehicle, slopes, np.zeros(4))
        np.testing.assert_allclose(actual, expected)

    def test_linear_disabled_corner_damper_has_no_click_operation(self):
        with self.assertRaisesRegex(ValueError, "unknown override"):
            apply_overrides(self.vehicle, {"corner-fl.click_compression": 2})

    def test_report_describes_active_physical_struts_not_legacy_corner_curves(self):
        description = suspension_description(self.vehicle.suspension)
        self.assertIn("parallel physical suspension", description)
        self.assertIn("all eight devices", description)
        self.assertIn("corner-fl: spring=0 N/m, preload=0 N, inertance=0 kg", description)
        self.assertIn("damper_scale=0", description)
        self.assertIn("MR=", description)
        self.assertIn("orientation=", description)
        self.assertIn("heave-front: spring=", description)
        self.assertIn("settings=[", description)
        self.assertIn("no corner bump stops or separate ARB", description)

    def test_legacy_quarter_car_metrics_reject_active_physical_layout(self):
        with self.assertRaisesRegex(ValueError, "quarter-car"):
            sweep_vertical_setup(FORMULATION, [{}], "step-response")
