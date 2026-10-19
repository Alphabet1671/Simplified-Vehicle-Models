"""Evaluation-domain certificate of the vertical 14-DoF plant: pass and fail-closed cases."""
import unittest

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.forward_driver import DriverCommand, SpeedPlan
from simplified_vehicle_models.solvers.vertical_between_step import (
    certify_vertical_between_step, _JOUNCE, _JOUNCE_RATE, _HEAVE, _VX, _WHEELS, _SOC)
from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant
from tools.run.run_iem24_forward_driver import VERTICAL_FORMULATION


class VerticalBetweenStepTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(str(VERTICAL_FORMULATION)).model
        cls.plan = SpeedPlan(np.array([0., 100.]), np.array([8., 8.]), np.zeros(2))
        plant = VerticalForwardPlant(cls.vehicle, cls.plan)
        cls.state = np.append(plant.initial_state(8.), 0.)

    def box(self, overrides=None, pad=1.e-4):
        lo = self.state - pad
        hi = self.state + pad
        for index, (a, b) in (overrides or {}).items():
            lo[index], hi[index] = a, b
        return np.stack([lo, hi], axis=-1)[None]

    def certify(self, bounds):
        return certify_vertical_between_step(self.vehicle, self.plan, bounds, None, True)

    def test_equilibrium_neighbourhood_is_certified(self):
        result = self.certify(self.box())
        self.assertTrue(result["certified"], result["checks"])
        self.assertTrue(all(result["checks"].values()))

    def test_wrong_shape_fails_closed(self):
        self.assertFalse(self.certify(self.box()[:, :30])["certified"])

    def test_non_finite_enclosure_fails_closed(self):
        bounds = self.box()
        bounds[0, 5, 1] = np.nan
        self.assertFalse(self.certify(bounds)["certified"])

    def test_inverted_enclosure_fails_closed(self):
        bounds = self.box()
        bounds[0, 5] = bounds[0, 5, ::-1] + np.array([1., -1.])
        self.assertFalse(self.certify(bounds)["certified"])

    def test_jounce_outside_spline_domain_fails(self):
        limit = self.vehicle.suspension.front.jounce_limits[1]
        result = self.certify(self.box({_JOUNCE[0]: (0., limit + 1.e-3)}))
        self.assertFalse(result["certified"])
        self.assertFalse(result["checks"]["spring_mr_jounce_steer_domain"])

    def test_damper_velocity_beyond_table_fails(self):
        result = self.certify(self.box({_JOUNCE_RATE[2]: (0., 2.0)}))
        self.assertFalse(result["certified"])
        self.assertFalse(result["checks"]["damper_velocity_inside_table"])

    def test_extreme_heave_leaves_tire_inverse_domain(self):
        result = self.certify(self.box({_HEAVE: (0., 5.0)}))
        self.assertFalse(result["certified"])
        self.assertFalse(result["checks"]["tire_deflection_in_inverse_domain"])

    def test_plant_guards_fail_closed(self):
        for index, interval in ((_VX, (0.1, 8.)), (_WHEELS[1], (-1., 40.)), (_SOC, (0.5, 1.2))):
            with self.subTest(index=index):
                result = self.certify(self.box({index: interval}))
                self.assertFalse(result["certified"])
                self.assertFalse(result["checks"]["plant_guards"] and
                                 result["checks"]["state_domain_and_path"])

    def test_empty_enclosure_fails_closed(self):
        self.assertFalse(self.certify(self.box()[:0])["certified"])

    def test_rhs_agrees_with_certified_box(self):
        """Every sampled state of a certified box evaluates without a plant exception."""
        bounds = self.box(pad=1.e-3)
        self.assertTrue(self.certify(bounds)["certified"])
        plant = VerticalForwardPlant(self.vehicle, self.plan)
        rng = np.random.default_rng(7)
        for throttle, brake in ((0.6, 0.), (0., 100.)):
            for _ in range(15):
                sample = rng.uniform(bounds[0, :, 0], bounds[0, :, 1])
                plant.evaluate(0., sample[:38], DriverCommand(throttle, brake, 0.))  # must not raise


if __name__ == "__main__":
    unittest.main()
