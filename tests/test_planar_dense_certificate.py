"""Adversarial checks of the planar dense-interval certificate."""
import unittest
from dataclasses import replace

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.planar_7dof import (
    PlanarState, evaluate_planar_dynamics)
from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
from simplified_vehicle_models.solvers.planar_dense_certificate import (
    certify_planar_dense_domain, contact_nonnegativity_conditions,
    curvature_enclosure)


def _bounds(count=3, pedal=True):
    """Return a benign enclosure around a cruising state."""
    nominal = np.array([50., 0.1, 0.02, 15., 0.2, 0.1, 50., 50., 50., 50., .8, .05, .2])
    if not pedal:
        nominal = nominal[:12]
    box = np.stack([nominal - 1.e-3, nominal + 1.e-3], axis=-1)
    return np.repeat(box[None], count, axis=0)


class DenseCertificateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(
            "models/formulations/iem24_murphy_real_planar_7dof.xml").model
        distance = np.linspace(0., 200., 401)
        cls.plan = SpeedPlan(distance, np.full(401, 10.), np.full(401, 0.05))

    def certify(self, bounds, pedal=True):
        return certify_planar_dense_domain(self.vehicle, self.plan, bounds, pedal)

    def test_benign_enclosure_certifies(self):
        result = self.certify(_bounds())
        self.assertTrue(result["certified"], result)
        self.assertEqual(result["intervals"], 3)

    def test_each_explicit_limit_fails_independently(self):
        cases = {
            "soc_floor": (10, 0, -0.85),
            "soc_ceiling": (10, 1, 0.5),
            "speed_over_minimum": (3, 0, -14.9),
            "wheel_speed_floor": (7, 0, -60.),
            "steering_headroom": (11, 1, 1.),
            "path_denominator_over_minimum": (1, 1, 15.),
            "path_progress_over_minimum": (3, 0, -14.9),
        }
        for name, (component, side, delta) in cases.items():
            with self.subTest(name):
                bounds = _bounds()
                bounds[1, component, side] += delta
                result = self.certify(bounds)
                self.assertFalse(result["certified"])
                self.assertIn(name, result["failed"])

    def test_violation_in_a_single_interval_is_found(self):
        bounds = _bounds(count=500)
        bounds[417, 11, 1] = 0.6
        result = self.certify(bounds)
        self.assertFalse(result["certified"])
        self.assertIn("steering_headroom", result["failed"])

    def test_heading_excursion_through_pi_kills_progress(self):
        bounds = _bounds()
        bounds[0, 2, :] = [3.0, 3.3]
        self.assertIn("path_progress_over_minimum", self.certify(bounds)["failed"])

    def test_nonfinite_or_malformed_enclosure_fails_closed(self):
        nan = _bounds()
        nan[0, 4, 0] = np.nan
        self.assertFalse(self.certify(nan)["certified"])
        inf = _bounds()
        inf[0, 3, 1] = np.inf
        self.assertFalse(self.certify(inf)["certified"])
        self.assertFalse(self.certify(_bounds(count=0))["certified"])
        self.assertFalse(self.certify(_bounds(pedal=False))["certified"])
        self.assertFalse(self.certify(_bounds()[:, :, 0])["certified"])

    def test_pedal_overshoot_is_reported_not_rejected(self):
        bounds = _bounds()
        bounds[0, 12, 1] = 1.2
        result = self.certify(bounds)
        self.assertTrue(result["certified"])
        self.assertGreater(result["margins"]["pedal_overshoot_beyond_travel"], 0.)

    def test_curvature_enclosure_includes_interior_node_peak(self):
        distance = np.array([0., 1., 2., 3., 4.])
        curvature = np.array([0., 0., 0.5, 0., 0.])
        low, high = curvature_enclosure(distance, curvature, np.array([0.5, 3.5]),
                                        np.array([2.5, 4.5]))
        np.testing.assert_allclose(high, [0.5, 0.])
        np.testing.assert_allclose(low, [0., 0.])
        # An interval that wraps over the seam.
        low, high = curvature_enclosure(distance, curvature, np.array([3.5]),
                                        np.array([6.5]))
        self.assertAlmostEqual(float(high[0]), 0.5)


class ContactConditionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(
            "models/formulations/iem24_murphy_real_planar_7dof.xml").model

    def test_shipped_vehicle_satisfies_conditions(self):
        result = contact_nonnegativity_conditions(self.vehicle)
        self.assertTrue(result["holds"], result)
        self.assertLess(result["ratios"]["lateral_lift"], 1.)

    def test_review_counterexamples_fail_the_checker(self):
        vehicle = self.vehicle
        cases = {
            "drag": replace(vehicle, aero=replace(vehicle.aero, drag_area=15.)),
            "geometry": replace(vehicle, track_width=3.0, wheelbase=0.9),
            "linear branch": replace(vehicle, tire=replace(vehicle.tire, slip_eps=1.e9)),
            "tall cg": replace(vehicle, cg_height=0.5),
            "no downforce with drag": replace(
                vehicle, aero=replace(vehicle.aero, downforce_area=0.)),
        }
        for name, case in cases.items():
            with self.subTest(name):
                self.assertFalse(contact_nonnegativity_conditions(case)["holds"])

    def test_counterexample_vehicles_really_lose_contact(self):
        """Ground the rejection: the flagged vehicles do produce negative loads."""
        drag = replace(self.vehicle, aero=replace(self.vehicle.aero, drag_area=15.))
        state = PlanarState(30., 0., 0., np.full(4, 150.), 0.)
        loads = evaluate_planar_dynamics(drag, state, np.zeros(4), 0.).normal_loads
        self.assertLess(float(np.min(loads)), 0.)

    def test_shipped_vehicle_never_goes_negative_on_random_states(self):
        random = np.random.default_rng(7)
        worst = np.inf
        for _ in range(400):
            speed = random.uniform(1., 40.)
            state = PlanarState(speed, random.uniform(-6., 6.), random.uniform(-3., 3.),
                                random.uniform(0., 1.5, 4) * speed / 0.2, 0.)
            torques = random.uniform(-300., 300., 4)
            steer = random.uniform(-0.5, 0.5)
            loads = evaluate_planar_dynamics(self.vehicle, state, torques, steer).normal_loads
            worst = min(worst, float(np.min(loads)))
        self.assertGreaterEqual(worst, -1.e-9)


if __name__ == "__main__":
    unittest.main()
