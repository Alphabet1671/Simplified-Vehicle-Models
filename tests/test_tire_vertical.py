"""Test the tire vertical load law: exact inverse above 0.5 mm, C1 ramp below."""

from __future__ import annotations

import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from simplified_vehicle_models.compiler.registry import component_descriptor
from simplified_vehicle_models.tires.unitire_simple import loaded_radius
from simplified_vehicle_models.tires.vertical import (
    TireVertical,
    require_exact_inverse_supported,
    tire_deflection_load,
    transition_knots,
)
from simplified_vehicle_models.xml.loader import load_model

ROOT = Path(__file__).resolve().parents[1]
TIRE_XML = ROOT / "models" / "tires" / "iem24_murphy_real_unitire.xml"
DAMPING = 400.0
# The tire carries the sprung and the unsprung weight of its corner.
FRONT_STATIC = 287 * 0.45 / 2 * 9.80665
REAR_STATIC = 287 * 0.55 / 2 * 9.80665


def _tire():
    model = load_model(TIRE_XML)
    return component_descriptor(model.model_type, model.subtype).compile_function(model)


def _load(tire, delta, rate=0.0, damping=DAMPING):
    return tire_deflection_load(
        tire, np.atleast_1d(np.asarray(delta, dtype=float)), np.atleast_1d(np.asarray(rate, dtype=float)), damping
    )


class ExactInverseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.tire = _tire()
        cls.delta0, cls.delta1, cls.f1, cls.k1 = transition_knots(cls.tire)

    def test_free_radius_is_first_radius_coefficient(self) -> None:
        self.assertEqual(float(loaded_radius(self.tire, 0.0)), self.tire.r_1)

    def test_inverse_residual_is_below_a_nanometre(self) -> None:
        loads = np.linspace(self.f1, 3000.0, 400)
        delta = self.tire.r_1 - loaded_radius(self.tire, loads)
        result = _load(self.tire, delta)
        recovered = self.tire.r_1 - loaded_radius(self.tire, result.load)
        self.assertLessEqual(float(np.max(np.abs(recovered - delta))), 1.0e-9)
        np.testing.assert_allclose(result.load, loads, rtol=1.0e-9, atol=1.0e-6)

    def test_first_knot_is_the_exact_load_at_half_millimetre(self) -> None:
        self.assertEqual(self.delta1, 5.0e-4)
        residual = self.tire.r_1 - loaded_radius(self.tire, self.f1) - self.delta1
        self.assertLessEqual(abs(float(residual)), 1.0e-12)
        self.assertAlmostEqual(self.f1, 39.7, delta=0.05)

    def test_static_stiffness_matches_measured_rates(self) -> None:
        delta_front = self.tire.r_1 - loaded_radius(self.tire, FRONT_STATIC)
        delta_rear = self.tire.r_1 - loaded_radius(self.tire, REAR_STATIC)
        front = _load(self.tire, delta_front).stiffness[0] / 1.0e3
        rear = _load(self.tire, delta_rear).stiffness[0] / 1.0e3
        self.assertAlmostEqual(front, 80.4, delta=0.1)
        self.assertAlmostEqual(rear, 80.7, delta=0.1)

    def test_stiffness_is_derivative_of_the_load(self) -> None:
        step = 1.0e-7
        for delta in (-0.0004, 0.0001, 0.0003, 0.002, 0.012, 0.03):
            with self.subTest(delta=delta):
                up = _load(self.tire, delta + step, damping=0.0).load[0]
                down = _load(self.tire, delta - step, damping=0.0).load[0]
                stiffness = _load(self.tire, delta, damping=0.0).stiffness[0]
                self.assertAlmostEqual(stiffness, (up - down) / (2 * step), delta=2.0e-3 * stiffness + 1e-6)


class LowLoadTransitionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.tire = _tire()
        cls.delta0, cls.delta1, cls.f1, cls.k1 = transition_knots(cls.tire)

    def test_lower_knot_follows_the_specified_formula(self) -> None:
        self.assertAlmostEqual(self.delta0, self.delta1 - 2.0 * self.f1 / self.k1, places=15)
        self.assertLess(self.delta0, 0.0)

    def test_quadratic_branch_matches_its_own_formula(self) -> None:
        delta = np.linspace(self.delta0, self.delta1, 41)
        expected = self.f1 * ((delta - self.delta0) / (self.delta1 - self.delta0)) ** 2
        np.testing.assert_allclose(_load(self.tire, delta, damping=0.0).load, expected, rtol=1e-12, atol=1e-12)

    def test_zero_deflection_has_about_ten_newtons(self) -> None:
        load = float(_load(self.tire, 0.0, damping=0.0).load[0])
        self.assertAlmostEqual(load, 9.9, delta=0.1)

    def test_load_is_zero_below_the_lower_knot_and_at_minus_one_millimetre(self) -> None:
        result = _load(self.tire, [-1.0e-3, -0.01, self.delta0, self.delta0 - 1e-9], 0.3)
        np.testing.assert_array_equal(result.load, np.zeros(4))
        np.testing.assert_array_equal(result.stiffness, np.zeros(4))
        self.assertFalse(np.any(result.contact))

    def test_load_is_never_negative(self) -> None:
        delta = np.linspace(-0.01, 0.04, 2001)
        for rate in (-5.0, -0.1, 0.0, 0.1, 5.0):
            with self.subTest(rate=rate):
                self.assertGreaterEqual(float(np.min(_load(self.tire, delta, rate).load)), 0.0)

    def test_contact_is_positive_elastic_load(self) -> None:
        delta = np.array([-1e-3, self.delta0, self.delta0 + 1e-7, 0.0, 0.01])
        result = _load(self.tire, delta, 0.5)
        np.testing.assert_array_equal(result.contact, [False, False, True, True, True])
        self.assertEqual(result.contact.dtype, bool)

    def test_value_and_slope_are_continuous_at_both_knots(self) -> None:
        eps = 1.0e-10
        for knot in (self.delta0, self.delta1):
            with self.subTest(knot=knot):
                below = _load(self.tire, knot - eps, damping=0.0)
                above = _load(self.tire, knot + eps, damping=0.0)
                self.assertAlmostEqual(below.load[0], above.load[0], delta=1.0e-4)
                self.assertAlmostEqual(below.stiffness[0], above.stiffness[0], delta=0.1)
        at_upper = _load(self.tire, self.delta1, damping=0.0)
        self.assertAlmostEqual(at_upper.load[0], self.f1, places=9)
        self.assertAlmostEqual(at_upper.stiffness[0], self.k1, delta=1.0e-6)
        self.assertEqual(float(_load(self.tire, self.delta0, damping=0.0).stiffness[0]), 0.0)

    def test_second_derivative_jumps_so_the_law_is_c1_not_c2(self) -> None:
        eps = 1.0e-6
        slope_below = (
            _load(self.tire, self.delta1 - 1e-6 + eps, damping=0.0).stiffness[0]
            - _load(self.tire, self.delta1 - 1e-6, damping=0.0).stiffness[0]
        ) / eps
        slope_above = (
            _load(self.tire, self.delta1 + 1e-6 + eps, damping=0.0).stiffness[0]
            - _load(self.tire, self.delta1 + 1e-6, damping=0.0).stiffness[0]
        ) / eps
        self.assertGreater(abs(slope_below - slope_above), 1.0e6)

    def test_blend_is_the_upper_knot(self) -> None:
        coarse = transition_knots(self.tire, blend=1.0e-3)
        self.assertEqual(coarse[1], 1.0e-3)
        with self.assertRaises(ValueError):
            transition_knots(self.tire, blend=0.0)


class DampingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.tire = _tire()

    def test_zero_rate_leaves_elastic_load_unchanged(self) -> None:
        delta = np.array([0.0, 0.001, 0.01])
        np.testing.assert_allclose(_load(self.tire, delta, 0.0).load, _load(self.tire, delta, 0.0, 0.0).load)

    def test_compression_adds_linear_damping_force(self) -> None:
        delta = 0.01
        elastic = float(_load(self.tire, delta, 0.0).load[0])
        damped = float(_load(self.tire, delta, 0.2).load[0])
        self.assertAlmostEqual(damped, elastic * (1.0 + DAMPING * 0.2 / (elastic + 1.0)), places=9)
        self.assertGreater(damped, elastic)

    def test_extension_reduces_load_but_never_below_zero(self) -> None:
        delta = 0.01
        elastic = float(_load(self.tire, delta, 0.0).load[0])
        for rate in (-0.01, -0.1, -1.0):
            with self.subTest(rate=rate):
                damped = float(_load(self.tire, delta, rate).load[0])
                self.assertLess(damped, elastic)
                self.assertGreater(damped, 0.0)
                self.assertAlmostEqual(
                    damped, elastic * (1.0 + np.tanh(DAMPING * rate / (elastic + 1.0))), places=9
                )
        for rate in (-50.0, -1.0e6):
            with self.subTest(rate=rate):
                self.assertGreaterEqual(float(_load(self.tire, delta, rate).load[0]), 0.0)

    def test_no_damping_without_static_load(self) -> None:
        result = _load(self.tire, [-0.002, -0.0006], [10.0, -10.0])
        np.testing.assert_array_equal(result.load, [0.0, 0.0])

    def test_damped_load_is_continuous_in_rate_and_deflection(self) -> None:
        rate = np.linspace(-2.0, 2.0, 40001)
        load = _load(self.tire, 0.003, rate).load
        self.assertLess(float(np.max(np.abs(np.diff(load)))), 0.1)
        slope_left = (_load(self.tire, 0.003, -1e-8).load[0] - _load(self.tire, 0.003, -2e-8).load[0]) / 1e-8
        slope_right = (_load(self.tire, 0.003, 2e-8).load[0] - _load(self.tire, 0.003, 1e-8).load[0]) / 1e-8
        self.assertAlmostEqual(slope_left, slope_right, delta=1.0e-3 * abs(slope_right))
        delta = np.linspace(-0.001, 0.002, 30001)
        self.assertLess(float(np.max(np.abs(np.diff(_load(self.tire, delta, -0.5).load)))), 0.05)

    def test_results_are_arrays_with_broadcast_shape(self) -> None:
        result = tire_deflection_load(self.tire, np.zeros((3, 4)), np.zeros(4), DAMPING)
        self.assertIsInstance(result, TireVertical)
        self.assertEqual(result.load.shape, (3, 4))
        self.assertEqual(result.stiffness.shape, (3, 4))
        self.assertEqual(result.contact.shape, (3, 4))

    def test_invalid_damping_is_rejected(self) -> None:
        for bad in (-1.0, float("nan"), float("inf")):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    _load(self.tire, 0.0, 0.0, bad)


class SupportCheckTests(unittest.TestCase):
    def test_current_tire_is_supported(self) -> None:
        require_exact_inverse_supported(_tire())

    def test_nonzero_lateral_shift_or_radius_correction_is_rejected(self) -> None:
        tire = _tire()
        for name in ("f_y_shift_0", "f_y_shift_fz", "k_rl_0", "k_rl_1", "k_rl_2"):
            with self.subTest(name=name):
                with self.assertRaisesRegex(ValueError, name):
                    require_exact_inverse_supported(replace(tire, **{name: 1.0e-3}))

    def test_nonmonotone_radius_polynomial_is_rejected(self) -> None:
        tire = _tire()
        with self.assertRaises(ValueError):
            require_exact_inverse_supported(replace(tire, r_2=0.001))
        with self.assertRaises(ValueError):
            require_exact_inverse_supported(replace(tire, r_3=1.0e-3))


if __name__ == "__main__":
    unittest.main()
