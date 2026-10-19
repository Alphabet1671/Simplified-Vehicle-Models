"""Steering-axis lift derivation and its use in the compiled IEM24 model."""

from pathlib import Path
import unittest

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from tools.model_data import derive_steering_lift as lift
from tools.model_data import generate_suspension_lut as gen

ROOT = Path(__file__).resolve().parents[1]
FORMULATION = ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"


def tilted_axis():
    """A steering axis with caster and KPI and an offset wheel centre [m, z down]."""
    lower = np.array([0.10, 0.50, -0.10])
    upper = lower + np.array([-0.03, -0.06, -0.19])
    return lift.SteeringAxis(upper, lower, np.array([0.12, 0.60, -0.20]))


class DerivationTests(unittest.TestCase):
    def test_zero_steer_gives_zero_rise(self):
        self.assertEqual(lift.wheel_centre_rise(tilted_axis(), 0.0), 0.0)

    def test_vertical_axis_through_wheel_centre_has_no_lift(self):
        axis = lift.SteeringAxis([0.0, 0.5, -0.3], [0.0, 0.5, -0.1], [0.0, 0.5, -0.2])
        for yaw in (-0.5, 0.3):
            self.assertAlmostEqual(lift.wheel_centre_rise(axis, yaw), 0.0, places=12)

    def test_vertical_axis_with_offset_wheel_centre_has_no_lift(self):
        axis = lift.SteeringAxis([0.0, 0.5, -0.3], [0.0, 0.5, -0.1], [0.07, 0.6, -0.2])
        self.assertAlmostEqual(lift.wheel_centre_rise(axis, 0.4), 0.0, places=12)

    def test_wheel_centre_stays_on_its_circle_and_reaches_the_requested_yaw(self):
        axis = tilted_axis()
        direction = axis.lower - axis.upper
        for yaw in (-0.5, 0.2, 0.5):
            angle = lift._axis_angle_for_yaw(direction, yaw)
            self.assertAlmostEqual(lift._yaw_of(direction, angle), yaw, places=10)
            moved = axis.lower + lift._rotation(direction, angle) @ (axis.wheel_centre - axis.lower)
            self.assertAlmostEqual(np.linalg.norm(moved - axis.lower),
                                   np.linalg.norm(axis.wheel_centre - axis.lower), places=12)

    def test_fit_matches_the_geometry_and_mirroring_flips_only_the_sine_term(self):
        axis = tilted_axis()
        right = lift.fit_lift(axis, 0.5)
        left = lift.fit_lift(axis.mirrored(), 0.5)
        self.assertLess(right.maximum_residual, 2.0e-4)
        self.assertAlmostEqual(left.sine, -right.sine, places=12)
        self.assertAlmostEqual(left.cosine, right.cosine, places=12)
        self.assertNotAlmostEqual(right.sine, 0.0, places=5)
        for yaw in (-0.4, 0.4):
            expected = right.sine*np.sin(yaw) + right.cosine*(1.0 - np.cos(yaw))
            self.assertAlmostEqual(lift.wheel_centre_rise(axis, yaw), expected, delta=2.0e-4)

    def test_mirrored_corner_rises_at_opposite_steer(self):
        axis = tilted_axis()
        self.assertAlmostEqual(lift.wheel_centre_rise(axis, 0.4),
                               lift.wheel_centre_rise(axis.mirrored(), -0.4), places=12)

    def test_degenerate_axis_is_rejected(self):
        with self.assertRaises(lift.SteeringLiftError):
            lift.SteeringAxis([0, 0, 0], [0, 0, 0], [0, 0, 1])


class GeneratorTests(unittest.TestCase):
    def test_kinematics_xml_emits_optional_lift_only_when_given(self):
        base = gen.kinematics_xml("k", "../d.npz", "0" * 64, 0.0, 0.0)
        self.assertNotIn("steering_lift", base)
        xml = gen.kinematics_xml("k", "../d.npz", "0" * 64, 0.0, 0.0, (0.001, -0.006, -0.001, -0.006))
        for name in gen.STEERING_LIFT_NAMES:
            self.assertIn(f'name="{name}"', xml)
        with self.assertRaises(gen.SuspensionTableError):
            gen.kinematics_xml("k", "../d.npz", "0" * 64, 0.0, 0.0, (float("nan"), 0, 0, 0))


class CompiledModelTests(unittest.TestCase):
    def test_iem24_front_lift_is_small_even_and_mirrored(self):
        kinematics = compile_formulation(str(FORMULATION)).model.suspension.kinematics
        heights, _, _ = kinematics.steering_lift([0.5236, 0.5236, 0.0, 0.0])
        negative, _, _ = kinematics.steering_lift([-0.5236, -0.5236, 0.0, 0.0])
        self.assertLess(float(np.max(np.abs(heights))), 0.0025)
        self.assertEqual(float(heights[2]), 0.0)
        self.assertEqual(float(heights[3]), 0.0)
        self.assertAlmostEqual(float(heights[0]), float(negative[1]), places=12)
        self.assertAlmostEqual(float(heights[1]), float(negative[0]), places=12)
        self.assertNotAlmostEqual(float(heights[0]), float(heights[1]), places=5)


if __name__ == "__main__":
    unittest.main()
