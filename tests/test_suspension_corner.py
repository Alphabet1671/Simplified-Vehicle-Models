import math
import re
import shutil
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from simplified_vehicle_models.compiler.capabilities import (
    SUSPENSION_CORNER,
    SuspensionCornerSource,
)
from simplified_vehicle_models.compiler.registry import component_descriptor
from simplified_vehicle_models.suspension.components import (
    compile_damper,
    compile_suspension_corner,
)
from simplified_vehicle_models.suspension.corner import (
    AxleSuspension,
    SuspensionModel,
    corner_forces,
    corner_toe_angles,
)
from simplified_vehicle_models.suspension.errors import DomainError
from simplified_vehicle_models.suspension.monotone_spline import MonotoneSpline
from simplified_vehicle_models.xml.loader import load_model

ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / "models" / "suspensions" / "iem24_murphy_real_suspension.xml"
FRONT_DAMPER = ROOT / "models" / "dampers" / "multimatic_dssv_vc01_front.xml"
REAR_DAMPER = ROOT / "models" / "dampers" / "multimatic_dssv_vc01_rear.xml"

K_FRONT = 450 * 4.4482216152605 / 0.0254
K_REAR = 350 * 4.4482216152605 / 0.0254
K_ARB = 120 * 4.4482216152605 / 0.0254
FRONT_LOAD = (287 * 0.45 - 26) / 2 * 9.80665
REAR_LOAD = (287 * 0.55 - 26) / 2 * 9.80665


class CornerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = compile_suspension_corner(load_model(XML))
        cls.dampers = (
            compile_damper(load_model(FRONT_DAMPER)),
            compile_damper(load_model(REAR_DAMPER)),
        )

    def forces(self, z, zdot=None, model=None):
        zdot = np.zeros(4) if zdot is None else np.asarray(zdot, dtype=float)
        return corner_forces(model or self.model, np.asarray(z, dtype=float), zdot, self.dampers)

    def test_registration(self):
        descriptor = component_descriptor("suspension-model", "corner-table")
        self.assertEqual(descriptor.capability, SUSPENSION_CORNER)
        self.assertIsInstance(self.model, SuspensionModel)
        self.assertIsInstance(self.model, SuspensionCornerSource)
        self.assertEqual(self.model.preload_reference, "static-equilibrium")

    def test_axle_scalars(self):
        front, rear = self.model.front, self.model.rear
        self.assertEqual(front.arb_wheel_rate, 0.0)
        self.assertAlmostEqual(rear.arb_wheel_rate, 21015.22, delta=0.01)
        self.assertEqual((front.anti_brake, front.anti_drive), (0.542, 0.320))
        self.assertEqual((rear.anti_brake, rear.anti_drive), (0.574, 0.550))
        self.assertEqual((front.static_toe, rear.static_toe), (0.0, 0.0))
        self.assertEqual((front.inertance, rear.inertance), (0.0, 0.0))
        self.assertEqual(front.jounce_limits, (-0.0318, 0.0318))
        self.assertEqual(rear.jounce_limits, (-0.0318, 0.0318))
        self.assertEqual(float(front.motion_ratio_damper(0.01)), 1.05)
        self.assertEqual(float(rear.motion_ratio_damper(-0.02)), 1.10)
        self.assertEqual(front.camber_gain, 25.1)
        self.assertEqual(rear.camber_gain, 45.3)

    def test_roll_centre_maps(self):
        front, rear = self.model.front, self.model.rear
        self.assertAlmostEqual(float(front.roll_centre_height(0.0)), 0.041275)
        self.assertAlmostEqual(float(rear.roll_centre_height(0.0)), 0.05715)
        roll_1g = math.radians(0.289)
        self.assertAlmostEqual(float(front.roll_centre_height(roll_1g)), 0.0455)
        self.assertAlmostEqual(float(rear.roll_centre_height(roll_1g)), 0.0653)
        self.assertAlmostEqual(float(front.roll_centre_lateral(roll_1g)), -0.006)
        self.assertAlmostEqual(float(rear.roll_centre_lateral(roll_1g)), -0.008)
        # Held constant above 1 g.
        self.assertAlmostEqual(float(front.roll_centre_height(2.0 * roll_1g)), 0.0455)

    def test_static_equilibrium_spring_force(self):
        spring = self.forces(np.zeros(4)).spring
        np.testing.assert_allclose(spring, [FRONT_LOAD, FRONT_LOAD, REAR_LOAD, REAR_LOAD], rtol=1e-12)
        self.assertAlmostEqual(spring[0], 506, delta=1)
        self.assertAlmostEqual(spring[2], 647, delta=1)

    def test_spring_slope_at_zero(self):
        for axle, rate in ((self.model.front, K_FRONT), (self.model.rear, K_REAR)):
            slope = float(axle.wheel_rate_spring.derivative(0.0))
            self.assertAlmostEqual(slope / rate, 1.0, delta=1e-6)
        self.assertAlmostEqual(K_FRONT, 78807.08, delta=0.01)
        self.assertAlmostEqual(K_REAR, 61294.39, delta=0.01)

    def test_bump_stop_rate_and_c1(self):
        for axle, rate in ((self.model.front, K_FRONT), (self.model.rear, K_REAR)):
            spring = axle.wheel_rate_spring
            for z in (0.0308, -0.0308):
                self.assertAlmostEqual(float(spring.derivative(z)) / rate, 10.0, delta=1e-3)
            # Rate below the blend is the base rate; the blend rises monotonically.
            self.assertAlmostEqual(float(spring.derivative(0.0270)) / rate, 1.0, delta=1e-6)
            zs = np.linspace(-0.0318, 0.0318, 4001)
            rates = np.array([float(spring.derivative(z)) for z in zs]) / rate
            self.assertTrue(np.all(np.isfinite(rates)))
            self.assertLess(np.max(np.abs(np.diff(rates))), 0.2)  # no jump: C1
            side = np.array([float(spring.derivative(z, "left")) for z in zs[1:-1]])
            np.testing.assert_allclose(side / rate, rates[1:-1], atol=1e-9)
            # Rate rises from 1 to 10. The Hermite slopes ripple by under 0.1 % of the base rate.
            blend = rates[zs > 0.0]
            self.assertTrue(np.all(np.diff(blend) >= -1e-3))
            self.assertLess(np.max(np.abs(rates)), 10.001)
            self.assertGreater(np.min(rates), 0.999)

    def test_domain_failure_beyond_travel(self):
        for z in (0.0319, -0.0319):
            with self.assertRaises(DomainError):
                self.forces([z, 0.0, 0.0, 0.0])
        self.forces([0.0318, -0.0318, 0.0318, -0.0318])  # the limits themselves are valid

    def test_arb(self):
        d = 0.01
        heave = self.forces([d, d, d, d])
        np.testing.assert_array_equal(heave.arb, np.zeros(4))
        roll = self.forces([d, -d, d, -d])
        np.testing.assert_array_equal(roll.arb[:2], [0.0, 0.0])  # front ARB is always 0
        self.assertAlmostEqual(roll.arb[2], 21015.22 * d, delta=1e-2)
        self.assertAlmostEqual(roll.arb[3], -21015.22 * d, delta=1e-2)
        mixed = self.forces([0.02, -0.01, 0.015, -0.005])
        self.assertEqual(mixed.arb[0], 0.0)
        self.assertAlmostEqual(mixed.arb[2], 0.5 * self.model.rear.arb_wheel_rate * 0.02)

    def test_damper_velocity_and_wheel_force(self):
        rate = np.array([-0.1, 0.1, -0.1, 0.1])
        result = self.forces(np.zeros(4), rate)
        np.testing.assert_allclose(result.damper_velocity, rate / np.array([1.05, 1.05, 1.10, 1.10]))
        f_front = self.dampers[0].force(-0.1 / 1.05).force / 1.05
        f_rear = self.dampers[1].force(0.1 / 1.10).force / 1.10
        self.assertAlmostEqual(result.damper[0], float(f_front))
        self.assertAlmostEqual(result.damper[3], float(f_rear))
        self.assertLess(result.damper[0], 0.0)
        self.assertGreater(result.damper[1], 0.0)
        np.testing.assert_allclose(result.total, result.spring + result.arb + result.damper)
        self.assertFalse(np.any(result.out_of_range))
        far = self.forces(np.zeros(4), [2.0, 0.0, 0.0, 0.0])
        self.assertTrue(far.out_of_range[0])

    def test_input_validation(self):
        with self.assertRaises(ValueError):
            corner_forces(self.model, np.zeros(3), np.zeros(4), self.dampers)
        with self.assertRaises(ValueError):
            corner_forces(self.model, np.array([np.nan, 0, 0, 0]), np.zeros(4), self.dampers)


class ToeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = compile_suspension_corner(load_model(XML))

    def with_front_steer(self, knots, static_toe=0.0, rear=None):
        z, toe = zip(*knots)
        front = replace(
            self.model.front,
            jounce_steer=MonotoneSpline(z, toe),
            static_toe=static_toe,
        )
        return SuspensionModel(front=front, rear=rear or self.model.rear)

    def test_zero_baseline_is_four_zeros(self):
        for z in (np.zeros(4), np.array([0.02, -0.03, 0.01, -0.01])):
            np.testing.assert_array_equal(corner_toe_angles(self.model, z), np.zeros(4))

    def test_single_wheel_offset(self):
        half = math.radians(0.5)
        model = self.with_front_steer([(-0.0318, 0.0), (0.0, 0.0), (0.02, half), (0.0318, half)])
        angles = corner_toe_angles(model, np.array([0.02, 0.0, 0.0, 0.0]))
        self.assertAlmostEqual(angles[0], half)
        np.testing.assert_array_equal(angles[1:], np.zeros(3))

    def test_equal_jounce_opposite_signs_and_opposite_jounce_difference(self):
        half = math.radians(0.5)
        model = self.with_front_steer([(-0.0318, -half), (0.0, 0.0), (0.0318, half)])
        equal = corner_toe_angles(model, np.array([0.02, 0.02, 0.0, 0.0]))
        self.assertAlmostEqual(equal[0], -equal[1])
        self.assertGreater(equal[0], 0.0)
        opposite = corner_toe_angles(model, np.array([0.02, -0.02, 0.0, 0.0]))
        # Opposite jounce gives equal-signed angles on both wheels: a net steer, not a toe change.
        self.assertAlmostEqual(opposite[0], opposite[1])
        self.assertGreater(abs(opposite[0]), 0.0)
        self.assertAlmostEqual(opposite[0], float(model.front.jounce_steer(0.02)))
        self.assertAlmostEqual(opposite[0] + opposite[1], 2 * float(model.front.jounce_steer(0.02)))

    def test_knots_match_signed_curve_both_sides(self):
        knots = [(-0.0318, -0.002), (-0.01, -0.001), (0.0, 0.0), (0.015, 0.0007), (0.0318, 0.003)]
        model = self.with_front_steer(knots)
        for z, toe in knots:
            left = corner_toe_angles(model, np.array([z, 0.0, 0.0, 0.0]))[0]
            right = corner_toe_angles(model, np.array([0.0, z, 0.0, 0.0]))[1]
            self.assertEqual(left, toe)
            self.assertEqual(right, -toe)

    def test_static_toe_sign_per_side(self):
        toe = math.radians(0.25)
        model = self.with_front_steer([(-0.0318, 0.0), (0.0, 0.0), (0.0318, 0.0)], static_toe=toe)
        angles = corner_toe_angles(model, np.zeros(4))
        np.testing.assert_array_equal(angles, [toe, -toe, 0.0, 0.0])

    def test_out_of_travel_fails(self):
        model = self.with_front_steer([(-0.0318, 0.0), (0.0, 0.0), (0.0318, 0.001)])
        with self.assertRaises(DomainError):
            corner_toe_angles(model, np.array([0.0319, 0.0, 0.0, 0.0]))


class AxleValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.axle = compile_suspension_corner(load_model(XML)).front

    def test_rejects_bad_axles(self):
        with self.assertRaises(ValueError):
            replace(self.axle, jounce_limits=(0.01, 0.02))
        with self.assertRaises(ValueError):
            replace(self.axle, arb_wheel_rate=-1.0)
        with self.assertRaises(ValueError):
            replace(self.axle, anti_brake=1.5)
        with self.assertRaises(ValueError):  # no explicit 0 mm knot
            replace(self.axle, jounce_steer=MonotoneSpline([-0.0318, -0.01, 0.01, 0.0318], [0, 0, 0, 0]))
        with self.assertRaises(ValueError):  # nonzero reference
            replace(self.axle, jounce_steer=MonotoneSpline([-0.0318, 0.0, 0.0318], [0, 0.001, 0]))
        with self.assertRaises(ValueError):  # does not reach the travel limit
            replace(self.axle, jounce_steer=MonotoneSpline([-0.0318, 0.0, 0.02], [0, 0, 0]))
        self.assertIsInstance(self.axle, AxleSuspension)


class ComponentFileTests(unittest.TestCase):
    def copy(self, directory, edit=None):
        directory = Path(directory)
        (directory / "data").mkdir()
        (directory / "suspensions").mkdir()
        shutil.copy(ROOT / "models" / "data" / "iem24_murphy_real_corners.npz", directory / "data")
        text = XML.read_text()
        if edit:
            text = edit(text)
        target = directory / "suspensions" / XML.name
        target.write_text(text)
        return target

    def test_xml_contents(self):
        text = XML.read_text()
        digest = re.search(r'sha256="([0-9a-f]{64})"', text).group(1)
        self.assertEqual(len(digest), 64)
        for name in (
            "arb_wheel_rate_front", "arb_wheel_rate_rear", "inertance_front",
            "inertance_rear", "static_toe_front", "static_toe_rear",
        ):
            self.assertIn(f'name="{name}"', text)
        self.assertIn("baseline, no measured jounce-steer data", text)
        self.assertIn('role="corner-table"', text)

    def test_npz_keys_and_metadata(self):
        with np.load(ROOT / "models/data/iem24_murphy_real_corners.npz", allow_pickle=False) as data:
            for key in (
                "jounce_steer_jounce_front", "jounce_steer_jounce_rear",
                "jounce_steer_toe_front", "jounce_steer_toe_rear",
            ):
                self.assertIn(key, data.files)
            np.testing.assert_array_equal(data["jounce_steer_jounce_front"], [-0.0318, 0.0, 0.0318])
            np.testing.assert_array_equal(data["jounce_steer_toe_rear"], np.zeros(3))
            self.assertIn("baseline, no measured jounce-steer data", str(data["metadata"]))
            self.assertIn("held constant above", str(data["metadata"]))

    def test_modified_sha_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            target = self.copy(directory, lambda t: re.sub(r'sha256="[0-9a-f]{64}"', 'sha256="' + "0" * 64 + '"', t))
            with self.assertRaisesRegex(ValueError, "sha256 mismatch"):
                compile_suspension_corner(load_model(target))

    def test_missing_or_unknown_parameter_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            target = self.copy(directory, lambda t: re.sub(r'\s*<parameter name="inertance_rear"[^>]*/>', "", t))
            with self.assertRaisesRegex(ValueError, "missing"):
                compile_suspension_corner(load_model(target))
        with tempfile.TemporaryDirectory() as directory:
            target = self.copy(directory, lambda t: t.replace("</parameters>", '<parameter name="extra" value="1" unit="1"/></parameters>'))
            with self.assertRaisesRegex(ValueError, "unknown"):
                compile_suspension_corner(load_model(target))

    def test_negative_arb_and_wrong_unit_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            target = self.copy(directory, lambda t: t.replace('name="arb_wheel_rate_front" value="0.0"', 'name="arb_wheel_rate_front" value="-1.0"'))
            with self.assertRaisesRegex(ValueError, "nonnegative"):
                compile_suspension_corner(load_model(target))
        with tempfile.TemporaryDirectory() as directory:
            target = self.copy(directory, lambda t: t.replace('unit="rad"', 'unit="deg"', 1))
            with self.assertRaisesRegex(ValueError, "unit"):
                compile_suspension_corner(load_model(target))


if __name__ == "__main__":
    unittest.main()
