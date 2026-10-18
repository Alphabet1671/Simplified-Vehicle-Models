import math
import re
import shutil
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from simplified_vehicle_models.compiler.capabilities import (
    SUSPENSION_FORCES,
    SuspensionSource,
)
from simplified_vehicle_models.compiler.registry import component_descriptor
from simplified_vehicle_models.suspension.components import (
    compile_axle_arb,
    compile_corner_springs,
    compile_damper,
    compile_suspension_kinematics,
)
from simplified_vehicle_models.suspension.elements import AxleArbRates, CornerSprings
from simplified_vehicle_models.suspension.errors import DomainError
from simplified_vehicle_models.suspension.kinematics import AxleGeometry, SuspensionKinematics
from simplified_vehicle_models.suspension.layouts.independent_corner import build_independent_corner
from simplified_vehicle_models.suspension.monotone_spline import MonotoneSpline
from simplified_vehicle_models.suspension.network import CompiledSuspension
from simplified_vehicle_models.xml.loader import load_model

ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / "models" / "suspensions" / "iem24_murphy_real_suspension.xml"
KINEMATICS_XML = ROOT / "models" / "suspension-kinematics" / "iem24_murphy_real_suspension_kinematics.xml"
SPRING_XML = ROOT / "models" / "springs" / "iem24_murphy_real_suspension_springs.xml"
ARB_XML = ROOT / "models" / "anti-roll-bars" / "iem24_murphy_real_suspension_arb.xml"
FRONT_DAMPER = ROOT / "models" / "dampers" / "multimatic_dssv_vc01_front.xml"
REAR_DAMPER = ROOT / "models" / "dampers" / "multimatic_dssv_vc01_rear.xml"

K_FRONT = 450 * 4.4482216152605 / 0.0254
K_REAR = 350 * 4.4482216152605 / 0.0254
K_ARB = 120 * 4.4482216152605 / 0.0254
FRONT_LOAD = (287 * 0.45 - 26) / 2 * 9.80665
REAR_LOAD = (287 * 0.55 - 26) / 2 * 9.80665


def build(inertance=(0.0, 0.0)):
    dampers = (compile_damper(load_model(FRONT_DAMPER)), compile_damper(load_model(REAR_DAMPER)))
    return build_independent_corner(
        kinematics=compile_suspension_kinematics(load_model(KINEMATICS_XML)),
        springs=compile_corner_springs(load_model(SPRING_XML)),
        arb=compile_axle_arb(load_model(ARB_XML)),
        damper_front=dampers[0], damper_rear=dampers[1],
        inertance_front=inertance[0], inertance_rear=inertance[1],
    )


class CornerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.suspension = build()
        cls.model = cls.suspension.kinematics
        cls.network = cls.suspension.network
        cls.dampers = cls.network.dampers

    def forces(self, z, zdot=None):
        zdot = np.zeros(4) if zdot is None else np.asarray(zdot, dtype=float)
        return self.network.forces(np.asarray(z, dtype=float), zdot)

    def test_registration(self):
        descriptor = component_descriptor("suspension-assembly", "independent-corner")
        self.assertEqual(descriptor.capability, SUSPENSION_FORCES)
        self.assertTrue(descriptor.nests_components)
        self.assertIsInstance(self.suspension, CompiledSuspension)
        self.assertIsInstance(self.suspension, SuspensionSource)
        self.assertIsInstance(self.model, SuspensionKinematics)
        self.assertEqual(self.model.preload_reference, "static-equilibrium")
        for kind in ("suspension-kinematics", "spring-model", "arb-model"):
            self.assertIsNone(component_descriptor(kind, "corner-table" if kind != "arb-model" else "axle-rate").capability)

    def test_axle_scalars(self):
        front, rear = self.model.front, self.model.rear
        self.assertEqual(self.network.arb.front, 0.0)
        self.assertAlmostEqual(self.network.arb.rear, 21015.22, delta=0.01)
        self.assertEqual((front.anti_brake, front.anti_drive), (0.542, 0.320))
        self.assertEqual((rear.anti_brake, rear.anti_drive), (0.574, 0.550))
        self.assertEqual((front.static_toe, rear.static_toe), (0.0, 0.0))
        self.assertEqual(self.network.inertance, (0.0, 0.0))
        self.assertEqual(tuple(front.jounce_limits), (-0.0318, 0.0318))
        self.assertEqual(tuple(rear.jounce_limits), (-0.0318, 0.0318))
        self.assertEqual(self.model.motion_ratio(0, 0.01), 1.05)
        self.assertEqual(self.model.motion_ratio(2, -0.02), 1.10)
        self.assertEqual(front.camber_gain, 25.1)
        self.assertEqual(rear.camber_gain, 45.3)

    def test_roll_centre_maps(self):
        k = self.model
        self.assertAlmostEqual(k.roll_centre_height(0, 0.0), 0.041275)
        self.assertAlmostEqual(k.roll_centre_height(1, 0.0), 0.05715)
        roll_1g = math.radians(0.289)
        self.assertAlmostEqual(k.roll_centre_height(0, roll_1g), 0.0455)
        self.assertAlmostEqual(k.roll_centre_height(1, roll_1g), 0.0653)
        self.assertAlmostEqual(k.roll_centre_lateral(0, roll_1g), -0.006)
        self.assertAlmostEqual(k.roll_centre_lateral(1, roll_1g), -0.008)
        # Held constant above 1 g.
        self.assertAlmostEqual(k.roll_centre_height(0, 2.0 * roll_1g), 0.0455)

    def test_static_equilibrium_spring_force(self):
        spring = self.forces(np.zeros(4)).elements["spring"]
        np.testing.assert_allclose(spring, [FRONT_LOAD, FRONT_LOAD, REAR_LOAD, REAR_LOAD], rtol=1e-12)
        self.assertAlmostEqual(spring[0], 506, delta=1)
        self.assertAlmostEqual(spring[2], 647, delta=1)

    def test_spring_slope_at_zero(self):
        for spring, rate in ((self.network.springs.front, K_FRONT), (self.network.springs.rear, K_REAR)):
            slope = float(spring.derivative(0.0))
            self.assertAlmostEqual(slope / rate, 1.0, delta=1e-6)
        self.assertAlmostEqual(K_FRONT, 78807.08, delta=0.01)
        self.assertAlmostEqual(K_REAR, 61294.39, delta=0.01)

    def test_bump_stop_rate_and_c1(self):
        for spring, rate in ((self.network.springs.front, K_FRONT), (self.network.springs.rear, K_REAR)):
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
        np.testing.assert_array_equal(heave.elements["arb"], np.zeros(4))
        roll = self.forces([d, -d, d, -d]).elements["arb"]
        np.testing.assert_array_equal(roll[:2], [0.0, 0.0])  # front ARB is always 0
        self.assertAlmostEqual(roll[2], 21015.22 * d, delta=1e-2)
        self.assertAlmostEqual(roll[3], -21015.22 * d, delta=1e-2)
        mixed = self.forces([0.02, -0.01, 0.015, -0.005]).elements["arb"]
        self.assertEqual(mixed[0], 0.0)
        self.assertAlmostEqual(mixed[2], 0.5 * self.network.arb.rear * 0.02)

    def test_damper_velocity_and_wheel_force(self):
        rate = np.array([-0.1, 0.1, -0.1, 0.1])
        result = self.forces(np.zeros(4), rate)
        damper = result.elements["damper"]
        np.testing.assert_allclose(result.damper_velocity, rate / np.array([1.05, 1.05, 1.10, 1.10]))
        f_front = self.dampers[0].force(-0.1 / 1.05).force / 1.05
        f_rear = self.dampers[1].force(0.1 / 1.10).force / 1.10
        self.assertAlmostEqual(damper[0], float(f_front))
        self.assertAlmostEqual(damper[3], float(f_rear))
        self.assertLess(damper[0], 0.0)
        self.assertGreater(damper[1], 0.0)
        np.testing.assert_allclose(
            result.total, result.elements["spring"] + result.elements["arb"] + damper)
        self.assertFalse(np.any(result.out_of_range))
        far = self.forces(np.zeros(4), [2.0, 0.0, 0.0, 0.0])
        self.assertTrue(far.out_of_range[0])

    def test_input_validation(self):
        with self.assertRaises(ValueError):
            self.network.forces(np.zeros(3), np.zeros(4))
        with self.assertRaises(ValueError):
            self.network.forces(np.array([np.nan, 0, 0, 0]), np.zeros(4))


class ToeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = compile_suspension_kinematics(load_model(KINEMATICS_XML))

    def with_front_steer(self, knots, static_toe=0.0):
        z, toe = zip(*knots)
        return self.model.with_axle(0, jounce_steer=MonotoneSpline(z, toe), static_toe=static_toe)

    def test_zero_baseline_is_four_zeros(self):
        for z in (np.zeros(4), np.array([0.02, -0.03, 0.01, -0.01])):
            np.testing.assert_array_equal(self.model.toe(z), np.zeros(4))

    def test_single_wheel_offset(self):
        half = math.radians(0.5)
        model = self.with_front_steer([(-0.0318, 0.0), (0.0, 0.0), (0.02, half), (0.0318, half)])
        angles = model.toe(np.array([0.02, 0.0, 0.0, 0.0]))
        self.assertAlmostEqual(angles[0], half)
        np.testing.assert_array_equal(angles[1:], np.zeros(3))

    def test_equal_jounce_opposite_signs_and_opposite_jounce_difference(self):
        half = math.radians(0.5)
        model = self.with_front_steer([(-0.0318, -half), (0.0, 0.0), (0.0318, half)])
        equal = model.toe(np.array([0.02, 0.02, 0.0, 0.0]))
        self.assertAlmostEqual(equal[0], -equal[1])
        self.assertGreater(equal[0], 0.0)
        opposite = model.toe(np.array([0.02, -0.02, 0.0, 0.0]))
        # Opposite jounce gives equal-signed angles on both wheels: a net steer, not a toe change.
        self.assertAlmostEqual(opposite[0], opposite[1])
        self.assertGreater(abs(opposite[0]), 0.0)
        steer = model.front.jounce_steer
        self.assertAlmostEqual(opposite[0], float(steer(0.02)))
        self.assertAlmostEqual(opposite[0] + opposite[1], 2 * float(steer(0.02)))

    def test_knots_match_signed_curve_both_sides(self):
        knots = [(-0.0318, -0.002), (-0.01, -0.001), (0.0, 0.0), (0.015, 0.0007), (0.0318, 0.003)]
        model = self.with_front_steer(knots)
        for z, toe in knots:
            left = model.toe(np.array([z, 0.0, 0.0, 0.0]))[0]
            right = model.toe(np.array([0.0, z, 0.0, 0.0]))[1]
            self.assertEqual(left, toe)
            self.assertEqual(right, -toe)

    def test_static_toe_sign_per_side(self):
        toe = math.radians(0.25)
        model = self.with_front_steer([(-0.0318, 0.0), (0.0, 0.0), (0.0318, 0.0)], static_toe=toe)
        np.testing.assert_array_equal(model.toe(np.zeros(4)), [toe, -toe, 0.0, 0.0])

    def test_out_of_travel_fails(self):
        model = self.with_front_steer([(-0.0318, 0.0), (0.0, 0.0), (0.0318, 0.001)])
        with self.assertRaises(DomainError):
            model.toe(np.array([0.0319, 0.0, 0.0, 0.0]))


class AxleValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.axle = compile_suspension_kinematics(load_model(KINEMATICS_XML)).front

    def test_rejects_bad_axles(self):
        with self.assertRaises(ValueError):
            replace(self.axle, jounce_limits=(0.01, 0.02))
        with self.assertRaises(ValueError):
            replace(self.axle, anti_brake=1.5)
        with self.assertRaises(ValueError):
            replace(self.axle, static_sprung_load=0.0)
        with self.assertRaises(ValueError):  # no explicit 0 mm knot
            replace(self.axle, jounce_steer=MonotoneSpline([-0.0318, -0.01, 0.01, 0.0318], [0, 0, 0, 0]))
        with self.assertRaises(ValueError):  # nonzero reference
            replace(self.axle, jounce_steer=MonotoneSpline([-0.0318, 0.0, 0.0318], [0, 0.001, 0]))
        with self.assertRaises(ValueError):  # does not reach the travel limit
            replace(self.axle, jounce_steer=MonotoneSpline([-0.0318, 0.0, 0.02], [0, 0, 0]))
        self.assertIsInstance(self.axle, AxleGeometry)
        with self.assertRaises(ValueError):
            AxleArbRates(front=-1.0, rear=0.0)


class ComponentFileTests(unittest.TestCase):
    """Edit copies of the four suspension component files in a scratch ``models`` tree."""

    def copy(self, directory, source, edit=None):
        directory = Path(directory).resolve()
        (directory / "data").mkdir(exist_ok=True)
        (directory / source.parent.name).mkdir(exist_ok=True)
        shutil.copy(ROOT / "models" / "data" / "iem24_murphy_real_corners.npz", directory / "data")
        text = source.read_text()
        if edit:
            text = edit(text)
        target = directory / source.parent.name / source.name
        target.write_text(text)
        return target

    def test_xml_contents(self):
        text = KINEMATICS_XML.read_text()
        digest = re.search(r'sha256="([0-9a-f]{64})"', text).group(1)
        self.assertEqual(digest, "04bb06c93b7f52de2e9bc4fcd4e934532d89a9442e75e79ab5886afadb38f0f0")
        self.assertEqual(re.search(r'sha256="([0-9a-f]{64})"', SPRING_XML.read_text()).group(1), digest)
        for name in ("static_toe_front", "static_toe_rear"):
            self.assertIn(f'name="{name}"', text)
        for name in ("arb_wheel_rate_front", "arb_wheel_rate_rear"):
            self.assertIn(f'name="{name}"', ARB_XML.read_text())
        for name in ("inertance_front", "inertance_rear"):
            self.assertIn(f'name="{name}"', XML.read_text())
        self.assertIn("baseline, no measured jounce-steer data", text)
        self.assertIn('role="corner-table"', text)

    def test_assembly_references_the_five_children(self):
        text = XML.read_text()
        for role in ("kinematics", "spring", "arb", "damper-front", "damper-rear"):
            self.assertIn(f'<component role="{role}"', text)

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
        for source, compile_function in ((KINEMATICS_XML, compile_suspension_kinematics),
                                         (SPRING_XML, compile_corner_springs)):
            with tempfile.TemporaryDirectory() as directory:
                target = self.copy(directory, source,
                                   lambda t: re.sub(r'sha256="[0-9a-f]{64}"', 'sha256="' + "0" * 64 + '"', t))
                with self.assertRaisesRegex(ValueError, "sha256 mismatch"):
                    compile_function(load_model(target))

    def test_missing_or_unknown_parameter_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            target = self.copy(directory, KINEMATICS_XML,
                               lambda t: re.sub(r'\s*<parameter name="static_toe_rear"[^>]*/>', "", t))
            with self.assertRaisesRegex(ValueError, "missing"):
                compile_suspension_kinematics(load_model(target))
        with tempfile.TemporaryDirectory() as directory:
            target = self.copy(directory, KINEMATICS_XML, lambda t: t.replace(
                "</parameters>", '<parameter name="extra" value="1" unit="1"/></parameters>'))
            with self.assertRaisesRegex(ValueError, "unknown"):
                compile_suspension_kinematics(load_model(target))

    def test_negative_arb_and_wrong_unit_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            target = self.copy(directory, ARB_XML, lambda t: t.replace(
                'name="arb_wheel_rate_front" value="0.0"', 'name="arb_wheel_rate_front" value="-1.0"'))
            with self.assertRaisesRegex(ValueError, "nonnegative"):
                compile_axle_arb(load_model(target))
        with tempfile.TemporaryDirectory() as directory:
            target = self.copy(directory, KINEMATICS_XML, lambda t: t.replace('unit="rad"', 'unit="deg"', 1))
            with self.assertRaisesRegex(ValueError, "unit"):
                compile_suspension_kinematics(load_model(target))


if __name__ == "__main__":
    unittest.main()
