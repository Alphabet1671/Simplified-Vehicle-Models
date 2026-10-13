"""Explicit 800 V test model: validates physics/LUTs, not hardware ratings."""
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

import numpy as np

from tools.generate_motor_lut import generate
from simplified_vehicle_models.powertrain.power_plant import (
    compile_electric_motor, evaluate_motor, evaluate_shaft_power_exact,
    motor_torque_limits, torque_for_bus_power,
)
from simplified_vehicle_models.xml.loader import load_model
from simplified_vehicle_models.xml.model import DataReference

ROOT = Path(__file__).resolve().parents[1]


class HighVoltageOperatingMapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.folder = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.folder.cleanup)
        path = Path(cls.folder.name)
        xml = path / "motor-800v.xml"
        source = (ROOT / "models/power-plants/amk_dd5_100c.xml").read_text()
        xml.write_text(source.replace(
            'name="maximum_bus_voltage" value="600"',
            'name="maximum_bus_voltage" value="800"',
        ))
        artifact = path / "map-800v.npz"
        digest = generate(artifact, voltage_points=25, speed_points=129,
                          load_points=65, xml_path=xml,
                          voltage_min=600.0, voltage_max=800.0)
        model = load_model(xml)
        model = replace(model, data=(DataReference("operating-map", artifact, "npz", digest),))
        cls.motor = compile_electric_motor(model)

    def test_high_voltage_nodes_match_analytical_source_in_both_directions(self):
        p = self.motor
        mapping = p.operating_map
        for vi in (0, 6, 24):
            for si in (0, 1, 32, 80, 128):
                rpm, voltage = float(mapping.speed[si]), float(mapping.voltage[vi])
                for load in (-0.75, -0.25, 0.0, 0.25, 0.75):
                    limit = (mapping.regen_torque if load < 0 else mapping.drive_torque)[vi, si]
                    torque = float(load * limit)
                    exact = evaluate_shaft_power_exact(p, rpm, torque, voltage)
                    lut = evaluate_motor(p, rpm, torque, voltage)
                    self.assertTrue(exact.feasible)
                    self.assertTrue(lut.feasible)
                    self.assertAlmostEqual(lut.bus_power, exact.bus_power, delta=1e-7)
                    self.assertAlmostEqual(lut.loss_power, exact.loss_power, delta=1e-7)
                    self.assertGreaterEqual(lut.loss_power, 0.0)

    def test_drive_and_regen_inverse_respect_signed_power_constraints(self):
        p = self.motor
        for voltage in (650.0, 725.0, 800.0):
            for rpm in (5000.0, 10000.0, 16000.0):
                limits = motor_torque_limits(p, rpm, voltage)
                target = evaluate_motor(p, rpm, 0.5 * limits.maximum_drive_torque, voltage).bus_power
                drive = torque_for_bus_power(p, rpm, voltage, target, "drive")
                self.assertTrue(drive.feasible)
                self.assertLessEqual(drive.operating_point.bus_power, target + 1e-3)
                self.assertLessEqual(drive.torque, limits.maximum_drive_torque + 1e-6)
                regen = torque_for_bus_power(p, rpm, voltage, -1000.0, "regen")
                self.assertTrue(regen.feasible)
                self.assertLessEqual(regen.torque, 0.0)
                self.assertGreaterEqual(regen.operating_point.bus_power, -1000.0 - 1e-3)

    def test_no_fake_low_speed_envelopes_and_productive_mask_is_exact(self):
        mapping = self.motor.operating_map
        self.assertGreater(float(np.min(mapping.drive_torque[:, :3])), 20.0)
        zero = mapping.load.size // 2
        np.testing.assert_array_equal(
            mapping.regen_feasible[:, :, :zero],
            mapping.bus_power[:, :, :zero] < -1e-9,
        )
        for voltage in (650.0, 800.0):
            for rpm in (0.0, 145.0, 1000.0):
                result = evaluate_motor(self.motor, rpm, 0.0, voltage)
                # Coarse speed interpolation adds a small loss error (< 1 W).
                self.assertLess(result.bus_power, 50.0)
                self.assertTrue(result.feasible)
