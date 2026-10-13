import math
import unittest
from dataclasses import replace

from simplified_vehicle_models.powertrain.power_plant import (
    calculate_available_voltage,
    compile_electric_motor,
    compile_analytical_motor_source,
    evaluate_shaft_power,
    maximum_motor_torque,
    scalar_efficiency_bus_power,
    solve_direct_current,
    _loss_interpolate,
)
from simplified_vehicle_models.xml.loader import load_model


class PowerPlantTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parameters = compile_electric_motor(
            load_model("models/power-plants/amk_dd5_100c.xml")
        )

    def test_zero_speed_and_limit(self):
        parameters = self.parameters
        self.assertAlmostEqual(
            maximum_motor_torque(parameters, 0.0, 600.0 / math.sqrt(3.0)),
            parameters.torque_values[0],
        )
        self.assertEqual(parameters.maximum_motor_speed, 18600.0)
        with self.assertRaises(ValueError):
            maximum_motor_torque(parameters, 18601.0, 300.0)

    def test_voltage_scaling_and_map_separation(self):
        parameters = self.parameters
        low_voltage = maximum_motor_torque(parameters, 10000.0, 200.0)
        high_voltage = maximum_motor_torque(parameters, 10000.0, 300.0)
        self.assertGreater(high_voltage, low_voltage)
        self.assertAlmostEqual(
            high_voltage,
            maximum_motor_torque(parameters, 10000.0 * 200.0 / 300.0, 200.0),
            delta=1.0e-9,
        )
        parameters_with_changed_fit = replace(parameters, flux_linkage=0.5)
        self.assertEqual(
            maximum_motor_torque(parameters, 10000.0, 300.0),
            maximum_motor_torque(parameters_with_changed_fit, 10000.0, 300.0),
        )

    def test_reference_basis_uses_measured_maximum_current_row(self):
        parameters = self.parameters
        line_line_peak = math.hypot(
            parameters.measured_reference_direct_voltage,
            parameters.measured_reference_quadrature_voltage,
        )
        phase_peak = line_line_peak / math.sqrt(3.0)
        reconstructed_bus = math.sqrt(3.0) * phase_peak
        self.assertAlmostEqual(reconstructed_bus, 600.0, delta=600.0 * 0.05)
        self.assertAlmostEqual(parameters.measured_reference_speed, 16300.0)
        self.assertAlmostEqual(
            parameters.measured_reference_direct_current,
            -18.0643575140,
        )
        self.assertAlmostEqual(
            parameters.measured_reference_quadrature_current,
            83.8073922015,
        )
        self.assertAlmostEqual(
            parameters.measured_reference_direct_voltage,
            -566.8960754228,
        )
        self.assertAlmostEqual(
            parameters.measured_reference_quadrature_voltage,
            196.5422083675,
        )
        self.assertAlmostEqual(line_line_peak, 600.0, delta=600.0 * 0.05)

    def test_available_voltage_uses_current_drop(self):
        parameters = self.parameters
        no_current = calculate_available_voltage(parameters, 600.0, 0.0)
        with_current = calculate_available_voltage(parameters, 600.0, 50.0)
        self.assertLess(with_current, no_current)

    def test_reduced_voltage_field_weakening_and_correction(self):
        parameters = self.parameters
        result = evaluate_shaft_power(parameters, 10000.0, 10.0, 600.0)
        # 450 V still has headroom at this point; 300 V genuinely binds.
        reduced = evaluate_shaft_power(parameters, 10000.0, 10.0, 300.0)
        self.assertTrue(result.feasible)
        self.assertTrue(reduced.feasible)
        self.assertEqual(result.field_weakening_correction, 0.0)
        self.assertNotEqual(reduced.field_weakening_correction, 0.0)

    def test_efficiency_and_unscaled_loss_map(self):
        parameters = self.parameters
        for speed in (0.0, 1000.0, 10000.0, 18000.0):
            for torque in (-10.0, 0.0, 10.0):
                result = evaluate_shaft_power(parameters, speed, torque, 600.0)
                self.assertTrue(result.feasible)
                self.assertGreaterEqual(result.efficiency, 0.0)
                self.assertLessEqual(result.efficiency, 1.0)
        baseline = evaluate_shaft_power(parameters, 1000.0, 10.0, 600.0)
        self.assertGreater(baseline.loss_power, 0.0)
        self.assertLess(baseline.loss_power, 1000.0)

    def test_infeasible_voltage_is_reported(self):
        parameters = self.parameters
        direct = solve_direct_current(parameters, 16300.0, 1000.0, 1.0)
        self.assertFalse(direct.feasible)
        result = evaluate_shaft_power(parameters, 16300.0, 1000.0, 1.0)
        self.assertFalse(result.feasible)
        self.assertEqual(result.active_limit, "voltage")

    def test_regen_and_scalar_reduction(self):
        parameters = self.parameters
        regen = evaluate_shaft_power(parameters, 1000.0, -10.0, 600.0)
        self.assertLess(
            abs(regen.bus_power),
            abs(1000.0 * 2.0 * math.pi / 60.0 * -10.0),
        )
        self.assertAlmostEqual(scalar_efficiency_bus_power(1000.0, 0.95), 1000.0 / 0.95)
        self.assertAlmostEqual(scalar_efficiency_bus_power(-1000.0, 0.95), -950.0)
        with self.assertRaises(ValueError):
            scalar_efficiency_bus_power(100.0, 0.0)

    def test_loss_map_clamps_torque_and_rejects_speed(self):
        parameters = self.parameters
        low = _loss_interpolate(parameters, 1000.0, -1000.0)
        high = _loss_interpolate(parameters, 1000.0, 1000.0)
        low_edge = _loss_interpolate(parameters, 1000.0, min(parameters.loss_torque))
        high_edge = _loss_interpolate(parameters, 1000.0, max(parameters.loss_torque))
        self.assertEqual(low, low_edge)
        self.assertEqual(high, high_edge)
        with self.assertRaises(ValueError):
            evaluate_shaft_power(parameters, 21000.0, 1.0, 600.0)


class VoltageInequalityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.motor = compile_analytical_motor_source(
            load_model("models/power-plants/amk_dd5_100c.xml")
        )

    def test_headroom_needs_no_direct_current(self):
        for rpm in (0.0, 145.0, 1000.0, 10000.0):
            for iq in (-10.0, 0.0, 10.0):
                result = solve_direct_current(self.motor, rpm, iq, 800.0)
                self.assertTrue(result.feasible)
                self.assertEqual(result.direct_current, 0.0)

    def test_negative_available_voltage_is_infeasible(self):
        result = solve_direct_current(self.motor, 0.0, 0.0, -1.0)
        self.assertFalse(result.feasible)

    def test_signed_current_grid_satisfies_voltage_inequality(self):
        p = self.motor
        binding = 0
        for rpm in (0.0, 145.0, 1000.0, 10000.0, 18000.0):
            w = p.pole_pairs * 2.0 * math.pi * rpm / 60.0
            for iq in (-50.0, 0.0, 50.0):
                for voltage in (50.0, 200.0, 350.0, 500.0):
                    result = solve_direct_current(p, rpm, iq, voltage)
                    if not result.feasible:
                        continue
                    id_ = result.direct_current
                    vd = p.resistance * id_ - w * p.phase_quadrature_inductance * iq
                    vq = p.resistance * iq + w * (p.phase_flux_linkage + p.phase_direct_inductance * id_)
                    self.assertLessEqual(vd * vd + vq * vq, voltage * voltage + 1e-7)
                    if id_ != 0.0:
                        binding += 1
                        self.assertAlmostEqual(vd * vd + vq * vq, voltage * voltage, delta=1e-7)
        self.assertGreater(binding, 0)

    def test_field_weakening_onset_is_continuous(self):
        p = self.motor
        w = p.pole_pairs * 2.0 * math.pi * 18000.0 / 60.0
        boundary = w * p.phase_flux_linkage
        headroom = solve_direct_current(p, 18000.0, 0.0, boundary + 1e-6)
        weak = solve_direct_current(p, 18000.0, 0.0, boundary - 1e-6)
        self.assertEqual(headroom.direct_current, 0.0)
        self.assertTrue(weak.feasible)
        self.assertLess(weak.direct_current, 0.0)
        self.assertLess(abs(weak.direct_current), 1e-4)

    def test_high_voltage_retains_map_loss_without_fake_penalty(self):
        p = replace(self.motor, maximum_bus_voltage=800.0)
        for voltage in (600.0, 650.0, 800.0):
            for rpm in (0.0, 145.0, 1000.0, 18000.0):
                result = evaluate_shaft_power(p, rpm, 0.0, voltage)
                self.assertTrue(result.feasible)
                self.assertEqual(result.field_weakening_correction, 0.0)
                self.assertAlmostEqual(result.loss_power, _loss_interpolate(p, max(rpm, p.torque_speed[0]), 0.0))
                if rpm <= 1000.0:
                    self.assertLess(result.bus_power, 50.0)

    def test_high_voltage_feasible_point_does_not_need_feasible_reference(self):
        p = replace(self.motor, maximum_bus_voltage=800.0)
        iq = -20.0 / (1.5 * p.pole_pairs * p.phase_flux_linkage)
        reference = solve_direct_current(p, 15000.0, iq, calculate_available_voltage(p, 600.0, iq))
        self.assertFalse(reference.feasible)
        result = evaluate_shaft_power(p, 15000.0, -20.0, 800.0)
        self.assertTrue(result.feasible)
        self.assertEqual(result.field_weakening_correction, 0.0)

    def test_reference_loss_baseline_is_not_regulatory_clamped(self):
        p = replace(self.motor, maximum_bus_voltage=300.0)
        independent = replace(self.motor, maximum_bus_voltage=800.0)
        limited = evaluate_shaft_power(p, 18000.0, 0.0, 600.0)
        actual = evaluate_shaft_power(independent, 18000.0, 0.0, 300.0)
        self.assertTrue(limited.feasible)
        self.assertEqual(limited, actual)
        self.assertGreater(limited.field_weakening_correction, 0.0)


if __name__ == "__main__":
    unittest.main()
