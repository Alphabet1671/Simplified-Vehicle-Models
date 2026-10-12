import math
import unittest
from dataclasses import replace

from simplified_vehicle_models.powertrain.power_plant import (
    calculate_available_voltage,
    compile_electric_motor,
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
        reduced = evaluate_shaft_power(parameters, 10000.0, 10.0, 450.0)
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


if __name__ == "__main__":
    unittest.main()
