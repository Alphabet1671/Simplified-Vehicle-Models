import unittest
from types import SimpleNamespace

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.controls.brake_inverse import invert_brake_effort
from simplified_vehicle_models.solvers.forward_driver import DriverCommand
from simplified_vehicle_models.solvers.forward_powertrain import evaluate_forward_powertrain
from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant


class BrakeInverseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation("models/formulations/iem24_murphy_real_planar_7dof.xml").model
        cls.plant = PlanarForwardPlant(cls.vehicle, SimpleNamespace(is_closed=False))

    def forward(self, force):
        v = self.vehicle
        return evaluate_forward_powertrain(v, self.plant.driven, self.plant.gearing,
            self.plant.gear_efficiency, np.full(4, 30.), v.storage.initial_state_of_charge,
            DriverCommand(0., force, 0.)).wheel_torques

    def test_actual_powertrain_roundtrip_force_grid(self):
        v = self.vehicle
        self.assertFalse(v.controller.regen_enabled)
        for fraction in (0., .1, .4, .8, 1.):
            force = fraction * v.maximum_pedal_force
            vector = self.forward(force)
            result = invert_brake_effort(v, float(np.sum(np.abs(vector))))
            self.assertTrue(result.feasible)
            self.assertAlmostEqual(result.pedal_force_n, force, delta=1.e-6)
            np.testing.assert_allclose(result.wheel_torques_nm, vector, rtol=0, atol=1.e-8)
            self.assertFalse(result.wheel_torques_nm.flags.writeable)

    def test_zero_unreachable_and_input_validation(self):
        v = self.vehicle
        zero = invert_brake_effort(v, 0.)
        self.assertEqual(zero.pedal_force_n, 0.)
        self.assertTrue(zero.feasible)
        cap_effort = float(np.sum(np.abs(self.forward(v.maximum_pedal_force))))
        short = invert_brake_effort(v, cap_effort + 1.)
        self.assertFalse(short.feasible)
        self.assertEqual(short.pedal_force_n, v.maximum_pedal_force)
        self.assertEqual(short.constraint, "maximum_pedal_force")
        self.assertAlmostEqual(short.residual_nm, -1.)
        for bad in (-1., float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                invert_brake_effort(v, bad)

    def test_regeneration_is_rejected(self):
        from dataclasses import replace
        v = replace(self.vehicle, controller=replace(self.vehicle.controller, regen_enabled=True,
            regen_pedal_force_limit=100., regen_motor_torque_limit=10.))
        with self.assertRaisesRegex(ValueError, "regenerative"):
            invert_brake_effort(v, 1.)

    def test_pressure_cap_plateau_and_shaped_command_plateau(self):
        from dataclasses import replace
        from simplified_vehicle_models.solvers.driver_maps import MonotoneTable1D, PedalActuatorCalibration
        base = self.vehicle
        low_pressure = replace(base, maximum_brake_pressure=1.0e6)
        # The exact-cap request must land on the analytic hydraulic onset,
        # independently established above, not an empirical torque-equality crossing.
        limited_forward = evaluate_forward_powertrain(low_pressure, self.plant.driven, self.plant.gearing,
            self.plant.gear_efficiency, np.full(4, 30.), low_pressure.storage.initial_state_of_charge,
            DriverCommand(0., base.maximum_pedal_force, 0.)).wheel_torques
        cap = float(np.sum(np.abs(limited_forward)))
        plateau = invert_brake_effort(low_pressure, cap)
        just_below = invert_brake_effort(low_pressure, cap - 1.0e-9)
        just_above = invert_brake_effort(low_pressure, np.nextafter(cap, np.inf))
        self.assertTrue(plateau.feasible)
        self.assertAlmostEqual(plateau.pedal_force_n, 191.90707657344865, delta=1.0e-6)
        from simplified_vehicle_models.controls.brakes import brake_torques
        mapped = brake_torques(
            plateau.pedal_force_n, base.maximum_pedal_force, base.pedal_curve,
            base.hydraulic_ratio_front, base.hydraulic_ratio_rear,
            low_pressure.maximum_brake_pressure, base.caliper_area, base.brake_bias,
            base.pad_mu_front, base.pad_mu_rear, base.brake_effective_radius_front,
            base.brake_effective_radius_rear)
        self.assertAlmostEqual(mapped.line_pressure, low_pressure.maximum_brake_pressure, delta=1e-7)
        self.assertLess(just_below.pedal_force_n, plateau.pedal_force_n)
        self.assertFalse(just_above.feasible)
        self.assertEqual(just_above.pedal_force_n, base.maximum_pedal_force)
        self.assertLess(just_above.residual_nm, 0.)
        table = MonotoneTable1D((0., .2, .4, 1.), (0., 0., .4, 1.), "1", "1")
        plateau_table = MonotoneTable1D((0., .2, .4, .6, 1.), (0., .15, .4, .4, 1.), "1", "1")
        calibration = PedalActuatorCalibration(plateau_table, plateau_table)
        shaped = replace(base, controller=replace(base.controller, actuator_calibration=calibration))
        # A flat section maps to zero hydraulic effort; zero always selects its
        # smallest input, and an interior request uses the same shaped chain.
        result = invert_brake_effort(shaped, 0.)
        self.assertEqual(result.pedal_force_n, 0.)
        target_force = .7 * base.maximum_pedal_force
        target = float(np.sum(np.abs(self.forward(target_force))))
        # Calibration changes the physical command, so derive target from the
        # real forward powertrain at the shaped input.
        forward = evaluate_forward_powertrain(shaped, self.plant.driven, self.plant.gearing,
            self.plant.gear_efficiency, np.full(4, 30.), shaped.storage.initial_state_of_charge,
            DriverCommand(0., target_force, 0.)).wheel_torques
        result = invert_brake_effort(shaped, float(np.sum(np.abs(forward))))
        self.assertAlmostEqual(result.pedal_force_n, target_force, delta=1.e-6)

    def test_force_ratio_product_certificate_rejects_nonmonotone(self):
        from dataclasses import replace
        from simplified_vehicle_models.controls.brakes import PedalRatioCurve
        curve = PedalRatioCurve(np.array([0., 100., 200.]), np.array([10., 1., .1]))
        v = replace(self.vehicle, pedal_curve=curve)
        with self.assertRaisesRegex(ValueError, "not monotone"):
            invert_brake_effort(v, 10.)


if __name__ == "__main__":
    unittest.main()
