"""Check transient wheel and powertrain physics in the forward adapter."""
import unittest
from dataclasses import replace
from unittest.mock import patch

import numpy as np
from scipy.integrate import solve_ivp

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.forward_driver import DriverCommand, SpeedPlan
from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant, body_accelerations


class PlanarForwardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(
            "models/formulations/iem24_murphy_real_planar_7dof.xml"
        ).model
        cls.plan = SpeedPlan(np.array([0., 100.]), np.array([8., 8.]), np.zeros(2))

    def setUp(self):
        self.plant = PlanarForwardPlant(self.vehicle, self.plan)
        self.state = self.plant.initial_state(8.)

    def test_body_acceleration_includes_rotating_frame_terms(self):
        state = np.zeros((2, 12))
        rates = np.zeros((2, 12))
        state[0, 3], state[0, 5] = 10., .2
        state[1, 3:6] = [8., 1., -.3]
        rates[1, 3:5] = [-2., .5]
        ax, ay = body_accelerations(state, rates)
        np.testing.assert_allclose(ax, [0., -1.7])
        np.testing.assert_allclose(ay, [2., -1.9])
        with self.assertRaises(ValueError):
            body_accelerations(np.zeros(12), np.zeros(11))

    def test_rear_only_drive_and_power_closure(self):
        point = self.plant.powertrain(self.state, DriverCommand(.1, 0., 0.))
        np.testing.assert_array_equal(point.wheel_torques[:2], [0., 0.])
        self.assertTrue(np.all(point.wheel_torques[2:] > 0.))
        self.assertAlmostEqual(point.bus_power, point.voltage * point.current, places=6)
        self.assertGreater(point.current, 0.)

    def test_torque_changes_wheel_acceleration_not_current_tire_force(self):
        coast = self.plant.evaluate(0., self.state, DriverCommand(0., 0., 0.)).solve()
        drive = self.plant.evaluate(0., self.state, DriverCommand(.1, 0., 0.)).solve()
        torque = self.plant.powertrain(self.state, DriverCommand(.1, 0., 0.)).wheel_torques
        np.testing.assert_allclose(drive[3:6], coast[3:6], atol=1.e-12)
        np.testing.assert_allclose(drive[6:10] - coast[6:10], torque / self.vehicle.wheel_inertia)
        lighter = PlanarForwardPlant(replace(self.vehicle, wheel_inertia=self.vehicle.wheel_inertia / 2), self.plan)
        np.testing.assert_allclose(lighter.evaluate(0., self.state, DriverCommand(.1, 0., 0.)).solve()[6:10],
                                   2 * drive[6:10])

    def test_coupled_transient_changes_slip_and_force(self):
        command = DriverCommand(.1, 0., 0.)
        initial = self.state.copy()
        result = solve_ivp(lambda t, y: self.plant.evaluate(t, y, command).solve(),
                           (0., .02), initial, method="Radau", rtol=1.e-7, atol=1.e-9)
        self.assertTrue(result.success)
        final = result.y[:, -1]
        self.assertGreater(float(np.mean(final[8:10])), float(np.mean(initial[8:10])))
        self.assertGreater(self.plant.evaluate(.02, final, command).solve()[3],
                           self.plant.evaluate(0., initial, command).solve()[3])
        np.testing.assert_array_equal(self.state, initial)

    def test_regen_and_hydraulic_mapping(self):
        actual = self.plant.powertrain(self.state, DriverCommand(0., 100., 0.))
        self.assertEqual(actual.bus_power, 0.)
        self.assertTrue(np.all(actual.wheel_torques < 0.))
        controller = replace(self.vehicle.controller, regen_enabled=True,
                             regen_pedal_force_limit=294.1995, regen_motor_torque_limit=10.)
        plant = PlanarForwardPlant(replace(self.vehicle, controller=controller), self.plan)
        point = plant.powertrain(self.state, DriverCommand(0., 100., 0.))
        self.assertLess(point.bus_power, 0.)
        np.testing.assert_array_equal(point.wheel_torques[:2], [0., 0.])
        self.assertTrue(np.all(point.wheel_torques[2:] < 0.))
        hydraulic = plant.powertrain(self.state, DriverCommand(0., 400., 0.))
        self.assertTrue(np.all(hydraulic.wheel_torques < 0.))

    def test_regen_survives_motor_speed_limit_without_hydraulic_braking(self):
        from simplified_vehicle_models.powertrain.motor_envelope import motor_speed_limit

        controller = replace(self.vehicle.controller, regen_enabled=True,
                             regen_pedal_force_limit=294.1995, regen_motor_torque_limit=10.)
        vehicle = replace(self.vehicle, controller=controller)
        plant = PlanarForwardPlant(vehicle, self.plan)
        state = self.state.copy()
        voltage = vehicle.storage.series_count * float(np.interp(
            self.state[10], vehicle.storage.state_of_charge_axis,
            vehicle.storage.open_circuit_voltage_axis))
        speed_limit = motor_speed_limit(vehicle.motor, voltage)
        state[6 + plant.driven] = (speed_limit * 1.01) / vehicle.motor.gear_ratio
        point = plant.powertrain(state, DriverCommand(0., 100., 0.))
        self.assertTrue(np.all(point.motor_wheel_torques[plant.driven] < 0.))
        self.assertLess(point.bus_power, 0.)
        np.testing.assert_array_equal(point.wheel_torques[:2], [0., 0.])
        np.testing.assert_allclose(point.wheel_torques, point.motor_wheel_torques)

    def test_curved_seed_avoids_zero_sideslip_load_spike(self):
        plan = SpeedPlan(np.array([0., 100.]), np.array([4., 4.]), np.full(2, -.1226))
        plant = PlanarForwardPlant(self.vehicle, plan)
        initial = plant.initial_state(4.)
        rate = plant.evaluate(0., initial, DriverCommand(0., 0., 0.)).solve()
        self.assertAlmostEqual(rate[1], 0., places=12)
        self.assertTrue(np.all(np.isfinite(rate)))

    def test_driver_and_soc_bounds_are_enforced(self):
        for command in (DriverCommand(1.1, 0., 0.), DriverCommand(.1, 10., 0.),
                        DriverCommand(0., self.vehicle.maximum_pedal_force + 1, 0.)):
            with self.assertRaises(ValueError):
                self.plant.powertrain(self.state, command)
        invalid = self.state.copy()
        invalid[10] = 1.1
        with self.assertRaises(ValueError):
            self.plant.evaluate(0., invalid, DriverCommand(0., 0., 0.))
        original = self.state.copy()
        point = self.plant.powertrain(self.state, DriverCommand(1., 0., 0.))
        peak = self.vehicle.controller.maximum_drive_motor_torque_nm * self.plant.gearing
        self.assertLessEqual(float(np.max(point.motor_wheel_torques)), peak)
        np.testing.assert_array_equal(self.state, original)

    def test_invalid_local_physics_is_rejected(self):
        from simplified_vehicle_models.formulations.planar_7dof import evaluate_planar_dynamics
        evaluation = evaluate_planar_dynamics(self.vehicle, self.plant.mechanical_state(self.state), np.zeros(4), 0.)
        with patch("simplified_vehicle_models.solvers.planar_forward.evaluate_planar_dynamics",
                   return_value=replace(evaluation, load_converged=False)):
            with self.assertRaisesRegex(RuntimeError, "load"):
                self.plant.evaluate(0., self.state, DriverCommand(0., 0., 0.))
        reverse = self.state.copy()
        reverse[6] = -1.
        with self.assertRaises(ValueError):
            self.plant.evaluate(0., reverse, DriverCommand(0., 0., 0.))


if __name__ == "__main__":
    unittest.main()
