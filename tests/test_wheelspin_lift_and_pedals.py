"""Wheelspin, wheel lift, pedal slew, and the UniTire-based live speed plan."""
import unittest

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations import point_mass
from simplified_vehicle_models.formulations.planar_7dof import evaluate_normal_loads
from simplified_vehicle_models.parameters.tire import UniTirePeakEnvelope
from simplified_vehicle_models.powertrain.motor_envelope import (
    INVERTER_SPEED_REGULATION_BAND, motor_speed_limit, motor_torque_limit,
    speed_regulated_torque_limit,
)
from simplified_vehicle_models.solvers.forward_driver import (
    DriverCommand, DriverObservation, PreviewDriver, RateSystem, SpeedPlan,
    build_speed_plan, solve_forward,
)
from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
from simplified_vehicle_models.suspension import quasi_static

PLANAR = "models/formulations/iem24_murphy_real_planar_7dof.xml"


class WheelLiftTests(unittest.TestCase):
    common = dict(total_mass=287., cg_height=.3, track_width=1.016,
                  lateral_transfer_distribution=.42)

    def loads(self, ay, lift, front=1500., rear=1300.):
        return np.array(quasi_static.lateral_transfer_loads(
            front, rear, lateral_acceleration=ay, wheel_lift=lift, **self.common))

    def test_default_transfer_is_unchanged(self):
        np.testing.assert_allclose(self.loads(9., False), self.loads(9., True))
        self.assertLess(self.loads(-16., False)[2], 0.)
        self.assertLess(self.loads(16., False)[3], 0.)

    def test_lifted_wheel_moves_moment_to_other_axle(self):
        for ay in (-16., 16.):
            loads = self.loads(ay, True)
            self.assertGreaterEqual(loads.min(), 0.)
            self.assertAlmostEqual(loads.sum(), 2800.)
            # The total roll moment is still reacted.
            transfer = .5 * (loads[0] - loads[1]) + .5 * (loads[2] - loads[3])
            self.assertAlmostEqual(transfer, 287. * ay * .3 / 1.016)
            self.assertEqual(min(loads[2], loads[3]), 0.)
            self.assertGreater(min(loads[0], loads[1]), 0.)

    def test_two_wheel_state_is_flagged_as_negative_load(self):
        self.assertLess(self.loads(30., True).min(), 0.)

    def test_planar_loads_use_wheel_lift(self):
        vehicle = compile_formulation(PLANAR).model
        loads = evaluate_normal_loads(vehicle, 15., 0., -17.)
        self.assertEqual(loads[2], 0.)
        self.assertGreaterEqual(loads.min(), 0.)


class SpeedRegulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(PLANAR).model
        cls.motor = cls.vehicle.motor

    def test_taper_is_continuous_and_zero_at_limit(self):
        voltage = 565.
        limit = motor_speed_limit(self.motor, voltage)
        below = limit * (1. - 2. * INVERTER_SPEED_REGULATION_BAND)
        np.testing.assert_allclose(speed_regulated_torque_limit(self.motor, below, voltage),
                                   motor_torque_limit(self.motor, below, voltage))
        speeds = np.linspace(.95 * limit, 1.05 * limit, 2001)
        torque = speed_regulated_torque_limit(self.motor, speeds, voltage)
        self.assertEqual(float(speed_regulated_torque_limit(self.motor, limit, voltage)), 0.)
        self.assertTrue(np.all(torque[speeds >= limit] == 0.))
        self.assertLess(np.max(np.abs(np.diff(torque))), .05)

    def test_spinning_driven_wheel_closes_the_voltage_loop(self):
        plan = SpeedPlan(np.array([0., 100.]), np.array([15., 15.]), np.zeros(2))
        plant = PlanarForwardPlant(self.vehicle, plan)
        state = plant.initial_state(15.)
        command = DriverCommand(.85, 0., 0.)
        limit = motor_speed_limit(self.motor, 560.) / self.motor.gear_ratio
        previous = None
        for fraction in np.linspace(.9, 1.05, 61):
            spun = state.copy()
            spun[8] = fraction * limit
            point = plant.powertrain(spun, command)
            self.assertAlmostEqual(point.bus_power, point.voltage * point.current, places=5)
            torque = point.motor_wheel_torques[2]
            if previous is not None:
                self.assertLessEqual(torque, previous + 1.e-9)
            previous = torque
        self.assertEqual(previous, 0.)
        rates = plant.evaluate(0., spun, command).solve()
        self.assertTrue(np.all(np.isfinite(rates)))
        # Above the limit the motor gives no torque and the tire slows the wheel.
        self.assertLess(rates[8], 0.)


class PedalSlewTests(unittest.TestCase):
    def setUp(self):
        distance = np.linspace(0., 100., 101)
        self.plan = build_speed_plan(distance, np.zeros(101), 15., 4., 2., 3.)
        self.driver = PreviewDriver(self.plan, 1.5, .5, 2., 1000., pedal_travel_time=.175)

    def test_rate_limit_and_exclusive_pedals(self):
        self.assertAlmostEqual(self.driver.pedal_rate(-1., 1.), 1. / .175)
        self.assertAlmostEqual(self.driver.pedal_rate(1., -1.), -1. / .175)
        self.assertAlmostEqual(self.driver.pedal_rate(.5, .5), 0.)
        brake = self.driver.realized_command(-.4, DriverCommand(1., 0., .3))
        self.assertEqual(brake.throttle, 0.)
        self.assertAlmostEqual(brake.brake_force, 400.)
        self.assertEqual(brake.steering_rate, .3)
        throttle = self.driver.realized_command(1.2, DriverCommand(0., 1000., 0.))
        self.assertEqual((throttle.throttle, throttle.brake_force), (1., 0.))
        with self.assertRaises(ValueError):
            PreviewDriver(self.plan, 1.5, .5, 2., 1000., pedal_travel_time=0.)

    def test_solver_integrates_the_pedal_with_a_bounded_rate(self):
        driver = self.driver

        class Plant:
            state_size = 2
            seen = []

            def observe(self, state):
                # Ask for full throttle until 0.3 s, then full brake.
                speed = 1. if state[1] < .3 else 30.
                return DriverObservation(state[0], 0., 0., speed, 0., 0.)

            def evaluate(self, time, state, command):
                self.seen.append((command.throttle, command.brake_force))
                return RateSystem(np.array([1., 1.]))

        plant = Plant()
        result = solve_forward(plant, driver, np.zeros(2), duration=1., maximum_step=.005)
        pedal = result.pedal
        self.assertEqual(result.state.shape[0], 2)
        self.assertEqual(pedal.shape, result.time.shape)
        rates = np.diff(pedal) / np.diff(result.time)
        self.assertLessEqual(np.max(np.abs(rates)), 1. / .175 + 1.e-6)
        self.assertAlmostEqual(pedal[np.searchsorted(result.time, .29)], 1., places=3)
        self.assertAlmostEqual(pedal[-1], -1., places=3)
        # Full throttle to full brake takes two pedal travels.
        crossing = result.time[np.argmax(pedal < -.99)]
        self.assertGreater(crossing, .3 + 2 * .175 - .02)
        self.assertTrue(all(t * b == 0. for t, b in plant.seen))

    def test_without_travel_time_pedals_stay_instantaneous(self):
        driver = PreviewDriver(self.plan, 1.5, .5, 2., 1000.)
        self.assertFalse(driver.has_pedal_dynamics)


class UniTirePlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(PLANAR).model

    def test_point_mass_rejects_raw_unitire_and_uses_its_envelope(self):
        with self.assertRaises(TypeError):
            point_mass.tire_force_capacities(self.vehicle, np.array([500.] * 4))
        view = point_mass.point_mass_view_of_planar(self.vehicle)
        self.assertIsInstance(view.tire, UniTirePeakEnvelope)
        self.assertTrue(view.wheel_lift)
        self.assertIs(view.drivetrain, self.vehicle.drivetrain)
        _, lateral = point_mass.tire_force_capacities(view, np.array([400., 1000.]))
        # The simplified UniTire peaks at mu_y * Fz with no load sensitivity.
        np.testing.assert_allclose(lateral / [400., 1000.], self.vehicle.tire.mu_0_y, rtol=1.e-3)
        self.assertIs(point_mass.point_mass_view_of_planar(self.vehicle), view)

    def test_live_plan_has_no_default_lateral_cap(self):
        radius = 20.
        distance = np.linspace(0., 2. * np.pi * radius, 252)
        curvature = np.full(distance.size, 1. / radius)
        plan = build_speed_plan(distance, curvature, 60., vehicle=self.vehicle, live_limit=True)
        lateral = plan.speed**2 / radius
        self.assertGreater(lateral.min(), 9.)
        capped = build_speed_plan(distance, curvature, 60., 3., vehicle=self.vehicle,
                                  live_limit=True)
        np.testing.assert_allclose(capped.speed**2 / radius, 3.)


if __name__ == "__main__":
    unittest.main()
