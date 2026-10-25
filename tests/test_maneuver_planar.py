import unittest
import numpy as np
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.controls.maneuvers.types import RobotCommand
from simplified_vehicle_models.solvers.maneuver_planar import ActuatorCalibration, ManeuverPlanarPlant
from simplified_vehicle_models.formulations.planar_7dof import PlanarState, evaluate_planar_dynamics
from simplified_vehicle_models.solvers.forward_driver import DriverCommand
from simplified_vehicle_models.solvers.forward_powertrain import evaluate_forward_powertrain


class ManeuverPlanarTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation('models/formulations/iem24_murphy_real_planar_7dof.xml').model
        cls.calibration = ActuatorCalibration.from_dict({
            'maximum_swa_rad': 8., 'maximum_swa_rate_rad_s': 2.,
            'rack_travel_per_revolution_m': .04, 'steer_arm_length_m': .10,
            'maximum_road_wheel_angle_rad': cls.vehicle.steering_maximum_angle,
            'ackermann_inner_angle_rad': [.0, .6], 'ackermann_outer_offset_rad': [.0, .1],
            'accelerator_travel': [0., 1.], 'accelerator_force_n': [0., cls.vehicle.controller.maximum_drive_motor_torque_nm],
            'brake_travel': [0., 1.], 'brake_force_n': [0., cls.vehicle.maximum_pedal_force],
        })

    def setUp(self):
        self.plant = ManeuverPlanarPlant(self.vehicle, self.calibration)

    def test_cartesian_heading_kinematics(self):
        state = self.plant.initial_state(10., heading_rad=np.pi / 2)
        rate = self.plant.evaluate(0., state, RobotCommand(0., 0., 0.)).solve()
        self.assertAlmostEqual(rate[0], 0., places=10)
        self.assertAlmostEqual(rate[1], 10., places=10)

    def test_calibration_and_ackermann(self):
        pos = self.calibration.wheel_angles(self.vehicle, 1.)
        neg = self.calibration.wheel_angles(self.vehicle, -1.)
        self.assertNotEqual(pos[0], pos[1])
        self.assertEqual(pos[2], 0.)
        self.assertEqual(pos[3], 0.)
        self.assertGreater(pos[1], pos[0])  # right turn: FR inner
        self.assertGreater(abs(neg[0]), abs(neg[1]))  # left turn: FL inner
        self.assertLess(pos[0], pos[1])
        self.assertLess(neg[0], neg[1])

    def test_swa_servo_rate_and_bounds(self):
        s = self.plant.initial_state(10.)
        out = self.plant.accepted_outputs(0., s, RobotCommand(9., .5, 0.))
        self.assertLessEqual(abs(out['swa_rate_rad_s']), 2.)
        self.assertLessEqual(abs(out['swa_rad']), self.calibration.maximum_swa_rad)

    def test_pedal_map_and_missing_calibration(self):
        self.assertLess(self.calibration.accelerator_force(.2), self.calibration.accelerator_force(.8))
        with self.assertRaises((ValueError, Exception)):
            ManeuverPlanarPlant(self.vehicle, None)

    def test_trial_rhs_does_not_poison_observation(self):
        accepted = self.plant.initial_state(10.)
        trial = accepted.copy(); trial[3:6] = [7., 2., .4]
        cmd = RobotCommand(0., 0., 0.)
        self.plant.evaluate(0., trial, cmd)
        obs1 = self.plant.observe(0., accepted, cmd)
        self.plant.evaluate(0., trial, cmd)
        obs2 = self.plant.observe(0., accepted, cmd)
        self.assertEqual(obs1, obs2)

    def test_pedal_axes_must_stay_in_unit_interval(self):
        payload = {'maximum_swa_rad': 8., 'maximum_swa_rate_rad_s': 2.,
            'rack_travel_per_revolution_m': .04, 'steer_arm_length_m': .1,
            'maximum_road_wheel_angle_rad': self.vehicle.steering_maximum_angle,
            'ackermann_inner_angle_rad': [.0, .6], 'ackermann_outer_offset_rad': [.0, .1],
            'accelerator_travel': [-.1, 1.], 'accelerator_force_n': [0., self.vehicle.controller.maximum_drive_motor_torque_nm],
            'brake_travel': [0., 1.], 'brake_force_n': [0., self.vehicle.maximum_pedal_force]}
        with self.assertRaises(ValueError): ActuatorCalibration.from_dict(payload)

    def test_absolute_accelerator_map_scale_is_not_renormalized(self):
        payload = {'maximum_swa_rad': 8., 'maximum_swa_rate_rad_s': 2.,
            'rack_travel_per_revolution_m': .04, 'steer_arm_length_m': .1,
            'maximum_road_wheel_angle_rad': self.vehicle.steering_maximum_angle,
            'ackermann_inner_angle_rad': [.0, .6], 'ackermann_outer_offset_rad': [.0, .1],
            'accelerator_travel': [0., 1.], 'accelerator_force_n': [0., 100.],
            'brake_travel': [0., 1.], 'brake_force_n': [0., self.vehicle.maximum_pedal_force]}
        with self.assertRaises(Exception): ManeuverPlanarPlant(self.vehicle, ActuatorCalibration.from_dict(payload))

    def test_world_heading_unconstrained(self):
        s = self.plant.initial_state(10., heading_rad=2.)
        self.plant.evaluate(0., s, RobotCommand(0., 0., 0.))

    def test_zero_and_small_angle_dynamics_match_planar_closure(self):
        # Independently assemble the planar-forward reference from vehicle
        # configuration and public closures: do not source torques/angles from
        # the maneuver plant's diagnostics, which would make this tautological.
        from simplified_vehicle_models.controls.steering import AckermannTable, steering_angles

        state = self.plant.initial_state(18.)
        state[3:6] = [18., 0.15, 0.025]
        command = RobotCommand(0., 0., 0.)
        transmission = self.vehicle.drivetrain.drivetrain.transmission
        wheel_names = ('fl', 'fr', 'rl', 'rr')
        driven = np.array([wheel_names.index(name) for name in transmission.driven_wheels])
        gear_efficiency = transmission.gear_efficiency
        gearing = self.vehicle.motor.gear_ratio * gear_efficiency
        for swa in (0., 0.01):
            state[11] = swa
            rate = self.plant.evaluate(0., state, command).solve()

            # Reproduce the documented calibration conversion, independently
            # from ManeuverPlanarPlant: steering geometry -> road-wheel angles;
            # calibrated zero pedals -> physical force/normalized throttle.
            c = self.calibration
            steer = steering_angles(
                swa, c.rack_travel_per_revolution_m, c.steer_arm_length_m,
                min(c.maximum_road_wheel_angle_rad, self.vehicle.steering_maximum_angle),
                AckermannTable(c.ackermann_inner_angle_rad, c.ackermann_outer_offset_rad))
            road_wheel_angles = np.asarray(steer.wheel_angles, dtype=float)
            accelerator_force = float(np.interp(0., c.accelerator_travel, c.accelerator_force_n))
            throttle = accelerator_force / float(np.interp(1., c.accelerator_travel, c.accelerator_force_n))
            brake_force = float(np.interp(0., c.brake_travel, c.brake_force_n))
            driver_command = DriverCommand(throttle, brake_force, 0.)
            powertrain = evaluate_forward_powertrain(
                self.vehicle, driven, gearing, gear_efficiency,
                state[6:10], state[10], driver_command)
            mechanical = PlanarState(state[3], state[4], state[5], state[6:10], state[10])
            reference = evaluate_planar_dynamics(
                self.vehicle, mechanical, powertrain.wheel_torques, road_wheel_angles)
            self.assertTrue(reference.load_converged)
            np.testing.assert_allclose(rate[3:10], np.r_[
                reference.longitudinal_acceleration,
                reference.lateral_acceleration,
                reference.yaw_acceleration,
                reference.wheel_accelerations], rtol=1e-11, atol=1e-11)

    def test_pose_rates_and_body_acceleration_rotating_frame(self):
        s = self.plant.initial_state(10., x_m=2., y_m=-1., heading_rad=.4)
        s[3:6] = [8., 1.2, .3]
        rate = self.plant.evaluate(0., s, RobotCommand(0., 0., 0.)).solve()
        self.assertAlmostEqual(rate[0], 8*np.cos(.4)-1.2*np.sin(.4), places=12)
        self.assertAlmostEqual(rate[1], 8*np.sin(.4)+1.2*np.cos(.4), places=12)
        obs = self.plant.observe(0., s)
        self.assertAlmostEqual(obs.ay_mps2, rate[4]+.3*8, places=12)

    def test_calibration_domain_validation(self):
        with self.assertRaises(ValueError):
            self.calibration.accelerator_force(1.1)
        with self.assertRaises(ValueError):
            ActuatorCalibration.from_dict({'maximum_swa_rad': 8.})


if __name__ == '__main__': unittest.main()
