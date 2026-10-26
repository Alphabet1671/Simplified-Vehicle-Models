import unittest
import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.controls.maneuvers.types import RobotCommand
from simplified_vehicle_models.solvers.maneuver_planar import ActuatorCalibration
from simplified_vehicle_models.solvers.maneuver_vertical import VerticalManeuverPlant
from simplified_vehicle_models.solvers.vertical_forward import MECHANICAL_INDEX


class VerticalManeuverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation('models/formulations/iem24_murphy_real_14dof_simple.xml').model
        cls.calibration = ActuatorCalibration.from_dict({
            'maximum_swa_rad': 8., 'maximum_swa_rate_rad_s': 2.,
            'rack_travel_per_revolution_m': .04, 'steer_arm_length_m': .10,
            'maximum_road_wheel_angle_rad': cls.vehicle.planar.steering_maximum_angle,
            'ackermann_inner_angle_rad': [.0, .6], 'ackermann_outer_offset_rad': [.0, .1],
            'accelerator_travel': [0., 1.], 'accelerator_force_n': [0., cls.vehicle.planar.controller.maximum_drive_motor_torque_nm],
            'brake_travel': [0., .5, 1.], 'brake_force_n': [0., 137., 420.],
        })

    def setUp(self):
        self.plant = VerticalManeuverPlant(self.vehicle, self.calibration)

    def test_vertical_adapter_keeps_vertical_state_layout_and_straight_world_pose(self):
        state = self.plant.initial_state(16.6666666667, x_m=2., y_m=-1., heading_rad=.1)
        self.assertEqual(state.shape, (self.plant.state_size,))
        observation = self.plant.observe(0., state, RobotCommand(0., 0., 0.))
        self.assertAlmostEqual(observation.x_m, 2.)
        self.assertAlmostEqual(observation.y_m, -1.)
        self.assertAlmostEqual(observation.heading_rad, .1)

    def test_command_calibration_and_accepted_vertical_channels(self):
        state = self.plant.initial_state(16.6666666667)
        outputs = self.plant.accepted_outputs(0., state, RobotCommand(0., 0., .5))
        self.assertEqual(outputs['brake_force_n'], self.calibration.brake_force(.5))
        for key in ('requested_swa_rad', 'swa_rad', 'throttle_fraction', 'body_ay_mps2',
                    'speed_mps', 'x_m', 'y_m', 'heading_rad', 'roll_rad', 'pitch_rad',
                    'heave_m', 'wheel_speeds_rad_s', 'jounce_m', 'jounce_rates_mps',
                    'acceleration-longitudinal', 'acceleration-lateral', 'roll', 'pitch',
                    'wheel-position-x', 'wheel-position-y', 'wheel-steering-angle',
                    'wheel-loaded-radius', 'tire-longitudinal-force', 'tire-lateral-force',
                    'tire-vertical-force', 'tire-slip-ratio', 'tire-slip-angle'):
            self.assertIn(key, outputs)
        for key in ('wheel_speeds_rad_s', 'wheel-position-x', 'wheel-position-y',
                    'wheel-steering-angle', 'wheel-loaded-radius', 'tire-longitudinal-force',
                    'tire-lateral-force', 'tire-vertical-force', 'tire-slip-ratio',
                    'tire-slip-angle'):
            self.assertEqual(len(outputs[key]), 4)
        self.assertTrue(np.all(np.asarray(outputs['tire-vertical-force']) >= 0.))

    def test_straight_heading_kinematics(self):
        state = self.plant.initial_state(10., heading_rad=np.pi / 2)
        rate = self.plant.evaluate(0., state, RobotCommand(0., 0., 0.)).solve()
        self.assertAlmostEqual(rate[0], 0., places=8)
        self.assertAlmostEqual(rate[1], 10., places=8)


if __name__ == '__main__':
    unittest.main()
