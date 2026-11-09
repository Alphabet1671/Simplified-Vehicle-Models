"""Focused historical preview command, pedal, and default wiring regressions."""
import unittest
import numpy as np

from simplified_vehicle_models.solvers.forward_driver import DriverObservation, PreviewDriver, SpeedPlan
from tools.run.run_iem24_forward_driver import parse_args
from tools.sim_gui.settings import RunSettings, forward_driver_config, validate
from tools.sim_gui.telemetry_schema import project, schema_document


class HistoricalPreviewRestorationTests(unittest.TestCase):
    def setUp(self):
        self.plan = SpeedPlan(np.array([0., 10., 20., 30.]), np.array([8., 10., 6., 8.]), np.array([.01, .03, -.02, .01]))
        self.driver = PreviewDriver(self.plan, 2., .5, 10., 100., throttle_apply_rate=4., throttle_release_rate=6., brake_apply_rate=5., brake_release_rate=7.)

    def test_exact_longitudinal_and_preview_steering(self):
        for speed in (2., 20.):
            o = DriverObservation(5., .03, .02, speed, .1, .01, .2, .425)
            command, diag = self.driver.command_with_context(o)
            target, curvature, feed = self.plan.sample(o.distance)
            a = feed + 2. * (target - speed)
            self.assertAlmostEqual(command.throttle, np.clip(a / 6., 0., 1.))
            self.assertAlmostEqual(command.brake_force, np.clip(-a / 6., 0., 1.) * 100. * .5)
            preview = self.plan.sample(o.distance + speed * .15)[1]
            angle = np.arctan(2. * preview) - 1.5 * (o.heading_error + np.arctan2(o.lateral_speed, max(speed, .25))) - np.arctan2(1.5 * o.lateral_error, max(speed, 1.)) - .1 * (o.yaw_rate - speed * curvature)
            expected = np.clip((np.clip(angle, -.5, .5) - o.steering) / .08, -10., 10.)
            self.assertAlmostEqual(command.steering_rate, expected)
            self.assertEqual(diag.to_dict()['schema'], 'historical-preview-v1')

    def test_smoothing_and_four_directional_rates(self):
        gradient = np.gradient(self.plan.speed ** 2, self.plan.distance) / 2.
        expected = []
        for s in self.plan.distance[:-1]:
            delta = (self.plan.distance[:-1] - s + 15.) % 30. - 15.
            expected.append(np.mean(gradient[:-1][np.abs(delta) <= 4.]))
        np.testing.assert_allclose(self.plan.acceleration, expected + expected[:1])
        for position, target, expected_rate in ((.5, 1., 4.), (.5, -1., -6.), (-.5, -1., -5.), (-.5, 1., 7.), (0., -1., -5.), (0., 1., 4.)):
            self.assertEqual(self.driver.pedal_rate(position, target), expected_rate)
        requested = self.driver.command(DriverObservation(0., 0., 0., 8., 0., 0.))
        for p in (-1., -.3, 0., .3, 1.):
            actual = self.driver.realized_command(p, requested)
            self.assertEqual(actual.throttle * actual.brake_force, 0.)

    def test_cli_gui_defaults_and_historical_telemetry(self):
        self.assertEqual(parse_args([]).longitudinal_mode, 'historical-preview-v1')
        s = RunSettings()
        self.assertEqual(forward_driver_config(s)['policy'], 'historical-preview-v1')
        validate(s, 'planar-7dof', None)
        d = self.driver.command_with_context(DriverObservation(0., 0., 0., 8., 0., 0.))[1].to_dict()
        row = project({'longitudinal_mode': 'historical-preview-v1', 'time': 0., 'policy_diagnostics': d})
        self.assertEqual(row['desired_acceleration_mps2'], d['acceleration_request'])
        self.assertEqual(schema_document(s.forward_longitudinal_mode)['schema'], 'historical-preview-v1')
        self.assertNotIn('planner_horizon_s', row)

if __name__ == '__main__':
    unittest.main()
