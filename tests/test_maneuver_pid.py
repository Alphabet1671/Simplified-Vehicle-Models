import unittest
from simplified_vehicle_models.controls.maneuvers.pid import PidConfig, SampledPid

class SampledPidTests(unittest.TestCase):
    def test_saturation_antiwindup_and_reset(self):
        pid = SampledPid(PidConfig(1., 1., 0., .02, -1., 1.))
        for _ in range(100): self.assertEqual(pid.update(10., 0., .01), 1.)
        self.assertLess(pid.update(0., 1., .01), 0.)
        pid.reset()
        self.assertEqual(pid.update(0., 0., .01), 0.)

    def test_filtered_derivative_is_on_measurement_no_setpoint_kick(self):
        pid = SampledPid(PidConfig(0., 0., 1., .02, -100., 100.))
        self.assertEqual(pid.update(0., 0., .01), 0.)
        self.assertEqual(pid.update(10., 0., .01), 0.)
        self.assertAlmostEqual(pid.update(10., 1., .01), -100/3)
        self.assertAlmostEqual(pid.update(20., 1., .01), -200/9)

    def test_measurement_rate_is_invariant_to_sample_interval(self):
        for dt in (.01, .02):
            pid = SampledPid(PidConfig(0., 0., 1., 0., -1000., 1000.))
            pid.update(0., 0., dt)
            self.assertAlmostEqual(pid.update(0., 2*dt, dt), -2.)

    def test_filtered_measurement_rate_uses_actual_dt(self):
        pid = SampledPid(PidConfig(0., 0., 1., .02, -1000., 1000.))
        pid.update(0., 0., .01)
        pid.update(0., 1., .01)
        self.assertAlmostEqual(pid.update(0., 3., .02), -200/3)

    def test_invalid_dt_rejected(self):
        pid = SampledPid(PidConfig(1., 0., 0., 0., -1., 1.))
        for dt in (0., -1., float('nan')):
            with self.assertRaises(ValueError): pid.update(0., 0., dt)

if __name__ == '__main__': unittest.main()
