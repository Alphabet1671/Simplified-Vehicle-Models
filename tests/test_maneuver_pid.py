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
        self.assertAlmostEqual(pid.update(10., 1., .01), -1/3)

    def test_invalid_dt_rejected(self):
        pid = SampledPid(PidConfig(1., 0., 0., 0., -1., 1.))
        for dt in (0., -1., float('nan')):
            with self.assertRaises(ValueError): pid.update(0., 0., dt)

if __name__ == '__main__': unittest.main()
