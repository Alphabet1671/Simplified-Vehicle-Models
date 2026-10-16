"""Paired point-mass MIS correlation artifacts and signal conditioning."""
import unittest
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfiltfilt

from tools.reports.report_iem24_point_mass_track_pair import filter_model

ROOT = Path(__file__).resolve().parents[1]
PAIR = ROOT / 'docs/validation/iem24-murphy/point_mass_track_pair'


class PointMassTrackPairTests(unittest.TestCase):
    def test_time_domain_filter_does_not_change_distance_or_clock(self):
        t = np.linspace(0, 10, 101)
        sample = {'elapsed_time_s': t, 'distance_m': 2*t,
                  'speed_mps': 5+np.sin(2*np.pi*.25*t)}
        result = filter_model(sample, ('speed_mps',))
        np.testing.assert_array_equal(result['distance_m'], sample['distance_m'])
        np.testing.assert_array_equal(result['elapsed_time_s'], t)
        self.assertLess(np.max(np.abs(result['speed_mps']-sample['speed_mps'])), 1.)
        self.assertGreater(np.max(np.abs(result['speed_mps']-sample['speed_mps'])), .001)

    def test_exported_model_traces_are_complete_and_symmetric(self):
        for name in ('original','experimental'):
            with self.subTest(track=name):
                path=PAIR/name/'simulated_lap.csv'
                self.assertTrue(path.is_file(), 'run paired solver before checking artifacts')
                data=np.genfromtxt(path,delimiter=',',names=True)
                self.assertTrue(np.all(np.isfinite(np.stack([data[k] for k in data.dtype.names]))))
                self.assertTrue(np.all(np.diff(data['distance_m'])>0))
                self.assertTrue(np.all(np.diff(data['elapsed_time_s'])>0))
                np.testing.assert_allclose(data['rl_power_kw'],data['rr_power_kw'],rtol=0,atol=0)
                self.assertTrue(np.all(data['motor_torque_request_nm']>=0))
                self.assertGreater(float(np.ptp(data['pack_voltage_v'])), .1)


if __name__=='__main__': unittest.main()
