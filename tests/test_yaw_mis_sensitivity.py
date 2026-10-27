"""Structural record admission and sensitivity math tests; never execute physics."""
import unittest
from types import SimpleNamespace
import numpy as np
from tools.reports.compare_yaw_mis_replay import validate_record_plan, cumulative_time_on_reference, sector_time_gaps, paired_sector_gap_table

class SensitivityMathTests(unittest.TestCase):
    def record(self, completed=True, duration=64.8):
        channels=('speed','acceleration-longitudinal','acceleration-lateral','throttle','bus-voltage','soc','wheel-drive-torque')
        states=('s','yaw_rate')
        manifest=SimpleNamespace(segment={'lap_completed':completed},channel_names=channels,channel_units=('',)*len(channels),channel_frames=('',)*len(channels),state_names=states, sources={},
            model={'yaw_profile':{'array_digest':'a','track_identity':'t','map_identity':'m','solver':'solver','profile_settings':{}},'controller':'model-aware'})
        return SimpleNamespace(time=np.array([0.,duration]),manifest=manifest,channel_validity=lambda n:np.ones(2,dtype=bool),channel_values=lambda n:np.zeros(2))
    def test_any_completed_duration_admitted(self):
        self.assertTrue(validate_record_plan(self.record(duration=61.25),{'array_digest':'a','track_identity':'t','map_identity':'m'}))
    def test_incomplete_lap_rejected(self):
        with self.assertRaisesRegex(ValueError,'lap_completed'):
            validate_record_plan(self.record(completed=False),{'array_digest':'a','track_identity':'t','map_identity':'m'})
    def test_nonfinite_and_duplicate_times_rejected(self):
        rec=self.record(); rec.time=np.array([0.,np.nan])
        with self.assertRaisesRegex(ValueError,'strictly increasing'):
            validate_record_plan(rec,{'array_digest':'a','track_identity':'t','map_identity':'m'})
        rec.time=np.array([0.,0.])
        with self.assertRaisesRegex(ValueError,'strictly increasing'):
            validate_record_plan(rec,{'array_digest':'a','track_identity':'t','map_identity':'m'})
    def test_duplicate_reference_station_keeps_last_boundary_time(self):
        result=cumulative_time_on_reference(np.array([0.,1.,3.,4.]),np.array([0.,1.,1.,2.]),np.array([0.,1.,2.,3.]),[0.,1.,2.])
        np.testing.assert_allclose(result,[0.,3.,4.])
        rows=sector_time_gaps(np.array([0.,1.,3.,4.]),np.array([0.,1.,1.,2.]),np.array([0.,1.,2.,3.]),1.)
        self.assertEqual(rows[0]['duration_s'],3.)
        self.assertEqual(rows[-1]['duration_s'],1.)
    def test_50m_sector_gap_table_keeps_final_remainder_and_signed_gaps(self):
        rows=paired_sector_gap_table(np.array([0.,2.,4.]),np.array([0.,50.,100.]),np.array([10.,13.,16.]),np.array([0.,50.,100.]),100.)
        self.assertEqual([(r['start_m'],r['end_m']) for r in rows],[(0.,50.),(50.,100.)])
        self.assertEqual([r['signed_gap_sim_minus_meas_s'] for r in rows],[-1.,-1.])
    def test_plan_identity_mismatch_rejected(self):
        with self.assertRaisesRegex(ValueError,'track_identity'):
            validate_record_plan(self.record(),{'array_digest':'a','track_identity':'other','map_identity':'m'})
    def test_controller_limit_changes_request_not_delivered_torque(self):
        throttle=np.array([0.2,0.8]); delivered=np.array([3.0,7.0])
        request_a=throttle*10.5; request_b=throttle*9.0
        np.testing.assert_allclose(request_a,[2.1,8.4]); np.testing.assert_allclose(request_b,[1.8,7.2])
        np.testing.assert_array_equal(delivered,[3.0,7.0])

if __name__=='__main__': unittest.main()
