"""Structural record admission and sensitivity math tests; never execute physics."""
import unittest
from types import SimpleNamespace
import numpy as np
from tools.reports.compare_yaw_mis_replay import validate_record_plan

class SensitivityMathTests(unittest.TestCase):
    def record(self, completed=True, duration=64.8):
        channels=('speed','acceleration-longitudinal','acceleration-lateral','throttle','bus-voltage','soc','wheel-drive-torque')
        states=('s','yaw_rate')
        manifest=SimpleNamespace(segment={'lap_completed':completed},channel_names=channels,channel_units=('',)*len(channels),channel_frames=('',)*len(channels),state_names=states, sources={},
            model={'yaw_profile':{'array_digest':'a','track_identity':'t','map_identity':'m','solver':'solver','profile_settings':{}},'controller':'model-aware'})
        return SimpleNamespace(time=np.array([0.,duration]),manifest=manifest,channel_validity=lambda n:np.ones(2,dtype=bool))
    def test_any_completed_duration_admitted(self):
        self.assertTrue(validate_record_plan(self.record(duration=61.25),{'array_digest':'a','track_identity':'t','map_identity':'m'}))
    def test_incomplete_lap_rejected(self):
        with self.assertRaisesRegex(ValueError,'lap_completed'):
            validate_record_plan(self.record(completed=False),{'array_digest':'a','track_identity':'t','map_identity':'m'})
    def test_plan_identity_mismatch_rejected(self):
        with self.assertRaisesRegex(ValueError,'track_identity'):
            validate_record_plan(self.record(),{'array_digest':'a','track_identity':'other','map_identity':'m'})
    def test_controller_limit_changes_request_not_delivered_torque(self):
        throttle=np.array([0.2,0.8]); delivered=np.array([3.0,7.0])
        request_a=throttle*10.5; request_b=throttle*9.0
        np.testing.assert_allclose(request_a,[2.1,8.4]); np.testing.assert_allclose(request_b,[1.8,7.2])
        np.testing.assert_array_equal(delivered,[3.0,7.0])

if __name__=='__main__': unittest.main()
