import unittest
from types import SimpleNamespace
import numpy as np

from simplified_vehicle_models.controls.maneuvers.analysis import analyze_maneuver


def result(*, radial=None, speeds=None, saturated=False, outcome='completed', reason='duration', times=None):
    n = len(speeds or [5., 5., 5., 5., 5.])
    speeds = speeds or [5.] * n
    times = np.arange(n, dtype=float) if times is None else np.asarray(times, dtype=float)
    radial = radial or [0.] * n
    return SimpleNamespace(
        time_s=times,
        state=np.column_stack([np.zeros((n, 3)), speeds]),
        state_rate=np.zeros((n, 4)),
        observations=tuple(SimpleNamespace(speed_mps=s, ay_mps2=.5, swa_rad=.1) for s in speeds),
        channels=tuple({'radial_error_m': e, 'saturated': saturated} for e in radial),
        requested_commands=(1,) * n,
        applied_commands=(1,) * n,
        events=(), execution_outcome=outcome, termination_reason=reason,
    )


def skidpad(**steering):
    config = {'mode': 'radius_controlled', 'target_speed_mps': 4., 'path_tolerance_m': .1}
    config.update(steering)
    return {'preset': 'circular_skidpad', 'steering': config, 'longitudinal': {'mode': 'speed'}}


class ManeuverAnalysisTests(unittest.TestCase):
    def test_transient_spike_is_not_physical_limit(self):
        summary = analyze_maneuver(skidpad(), result(radial=[0., .5, 0., 0., 0.]))
        self.assertEqual(summary['objective']['limit_status'], 'not_established')
        self.assertIsNone(summary['objective']['last_valid_speed_mps'])

    def test_persistent_path_error_brackets_limit(self):
        summary = analyze_maneuver(skidpad(), result(radial=[0., 0., .2, .2, .2]))
        self.assertEqual(summary['objective']['limit_status'], 'bracketed_physical_limit')
        self.assertEqual(summary['objective']['last_valid_speed_mps'], 5.)
        self.assertEqual(summary['objective']['first_invalid_speed_mps'], 5.)

    def test_saturation_speed_or_model_failure_is_not_physical_limit(self):
        saturated = analyze_maneuver(skidpad(), result(radial=[.2] * 5, saturated=True))
        low_speed = analyze_maneuver(skidpad(), result(radial=[.2] * 5, speeds=[2.] * 5))
        failed = analyze_maneuver(skidpad(), result(radial=[.2] * 5, outcome='failed', reason='model_limit'))
        for summary in (saturated, low_speed, failed):
            self.assertEqual(summary['objective']['limit_status'], 'not_established')

    def test_locked_swa_and_malformed_samples_are_safe(self):
        locked = analyze_maneuver({'steering': {'mode': 'locked_swa'}}, result())
        self.assertIn('not a peak-limit', locked['objective']['research_classification'])
        bad = SimpleNamespace(time_s=np.array([0., 1., .5]), state=np.zeros((3, 4)),
                              state_rate=np.zeros((2, 4)), observations=(), channels=(),
                              requested_commands=(), applied_commands=(), events=(),
                              execution_outcome='completed', termination_reason='duration')
        summary = analyze_maneuver({}, bad)
        self.assertFalse(summary['execution']['sampled_arrays_valid'])
        self.assertIsNone(summary['metrics']['integrated_distance_m'])


if __name__ == '__main__':
    unittest.main()
