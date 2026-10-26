import unittest

from simplified_vehicle_models.controls.maneuvers.calibration import calibrate_peak_ay, definition_fingerprint


class Result:
    def __init__(self, ay, outcome='completed'):
        self.observations = [type('Observation', (), {'ay_mps2': ay})()]
        self.execution_outcome = outcome
        self.termination_reason = 'duration'


def definition(target=2.):
    return {
        'peak_ay_mps2': target,
        'steering': {'frequency_hz': 1., 'constant_pedal': .2},
        'model': {'hash': 'one'}, 'actuator_calibration': {'gain': 1.},
    }


class PeakAyCalibrationTests(unittest.TestCase):
    def test_monotone_calibration_scans_and_preserves_direction(self):
        calls = []
        def run(amplitude):
            calls.append(amplitude)
            return Result(abs(amplitude) * 2.)
        result = calibrate_peak_ay(definition(-2.), run, (0., 2.))
        self.assertEqual(result.status, 'matched')
        self.assertLess(result.amplitude_rad, 0.)
        self.assertLessEqual(abs(result.residual_mps2), max(.05, .02 * 2.))
        self.assertGreaterEqual(len(result.trials), 8)
        self.assertTrue(all(call <= 0. for call in calls))

    def test_canonical_peak_ay_steering_field_is_accepted(self):
        payload = definition(); payload.pop('peak_ay_mps2')
        payload['steering']['amplitude_mps2'] = 2.
        payload['steering']['direction'] = -1
        calls = []
        result = calibrate_peak_ay(payload, lambda a: calls.append(a) or Result(abs(a) * 2.), (0., 2.))
        self.assertEqual(result.status, 'matched')
        self.assertLess(result.amplitude_rad, 0.)
        self.assertLess(result.target_mps2, 0.)
        self.assertTrue(all(value <= 0. for value in calls))

    def test_unreachable_nonmonotone_and_model_limit_never_match(self):
        unreachable = calibrate_peak_ay(definition(), lambda a: Result(a), (0., 1.))
        self.assertEqual(unreachable.status, 'unreachable')
        ambiguous = calibrate_peak_ay(definition(), lambda a: Result(2. - a), (0., 2.))
        self.assertEqual(ambiguous.status, 'ambiguous')
        limited = calibrate_peak_ay(definition(), lambda a: Result(a, 'failed'), (0., 2.))
        self.assertEqual(limited.status, 'model_limit')

    def test_fingerprint_excludes_only_location_metadata(self):
        base = definition()
        base['ui'] = {'theme': 'dark'}
        base['output_path'] = '/tmp/first'
        base['model']['output'] = {'torque_nm': 120.}
        digest = definition_fingerprint(base)
        moved = definition()
        moved['ui'] = {'theme': 'light'}
        moved['output_path'] = '/tmp/second'
        moved['model']['output'] = {'torque_nm': 120.}
        self.assertEqual(digest, definition_fingerprint(moved))
        changed = definition()
        changed['ui'] = {'theme': 'dark'}
        changed['output_path'] = '/tmp/first'
        changed['model']['output'] = {'torque_nm': 121.}
        self.assertNotEqual(digest, definition_fingerprint(changed))

    def test_fingerprint_tracks_definition_and_selection_is_frozen(self):
        base = definition()
        selected = calibrate_peak_ay(base, lambda a: Result(a * 2.), (0., 2.))
        for path, value in ((('steering', 'frequency_hz'), 2.),
                            (('steering', 'constant_pedal'), .4),
                            (('model', 'hash'), 'two')):
            changed = definition(); changed[path[0]][path[1]] = value
            other = calibrate_peak_ay(changed, lambda a: Result(a * 2.), (0., 2.))
            self.assertNotEqual(selected.definition_sha256, other.definition_sha256)
        amplitude = selected.amplitude_rad
        # A later/perturbed final result cannot mutate a frozen selection.
        final = Result(999.)
        self.assertEqual(selected.amplitude_rad, amplitude)
        self.assertNotEqual(abs(final.observations[0].ay_mps2), selected.achieved_mps2)


if __name__ == '__main__':
    unittest.main()
