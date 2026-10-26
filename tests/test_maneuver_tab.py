import math
import tempfile
import unittest
from pathlib import Path

from tools.sim_gui.qt.maneuver_tab import ManeuverDraft, csv_file_snapshot


def payload():
    return {
        'schema': 'maneuver-v1', 'name': 'draft', 'preset': 'step_steer',
        'backend': 'simulation', 'vehicle': {'model': 'planar-7dof'},
        'initial': {'speed_mps': 10., 'x_m': 0., 'y_m': 0., 'heading_rad': 0.},
        'clock': {'controller_tick_s': .01},
        'steering': {'mode': 'peak_swa', 'amplitude_rad': .2, 'direction': 1},
        'longitudinal': {'mode': 'constant_accelerator', 'accelerator': .1},
        'profile': {'lead_in_s': 1., 'rise_s': .2, 'hold_s': 1., 'release_s': .2, 'recovery_s': 1., 'duration_s': 3.4},
        'limits': {'max_swa_rad': 1., 'max_swa_rate_rad_s': 2.},
        'termination': {'duration_s': 3.4}, 'calibration': None, 'csv': None,
    }


class ManeuverDraftTests(unittest.TestCase):
    def test_from_dict_is_defensive_and_si_unit_presentation_round_trips(self):
        source = payload()
        draft = ManeuverDraft.from_dict(source)
        source['steering']['amplitude_rad'] = 0
        self.assertAlmostEqual(draft.present('steering.amplitude_rad', 'deg'), math.degrees(.2))
        draft.assign_presented('steering.amplitude_rad', 90, 'deg')
        self.assertAlmostEqual(draft.to_dict()['steering']['amplitude_rad'], math.pi / 2)
        self.assertAlmostEqual(draft.present('initial.speed_mps', 'km/h'), 36.)
        self.assertAlmostEqual(draft.present('clock.controller_tick_s', 'ms'), 10.)

    def test_schema_validation_returns_inline_field_error(self):
        data = payload(); data['clock']['controller_tick_s'] = 0
        definition, errors = ManeuverDraft.from_dict(data).validate()
        self.assertIsNone(definition)
        self.assertIn('clock.controller_tick_s', errors)
        self.assertIsNotNone(ManeuverDraft.from_dict(payload()).validate()[0])

    def test_mode_enablement(self):
        draft = ManeuverDraft.from_dict(payload())
        enabled = draft.enabled_fields()
        self.assertTrue(enabled['steering.amplitude_rad'])
        self.assertFalse(enabled['steering.amplitude_mps2'])
        data = payload(); data['steering'] = {'mode': 'peak_ay', 'amplitude_mps2': 4., 'direction': 1}
        self.assertTrue(ManeuverDraft.from_dict(data).enabled_fields()['steering.amplitude_mps2'])
        data['preset'] = 'csv_playback'; data['csv'] = {'mapping': {'time': 't', 'swa': 's', 'accelerator': 'a', 'brake': 'b'}}
        data['steering'] = {'mode': 'csv'}; data['longitudinal'] = {'mode': 'csv'}
        self.assertTrue(ManeuverDraft.from_dict(data).enabled_fields()['csv'])

    def test_preset_guidance_preserves_schema_valid_modes(self):
        guidance = {
            'step_steer': 'Steering waveform', 'ramp_steer': 'Steering waveform',
            'sine_steer': 'Steering waveform', 'straight_maintain': 'Straight',
            'straight_accel_brake': 'Straight', 'circular_skidpad': 'Radius',
            'csv_playback': 'CSV supplies',
        }
        for preset, expected_guidance in guidance.items():
            with self.subTest(preset=preset):
                data = payload(); data['preset'] = preset
                draft = ManeuverDraft.from_dict(data)
                policy = draft.preset_policy()
                self.assertIn(expected_guidance.lower(), policy['guidance'].lower())
                self.assertIn('does not disable schema-valid modes', policy['mode_preservation'])
                self.assertIn('owner', policy)
                self.assertNotIn('disabled', policy)
                self.assertNotIn('allowed', policy)
                self.assertEqual(draft.to_dict()['steering']['mode'], data['steering']['mode'])
        # Preset guidance cannot hide valid controller modes: the schema allows
        # these alternate non-CSV combinations, and mode ownership enables them.
        alternate_modes = {
            'step_steer': ({'mode': 'path_pid', 'target_heading_rad': 0.}, 'steering.target_heading_rad'),
            'straight_maintain': ({'mode': 'peak_swa', 'amplitude_rad': .2, 'direction': 1}, 'steering.amplitude_rad'),
            'circular_skidpad': ({'mode': 'path_pid', 'target_heading_rad': 0.}, 'steering.target_heading_rad'),
        }
        for preset, (steering, active_field) in alternate_modes.items():
            with self.subTest(alternate=preset):
                data = payload(); data['preset'] = preset; data['steering'] = steering
                draft = ManeuverDraft.from_dict(data)
                self.assertIsNotNone(draft.validate()[0])
                self.assertTrue(draft.enabled_fields()[active_field])
        with self.subTest(preset='csv_playback'):
            data = payload(); data['preset'] = 'csv_playback'
            data['csv'] = {'mapping': {'time': 't', 'swa': 's', 'accelerator': 'a', 'brake': 'b'}}
            data['steering'] = {'mode': 'csv'}; data['longitudinal'] = {'mode': 'csv'}
            draft = ManeuverDraft.from_dict(data)
            fields = draft.enabled_fields()
            self.assertTrue(fields['csv'])
            self.assertTrue(all(not fields[key] for key in fields if key != 'csv'))
            self.assertEqual(draft.preset_policy()['owner'], 'CSV owns SWA, accelerator, and brake.')

    def test_preflight_calibration_fingerprint_and_physical_backend(self):
        data = payload(); data['steering'] = {'mode': 'peak_ay', 'amplitude_mps2': 4., 'direction': 1}
        data['calibration'] = {'id': 'cal'}
        draft = ManeuverDraft.from_dict(data)
        self.assertIn('calibration', draft.preflight().errors)
        self.assertIn('calibration', draft.preflight(calibration_status='matched', calibration_fingerprint='old', expected_fingerprint='new').errors)
        result = draft.preflight(calibration_status='matched', calibration_fingerprint='same', expected_fingerprint='same')
        self.assertTrue(result.ok, result.errors)
        physical = payload(); physical['backend'] = 'physical'; physical['calibration'] = {'id': 'cal'}
        result = ManeuverDraft.from_dict(physical).preflight(calibration_status='matched')
        self.assertIn('backend', result.errors)

    def test_csv_dirty_detection_hash_mtime_and_size(self):
        data = payload(); data['preset'] = 'csv_playback'
        data['steering'] = {'mode': 'csv'}; data['longitudinal'] = {'mode': 'csv'}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'commands.csv'; path.write_text('t,s,a,b\n0,0,0,0\n')
            data['csv'] = {'source': str(path), 'mapping': {'time': 't', 'swa': 's', 'accelerator': 'a', 'brake': 'b'}}
            draft = ManeuverDraft.from_dict(data); snapshot = csv_file_snapshot(path)
            self.assertTrue(draft.preflight(csv_snapshot=snapshot).ok)
            path.write_text('t,s,a,b\n0,1,0,0\n')
            result = draft.preflight(csv_snapshot=snapshot)
            self.assertIn('csv.source', result.errors)

    def test_preview_keeps_requested_and_applied_separate(self):
        draft = ManeuverDraft.from_dict(payload())
        applied = {'steering': {'amplitude_rad': .19}}
        preview = draft.preview(applied)
        self.assertEqual(preview['requested']['steering']['amplitude_rad'], .2)
        self.assertEqual(preview['applied'], applied)
        self.assertIsNone(draft.preview()['applied'])


if __name__ == '__main__':
    unittest.main()
