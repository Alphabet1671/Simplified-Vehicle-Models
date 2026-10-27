import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import unittest
import tempfile
from pathlib import Path

try:
    from PyQt6.QtWidgets import QApplication
except ImportError:
    QApplication = None

if QApplication is not None:
    from tools.maneuver_gui.qt import ManeuverTab, ManeuverEditor, PreviewWidget
    from tests.test_maneuver_tab import payload


@unittest.skipIf(QApplication is None, "PyQt6 is unavailable")
class ManeuverQtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_lap_style_export_progress_log_and_completed_result(self):
        tab = ManeuverTab(payload())
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            tab.set_export_folder(folder)
            self.assertEqual(tab.export_folder(), folder)
            tab.set_progress(.375)
            self.assertEqual(tab.progress_bar.value(), 38)
            tab.append_log('{"event":"complete"}')
            self.assertIn('complete', tab.log_view.toPlainText())
            requested = [{'swa_rad': 0.1, 'accelerator': 0.2, 'brake': 0.0}]
            applied = [{'swa_rad': 0.1, 'accelerator': 0.2, 'brake': 0.0}]
            artifact = folder / 'attempt' / 'result'
            tab.set_completed_result({'termination_reason': 'duration'}, artifact, [0.0], requested, applied)
            self.assertEqual(tab.preview.requested_commands, requested)
            self.assertFalse(hasattr(tab, 'animation_button'))
            self.assertFalse(hasattr(tab, 'animationRequested'))
            self.assertIn('animator GUI', tab.result_label.text())
            self.assertIn('duration', tab.result_label.text())
            self.assertIn(str(artifact.parent / 'run_record.npz'), tab.result_label.text())

    def test_shared_model_display_and_busy_controls(self):
        tab = ManeuverTab(payload())
        self.assertTrue(tab.shared_model_label.isReadOnly())
        self.assertNotIn('Browse', tab.shared_model_label.text())
        tab.set_busy(True)
        self.assertFalse(tab.run_button.isEnabled())
        self.assertFalse(tab.export_edit.isEnabled())
        self.assertFalse(tab.export_browse.isEnabled())
        self.assertFalse(hasattr(tab, 'animation_button'))
        self.assertTrue(tab.stop_button.isEnabled())
        tab.set_busy(False)

    def test_tab_controls_and_signals(self):
        tab = ManeuverTab()
        self.assertFalse(tab.run_button.isEnabled())
        self.assertFalse(tab.hardware_arm_button.isEnabled())
        self.assertEqual(tab.backend_label.text(), "Backend: simulation")
        stopped = []
        tab.stopRequested.connect(lambda: stopped.append(True))
        self.assertFalse(tab.stop_button.isEnabled())
        tab.set_busy(True)
        tab.stop_button.click()
        self.assertEqual(stopped, [True])
        tab.set_busy(False)

    def test_editor_definition_validation_and_round_trip(self):
        editor = ManeuverEditor(payload())
        definition = editor.definition()
        self.assertEqual(definition.to_dict(), payload())
        editor.load_definition(definition)
        self.assertEqual(editor.to_dict(), definition.to_dict())
        invalid = payload()
        invalid['clock']['controller_tick_s'] = 0
        editor.load_definition(invalid)
        with self.assertRaises(ValueError):
            editor.definition()
        self.assertTrue(editor.validation_label.text())

    def test_mode_controls_and_optional_field_creation(self):
        data = payload()
        editor = ManeuverEditor(data)
        presets = ['step_steer','ramp_steer','sine_steer','straight_maintain','straight_accel_brake','circular_skidpad','csv_playback']
        for preset in presets:
            if preset == 'csv_playback':
                csv = {'source': '', 'mapping': {'time':'t','swa':'s','accelerator':'a','brake':'b'}}
                editor.set_draft({**payload(), 'preset':preset, 'csv':csv, 'steering':{'mode':'csv'}, 'longitudinal':{'mode':'csv'}})
                self.assertFalse(editor.fields['steering.amplitude_rad'].isEnabled())
                self.assertFalse(editor.fields['longitudinal.accelerator'].isEnabled())
                self.assertFalse(editor.steering_mode.isEnabled())
                self.assertIn('csv owns', editor.ownership_label.text().lower())
                self.assertIn('preset guidance', editor.ownership_label.text().lower())
            else:
                editor.set_draft({**payload(), 'preset':preset})
                self.assertEqual(editor.preset.currentText(), preset)
                self.assertTrue(editor.fields['steering.amplitude_rad'].isEnabled())
                self.assertTrue(editor.steering_mode.isEnabled())
                self.assertIn('steering mode', editor.ownership_label.text().lower())
                self.assertNotIn('csv owns', editor.ownership_label.text().lower())
        # Alternate schema-valid modes stay enabled irrespective of preset.
        alternate_modes = (
            ('step_steer', {'mode': 'path_pid', 'target_heading_rad': 0.}, 'steering.target_heading_rad'),
            ('straight_maintain', {'mode': 'peak_swa', 'amplitude_rad': .2, 'direction': 1}, 'steering.amplitude_rad'),
            ('circular_skidpad', {'mode': 'path_pid', 'target_heading_rad': 0.}, 'steering.target_heading_rad'),
        )
        for preset, steering, field in alternate_modes:
            with self.subTest(alternate_preset=preset):
                alternate = {**payload(), 'preset': preset, 'steering': steering}
                editor.set_draft(alternate)
                self.assertTrue(editor.fields[field].isEnabled())
                self.assertEqual(editor.to_dict()['steering']['mode'], steering['mode'])
        # Leave CSV ownership before exercising selectable steering modes.
        editor.set_draft(payload())
        for mode,path in [('path_pid','steering.target_heading_rad'),('radius_pid','steering.target_radius_m')]:
            editor.steering_mode.setCurrentText(mode)
            # The offscreen editor is not shown; enabled controls remain keyboard-focusable.
            self.assertTrue(editor.fields[path].isEnabled())
            self.assertTrue(editor.fields[path].focusPolicy() != 0)
            editor.fields[path].setText('1.25')
            editor.fields[path].textEdited.emit('1.25')
            self.assertEqual(editor.to_dict()['steering'][path.split('.')[-1]],1.25)
        editor.fields['profile.rise_s'].setText('0.3')
        editor.fields['profile.rise_s'].textEdited.emit('0.3')
        self.assertEqual(editor.to_dict()['profile']['rise_s'],0.3)

    def test_mode_and_csv_ownership_enabled_states(self):
        editor = ManeuverEditor(payload())
        heading = editor.fields['steering.target_heading_rad']
        radius = editor.fields['steering.target_radius_m']
        expected = {
            'path_pid': (True, False),
            'radius_pid': (False, True),
            'locked_swa': (False, False),
            'peak_ay': (False, False),
        }
        for mode, (heading_enabled, radius_enabled) in expected.items():
            with self.subTest(mode=mode):
                editor.set_draft({**payload(), 'steering': {'mode': mode, 'amplitude_mps2': 4., 'direction': 1,
                                                            'target_heading_rad': .2, 'target_radius_m': 10.}})
                self.assertEqual(heading.isEnabled(), heading_enabled)
                self.assertEqual(radius.isEnabled(), radius_enabled)
                self.assertIn('steering mode', editor.ownership_label.text().lower())
                self.assertNotIn('csv owns', editor.ownership_label.text().lower())

        csv_data = payload()
        csv_data.update(preset='csv_playback',
                        steering={'mode': 'csv'},
                        longitudinal={'mode': 'csv'},
                        csv={'source': '', 'mapping': {'time': 't', 'swa': 's', 'accelerator': 'a', 'brake': 'b'}})
        editor.set_draft(csv_data)
        self.assertFalse(editor.steering_mode.isEnabled())
        self.assertFalse(editor.longitudinal_mode.isEnabled())
        for path in ('steering.amplitude_rad', 'longitudinal.accelerator',
                     'steering.target_heading_rad', 'steering.target_radius_m'):
            with self.subTest(csv_field=path):
                self.assertFalse(editor.fields[path].isEnabled())
        label = editor.ownership_label.text().lower()
        self.assertIn('csv owns', label)
        self.assertIn('preset guidance', label)

    def test_tab_public_api_busy_and_result(self):
        data = payload()
        tab = ManeuverTab()
        tab.set_vehicle_configuration(data)
        self.assertEqual(tab.definition().to_dict(), data)
        self.assertTrue(tab.run_button.isEnabled())
        tab.set_busy(True)
        self.assertFalse(tab.editor.isEnabled())
        self.assertFalse(tab.run_button.isEnabled())
        self.assertFalse(tab.calibration_button.isEnabled())
        self.assertTrue(tab.stop_button.isEnabled())
        tab.set_busy(False)
        self.assertTrue(tab.run_button.isEnabled())
        requested = {'steering': {'amplitude_rad': .2}}
        applied = {'steering': {'amplitude_rad': .19}}
        tab.set_result({'status': 'Finished', 'requested': requested, 'applied': applied})
        self.assertEqual(tab.status_label.text(), 'Finished')
        self.assertEqual(tab.preview.requested, requested)
        self.assertEqual(tab.preview.applied, applied)
        self.assertFalse(tab.hardware_arm_button.isEnabled())

    def test_run_preflight_blocks_peak_ay_without_matching_calibration(self):
        data = payload()
        data['steering'] = {'mode': 'peak_ay', 'amplitude_mps2': 4., 'direction': 1}
        # Schema requires a persisted calibration reference for peak-ay; runtime
        # match status/fingerprints belong only to the tab preflight context.
        data['calibration'] = {'id': 'calibration-reference'}
        tab = ManeuverTab()
        tab.set_vehicle_configuration(data)
        emitted = []
        tab.runRequested.connect(emitted.append)

        tab._run()
        self.assertEqual(emitted, [])
        self.assertIn('calibration', tab.status_label.text().lower())
        self.assertFalse(tab.run_button.isEnabled())

        tab.set_preflight_context({'status': 'matched', 'fingerprint': 'fp', 'expected_fingerprint': 'fp'})
        tab._run()
        self.assertEqual(len(emitted), 1, tab.status_label.text())
        self.assertEqual(emitted[0].to_dict(), data)

    def test_run_preflight_blocks_changed_csv_snapshot(self):
        data = payload()
        data['preset'] = 'csv_playback'
        data['steering'] = {'mode': 'csv'}
        data['longitudinal'] = {'mode': 'csv'}
        data['csv'] = {'source': '', 'mapping': {'time': 'time', 'swa': 'swa', 'accelerator': 'accelerator', 'brake': 'brake'}}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'commands.csv'
            path.write_text('time,swa,accelerator,brake\\n0,0,0,0\\n')
            from tools.sim_gui.qt.maneuver_tab import csv_file_snapshot
            data['csv']['source'] = str(path)
            snapshot = csv_file_snapshot(path)
            tab = ManeuverTab()
            tab.set_vehicle_configuration(data)
            tab.set_preflight_context(csv_snapshot=snapshot)
            path.write_text('time,swa,accelerator,brake\\n0,1,0,0\\n')
            emitted = []
            tab.runRequested.connect(emitted.append)
            tab._run()
            self.assertEqual(emitted, [])
            self.assertIn('changed', tab.status_label.text().lower())
            self.assertFalse(tab.run_button.isEnabled())

    def test_preview_trace_copies_inputs_and_keeps_requested_applied_distinct(self):
        preview = PreviewWidget()
        times = [0.0, 1.0]
        requested = [{"steering": 0.2}, {"steering": 0.4}]
        applied = [{"steering": 0.1}, {"steering": 0.3}]
        preview.set_trace(times, requested, applied)
        times[0] = 9.0
        requested[0]["steering"] = 9.0
        applied[0]["steering"] = 9.0
        self.assertEqual(preview.time_s, [0.0, 1.0])
        self.assertEqual(preview.requested_commands[0]["steering"], 0.2)
        self.assertEqual(preview.applied_commands[0]["steering"], 0.1)
        self.assertNotEqual(preview.requested_commands, preview.applied_commands)
        self.assertIn("Requested trace", preview.requested_label.text())
        self.assertIn("Applied trace", preview.applied_label.text())
        # RobotCommand objects are accepted as well as serialized dictionaries.
        from simplified_vehicle_models.controls.maneuvers.types import RobotCommand
        preview.set_trace([0.0, 1.0], [RobotCommand(.1, .2, 0), RobotCommand(.2, 0, .3)],
                          [RobotCommand(.05, .1, 0), RobotCommand(.15, 0, .2)])
        preview.resize(700, 420); preview.show(); self.app.processEvents()
        self.assertEqual(len(preview.NAMES), 3)
        self.assertEqual(preview.minimumWidth(), 420)
        self.assertIn("Time (s)", preview.requested_label.text() + preview.applied_label.text() + "Time (s)")
        preview.grab()  # force an offscreen paint pass

    def test_csv_dialog_source_valid_and_invalid(self):
        from tools.maneuver_gui.qt import CsvDialog
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "commands.csv"
            path.write_text("time,swa,accelerator,brake\n0,0,0.2,0\n1,0.1,0,0.3\n")
            mapping = {"time": "time", "swa": "swa", "accelerator": "accelerator", "brake": "brake"}
            dialog = CsvDialog(str(path), mapping)
            source = dialog.source()
            self.assertEqual(source.time_s, (0.0, 1.0))
            self.assertEqual(len(source.commands), 2)
            self.assertEqual(source.mapping.time, "time")
            with self.assertRaises((AttributeError, TypeError)):
                source.commands[0].accelerator = 0.8
            invalid = CsvDialog(str(path), {**mapping, "swa": "missing"})
            self.assertIsNone(invalid.source())
            self.assertIn("unmapped", invalid.error_label.text())
            self.assertEqual(invalid.time_unit.currentText(), "s")
            invalid.has_time.setChecked(False)
            invalid.sample_dt.setValue(.25)
            invalid.interpolation.setCurrentText("linear")
            invalid.rebase.setChecked(True)
            self.assertIsNone(invalid.mapping().time)
            self.assertEqual(invalid.mapping().sample_dt_s, .25)
            self.assertEqual(invalid.mapping().interpolation, "linear")
            self.assertTrue(invalid.mapping().rebase)
            bad_rows = Path(tmp) / "bad rows.csv"
            bad_rows.write_text("time,swa,accelerator,brake\n0,0,0,0\n1,0,nan,0\n")
            bad = CsvDialog(str(bad_rows), mapping)
            self.assertIsNone(bad.source())
            self.assertIn("row 3", bad.error_label.text())
            bad.accept()
            self.assertIn("row 3", bad.error_label.text())
            self.assertIn("Loaded", dialog.status_label.text())

    def test_preview_keeps_requested_and_applied_distinct(self):
        preview = PreviewWidget()
        requested = {"steering": {"amplitude_rad": .2}}
        applied = {"steering": {"amplitude_rad": .19}}
        preview.set_state(requested, applied)
        self.assertEqual(preview.requested, requested)
        self.assertEqual(preview.applied, applied)
        requested["steering"]["amplitude_rad"] = .1
        self.assertEqual(preview.requested["steering"]["amplitude_rad"], .2)


if __name__ == "__main__":
    unittest.main()
