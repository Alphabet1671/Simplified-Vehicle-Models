import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

try:
    from PyQt6.QtWidgets import QApplication, QMessageBox
    from tools.tire_gui.qt.main_window import MainWindow
except ImportError as exc:
    import unittest
    raise unittest.SkipTest(f'optional tire GUI dependencies unavailable: {exc}')

from pathlib import Path
import unittest
import tempfile
import time
from tools.tire_gui.model import with_parameter
from tools.tire_gui.contracts import FitResult
from tools.tire_gui.qt.fit_worker import FitWorker
from PyQt6.QtTest import QSignalSpy
from PyQt6.QtCore import QThread, QTimer, QEventLoop, Qt
from tools.tire_gui.contracts import FitCancelled, RunData
from tools.tire_gui.events import TransientEvent
from tools.tire_gui.save import save_session
import numpy as np


class QtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def window(self):
        w = MainWindow(settings_store=None)
        self.assertTrue(w.open_path(Path('models/tires/hoosier-43075-r20-simple.xml')))
        return w

    def test_numeric_edit_and_stale_candidate_refused(self):
        w = self.window(); old = w.snapshot
        w.set_parameter('mu_0_y', old.values['mu_0_y'] + .1)
        candidate = FitResult(old.revision, old, {}, True, (), {})
        self.assertFalse(w.apply_candidate(candidate)); w.dirty = False; w.close()

    def test_dirty_close_can_be_cancelled(self):
        from unittest.mock import patch
        w = self.window()
        w.show()
        w.dirty = True
        with patch('tools.tire_gui.qt.main_window.QMessageBox.question', return_value=QMessageBox.StandardButton.No):
            self.assertFalse(w.close())
        self.assertTrue(w.isVisible())
        self.assertTrue(w.dirty)
        with patch('tools.tire_gui.qt.main_window.QMessageBox.question', return_value=QMessageBox.StandardButton.Yes):
            self.assertTrue(w.close())

    def test_bounds_locks_history_and_candidate(self):
        w=self.window(); old=w.snapshot
        w.set_parameter('mu_0_y',old.values['mu_0_y']+.1)
        self.assertTrue(w.undo()); self.assertEqual(w.snapshot.values['mu_0_y'],old.values['mu_0_y'])
        self.assertTrue(w.redo()); self.assertNotEqual(w.snapshot.values['mu_0_y'],old.values['mu_0_y'])
        current=w.snapshot; candidate=FitResult(current.revision,with_parameter(current,'mu_0_y',current.values['mu_0_y']+.02),{},True,(),{})
        w._fit_generation=w.generation
        self.assertTrue(w.apply_candidate(candidate)); w.dirty = False; w.close()

    def test_tabs_and_missing_data(self):
        w=self.window(); self.assertGreaterEqual(w.tabs.count(),3)
        with self.assertRaises(Exception):
            w.load_data_path(Path('/definitely/missing/run.mat'))
        self.assertFalse(w.fit_longitudinal_button.isEnabled()); w.dirty = False; w.close()

    def test_spinbox_slider_locks_and_editable_bounds(self):
        w = self.window()
        spin, slider, lock, lower, upper = w.parameter_panel.controls['mu_0_y']
        edits = QSignalSpy(w.parameter_panel.parameterChanged)
        spin.setValue(spin.value() + .125)
        self.assertEqual(len(edits), 1)
        self.assertEqual(slider.value(), round((spin.value()-lower.value())/(upper.value()-lower.value())*10000))
        slider.setValue(2500)
        self.assertAlmostEqual(spin.value(), lower.value()+.25*(upper.value()-lower.value()), places=10)
        lock.setChecked(not lock.isChecked())
        self.assertIn('mu_0_y', w.parameter_panel.locked())
        lower.setValue(lower.value() + .01)
        self.assertAlmostEqual(w.parameter_panel.bounds()['mu_0_y'][0], lower.value())
        w.dirty = False
        w.close()

    def test_real_compatibility_segment_holdout_and_load_filter(self):
        from tools.tire_gui.segments import compatibility_selection
        w = self.window()
        def fake_run(run_id, tire, count):
            z = np.zeros(count)
            channels = {'alpha': z, 'kappa': z, 'fz': z + 600, 'fy': z, 'fx': z, 'mz': z,
                        'time': np.arange(count, dtype=float) * .01}
            return RunData(run_id, tire, channels, {}, np.arange(count))
        selection = compatibility_selection(fake_run(8, '43075 R20', 21168), fake_run(72, '43100 R20', 25992))
        w.full_selection = selection
        w._populate_load_filter()
        w._populate_compatibility_segments()
        identities = [w.segment_list.item(i).data(Qt.ItemDataRole.UserRole) for i in range(w.segment_list.count())]
        self.assertIn(('run72_combined', 0), identities)
        self.assertIn(('run72_sa0_fx', 0), identities)
        self.assertIn(('run8', 0), identities)
        item = next(w.segment_list.item(i) for i in range(w.segment_list.count()) if w.segment_list.item(i).data(Qt.ItemDataRole.UserRole) == ('run72_combined', 0))
        item.setCheckState(Qt.CheckState.Checked)
        self.assertEqual(len(w.selection.combined_fx.target), len(selection.combined_fx.target) - 180)
        self.assertEqual(len(w.selection.combined_fy.target), len(selection.combined_fy.target) - 180)
        self.assertEqual(len(w.holdout_selection.combined_fx.target), 180)
        self.assertEqual(len(w.holdout_selection.combined_fy.target), 180)
        self.assertEqual(len(w.selection.longitudinal.target), len(selection.longitudinal.target))
        w._refresh_diagnostics(FitResult(w.snapshot.revision, w.snapshot, {}, True, (), {}))
        diagnostic_text = w.diagnostics.toPlainText()
        self.assertIn('combined Fx:', diagnostic_text)
        self.assertIn('combined Fy:', diagnostic_text)
        self.assertIn('n=180', diagnostic_text)
        load_ix = w.load_filter.findData(660.0)
        self.assertGreaterEqual(load_ix, 0)
        w.load_filter.setCurrentIndex(load_ix)
        self.assertTrue(np.all(w.selection.lateral.fz == 660.0))
        self.assertTrue(np.all(w.active_selection.combined_fx.fz == 660.0))
        w.dirty = False
        w.close()

    def test_legacy_holdout_session_migrates_all_matching_groups(self):
        from unittest.mock import patch
        from tools.tire_gui.segments import compatibility_selection
        w = self.window()
        def fake_run(run_id, tire, count):
            z = np.zeros(count)
            return RunData(run_id, tire, {'alpha':z,'kappa':z,'fz':z+600,'fy':z,'fx':z,'mz':z,'time':np.arange(count)*.01}, {}, np.arange(count))
        selection = compatibility_selection(fake_run(8,'43075 R20',21168), fake_run(72,'43100 R20',25992))
        with tempfile.TemporaryDirectory() as folder:
            save_session(w.snapshot, Path(folder), 'legacy', include_transient=True,
                         metadata={'selections': {'holdout_ids': [[72, 0]]}})
            bundle = Path(folder) / 'legacy'
            manifest = bundle / 'session.json'
            import json
            data = json.loads(manifest.read_text())
            data['selections']['holdout_ids'] = [[72, 0]]
            manifest.write_text(json.dumps(data))
            # Exercise legacy metadata through actual bundle validation/loading.
            w.load_session_path(bundle)
            w.runs = {8: fake_run(8,'43075 R20',21168), 72: fake_run(72,'43100 R20',25992)}
            self.assertIn(('run72_sa0_fx', 0), w.holdout_ids)
            self.assertIn(('run72_combined', 0), w.holdout_ids)
            w.full_selection = selection
            w._apply_selection_filters()
            self.assertEqual(len(w.selection.longitudinal.target), len(selection.longitudinal.target)-180)
            self.assertEqual(len(w.selection.combined_fx.target), len(selection.combined_fx.target)-180)
            self.assertEqual(len(w.holdout_selection.longitudinal.target), 180)
            self.assertEqual(len(w.holdout_selection.combined_fy.target), 180)
            self.assertIn('Legacy run/segment holdouts were migrated', w.status.text())
        w.dirty = False
        w.close()

    def test_alternate_output_cancel_discloses_privacy_without_writing(self):
        from unittest.mock import patch
        w = self.window()
        with patch('tools.tire_gui.qt.main_window.QFileDialog.getExistingDirectory', return_value='/tmp'), patch('tools.tire_gui.qt.main_window.QInputDialog.getText', return_value=('private-test', True)), patch('tools.tire_gui.qt.main_window.QMessageBox.question', return_value=QMessageBox.StandardButton.No) as question, patch.object(w, 'save_session') as save:
            w._save_dialog()
        self.assertIn('privacy', question.call_args.args[2].lower())
        save.assert_not_called()
        w.dirty = False
        w.close()

    def test_settings_restore_parameter_controls(self):
        from tools.tire_gui.model import load_snapshot
        from tools.tire_gui.qt.parameter_panel import ParameterPanel
        panel = ParameterPanel()
        snap = load_snapshot(Path('models/tires/hoosier-43075-r20-simple.xml'))
        from dataclasses import replace
        snap = replace(snap, session_metadata={'locks': {'mu_0_y': True}, 'bounds': {'mu_0_y': [.7, 2.2]}})
        panel.set_snapshot(snap)
        spin, slider, lock, lower, upper = panel.controls['mu_0_y']
        self.assertTrue(lock.isChecked())
        self.assertEqual((lower.value(), upper.value()), (.7, 2.2))
        panel.deleteLater()

    def test_real_thread_cancel_and_close_teardown(self):
        w = self.window()
        entered = __import__('threading').Event()
        def runner(_snapshot, cancelled):
            entered.set()
            while not cancelled():
                time.sleep(.005)
            raise FitCancelled()
        w.worker_runner = runner
        w._start_worker('steady', None, ())
        self.assertTrue(entered.wait(2))
        self.assertTrue(w.thread.isRunning())
        w.cancel_fit()
        finished = QEventLoop()
        w.thread.finished.connect(finished.quit)
        QTimer.singleShot(3000, finished.quit)
        finished.exec()
        self.app.processEvents()
        self.assertIsNone(w.thread)
        self.assertIsNone(w.worker)
        w.dirty = False
        w.close()

    def test_edited_event_boundaries_restore_exactly_from_session(self):
        w = self.window()
        n = 80
        t = np.arange(n) * .01
        z = np.zeros(n)
        run = RunData(7, 'synthetic', {'time':t,'alpha':z,'kappa':z,'fz':z+600,'speed':z+10,'omega':z+30,'ia':z,'pressure':z+80000,'fx':z,'fy':z,'mz':z}, {}, np.arange(n))
        w.runs[7] = run
        w.source_files.append('/missing/run7.mat')
        w.events = [TransientEvent(run, 10, 70, 30, False, 'lateral', ())]
        with tempfile.TemporaryDirectory() as folder:
            w.save_session(folder, 'event-test')
            from tools.tire_gui.save import load_session
            saved = load_session(Path(folder)/'event-test').session_metadata['events'][0]
        self.assertEqual((saved['start'], saved['stop'], saved['baseline_stop']), (10,70,30))
        self.assertEqual(saved['source_file'], '/missing/run7.mat')
        w.dirty = False
        w.close()

    def test_real_thread_result_stale_after_generation_edit(self):
        w = self.window()
        entered = __import__('threading').Event()
        release = __import__('threading').Event()
        outcomes = []
        def runner(snapshot, _cancelled):
            entered.set()
            release.wait(2)
            return FitResult(snapshot.revision, snapshot, {}, True, (), {})
        w.worker_runner = runner
        w._fit_done = lambda result: outcomes.append(w._fit_generation == w.generation and result.source_revision == w.snapshot.revision)
        w._start_worker('steady', None, ())
        self.assertTrue(entered.wait(2))
        w.generation += 1
        release.set()
        finished = QEventLoop()
        w.thread.finished.connect(finished.quit)
        QTimer.singleShot(3000, finished.quit)
        finished.exec()
        self.app.processEvents()
        self.assertEqual(outcomes, [False])
        w.dirty = False
        w.close()
