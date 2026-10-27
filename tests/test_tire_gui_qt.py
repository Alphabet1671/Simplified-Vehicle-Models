import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

try:
    from PyQt6.QtWidgets import QApplication
    from tools.tire_gui.qt.main_window import MainWindow
except ImportError as exc:
    import unittest
    raise unittest.SkipTest(f'optional tire GUI dependencies unavailable: {exc}')

from pathlib import Path
import unittest
from tools.tire_gui.model import with_parameter
from tools.tire_gui.contracts import FitResult
from tools.tire_gui.qt.fit_worker import FitWorker
from PyQt6.QtTest import QSignalSpy


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
        self.assertFalse(w.apply_candidate(candidate)); w.close()

    def test_bounds_locks_history_and_candidate(self):
        w=self.window(); old=w.snapshot
        w.set_parameter('mu_0_y',old.values['mu_0_y']+.1)
        self.assertTrue(w.undo()); self.assertEqual(w.snapshot.values['mu_0_y'],old.values['mu_0_y'])
        self.assertTrue(w.redo()); self.assertNotEqual(w.snapshot.values['mu_0_y'],old.values['mu_0_y'])
        current=w.snapshot; candidate=FitResult(current.revision,with_parameter(current,'mu_0_y',current.values['mu_0_y']+.02),{},True,(),{})
        w._fit_generation=w.generation
        self.assertTrue(w.apply_candidate(candidate)); w.close()

    def test_tabs_and_missing_data(self):
        w=self.window(); self.assertGreaterEqual(w.tabs.count(),3)
        with self.assertRaises(Exception):
            w.load_data_path(Path('/definitely/missing/run.mat'))
        self.assertFalse(w.fit_longitudinal_button.isEnabled()); w.close()

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

    def test_worker_cancel_signal_and_thread_close(self):
        w = self.window()
        worker = FitWorker(w.snapshot, 'steady', None)
        worker.cancel()
        self.assertTrue(worker._cancel.is_set())
        w.dirty = False
        w.close()
