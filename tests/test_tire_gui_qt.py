import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

try:
    from PyQt6.QtWidgets import QApplication, QMessageBox, QDoubleSpinBox, QLabel
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

    def test_open_transient_xml_replaces_current_steady_and_selects_transient_tab(self):
        w = self.window()
        w.set_parameter('mu_0_y', 1.23456789)
        steady = w.snapshot.steady
        before = w.generation
        w.candidate = FitResult(w.snapshot.revision, w.snapshot, {}, True, (), {})
        path = Path('models/tires/iem24_murphy_real_unitire_transient.xml')
        self.assertTrue(w.open_path(path), w.status.text())
        self.assertNotEqual(w.snapshot.steady, steady)
        self.assertEqual(w.snapshot.values['trail_lead_ratio'], .25)
        self.assertEqual(w.tabs.currentWidget(), w.transient_plot)
        self.assertIsNone(w.candidate)
        self.assertGreater(w.generation, before)
        self.assertIn(str(path.resolve()), w.source_files)
        self.assertFalse(w.dirty)
        w.dirty = False
        w.close()

    def test_open_transient_first_never_prompts_for_companion(self):
        from unittest.mock import patch
        w = MainWindow()
        path = Path('models/tires/iem24_murphy_real_unitire_transient.xml')
        with patch('tools.tire_gui.qt.main_window.QFileDialog.getOpenFileName', side_effect=AssertionError('unexpected companion prompt')):
            self.assertTrue(w.open_path(path), w.status.text())
        self.assertIsNotNone(w.snapshot)
        self.assertIn('provisional', w.status.text())
        self.assertFalse(w.dirty)
        w.close()

    def test_bad_transient_or_wrong_companion_does_not_replace_live_model(self):
        w = self.window()
        snapshot = w.snapshot
        path = Path('models/tires/iem24_murphy_real_unitire_transient.xml')
        self.assertFalse(w.open_path(path, steady_path=path))
        self.assertIs(w.snapshot, snapshot)
        self.assertIn('Open failed', w.status.text())
        w.close()

    def test_pending_steady_candidate_plots_without_live_mutation(self):
        w = self.window()
        from tools.tire_gui.model import with_parameters
        old = w.snapshot
        candidate_model = with_parameters(old, {'mu_0_y': old.values['mu_0_y'] * .9})
        result = FitResult(old.revision, candidate_model, {}, True, (), {'mu_0_y': 'fitted'})
        w._fit_generation = w.generation
        w._fit_mode = 'steady'
        w._fit_selection = None
        w.candidate = None
        w._fit_selection = None
        from tools.tire_gui.segments import compatibility_selection
        def run(run_id, tire, count):
            z = np.zeros(count)
            channels = {'alpha': z, 'kappa': z, 'fz': z + 600, 'fy': z, 'fx': z, 'mz': z, 'time': np.arange(count) * .01}
            return RunData(run_id, tire, channels, {}, np.arange(count))
        w.runs = {8: run(8, '43075 R20', 21168), 72: run(72, '43100 R20', 25992)}
        w.full_selection = compatibility_selection(w.runs[8], w.runs[72])
        w._populate_load_filter(); w._populate_compatibility_segments(); w._apply_selection_filters()
        w._fit_selection = w.selection
        w._fit_done(result)
        self.assertIs(w.snapshot, old)
        self.assertEqual(dict(w.snapshot.values), dict(old.values))
        self.assertTrue(any(line.get_label() == 'Pending candidate' for line in w.steady_plot.axes[0].lines))
        self.assertIn('mu_0_y:', w.diagnostics.toPlainText())
        self.assertIn('Pending candidate parameters', w.diagnostics.toPlainText())
        candidate_line = next(line for line in w.steady_plot.axes[0].lines if line.get_label() == 'Pending candidate')
        self.assertEqual(len(candidate_line.get_ydata()), 121)
        # Every model has one line per selected segment, never one joined line.
        segment_count = len(np.unique(w.selection.lateral.segment_ids))
        self.assertEqual(len(w.steady_plot.axes[0].lines), 3 * segment_count)
        w.discard_candidate()
        self.assertFalse(any(line.get_label() == 'Pending candidate' for line in w.steady_plot.axes[0].lines))
        self.assertIs(w.snapshot, old)
        w.dirty = False; w.close()

    def test_curves_are_independent_and_combined_axes_are_model_predictions(self):
        from tools.tire_gui.contracts import SampleSet, SteadySelection
        from tools.tire_gui.model import predict_steady, with_parameters
        from tools.tire_gui.plotting import model_sweeps
        w = self.window()
        alpha = np.tile([-.1, 0., .1], 2)
        kappa = np.tile([-.2, 0., .2], 2)
        loads = np.repeat([400., 800.], 3)
        segments = np.repeat([0, 1], 3)
        def samples(a, k, target):
            return SampleSet(a, k, loads, np.full(6, target), segments, raw_indices=np.arange(6))
        lat = samples(alpha, np.zeros(6), 123.)
        lon = samples(np.zeros(6), kappa, 456.)
        combined = samples(np.repeat([.05, .1], 3), kappa, 987.)
        fy = samples(combined.alpha, kappa, -321.)
        data = SteadySelection(lat, lon, combined, fy, lat, 40.193 / 3.6)
        w.runs = {8: object(), 72: object()}
        w.full_selection = w.active_selection = data
        candidate = with_parameters(w.snapshot, {'mu_0_y': w.snapshot.values['mu_0_y'] * .8})
        w.candidate = FitResult(w.snapshot.revision, candidate, {}, True)
        w._fit_mode, w._fit_selection = 'steady', data
        w._refresh_plots()
        models = [w.snapshot, candidate, w.seed]
        for axis, source, variable, component in [
            (w.steady_plot.axes[0], lat, 'alpha', 'lateral_force'),
            (w.steady_plot.axes[1], lon, 'kappa', 'longitudinal_force'),
            (w.steady_plot.axes[2], combined, 'kappa', 'longitudinal_force'),
            (w.steady_plot.axes[3], lat, 'alpha', 'aligning_moment'),
        ]:
            self.assertEqual(len(axis.lines), 6)
            for group, sweep in enumerate(model_sweeps(source, variable)):
                for model_index, model in enumerate(models):
                    pred = predict_steady(model, sweep, data.belt_speed)
                    line = axis.lines[group * 3 + model_index]
                    if axis is w.steady_plot.axes[2]:
                        np.testing.assert_allclose(line.get_xdata(), pred.lateral_force / sweep.fz)
                        np.testing.assert_allclose(line.get_ydata(), pred.longitudinal_force / sweep.fz)
                    else:
                        np.testing.assert_allclose(line.get_xdata(), getattr(sweep, variable))
                        np.testing.assert_allclose(line.get_ydata(), getattr(pred, component))
            self.assertEqual([c.get_label() for c in axis.collections], ['Measured', 'Residual'])
        w.dirty = False
        w.close()

    def test_transient_tabs_share_input_alignment_and_correct_force_columns(self):
        from tools.tire_gui.transient_plotting import RampGroup, aligned_time
        from tools.tire_gui.qt.transient_panel import TransientPanel
        n = 121
        t = np.arange(n) * .01
        z = np.zeros(n)
        a = np.deg2rad(np.clip((t-.2)*10, 0, 6))
        run = RunData(9, 'synthetic', dict(time=t, alpha=a, kappa=z, fz=z+600,
                      speed=z+11, omega=z+30, ia=z, pressure=z+80000,
                      fx=z+123, fy=z+456), {}, np.arange(n))
        event = TransientEvent(run, 0, n, 20)
        group = RampGroup((event,), 11., .6, 10., 1, 'lateral')
        panel = TransientPanel()
        panel.show_groups((group,))
        predictions = np.column_stack((z+1000, z+2000, z+3000))
        panel.add_replay(event, (predictions, predictions * 99))
        panel.add_replay(event, (predictions, predictions * 99))
        for component, measured, model in (('fy', 456, 2000),):
            axis = panel.panels[component].axes[0]
            self.assertEqual(len(axis.lines), 2)
            np.testing.assert_allclose(axis.lines[0].get_ydata(), measured)
            np.testing.assert_allclose(axis.lines[1].get_ydata(), model)
            np.testing.assert_allclose(axis.lines[0].get_xdata(), aligned_time(event))
            np.testing.assert_allclose(axis.lines[1].get_xdata(), aligned_time(event))
            command = panel.panels[component].figure.axes[1].lines[0]
            np.testing.assert_allclose(command.get_xdata(), aligned_time(event))
            np.testing.assert_allclose(command.get_ydata(), np.rad2deg(a))
        panel.deleteLater()

    def test_transient_commands_are_labeled_and_unused_axes_removed(self):
        from tools.tire_gui.transient_plotting import RampGroup
        from tools.tire_gui.qt.transient_panel import TransientPanel
        n = 60
        t = np.arange(n) * .01
        z = np.zeros(n)
        run = RunData(9, 'synthetic', dict(time=t, alpha=z, kappa=np.linspace(0, .2, n), fz=z+600,
                      speed=z+11, omega=z+30, ia=z, pressure=z+80000, fx=z, fy=z), {}, np.arange(n))
        events = tuple(TransientEvent(run, 0, n, 10, axis='longitudinal') for _ in range(3))
        groups = tuple(RampGroup((event,), 11., .5, .4, 1, 'longitudinal') for event in events)
        panel = TransientPanel()
        panel.show_groups(groups)
        self.assertEqual(len(panel.panels['fy'].axes), 3)
        self.assertEqual(len(panel.panels['fy'].figure.axes), 6)
        legend_text = [text.get_text() for text in panel.panels['fy'].axes[0].get_legend().get_texts()]
        self.assertTrue(any('SA command' in text for text in legend_text))
        self.assertTrue(any('slip command' in text for text in legend_text))
        self.assertIn('SA 0.00°', panel.panels['fy'].axes[0].get_title())
        panel.add_replay(events[0], (np.zeros((n,3)), None))
        panel.group_pages['fy'].setCurrentIndex(1) if panel.group_pages['fy'].count() > 1 else None
        panel.deleteLater()

    def test_stale_transient_replay_does_not_enter_current_cache(self):
        w = self.window()
        key = ('old',)
        w._replay_done(key, (w.generation-1, np.zeros((2,3)), None))
        self.assertNotIn(key, w.replay_cache)
        w.close()

    def test_replay_cache_prunes_unshown_conditions_and_source_tokens_do_not_collide(self):
        w = self.window()
        class E:
            def __init__(self, run):
                self.run, self.start, self.stop, self.baseline_stop, self.axis = run, 0, 2, 1, 'lateral'
        class R:
            def __init__(self, run_id): self.run_id = run_id
        seven, seventy_two = R(7), R(72)
        w.run_sources = {7: '/tmp/run7.mat', 72: '/tmp/run72.mat'}
        self.assertEqual(w._source_for_run(seven), '/tmp/run7.mat')
        self.assertEqual(w._source_for_run(seventy_two), '/tmp/run72.mat')
        w._display_replay_keys = {('shown',): None}
        w.replay_cache = {('shown',): ('ok',), ('old-generation',): ('stale',)}
        w.replay_failures = {('shown',), ('old-generation',)}
        w._prune_replay_cache()
        self.assertEqual(set(w.replay_cache), {('shown',)})
        self.assertEqual(w.replay_failures, {('shown',)})
        w.close()

    def test_sidebar_separates_coefficients_from_data_controls(self):
        w = self.window()
        self.assertEqual([w.sidebar_tabs.tabText(i) for i in range(w.sidebar_tabs.count())],
                         ['Coefficients', 'Data & fitting'])
        self.assertTrue(w.coefficients_page.isAncestorOf(w.parameter_panel))
        self.assertFalse(w.data_controls_page.isAncestorOf(w.parameter_panel))
        for control in (w.run_combo, w.speed_source, w.transient_pressure,
                        w.load_filter, w.segment_list, w.event_list,
                        w.event_from, w.event_to, w.baseline_to, w.approve_button):
            self.assertTrue(w.data_controls_page.isAncestorOf(control))
            self.assertFalse(w.coefficients_page.isAncestorOf(control))
        self.assertEqual(w.sidebar_tabs.currentWidget(), w.coefficients_page)
        w.sidebar_tabs.setCurrentWidget(w.data_controls_page)
        self.assertIs(w.sidebar_tabs.currentWidget(), w.data_controls_page)
        w.close()

    def test_long_sliders_and_wide_sidebar(self):
        w = self.window()
        self.assertGreaterEqual(w.main_splitter.widget(0).minimumWidth(), 620)
        for _, slider, _, _, _ in w.parameter_panel.controls.values():
            self.assertGreaterEqual(slider.minimumWidth(), 480)
        w.close()

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
        w.open_path(Path('models/tires/hoosier-43075-r20-simple.xml'))
        self.assertTrue(any(line.get_label() == 'Current fit' for line in w.steady_plot.axes[0].lines))
        self.assertTrue(any('No measurements — model-only sweep' in text.get_text() for text in w.steady_plot.axes[0].texts))
        combined = w.steady_plot.axes[2].lines[0]
        self.assertGreater(np.max(np.abs(combined.get_xdata())), 0.01)
        self.assertGreater(np.ptp(combined.get_ydata()), 0.01)
        with self.assertRaises(Exception):
            w.load_data_path(Path('/definitely/missing/run.mat'))
        self.assertFalse(w.fit_longitudinal_button.isEnabled()); w.dirty = False; w.close()

    def test_parameter_tabs_rebuild_without_duplicate_controls_and_plots_use_two_rows(self):
        from tools.tire_gui.qt.parameter_panel import ParameterPanel
        panel = ParameterPanel()
        source_window = self.window()
        first = source_window.snapshot
        panel.set_snapshot(first)
        self.assertEqual([panel.tabs.tabText(i) for i in range(panel.tabs.count())],
                         ['Mu', 'Stiffness factors', 'Shape factors', 'Misc'])
        self.assertEqual(set(panel.controls), set(first.values))
        self.assertEqual(sum(len(page.findChildren(QDoubleSpinBox)) for page in panel.pages),
                         3 * len(first.values))
        self.assertEqual(sum(len(page.findChildren(QLabel)) for page in panel.pages),
                         3 * len(first.values))
        panel.tabs.setCurrentIndex(2)
        panel.set_snapshot(first)
        self.assertEqual(panel.tabs.currentIndex(), 2)
        self.assertEqual(set(panel.controls), set(first.values))
        self.assertEqual([layout.count() for layout in panel.group_layouts],
                         [sum(1 for name in first.values if panel.parameter_group(name) == i)
                          for i in range(4)])
        w = self.window()
        self.assertEqual(w.transient_plot.count(), 1)
        self.assertEqual(set(w.transient_plot.panels), {'fy'})
        self.assertIn('Lateral', w.transient_plot.tabText(0))
        for plot, count in ((w.steady_plot, 4),):
            self.assertEqual(len(plot.axes), count)
            self.assertEqual(plot.axes[0].get_subplotspec().get_gridspec().nrows, 2)
            self.assertEqual(plot.axes[0].get_subplotspec().get_gridspec().ncols, count // 2)
        panel.deleteLater()
        w.dirty = False
        w.close()
        source_window.dirty = False
        source_window.close()

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
        edits_before_bounds = len(edits)
        lower.setValue(lower.value() + .01)
        self.assertEqual(len(edits), edits_before_bounds)
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

    def test_transient_display_uses_isolated_step_selector_not_sweep_detector(self):
        from unittest.mock import patch
        w = self.window()
        run = object()
        w.runs = {7: run}
        with patch('tools.tire_gui.transient_plotting.isolated_step_events', return_value=()) as select:
            w._rebuild_ramps()
        select.assert_called_once_with(run)
        self.assertEqual(w.ramp_events, [])
        w.runs = {}
        w.close()

    def test_restored_transient_events_are_scoped_to_selected_run(self):
        w = self.window()
        n = 80
        t = np.arange(n) * .01
        z = np.zeros(n)
        def run(run_id):
            channels = {'time':t, 'alpha':z, 'kappa':z, 'fz':z+600, 'speed':z+10,
                        'omega':z+30, 'ia':z, 'pressure':z+80000, 'fx':z, 'fy':z, 'mz':z}
            return RunData(run_id, 'synthetic', channels, {}, np.arange(n))
        run7, run9 = run(7), run(9)
        event7 = TransientEvent(run7, 0, 70, 30, False, 'lateral', ())
        event9 = TransientEvent(run9, 0, 70, 30, True, 'longitudinal', ())
        w.runs = {7:run7, 9:run9}
        w.ramp_events = [event7, event9]
        w.run_sources = {7:'/tmp/run7.mat', 9:'/tmp/run9.mat'}
        w.run_combo.clear(); w._run_label(run7); w._run_label(run9)
        w._restored_events = [
            {'run_id':7,'start':0,'stop':70,'baseline_stop':30,'approved':False,'axis':'lateral','source_file':'/tmp/run7.mat'},
            {'run_id':9,'start':0,'stop':70,'baseline_stop':30,'approved':True,'axis':'longitudinal','source_file':'/tmp/run9.mat'},
        ]
        w._refresh_plots = lambda: None
        w.run_combo.setCurrentIndex(w.run_combo.findData(9))
        w._refresh_run()
        self.assertEqual({event.run.run_id for event in w.events}, {9})
        self.assertTrue(w.events[0].approved)
        self.assertEqual(w.events[0].axis, 'longitudinal')
        w.run_combo.setCurrentIndex(w.run_combo.findData(7))
        w._refresh_run()
        self.assertEqual({event.run.run_id for event in w.events}, {7})
        self.assertFalse(w.events[0].approved)
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
