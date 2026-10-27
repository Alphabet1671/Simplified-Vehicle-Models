"""Native PyQt interface for TTC tire correlation."""
from dataclasses import replace
from pathlib import Path
import numpy as np

from PyQt6.QtCore import QSettings, QTimer, QThread, Qt
from PyQt6.QtWidgets import (
    QCheckBox, QComboBox, QFileDialog, QHBoxLayout, QInputDialog, QLabel,
    QListWidget, QListWidgetItem, QMainWindow, QMessageBox, QPlainTextEdit,
    QProgressBar, QPushButton, QSpinBox, QSplitter, QTabWidget, QVBoxLayout,
    QWidget,
)

from tools.gui_common import REPOSITORY_ROOT
from ..contracts import FitResult
from ..data import load_run
from ..events import candidate_events, validate_event
from ..model import import_mat_seed, load_snapshot, with_parameter
from ..save import load_session, save_session
from ..segments import compatibility_selection
from .fit_worker import FitWorker
from .parameter_panel import ParameterPanel
from .plot_panel import PlotPanel
from .replay_worker import ReplayWorker

DEFAULT_DATA = REPOSITORY_ROOT / 'private/ttc/TTC'
DEFAULT_OUTPUT = REPOSITORY_ROOT / 'tire_fits_out'


class MainWindow(QMainWindow):
    def __init__(self, settings_store=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Tire correlation — 43075 lateral / 43100 longitudinal proxy')
        self.settings = settings_store or QSettings('Simplified-Vehicle-Models', 'tire-gui')
        self.snapshot = self.seed = None
        self.runs = {}
        self.events = []
        self.replay_cache = {}
        self.replay_failures = set()
        self.replay_thread = self.replay_worker = None
        self.generation = 0
        self.worker = self.thread = self.candidate = None
        self.history = []
        self.history_index = -1
        self.source_files = []
        self.selection = None
        self.holdout_ids = set()
        self.active_selection = None
        self.full_selection = None
        self.dirty = False

        root = QWidget(self)
        layout = QVBoxLayout(root)
        bar = QHBoxLayout()
        self.open_button = QPushButton('Open XML')
        self.seed_button = QPushButton('Import MATLAB seed')
        self.data_button = QPushButton('Data folder')
        self.session_button = QPushButton('Restore session…')
        self.save_button = QPushButton('Save fit as…')
        for button in (self.open_button, self.seed_button, self.data_button, self.session_button, self.save_button):
            bar.addWidget(button)
        layout.addLayout(bar)

        self.tabs = QTabWidget()
        self.steady_plot = PlotPanel(count=4)
        self.transient_plot = PlotPanel(count=6)
        self.diagnostics = QPlainTextEdit()
        self.diagnostics.setReadOnly(True)
        self.tabs.addTab(self.steady_plot, 'Steady-state')
        self.tabs.addTab(self.transient_plot, 'Transient')
        self.tabs.addTab(self.diagnostics, 'Fit diagnostics')

        side = QWidget()
        side_layout = QVBoxLayout(side)
        side_layout.addWidget(QLabel('Model coefficients • finite engineering bounds • locks'))
        self.parameter_panel = ParameterPanel()
        side_layout.addWidget(self.parameter_panel)
        self.run_combo = QComboBox()
        self.run_combo.setToolTip('Select the run whose transient events are displayed; steady fitting always uses the fixed 43075/43100 compatibility selection.')
        self.segment_list = QListWidget()
        self.load_filter = QComboBox()
        self.load_filter.addItem('All compatibility loads', None)
        self.load_filter.currentIndexChanged.connect(self._condition_filter_changed)
        self.event_list = QListWidget()
        self.event_from = QSpinBox()
        self.event_to = QSpinBox()
        self.baseline_to = QSpinBox()
        self.event_from.setPrefix('Start sample ')
        self.event_to.setPrefix('Stop sample ')
        self.baseline_to.setPrefix('Baseline stop ')
        self.approve_button = QPushButton('Approve selected event')
        self.condition_label = QLabel('Run conditions: load / pressure / IA / speed not loaded')
        for label, widget in (
            ('Transient event run (steady fit uses fixed compatibility runs 8 + 72)', self.run_combo),
            ('Nominal load condition (applies to steady fit and diagnostics)', self.load_filter),
            ('Compatibility segments — check to hold out whole group-qualified segment', self.segment_list),
            ('Transient candidates (Run 7)', self.event_list), ('', self.condition_label), ('', self.event_from),
            ('', self.baseline_to), ('', self.event_to), ('', self.approve_button),
        ):
            if label:
                side_layout.addWidget(QLabel(label))
            side_layout.addWidget(widget)

        self.fit_button = QPushButton('Fit steady')
        self.fit_transient_button = QPushButton('Fit approved events')
        self.fit_longitudinal_button = QPushButton('Fit longitudinal x (disabled until eligible event)')
        self.cancel_button = QPushButton('Cancel')
        self.apply_button = QPushButton('Apply candidate')
        self.discard_button = QPushButton('Discard candidate')
        self.undo_button = QPushButton('Undo')
        self.redo_button = QPushButton('Redo')
        self.reset_button = QPushButton('Reset group')
        self.restore_seed_button = QPushButton('Restore seed')
        self.progress = QProgressBar()
        self.status = QLabel('No model loaded')
        actions = QHBoxLayout()
        for button in (self.fit_button, self.fit_transient_button, self.fit_longitudinal_button,
                       self.cancel_button, self.apply_button, self.discard_button, self.undo_button,
                       self.redo_button, self.reset_button, self.restore_seed_button):
            actions.addWidget(button)
        layout.addLayout(actions)
        layout.addWidget(self.progress)
        layout.addWidget(self.status)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(side)
        splitter.addWidget(self.tabs)
        splitter.setStretchFactor(1, 3)
        layout.addWidget(splitter, 1)
        self.setCentralWidget(root)

        self.plot_timer = QTimer(self)
        self.plot_timer.setSingleShot(True)
        self.plot_timer.setInterval(75)
        self.plot_timer.timeout.connect(self._refresh_plots)
        self.parameter_panel.parameterChanged.connect(self.set_parameter)
        self.parameter_panel.locksChanged.connect(self._locks_changed)
        self.parameter_panel.boundsChanged.connect(self._locks_changed)
        self.open_button.clicked.connect(self._choose_xml)
        self.seed_button.clicked.connect(self._choose_seed)
        self.data_button.clicked.connect(self._choose_data)
        self.session_button.clicked.connect(self._choose_session)
        self.save_button.clicked.connect(self._save_dialog)
        self.fit_button.clicked.connect(self.start_steady_fit)
        self.fit_transient_button.clicked.connect(self.start_transient_fit)
        self.fit_longitudinal_button.clicked.connect(self.start_longitudinal_fit)
        self.cancel_button.clicked.connect(self.cancel_fit)
        self.apply_button.clicked.connect(self._apply_current_candidate)
        self.discard_button.clicked.connect(self.discard_candidate)
        self.undo_button.clicked.connect(self.undo)
        self.redo_button.clicked.connect(self.redo)
        self.reset_button.clicked.connect(self.reset_group)
        self.restore_seed_button.clicked.connect(self.restore_seed)
        self.approve_button.clicked.connect(self.approve_event)
        self.run_combo.currentIndexChanged.connect(self._refresh_run)
        self.event_list.currentRowChanged.connect(self._select_event)
        self.event_from.valueChanged.connect(self._edit_event_bound)
        self.event_to.valueChanged.connect(self._edit_event_bound)
        self.baseline_to.valueChanged.connect(self._edit_event_bound)
        self.segment_list.itemChanged.connect(self._segment_toggled)
        self.fit_longitudinal_button.setEnabled(False)
        self.fit_transient_button.setEnabled(False)
        self.apply_button.setEnabled(False)
        self.discard_button.setEnabled(False)
        self.cancel_button.setEnabled(False)
        geometry = self.settings.value('geometry')
        if geometry is not None:
            self.restoreGeometry(geometry)
        else:
            self.resize(1480, 920)

    def _invalidate_candidate(self):
        self.candidate = None
        self._fit_mode = None
        self._fit_selection = None
        self.apply_button.setEnabled(False)
        self.discard_button.setEnabled(False)

    def _locks_changed(self, _locks):
        self.generation += 1
        self._invalidate_candidate()

    def open_path(self, path):
        try:
            path = Path(path)
            snapshot = load_snapshot(path)
            self.snapshot = self.seed = snapshot
            self.source_files = [str(path)]
            self.generation += 1
            self._history_reset()
            self.parameter_panel.set_snapshot(snapshot)
            self.status.setText(f'{path.name} • repository lab seed • hybrid proxy: 43075 lateral / 43100 longitudinal')
            self._refresh_plots()
            return True
        except Exception as exc:
            self.status.setText(f'Open failed: {exc}')
            return False

    def _choose_xml(self):
        path, _ = QFileDialog.getOpenFileName(self, 'Open tire XML', str(REPOSITORY_ROOT / 'models/tires'), 'Tire XML (*.xml)')
        if path:
            self.open_path(path)

    def _choose_seed(self):
        path, _ = QFileDialog.getOpenFileName(self, 'Import requested MATLAB tire seed', str(REPOSITORY_ROOT / 'private/ttc'), 'MATLAB files (*.mat)')
        if path and self.snapshot:
            try:
                self.snapshot = import_mat_seed(Path(path), self.snapshot)
                self.seed = self.snapshot
                self.source_files.append(path)
                self.generation += 1
                self.parameter_panel.set_snapshot(self.snapshot)
                self._history_reset()
                self.status.setText('Requested MATLAB fit seed imported; no fitData arrays retained.')
                self._refresh_plots()
            except Exception as exc:
                self.status.setText(f'MAT seed import failed: {exc}')

    def set_parameter(self, name, value):
        if self.snapshot is None:
            return
        try:
            if self.snapshot.values.get(name) == float(value):
                return
            self._push_history()
            self.snapshot = with_parameter(self.snapshot, name, value)
            self.dirty = True
            self.history.append(self.snapshot)
            self.history_index = len(self.history) - 1
            self.generation += 1
            self._invalidate_candidate()
            self.status.setText('Edited • transient fit status stale after steady edits')
            self.plot_timer.start()
            self._refresh_diagnostics()
        except Exception as exc:
            self.status.setText(f'Edit refused: {exc}')

    def _history_reset(self):
        self.history = [self.snapshot]
        self.history_index = 0

    def _push_history(self):
        self.history = self.history[:self.history_index + 1]

    def undo(self):
        if self.history_index <= 0:
            return False
        self.history_index -= 1
        self.snapshot = self.history[self.history_index]
        self.generation += 1
        self._invalidate_candidate()
        self.parameter_panel.set_snapshot(self.snapshot)
        self.plot_timer.start()
        return True

    def redo(self):
        if self.history_index + 1 >= len(self.history):
            return False
        self.history_index += 1
        self.snapshot = self.history[self.history_index]
        self.generation += 1
        self._invalidate_candidate()
        self.parameter_panel.set_snapshot(self.snapshot)
        self.plot_timer.start()
        return True

    def reset_group(self):
        if self.snapshot is None:
            return False
        names = [name for name in self.snapshot.values if name.split('_')[0] in {'k', 'mu', 'h', 'v', 'e', 'd', 'longitudinal', 'lateral', 'trail'}]
        return self._restore_values({name: self.seed.values[name] for name in names})

    def restore_seed(self):
        if self.seed is None:
            return False
        return self._restore_values(dict(self.seed.values))

    def _restore_values(self, values):
        self._push_history()
        for name, value in values.items():
            self.snapshot = with_parameter(self.snapshot, name, value)
        self.history.append(self.snapshot)
        self.history_index = len(self.history) - 1
        self.generation += 1
        self.dirty = True
        self._invalidate_candidate()
        self.parameter_panel.set_snapshot(self.snapshot)
        self.plot_timer.start()
        return True

    def load_data_path(self, path):
        path = Path(path)
        run = load_run(path, require_monotonic_time=False)
        self.runs[run.run_id] = run
        self.full_selection = None
        self.active_selection = None
        self.source_files.append(str(path))
        self._run_label(run)
        self.generation += 1
        self._invalidate_candidate()
        self._refresh_run()
        return run

    def _choose_data(self):
        folder = QFileDialog.getExistingDirectory(self, 'Private TTC data folder', str(DEFAULT_DATA))
        if not folder:
            return
        for path in sorted(Path(folder).glob('*.mat')):
            try:
                self.load_data_path(path)
            except Exception as exc:
                self.status.setText(f'{path.name}: {exc}')

    def _source_for_run(self, run):
        for path in reversed(self.source_files):
            if f'run{run.run_id}' in Path(path).name.lower():
                return str(path)
        return str(self.source_files[-1]) if self.source_files else ''

    def _run_label(self, run):
        label = f'Run {run.run_id} — {run.tire_id}' + (' (43100 longitudinal proxy)' if run.run_id == 72 else '')
        if run.run_id not in [self.run_combo.itemData(i) for i in range(self.run_combo.count())]:
            self.run_combo.addItem(label, run.run_id)

    def _select_event(self, index):
        if index < 0 or index >= len(self.events):
            self.approve_button.setEnabled(False)
            return
        event = self.events[index]
        for box, value in ((self.event_from, event.start), (self.event_to, event.stop), (self.baseline_to, event.baseline_stop)):
            box.blockSignals(True)
            box.setValue(value)
            box.blockSignals(False)
        self.approve_button.setEnabled(not event.approved)
        c = event.run.channels
        sl = slice(event.start, event.stop)
        self.condition_label.setText(f"Event conditions: Fz {np.mean(c['fz'][sl]):.0f} N; pressure {np.mean(c['pressure'][sl])/1000:.2f} kPa; IA {np.rad2deg(np.mean(c['ia'][sl])):.2f}°; speed {np.mean(c['speed'][sl]):.2f} m/s")
        self.status.setText('Approved event' if event.approved else ('Candidate exclusions: ' + ('; '.join(event.exclusions) if event.exclusions else 'review required')))
        self._refresh_transient_event()

    def _edit_event_bound(self, *_):
        index = self.event_list.currentRow()
        if index < 0 or index >= len(self.events):
            return
        event = self.events[index]
        try:
            updated = replace(event, start=self.event_from.value(), stop=self.event_to.value(),
                              baseline_stop=self.baseline_to.value(), approved=False, exclusions=())
            validate_event(updated, require_approval=False)
            self.events[index] = updated
            self.generation += 1
            self._invalidate_candidate()
            self.event_list.item(index).setText(
                f'Edited candidate: {updated.run.channels["time"][updated.start]:.2f}–'
                f'{updated.run.channels["time"][updated.stop - 1]:.2f} s; approval required')
            self.event_list.item(index).setData(Qt.ItemDataRole.UserRole, (updated.start, updated.stop, updated.baseline_stop))
            self.approve_button.setEnabled(True)
            self.status.setText('Event boundary edited; explicit approval required.')
            self._refresh_transient_event()
        except Exception as exc:
            self.status.setText(f'Event boundary invalid: {exc}')

    def _refresh_transient_event(self):
        index = self.event_list.currentRow()
        if index < 0 or index >= len(self.events):
            return
        event = self.events[index]
        channels = event.run.channels
        start, stop = max(0, event.start - 100), min(len(channels['time']), event.stop + 150)
        times = channels['time'][start:stop]
        for axis in self.transient_plot.axes:
            axis.clear()
        self.transient_plot.axes[0].plot(times, channels['alpha'][start:stop] * 180 / 3.141592653589793, label='SA command (deg)')
        self.transient_plot.axes[1].plot(times, channels['fy'][start:stop], label='Measured Fy (N)')
        if 'mz' in channels:
            self.transient_plot.axes[2].plot(times, channels['mz'][start:stop], label='Measured Mz (N·m)')
        self.transient_plot.axes[3].plot(times, channels['fx'][start:stop], label='Measured Fx (N)')
        self.transient_plot.axes[4].set_title('Replay residuals (current replay pending)')
        self.transient_plot.axes[5].set_title('Relaxation lengths (m)')
        for axis in self.transient_plot.axes:
            axis.grid(True, alpha=.25)
            axis.legend(fontsize='small')
            axis.set_xlabel('Elapsed time (s)')
        key = (self.snapshot.steady_fingerprint, tuple(self.snapshot.values[n] for n in ('longitudinal_length_scale','lateral_length_scale','trail_pole_ratio','trail_lead_ratio')), event.run.run_id, event.start, event.stop, event.baseline_stop, self.generation)
        if key in self.replay_cache:
            self._plot_replay(key, self.replay_cache[key])
        elif key in self.replay_failures:
            self.transient_plot.axes[1].set_title('Replay unavailable; see status')
        elif self.replay_thread is not None:
            self.transient_plot.axes[1].set_title('Current/seed replay queued after active replay')
        elif self.snapshot is not None:
            self.replay_thread = QThread(self)
            self.replay_worker = ReplayWorker(self.snapshot, self.seed, event, self.generation)
            self.replay_worker.moveToThread(self.replay_thread)
            self.replay_thread.started.connect(self.replay_worker.run)
            self.replay_worker.finished.connect(lambda result, key=key: self._replay_done(key, result))
            self.replay_worker.failed.connect(lambda text, key=key: self._replay_failed(key, text))
            direct = Qt.ConnectionType.DirectConnection
            self.replay_worker.finished.connect(self.replay_thread.quit, direct)
            self.replay_worker.failed.connect(self.replay_thread.quit, direct)
            self.replay_thread.finished.connect(self._replay_stopped)
            self.replay_thread.finished.connect(self.replay_thread.deleteLater)
            self.replay_thread.start()
        self.transient_plot.figure.tight_layout()
        self.transient_plot.canvas.draw_idle()

    def _replay_done(self, key, result):
        generation, current, seed = result
        self.replay_cache[key] = (current, seed)
        if generation == self.generation and key[-1] == self.generation:
            self._plot_replay(key, (current, seed))

    def _replay_failed(self, key, text):
        # Cache against the immutable request so failed old replays cannot poison
        # a newer selection made while the worker was still running.
        self.replay_failures.add(key)
        self.status.setText('Transient replay unavailable: ' + text)

    def _replay_stopped(self):
        if self.replay_worker:
            self.replay_worker.deleteLater()
        self.replay_worker = self.replay_thread = None
        index = self.event_list.currentRow()
        if 0 <= index < len(self.events):
            event = self.events[index]
            key = (self.snapshot.steady_fingerprint, tuple(self.snapshot.values[n] for n in ('longitudinal_length_scale','lateral_length_scale','trail_pole_ratio','trail_lead_ratio')), event.run.run_id, event.start, event.stop, event.baseline_stop, self.generation)
            if key in self.replay_cache:
                self._plot_replay(key, self.replay_cache[key])
            else:
                self._refresh_transient_event()

    def _plot_replay(self, key, predictions):
        current, seed = predictions
        index = self.event_list.currentRow()
        if index < 0 or index >= len(self.events):
            return
        event = self.events[index]
        if (event.run.run_id, event.start, event.stop, event.baseline_stop) != (key[2], key[3], key[4], key[5]):
            return
        t = event.run.channels['time'][event.start:event.stop]
        for index, (channel, units, column) in enumerate((('fx','N',0),('fy','N',1),('mz','N·m',2))):
            if channel not in event.run.channels:
                continue
            measured = event.run.channels[channel][event.start:event.stop]
            axis = self.transient_plot.axes[index + 1]
            axis.plot(t, current[:, column], label='Current replay', color='#1665a8')
            if seed is not None:
                axis.plot(t, seed[:, column], label='Seed replay', color='#bd5a20', linestyle='--')
            residual_axis = self.transient_plot.axes[4]
            residual_axis.plot(t, measured-current[:, column], label=f'{channel.upper()} residual')
            axis.set_ylabel(f'{channel.upper()} ({units})')
        # Plot load-dependent relaxation lengths (m) in the fourth axis.
        from simplified_vehicle_models.tires.unitire_transient import relaxation_lengths
        fz = event.run.channels['fz'][event.start:event.stop]
        sx, sy = zip(*(relaxation_lengths(self.snapshot.steady, self.snapshot.transient, float(load)) for load in fz))
        axis = self.transient_plot.axes[5]
        axis.plot(t, sx, label='sigma_x (m)')
        axis.plot(t, sy, label='sigma_y (m)')
        axis.set_ylabel('Relaxation length (m)')
        for axis in self.transient_plot.axes:
            axis.legend(fontsize='small')
        self.transient_plot.canvas.draw_idle()

    def approve_event(self):
        index = self.event_list.currentRow()
        if index < 0:
            return False
        try:
            event = replace(self.events[index], start=self.event_from.value(), stop=self.event_to.value(),
                            baseline_stop=self.baseline_to.value(), approved=True, exclusions=())
            validate_event(event)
            self.events[index] = event
            self.generation += 1
            self._invalidate_candidate()
            self.event_list.item(index).setText(f'APPROVED {event.axis}: samples {event.start}:{event.stop}; baseline stop {event.baseline_stop}')
            self.event_list.item(index).setData(Qt.ItemDataRole.UserRole, (event.start, event.stop, event.baseline_stop))
            self.fit_transient_button.setEnabled(True)
            self.approve_button.setEnabled(False)
            self.status.setText('Event explicitly approved after boundary/condition validation.')
            self._refresh_transient_event()
            return True
        except Exception as exc:
            self.status.setText(f'Approval refused: {exc}')
            return False

    def _condition_filter_changed(self, *_):
        if self.full_selection is None:
            return
        self.generation += 1
        self._invalidate_candidate()
        self._apply_selection_filters()
        self._refresh_plots()

    def _segment_toggled(self, item):
        segment_id = item.data(Qt.ItemDataRole.UserRole)
        if segment_id is None:
            return
        if item.checkState() == Qt.CheckState.Checked:
            self.holdout_ids.add(segment_id)
        else:
            self.holdout_ids.discard(segment_id)
        self.generation += 1
        self._invalidate_candidate()
        label = item.text().split(' — ')[0]
        item.setText(label + (' — Holdout' if segment_id in self.holdout_ids else ' — Train'))
        if self.full_selection is not None:
            self._apply_selection_filters()
            self._refresh_plots()

    def _refresh_run(self, *_):
        run_id = self.run_combo.currentData()
        run = self.runs.get(run_id)
        self.event_list.clear()
        self.events = []
        if not run:
            self._refresh_plots()
            return
        if run_id == 7:
            try:
                self.events = list(candidate_events(run))
            except Exception as exc:
                self.status.setText(str(exc))
            source = self._source_for_run(run)
            rows = [row for row in getattr(self, '_restored_events', []) if int(row.get('run_id', -1)) == 7 and (not row.get('source_file') or row.get('source_file') == source)]
            for row in rows:
                start, stop = int(row['start']), int(row['stop'])
                existing = next((i for i, event in enumerate(self.events) if event.start == start and event.stop == stop), None)
                if existing is None and 0 <= start < int(row.get('baseline_stop', start + 20)) < stop <= len(run.raw_indices):
                    from ..events import TransientEvent
                    event = TransientEvent(run, start, stop, int(row['baseline_stop']), bool(row.get('approved', False)), row.get('axis', 'lateral'), ())
                    self.events.append(event)
            for index, event in enumerate(self.events):
                reasons = '; '.join(event.exclusions) or 'review required'
                self.event_list.addItem(f'Candidate {index + 1}: {run.channels["time"][event.start]:.2f}–{run.channels["time"][event.stop - 1]:.2f} s; baseline {event.baseline_stop - event.start} samples; {reasons}')
                self.event_list.item(index).setData(Qt.ItemDataRole.UserRole, (event.start, event.stop, event.baseline_stop))
            self.event_from.setRange(0, max(0, len(run.raw_indices) - 1))
            self.event_to.setRange(1, len(run.raw_indices))
            self.baseline_to.setRange(1, len(run.raw_indices))
            if self.events:
                event = self.events[0]
                self.event_from.setValue(event.start)
                self.event_to.setValue(event.stop)
                self.baseline_to.setValue(event.baseline_stop)
            self.transient_plot.message('Run 7 — elapsed time only (0.15–798.652 s); acquisition date unavailable. Review candidates; no calendar timestamp inferred.')
            if self.events:
                self.event_list.setCurrentRow(0)
        if self.full_selection is None and 8 in self.runs and 72 in self.runs:
            self.full_selection = compatibility_selection(self.runs[8], self.runs[72])
            self._populate_load_filter()
        if self.full_selection is not None:
            self._populate_compatibility_segments()
            self._apply_selection_filters()
        self.fit_transient_button.setEnabled(any(event.approved for event in self.events))
        self.fit_longitudinal_button.setEnabled(any(event.approved and event.axis == 'longitudinal' for event in self.events))
        self._refresh_plots()

    @staticmethod
    def _only_segments(samples, included):
        mask = np.asarray([(samples.run_id, int(segment)) in included for segment in samples.segment_ids], dtype=bool)
        from ..contracts import SampleSet
        return SampleSet(samples.alpha[mask], samples.kappa[mask], samples.fz[mask], samples.target[mask],
                         samples.segment_ids[mask], samples.run_id, samples.tire_id, samples.raw_indices[mask])

    @staticmethod
    def _without_segments(samples, excluded):
        mask = [not (samples.run_id, int(segment)) in excluded for segment in samples.segment_ids]
        from ..contracts import SampleSet
        return SampleSet(samples.alpha[mask], samples.kappa[mask], samples.fz[mask], samples.target[mask],
                         samples.segment_ids[mask], samples.run_id, samples.tire_id, samples.raw_indices[mask])

    def _populate_compatibility_segments(self):
        if self.full_selection is None:
            return
        self.segment_list.blockSignals(True)
        self.segment_list.clear()
        groups = (('run8', 'Run 8 Fy / Mz', self.full_selection.lateral),
                  ('run72_sa0_fx', 'SA0 Fx', self.full_selection.longitudinal),
                  ('run72_combined', 'Combined Fx / Fy', self.full_selection.combined_fx))
        shown = set()
        for prefix, label, samples in groups:
            for segment in sorted(set(map(int, samples.segment_ids))):
                identity = (prefix, segment)
                if identity in shown:
                    continue
                shown.add(identity)
                mask = samples.segment_ids == segment
                loads = np.unique(samples.fz[mask])
                load_label = f'{float(loads[0]):g} N' if len(loads) == 1 else 'mixed load'
                run_id = 8 if prefix == 'run8' else 72
                run = self.runs.get(run_id)
                conditions = []
                if run is not None:
                    raw = samples.raw_indices[mask].astype(int)
                    for channel, label_text, factor, unit in (('ia', 'IA', 180/np.pi, 'deg'), ('pressure', 'P', .001, 'kPa'), ('speed', 'V', 1.0, 'm/s')):
                        if channel in run.channels and len(raw) and raw.max() < len(run.channels[channel]):
                            vals = np.asarray(run.channels[channel])[raw] * factor
                            conditions.append(f'{label_text} {np.mean(vals):.2f}±{np.std(vals):.2f} {unit}')
                condition_text = '; '.join(conditions) if conditions else 'conditions unavailable'
                item = QListWidgetItem(f'{label} segment {segment + 1} • {load_label} • {condition_text} — Train')
                item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
                item.setCheckState(Qt.CheckState.Checked if identity in self.holdout_ids else Qt.CheckState.Unchecked)
                item.setData(Qt.ItemDataRole.UserRole, identity)
                self.segment_list.addItem(item)
        self.segment_list.blockSignals(False)

    @staticmethod
    def _filter_group(samples, prefix, excluded, load):
        mask = np.ones(len(samples.target), dtype=bool)
        if load is not None:
            mask &= np.isclose(samples.fz, float(load), rtol=0, atol=1e-9)
        holdout = np.asarray([(prefix, int(segment)) in excluded for segment in samples.segment_ids], dtype=bool)
        return (MainWindow._sample_mask(samples, mask & ~holdout),
                MainWindow._sample_mask(samples, mask & holdout))

    @staticmethod
    def _sample_mask(samples, mask):
        from ..contracts import SampleSet
        return SampleSet(samples.alpha[mask], samples.kappa[mask], samples.fz[mask], samples.target[mask],
                         samples.segment_ids[mask], samples.run_id, samples.tire_id, samples.raw_indices[mask])

    def _apply_selection_filters(self):
        if self.full_selection is None:
            return
        load = self.load_filter.currentData()
        pairs = {}
        for name, prefix, samples in (
            ('lateral', 'run8', self.full_selection.lateral),
            ('mz', 'run8', self.full_selection.mz),
            ('longitudinal', 'run72_sa0_fx', self.full_selection.longitudinal),
            ('combined_fx', 'run72_combined', self.full_selection.combined_fx),
            ('combined_fy', 'run72_combined', self.full_selection.combined_fy)):
            pairs[name] = self._filter_group(samples, prefix, self.holdout_ids, load)
        train = {name: pair[0] for name, pair in pairs.items()}
        held = {name: pair[1] for name, pair in pairs.items()}
        self.selection = replace(self.full_selection, **train)
        self.holdout_selection = replace(self.full_selection, **held)
        active = {name: self._sample_mask(samples, np.ones(len(samples.target), dtype=bool) if load is None else np.isclose(samples.fz, float(load), rtol=0, atol=1e-9))
                  for name, samples in (('lateral',self.full_selection.lateral),('mz',self.full_selection.mz),('longitudinal',self.full_selection.longitudinal),('combined_fx',self.full_selection.combined_fx),('combined_fy',self.full_selection.combined_fy))}
        self.active_selection = replace(self.full_selection, **active)
        # Held-out segments remain available for evaluation even if their samples
        # are removed from training; the condition selector scopes both.
        self.condition_label.setText('Steady conditions: ' + (f'{load:g} N nominal load' if load is not None else 'all nominal loads') + '; fixed run 8/72 source-compatible groups')

    def _selection_for_fit(self):
        if self.full_selection is None:
            full = compatibility_selection(self.runs[8], self.runs[72])
            self.full_selection = full
        self._populate_load_filter()
        self._populate_compatibility_segments()
        self._apply_selection_filters()
        return self.selection

    def _populate_load_filter(self):
        if self.full_selection is None:
            return
        loads = sorted(set(float(x) for samples in (self.full_selection.lateral, self.full_selection.longitudinal, self.full_selection.combined_fx) for x in np.unique(samples.fz)))
        selected = self.load_filter.currentData()
        self.load_filter.blockSignals(True)
        self.load_filter.clear()
        self.load_filter.addItem('All compatibility loads', None)
        for load in loads:
            self.load_filter.addItem(f'{load:g} N', load)
        index = self.load_filter.findData(selected)
        if index < 0 and getattr(self, '_restored_load_n', None) is not None:
            index = self.load_filter.findData(self._restored_load_n)
        self.load_filter.setCurrentIndex(max(0, index))
        self.load_filter.blockSignals(False)
        self._restored_load_n = None

    def start_steady_fit(self):
        try:
            self.selection = self._selection_for_fit()
            self._start_worker('steady', self.selection, ())
        except Exception as exc:
            self.status.setText(f'Steady fit requires compatible runs 8 and 72: {exc}')

    def start_longitudinal_fit(self):
        approved = tuple(event for event in self.events if event.approved and event.axis == 'longitudinal')
        if not approved:
            self.status.setText('No independently excited longitudinal time-series event approved. Longitudinal scale remains unidentified / provisional.')
            return
        self._start_worker('transient', None, approved)

    def start_transient_fit(self):
        approved = tuple(event for event in self.events if event.approved and event.axis == 'lateral')
        if not approved:
            self.status.setText('No approved eligible transient events. Longitudinal scale remains unidentified / provisional.')
            return
        self._start_worker('transient', None, approved)

    def _start_worker(self, mode, selection, events):
        if self.worker is not None or self.snapshot is None:
            return
        self._invalidate_candidate()
        self.fit_button.setEnabled(False)
        self.fit_transient_button.setEnabled(False)
        self.fit_longitudinal_button.setEnabled(False)
        self.cancel_button.setEnabled(True)
        self._fit_generation = self.generation
        self._fit_mode = mode
        self._fit_selection = selection
        self.thread = QThread(self)
        self.worker = FitWorker(self.snapshot, mode, selection, self.parameter_panel.locked(), events, self.parameter_panel.bounds(), runner=getattr(self, 'worker_runner', None))
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.progress.connect(self._progress)
        self.worker.finished.connect(self._fit_done)
        self.worker.failed.connect(self._fit_failed)
        self.worker.cancelled.connect(self._fit_cancelled)
        direct = Qt.ConnectionType.DirectConnection
        self.worker.finished.connect(self.thread.quit, direct)
        self.worker.failed.connect(self.thread.quit, direct)
        self.worker.cancelled.connect(self.thread.quit, direct)
        self.thread.finished.connect(self._worker_stopped)
        self.thread.finished.connect(self.thread.deleteLater)
        self.thread.start()
        self.status.setText(f'{mode.title()} fit running on immutable snapshot…')

    def _progress(self, progress):
        self.status.setText(f"Fitting {progress.get('stage', '')} — {progress}")

    def _fit_done(self, result):
        if self._fit_generation != self.generation or result.source_revision != self.snapshot.revision:
            self.status.setText('Fit completed for an older snapshot; candidate discarded as stale.')
            self._invalidate_candidate()
            return
        self.candidate = result
        self.apply_button.setEnabled(True)
        self.discard_button.setEnabled(True)
        self.status.setText('Candidate ready — review diagnostics, then explicitly Apply or Discard.')
        self._refresh_diagnostics(result)
        if getattr(self, '_fit_mode', None) == 'steady':
            self._refresh_plots()
            self._refresh_diagnostics(result)

    def _fit_failed(self, text):
        self.status.setText('Fit failed: ' + text)

    def _fit_cancelled(self):
        self.status.setText('Fit cancelled; live snapshot unchanged.')

    def _worker_stopped(self):
        if self.worker:
            self.worker.deleteLater()
        self.worker = self.thread = None
        self.fit_button.setEnabled(True)
        self.fit_transient_button.setEnabled(any(event.approved for event in self.events))
        self.fit_longitudinal_button.setEnabled(any(event.approved and event.axis == 'longitudinal' for event in self.events))
        self.cancel_button.setEnabled(False)

    def cancel_fit(self):
        if self.worker:
            self.worker.cancel()
            self.status.setText('Cancellation requested; waiting for safe solver boundary…')

    def _apply_current_candidate(self):
        if self.candidate:
            self.apply_candidate(self.candidate)

    def apply_candidate(self, candidate: FitResult):
        if candidate is None or self.snapshot is None or candidate.source_revision != self.snapshot.revision or getattr(self, '_fit_generation', None) != self.generation:
            return False
        self._push_history()
        self.snapshot = candidate.candidate
        self.history.append(self.snapshot)
        self.history_index = len(self.history) - 1
        self.generation += 1
        self.parameter_panel.set_snapshot(self.snapshot)
        self._invalidate_candidate()
        self.status.setText('Candidate explicitly applied.')
        self._refresh_plots()
        return True

    def discard_candidate(self):
        self._invalidate_candidate()
        self._refresh_plots()
        self._refresh_diagnostics()
        self.status.setText('Candidate discarded; live model unchanged.')

    def _refresh_plots(self):
        if self.snapshot is None:
            return
        try:
            from ..model import predict_steady
            no_measurements = 8 not in self.runs or 72 not in self.runs
            if no_measurements:
                from ..contracts import SampleSet, SteadySelection
                alpha = np.linspace(-0.3, 0.3, 121)
                kappa = np.linspace(-0.3, 0.3, 121)
                load = min(600.0, max(100.0, self.snapshot.load_domain[1] * .4))
                def synthetic(a, k):
                    return SampleSet(a, k, np.full(len(a), load), np.zeros(len(a)), np.full(len(a), -1, dtype=int), raw_indices=np.arange(len(a), dtype=int))
                lat, lon = synthetic(alpha, np.zeros_like(alpha)), synthetic(np.zeros_like(kappa), kappa)
                mixed, mz = synthetic(alpha, kappa), synthetic(alpha, np.zeros_like(alpha))
                data = SteadySelection(lat, lon, mixed, mixed, mz, 40.193 / 3.6)
            elif self.full_selection is None:
                self.full_selection = compatibility_selection(self.runs[8], self.runs[72])
                self._populate_load_filter()
                self._populate_compatibility_segments()
                self._apply_selection_filters()
            candidate_model = self.candidate.candidate if self.candidate is not None and getattr(self, '_fit_mode', None) == 'steady' else None
            if not no_measurements:
                data = (self._fit_selection if candidate_model is not None else None) or self.active_selection or compatibility_selection(self.runs[8], self.runs[72])
            groups = [
                ('Fy vs alpha', data.lateral, 'lateral_force', data.lateral.alpha, 'alpha (rad)', 'Fy (N)'),
                ('Fx vs kappa', data.longitudinal, 'longitudinal_force', data.longitudinal.kappa, 'kappa', 'Fx (N)'),
                ('Combined slip Fx/Fz vs Fy/Fz', data.combined_fx, 'longitudinal_force', None, 'Fy/Fz', 'Fx/Fz'),
                ('Mz vs alpha', data.mz, 'aligning_moment', data.mz.alpha, 'alpha (rad)', 'Mz (N·m)'),
            ]
            for axis, (title, samples, component, x, xlabel, ylabel) in zip(self.steady_plot.axes, groups):
                axis.clear()
                denominator = samples.fz if '/Fz' in ylabel else 1
                measured = samples.target / denominator
                prediction = predict_steady(self.snapshot, samples, data.belt_speed)
                current = getattr(prediction, component) / denominator
                if title.startswith('Combined slip'):
                    if no_measurements:
                        x = samples.alpha
                        order = np.argsort(x)
                    else:
                        fy = data.combined_fy
                        fy_measured = {int(ix): float(val / fz) for ix, val, fz in zip(fy.raw_indices, fy.target, fy.fz)}
                        fx_measured = {int(ix): float(val / fz) for ix, val, fz in zip(samples.raw_indices, samples.target, samples.fz)}
                        common = sorted(set(fy_measured) & set(fx_measured))
                        x = np.asarray([fy_measured[i] for i in common])
                        measured = np.asarray([fx_measured[i] for i in common])
                        fx_prediction = predict_steady(self.snapshot, samples, data.belt_speed)
                        current_by_raw = {int(ix): float(val / fz) for ix, val, fz in zip(samples.raw_indices, fx_prediction.longitudinal_force, samples.fz)}
                        current = np.asarray([current_by_raw[i] for i in common])
                        order = np.argsort(x)
                    if not no_measurements:
                        axis.scatter(x, measured, s=5, c='#555555', label='Measured', alpha=.65)
                    axis.plot(x[order], current[order], label='Current fit', color='#1665a8')
                    if candidate_model is not None:
                        candidate_pred = predict_steady(candidate_model, samples, data.belt_speed)
                        if no_measurements:
                            axis.plot(x[order], candidate_pred.longitudinal_force[order] / denominator, label='Pending candidate', color='#238636', linestyle=':')
                        else:
                            candidate_by_raw = {int(ix): float(val / fz) for ix, val, fz in zip(samples.raw_indices, candidate_pred.longitudinal_force, samples.fz)}
                            axis.plot(x[order], [candidate_by_raw[i] for i in np.asarray(common)[order]], label='Pending candidate', color='#238636', linestyle=':')
                    if self.seed:
                        if no_measurements:
                            seed = predict_steady(self.seed, samples, data.belt_speed).longitudinal_force / denominator
                            axis.plot(x[order], seed[order], label='Seed', color='#bd5a20', linestyle='--')
                        else:
                            seed_pred = predict_steady(self.seed, samples, data.belt_speed)
                            seed_by_raw = {int(ix): float(val / fz) for ix, val, fz in zip(samples.raw_indices, seed_pred.longitudinal_force, samples.fz)}
                            axis.plot(x[order], [seed_by_raw[i] for i in np.asarray(common)[order]], label='Seed', color='#bd5a20', linestyle='--')
                    if not no_measurements:
                        axis.plot(x[order], (measured-current)[order], label='Residual', color='#8a3b3b', alpha=.7)
                else:
                    order = x.argsort()
                    if not no_measurements:
                        axis.scatter(x, measured, s=5, c='#555555', label='Measured', alpha=.65)
                    axis.plot(x[order], current[order], label='Current fit', color='#1665a8')
                    if candidate_model is not None:
                        proposed = getattr(predict_steady(candidate_model, samples, data.belt_speed), component) / denominator
                        axis.plot(x[order], proposed[order], label='Pending candidate', color='#238636', linestyle=':')
                    if self.seed and not no_measurements:
                        seed = getattr(predict_steady(self.seed, samples, data.belt_speed), component) / denominator
                        axis.plot(x[order], seed[order], label='Seed', color='#bd5a20', linestyle='--')
                    if not no_measurements:
                        axis.plot(x[order], (measured-current)[order], label='Residual', color='#8a3b3b', alpha=.7)
                if no_measurements:
                    axis.text(.02, .98, 'No measurements — model-only sweep', transform=axis.transAxes, va='top', fontsize='small')
                axis.set_title(title)
                axis.set_xlabel(xlabel)
                axis.set_ylabel(ylabel)
                axis.grid(True, alpha=.25)
                if axis.lines or axis.collections:
                    axis.legend(fontsize='small')
            self.steady_plot.figure.tight_layout()
            self.steady_plot.canvas.draw_idle()
        except Exception as exc:
            self.steady_plot.message(f'Plots unavailable: {exc}')
        self._refresh_diagnostics()

    def _refresh_diagnostics(self, result=None):
        if self.snapshot is None:
            return
        lines = [
            'Hybrid tire proxy: 43075 R20 lateral / 43100 R20 longitudinal; run 72 combined-slip Fy is 43100.',
            'Run 7: ET is elapsed time; acquisition calendar date unavailable.',
            'No run-7 longitudinal excitation approved: longitudinal_length_scale unidentified / provisional.',
            'Source: ' + ', '.join(self.source_files),
            f'Revision / generation: {self.snapshot.revision} / {self.generation}',
        ]
        lines.extend(f'{name}: {status}' for name, status in self.snapshot.parameter_status.items())
        if self.snapshot.provenance.get('transient_fit_status') in {'stale', 'provisional'}:
            lines.append('Transient replay shown is based on current coefficients; prior transient fit status is stale/provisional after steady-model edits.')
        lines.extend(f'{name} bounds: {bounds[0]:g} … {bounds[1]:g}' for name, bounds in self.parameter_panel.bounds().items())
        if result:
            lines.extend((f'converged: {result.converged}', f'metrics: {dict(result.metrics)}', *result.warnings))
            lines.append('Pending candidate parameters (live model unchanged until Apply):')
            lines.extend(f'  {name}: {value:.9g} — {result.status_by_parameter.get(name, result.candidate.parameter_status.get(name, "unchanged"))}' for name, value in result.candidate.values.items() if result.candidate.values[name] != self.snapshot.values.get(name))
            lines.extend(f'  {name}: {result.candidate.values[name]:.9g} — {result.status_by_parameter.get(name, "unchanged")}' for name in result.status_by_parameter if result.candidate.values.get(name) == self.snapshot.values.get(name))
            holdout = getattr(self, 'holdout_selection', None)
            if holdout is not None and any(len(getattr(holdout, name).target) for name in ('lateral','longitudinal','combined_fx','combined_fy','mz')):
                from ..model import predict_steady
                hold = replace(self.holdout_selection, **{name: self._sample_mask(getattr(self.holdout_selection, name), np.ones(len(getattr(self.holdout_selection, name).target), dtype=bool) if self.load_filter.currentData() is None else np.isclose(getattr(self.holdout_selection, name).fz, float(self.load_filter.currentData()), rtol=0, atol=1e-9)) for name in ('lateral','longitudinal','combined_fx','combined_fy','mz')})
                lines.append('Whole-segment holdout dimensional RMSE (not used in training):')
                for key, samples, component in [('Fy',hold.lateral,'lateral_force'),('Fx',hold.longitudinal,'longitudinal_force'),('combined Fx',hold.combined_fx,'longitudinal_force'),('combined Fy',hold.combined_fy,'lateral_force'),('Mz',hold.mz,'aligning_moment')]:
                    if len(samples.target):
                        pred = getattr(predict_steady(result.candidate, samples, hold.belt_speed), component)
                        rmse = float(np.sqrt(np.mean((pred-samples.target)**2)))
                        lines.append(f'  {key}: {rmse:.6g}; n={len(samples.target)}')
        self.diagnostics.setPlainText('\n'.join(lines))

    def save_session(self, output_dir, stem, overwrite=False):
        if self.snapshot is None:
            raise ValueError('no model loaded')
        metadata = {
            'locks': {name: name in self.parameter_panel.locked() for name in self.snapshot.values},
            'bounds': {name: list(pair) for name, pair in self.parameter_panel.bounds().items()},
            'source_files': self.source_files,
            'selections': {'run_id': self.run_combo.currentData(), 'load_n': self.load_filter.currentData(), 'holdout_ids': [f'{group}:{segment}' for group, segment in sorted(self.holdout_ids)]},
            'events': [{'id': f'{e.run.run_id}:{i}', 'start': int(e.start), 'stop': int(e.stop), 'run_id': int(e.run.run_id), 'axis': str(e.axis), 'approved': bool(e.approved), 'reason': '; '.join(e.exclusions), 'source_file': self._source_for_run(e.run), 'segment_id': str(i), 'baseline_stop': int(e.baseline_stop)} for i, e in enumerate(self.events)],
            'diagnostics': {'generation': self.generation},
        }
        return save_session(self.snapshot, Path(output_dir), stem, include_transient=True, overwrite=overwrite, metadata=metadata)

    def _choose_session(self):
        path = QFileDialog.getExistingDirectory(self, 'Restore private tire fit bundle', str(DEFAULT_OUTPUT))
        if not path:
            return
        try:
            self.load_session_path(path)
        except Exception as exc:
            QMessageBox.warning(self, 'Session restore refused', str(exc))

    def _save_dialog(self):
        if self.snapshot is None:
            return
        output_dir = QFileDialog.getExistingDirectory(self, 'Choose private output directory', str(DEFAULT_OUTPUT))
        if not output_dir:
            return
        stem, accepted = QInputDialog.getText(self, 'Save fit bundle', 'Private bundle name:')
        if not accepted:
            return
        output_path = Path(output_dir).expanduser().resolve()
        private_root = DEFAULT_OUTPUT.expanduser().resolve()
        try:
            is_private = output_path == private_root or output_path.is_relative_to(private_root)
        except (OSError, ValueError):
            is_private = False
        if not is_private:
            disclosure = ('Privacy warning: the fit bundle will be written outside the ignored private output root. '
                          'It contains source-file references, parameter values, event/selection metadata, and derived XML. '
                          'Confirm this destination is private and not shared before continuing.\n\nDestination: ' + str(output_path))
            if QMessageBox.question(self, 'Confirm private alternate destination', disclosure,
                                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                    QMessageBox.StandardButton.No) != QMessageBox.StandardButton.Yes:
                return
        destination = output_path / stem.strip()
        overwrite = False
        if destination.exists():
            if QMessageBox.question(self, 'Overwrite private bundle?', f'Overwrite recognized bundle {destination}?') != QMessageBox.StandardButton.Yes:
                return
            overwrite = True
        try:
            files = self.save_session(output_dir, stem.strip(), overwrite)
            self.dirty = False
            self.status.setText('Saved private validated bundle: ' + ', '.join(path.name for path in files))
        except Exception as exc:
            self.status.setText(f'Save refused: {exc}')

    def load_session_path(self, path):
        snapshot = load_session(Path(path))
        metadata = snapshot.session_metadata
        self.runs = {}
        self.run_combo.clear()
        self.snapshot = snapshot
        self.seed = snapshot
        self.generation += 1
        self.source_files = list(metadata.get('source_files', []))
        self._history_reset()
        self.parameter_panel.set_snapshot(snapshot)
        selections = metadata.get('selections', {})
        run_id = selections.get('run_id')
        run_index = self.run_combo.findData(run_id)
        if run_index >= 0:
            self.run_combo.setCurrentIndex(run_index)
        self._restored_load_n = selections.get('load_n')
        saved_holdouts = selections.get('holdout_ids', [])
        self.holdout_ids = set()
        legacy_holdout_migrated = False
        for item in saved_holdouts:
            if isinstance(item, str) and ':' in item:
                group, segment = item.rsplit(':', 1)
                if group in {'run8', 'run72_sa0_fx', 'run72_combined', 'run72_combined_fx', 'run72_combined_fy'}:
                    self.holdout_ids.add((group, int(segment)))
            elif isinstance(item, (list, tuple)) and len(item) == 2:
                # Legacy v1 bundles used (run_id, segment_id); preserve old
                # behavior by holding out every group sharing that segment index.
                if isinstance(item[0], str) and item[0] in {'run8', 'run72_sa0_fx', 'run72_combined', 'run72_combined_fx', 'run72_combined_fy'}:
                    self.holdout_ids.add((item[0], int(item[1])))
                elif item[0] == 8:
                    self.holdout_ids.add(('run8', int(item[1])))
                    legacy_holdout_migrated = True
                elif item[0] == 72:
                    self.holdout_ids.update({('run72_sa0_fx', int(item[1])), ('run72_combined', int(item[1]))})
                    legacy_holdout_migrated = True
        restored_load = selections.get('load_n')
        self._restored_load_n = float(restored_load) if isinstance(restored_load, (int, float)) else None
        self._restored_events = list(metadata.get('events', []))
        self.runs = {}
        self.run_combo.clear()
        source_paths = [Path(p) for p in self.source_files if Path(p).is_file()]
        for source in source_paths:
            try:
                self.load_data_path(source)
            except Exception:
                continue
        run_index = self.run_combo.findData(run_id)
        if run_index >= 0:
            self.run_combo.setCurrentIndex(run_index)
        saved_events = {(int(row.get('run_id', -1)), int(row['start']), int(row['stop'])): row for row in metadata.get('events', [])}
        for run_id, run in self.runs.items():
            for row in (row for row in metadata.get('events', []) if int(row.get('run_id', -1)) == run_id):
                start, stop = int(row['start']), int(row['stop'])
                if not any(e.run.run_id == run_id and e.start == start and e.stop == stop for e in self.events) and 0 <= start < int(row.get('baseline_stop', start + 20)) < stop <= len(run.raw_indices):
                    from ..events import TransientEvent
                    self.events.append(TransientEvent(run, start, stop, int(row['baseline_stop']), bool(row.get('approved', False)), row.get('axis', 'lateral'), ()))
        for i, event in enumerate(self.events):
            row = saved_events.get((int(event.run.run_id), int(event.start), int(event.stop)))
            if row:
                self.events[i] = replace(event, baseline_stop=int(row.get('baseline_stop', event.baseline_stop)), approved=bool(row.get('approved', False)), axis=row.get('axis', event.axis), exclusions=())
        if self.run_combo.currentData() == 7:
            self._refresh_run()
        for index in range(self.segment_list.count()):
            item = self.segment_list.item(index)
            if item.data(Qt.ItemDataRole.UserRole) in self.holdout_ids:
                item.setCheckState(Qt.CheckState.Checked)
        self.dirty = False
        message = 'Session restored; available source references and GUI options restored, samples not embedded.'
        if legacy_holdout_migrated:
            message += ' Legacy run/segment holdouts were migrated to all matching qualified groups; save the session to persist the migration.'
        self.status.setText(message)
        return snapshot

    def closeEvent(self, event):
        if self.dirty and self.isVisible():
            answer = QMessageBox.question(self, 'Unsaved changes', 'Discard unsaved tire model edits and close?', QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No, QMessageBox.StandardButton.No)
            if answer != QMessageBox.StandardButton.Yes:
                event.ignore()
                return
        self.settings.setValue('geometry', self.saveGeometry())
        # Replay integration may be processing a large event; cooperatively stop it
        # and keep the window alive until its QThread has actually returned.
        if self.replay_worker:
            self.replay_worker.cancel()
            if self.replay_thread:
                self.replay_thread.quit()
                if not self.replay_thread.wait(5000):
                    event.ignore()
                    self.status.setText('Waiting for safe replay cancellation…')
                    return
                self.replay_worker.deleteLater()
                self.replay_thread.deleteLater()
                self.replay_worker = self.replay_thread = None
        if self.worker:
            self.worker.cancel()
            if self.thread:
                self.thread.quit()
                if not self.thread.wait(10000):
                    event.ignore()
                    self.status.setText('Waiting for safe fitting cancellation…')
                    return
            self.worker.deleteLater()
            self.thread.deleteLater()
            self.worker = self.thread = None
        event.accept()
