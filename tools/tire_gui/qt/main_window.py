"""Native PyQt interface for TTC tire correlation."""
from dataclasses import replace
from pathlib import Path

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
        self.generation = 0
        self.worker = self.thread = self.candidate = None
        self.history = []
        self.history_index = -1
        self.source_files = []
        self.selection = None
        self.holdout_ids = set()

        root = QWidget(self)
        layout = QVBoxLayout(root)
        bar = QHBoxLayout()
        self.open_button = QPushButton('Open XML')
        self.seed_button = QPushButton('Import MATLAB seed')
        self.data_button = QPushButton('Data folder')
        self.save_button = QPushButton('Save fit as…')
        for button in (self.open_button, self.seed_button, self.data_button, self.save_button):
            bar.addWidget(button)
        layout.addLayout(bar)

        self.tabs = QTabWidget()
        self.steady_plot = PlotPanel(count=4)
        self.transient_plot = PlotPanel(count=4)
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
        self.segment_list = QListWidget()
        self.event_list = QListWidget()
        self.event_from = QSpinBox()
        self.event_to = QSpinBox()
        self.baseline_to = QSpinBox()
        self.event_from.setPrefix('Start sample ')
        self.event_to.setPrefix('Stop sample ')
        self.baseline_to.setPrefix('Baseline stop ')
        self.approve_button = QPushButton('Approve selected event')
        for label, widget in (
            ('Run / tire', self.run_combo), ('Segments — check to hold out whole segment', self.segment_list),
            ('Transient candidates (Run 7)', self.event_list), ('', self.event_from),
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
        self.progress = QProgressBar()
        self.status = QLabel('No model loaded')
        actions = QHBoxLayout()
        for button in (self.fit_button, self.fit_transient_button, self.fit_longitudinal_button,
                       self.cancel_button, self.apply_button, self.discard_button):
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
        self.save_button.clicked.connect(self._save_dialog)
        self.fit_button.clicked.connect(self.start_steady_fit)
        self.fit_transient_button.clicked.connect(self.start_transient_fit)
        self.cancel_button.clicked.connect(self.cancel_fit)
        self.apply_button.clicked.connect(self._apply_current_candidate)
        self.discard_button.clicked.connect(self.discard_candidate)
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

    def load_data_path(self, path):
        path = Path(path)
        run = load_run(path, require_monotonic_time=False)
        self.runs[run.run_id] = run
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
        for axis in self.transient_plot.axes:
            axis.grid(True, alpha=.25)
            axis.legend(fontsize='small')
            axis.set_xlabel('Elapsed time (s)')
        self.transient_plot.figure.tight_layout()
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
            self.fit_transient_button.setEnabled(True)
            self.approve_button.setEnabled(False)
            self.status.setText('Event explicitly approved after boundary/condition validation.')
            self._refresh_transient_event()
            return True
        except Exception as exc:
            self.status.setText(f'Approval refused: {exc}')
            return False

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

    def _refresh_run(self, *_):
        run_id = self.run_combo.currentData()
        run = self.runs.get(run_id)
        self.event_list.clear()
        self.segment_list.clear()
        self.events = []
        self.holdout_ids = set()
        if not run:
            self._refresh_plots()
            return
        if run_id == 7:
            try:
                self.events = list(candidate_events(run))
            except Exception as exc:
                self.status.setText(str(exc))
            for index, event in enumerate(self.events):
                reasons = '; '.join(event.exclusions) or 'review required'
                self.event_list.addItem(f'Candidate {index + 1}: {run.channels["time"][event.start]:.2f}–{run.channels["time"][event.stop - 1]:.2f} s; baseline {event.baseline_stop - event.start} samples; {reasons}')
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
        else:
            # IDs match the compatibility selector's lateral / longitudinal groups.
            count = 16 if run_id == 8 else 36 if run_id == 72 else 12
            for index in range(count):
                item = QListWidgetItem(f'Run {run_id} source-compatible segment {index + 1} — Train')
                item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
                item.setCheckState(Qt.CheckState.Unchecked)
                item.setData(Qt.ItemDataRole.UserRole, (run_id, index))
                self.segment_list.addItem(item)
        self.fit_transient_button.setEnabled(any(event.approved for event in self.events))
        self.fit_longitudinal_button.setEnabled(any(event.approved and event.axis == 'longitudinal' for event in self.events))
        self._refresh_plots()

    @staticmethod
    def _without_segments(samples, excluded):
        mask = [not (samples.run_id, int(segment)) in excluded for segment in samples.segment_ids]
        from ..contracts import SampleSet
        return SampleSet(samples.alpha[mask], samples.kappa[mask], samples.fz[mask], samples.target[mask],
                         samples.segment_ids[mask], samples.run_id, samples.tire_id, samples.raw_indices[mask])

    def _selection_for_fit(self):
        lat = next(run for run in self.runs.values() if run.run_id == 8)
        lon = next(run for run in self.runs.values() if run.run_id == 72)
        data = compatibility_selection(lat, lon)
        excluded = set(self.holdout_ids)
        return replace(data,
                       lateral=self._without_segments(data.lateral, excluded),
                       longitudinal=self._without_segments(data.longitudinal, excluded),
                       combined_fx=self._without_segments(data.combined_fx, excluded),
                       combined_fy=self._without_segments(data.combined_fy, excluded),
                       mz=self._without_segments(data.mz, excluded))

    def start_steady_fit(self):
        try:
            self.selection = self._selection_for_fit()
            self._start_worker('steady', self.selection, ())
        except Exception as exc:
            self.status.setText(f'Steady fit requires compatible runs 8 and 72: {exc}')

    def start_transient_fit(self):
        approved = tuple(event for event in self.events if event.approved)
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
        self.cancel_button.setEnabled(True)
        self._fit_generation = self.generation
        self.thread = QThread(self)
        self.worker = FitWorker(self.snapshot, mode, selection, self.parameter_panel.locked(), events, self.parameter_panel.bounds())
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.progress.connect(self._progress)
        self.worker.finished.connect(self._fit_done)
        self.worker.failed.connect(self._fit_failed)
        self.worker.cancelled.connect(self._fit_cancelled)
        self.worker.finished.connect(self.thread.quit)
        self.worker.failed.connect(self.thread.quit)
        self.worker.cancelled.connect(self.thread.quit)
        self.thread.finished.connect(self._worker_stopped)
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
        self.status.setText('Candidate discarded; live model unchanged.')

    def _refresh_plots(self):
        if self.snapshot is None:
            return
        if 8 not in self.runs or 72 not in self.runs:
            self.steady_plot.message('Load private TTC runs 8 and 72 to compare measured / current / seed predictions.\n43075 Fy/Mz; 43100 Fx and combined-slip proxy.')
            self._refresh_diagnostics()
            return
        try:
            from ..model import predict_steady
            data = compatibility_selection(self.runs[8], self.runs[72])
            groups = [
                ('Fy vs alpha', data.lateral, 'lateral_force', data.lateral.alpha, 'alpha (rad)', 'Fy (N)'),
                ('Fx vs kappa', data.longitudinal, 'longitudinal_force', data.longitudinal.kappa, 'kappa', 'Fx (N)'),
                ('Combined slip', data.combined_fx, 'longitudinal_force', data.combined_fx.alpha, 'alpha / combined command', 'Fx/Fz'),
                ('Mz vs alpha', data.mz, 'aligning_moment', data.mz.alpha, 'alpha (rad)', 'Mz (N·m)'),
            ]
            for axis, (title, samples, component, x, xlabel, ylabel) in zip(self.steady_plot.axes, groups):
                axis.clear()
                denominator = samples.fz if '/Fz' in ylabel else 1
                measured = samples.target / denominator
                prediction = predict_steady(self.snapshot, samples, data.belt_speed)
                current = getattr(prediction, component) / denominator
                order = x.argsort()
                axis.scatter(x, measured, s=5, c='#555555', label='Measured', alpha=.65)
                axis.plot(x[order], current[order], label='Current fit', color='#1665a8')
                if self.seed:
                    seed = getattr(predict_steady(self.seed, samples, data.belt_speed), component) / denominator
                    axis.plot(x[order], seed[order], label='Seed', color='#bd5a20', linestyle='--')
                axis.plot(x[order], (measured - current)[order], label='Residual', color='#8a3b3b', alpha=.7)
                axis.set_title(title)
                axis.set_xlabel(xlabel)
                axis.set_ylabel(ylabel)
                axis.grid(True, alpha=.25)
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
        lines.extend(f'{name} bounds: {bounds[0]:g} … {bounds[1]:g}' for name, bounds in self.parameter_panel.bounds().items())
        if result:
            lines.extend((f'converged: {result.converged}', f'metrics: {dict(result.metrics)}', *result.warnings))
        self.diagnostics.setPlainText('\n'.join(lines))

    def save_session(self, output_dir, stem, overwrite=False):
        if self.snapshot is None:
            raise ValueError('no model loaded')
        metadata = {
            'locks': {name: name in self.parameter_panel.locked() for name in self.snapshot.values},
            'bounds': {name: list(pair) for name, pair in self.parameter_panel.bounds().items()},
            'source_files': self.source_files,
            'diagnostics': {'generation': self.generation},
        }
        return save_session(self.snapshot, Path(output_dir), stem, include_transient=True, overwrite=overwrite, metadata=metadata)

    def _save_dialog(self):
        if self.snapshot is None:
            return
        stem, accepted = QInputDialog.getText(self, 'Save fit bundle', 'Private bundle name:')
        if not accepted:
            return
        destination = DEFAULT_OUTPUT / stem.strip()
        overwrite = False
        if destination.exists():
            if QMessageBox.question(self, 'Overwrite private bundle?', f'Overwrite recognized bundle {destination}?') != QMessageBox.StandardButton.Yes:
                return
            overwrite = True
        try:
            files = self.save_session(DEFAULT_OUTPUT, stem.strip(), overwrite)
            self.status.setText('Saved private validated bundle: ' + ', '.join(path.name for path in files))
        except Exception as exc:
            self.status.setText(f'Save refused: {exc}')

    def load_session_path(self, path):
        snapshot = load_session(Path(path))
        metadata = snapshot.session_metadata
        self.snapshot = snapshot
        self.generation += 1
        self.source_files = list(metadata.get('source_files', []))
        self._history_reset()
        self.parameter_panel.set_snapshot(snapshot)
        self.status.setText('Session restored; source references retained, samples not embedded.')
        return snapshot

    def closeEvent(self, event):
        self.settings.setValue('geometry', self.saveGeometry())
        if self.worker:
            self.worker.cancel()
            if self.thread:
                self.thread.quit()
                self.thread.wait(10000)
                if self.thread.isRunning():
                    event.ignore()
                    self.status.setText('Waiting for safe fitting cancellation…')
                    return
            self.worker.deleteLater()
            self.worker = self.thread = None
        event.accept()
