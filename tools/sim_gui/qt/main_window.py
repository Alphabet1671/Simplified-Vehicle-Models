"""Main window for the simulation GUI."""
from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

from PyQt6.QtCore import QEventLoop, QSettings, QThread, QTimer, QUrl, Qt
from PyQt6.QtGui import QAction
from PyQt6.QtWidgets import (
    QDockWidget,
    QFileDialog,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressDialog,
    QTabWidget,
    QToolBar,
)
from PyQt6.QtGui import QDesktopServices

from tools.gui_common.files import protected_directory_problem
from tools.sim_gui import REPOSITORY_ROOT
from tools.sim_gui.execute import prepare_case, validate_case, child_command, finish_case, new_attempt, parse_progress
from tools.sim_gui.maneuver_execute import (
    ManeuverStudyDefinition, create_maneuver_study, expand_maneuvers,
    finish_maneuver_case, maneuver_child_command, maneuver_progress,
    new_maneuver_attempt, prepare_maneuver_case,
)
from tools.sim_gui.qt.model_panel import ModelPanel
from tools.maneuver_gui.qt import ManeuverTab
from tools.maneuver_gui.qt.controller import ManeuverController
from tools.sim_gui.qt.queue import CaseQueue, QueueTable
from tools.sim_gui.qt.settings_tab import SettingsTab
from tools.sim_gui.qt.single_run_tab import SingleRunTab
from tools.sim_gui.qt.study_tab import StudyTab
from tools.sim_gui.qt.track_tab import TrackTab
from tools.sim_gui.qt.workers import ValidationWorker
from tools.sim_gui.run_study import _study_directory
from tools.sim_gui.study import MAXIMUM_CASES, StudyDefinition, StudyIndex, WARNING_CASES, _atomic, expand
from tools.sim_gui.vehicle import VehicleConfiguration

_warn = QMessageBox.warning
_confirm = QMessageBox.question


class MainWindow(QMainWindow):
    """Coordinate model selection, study preparation, and case execution."""

    def __init__(self, output_root=REPOSITORY_ROOT / "runs", settings_store=None, parent=None) -> None:
        """Create the main simulation GUI window."""
        super().__init__(parent)
        self.output_root = Path(output_root)
        self.settings_store = settings_store or QSettings("Simplified-Vehicle-Models", "sim-gui")
        self.configuration = None
        self.study_index = None
        self._maneuver_study: ManeuverStudyDefinition | None = None
        self._prepared = []
        self._running = False
        self._run_requested = False
        self._run_mode = None
        self._syncing_track = False
        self._case_error = None
        self._queue = CaseQueue(parent=self)
        self._validation_thread = None
        self._validation_worker = None
        self._validation_cancelled = False
        self._progress_dialog = None
        self._queue.caseFinished.connect(self._case_finished)
        self._queue.caseProgress.connect(self._case_progress)
        self._queue.caseProgress.connect(self._single_progress)
        self._queue.allFinished.connect(self._all_finished)

        self.model_panel = ModelPanel(parent=self)
        self.single_tab = SingleRunTab(self)
        self.maneuver_tab = ManeuverTab(parent=self)
        self.maneuver_controller = ManeuverController(self.maneuver_tab, output_root=self.output_root, parent=self)
        self.track_tab = TrackTab(self)
        self.settings_tab = SettingsTab(self)
        self.study_tab = None
        self.queue_table = QueueTable(self)
        self.log_view = QPlainTextEdit(self)
        self.log_view.setReadOnly(True)
        self.tabs = QTabWidget(self)
        self.tabs.addTab(self.single_tab, "Single Run")
        self.tabs.addTab(self.maneuver_tab, "Maneuvers")
        self.tabs.addTab(self.track_tab, "Track")
        self.tabs.addTab(self.settings_tab, "Run Settings")
        self.tabs.addTab(self.queue_table, "Queue")
        self.tabs.addTab(self.log_view, "Log")
        self.setCentralWidget(self.tabs)
        dock = QDockWidget("Model", self)
        dock.setWidget(self.model_panel)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, dock)

        self.model_panel.configurationChanged.connect(self._configuration_changed)
        self.settings_tab.settingsChanged.connect(self._settings_changed)
        self.queue_table.itemSelectionChanged.connect(self._selection_changed)
        self.single_tab.goRequested.connect(self.run_single)
        self.single_tab.stopRequested.connect(self.stop_cases)
        self.single_tab.modelSelected.connect(self._single_model_selected)
        self.single_tab.trackSelected.connect(self._single_track_selected)
        self.maneuver_tab.runRequested.connect(self._run_maneuver)
        self.maneuver_tab.stopRequested.connect(self.stop_cases)
        self.maneuver_controller.busyChanged.connect(self._maneuver_busy_changed)
        self.maneuver_controller.progress.connect(self._maneuver_progress)
        self.maneuver_controller.finished.connect(self._maneuver_finished)
        self.maneuver_controller.animationFinished.connect(self._maneuver_animation_finished)
        self.maneuver_tab.log_view.textChanged.connect(self._mirror_maneuver_log)
        self.track_tab.trackChanged.connect(self._track_changed)
        stored = self.settings_store.value("single_run_output")
        self.single_tab.set_export_folder(Path(stored) if stored else self.output_root)
        self.single_tab.set_track(self.track_tab.track())
        self._build_actions()
        self._log_timer = QTimer(self)
        self._log_timer.setInterval(1000)
        self._log_timer.timeout.connect(self._refresh_log)
        self._single_log_timer = QTimer(self)
        self._single_log_timer.setInterval(1000)
        self._single_log_timer.timeout.connect(self._refresh_single_log)
        self.status = self.statusBar()
        self._recent = list(self.settings_store.value("recent_files", []))
        geometry = self.settings_store.value("geometry")
        if geometry is not None:
            self.restoreGeometry(geometry)
        self._refresh_actions()

    def _build_actions(self) -> None:
        toolbar = QToolBar("Simulation", self)
        self.addToolBar(toolbar)
        self.open_action = QAction("Open Vehicle Model…", self)
        self.open_action.setShortcut("Ctrl+O")
        self.open_action.triggered.connect(self._open_dialog)
        self.save_action = QAction("Save Study…", self)
        self.save_action.triggered.connect(self.save_study)
        self.open_study_action = QAction("Open Study…", self)
        self.open_study_action.triggered.connect(self.open_study)
        self.build_action = QAction("Build Cases", self)
        self.build_action.triggered.connect(self.build_cases)
        self.go_action = QAction("Go", self)
        self.go_action.triggered.connect(self.run_single)
        self.run_study_action = QAction("Run Study", self)
        self.run_study_action.triggered.connect(self.run_cases)
        self.stop_action = QAction("Stop", self)
        self.stop_action.triggered.connect(self.stop_cases)
        self.go_action.setShortcut("F5")
        self.save_action.setShortcut("Ctrl+S")
        self.open_output_action = QAction("Open Output Folder", self)
        self.open_output_action.triggered.connect(self.open_output_folder)
        self.quit_action = QAction("Quit", self)
        self.quit_action.triggered.connect(self.close)
        actions = (self.open_action, self.open_study_action, self.save_action,
                   self.build_action, self.go_action, self.run_study_action, self.stop_action,
                   self.open_output_action, self.quit_action)
        for action in actions:
            toolbar.addAction(action)
        menu = self.menuBar().addMenu("File")
        menu.addActions(actions[:3])
        menu.addSeparator()
        menu.addActions(actions[3:])
        self.stop_action.setEnabled(False)

    def _refresh_actions(self) -> None:
        """Enable the run actions only for a runnable, idle configuration."""
        enabled = (
            self.configuration is not None
            and self.configuration.runnable
            and self._run_mode is None
        )
        self.go_action.setEnabled(enabled)
        self.run_study_action.setEnabled(enabled)
        self.stop_action.setEnabled(self._run_mode is not None)

    def _configuration_changed(self, configuration) -> None:
        self.configuration = configuration
        self.maneuver_tab.set_vehicle_configuration(self._maneuver_definition_seed(configuration))
        self.maneuver_tab.clear_run()
        if self.study_tab is not None:
            self.study_tab.configuration = configuration
            self.study_tab.settings = self.settings_tab._settings
        self.settings_tab.set_model(configuration.model)
        self._refresh_actions()

    def _maneuver_definition_seed(self, configuration) -> dict:
        """Create the initial simulation-only maneuver draft for a selected model."""
        try:
            model = configuration.source.resolve().relative_to(REPOSITORY_ROOT).as_posix()
        except ValueError:
            model = str(configuration.source)
        return {
            "schema": "maneuver-v1", "name": "maneuver", "preset": "step_steer",
            "backend": "simulation", "vehicle": {"model": model},
            "initial": {"speed_mps": 10.0, "x_m": 0.0, "y_m": 0.0, "heading_rad": 0.0},
            "clock": {"controller_tick_s": 0.01},
            "steering": {"mode": "peak_swa", "amplitude_rad": 0.1, "direction": 1},
            "longitudinal": {"mode": "constant_accelerator", "accelerator": 0.1},
            "profile": {"duration_s": 3.0}, "limits": {},
            "termination": {"duration_s": 3.0}, "calibration": None, "csv": None,
        }

    def _run_maneuver(self, definition) -> None:
        """Build a native one-case study and run it through the shared queue."""
        if self.configuration is None or self._run_mode is not None:
            return
        data = definition.to_dict() if hasattr(definition, "to_dict") else dict(definition)
        source = data.get("vehicle", {}).get("formulation") or data.get("vehicle", {}).get("model")
        if not source:
            _warn(self, "Maneuver", "Select a vehicle model before running a maneuver.")
            return
        study = ManeuverStudyDefinition(str(data.get("name") or "maneuver"), str(source), (data,))
        try:
            cases = expand_maneuvers(study)
            self._maneuver_study = study
            directory = _study_directory(self.maneuver_tab.export_folder(), study)
            self.study_index = create_maneuver_study(directory, study, cases)
            self._prepared = [prepare_maneuver_case(directory, study, case) for case in cases]
        except (OSError, ValueError) as error:
            _warn(self, "Maneuver", str(error))
            return
        self._queue.study_index = self.study_index
        self._queue.command_factory = maneuver_child_command
        self._queue.finish_factory = finish_maneuver_case
        self._queue.attempt_factory = new_maneuver_attempt
        self._queue.progress_factory = maneuver_progress
        self.queue_table.set_cases(self.study_index.cases)
        self.maneuver_tab.clear_run()
        self.maneuver_tab.set_busy(True)
        self._run_mode = "maneuver"
        self._running = True
        self._log_timer.start()
        self.tabs.setCurrentWidget(self.queue_table)
        self._refresh_actions()
        self._queue.start(self._prepared, 1)

    def _run_maneuver_study(self, study: ManeuverStudyDefinition) -> None:
        """Run an opened multi-definition maneuver study through CaseQueue."""
        try:
            cases = expand_maneuvers(study)
            directory = _study_directory(self.maneuver_tab.export_folder(), study)
            self.study_index = create_maneuver_study(directory, study, cases)
            self._prepared = [prepare_maneuver_case(directory, study, case) for case in cases]
        except (OSError, ValueError) as error:
            _warn(self, "Maneuver Study", str(error))
            return
        self._maneuver_study = study
        self._queue.study_index = self.study_index
        self._queue.command_factory = maneuver_child_command
        self._queue.finish_factory = finish_maneuver_case
        self._queue.attempt_factory = new_maneuver_attempt
        self._queue.progress_factory = maneuver_progress
        self.queue_table.set_cases(self.study_index.cases)
        self.maneuver_tab.clear_run()
        self.maneuver_tab.set_busy(True)
        self._run_mode = "maneuver"
        self._running = True
        self._log_timer.start()
        self.tabs.setCurrentWidget(self.queue_table)
        self._refresh_actions()
        self._queue.start(self._prepared, 1)

    def _maneuver_busy_changed(self, busy: bool) -> None:
        if busy:
            self._run_mode = "maneuver"
            self.status.showMessage("Maneuver running…")
        elif self._run_mode == "maneuver":
            self._run_mode = None
        self._refresh_actions()

    def _mirror_maneuver_log(self) -> None:
        """Mirror newly appended tab log lines into the application log."""
        text = self.maneuver_tab.log_view.toPlainText()
        count = getattr(self, "_maneuver_log_lines", 0)
        lines = text.splitlines()
        if count > len(lines):
            count = 0
        for line in lines[count:]:
            self.log_view.appendPlainText(f"maneuver: {line}")
        self._maneuver_log_lines = len(lines)

    def _maneuver_progress(self, event: dict) -> None:
        """Forward simulation progress into the shared status bar."""
        if not isinstance(event, dict):
            return
        fraction = event.get("fraction")
        phase = event.get("phase", "running")
        time_s = event.get("time_s")
        detail = f"Maneuver: {phase}"
        if time_s is not None:
            detail += f" at {float(time_s):.3f} s"
        if isinstance(fraction, (int, float)):
            detail += f" ({float(fraction) * 100:.0f}%)"
        self.status.showMessage(detail)

    def _maneuver_finished(self, summary: dict) -> None:
        status = summary.get("status", "Finished") if isinstance(summary, dict) else "Finished"
        self.status.showMessage(f"Maneuver {status.lower()}")

    def _maneuver_animation_finished(self, summary: dict) -> None:
        status = summary.get("status", "Animation finished") if isinstance(summary, dict) else "Animation finished"
        self.status.showMessage(f"Maneuver {status.lower()}")

    def _settings_changed(self, settings) -> None:
        if self.study_tab is not None:
            self.study_tab.settings = settings

    def open_output_folder(self) -> None:
        folder = self.study_index.directory if self.study_index is not None else self.output_root
        folder.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(folder)))

    def _selection_changed(self) -> None:
        self._refresh_log()

    def open_model(self, path: Path) -> None:
        """Open a vehicle configuration and refresh dependent tabs."""
        self.configuration = VehicleConfiguration.open(path)
        self.model_panel.set_configuration(self.configuration)
        self.maneuver_tab.set_vehicle_configuration(self._maneuver_definition_seed(self.configuration))
        self.maneuver_tab.clear_run()
        self.settings_tab.set_model(self.configuration.model)
        try:
            current_settings = self.settings_tab.settings()
        except ValueError:
            current_settings = self.settings_tab._settings
        self._replace_study_tab(StudyTab(self.configuration, current_settings, self))
        self.single_tab.set_model(Path(path))
        self._refresh_actions()
        self._remember(Path(path))

    def _replace_study_tab(self, tab: StudyTab) -> None:
        """Install one Study tab before the Queue tab and drop the previous one."""
        previous = self.study_tab
        self.study_tab = tab
        self.tabs.insertTab(self.tabs.indexOf(self.queue_table), tab, "Study")
        if previous is not None:
            index = self.tabs.indexOf(previous)
            if index >= 0:
                self.tabs.removeTab(index)
            previous.deleteLater()

    def _open_dialog(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Open Vehicle Model", filter="XML (*.xml)")
        if path:
            self.open_model(Path(path))

    def _remember(self, path: Path) -> None:
        values = [str(path)] + [item for item in self._recent if item != str(path)]
        self._recent = values[:10]
        self.settings_store.setValue("recent_files", self._recent)

    def _definition(self) -> StudyDefinition:
        track = self.track_tab.track()
        track_text = track.as_posix() if track is not None else ""
        if self.study_tab is None:
            return StudyDefinition(
                "study",
                str(self.configuration.source),
                track_text,
                self.configuration.model,
                {key: str(value) for key, value in self.configuration.choices.items()},
                self.settings_tab.settings(),
                "one-at-a-time",
                (),
            )
        definition = self.study_tab.definition(self.configuration)
        return replace(
            definition,
            track=track_text,
            settings=self.settings_tab.settings(),
        )

    def build_cases(self) -> None:
        """Prepare and validate the current study cases."""
        if self._run_mode is not None or self._validation_thread is not None:
            return
        definition = self._definition()
        if not definition.track:
            _warn(self, "Study", "Select a track model.")
            return
        try:
            cases = expand(definition)
        except ValueError as error:
            _warn(self, "Study", str(error))
            return
        if len(cases) > MAXIMUM_CASES:
            _warn(self, "Study", f"study has {len(cases)} cases; maximum is {MAXIMUM_CASES}")
            return
        if (
            len(cases) > WARNING_CASES
            and _confirm(self, "Study", f"Build {len(cases)} cases?")
            != QMessageBox.StandardButton.Yes
        ):
            return
        study_dir = _study_directory(self.output_root, definition)
        self.study_index = StudyIndex.create(study_dir, definition, cases)
        self._queue.study_index = self.study_index
        self._prepared = []
        self.queue_table.set_cases(self.study_index.cases)
        self._refresh_status()
        thread = QThread(self)
        worker = ValidationWorker(study_dir, definition, cases)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.caseReady.connect(self._case_ready)
        worker.casePrepareFailed.connect(self._mark_invalid)
        worker.progress.connect(self._validation_progress)
        dialog = QProgressDialog("Building cases…", "Cancel", 0, len(cases), self)
        dialog.setAutoClose(False)
        dialog.canceled.connect(self._cancel_validation)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self._validation_finished)
        self._validation_thread = thread
        self._validation_worker = worker
        dialog.show()
        self._progress_dialog = dialog
        thread.start()

    def _cancel_validation(self) -> None:
        self._validation_cancelled = True
        if self._validation_worker is not None:
            self._validation_worker.cancel()

    def _validation_progress(self, done, total) -> None:
        if self._progress_dialog is not None:
            self._progress_dialog.setMaximum(total)
            self._progress_dialog.setValue(done)

    def _validation_finished(self) -> None:
        single = self._run_mode == "single"
        self._validation_thread = None
        self._validation_worker = None
        if self._progress_dialog is not None:
            # QProgressDialog.close() emits canceled; a normal finish is not a cancel.
            self._progress_dialog.canceled.disconnect(self._cancel_validation)
            self._progress_dialog.close()
            self._progress_dialog = None
        if self._validation_cancelled:
            self._validation_cancelled = False
            self._prepared = []
            if single:
                self._finish_single_without_queue("cancelled")
            else:
                self._run_requested = False
                self._refresh_actions()
            return
        if single:
            prepared = self._prepared[0] if self._prepared else None
            self._prepared = []
            if prepared is None:
                self._finish_single_without_queue(
                    "failed", self._case_error or "case could not be prepared"
                )
                return
            self._running = True
            self.single_tab.set_status("running")
            self._single_log_timer.start()
            self._queue.start([prepared], 1)
            self.single_tab.set_running(True)
            self._refresh_actions()
            return
        self.queue_table.set_cases(self.study_index.cases)
        self._refresh_status()
        if self._run_requested:
            self._run_requested = False
            if self._prepared:
                self.run_cases()
            else:
                self.status.showMessage("No valid cases to run")
                self._refresh_actions()

    def _finish_single_without_queue(self, status: str, result: str | None = None) -> None:
        """Leave a single run that never reached the queue in a clean state."""
        self._run_mode = None
        self._running = False
        self._single_log_timer.stop()
        self.single_tab.set_running(False)
        self.single_tab.set_status(status)
        self.single_tab.set_result(result if result is not None else "")
        self._refresh_actions()

    def _case_ready(self, prepared, error) -> None:
        """Keep a valid case, or record an invalid one as run_study does."""
        if prepared is None:
            return  # casePrepareFailed records it, because only it knows the case id.
        case_id = prepared.directory.name
        if error is None:
            self._prepared.append(prepared)
            self._update_case(case_id, directory=f"cases/{case_id}", status="pending", error=None)
        else:
            self._mark_invalid(case_id, error)

    def _mark_invalid(self, case_id: str, error: str) -> None:
        self._case_error = error
        if self._run_mode == "single":
            self.single_tab.set_result(error)
        self._update_case(case_id, directory=f"cases/{case_id}", status="invalid", error=error)

    def _update_case(self, case_id: str, **fields) -> None:
        if self.study_index is None:
            return
        self.study_index.update(case_id, **fields)
        if self._run_mode == "single":
            return
        self.queue_table.set_cases(self.study_index.cases)
        self._refresh_status()

    def run_cases(self) -> None:
        """Build a baseline if needed and start the study process queue."""
        if self._run_mode is not None or self._validation_thread is not None:
            return
        if self.tabs.currentWidget() is self.maneuver_tab and self._maneuver_study is not None:
            self._run_maneuver_study(self._maneuver_study)
            return
        if not self._prepared:
            self._run_requested = True
            self.build_cases()
        if self._prepared and self._validation_thread is None:
            self._start_study_queue()

    def _start_study_queue(self) -> None:
        """Show the prepared cases in the Queue tab and start them."""
        self._run_mode = "study"
        self.queue_table.set_cases(self.study_index.cases)
        self._running = True
        self._log_timer.start()
        self._refresh_actions()
        parallel = self.study_tab.parallel_cases.value() if self.study_tab is not None else 1
        self._queue.start(self._prepared, parallel)

    def stop_cases(self) -> None:
        """Stop the active run, cancelling validation when it has not started."""
        if self._run_mode == "maneuver":
            self._queue.stop()
            return
        if self._run_mode == "single" and self._validation_thread is not None:
            self._cancel_validation()
            return
        self._queue.stop()

    def run_single(self) -> None:
        """Prepare and run one case outside the Queue and Log tabs."""
        if self.configuration is None or not self.configuration.runnable:
            return
        if self._run_mode is not None or self._validation_thread is not None:
            return
        track = self.single_tab.track_path()
        if track is None:
            _warn(self, "Single run", "Select a track model.")
            return
        export_folder = self.single_tab.export_folder()
        problem = self._export_folder_problem(export_folder)
        if problem is not None:
            _warn(self, "Single run", problem)
            return
        try:
            export_folder.mkdir(parents=True, exist_ok=True)
        except OSError as error:
            _warn(self, "Single run", f"cannot create {export_folder}: {error}")
            return
        self.settings_store.setValue("single_run_output", str(export_folder))
        definition = StudyDefinition(
            "single-run",
            str(self.configuration.source),
            track.as_posix(),
            self.configuration.model,
            {key: str(value) for key, value in self.configuration.choices.items()},
            self.settings_tab.settings(),
            "one-at-a-time",
            (),
        )
        try:
            cases = expand(definition)
        except ValueError as error:
            _warn(self, "Single run", str(error))
            return
        if len(cases) != 1:
            _warn(self, "Single run", f"single run expanded to {len(cases)} cases")
            return
        study_dir = _study_directory(export_folder, definition)
        index = StudyIndex.create(study_dir, definition, cases)
        self.study_index = index
        self._queue.study_index = index
        self._prepared = []
        self._case_error = None
        self._validation_cancelled = False
        self._run_mode = "single"
        self.single_tab.clear_run()
        self.single_tab.set_log("")
        self.single_tab.set_status("Preparing…")
        self.single_tab.set_running(True)
        self._refresh_actions()
        thread = QThread(self)
        worker = ValidationWorker(study_dir, definition, cases)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.caseReady.connect(self._case_ready)
        worker.casePrepareFailed.connect(self._mark_invalid)
        worker.progress.connect(self._validation_progress)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self._validation_finished)
        self._validation_thread = thread
        self._validation_worker = worker
        thread.start()

    def _export_folder_problem(self, folder: Path) -> str | None:
        """Return a warning message for an unsafe export folder."""
        problem = protected_directory_problem(folder, allow_models=False)
        if problem is not None:
            return problem
        if folder.exists() and not folder.is_dir():
            return f"{folder} is not a directory"
        return None

    def _single_model_selected(self, path) -> None:
        """Open a model chosen in the single tab, or restore the old selection."""
        previous = self.configuration.source if self.configuration is not None else None
        try:
            self.open_model(Path(path))
        except (OSError, ValueError) as error:
            _warn(self, "Vehicle model", str(error))
            self.single_tab.set_model(previous)
            self._refresh_actions()

    def _single_track_selected(self, path) -> None:
        """Mirror a single-tab track choice into the Track tab."""
        if self._syncing_track:
            return
        self._syncing_track = True
        try:
            self.track_tab.set_track(Path(path))
        finally:
            self._syncing_track = False
        # The Track tab refuses an unreadable file; show what it accepted.
        accepted = self.track_tab.track()
        if accepted is not None:
            self.single_tab.set_track(accepted)

    def _track_changed(self, path) -> None:
        """Mirror a Track tab choice into the single-run tab."""
        if self._syncing_track:
            return
        self._syncing_track = True
        try:
            self.single_tab.set_track(Path(path))
        finally:
            self._syncing_track = False

    def _single_progress(self, key, value) -> None:
        """Show queue progress in the single tab while a single run is active."""
        if self._run_mode != "single":
            return
        self.single_tab.set_progress(value)
        self._refresh_single_log()

    def _refresh_single_log(self) -> None:
        """Tail the active single-run attempt log into the single-run tab."""
        if self.study_index is None or not self.study_index.cases:
            return
        attempt = self.study_index.cases[0].get("attempt")
        if not attempt:
            return
        log = self.study_index.directory / attempt / "run.log"
        if log.exists():
            self.single_tab.set_log(log.read_text(errors="replace"))

    def save_study(self) -> None:
        """Save a lap study or the current maneuver definition collection."""
        path, _ = QFileDialog.getSaveFileName(self, "Save Study", filter="Study (*.study.json)")
        if not path:
            return
        if self.tabs.currentWidget() is self.maneuver_tab:
            data = self.maneuver_tab.definition().to_dict()
            source = data.get("vehicle", {}).get("formulation") or data.get("vehicle", {}).get("model", "")
            _atomic(Path(path), ManeuverStudyDefinition(data.get("name", "maneuver"), str(source), (data,)).to_json())
        else:
            _atomic(Path(path), self._definition().to_json())
        self._remember(Path(path))

    def open_study(self) -> None:
        """Open a definition.study.json and restore its model."""
        path, _ = QFileDialog.getOpenFileName(self, "Open Study", filter="JSON (*.json)")
        if not path:
            return
        raw = json.loads(Path(path).read_text())
        if raw.get("schema") == "maneuver-study-v1":
            maneuver = ManeuverStudyDefinition.from_json(raw)
            try:
                cases = expand_maneuvers(maneuver)
            except ValueError as error:
                _warn(self, "Maneuver Study", str(error))
                return
            baseline = VehicleConfiguration.open(Path(maneuver.source))
            self.configuration = baseline
            self.model_panel.set_configuration(baseline)
            self._maneuver_study = maneuver
            # Edit the first member while retaining the full study collection for
            # explicit Run Study, rather than discarding batch definitions.
            self.maneuver_tab.set_vehicle_configuration(cases[0].definition)
            self.maneuver_tab.clear_run()
            self.tabs.setCurrentWidget(self.maneuver_tab)
            self._refresh_actions()
            self._remember(Path(path))
            return
        definition = StudyDefinition.from_json(raw)
        baseline = VehicleConfiguration.open(Path(definition.source))
        configuration = baseline.with_model(definition.model)
        for key, value in definition.choices.items():
            if configuration.choices.get(key) != Path(value).resolve():
                configuration = configuration.with_choice(key, Path(value))
        self.configuration = configuration
        self.model_panel.set_configuration(configuration, baseline=baseline)
        self.settings_tab.set_model(configuration.model)
        self.settings_tab.set_settings(definition.settings)
        self.single_tab.set_model(Path(definition.source))
        if definition.track:
            self.track_tab.set_track(Path(definition.track))
        tab = StudyTab(configuration, definition.settings, self)
        tab.set_definition(definition)
        self._replace_study_tab(tab)
        self._refresh_actions()
        self._remember(Path(path))

    def _case_finished(self, key, fields) -> None:
        if self._run_mode == "maneuver":
            if self.study_index is not None:
                case = next((item for item in self.study_index.cases if item.get("id") == key), {})
                attempt = case.get("attempt")
                if attempt:
                    self.maneuver_tab.result_label.setText(
                        f"Status: {case.get('status', 'failed')}\nAttempt: {self.study_index.directory / attempt}\n"
                        f"RunRecord: {case.get('run_record') or 'not generated'}")
                if case.get("status") == "completed" and case.get("run_record"):
                    self.maneuver_tab.animation_button.setEnabled(True)
                    self.maneuver_tab._completed_artifact = self.study_index.directory / attempt
                    self.maneuver_tab._has_completed_artifact = True
            self.queue_table.set_cases(self.study_index.cases)
            self._refresh_status()
            return
        if self._run_mode == "single":
            self.single_tab.set_status(str(fields.get("status", "")))
            self._refresh_single_log()
            return
        if self.study_index is not None:
            self.queue_table.set_cases(self.study_index.cases)
        self._refresh_status()

    def _case_progress(self, key, value) -> None:
        if self._run_mode == "single":
            return
        self._refresh_log()

    def _all_finished(self) -> None:
        """Finish the active mode without changing the other mode's widgets."""
        mode = self._run_mode
        # The queue is configured per run kind; restore its lap defaults once
        # a maneuver queue has drained so subsequent lap studies are unchanged.
        if mode == "maneuver":
            self._queue.command_factory = child_command
            self._queue.finish_factory = finish_case
            self._queue.attempt_factory = new_attempt
            self._queue.progress_factory = parse_progress
            self.maneuver_tab.set_busy(False)
            self._running = False
            self._log_timer.stop()
            self._run_mode = None
            self._refresh_actions()
            return
        self._running = False
        self._log_timer.stop()
        self._single_log_timer.stop()
        if mode == "single":
            self.single_tab.set_running(False)
            case = self.study_index.cases[0] if self.study_index is not None else {}
            self.single_tab.set_status(str(case.get("status", "failed")))
            self.single_tab.set_result(self._single_result_text(case))
            self._run_mode = None
            self._refresh_actions()
            return
        self._run_mode = None
        self._refresh_actions()

    def _single_result_text(self, case: dict) -> str:
        """Return the lap time, record path, or error text for one finished case."""
        summary = case.get("summary") or {}
        lap_time = summary.get("lap_time")
        if lap_time is not None:
            return f"lap time {lap_time} s"
        record = case.get("run_record")
        if record:
            base = self.study_index.directory if self.study_index is not None else Path()
            return str(base / record)
        return str(case.get("error") or "")

    def _refresh_log(self) -> None:
        if self._run_mode == "single":
            return
        if self.study_index is None:
            return
        row = self.queue_table.currentRow()
        if row < 0 or row >= len(self.study_index.cases):
            return
        case = self.study_index.cases[row]
        attempt = case.get("attempt")
        if not attempt:
            return
        log = self.study_index.directory / attempt / "run.log"
        if log.exists():
            self.log_view.setPlainText("\n".join(log.read_text(errors="replace").splitlines()[-2000:]))

    def _refresh_status(self) -> None:
        if self._run_mode == "single":
            return
        counts = {}
        if self.study_index is not None:
            for case in self.study_index.cases:
                counts[case.get("status", "pending")] = counts.get(case.get("status", "pending"), 0) + 1
        self.status.showMessage(" · ".join(f"{key} {value}" for key, value in sorted(counts.items())))

    def closeEvent(self, event) -> None:
        """Stop active processes before closing; always shut down the queue."""
        if self._validation_thread is not None:
            self._validation_thread.quit()
            self._validation_thread.wait(10000)
        if self.maneuver_controller.is_running():
            self.maneuver_controller.cancel()
            maneuver_loop = QEventLoop()
            self.maneuver_controller.finished.connect(maneuver_loop.quit)
            QTimer.singleShot(10000, maneuver_loop.quit)
            maneuver_loop.exec()
            if self.maneuver_controller.is_running():
                event.ignore()
                return
        if self._running:
            answer = _confirm(self, "Stop the running cases and quit?", "Stop the running cases and quit?")
            if answer != QMessageBox.StandardButton.Yes:
                event.ignore()
                return
            self._queue.stop()
            loop = QEventLoop()
            self._queue.allFinished.connect(loop.quit)
            QTimer.singleShot(10000, loop.quit)
            loop.exec()
        self._queue.shutdown()
        self.settings_store.setValue("geometry", self.saveGeometry())
        event.accept()
