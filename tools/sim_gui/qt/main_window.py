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

from tools.sim_gui import REPOSITORY_ROOT
from tools.sim_gui.execute import prepare_case, validate_case
from tools.sim_gui.qt.model_panel import ModelPanel
from tools.sim_gui.qt.queue import CaseQueue, QueueTable
from tools.sim_gui.qt.settings_tab import SettingsTab
from tools.sim_gui.qt.study_tab import StudyTab
from tools.sim_gui.qt.track_tab import TrackTab
from tools.sim_gui.qt.workers import ValidationWorker
from tools.sim_gui.run_study import _study_directory
from tools.sim_gui.study import MAXIMUM_CASES, StudyDefinition, StudyIndex, WARNING_CASES, _atomic, expand

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
        self._prepared = []
        self._running = False
        self._run_requested = False
        self._queue = CaseQueue(parent=self)
        self._validation_thread = None
        self._validation_worker = None
        self._validation_cancelled = False
        self._progress_dialog = None
        self._queue.caseFinished.connect(self._case_finished)
        self._queue.caseProgress.connect(self._case_progress)
        self._queue.allFinished.connect(self._all_finished)

        self.model_panel = ModelPanel(parent=self)
        self.track_tab = TrackTab(self)
        self.settings_tab = SettingsTab(self)
        self.study_tab = None
        self.queue_table = QueueTable(self)
        self.log_view = QPlainTextEdit(self)
        self.log_view.setReadOnly(True)
        self.tabs = QTabWidget(self)
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
        self._build_actions()
        self._log_timer = QTimer(self)
        self._log_timer.setInterval(1000)
        self._log_timer.timeout.connect(self._refresh_log)
        self.status = self.statusBar()
        self._recent = list(self.settings_store.value("recent_files", []))
        geometry = self.settings_store.value("geometry")
        if geometry is not None:
            self.restoreGeometry(geometry)

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
        self.run_action = QAction("Run", self)
        self.run_action.triggered.connect(self.run_cases)
        self.stop_action = QAction("Stop", self)
        self.stop_action.triggered.connect(self.stop_cases)
        self.run_action.setShortcut("F5")
        self.save_action.setShortcut("Ctrl+S")
        self.open_output_action = QAction("Open Output Folder", self)
        self.open_output_action.triggered.connect(self.open_output_folder)
        self.quit_action = QAction("Quit", self)
        self.quit_action.triggered.connect(self.close)
        actions = (self.open_action, self.open_study_action, self.save_action,
                   self.build_action, self.run_action, self.stop_action,
                   self.open_output_action, self.quit_action)
        for action in actions:
            toolbar.addAction(action)
        menu = self.menuBar().addMenu("File")
        menu.addActions(actions[:3])
        menu.addSeparator()
        menu.addActions(actions[3:])
        self.stop_action.setEnabled(False)

    def _configuration_changed(self, configuration) -> None:
        self.configuration = configuration
        if self.study_tab is not None:
            self.study_tab.configuration = configuration
            self.study_tab.settings = self.settings_tab._settings
        self.settings_tab.set_model(configuration.model)
        self.run_action.setEnabled(configuration.runnable and not self._running)

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
        self.configuration = __import__("tools.sim_gui.vehicle", fromlist=["VehicleConfiguration"]).VehicleConfiguration.open(path)
        self.model_panel.set_configuration(self.configuration)
        self.settings_tab.set_model(self.configuration.model)
        try:
            current_settings = self.settings_tab.settings()
        except ValueError:
            current_settings = self.settings_tab._settings
        self._replace_study_tab(StudyTab(self.configuration, current_settings, self))
        self.run_action.setEnabled(self.configuration.runnable)
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
        definition = self._definition()
        try:
            cases = expand(definition)
        except ValueError as error:
            _warn(self, "Study", str(error))
            return
        if len(cases) > MAXIMUM_CASES:
            _warn(self, "Study", f"study has {len(cases)} cases; maximum is {MAXIMUM_CASES}")
            return
        if len(cases) > WARNING_CASES and _confirm(self, "Study", f"Build {len(cases)} cases?") != QMessageBox.StandardButton.Yes:
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
        self._validation_thread = None
        self._validation_worker = None
        if self._progress_dialog is not None:
            # QProgressDialog.close() emits canceled; a normal finish is not a cancel.
            self._progress_dialog.canceled.disconnect(self._cancel_validation)
            self._progress_dialog.close()
            self._progress_dialog = None
        if self._validation_cancelled:
            self._run_requested = False
            self._prepared = []
            self.run_action.setEnabled(self.configuration is not None and self.configuration.runnable)
            self._validation_cancelled = False
            return
        self.queue_table.set_cases(self.study_index.cases)
        self._refresh_status()
        if self._run_requested:
            self._run_requested = False
            if self._prepared:
                self.run_cases()
            else:
                self.status.showMessage("No valid cases to run")
                self.run_action.setEnabled(self.configuration is not None and self.configuration.runnable)

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
        self._update_case(case_id, directory=f"cases/{case_id}", status="invalid", error=error)

    def _update_case(self, case_id: str, **fields) -> None:
        if self.study_index is None:
            return
        self.study_index.update(case_id, **fields)
        self.queue_table.set_cases(self.study_index.cases)
        self._refresh_status()

    def run_cases(self) -> None:
        """Build a baseline if needed and start the process queue."""
        if not self._prepared:
            self._run_requested = True
            self.build_cases()
        if self._prepared and self._validation_thread is None:
            self.queue_table.set_cases(self.study_index.cases)
            self._running = True
            self.run_action.setEnabled(False)
            self.stop_action.setEnabled(True)
            self._log_timer.start()
            parallel = self.study_tab.parallel_cases.value() if self.study_tab is not None else 1
            self._queue.start(self._prepared, parallel)

    def stop_cases(self) -> None:
        """Stop all running cases."""
        self._queue.stop()

    def save_study(self) -> None:
        """Save the current definition atomically to a study JSON file."""
        path, _ = QFileDialog.getSaveFileName(self, "Save Study", filter="Study (*.study.json)")
        if path:
            _atomic(Path(path), self._definition().to_json())
            self._remember(Path(path))

    def open_study(self) -> None:
        """Open a definition.study.json and restore its model."""
        path, _ = QFileDialog.getOpenFileName(self, "Open Study", filter="JSON (*.json)")
        if not path:
            return
        definition = StudyDefinition.from_json(json.loads(Path(path).read_text()))
        from tools.sim_gui.vehicle import VehicleConfiguration
        baseline = VehicleConfiguration.open(Path(definition.source))
        configuration = baseline.with_model(definition.model)
        for key, value in definition.choices.items():
            if configuration.choices.get(key) != Path(value).resolve():
                configuration = configuration.with_choice(key, Path(value))
        self.configuration = configuration
        self.model_panel.set_configuration(configuration, baseline=baseline)
        self.settings_tab.set_model(configuration.model)
        self.settings_tab.set_settings(definition.settings)
        if definition.track:
            self.track_tab.set_track(Path(definition.track))
        tab = StudyTab(configuration, definition.settings, self)
        tab.set_definition(definition)
        self._replace_study_tab(tab)
        self.run_action.setEnabled(configuration.runnable)
        self._remember(Path(path))

    def _case_finished(self, key, fields) -> None:
        if self.study_index is not None:
            self.queue_table.set_cases(self.study_index.cases)
        self._refresh_status()

    def _case_progress(self, key, value) -> None:
        self._refresh_log()

    def _all_finished(self) -> None:
        self._running = False
        self.run_action.setEnabled(self.configuration is not None and self.configuration.runnable)
        self.stop_action.setEnabled(False)
        self._log_timer.stop()

    def _refresh_log(self) -> None:
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
