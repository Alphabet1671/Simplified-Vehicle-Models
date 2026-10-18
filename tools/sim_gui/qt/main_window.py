"""Main window for the simulation GUI."""
from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

from PyQt6.QtCore import QEventLoop, QSettings, QTimer, QUrl, Qt, pyqtSignal
from PyQt6.QtGui import QAction
from PyQt6.QtWidgets import (
    QDockWidget,
    QFileDialog,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
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
from tools.sim_gui.qt.workers import ValidationWorker
from tools.sim_gui.run_study import _study_directory
from tools.sim_gui.study import MAXIMUM_CASES, StudyDefinition, StudyIndex, WARNING_CASES, expand

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
        self._queue = CaseQueue(parent=self)
        self._queue.caseFinished.connect(self._case_finished)
        self._queue.caseProgress.connect(self._case_progress)
        self._queue.allFinished.connect(self._all_finished)

        self.model_panel = ModelPanel(parent=self)
        self.settings_tab = SettingsTab(self)
        self.study_tab = None
        self.queue_table = QueueTable(self)
        self.log_view = QPlainTextEdit(self)
        self.log_view.setReadOnly(True)
        self.tabs = QTabWidget(self)
        self.tabs.addTab(self.settings_tab, "Run Settings")
        self.tabs.addTab(self.queue_table, "Queue")
        self.tabs.addTab(self.log_view, "Log")
        self.setCentralWidget(self.tabs)
        dock = QDockWidget("Model", self)
        dock.setWidget(self.model_panel)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, dock)

        self._build_actions()
        self._log_timer = QTimer(self)
        self._log_timer.setInterval(1000)
        self._log_timer.timeout.connect(self._refresh_log)
        self.status = self.statusBar()
        self._recent = list(self.settings_store.value("recent_files", []))

    def _build_actions(self) -> None:
        toolbar = QToolBar("Simulation", self)
        self.addToolBar(toolbar)
        self.open_action = QAction("Open Vehicle Model…", self)
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
        for action in (self.open_action, self.open_study_action, self.save_action, self.build_action, self.run_action, self.stop_action):
            toolbar.addAction(action)
        self.stop_action.setEnabled(False)

    def open_model(self, path: Path) -> None:
        """Open a vehicle configuration and refresh dependent tabs."""
        self.configuration = __import__("tools.sim_gui.vehicle", fromlist=["VehicleConfiguration"]).VehicleConfiguration.open(path)
        self.model_panel.set_configuration(self.configuration)
        self.settings_tab._model = self.configuration.model
        try:
            current_settings = self.settings_tab.settings()
        except ValueError:
            current_settings = self.settings_tab._settings
        self.study_tab = StudyTab(self.configuration, current_settings, self)
        self.tabs.insertTab(1, self.study_tab, "Study")
        self.run_action.setEnabled(self.configuration.runnable)
        self._remember(Path(path))

    def _open_dialog(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Open Vehicle Model", filter="XML (*.xml)")
        if path:
            self.open_model(Path(path))

    def _remember(self, path: Path) -> None:
        values = [str(path)] + [item for item in self._recent if item != str(path)]
        self._recent = values[:10]
        self.settings_store.setValue("recent_files", self._recent)

    def _definition(self) -> StudyDefinition:
        if self.study_tab is None:
            return StudyDefinition("study", str(self.configuration.source), self.model_panel.track().as_posix(), self.configuration.model, {key: str(value) for key, value in self.configuration.choices.items()}, self.settings_tab.settings(), "one-at-a-time", ())
        definition = self.study_tab.definition(self.configuration)
        track = self.model_panel.track()
        return replace(
            definition,
            track=track.as_posix() if track is not None else "",
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
        worker = ValidationWorker(study_dir, definition, cases, self)
        worker.caseReady.connect(self._case_ready)
        worker.finished.connect(worker.deleteLater)
        worker.run()
        self.queue_table.set_cases(self.study_index.cases)
        self._refresh_status()

    def _case_ready(self, prepared, error) -> None:
        if error is None:
            self._prepared.append(prepared)

    def run_cases(self) -> None:
        """Build a baseline if needed and start the process queue."""
        if not self._prepared:
            self.build_cases()
        if self._prepared:
            self.queue_table.set_cases(self.study_index.cases)
            self._running = True
            self.run_action.setEnabled(False)
            self.stop_action.setEnabled(True)
            self._queue.start(self._prepared, 1)

    def stop_cases(self) -> None:
        """Stop all running cases."""
        self._queue.stop()

    def save_study(self) -> None:
        """Save the current definition to a study JSON file."""
        path, _ = QFileDialog.getSaveFileName(self, "Save Study", filter="Study (*.study.json)")
        if path:
            Path(path).write_text(json.dumps(self._definition().to_json(), indent=2))

    def open_study(self) -> None:
        """Open a definition.study.json and restore its model."""
        path, _ = QFileDialog.getOpenFileName(self, "Open Study", filter="JSON (*.json)")
        if not path:
            return
        definition = StudyDefinition.from_json(json.loads(Path(path).read_text()))
        self.open_model(Path(definition.source))
        self.settings_tab.set_settings(definition.settings)
        self.study_tab.set_definition(definition)

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
        logs = sorted(self.study_index.directory.glob("cases/*/attempt_*/run.log"), key=lambda p: p.stat().st_mtime)
        if logs:
            self.log_view.setPlainText("\n".join(logs[-1].read_text(errors="replace").splitlines()[-2000:]))

    def _refresh_status(self) -> None:
        counts = {}
        if self.study_index is not None:
            for case in self.study_index.cases:
                counts[case.get("status", "pending")] = counts.get(case.get("status", "pending"), 0) + 1
        self.status.showMessage(" · ".join(f"{key} {value}" for key, value in sorted(counts.items())))

    def closeEvent(self, event) -> None:
        """Stop active processes before closing; always shut down the queue."""
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
        event.accept()
