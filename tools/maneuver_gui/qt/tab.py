"""Simulation-only maneuver tab with lap-style run controls."""
from pathlib import Path

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QFileDialog, QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QLabel,
    QPushButton, QLineEdit, QProgressBar, QPlainTextEdit,
)

from .editor import ManeuverEditor
from .preview import PreviewWidget
from tools.sim_gui.qt.maneuver_tab import ManeuverDraft

MAXIMUM_LOG_LINES = 2000
DEFAULT_EXPORT_FOLDER = Path.cwd() / "runs"


class ManeuverTab(QWidget):
    runRequested = pyqtSignal(object)
    stopRequested = pyqtSignal()
    calibrationRequested = pyqtSignal(object)
    animationRequested = pyqtSignal(object, object)

    def __init__(self, config=None, parent=None):
        super().__init__(parent)
        self.editor = None
        self._calibration_context = None
        self._csv_snapshot = None
        self._preflight_error = ""
        self._busy = False

        self.backend_label = QLabel("Backend: simulation", self)
        self.shared_model_label = QLineEdit(self)
        self.shared_model_label.setReadOnly(True)
        self.shared_model_label.setPlaceholderText("Select a vehicle in the shared Model dock")
        self.export_edit = QLineEdit(str(DEFAULT_EXPORT_FOLDER), self)
        self.export_browse = QPushButton("Browse…", self)
        self.run_button = QPushButton("Run", self)
        self.stop_button = QPushButton("Stop", self)
        self.progress_bar = QProgressBar(self)
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.status_label = QLabel("Ready", self)
        self.result_label = QLabel(self)
        self.result_label.setWordWrap(True)
        self.result_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.log_view = QPlainTextEdit(self)
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(MAXIMUM_LOG_LINES)
        self.animation_button = QPushButton("Export animation…", self)
        self.animation_button.setEnabled(False)
        self.calibration_button = QPushButton("Calibration", self)
        self.hardware_arm_button = QPushButton("Arm hardware", self)
        self.hardware_arm_button.setEnabled(False)
        self.preview = PreviewWidget(self)

        form = QFormLayout()
        form.addRow("Vehicle model (shared)", self.shared_model_label)
        export_row = QHBoxLayout()
        export_row.addWidget(self.export_edit, 1)
        export_row.addWidget(self.export_browse)
        form.addRow("Export folder", export_row)
        run_row = QHBoxLayout()
        run_row.addWidget(self.run_button)
        run_row.addWidget(self.stop_button)
        run_row.addWidget(self.progress_bar, 1)
        run_row.addWidget(self.animation_button)

        self.layout = QVBoxLayout(self)
        self.layout.addWidget(self.backend_label)
        self.layout.addLayout(form)
        self.layout.addLayout(run_row)
        self.layout.addWidget(self.status_label)
        self.layout.addWidget(self.result_label)
        self.layout.addWidget(self.preview, 1)
        self.layout.addWidget(self.log_view)
        self.layout.addWidget(self.calibration_button)
        self.layout.addWidget(self.hardware_arm_button)

        self.run_button.clicked.connect(self._run)
        self.stop_button.clicked.connect(self.stopRequested.emit)
        self.calibration_button.clicked.connect(self._calibrate)
        self.export_browse.clicked.connect(self._browse_export)
        self.animation_button.clicked.connect(self._browse_animation)
        if config is not None:
            self.set_vehicle_configuration(config)
        self.set_busy(False)

    def set_vehicle_configuration(self, configuration):
        if self.editor is not None:
            self.layout.removeWidget(self.editor)
            self.editor.deleteLater()
        self.editor = ManeuverEditor(configuration, self)
        self.layout.insertWidget(2, self.editor)
        model_name = getattr(getattr(configuration, "model", None), "name", None)
        if model_name is None and isinstance(configuration, dict):
            model_name = configuration.get("model", {}).get("name") if isinstance(configuration.get("model"), dict) else configuration.get("model_name")
        self.shared_model_label.setText(str(model_name or "Shared model configuration loaded"))
        self._refresh_validity()
        return self.editor

    def set_export_folder(self, path: Path):
        self.export_edit.setText(str(Path(path)))

    def export_folder(self) -> Path:
        text = self.export_edit.text().strip()
        return Path(text) if text else DEFAULT_EXPORT_FOLDER

    def set_progress(self, fraction: float):
        self.progress_bar.setValue(max(0, min(100, int(round(float(fraction) * 100)))))

    def append_log(self, text: str):
        self.log_view.appendPlainText(str(text))

    def clear_run(self):
        self.progress_bar.setValue(0)
        self.status_label.setText("Ready")
        self.result_label.clear()
        self.log_view.clear()
        self.preview.set_trace([], [], [])
        self.animation_button.setEnabled(False)

    def set_completed_result(self, summary: dict, artifact: Path, time_s, requested, applied):
        self.preview.set_trace(list(time_s), list(requested), list(applied))
        self._has_completed_artifact = True
        self._completed_artifact = Path(artifact)
        reason = summary.get("termination_reason", "completed") if isinstance(summary, dict) else "completed"
        self.result_label.setText(f"Termination: {reason}\nArtifact: {Path(artifact)}")
        self.status_label.setText("Completed")
        self.animation_button.setEnabled(not self._busy)

    def set_preflight_context(self, calibration=None, csv_snapshot=None):
        """Set calibration metadata and loaded-CSV snapshot used before runs."""
        self._calibration_context = calibration
        self._csv_snapshot = csv_snapshot
        self._preflight_error = ""
        return self._refresh_validity()

    def _refresh_validity(self):
        definition, errors = self.editor.validate() if self.editor else (None, {"definition": "vehicle configuration required"})
        if definition is None:
            self.status_label.setText("; ".join(errors.values()))
        elif not self._preflight_error:
            self.status_label.setText("Ready")
        self.run_button.setEnabled(definition is not None and not self._busy)
        return definition, errors

    def definition(self):
        definition, errors = self._refresh_validity()
        if definition is None:
            raise ValueError("; ".join(errors.values()))
        return definition

    def set_busy(self, busy):
        self._busy = bool(busy)
        self.run_button.setEnabled(not self._busy and self._refresh_validity()[0] is not None)
        self.export_edit.setEnabled(not self._busy)
        self.export_browse.setEnabled(not self._busy)
        self.animation_button.setEnabled(not self._busy and getattr(self, "_has_completed_artifact", False))
        self.calibration_button.setEnabled(not self._busy)
        self.stop_button.setEnabled(self._busy)
        if self.editor:
            self.editor.setEnabled(not self._busy)

    def set_result(self, summary):
        requested = summary.get("requested") if isinstance(summary, dict) else None
        applied = summary.get("applied") if isinstance(summary, dict) else None
        self.preview.set_state(requested, applied)
        self.status_label.setText(str(summary.get("status", "Complete")) if isinstance(summary, dict) else str(summary))

    def _run(self):
        try:
            definition = self.definition()
        except ValueError as exc:
            self._preflight_error = str(exc)
            self.status_label.setText(self._preflight_error)
            self._refresh_validity()
            return
        context = self._calibration_context or {}
        result = ManeuverDraft.from_dict(definition.to_dict()).preflight(
            calibration_status=context.get("status"),
            calibration_fingerprint=context.get("definition_sha256", context.get("fingerprint")),
            expected_fingerprint=context.get("expected_fingerprint"),
            csv_snapshot=self._csv_snapshot,
        )
        if not result.ok:
            self._preflight_error = "; ".join(result.errors.values())
            self.status_label.setText(self._preflight_error)
            self.run_button.setEnabled(False)
            return
        self._preflight_error = ""
        self.status_label.setText("Ready")
        self.run_button.setEnabled(not self._busy)
        self.runRequested.emit(definition)

    def _browse_export(self):
        folder = QFileDialog.getExistingDirectory(self, "Select export folder", str(self.export_folder()))
        if folder:
            self.set_export_folder(Path(folder))

    def _browse_animation(self):
        if self._busy or not self._has_completed_artifact:
            return
        output, _ = QFileDialog.getSaveFileName(self, "Export animation", "", "MP4 (*.mp4)")
        if output:
            self.animationRequested.emit(self._completed_artifact, Path(output))

    def _calibrate(self):
        if self.editor:
            self.calibrationRequested.emit(self.editor.to_dict())
