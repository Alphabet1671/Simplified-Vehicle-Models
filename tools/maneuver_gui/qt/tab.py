"""Minimal simulation-only maneuver tab."""
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton
from .editor import ManeuverEditor
from .preview import PreviewWidget

class ManeuverTab(QWidget):
    runRequested = pyqtSignal(object)
    stopRequested = pyqtSignal()
    calibrationRequested = pyqtSignal(object)

    def __init__(self, config=None, parent=None):
        super().__init__(parent)
        self.editor = None
        self.backend_label = QLabel("Backend: simulation", self)
        self.status_label = QLabel("Ready", self)
        self.run_button = QPushButton("Run", self)
        self.stop_button = QPushButton("Stop", self)
        self.calibration_button = QPushButton("Calibration", self)
        self.hardware_arm_button = QPushButton("Arm hardware", self)
        self.hardware_arm_button.setEnabled(False)
        self.preview = PreviewWidget(self)
        self.layout = QVBoxLayout(self)
        for widget in (self.backend_label, self.status_label, self.preview, self.run_button, self.stop_button, self.calibration_button, self.hardware_arm_button): self.layout.addWidget(widget)
        self.run_button.clicked.connect(self._run)
        self.stop_button.clicked.connect(self.stopRequested.emit)
        self.calibration_button.clicked.connect(self._calibrate)
        if config is not None: self.set_vehicle_configuration(config)
        self.set_busy(False)

    def set_vehicle_configuration(self, configuration):
        if self.editor is not None:
            self.layout.removeWidget(self.editor); self.editor.deleteLater()
        self.editor = ManeuverEditor(configuration, self)
        self.layout.insertWidget(2, self.editor)
        self._refresh_validity()
        return self.editor

    def _refresh_validity(self):
        definition, errors = self.editor.validate() if self.editor else (None, {"definition": "vehicle configuration required"})
        self.run_button.setEnabled(definition is not None and not getattr(self, "_busy", False))
        return definition, errors

    def definition(self):
        definition, errors = self._refresh_validity()
        if definition is None: raise ValueError("; ".join(errors.values()))
        return definition

    def set_busy(self, busy):
        self._busy = bool(busy)
        self.run_button.setEnabled(not self._busy and self._refresh_validity()[0] is not None)
        self.calibration_button.setEnabled(not self._busy)
        self.stop_button.setEnabled(self._busy)
        if self.editor: self.editor.setEnabled(not self._busy)

    def set_result(self, summary):
        requested = summary.get("requested") if isinstance(summary, dict) else None
        applied = summary.get("applied") if isinstance(summary, dict) else None
        self.preview.set_state(requested, applied)
        self.status_label.setText(str(summary.get("status", "Complete")) if isinstance(summary, dict) else str(summary))

    def _run(self):
        try: definition = self.definition()
        except ValueError: return
        self.runRequested.emit(definition)

    def _calibrate(self):
        if self.editor: self.calibrationRequested.emit(self.editor.to_dict())
