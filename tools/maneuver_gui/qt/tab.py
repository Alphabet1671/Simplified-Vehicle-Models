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
        self.editor = ManeuverEditor(config, self) if config is not None else None
        self.backend_label = QLabel("Backend: simulation", self)
        self.run_button = QPushButton("Run", self)
        self.stop_button = QPushButton("Stop", self)
        self.calibration_button = QPushButton("Calibration", self)
        self.hardware_arm_button = QPushButton("Arm hardware", self)
        self.hardware_arm_button.setEnabled(False)
        self.preview = PreviewWidget(self)
        layout = QVBoxLayout(self)
        layout.addWidget(self.backend_label)
        if self.editor:
            layout.addWidget(self.editor)
        layout.addWidget(self.preview)
        for button in (self.run_button, self.stop_button, self.calibration_button, self.hardware_arm_button):
            layout.addWidget(button)
        self.run_button.setEnabled(self.editor is not None and self.editor.validate()[0] is not None)
        self.run_button.clicked.connect(self._run)
        self.stop_button.clicked.connect(self.stopRequested.emit)
        self.calibration_button.clicked.connect(self._calibrate)
        if self.editor:
            self.preview.set_state(self.editor.to_dict())

    def _run(self):
        if self.editor is not None:
            definition, errors = self.editor.validate()
            if definition is not None:
                self.runRequested.emit(definition)

    def _calibrate(self):
        self.calibrationRequested.emit(self.editor.to_dict() if self.editor else {})
