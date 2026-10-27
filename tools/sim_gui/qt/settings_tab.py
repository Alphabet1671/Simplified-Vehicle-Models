"""Qt widget for point-mass and yaw-aware simulation settings."""
from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QFileDialog, QComboBox, QDoubleSpinBox, QFormLayout, QHBoxLayout, QLabel,
    QLineEdit, QPushButton, QSpinBox, QWidget,
)

from tools.sim_gui.settings import RunSettings, validate


class SettingsTab(QWidget):
    settingsChanged = pyqtSignal(object)
    FLOAT_FIELDS = (
        "point_mass_duration", "point_mass_spacing", "point_mass_maximum_speed",
        "point_mass_pedal_travel_time", "point_mass_preview_seconds", "point_mass_heading_gain",
        "point_mass_lateral_gain", "point_mass_rtol", "spacing", "control_spacing",
        "profile_seconds", "line_seconds", "duration", "replay_seconds", "maximum_step",
    )
    INT_FIELDS = ("profile_rounds", "line_evaluations", "line_outer_passes", "line_optimizer_iterations")
    POINT_MASS_FIELDS = frozenset(FLOAT_FIELDS[:8])
    LABELS = {
        "point_mass_duration": "Duration (s)", "point_mass_spacing": "Spacing (m)",
        "point_mass_maximum_speed": "Maximum speed (m/s)",
        "point_mass_pedal_travel_time": "Pedal travel time (s)",
        "point_mass_preview_seconds": "Preview horizon (s)", "point_mass_heading_gain": "Heading gain",
        "point_mass_lateral_gain": "Lateral gain", "point_mass_rtol": "Relative tolerance",
        "spacing": "Profile spacing (m)", "control_spacing": "Line control spacing (m)",
        "profile_seconds": "Profile budget (s)", "line_seconds": "Line budget (s)",
        "duration": "Replay duration (s)", "replay_seconds": "Replay budget (s)",
        "maximum_step": "Maximum integration step (s)", "profile_rounds": "Profile rounds",
        "line_evaluations": "Line evaluations", "line_outer_passes": "Line outer passes",
        "line_optimizer_iterations": "Line optimizer iterations",
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self._settings = RunSettings.for_model("planar-7dof")
        self._model = "planar-7dof"
        self._widgets = {}
        self._form = QFormLayout(self)
        self.solver_kind = QComboBox()
        self.solver_kind.addItem("Point-mass quick investigation", "point-mass")
        self.solver_kind.addItem("Yaw-aware transient", "yaw-aware")
        self.solver_kind.setCurrentIndex(self.solver_kind.findData(self._settings.solver_kind))
        self.solver_kind.currentIndexChanged.connect(self._changed)
        self._form.addRow("Solver workflow", self.solver_kind)
        self.prerequisite = QLabel("Yaw-aware requires an existing completed, identity-checked vehicle-limit yaw map; this GUI never rebuilds it.")
        self.prerequisite.setWordWrap(True)
        self._form.addRow(self.prerequisite)
        self.limitation = QLabel("Yaw-aware output is sampled/discrete evidence, not continuous feasibility or containment certification.")
        self.limitation.setWordWrap(True)
        self._form.addRow(self.limitation)
        self.yaw_map = QLineEdit()
        self.yaw_map.textChanged.connect(self._changed)
        self.map_row = QWidget()
        row = QHBoxLayout(self.map_row)
        row.setContentsMargins(0, 0, 0, 0)
        row.addWidget(self.yaw_map)
        self.browse_map = QPushButton("Browse…")
        self.browse_map.clicked.connect(self._browse_map)
        row.addWidget(self.browse_map)
        self._form.addRow("Completed yaw map", self.map_row)
        self.mode = QComboBox()
        self.mode.addItems(("fixed", "line", "replay"))
        self.mode.currentTextChanged.connect(self._mode_changed)
        self._form.addRow("Yaw mode", self.mode)
        self.plan = QLineEdit()
        self.plan.textChanged.connect(self._changed)
        self.plan_row = QWidget()
        row = QHBoxLayout(self.plan_row)
        row.setContentsMargins(0, 0, 0, 0)
        row.addWidget(self.plan)
        self.browse_plan = QPushButton("Browse…")
        self.browse_plan.clicked.connect(self._browse_plan)
        row.addWidget(self.browse_plan)
        self._form.addRow("Replay plan", self.plan_row)
        for name in self.FLOAT_FIELDS:
            widget = QDoubleSpinBox()
            widget.setRange(0.000001, 1e9)
            widget.setDecimals(6)
            widget.setValue(getattr(self._settings, name))
            widget.valueChanged.connect(self._changed)
            self._widgets[name] = widget
            setattr(self, name, widget)
            self._form.addRow(self.LABELS[name], widget)
        for name in self.INT_FIELDS:
            widget = QSpinBox()
            widget.setRange(1, 1_000_000)
            widget.setValue(getattr(self._settings, name))
            widget.valueChanged.connect(self._changed)
            self._widgets[name] = widget
            setattr(self, name, widget)
            self._form.addRow(self.LABELS[name], widget)
        self.controller = QComboBox()
        self.controller.addItems(("preview", "model-aware"))
        self.controller.currentTextChanged.connect(self._changed)
        self._form.addRow("Controller", self.controller)
        self._sync_visibility()

    def _browse_map(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select completed yaw map", filter="Yaw map (*.npz)")
        if path:
            self.yaw_map.setText(path)

    def _browse_plan(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select saved yaw plan", filter="Yaw plan (*.npz)")
        if path:
            self.plan.setText(path)

    def _mode_changed(self, *_args):
        self._sync_visibility()
        self._changed()

    def _sync_visibility(self):
        yaw = self.solver_kind.currentData() == "yaw-aware"
        for widget in (self.prerequisite, self.limitation, self.map_row, self.mode):
            widget.setVisible(yaw)
        self.plan_row.setVisible(yaw and self.mode.currentText() == "replay")
        for name in self.FLOAT_FIELDS:
            self._widgets[name].setVisible((name not in self.POINT_MASS_FIELDS) if yaw else (name in self.POINT_MASS_FIELDS))
        for name in self.INT_FIELDS:
            self._widgets[name].setVisible(yaw)
        self.controller.setVisible(yaw)
        hidden_fields = {widget for name, widget in self._widgets.items() if not widget.isVisible()}
        hidden_fields.update(widget for widget in (self.map_row, self.plan_row) if not widget.isVisible())
        for row in range(self._form.rowCount()):
            field = self._form.itemAt(row, QFormLayout.ItemRole.FieldRole)
            label = self._form.itemAt(row, QFormLayout.ItemRole.LabelRole)
            if field and label and field.widget() in hidden_fields:
                label.widget().setVisible(False)

    def settings(self):
        return self._settings

    def set_settings(self, settings: RunSettings):
        self._settings = settings
        self.solver_kind.setCurrentIndex(self.solver_kind.findData(settings.solver_kind))
        self.yaw_map.setText(settings.yaw_map)
        self.mode.setCurrentText(settings.mode)
        self.plan.setText(settings.plan or "")
        for name, widget in self._widgets.items():
            widget.setValue(getattr(settings, name))
        self.controller.setCurrentText(settings.controller)
        self._sync_visibility()
        self.settingsChanged.emit(settings)

    def set_model(self, name: str):
        self._model = name

    def _changed(self, *_args):
        values = {name: widget.value() for name, widget in self._widgets.items()}
        values.update(solver_kind=self.solver_kind.currentData(), yaw_map=self.yaw_map.text().strip(),
                      mode=self.mode.currentText(), plan=self.plan.text().strip() or None,
                      controller=self.controller.currentText())
        self._settings = RunSettings(**values)
        self._sync_visibility()
        self.settingsChanged.emit(self._settings)
