"""Qt settings widget for forward-driver and point-mass workflows."""
from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QComboBox, QDoubleSpinBox, QFormLayout, QLabel, QSpinBox, QWidget

from tools.sim_gui.settings import RunSettings


class SettingsTab(QWidget):
    settingsChanged = pyqtSignal(object)
    FLOAT_FIELDS = (
        "point_mass_duration", "point_mass_spacing", "point_mass_maximum_speed",
        "point_mass_pedal_travel_time", "point_mass_preview_seconds", "point_mass_heading_gain",
        "point_mass_lateral_gain", "point_mass_rtol", "forward_duration", "forward_wall_seconds",
        "forward_maximum_speed", "forward_spacing", "forward_deceleration_margin",
        "forward_feedforward_distance",
    )
    INT_FIELDS = ("forward_speed_points",)
    POINT_MASS_FIELDS = frozenset(FLOAT_FIELDS[:8])
    LABELS = {
        "point_mass_duration": "Duration (s)", "point_mass_spacing": "Spacing (m)",
        "point_mass_maximum_speed": "Maximum speed (m/s)",
        "point_mass_pedal_travel_time": "Pedal travel time (s)",
        "point_mass_preview_seconds": "Preview horizon (s)", "point_mass_heading_gain": "Heading gain",
        "point_mass_lateral_gain": "Lateral gain", "point_mass_rtol": "Relative tolerance",
        "forward_duration": "Forward simulated duration (s)", "forward_wall_seconds": "Forward runtime budget (s)",
        "forward_maximum_speed": "Forward speed cap (m/s)", "forward_spacing": "Envelope spacing (m)",
        "forward_deceleration_margin": "Deceleration margin", "forward_feedforward_distance": "Chord distance (m)",
        "forward_speed_points": "Envelope speed points",
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self._settings = RunSettings.for_model("planar-7dof")
        self._model = "planar-7dof"
        self._widgets = {}
        self._form = QFormLayout(self)
        self.solver_kind = QComboBox()
        self.solver_kind.addItem("Forward driver (braking envelope)", "forward-driver")
        self.solver_kind.addItem("Point-mass quick investigation", "point-mass")
        self.solver_kind.setCurrentIndex(0)
        self.solver_kind.currentIndexChanged.connect(self._mode_changed)
        self._form.addRow("Solver workflow", self.solver_kind)
        self.forward_note = QLabel("Current continuous braking-envelope policy with PreviewDriver steering and actuator dynamics; sampled envelope is not a safety certificate.")
        self.forward_note.setWordWrap(True)
        self._form.addRow(self.forward_note)
        for name in self.FLOAT_FIELDS:
            widget = QDoubleSpinBox()
            widget.setRange(0.000001, 1e9)
            widget.setDecimals(6)
            widget.setValue(getattr(self._settings, name))
            widget.valueChanged.connect(self._changed)
            widget.setObjectName(name)
            self._widgets[name] = widget
            setattr(self, name, widget)
            self._form.addRow(self.LABELS[name], widget)
        for name in self.INT_FIELDS:
            widget = QSpinBox()
            widget.setRange(2, 1_000_000)
            widget.setValue(getattr(self._settings, name))
            widget.valueChanged.connect(self._changed)
            widget.setObjectName(name)
            self._widgets[name] = widget
            setattr(self, name, widget)
            self._form.addRow(self.LABELS[name], widget)
        self._sync_visibility()

    def _mode_changed(self, *_args):
        self._sync_visibility()
        self._changed()

    def _sync_visibility(self):
        kind = self.solver_kind.currentData()
        forward = kind == "forward-driver"
        self.forward_note.setVisible(forward)
        for name, widget in self._widgets.items():
            visible = name.startswith("forward_") if forward else name in self.POINT_MASS_FIELDS
            widget.setVisible(visible)
            row = self._form.getWidgetPosition(widget)[0]
            label = self._form.itemAt(row, QFormLayout.ItemRole.LabelRole)
            if label and label.widget():
                label.widget().setVisible(visible)

    def settings(self):
        return self._settings

    def set_settings(self, settings: RunSettings):
        self._settings = settings
        widgets = (self.solver_kind, *self._widgets.values())
        previous = tuple(widget.blockSignals(True) for widget in widgets)
        try:
            self.solver_kind.setCurrentIndex(self.solver_kind.findData(settings.solver_kind))
            for name, widget in self._widgets.items():
                widget.setValue(getattr(settings, name))
            self._sync_visibility()
        finally:
            for widget, was_blocked in zip(widgets, previous):
                widget.blockSignals(was_blocked)
        self.settingsChanged.emit(settings)

    def set_model(self, name: str):
        self._model = name

    def _changed(self, *_args):
        values = {name: widget.value() for name, widget in self._widgets.items()}
        values["solver_kind"] = self.solver_kind.currentData()
        self._settings = RunSettings(**values)
        self._sync_visibility()
        self.settingsChanged.emit(self._settings)
