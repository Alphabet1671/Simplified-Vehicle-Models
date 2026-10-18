"""Qt widget for forward-run settings."""
from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QPushButton,
    QWidget,
)

from tools.sim_gui.settings import RunSettings


class SettingsTab(QWidget):
    """Edit and emit immutable :class:`RunSettings` values."""

    settingsChanged = pyqtSignal(object)

    _FLOAT_FIELDS = (
        "duration",
        "timeout_seconds",
        "maximum_speed",
        "pedal_travel_time",
        "preview_seconds",
        "heading_gain",
        "lateral_gain",
        "maximum_step",
        "rtol",
        "spacing",
    )
    _CHOICES = {
        "method": ("Radau", "BDF"),
        "planner": ("conservative", "live"),
        "record_precision": ("float32", "float64"),
    }

    def __init__(self, parent: QWidget | None = None) -> None:
        """Create a settings form with the reference-lap defaults."""
        super().__init__(parent)
        self._settings = RunSettings()
        self._model = "planar-7dof"
        self._fields: dict[str, QDoubleSpinBox] = {}
        form = QFormLayout(self)

        labels = {
            "duration": "Duration (s)",
            "timeout_seconds": "Timeout (s)",
            "maximum_speed": "Maximum speed (m/s)",
            "pedal_travel_time": "Pedal travel time (s)",
            "preview_seconds": "Preview horizon (s)",
            "heading_gain": "Heading gain",
            "lateral_gain": "Lateral gain",
            "maximum_step": "Maximum step (s)",
            "rtol": "Relative tolerance",
            "spacing": "Spacing (m)",
        }
        for name in self._FLOAT_FIELDS:
            widget = QDoubleSpinBox()
            widget.setRange(1e-6 if name in ("rtol", "maximum_step") else 0.001, 1e9)
            widget.setDecimals(6 if name in ("rtol", "maximum_step") else 3)
            widget.setValue(getattr(self._settings, name))
            widget.valueChanged.connect(self._changed)
            self._fields[name] = widget
            setattr(self, name, widget)
            form.addRow(labels[name], widget)

        self.lateral_cap = QCheckBox()
        self.lateral_value = QDoubleSpinBox()
        self.lateral_value.setRange(0.001, 1e9)
        self.lateral_value.setDecimals(3)
        self.lateral_cap.toggled.connect(self._lateral_toggled)
        self.lateral_value.valueChanged.connect(self._changed)
        form.addRow("Lateral cap", self.lateral_cap)
        form.addRow("Lateral acceleration (m/s²)", self.lateral_value)
        self.lateral_value.setEnabled(False)

        for name, choices in self._CHOICES.items():
            widget = QComboBox()
            widget.addItems(choices)
            widget.setCurrentText(getattr(self._settings, name))
            widget.currentTextChanged.connect(self._changed)
            setattr(self, name, widget)
            form.addRow({"method": "Method", "planner": "Planner", "record_precision": "Record precision"}[name], widget)

        self.quasi_static_attitude = QCheckBox()
        self.quasi_static_attitude.toggled.connect(self._changed)
        form.addRow("Quasi-static attitude", self.quasi_static_attitude)

        preset = QPushButton("Preset: Reference lap")
        preset.clicked.connect(self._preset)
        form.addRow(preset)
        self.preset_button = preset

    def settings(self) -> RunSettings:
        """Return the current settings object."""
        return self._settings

    def set_settings(self, settings: RunSettings) -> None:
        """Replace all values and emit one consolidated change notification."""
        self._block_widgets(True)
        try:
            self._settings = settings
            for name, widget in self._fields.items():
                widget.setValue(getattr(settings, name))
            for name in self._CHOICES:
                getattr(self, name).setCurrentText(getattr(settings, name))
            self.quasi_static_attitude.setChecked(settings.quasi_static_attitude)
            has_cap = settings.lateral_acceleration is not None
            self.lateral_cap.setChecked(has_cap)
            if has_cap:
                self.lateral_value.setValue(settings.lateral_acceleration)
            self.lateral_value.setEnabled(has_cap)
        finally:
            self._block_widgets(False)
        self.settingsChanged.emit(self._settings)

    def set_model(self, name: str) -> None:
        """Apply model-specific timeout limits and quasi-static availability."""
        self._model = name
        maximum = 300.0 if name == "planar-7dof" else 1e6
        defaults = RunSettings.for_model(name)
        self._block_widgets(True)
        try:
            self.timeout_seconds.setMaximum(maximum)
            self.timeout_seconds.setValue(defaults.timeout_seconds)
            enabled = name == "planar-7dof"
            self.quasi_static_attitude.setEnabled(enabled)
            if not enabled:
                self.quasi_static_attitude.setChecked(False)
        finally:
            self._block_widgets(False)
        self._sync_settings(emit=True)

    def _block_widgets(self, blocked: bool) -> None:
        for widget in (*self._fields.values(), self.lateral_cap, self.lateral_value,
                       self.method, self.planner, self.record_precision,
                       self.quasi_static_attitude):
            widget.blockSignals(blocked)

    def _lateral_toggled(self, checked: bool) -> None:
        self.lateral_value.setEnabled(checked)
        self._changed()

    def _sync_settings(self, emit: bool = True) -> None:
        values = {name: widget.value() for name, widget in self._fields.items()}
        values.update(
            method=self.method.currentText(),
            planner=self.planner.currentText(),
            record_precision=self.record_precision.currentText(),
            quasi_static_attitude=self.quasi_static_attitude.isChecked(),
            lateral_acceleration=(self.lateral_value.value() if self.lateral_cap.isChecked() else None),
        )
        self._settings = RunSettings(**values)
        if emit:
            self.settingsChanged.emit(self._settings)

    def _changed(self, *_args) -> None:
        """Synchronize the dataclass after an individual control changes."""
        self._sync_settings()

    def _preset(self) -> None:
        """Load the reference-lap preset and emit it once."""
        self.set_settings(RunSettings.for_model(self._model))
