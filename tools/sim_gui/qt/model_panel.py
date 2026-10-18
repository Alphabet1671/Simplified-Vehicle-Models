"""Qt widget for selecting a vehicle model and its components."""
from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import QObject, QEvent, Qt, pyqtSignal
from PyQt6.QtGui import QFontMetrics
from PyQt6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QTableWidget,
    QTableWidgetItem,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from tools.run import run_iem24_forward_driver as runner
from tools.sim_gui.catalogue import candidates, read_entry, scan_catalogue, tracks
from tools.sim_gui.vehicle import VehicleConfiguration

_warn = QMessageBox.warning


class _FocusFilter(QObject):
    """Emit a callback when a slot combo receives keyboard focus."""

    def __init__(self, panel: "ModelPanel", key: str) -> None:
        super().__init__(panel)
        self._panel = panel
        self._key = key

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:
        if event.type() == QEvent.Type.FocusIn:
            self._panel._show_params(self._key)
        return super().eventFilter(watched, event)


class ModelPanel(QWidget):
    """Display and edit the selected vehicle configuration."""

    configurationChanged = pyqtSignal(object)
    slotSelected = pyqtSignal(str)

    def __init__(self, configuration: VehicleConfiguration | None = None, parent: QWidget | None = None) -> None:
        """Create a model panel, optionally populated with ``configuration``."""
        super().__init__(parent)
        self._cfg: VehicleConfiguration | None = None
        self._baseline: VehicleConfiguration | None = None
        self._slot_widgets: dict[str, tuple[QComboBox, QToolButton, QToolButton]] = {}
        self._catalogue = scan_catalogue()
        self._building = False

        self._source = QLabel()
        self._source.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self._source.setToolTip("")
        self._error = QLabel()
        self._error.setStyleSheet("color: red")
        self._form = QFormLayout()
        self._table = QTableWidget(0, 3)
        self._table.setHorizontalHeaderLabels(("Name", "Value", "Unit"))
        self._table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)

        layout = QVBoxLayout(self)
        layout.addWidget(self._source)
        layout.addWidget(self._error)
        layout.addLayout(self._form)
        layout.addWidget(QLabel("Parameters"))
        layout.addWidget(self._table)
        if configuration is not None:
            self.set_configuration(configuration)

    def resizeEvent(self, event) -> None:
        """Elide the source path in the middle when the widget is resized."""
        super().resizeEvent(event)
        self._update_source_text()

    def set_configuration(self, cfg: VehicleConfiguration, baseline: VehicleConfiguration | None = None) -> None:
        """Set the current configuration and optional reset baseline."""
        self._cfg = cfg
        self._baseline = baseline if baseline is not None else cfg
        self._rebuild()

    def configuration(self) -> VehicleConfiguration | None:
        """Return the current configuration."""
        return self._cfg

    def track(self) -> Path | None:
        """Return the selected track path, or ``None`` when none is selected."""
        if not hasattr(self, "_track"):
            return None
        value = self._track.currentData()
        return Path(value) if value else None

    def baseline(self) -> VehicleConfiguration | None:
        """Return the initial configuration used for reset comparisons."""
        return self._baseline

    def _update_source_text(self) -> None:
        if self._cfg is None:
            return
        path = str(self._cfg.source)
        width = max(0, self._source.width())
        text = QFontMetrics(self._source.font()).elidedText(
            path, Qt.TextElideMode.ElideMiddle, width
        )
        self._source.setText(text)
        self._source.setToolTip(path)

    def _clear_form(self) -> None:
        """Remove form rows and schedule their containers for deletion."""
        while self._form.rowCount():
            row = self._form.takeRow(0)
            for item in (row.fieldItem, row.labelItem):
                if item is not None and item.widget() is not None:
                    item.widget().deleteLater()
        self._slot_widgets.clear()

    def _rebuild(self) -> None:
        if self._cfg is None or self._baseline is None:
            return
        self._building = True
        self._clear_form()
        self._source.setText(str(self._cfg.source))
        self._update_source_text()

        model = QComboBox()
        model_names = list(runner.MODELS)
        if self._cfg.model not in model_names:
            model_names.append(self._cfg.model)
        model.addItems(model_names)
        model.setCurrentText(self._cfg.model)
        model.currentTextChanged.connect(self._change_model)
        self._form.addRow("Model", model)
        self._model = model

        self._track = QComboBox()
        track_entries = tracks(self._catalogue)
        for entry in track_entries:
            self._track.addItem(entry.path.stem, str(entry.path))
        if track_entries:
            default_track = getattr(runner, "DEFAULT_TRACK", None)
            index = self._track.findData(str(default_track)) if default_track else -1
            self._track.setCurrentIndex(index if index >= 0 else 0)
        self._form.addRow("Track", self._track)

        for slot in self._cfg.slots():
            combo = QComboBox()
            entry = read_entry(self._cfg.choices[slot.key])
            for candidate in candidates(entry, self._catalogue):
                combo.addItem(candidate.path.stem, str(candidate.path))
            combo.setCurrentIndex(max(0, combo.findData(str(self._cfg.choices[slot.key]))))
            combo.currentIndexChanged.connect(
                lambda index, key=slot.key, widget=combo: self._change_slot(key, widget, index)
            )
            combo.installEventFilter(_FocusFilter(self, slot.key))

            browse = QToolButton()
            browse.setText("Browse…")
            browse.clicked.connect(
                lambda *_, key=slot.key, widget=combo: self._browse(key, widget)
            )
            reset = QToolButton()
            reset.setText("Reset")
            changed = self._cfg.choices[slot.key] != self._baseline.choices.get(slot.key)
            reset.setEnabled(changed)
            reset.clicked.connect(lambda *_, key=slot.key: self._reset(key))
            font = combo.font()
            font.setBold(changed)
            combo.setFont(font)

            row = QWidget()
            row_layout = QHBoxLayout(row)
            row_layout.setContentsMargins((slot.key.count("/")) * 16, 0, 0, 0)
            row_layout.addWidget(combo)
            row_layout.addWidget(browse)
            row_layout.addWidget(reset)
            self._form.addRow(slot.label, row)
            self._slot_widgets[slot.key] = (combo, browse, reset)

        self._building = False
        if self._cfg.slots():
            self._show_params(self._cfg.slots()[0].key)
        self._error.setText(
            "" if self._cfg.runnable else "no forward runner for point-mass; select planar-7dof or 14dof-simple"
        )

    def _change_model(self, name: str) -> None:
        if self._building or self._cfg is None:
            return
        try:
            cfg = self._cfg.with_model(name)
        except ValueError:
            return
        self._cfg = cfg
        self._rebuild()
        if cfg.runnable:
            self.configurationChanged.emit(cfg)

    def _change_slot(self, key: str, combo: QComboBox, index: int) -> None:
        if self._building or self._cfg is None:
            return
        old = self._cfg.choices[key]
        try:
            self._cfg = self._cfg.with_choice(key, Path(combo.itemData(index)))
        except ValueError as error:
            _warn(self, "Incompatible component", str(error))
            combo.blockSignals(True)
            combo.setCurrentIndex(combo.findData(str(old)))
            combo.blockSignals(False)
            return
        if any(slot.key.startswith(key + "/") for slot in self._cfg.slots()):
            self._rebuild()
        else:
            changed = self._cfg.choices[key] != self._baseline.choices.get(key)
            font = combo.font()
            font.setBold(changed)
            combo.setFont(font)
            self._slot_widgets[key][2].setEnabled(changed)
        if self._cfg.runnable:
            self.configurationChanged.emit(self._cfg)

    def _show_params(self, key: str) -> None:
        if self._cfg is None:
            return
        try:
            rows = self._cfg.parameters(key)
        except ValueError:
            rows = []
        self._table.setRowCount(len(rows))
        for row_index, row in enumerate(rows):
            for column, value in enumerate(row):
                self._table.setItem(row_index, column, QTableWidgetItem(str(value)))
        self.slotSelected.emit(key)

    def _browse(self, key: str, combo: QComboBox) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Select component", filter="XML (*.xml)")
        if path:
            combo.addItem(Path(path).stem, path)
            combo.setCurrentIndex(combo.count() - 1)

    def _reset(self, key: str) -> None:
        combo = self._slot_widgets[key][0]
        index = combo.findData(str(self._baseline.choices[key]))
        if index < 0:
            return
        combo.blockSignals(True)
        combo.setCurrentIndex(index)
        combo.blockSignals(False)
        self._change_slot(key, combo, index)
