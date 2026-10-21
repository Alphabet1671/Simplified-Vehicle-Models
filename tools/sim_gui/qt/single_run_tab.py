"""Qt tab for one standalone simulation run."""
from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from simplified_vehicle_models.tracks.profile import load_track
from tools.run import run_iem24_forward_driver as runner
from tools.sim_gui import REPOSITORY_ROOT
from tools.sim_gui.catalogue import scan_catalogue, tracks
from tools.sim_gui.vehicle import VehicleConfiguration

_warn = QMessageBox.warning

MAXIMUM_LOG_LINES = 2000


class SingleRunTab(QWidget):
    """Choose one vehicle model, one track, and a folder for a single run."""

    goRequested = pyqtSignal(object)
    stopRequested = pyqtSignal()
    modelSelected = pyqtSignal(object)
    trackSelected = pyqtSignal(object)

    def __init__(self, parent: QWidget | None = None) -> None:
        """Create the single-run tab with a default track and export folder."""
        super().__init__(parent)
        self.model_combo = QComboBox()
        self.model_browse = QPushButton("Browse…")
        self.track_combo = QComboBox()
        self.track_browse = QPushButton("Browse…")
        self.export_edit = QLineEdit(str(REPOSITORY_ROOT / "runs"))
        self.export_browse = QPushButton("Browse…")
        self.go_button = QPushButton("Go")
        self.stop_button = QPushButton("Stop")
        self.progress_bar = QProgressBar()
        self.status_label = QLabel()
        self.result_label = QLabel()
        self.log_view = QPlainTextEdit()

        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.result_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.result_label.setWordWrap(True)
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(MAXIMUM_LOG_LINES)
        self.stop_button.setEnabled(False)

        for path in sorted((REPOSITORY_ROOT / "models/formulations").glob("*.xml")):
            self.model_combo.addItem(path.stem, str(path.resolve()))
        catalogue = scan_catalogue()
        for entry in tracks(catalogue):
            self.track_combo.addItem(entry.path.stem, str(entry.path))
        default = (REPOSITORY_ROOT / runner.DEFAULT_TRACK).resolve()
        default_index = self._index_of(self.track_combo, default)
        if default_index < 0 and self.track_combo.count():
            default_index = 0
        if default_index >= 0:
            self.track_combo.blockSignals(True)
            self.track_combo.setCurrentIndex(default_index)
            self.track_combo.blockSignals(False)

        self.model_combo.currentIndexChanged.connect(self._model_changed)
        self.track_combo.currentIndexChanged.connect(self._track_changed)
        self.model_browse.clicked.connect(self._browse_model)
        self.track_browse.clicked.connect(self._browse_track)
        self.export_browse.clicked.connect(self._browse_export)
        self.go_button.clicked.connect(self._go_clicked)
        self.stop_button.clicked.connect(self.stopRequested.emit)

        model_row = QHBoxLayout()
        model_row.addWidget(self.model_combo, 1)
        model_row.addWidget(self.model_browse)
        track_row = QHBoxLayout()
        track_row.addWidget(self.track_combo, 1)
        track_row.addWidget(self.track_browse)
        export_row = QHBoxLayout()
        export_row.addWidget(self.export_edit, 1)
        export_row.addWidget(self.export_browse)
        buttons = QHBoxLayout()
        buttons.addWidget(self.go_button)
        buttons.addWidget(self.stop_button)
        buttons.addWidget(self.progress_bar, 1)

        form = QFormLayout()
        form.addRow("Vehicle model", model_row)
        form.addRow("Track", track_row)
        form.addRow("Export folder", export_row)
        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addLayout(buttons)
        layout.addWidget(self.status_label)
        layout.addWidget(self.result_label)
        layout.addWidget(self.log_view, 1)

    def export_folder(self) -> Path:
        """Return the export folder that the line edit shows."""
        text = self.export_edit.text().strip()
        return Path(text) if text else REPOSITORY_ROOT / "runs"

    def set_export_folder(self, path: Path) -> None:
        """Show ``path`` in the export folder line edit."""
        self.export_edit.setText(str(path))

    def model_path(self) -> Path | None:
        """Return the selected vehicle model path, or ``None``."""
        value = self.model_combo.currentData()
        return Path(value).resolve() if value else None

    def set_model(self, path: Path | None) -> None:
        """Select a vehicle model without emitting :attr:`modelSelected`."""
        if path is None:
            self.model_combo.blockSignals(True)
            self.model_combo.setCurrentIndex(-1)
            self.model_combo.blockSignals(False)
            return
        target = Path(path).resolve()
        self._select(self.model_combo, target, target.stem)

    def track_path(self) -> Path | None:
        """Return the selected track path, or ``None``."""
        value = self.track_combo.currentData()
        return Path(value).resolve() if value else None

    def set_track(self, path: Path | None) -> None:
        """Select a track without emitting :attr:`trackSelected`."""
        if path is None:
            self.track_combo.blockSignals(True)
            self.track_combo.setCurrentIndex(-1)
            self.track_combo.blockSignals(False)
            return
        target = Path(path).resolve()
        self._select(self.track_combo, target, target.stem)

    def set_running(self, running: bool) -> None:
        """Enable Stop and disable Go while a run is active."""
        self.go_button.setEnabled(not running)
        self.stop_button.setEnabled(running)

    def set_progress(self, value: float) -> None:
        """Show a progress fraction from 0.0 to 1.0 as a percentage."""
        self.progress_bar.setValue(max(0, min(100, int(round(value * 100)))))

    def set_status(self, text: str) -> None:
        """Show one line of run state."""
        self.status_label.setText(text)

    def set_result(self, text: str) -> None:
        """Show the record path, lap time, or error text."""
        self.result_label.setText(text)

    def set_log(self, text: str) -> None:
        """Show at most the last 2000 lines of run output."""
        lines = text.splitlines()[-MAXIMUM_LOG_LINES:]
        self.log_view.setPlainText("\n".join(lines))

    def clear_run(self) -> None:
        """Reset progress, status, and result for a new run."""
        self.progress_bar.setValue(0)
        self.status_label.setText("")
        self.result_label.setText("")

    def _model_changed(self, index: int) -> None:
        value = self.model_combo.itemData(index)
        if value:
            self.modelSelected.emit(Path(value).resolve())

    def _track_changed(self, index: int) -> None:
        value = self.track_combo.itemData(index)
        if value:
            self.trackSelected.emit(Path(value).resolve())

    def _browse_model(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Select vehicle model", filter="XML (*.xml)")
        if not path:
            return
        target = Path(path).resolve()
        try:
            VehicleConfiguration.open(target)
        except ValueError as error:
            _warn(self, "Vehicle model", str(error))
            return
        self._select(self.model_combo, target, target.stem)
        self.modelSelected.emit(target)

    def _browse_track(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Select track", filter="XML (*.xml)")
        if not path:
            return
        target = Path(path).resolve()
        try:
            load_track(target)
        except ValueError as error:
            _warn(self, "Track model", str(error))
            return
        self._select(self.track_combo, target, target.stem)
        self.trackSelected.emit(target)

    def _browse_export(self) -> None:
        folder = QFileDialog.getExistingDirectory(self, "Select export folder", str(self.export_folder()))
        if folder:
            self.set_export_folder(Path(folder))

    def _go_clicked(self) -> None:
        self.goRequested.emit(self.export_folder())

    def _select(self, combo: QComboBox, target: Path, label: str) -> None:
        combo.blockSignals(True)
        try:
            index = self._index_of(combo, target)
            if index < 0:
                combo.addItem(label, str(target))
                index = combo.count() - 1
            combo.setCurrentIndex(index)
        finally:
            combo.blockSignals(False)

    @staticmethod
    def _index_of(combo: QComboBox, target: Path) -> int:
        for index in range(combo.count()):
            value = combo.itemData(index)
            if value and Path(value).resolve() == target:
                return index
        return -1
