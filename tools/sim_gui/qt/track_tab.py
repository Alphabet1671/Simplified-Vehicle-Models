"""Qt tab for selecting a track model and previewing its centerline."""
from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import QPointF, QSize, Qt, pyqtSignal
from PyQt6.QtGui import QPainter, QPalette, QPen, QPolygonF
from PyQt6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.types import TrackProfile
from tools.run import run_iem24_forward_driver as runner
from tools.sim_gui import REPOSITORY_ROOT
from tools.sim_gui.catalogue import scan_catalogue, tracks

_warn = QMessageBox.warning

NO_CENTERLINE_MESSAGE = "No signed centerline for this track model."
_MARGIN = 12.0
_START_RADIUS = 4.0


class TrackPreview(QWidget):
    """Paint the centerline of one selected track profile."""

    def __init__(self, parent: QWidget | None = None) -> None:
        """Create an empty preview that shows the missing-centerline message."""
        super().__init__(parent)
        self._points: tuple[tuple[float, float], ...] = ()
        self._message = NO_CENTERLINE_MESSAGE

    def points(self) -> tuple[tuple[float, float], ...]:
        """Return the drawn centerline as (east, north) pairs."""
        return self._points

    def message(self) -> str:
        """Return the text shown when there is no centerline, or an empty string."""
        return self._message

    def set_track(self, profile: TrackProfile | None) -> None:
        """Replace the previewed profile and repaint the widget."""
        coordinates = None if profile is None else profile.path_coordinates
        if coordinates is None:
            self._points = ()
            self._message = NO_CENTERLINE_MESSAGE
        else:
            self._points = tuple(
                (float(east), float(north))
                for north, east in zip(coordinates.x_north, coordinates.y_east)
            )
            self._message = ""
        self.update()

    def minimumSizeHint(self) -> QSize:
        """Return the smallest useful preview size."""
        return QSize(240, 180)

    def paintEvent(self, event) -> None:
        """Draw the centerline, the start point, or the empty message."""
        painter = QPainter(self)
        palette = self.palette()
        base = palette.color(QPalette.ColorRole.Base)
        text = palette.color(QPalette.ColorRole.Text)
        painter.fillRect(self.rect(), base)
        painter.setPen(QPen(text, 2))
        if not self._points:
            painter.drawText(
                self.rect(), Qt.AlignmentFlag.AlignCenter, self._message
            )
            return
        east = [point[0] for point in self._points]
        north = [point[1] for point in self._points]
        minimum_east = min(east)
        maximum_north = max(north)
        span_east = max(east) - minimum_east
        span_north = maximum_north - min(north)
        span = max(span_east, span_north)
        available = min(self.width(), self.height()) - 2.0 * _MARGIN
        scale = available / span if span > 0.0 and available > 0.0 else 1.0
        offset_x = (self.width() - span_east * scale) / 2.0
        offset_y = (self.height() - span_north * scale) / 2.0
        polygon = QPolygonF(
            [
                QPointF(
                    offset_x + (east_value - minimum_east) * scale,
                    offset_y + (maximum_north - north_value) * scale,
                )
                for east_value, north_value in self._points
            ]
        )
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawPolyline(polygon)
        start = self._points[0]
        painter.setBrush(text)
        painter.drawEllipse(
            QPointF(
                offset_x + (start[0] - minimum_east) * scale,
                offset_y + (maximum_north - start[1]) * scale,
            ),
            _START_RADIUS,
            _START_RADIUS,
        )


class TrackTab(QWidget):
    """Select a track model and preview its signed centerline."""

    trackChanged = pyqtSignal(object)

    def __init__(self, parent: QWidget | None = None) -> None:
        """Create the track tab and preselect the default track."""
        super().__init__(parent)
        self._catalogue = scan_catalogue()
        self._previous_index = -1
        self._combo = QComboBox()
        self._browse = QPushButton("Browse…")
        row = QHBoxLayout()
        row.addWidget(self._combo, 1)
        row.addWidget(self._browse)
        form = QFormLayout()
        form.addRow("Track model", row)
        self._info = QLabel()
        self.preview = TrackPreview()

        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(self._info)
        layout.addWidget(self.preview, 1)

        for entry in tracks(self._catalogue):
            self._combo.addItem(entry.path.stem, str(entry.path))
        default = (REPOSITORY_ROOT / runner.DEFAULT_TRACK).resolve()
        index = self._index_of(default)
        if index < 0:
            index = 0 if self._combo.count() else -1
        self._combo.setCurrentIndex(index)
        self._previous_index = index
        if index >= 0:
            self._load(index, emit=False)
        self._combo.currentIndexChanged.connect(self._change)
        self._browse.clicked.connect(self._browse_track)

    def track(self) -> Path | None:
        """Return the selected track path, or ``None`` when none is selected."""
        value = self._combo.currentData()
        return Path(value) if value else None

    def set_track(self, path: Path) -> None:
        """Select a track by resolved path, adding it when not catalogued."""
        target = Path(path).resolve()
        current = self.track()
        if current is not None and current.resolve() == target:
            return
        index = self._index_of(target)
        if index < 0:
            self._combo.addItem(target.stem, str(target))
            index = self._combo.count() - 1
        self._combo.setCurrentIndex(index)

    def _index_of(self, target: Path) -> int:
        for index in range(self._combo.count()):
            value = self._combo.itemData(index)
            if value and Path(value).resolve() == target:
                return index
        return -1

    def _change(self, index: int) -> None:
        self._load(index, emit=True)

    def _load(self, index: int, emit: bool) -> None:
        value = self._combo.itemData(index)
        if not value:
            return
        path = Path(value).resolve()
        try:
            profile = load_track(path)
        except ValueError as error:
            _warn(self, "Track model", str(error))
            self._combo.blockSignals(True)
            self._combo.setCurrentIndex(self._previous_index)
            self._combo.blockSignals(False)
            return
        self._previous_index = index
        self.preview.set_track(profile)
        self._info.setText(self._info_text(profile))
        if emit:
            self.trackChanged.emit(path)

    @staticmethod
    def _info_text(profile: TrackProfile) -> str:
        parts = [
            profile.title,
            f"{profile.lap_length:.3f} m",
            "closed" if profile.is_closed else "open",
        ]
        if profile.path_coordinates is None:
            parts.append("no signed centerline")
        else:
            parts.append(f"{len(profile.path_coordinates.distance)} nodes")
        return " · ".join(parts)

    def _browse_track(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Select track", filter="XML (*.xml)"
        )
        if not path:
            return
        target = Path(path).resolve()
        index = self._index_of(target)
        if index < 0:
            self._combo.addItem(target.stem, str(target))
            index = self._combo.count() - 1
        self._combo.setCurrentIndex(index)
