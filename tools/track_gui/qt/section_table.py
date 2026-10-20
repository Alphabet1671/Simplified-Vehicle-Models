"""The editable section table."""
from __future__ import annotations

import math
from dataclasses import fields, replace

from PyQt6.QtCore import QAbstractTableModel, QModelIndex, Qt, pyqtSignal

from tools.track_gui.design import KINDS, SectionSpec

COLUMNS = [("kind", "Type"), ("name", "Name"), ("length", "Length (m)"), ("radius", "Radius (m)"),
           ("angle_deg", "Angle (deg)"), ("curvature", "Curvature (1/m)"),
           ("end_curvature", "End curvature (1/m)"), ("rate", "Rate (1/m²)")]
USED = {"straight": {"length"}, "arc": {"length", "curvature"}, "arc-angle": {"radius", "angle_deg"},
        "clothoid": {"curvature", "end_curvature", "rate"}}


class SectionTableModel(QAbstractTableModel):
    changed = pyqtSignal()

    def __init__(self, sections=(), parent=None) -> None:
        super().__init__(parent)
        self._rows: list[SectionSpec] = list(sections)

    def sections(self) -> tuple[SectionSpec, ...]:
        return tuple(self._rows)

    def set_sections(self, sections) -> None:
        self.beginResetModel()
        self._rows = list(sections)
        self.endResetModel()
        self.changed.emit()

    def rowCount(self, parent=QModelIndex()) -> int:
        return 0 if parent.isValid() else len(self._rows)

    def columnCount(self, parent=QModelIndex()) -> int:
        return 0 if parent.isValid() else len(COLUMNS)

    def headerData(self, section, orientation, role=Qt.ItemDataRole.DisplayRole):
        if role == Qt.ItemDataRole.DisplayRole:
            return COLUMNS[section][1] if orientation == Qt.Orientation.Horizontal else str(section + 1)
        return None

    def flags(self, index):
        if not index.isValid():
            return Qt.ItemFlag.NoItemFlags
        flags = Qt.ItemFlag.ItemIsSelectable | Qt.ItemFlag.ItemIsEnabled
        key = COLUMNS[index.column()][0]
        row = self._rows[index.row()]
        if key in ("kind", "name") or key in USED[row.kind]:
            flags |= Qt.ItemFlag.ItemIsEditable
        return flags

    def data(self, index, role=Qt.ItemDataRole.DisplayRole):
        if not index.isValid():
            return None
        key = COLUMNS[index.column()][0]
        row = self._rows[index.row()]
        value = getattr(row, key)
        if role in (Qt.ItemDataRole.DisplayRole, Qt.ItemDataRole.EditRole):
            return value if isinstance(value, str) else f"{value:.10g}"
        if role == Qt.ItemDataRole.ForegroundRole and key not in ("kind", "name") and key not in USED[row.kind]:
            from PyQt6.QtGui import QColor
            return QColor(170, 170, 170)
        return None

    def setData(self, index, value, role=Qt.ItemDataRole.EditRole) -> bool:
        if not index.isValid() or role != Qt.ItemDataRole.EditRole or not (0 <= index.row() < len(self._rows)):
            return False
        key = COLUMNS[index.column()][0]
        row = self._rows[index.row()]
        if key not in ("kind", "name") and key not in USED[row.kind]:
            return False
        try:
            if key == "kind":
                if str(value) not in KINDS:
                    return False
                new = replace(row, kind=str(value))
            elif key == "name":
                new = replace(row, name=str(value))
            else:
                number = float(value)
                if not math.isfinite(number):
                    return False
                new = replace(row, **{key: number})
        except (TypeError, ValueError):
            return False
        self._rows[index.row()] = new
        self.dataChanged.emit(self.index(index.row(), 0), self.index(index.row(), len(COLUMNS) - 1))
        self.changed.emit()
        return True

    def add(self, kind: str = "straight", after: int | None = None) -> int:
        if kind not in KINDS:
            raise ValueError(f"unknown section kind {kind!r}")
        position = len(self._rows) if after is None else max(0, min(after + 1, len(self._rows)))
        self.beginInsertRows(QModelIndex(), position, position)
        self._rows.insert(position, SectionSpec(kind=kind))
        self.endInsertRows()
        self.changed.emit()
        return position

    def remove(self, row: int) -> None:
        if 0 <= row < len(self._rows):
            self.beginRemoveRows(QModelIndex(), row, row)
            del self._rows[row]
            self.endRemoveRows()
            self.changed.emit()

    def move(self, row: int, offset: int) -> int:
        target = row + offset
        if not (0 <= row < len(self._rows) and 0 <= target < len(self._rows)):
            return row
        rows = list(self._rows)
        rows[row], rows[target] = rows[target], rows[row]
        self.set_sections(rows)
        return target


assert [name for name, _ in COLUMNS] == [f.name for f in fields(SectionSpec)]
