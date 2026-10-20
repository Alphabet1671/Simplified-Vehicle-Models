"""The parameter table of one slot."""
from __future__ import annotations

from PyQt6.QtCore import QAbstractTableModel, QModelIndex, Qt, pyqtSignal
from PyQt6.QtGui import QColor

from tools.vehicle_gui.model import ParameterRow

HEADERS = ("Name", "Value", "Unit", "File value")


class ParameterTableModel(QAbstractTableModel):
    """Rows of one slot. ``edited(name, value)`` fires for a valid number; bad text is refused."""

    edited = pyqtSignal(str, float)
    refused = pyqtSignal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._rows: list[ParameterRow] = []
        self.locked_reason: str | None = None

    def set_rows(self, rows: list[ParameterRow], locked_reason: str | None = None) -> None:
        self.beginResetModel()
        self._rows, self.locked_reason = list(rows), locked_reason
        self.endResetModel()

    def rowCount(self, parent=QModelIndex()) -> int:
        return 0 if parent.isValid() else len(self._rows)

    def columnCount(self, parent=QModelIndex()) -> int:
        return 0 if parent.isValid() else len(HEADERS)

    def headerData(self, section, orientation, role=Qt.ItemDataRole.DisplayRole):
        if (
            role == Qt.ItemDataRole.DisplayRole
            and orientation == Qt.Orientation.Horizontal
            and 0 <= section < len(HEADERS)
        ):
            return HEADERS[section]
        return None

    def flags(self, index):
        flags = Qt.ItemFlag.ItemIsSelectable | Qt.ItemFlag.ItemIsEnabled
        if not index.isValid() or not 0 <= index.row() < len(self._rows):
            return flags
        if index.column() == 1 and self.locked_reason is None and self._rows[index.row()].numeric:
            flags |= Qt.ItemFlag.ItemIsEditable
        return flags

    def data(self, index, role=Qt.ItemDataRole.DisplayRole):
        if not index.isValid() or not 0 <= index.row() < len(self._rows) or not 0 <= index.column() < len(HEADERS):
            return None
        row = self._rows[index.row()]
        if role in (Qt.ItemDataRole.DisplayRole, Qt.ItemDataRole.EditRole):
            return (row.name, row.value_text, row.unit, row.original)[index.column()]
        if role == Qt.ItemDataRole.BackgroundRole and row.edited is not None:
            return QColor(255, 244, 200)
        if role == Qt.ItemDataRole.ToolTipRole and self.locked_reason:
            return self.locked_reason
        return None

    def setData(self, index, value, role=Qt.ItemDataRole.EditRole) -> bool:
        if (
            role != Qt.ItemDataRole.EditRole
            or not index.isValid()
            or not 0 <= index.row() < len(self._rows)
            or index.column() != 1
        ):
            return False
        row = self._rows[index.row()]
        if self.locked_reason is not None:
            self.refused.emit(f"{row.name}: the table is read-only ({self.locked_reason})")
            return False
        if not row.numeric:
            self.refused.emit(f"{row.name}: is not a number and cannot be edited here")
            return False
        try:
            number = float(str(value).strip())
        except ValueError:
            self.refused.emit(f"{row.name}: {value!r} is not a number")
            return False
        if number != number or number in (float("inf"), float("-inf")):
            self.refused.emit(f"{row.name}: the value must be finite")
            return False
        self.edited.emit(row.name, number)
        return True
