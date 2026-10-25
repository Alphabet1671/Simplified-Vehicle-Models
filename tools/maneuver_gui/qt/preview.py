"""Simple requested/applied maneuver state preview."""
from copy import deepcopy

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPainter
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel


class PreviewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.requested = None
        self.applied = None
        self.requested_label = QLabel("Requested: —", self)
        self.applied_label = QLabel("Applied: —", self)
        layout = QVBoxLayout(self)
        layout.addWidget(self.requested_label)
        layout.addWidget(self.applied_label)

    def set_state(self, requested, applied=None):
        self.requested = deepcopy(requested)
        self.applied = deepcopy(applied)
        self.requested_label.setText(f"Requested: {self.requested}")
        self.applied_label.setText(f"Applied: {self.applied if self.applied is not None else '—'}")
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setPen(Qt.GlobalColor.darkBlue)
        painter.drawText(self.rect().adjusted(4, 4, -4, -4), Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignRight,
                         "requested / applied")
