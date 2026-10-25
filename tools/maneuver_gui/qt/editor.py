"""Small Qt adapter around the pure ManeuverDraft model."""
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel

from tools.sim_gui.qt.maneuver_tab import ManeuverDraft


class ManeuverEditor(QWidget):
    """Expose validation and draft replacement without duplicating schema logic."""
    def __init__(self, payload=None, parent=None):
        super().__init__(parent)
        self.draft = ManeuverDraft.from_dict(payload or {})
        self.validation_label = QLabel(self)
        layout = QVBoxLayout(self)
        layout.addWidget(self.validation_label)
        self.validate()

    def set_draft(self, payload):
        self.draft = ManeuverDraft.from_dict(payload)
        return self.validate()

    def validate(self):
        definition, errors = self.draft.validate()
        self.validation_label.setText("; ".join(errors.values()))
        return definition, errors

    def to_dict(self):
        return self.draft.to_dict()
