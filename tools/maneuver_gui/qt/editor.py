"""Small Qt adapter around the pure ManeuverDraft model."""
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QLineEdit, QFormLayout
from copy import deepcopy
from simplified_vehicle_models.controls.maneuvers.types import ManeuverDefinition

from tools.sim_gui.qt.maneuver_tab import ManeuverDraft


class ManeuverEditor(QWidget):
    """Expose validation and draft replacement without duplicating schema logic."""
    def __init__(self, payload=None, parent=None):
        super().__init__(parent)
        self.draft = ManeuverDraft.from_dict(payload or {})
        self.validation_label = QLabel(self)
        layout = QVBoxLayout(self)
        layout.addWidget(self.validation_label)
        self.fields = {}
        self._build_fields()
        self.validate()

    def _build_fields(self):
        form = QFormLayout()
        for path, unit in [('steering.amplitude_rad','rad'), ('steering.amplitude_mps2','m/s²'), ('initial.speed_mps','m/s'), ('longitudinal.accelerator','fraction')]:
            edit = QLineEdit(self); self.fields[path] = edit
            form.addRow(path, edit)
        self.layout().addLayout(form)

    def definition(self):
        definition, errors = self.validate()
        if definition is None:
            message = '; '.join(errors.values()) or 'Invalid maneuver definition'
            self.validation_label.setText(message)
            raise ValueError(message)
        return definition

    def load_definition(self, definition):
        """Replace editor state with a definition or schema-shaped mapping."""
        data = definition.to_dict() if isinstance(definition, ManeuverDefinition) else deepcopy(definition)
        return self.set_draft(data)

    def set_draft(self, payload):
        self.draft = ManeuverDraft.from_dict(payload)
        return self.validate()

    def validate(self):
        definition, errors = self.draft.validate()
        self.validation_label.setText("; ".join(errors.values()))
        enabled = self.draft.enabled_fields()
        for path, edit in self.fields.items(): edit.setEnabled(enabled.get(path, True))
        return definition, errors

    def to_dict(self):
        return self.draft.to_dict()
