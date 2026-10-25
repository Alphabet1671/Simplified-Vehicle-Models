"""Minimal CSV source and column-mapping dialog."""
from pathlib import Path
from simplified_vehicle_models.controls.maneuvers.csv_source import CsvMapping, load_csv_source

from PyQt6.QtWidgets import QDialog, QFormLayout, QLineEdit, QDialogButtonBox, QLabel, QVBoxLayout


class CsvDialog(QDialog):
    REQUIRED = ("time", "swa", "accelerator", "brake")

    def __init__(self, source="", mapping=None, parent=None):
        super().__init__(parent)
        self.source_edit = QLineEdit(str(source), self)
        self.mapping_edits = {name: QLineEdit((mapping or {}).get(name, ""), self)
                              for name in self.REQUIRED}
        self.error_label = QLabel(self)
        form = QFormLayout()
        form.addRow("CSV source", self.source_edit)
        for name, edit in self.mapping_edits.items():
            form.addRow(name, edit)
        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(self.error_label)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel, self)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def values(self):
        return {"source": self.source_edit.text().strip(),
                "mapping": {key: edit.text().strip() for key, edit in self.mapping_edits.items()}}

    def validate(self):
        data = self.values()
        errors = {}
        path = Path(data["source"]) if data["source"] else None
        if path is None or not path.is_file():
            errors["csv.source"] = "CSV source must be an existing file"
        missing = [key for key, value in data["mapping"].items() if not value]
        if missing:
            errors["csv.mapping"] = "Column mapping required for: " + ", ".join(missing)
        self.error_label.setText("; ".join(errors.values()))
        return not errors, errors

    def mapping(self):
        ok, errors = self.validate()
        if not ok:
            raise ValueError(self.error_label.text() or "; ".join(errors.values()))
        columns = self.values()["mapping"]
        try:
            return CsvMapping(columns["time"], columns["swa"], columns["accelerator"],
                              columns["brake"], "s", "rad", "fraction", "hold")
        except (TypeError, ValueError) as exc:
            message = str(exc)
            self.error_label.setText(message)
            raise ValueError(message) from exc

    def source(self):
        mapping = self.mapping()
        return load_csv_source(Path(self.source_edit.text().strip()), mapping)

    def accept(self):
        ok, _ = self.validate()
        if ok:
            super().accept()
