"""Strict CSV playback import and mapping dialog."""
from pathlib import Path

from simplified_vehicle_models.controls.maneuvers.csv_source import CsvMapping, load_csv_source
from PyQt6.QtWidgets import (QDialog, QFormLayout, QLineEdit, QDialogButtonBox, QLabel,
                             QVBoxLayout, QComboBox, QCheckBox, QDoubleSpinBox, QScrollArea,
                             QWidget, QSizePolicy)


class CsvDialog(QDialog):
    REQUIRED = ("swa", "accelerator", "brake")

    def __init__(self, source="", mapping=None, parent=None):
        super().__init__(parent)
        mapping = mapping or {}
        self.setWindowTitle("Import CSV commands")
        self.setMinimumSize(480, 520)
        self.source_edit = QLineEdit(str(source), self)
        self.source_edit.setMinimumWidth(320)
        self.mapping_edits = {name: QLineEdit(str(mapping.get(name) or ""), self)
                              for name in (*self.REQUIRED, "time")}
        self.has_time = QCheckBox("Use time column", self)
        self.has_time.setChecked(bool(mapping.get("time", "time")))
        self.has_time.toggled.connect(self._time_toggled)
        self.time_edit = self.mapping_edits["time"]
        if not self.time_edit.text(): self.time_edit.setText("time")
        self.sample_dt = QDoubleSpinBox(self)
        self.sample_dt.setRange(0.000001, 1e6); self.sample_dt.setDecimals(6); self.sample_dt.setValue(0.01)
        self.sample_dt.setSuffix(" s")
        self.time_unit = self._combo(("s", "ms"), mapping.get("time_unit", "s"))
        self.swa_unit = self._combo(("rad", "deg"), mapping.get("swa_unit", "rad"))
        self.pedal_unit = self._combo(("fraction", "percent"), mapping.get("pedal_unit", "fraction"))
        self.interpolation = self._combo(("hold", "linear"), mapping.get("interpolation", "hold"))
        self.delimiter = self._combo((("Comma (,) ", ","), ("Semicolon (;) ", ";")), mapping.get("delimiter", ","), data=True)
        self.decimal = self._combo((("Period (.)", "."), ("Comma (,)", ",")), mapping.get("decimal", "."), data=True)
        self.rebase = QCheckBox("Rebase nonzero time origin to zero", self)
        self.rebase.setChecked(bool(mapping.get("rebase", False)))
        self.error_label = QLabel(self); self.error_label.setWordWrap(True)
        self.status_label = QLabel("Choose columns and options; CSV bounds and preview appear after validation.", self)
        self.status_label.setWordWrap(True)
        form = QFormLayout()
        form.addRow("CSV source", self.source_edit)
        form.addRow("Time base", self.has_time)
        form.addRow("Time column", self.time_edit)
        form.addRow("Sample interval (no time column)", self.sample_dt)
        for name in self.REQUIRED: form.addRow(f"{name.title()} column", self.mapping_edits[name])
        form.addRow("Time unit", self.time_unit); form.addRow("SWA unit", self.swa_unit)
        form.addRow("Pedal unit", self.pedal_unit); form.addRow("Interpolation", self.interpolation)
        form.addRow("Delimiter", self.delimiter); form.addRow("Decimal", self.decimal)
        form.addRow("Time origin", self.rebase)
        contents = QWidget(self); contents.setLayout(form)
        scroll = QScrollArea(self); scroll.setWidgetResizable(True); scroll.setWidget(contents)
        scroll.setFocusPolicy(self.focusPolicy()); scroll.setAccessibleName("CSV mapping options")
        layout = QVBoxLayout(self); layout.addWidget(scroll); layout.addWidget(self.status_label); layout.addWidget(self.error_label)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel, self)
        buttons.accepted.connect(self.accept); buttons.rejected.connect(self.reject); layout.addWidget(buttons)
        self._time_toggled(self.has_time.isChecked())

    @staticmethod
    def _combo(options, selected, data=False):
        combo = QComboBox()
        for option in options:
            if data: combo.addItem(option[0], option[1])
            else: combo.addItem(option)
        index = combo.findData(selected) if data else combo.findText(selected)
        combo.setCurrentIndex(max(0, index))
        return combo

    def _time_toggled(self, checked):
        self.time_edit.setEnabled(checked); self.sample_dt.setEnabled(not checked)

    def values(self):
        return {"source": self.source_edit.text().strip(),
                "mapping": {key: edit.text().strip() for key, edit in self.mapping_edits.items()}}

    def validate(self):
        data = self.values(); errors = {}
        if not data["source"] or not Path(data["source"]).is_file(): errors["csv.source"] = "CSV source must be an existing file"
        cols = [*self.REQUIRED, *(('time',) if self.has_time.isChecked() else ())]
        missing = [key for key in cols if not data["mapping"][key]]
        if missing: errors["csv.mapping"] = "Column mapping required for: " + ", ".join(missing)
        self.error_label.setText("; ".join(errors.values()))
        return not errors, errors

    def mapping(self):
        ok, errors = self.validate()
        if not ok: raise ValueError(self.error_label.text() or "; ".join(errors.values()))
        cols = self.values()["mapping"]
        try:
            return CsvMapping(cols["time"] if self.has_time.isChecked() else None, cols["swa"], cols["accelerator"], cols["brake"],
                              self.time_unit.currentText(), self.swa_unit.currentText(), self.pedal_unit.currentText(),
                              self.interpolation.currentText(), None if self.has_time.isChecked() else self.sample_dt.value(),
                              self.rebase.isChecked(), self.delimiter.currentData(), self.decimal.currentData())
        except (TypeError, ValueError) as exc:
            self.error_label.setText(str(exc))
            raise

    def _load(self):
        return load_csv_source(Path(self.source_edit.text().strip()), self.mapping())

    def source(self):
        try:
            source = self._load()
            self.error_label.clear()
            self.status_label.setText(f"Loaded {len(source.time_s)} samples; {source.time_s[0]:g}–{source.time_s[-1]:g} s; {source.mapping.interpolation} interpolation; "
                                      f"{'rebased' if source.mapping.rebase else 'origin retained'}; bounds 0–{source.time_s[-1]:g} s.")
            return source
        except (OSError, TypeError, ValueError) as exc:
            self.error_label.setText(str(exc)); return None

    def accept(self):
        source = self.source()
        if source is not None: super().accept()
