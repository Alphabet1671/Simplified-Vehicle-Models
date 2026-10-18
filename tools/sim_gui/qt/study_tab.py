"""Qt study definition and case preview tab."""
from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import QRegularExpression
from PyQt6.QtGui import QRegularExpressionValidator
from PyQt6.QtWidgets import (
    QLineEdit,
    QFileDialog,
    QFormLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from tools.sim_gui import REPOSITORY_ROOT
from tools.sim_gui.catalogue import candidates, read_entry, scan_catalogue, tracks
from tools.sim_gui.study import (
    Dimension,
    StudyDefinition,
    TooManyCases,
    WARNING_CASES,
    dimension_from_sweep,
    expand,
)


class StudyTab(QWidget):
    """Edit study dimensions and preview expanded cases."""

    def __init__(self, configuration, settings, parent=None) -> None:
        """Create a study tab based on a vehicle configuration and settings."""
        super().__init__(parent)
        self.configuration = configuration
        self.settings = settings
        self._catalogue = scan_catalogue()
        self._dimensions: list[Dimension] = []
        self._dimension_values: dict[str, tuple[str, ...]] = {}
        self._dimension_labels: dict[str, tuple[str, ...]] = {}

        self.one_at_a_time = QRadioButton("One-at-a-time")
        self.full_factorial = QRadioButton("Full factorial")
        self.one_at_a_time.setChecked(True)
        self.one_at_a_time.toggled.connect(self._refresh_preview)
        self.full_factorial.toggled.connect(self._refresh_preview)
        self.study_name = QLineEdit("study")
        self.study_name.setValidator(QRegularExpressionValidator(QRegularExpression(r"[A-Za-z0-9_-]{1,40}"), self.study_name))
        self.study_name.textChanged.connect(self._sanitize_name)
        self.dimension_list = QListWidget()
        self.dimension_list.currentRowChanged.connect(self._select_dimension)
        self.member_list = QListWidget()
        self.preview_label = QLabel("Case preview")
        self.preview_count = QLabel()
        self.preview = QTableWidget(0, 2)
        self.preview.setHorizontalHeaderLabels(("ID", "Label"))

        add = QPushButton("Add dimension")
        add.clicked.connect(self._add_dimension)
        add_sweep = QPushButton("Add sweep folder…")
        add_sweep.clicked.connect(self._add_sweep)
        remove = QPushButton("Remove")
        remove.clicked.connect(self._remove_dimension)

        form = QFormLayout()
        form.addRow("Study name", self.study_name)
        form.addRow(self.one_at_a_time, self.full_factorial)
        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(add)
        layout.addWidget(add_sweep)
        layout.addWidget(remove)
        layout.addWidget(self.dimension_list)
        layout.addWidget(self.member_list)
        layout.addWidget(self.preview_label)
        layout.addWidget(self.preview_count)
        layout.addWidget(self.preview)

    def _sanitize_name(self, text: str) -> None:
        """Keep the study name within its documented character set."""
        sanitized = "".join(char for char in text if char.isascii() and (char.isalnum() or char in "_-") )[:40]
        if sanitized != text:
            self.study_name.blockSignals(True)
            self.study_name.setText(sanitized)
            self.study_name.blockSignals(False)

    def definition(self, base) -> StudyDefinition:
        """Return a study definition using the selected dimensions."""
        return StudyDefinition(
            self.study_name.text(), str(base.source), base.track if hasattr(base, "track") else "",
            base.model, {key: str(value) for key, value in base.choices.items()}, self.settings,
            "one-at-a-time" if self.one_at_a_time.isChecked() else "full-factorial",
            tuple(self._dimensions),
        )

    def set_definition(self, definition: StudyDefinition) -> None:
        """Replace the tab's controls from a serialized definition."""
        self.study_name.setText(definition.name)
        self.one_at_a_time.setChecked(definition.mode == "one-at-a-time")
        self.full_factorial.setChecked(definition.mode == "full-factorial")
        self._dimensions = list(definition.dimensions)
        self._rebuild_dimension_list()
        self._refresh_preview()

    def _add_dimension(self) -> None:
        keys = [slot.key for slot in self.configuration.slots()] + ["model", "track"]
        for key in keys:
            if key not in {dimension.key for dimension in self._dimensions}:
                dimension = self._dimension_for_key(key)
                self._dimensions.append(dimension)
                self._rebuild_dimension_list()
                self._refresh_preview()
                break

    def _dimension_for_key(self, key: str) -> Dimension:
        if key == "model":
            values = ("planar-7dof", "14dof-simple")
            return Dimension(key, values)
        if key == "track":
            entries = tracks(self._catalogue)
            values = tuple(str(entry.path) for entry in entries)
            return Dimension(key, values, tuple(entry.path.stem for entry in entries))
        current = read_entry(self.configuration.choices[key])
        entries = candidates(current, self._catalogue)
        return Dimension(key, tuple(str(entry.path) for entry in entries), tuple(entry.path.stem for entry in entries))

    def _add_sweep(self) -> None:
        folder = QFileDialog.getExistingDirectory(self, "Select sweep folder", str(REPOSITORY_ROOT / "models/sweeps"))
        if not folder:
            return
        try:
            dimension = dimension_from_sweep(Path(folder), self.configuration)
        except ValueError as error:
            QMessageBox.warning(self, "Invalid sweep", str(error))
            return
        self._dimensions.append(dimension)
        self._rebuild_dimension_list()
        self._refresh_preview()

    def _remove_dimension(self) -> None:
        row = self.dimension_list.currentRow()
        if row >= 0:
            self._dimensions.pop(row)
            self._rebuild_dimension_list()
            self._refresh_preview()

    def _rebuild_dimension_list(self) -> None:
        self.dimension_list.clear()
        for dimension in self._dimensions:
            label = dimension.key
            if dimension.sweep:
                label = f"{label} ← sweeps/{Path(dimension.sweep).name} ({len(dimension.values)})"
            self.dimension_list.addItem(label)

    def _select_dimension(self, row: int) -> None:
        self.member_list.clear()
        if 0 <= row < len(self._dimensions):
            dimension = self._dimensions[row]
            for label in dimension.labels:
                item = QListWidgetItem(label, self.member_list)
                item.setCheckState(__import__("PyQt6.QtCore", fromlist=["Qt"]).Qt.CheckState.Checked)

    def _refresh_preview(self, *_args) -> None:
        try:
            definition = self.definition(self.configuration)
            cases = expand(definition)
        except TooManyCases as error:
            self.preview_count.setText(str(error))
            self.preview_count.setStyleSheet("color: red")
            self.preview.setRowCount(0)
            return
        except (ValueError, TypeError) as error:
            self.preview_count.setText(str(error))
            self.preview_count.setStyleSheet("color: red")
            self.preview.setRowCount(0)
            return
        self.preview_count.setText(f"{len(cases)} cases")
        self.preview_count.setStyleSheet("color: red" if len(cases) > WARNING_CASES else "")
        self.preview.setRowCount(len(cases))
        for row, case in enumerate(cases):
            self.preview.setItem(row, 0, QTableWidgetItem(case.id))
            self.preview.setItem(row, 1, QTableWidgetItem(case.label))
