"""The vehicle GUI main window."""
from __future__ import annotations

from pathlib import Path

from PyQt6.QtCore import QSettings, Qt
from PyQt6.QtWidgets import (QCheckBox, QComboBox, QFileDialog, QFormLayout, QHBoxLayout, QLabel, QLineEdit,
                             QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QSplitter, QTableView,
                             QTreeWidget, QTreeWidgetItem, QVBoxLayout, QWidget)

from tools.gui_common import REPOSITORY_ROOT
from tools.sim_gui.catalogue import candidates, read_entry, scan_catalogue
from tools.vehicle_gui import save
from tools.vehicle_gui.model import ASSEMBLY, VehicleEdit
from tools.vehicle_gui.qt.parameter_table import ParameterTableModel

DEFAULT_OUTPUT = REPOSITORY_ROOT / "vehicles_out"
DEFAULT_OPEN = REPOSITORY_ROOT / "models" / "formulations"


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Vehicle GUI")
        self.settings = QSettings("Simplified-Vehicle-Models", "vehicle-gui")
        self.edit: VehicleEdit | None = None
        self.catalogue = ()
        self.key = ASSEMBLY
        self.tree = QTreeWidget(); self.tree.setHeaderLabels(["Slot", "File", ""]); self.tree.currentItemChanged.connect(self._select)
        self.heading = QLabel("Open a vehicle formulation.")
        self.lock = QLabel(); self.lock.setWordWrap(True); self.lock.setStyleSheet("color: #8a5a00")
        self.table = ParameterTableModel(); self.table.edited.connect(self._edited); self.table.refused.connect(lambda text: self.statusBar().showMessage(text, 6000))
        self.view = QTableView(); self.view.setModel(self.table); self.view.horizontalHeader().setStretchLastSection(True)
        self.swap = QComboBox(); self.swap_button = QPushButton("Use this component"); self.swap_button.clicked.connect(self._swap)
        self.revert_button = QPushButton("Revert slot"); self.revert_button.clicked.connect(self._revert_slot)
        self.checks = QPlainTextEdit(); self.checks.setReadOnly(True)
        self.verdict = QLabel(); self.verdict.setWordWrap(True)
        self.folder = QLineEdit(str(DEFAULT_OUTPUT)); browse = QPushButton("Browse…"); browse.clicked.connect(self._browse_folder)
        self.stem = QLineEdit("my_vehicle"); self.overwrite = QCheckBox("Overwrite existing files"); self.allow_models = QCheckBox("Allow writing inside models/")
        self.save_button = QPushButton("Save as new vehicle"); self.save_button.clicked.connect(self.save_vehicle)
        self.component_button = QPushButton("Save this slot as a new component…"); self.component_button.clicked.connect(self.save_component)
        self.open_button = QPushButton("Open…"); self.open_button.clicked.connect(self._open_dialog)
        right = QVBoxLayout(); right.addWidget(self.heading); right.addWidget(self.lock); right.addWidget(self.view, 3)
        row = QHBoxLayout(); row.addWidget(self.swap, 1); row.addWidget(self.swap_button); row.addWidget(self.revert_button); right.addLayout(row)
        right.addWidget(QLabel("Checks (the real compiler)")); right.addWidget(self.verdict); right.addWidget(self.checks, 2)
        form = QFormLayout(); line = QHBoxLayout(); line.addWidget(self.folder); line.addWidget(browse); form.addRow("Folder", line); form.addRow("File name", self.stem); form.addRow(self.overwrite); form.addRow(self.allow_models); right.addLayout(form)
        buttons = QHBoxLayout(); buttons.addWidget(self.component_button); buttons.addWidget(self.save_button); right.addLayout(buttons)
        right_widget = QWidget(); right_widget.setLayout(right)
        left = QVBoxLayout(); left.addWidget(self.open_button); left.addWidget(self.tree, 1); left_widget = QWidget(); left_widget.setLayout(left)
        splitter = QSplitter(Qt.Orientation.Horizontal); splitter.addWidget(left_widget); splitter.addWidget(right_widget); splitter.setStretchFactor(1, 2); self.setCentralWidget(splitter)
        geometry = self.settings.value("geometry")
        if geometry is not None: self.restoreGeometry(geometry)
        else: self.resize(1200, 780)
        self._refresh_buttons()

    def _open_dialog(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Open vehicle", str(DEFAULT_OPEN), "Model XML (*.xml)")
        if path: self.open_path(Path(path))

    def open_path(self, path: Path) -> bool:
        try:
            self.edit = VehicleEdit.open(path); self.catalogue = scan_catalogue()
        except ValueError as error:
            QMessageBox.warning(self, "Not opened", str(error)); return False
        self.key = ASSEMBLY; self._fill_tree(); self.heading.setText(f"{path.name}  (model {self.edit.configuration.model})"); self._refresh(); return True

    def _fill_tree(self) -> None:
        self.tree.blockSignals(True); self.tree.clear(); items = {}
        root = QTreeWidgetItem(["vehicle", self.edit.file(ASSEMBLY).name, ""]); root.setData(0, Qt.ItemDataRole.UserRole, ASSEMBLY); self.tree.addTopLevelItem(root); items[ASSEMBLY] = root
        for slot in self.edit.configuration.slots():
            item = QTreeWidgetItem([slot.key.rsplit("/", 1)[-1], self.edit.file(slot.key).name, ""]); item.setData(0, Qt.ItemDataRole.UserRole, slot.key); items[slot.parent or ASSEMBLY].addChild(item); items[slot.key] = item
        self.tree.expandAll(); self.tree.resizeColumnToContents(0); self.tree.blockSignals(False); self.tree.setCurrentItem(root); self._items = items; self._select(root)

    def _select(self, item, _previous=None) -> None:
        if item is None or self.edit is None: return
        self.key = item.data(0, Qt.ItemDataRole.UserRole); self._show_slot()

    def _show_slot(self) -> None:
        edit, key = self.edit, self.key; generator = edit.generator(key)
        self.lock.setText(f"{edit.file(key).name} is generated by {generator}. Its values are read-only here; change the inputs of that tool." if generator else "")
        self.table.set_rows(edit.rows(key), f"generated by {generator}" if generator else None); self.swap.clear()
        if key != ASSEMBLY:
            current = edit.file(key)
            for entry in candidates(read_entry(current), self.catalogue):
                self.swap.addItem(entry.path.relative_to(REPOSITORY_ROOT / "models").as_posix(), entry.path)
                if entry.path == current: self.swap.setCurrentIndex(self.swap.count() - 1)
        self._refresh_buttons()

    def _refresh_buttons(self) -> None:
        has = self.edit is not None; self.swap.setEnabled(has and self.key != ASSEMBLY); self.swap_button.setEnabled(has and self.key != ASSEMBLY); self.revert_button.setEnabled(has); self.save_button.setEnabled(has); self.component_button.setEnabled(has)

    def _refresh(self) -> None:
        if self.edit is None: return
        for key, item in self._items.items():
            marks = []; 
            if key in self.edit.edited_keys(): marks.append("edited")
            if key in save.diverging_keys(self.edit): marks.append("swapped")
            item.setText(1, self.edit.file(key).name); item.setText(2, ", ".join(marks))
        self._show_slot(); verdict = save.check_files(self.edit)
        if verdict.ok:
            self.verdict.setText("The vehicle compiles." + ("" if self.edit.changed else " (no changes)")); self.verdict.setStyleSheet("color: #1b6e2d")
            lines = (["Parameters the chosen model does not read:"] + [f"  {text}" for text in verdict.unconsumed] if verdict.unconsumed else [])
            if verdict.unused_roles: lines.append("Slots the chosen model does not use: " + ", ".join(verdict.unused_roles))
            self.checks.setPlainText("\n".join(lines) or "Nothing is unused.")
        else:
            self.verdict.setText("The vehicle does not compile: " + verdict.error); self.verdict.setStyleSheet("color: #b00020"); self.checks.setPlainText(verdict.error)

    def _edited(self, name: str, value: float) -> None:
        try: self.edit = self.edit.with_parameter(self.key, name, value)
        except ValueError as error: self.statusBar().showMessage(str(error), 6000)
        self._refresh()

    def _swap(self) -> None:
        path = self.swap.currentData()
        if path is None or Path(path) == self.edit.file(self.key): return
        try: self.edit = self.edit.with_choice(self.key, Path(path))
        except ValueError as error: QMessageBox.warning(self, "Not swapped", str(error)); return
        self._fill_tree(); self.tree.setCurrentItem(self._items[self.key]); self._refresh()

    def _revert_slot(self) -> None:
        if self.edit is None: return
        edit = self.edit
        for row in edit.rows(self.key): edit = edit.without_parameter(self.key, row.name)
        self.edit = edit; self._refresh()

    def _browse_folder(self) -> None:
        chosen = QFileDialog.getExistingDirectory(self, "Output folder", self.folder.text())
        if chosen: self.folder.setText(chosen)

    def save_vehicle(self) -> None:
        if self.edit is None: return
        try: saved = save.save_vehicle(self.edit, Path(self.folder.text()), self.stem.text().strip(), allow_models=self.allow_models.isChecked(), overwrite=self.overwrite.isChecked())
        except (save.SaveRefused, ValueError) as error: QMessageBox.warning(self, "Not saved", str(error)); return
        self.statusBar().showMessage(f"saved {saved.entry}", 10000)

    def save_component(self, *, model_id: str | None = None) -> None:
        if self.edit is None or self.key == ASSEMBLY: return
        from PyQt6.QtWidgets import QInputDialog
        if model_id is None:
            model_id, accepted = QInputDialog.getText(self, "New component", "Model id of the new component:", text=f"{self.edit.file(self.key).stem.replace('_', '-')}-copy")
            if not accepted: return
        values = dict(self.edit.parameters.get(self.key, {}))
        try: target = save.save_component(self.edit.file(self.key), values, Path(self.folder.text()), self.stem.text().strip(), model_id.strip(), allow_models=self.allow_models.isChecked(), overwrite=self.overwrite.isChecked())
        except (save.SaveRefused, ValueError) as error: QMessageBox.warning(self, "Not saved", str(error)); return
        self.statusBar().showMessage(f"saved {target}", 10000)

    def closeEvent(self, event) -> None:
        if self.edit is not None and self.edit.changed:
            if QMessageBox.question(self, "Quit", "There are unsaved changes. Quit anyway?") != QMessageBox.StandardButton.Yes:
                event.ignore(); return
        self.settings.setValue("geometry", self.saveGeometry()); event.accept()
