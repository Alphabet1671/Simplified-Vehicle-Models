"""Vehicle model selection widget."""
from __future__ import annotations
from pathlib import Path
from xml.etree import ElementTree
from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (QComboBox, QFileDialog, QFormLayout, QLabel, QMessageBox,
    QPushButton, QTableWidget, QTableWidgetItem, QToolButton, QVBoxLayout, QWidget)
from tools.sim_gui.catalogue import candidates, read_entry, scan_catalogue, tracks
from tools.sim_gui.vehicle import VehicleConfiguration

_warn = QMessageBox.warning

class ModelPanel(QWidget):
    configurationChanged = pyqtSignal(object)
    slotSelected = pyqtSignal(str)
    def __init__(self, configuration=None, parent=None):
        super().__init__(parent); self._cfg = None; self._baseline = None; self._slot_widgets = {}
        self._catalogue = scan_catalogue(); self._building = False
        self._source = QLabel(); self._source.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self._error = QLabel(); self._error.setStyleSheet("color: red")
        self._form = QFormLayout(); self._table = QTableWidget(0, 3); self._table.setHorizontalHeaderLabels(("Name","Value","Unit")); self._table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        layout = QVBoxLayout(self); layout.addWidget(self._source); layout.addWidget(self._error); layout.addLayout(self._form); layout.addWidget(QLabel("Parameters")); layout.addWidget(self._table)
        if configuration is not None: self.set_configuration(configuration)
    def set_configuration(self, cfg):
        self._cfg = cfg; self._baseline = cfg if self._baseline is None else self._baseline; self._rebuild()
    def configuration(self): return self._cfg
    def track(self): return self._track.currentData() if hasattr(self, '_track') else None
    def baseline(self): return self._baseline
    def _clear_form(self):
        while self._form.rowCount(): self._form.removeRow(0)
        self._slot_widgets.clear()
    def _rebuild(self):
        self._building = True; self._clear_form(); self._source.setText(str(self._cfg.source)); self._source.setToolTip(str(self._cfg.source))
        model = QComboBox(); model.addItems(("planar-7dof", "14dof-simple")); model.setCurrentText(self._cfg.model); model.currentTextChanged.connect(self._change_model); self._form.addRow("Model", model); self._model=model
        self._track=QComboBox(); ts=tracks(self._catalogue); self._track.addItem("", None)
        for e in ts: self._track.addItem(e.path.stem, str(e.path))
        self._form.addRow("Track", self._track)
        for slot in self._cfg.slots():
            combo=QComboBox(); entry=read_entry(self._cfg.choices[slot.key]); opts=candidates(entry,self._catalogue)
            for e in opts: combo.addItem(e.path.stem, str(e.path))
            combo.setCurrentIndex(max(0, combo.findData(str(self._cfg.choices[slot.key])))); combo.currentIndexChanged.connect(lambda i,k=slot.key,c=combo: self._change_slot(k,c,i)); combo.currentTextChanged.connect(lambda _,k=slot.key: self._show_params(k)); combo.focusInEvent=lambda event,k=slot.key,orig=combo.focusInEvent: (self.slotSelected.emit(k), orig(event))[1]
            browse=QToolButton(); browse.setText("Browse…"); browse.clicked.connect(lambda _,k=slot.key,c=combo:self._browse(k,c)); reset=QToolButton(); reset.setText("Reset"); reset.setEnabled(self._cfg.choices[slot.key] != self._baseline.choices.get(slot.key)); reset.clicked.connect(lambda _,k=slot.key:self._reset(k))
            row=QWidget(); from PyQt6.QtWidgets import QHBoxLayout; h=QHBoxLayout(row); h.setContentsMargins(slot.key.count('/')*16,0,0,0); h.addWidget(combo); h.addWidget(browse); h.addWidget(reset); self._form.addRow(slot.label,row); self._slot_widgets[slot.key]=(combo,browse,reset)
        self._building=False
        if self._cfg.slots(): self._show_params(self._cfg.slots()[0].key)
        self._error.setText("" if self._cfg.runnable else "no forward runner for point-mass; select planar-7dof or 14dof-simple")
    def _change_model(self,name):
        try: self._cfg=self._cfg.with_model(name); self._rebuild(); self.configurationChanged.emit(self._cfg)
        except ValueError: pass
    def _change_slot(self,key,combo,index):
        if self._building: return
        old=self._cfg.choices[key]
        try:
            self._cfg=self._cfg.with_choice(key,Path(combo.itemData(index)))
            if any(s.key.startswith(key + "/") for s in self._cfg.slots()):
                self._rebuild()
            else:
                font=combo.font(); font.setBold(self._cfg.choices[key] != self._baseline.choices.get(key)); combo.setFont(font)
                self._slot_widgets[key][2].setEnabled(self._cfg.choices[key] != self._baseline.choices.get(key))
            self.configurationChanged.emit(self._cfg)
        except ValueError as e:
            _warn(self,"Incompatible component",str(e)); combo.blockSignals(True); combo.setCurrentIndex(combo.findData(str(old))); combo.blockSignals(False)
    def _show_params(self,key):
        try: rows=self._cfg.parameters(key)
        except ValueError: rows=[]
        self._table.setRowCount(len(rows))
        for r,row in enumerate(rows):
            for c,val in enumerate(row): self._table.setItem(r,c,QTableWidgetItem(str(val)))
        self.slotSelected.emit(key)
    def _browse(self,key,combo):
        path,_=QFileDialog.getOpenFileName(self,"Select component",filter="XML (*.xml)")
        if path:
            combo.addItem(Path(path).stem,path); combo.setCurrentIndex(combo.count()-1)
    def _reset(self,key):
        self._change_slot(key,self._slot_widgets[key][0],self._slot_widgets[key][0].findData(str(self._baseline.choices[key])))
