"""Run settings widget."""
from __future__ import annotations
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QWidget,QFormLayout,QDoubleSpinBox,QCheckBox,QComboBox,QPushButton
from tools.sim_gui.settings import RunSettings
class SettingsTab(QWidget):
 settingsChanged=pyqtSignal(object)
 def __init__(self,parent=None):
  super().__init__(parent); self._settings=RunSettings(); self._model='planar-7dof'; f=QFormLayout(self)
  self._fields={}
  for name in ('duration','timeout_seconds','maximum_speed','pedal_travel_time','preview_seconds','heading_gain','lateral_gain','maximum_step','rtol','spacing'):
   w=QDoubleSpinBox(); w.setRange(0,1e9); w.setValue(getattr(self._settings,name)); w.setDecimals(6 if name in ('rtol','maximum_step') else 3); self._fields[name]=w; setattr(self,name,w); f.addRow(name.replace('_',' ').title(),w); w.valueChanged.connect(self._changed)
  self.lateral_cap=QCheckBox(); self.lateral_value=QDoubleSpinBox(); self.lateral_value.setRange(0,1e9); f.addRow('Lateral cap',self.lateral_cap); f.addRow('',self.lateral_value); self.lateral_cap.toggled.connect(self._changed); self.lateral_value.valueChanged.connect(self._changed)
  for name,vals in (('method',('Radau','BDF')),('planner',('conservative','live')),('record_precision',('float32','float64'))):
   w=QComboBox(); w.addItems(vals); w.setCurrentText(getattr(self._settings,name)); setattr(self,name,w); f.addRow(name.title(),w); w.currentTextChanged.connect(self._changed)
  self.quasi_static_attitude=QCheckBox(); f.addRow('Quasi-static attitude',self.quasi_static_attitude); self.quasi_static_attitude.toggled.connect(self._changed)
  b=QPushButton('Preset: Reference lap'); f.addRow(b); b.clicked.connect(self._preset)
 def settings(self): return self._settings
 def set_settings(self,s):
  self._settings=s
  for n,w in self._fields.items(): w.setValue(getattr(s,n))
  for n in ('method','planner','record_precision'): getattr(self,n).setCurrentText(getattr(s,n))
  self.quasi_static_attitude.setChecked(s.quasi_static_attitude)
 def set_model(self,name):
  self._model=name; self.timeout_seconds.setMaximum(300.0 if name=='planar-7dof' else 1e6); self.timeout_seconds.setValue(RunSettings.for_model(name).timeout_seconds); self.quasi_static_attitude.setEnabled(name=='planar-7dof')
 def _preset(self): self.set_settings(RunSettings.for_model(self._model)); self._changed()
 def _changed(self,*_):
  vals={n:w.value() for n,w in self._fields.items()}; vals.update(method=self.method.currentText(),planner=self.planner.currentText(),record_precision=self.record_precision.currentText(),quasi_static_attitude=self.quasi_static_attitude.isChecked(),lateral_acceleration=self.lateral_value.value() if self.lateral_cap.isChecked() else None); self._settings=RunSettings(**vals); self.settingsChanged.emit(self._settings)
