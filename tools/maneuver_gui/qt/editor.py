"""Qt adapter binding editable controls to the maneuver draft."""
from copy import deepcopy

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QLabel, QLineEdit, QFormLayout,
                             QComboBox, QScrollArea)
from simplified_vehicle_models.controls.maneuvers.types import ManeuverDefinition
from tools.sim_gui.qt.maneuver_tab import ManeuverDraft


class ManeuverEditor(QWidget):
    """Editable schema-backed definition with inline validation."""
    def __init__(self, payload=None, parent=None):
        super().__init__(parent)
        self.draft = ManeuverDraft.from_dict(payload or {})
        self.validation_label = QLabel(self)
        self.ownership_label = QLabel(self)
        self.fields = {}
        self._building = True
        inner = QWidget(self)
        form = QFormLayout(inner)
        self.preset = QComboBox(inner)
        self.preset.setObjectName('preset')
        self.preset.addItems(['step_steer','ramp_steer','sine_steer','straight_maintain','straight_accel_brake','circular_skidpad','csv_playback'])
        self.steering_mode = QComboBox(inner)
        self.steering_mode.setObjectName('steering.mode')
        self.steering_mode.addItems(['peak_swa','peak_ay','path_pid','radius_pid','locked_swa','csv'])
        self.longitudinal_mode = QComboBox(inner)
        self.longitudinal_mode.setObjectName('longitudinal.mode')
        self.longitudinal_mode.addItems(['cruise_pid','constant_accelerator','csv'])
        for label, widget in [('Preset',self.preset),('Steering mode',self.steering_mode),('Longitudinal mode',self.longitudinal_mode)]: form.addRow(label,widget)
        for path in ('steering.amplitude_rad','steering.amplitude_mps2','initial.speed_mps','longitudinal.accelerator','longitudinal.target_speed_mps','profile.lead_in_s','profile.rise_s','profile.hold_s','profile.release_s','profile.recovery_s','profile.duration_s'):
            edit=QLineEdit(inner); edit.setObjectName(path); self.fields[path]=edit; form.addRow(path,edit)
        scroll=QScrollArea(self); scroll.setWidgetResizable(True); scroll.setWidget(inner)
        layout=QVBoxLayout(self); layout.addWidget(self.validation_label); layout.addWidget(self.ownership_label); layout.addWidget(scroll)
        self.preset.currentTextChanged.connect(self._preset_changed)
        self.steering_mode.currentTextChanged.connect(self._mode_changed)
        self.longitudinal_mode.currentTextChanged.connect(self._mode_changed)
        for path,edit in self.fields.items(): edit.textEdited.connect(lambda text,p=path:self._field_changed(p,text))
        self._building=False
        self._sync_from_draft()

    def _sync_from_draft(self):
        data=self.draft.to_dict(); self._building=True
        for widget,value in ((self.preset,data.get('preset')),(self.steering_mode,data.get('steering',{}).get('mode')),(self.longitudinal_mode,data.get('longitudinal',{}).get('mode'))):
            if value and widget.findText(str(value))>=0: widget.setCurrentText(str(value))
        for path,edit in self.fields.items():
            try:
                value=data
                for key in path.split('.'): value=value[key]
                edit.setText(str(value))
            except (KeyError,TypeError): edit.clear()
        self._building=False; self.validate()

    def _field_changed(self,path,text):
        if self._building:return
        keys=path.split('.'); value=self.draft.to_dict()
        try:
            node=value
            for key in keys[:-1]: node=node[key]
            old=node[keys[-1]]
            node[keys[-1]]=float(text) if isinstance(old,(int,float)) and not isinstance(old,bool) else text
            self.draft=ManeuverDraft.from_dict(value)
        except (ValueError,KeyError,TypeError): pass
        self.validate()

    def _set_modes(self):
        data=self.draft.to_dict(); data['preset']=self.preset.currentText(); data.setdefault('steering',{})['mode']=self.steering_mode.currentText(); data.setdefault('longitudinal',{})['mode']=self.longitudinal_mode.currentText()
        if data['preset']=='csv_playback': data['steering']['mode']='csv'; data['longitudinal']['mode']='csv'; self._building=True; self.steering_mode.setCurrentText('csv'); self.longitudinal_mode.setCurrentText('csv'); self._building=False
        self.draft=ManeuverDraft.from_dict(data); self.validate()

    def _preset_changed(self,*_):
        if not self._building:self._set_modes()
    def _mode_changed(self,*_):
        if not self._building:self._set_modes()

    def definition(self):
        definition,errors=self.validate()
        if definition is None: raise ValueError('; '.join(errors.values()) or 'Invalid maneuver definition')
        return definition

    def load_definition(self,definition):
        data=definition.to_dict() if isinstance(definition,ManeuverDefinition) else deepcopy(definition)
        return self.set_draft(data)
    def set_draft(self,payload):
        self.draft=ManeuverDraft.from_dict(payload); self._sync_from_draft(); return self.validate()
    def validate(self):
        definition,errors=self.draft.validate(); self.validation_label.setText('; '.join(errors.values()))
        enabled=self.draft.enabled_fields()
        for path,edit in self.fields.items():
            alias={'initial.speed_mps':True,'profile.lead_in_s':True,'profile.rise_s':True,'profile.hold_s':True,'profile.release_s':True,'profile.recovery_s':True,'profile.duration_s':True}
            edit.setEnabled(enabled.get(path,alias.get(path,True)))
        is_csv=self.draft.to_dict().get('preset')=='csv_playback'
        self.steering_mode.setEnabled(not is_csv); self.longitudinal_mode.setEnabled(not is_csv)
        self.ownership_label.setText('Channel ownership: CSV owns SWA, accelerator, and brake; PID disabled.' if is_csv else 'Channel ownership: steering mode / longitudinal mode')
        return definition,errors
    def to_dict(self): return self.draft.to_dict()
