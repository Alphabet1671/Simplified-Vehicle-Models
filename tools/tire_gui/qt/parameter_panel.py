from PyQt6.QtCore import Qt, QSignalBlocker, pyqtSignal
from PyQt6.QtWidgets import QWidget,QVBoxLayout,QHBoxLayout,QLabel,QSlider,QDoubleSpinBox,QCheckBox,QDoubleSpinBox
from ..parameters import specs

class ParameterPanel(QWidget):
    parameterChanged=pyqtSignal(str,float)
    locksChanged=pyqtSignal(object)
    boundsChanged=pyqtSignal(object)
    def __init__(self,parent=None):
        super().__init__(parent); self.specs={s.name:s for s in specs()}; self.controls={}; self.snapshot=None
        self.layout=QVBoxLayout(self); self.rows=QVBoxLayout(); self.layout.addLayout(self.rows); self.layout.addStretch()
    def set_snapshot(self,snapshot):
        self.snapshot=snapshot
        metadata = dict(getattr(snapshot, 'session_metadata', {}) or {})
        saved_bounds = metadata.get('bounds', {})
        saved_locks = metadata.get('locks', {})
        while self.rows.count():
            item=self.rows.takeAt(0); widget=item.widget()
            if widget: widget.deleteLater()
        self.controls={}
        for name,value in snapshot.values.items():
            spec=self.specs[name]; row=QHBoxLayout(); label=QLabel(f'{name} ({spec.unit})')
            spin=QDoubleSpinBox(); spin.setDecimals(12); spin.setRange(-1e12,1e12); spin.setKeyboardTracking(False); spin.setValue(value); spin.setAccessibleName(name+' value')
            lo,hi=saved_bounds.get(name,self._bounds(name,value)); lo=float(lo); hi=float(hi)
            if not lo < hi: lo,hi=self._bounds(name,value)
            slider=QSlider(Qt.Orientation.Horizontal); slider.setRange(0,10000); slider.setValue(round((value-lo)/(hi-lo)*10000) if hi>lo else 0)
            lock=QCheckBox('Lock'); lock.setChecked(saved_locks.get(name,spec.default_locked)); lock.setAccessibleName(name+' lock')
            lower=QDoubleSpinBox(); upper=QDoubleSpinBox()
            for box,v in ((lower,lo),(upper,hi)): box.setDecimals(12); box.setRange(-1e12,1e12); box.setValue(v)
            self.controls[name]=(spin,slider,lock,lower,upper)
            spin.valueChanged.connect(lambda v,n=name:self._changed(n,v))
            slider.valueChanged.connect(lambda v,n=name:self._from_slider(n,v))
            lock.toggled.connect(self._locks)
            lower.valueChanged.connect(lambda _v,n=name:self._bounds_changed(n))
            upper.valueChanged.connect(lambda _v,n=name:self._bounds_changed(n))
            row.addWidget(label); row.addWidget(lower); row.addWidget(slider,1); row.addWidget(spin); row.addWidget(upper); row.addWidget(lock); self.rows.addLayout(row)
    def _bounds(self,name,value):
        groups={'k_x':(0,50000),'k_y':(0,50000),'mu':(.05,5),'h_':(-3,3),'v_m':(.001,5),'e_':(-2,5),'d_':(-.5,10),'length_scale':(.05,10),'trail_pole_ratio':(.05,20),'trail_lead_ratio':(0,1),'f_z':(100,3000),'k_c':(1,2e6),'r_':(0,1)}
        for key,b in groups.items():
            if key in name:return b
        return (min(value-1,0),max(value+1,2))
    def _changed(self,name,value):
        spin,slider,lock,lo,hi=self.controls[name]; lower,upper=lo.value(),hi.value()
        if lower<upper: slider.blockSignals(True); slider.setValue(round(max(0,min(10000,(value-lower)/(upper-lower)*10000)))); slider.blockSignals(False)
        self.parameterChanged.emit(name,float(value))
    def _from_slider(self,name,v):
        spin,_,_,lo,hi=self.controls[name]; value=lo.value()+v/10000*(hi.value()-lo.value()); blocker=QSignalBlocker(spin); spin.setValue(value); del blocker; self.parameterChanged.emit(name,float(value))
    def _bounds_changed(self,name):
        spin,slider,_,lo,hi=self.controls[name]
        if hi.value()>lo.value(): slider.setValue(round((spin.value()-lo.value())/(hi.value()-lo.value())*10000))
        self.boundsChanged.emit(self.bounds())
    def _locks(self,*_): self.locksChanged.emit(frozenset(n for n,c in self.controls.items() if c[2].isChecked()))
    def locked(self): return frozenset(n for n,c in self.controls.items() if c[2].isChecked())
    def bounds(self): return {n:(c[3].value(),c[4].value()) for n,c in self.controls.items()}
