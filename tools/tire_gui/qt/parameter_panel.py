from PyQt6.QtCore import Qt, QSignalBlocker, pyqtSignal
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QSlider, QDoubleSpinBox,
    QCheckBox, QTabWidget, QScrollArea, QGridLayout,
)
from ..parameters import specs


class ParameterPanel(QWidget):
    parameterChanged = pyqtSignal(str, float)
    locksChanged = pyqtSignal(object)
    boundsChanged = pyqtSignal(object)
    GROUPS = ('Mu', 'Stiffness factors', 'Shape factors', 'Misc')

    def __init__(self, parent=None):
        super().__init__(parent)
        self.specs = {s.name: s for s in specs()}
        self.controls = {}
        self.snapshot = None
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(2, 2, 2, 2)
        self.tabs = QTabWidget()
        self.group_layouts = []
        self.pages = []
        for title in self.GROUPS:
            page = QWidget()
            page_layout = QVBoxLayout(page)
            page_layout.setContentsMargins(4, 4, 4, 4)
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            body = QWidget()
            grid = QGridLayout(body)
            grid.setContentsMargins(2, 2, 2, 2)
            grid.setHorizontalSpacing(5)
            grid.setVerticalSpacing(3)
            scroll.setWidget(body)
            page_layout.addWidget(scroll)
            self.tabs.addTab(page, title)
            self.pages.append(body)
            self.group_layouts.append(grid)
        self.tabs.setMinimumHeight(220)
        self.layout.addWidget(self.tabs)

    @staticmethod
    def parameter_group(name):
        if name.startswith(('mu_', 'h_', 'v_m')):
            return 0
        if name.startswith('k_'):
            return 1
        if name.startswith(('e_', 'd_')):
            return 2
        return 3

    def set_snapshot(self, snapshot):
        active_tab = self.tabs.currentIndex()
        self.snapshot = snapshot
        metadata = dict(getattr(snapshot, 'session_metadata', {}) or {})
        saved_bounds = metadata.get('bounds', {})
        saved_locks = metadata.get('locks', {})
        for grid, body in zip(self.group_layouts, self.pages):
            while grid.count():
                item = grid.takeAt(0)
                widget = item.widget()
                if widget:
                    widget.hide()
                    widget.setParent(None)
                    widget.deleteLater()
            body.adjustSize()
        self.controls = {}
        group_rows = [0] * len(self.GROUPS)
        for name, value in snapshot.values.items():
            spec = self.specs[name]
            group = self.parameter_group(name)
            row_num = group_rows[group]
            group_rows[group] += 1
            row = QWidget()
            editor = QGridLayout(row)
            editor.setContentsMargins(0, 0, 0, 0)
            editor.setHorizontalSpacing(4)
            label = QLabel(f'{name} ({spec.unit})')
            spin = QDoubleSpinBox()
            spin.setDecimals(12)
            spin.setRange(-1e12, 1e12)
            spin.setKeyboardTracking(False)
            spin.setValue(value)
            spin.setAccessibleName(name + ' value')
            lo, hi = saved_bounds.get(name, self._bounds(name, value))
            lo, hi = float(lo), float(hi)
            if not lo < hi:
                lo, hi = self._bounds(name, value)
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(0, 10000)
            slider.setValue(round((value-lo)/(hi-lo)*10000) if hi > lo else 0)
            lock = QCheckBox('Lock')
            lock.setChecked(saved_locks.get(name, spec.default_locked))
            lock.setAccessibleName(name + ' lock')
            lower, upper = QDoubleSpinBox(), QDoubleSpinBox()
            for box, v in ((lower, lo), (upper, hi)):
                box.setDecimals(12)
                box.setRange(-1e12, 1e12)
                box.setValue(v)
                box.setMaximumWidth(90)
            label.setMinimumWidth(130)
            editor.addWidget(label, 0, 0)
            editor.addWidget(lower, 0, 1)
            slider.setMinimumWidth(480)
            editor.addWidget(slider, 2, 0, 1, 4)
            editor.addWidget(lock, 0, 3)
            editor.addWidget(QLabel('Value'), 1, 0)
            editor.addWidget(spin, 1, 1)
            editor.addWidget(QLabel('to'), 1, 2)
            editor.addWidget(upper, 1, 3)
            self.controls[name] = (spin, slider, lock, lower, upper)
            spin.valueChanged.connect(lambda v, n=name: self._changed(n, v))
            slider.valueChanged.connect(lambda v, n=name: self._from_slider(n, v))
            lock.toggled.connect(self._locks)
            lower.valueChanged.connect(lambda _v, n=name: self._bounds_changed(n))
            upper.valueChanged.connect(lambda _v, n=name: self._bounds_changed(n))
            self.group_layouts[group].addWidget(row, row_num, 0)
        for group, grid in enumerate(self.group_layouts):
            grid.setRowStretch(group_rows[group], 1)
        self.tabs.setCurrentIndex(min(max(active_tab, 0), self.tabs.count() - 1))

    def _bounds(self, name, value):
        groups = {'k_x': (0, 50000), 'k_y': (0, 50000), 'mu': (.05, 5), 'h_': (-3, 3), 'v_m': (.001, 5), 'e_': (-2, 5), 'd_': (-.5, 10), 'length_scale': (.05, 10), 'trail_pole_ratio': (.05, 20), 'trail_lead_ratio': (0, 1), 'f_z': (100, 3000), 'k_c': (1, 2e6), 'r_': (0, 1)}
        for key, bounds in groups.items():
            if key in name:
                return bounds
        return (min(value - 1, 0), max(value + 1, 2))

    def _changed(self, name, value):
        spin, slider, lock, lo, hi = self.controls[name]
        lower, upper = lo.value(), hi.value()
        if lower < upper:
            slider.blockSignals(True)
            slider.setValue(round(max(0, min(10000, (value-lower)/(upper-lower)*10000))))
            slider.blockSignals(False)
        self.parameterChanged.emit(name, float(value))

    def _from_slider(self, name, v):
        spin, _, _, lo, hi = self.controls[name]
        value = lo.value() + v/10000*(hi.value()-lo.value())
        blocker = QSignalBlocker(spin)
        spin.setValue(value)
        del blocker
        self.parameterChanged.emit(name, float(value))

    def _bounds_changed(self, name):
        spin, slider, _, lo, hi = self.controls[name]
        if hi.value() > lo.value():
            blocker = QSignalBlocker(slider)
            slider.setValue(round((spin.value()-lo.value())/(hi.value()-lo.value())*10000))
            del blocker
        self.boundsChanged.emit(self.bounds())

    def _locks(self, *_):
        self.locksChanged.emit(self.locked())

    def locked(self):
        return frozenset(n for n, c in self.controls.items() if c[2].isChecked())

    def bounds(self):
        return {n: (c[3].value(), c[4].value()) for n, c in self.controls.items()}
