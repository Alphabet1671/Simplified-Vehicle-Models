"""Aligned requested/applied command trace preview."""
from copy import deepcopy
import math

from PyQt6.QtCore import Qt, QRectF
from PyQt6.QtGui import QPainter, QPen, QColor
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel


class PreviewWidget(QWidget):
    COLORS = {"requested": "#1769aa", "applied": "#d35400"}
    NAMES = ("Steering (rad)", "Accelerator", "Brake")

    def __init__(self, parent=None):
        super().__init__(parent)
        self.requested = self.applied = None
        self.time_s = []
        self.requested_commands = self.applied_commands = None
        self.requested_label = QLabel("Requested: —", self)
        self.applied_label = QLabel("Applied: —", self)
        self.requested_label.setMinimumHeight(22); self.applied_label.setMinimumHeight(22)
        self.setMinimumSize(420, 250)
        layout = QVBoxLayout(self); layout.addWidget(self.requested_label); layout.addWidget(self.applied_label)

    @staticmethod
    def _value(command, name):
        name = {"Steering (rad)": "steering", "Accelerator": "accelerator", "Brake": "brake"}.get(name, name)
        if isinstance(command, dict):
            if name in command: return float(command[name])
            if name == "steering":
                steering = command.get("steering", {})
                return float(steering.get("swa_rad", steering.get("amplitude_rad", 0))) if isinstance(steering, dict) else float(steering)
            return float(command.get(name, 0))
        aliases = {"steering": ("swa_rad", "steering"), "accelerator": ("accelerator",), "brake": ("brake",)}
        for attr in aliases[name]:
            if hasattr(command, attr): return float(getattr(command, attr))
        return 0.0

    def set_trace(self, time_s, requested_commands, applied_commands=None):
        self.time_s = deepcopy(list(time_s)); self.requested_commands = deepcopy(list(requested_commands))
        self.applied_commands = deepcopy(list(applied_commands)) if applied_commands is not None else None
        self.requested, self.applied = self.requested_commands, self.applied_commands
        self.requested_label.setText("Requested trace (blue): " + self._summary(self.requested_commands))
        self.applied_label.setText("Applied trace (orange): " + (self._summary(self.applied_commands) if self.applied_commands is not None else "—"))
        self.update()

    def _summary(self, commands):
        return f"{len(commands)} samples, {self.time_s[0]:g}–{self.time_s[-1]:g} s" if commands and self.time_s else "—"

    def set_state(self, requested, applied=None):
        self.requested = deepcopy(requested); self.applied = deepcopy(applied)
        self.requested_label.setText(f"Requested: {self.requested}")
        self.applied_label.setText(f"Applied: {self.applied if self.applied is not None else '—'}")
        self.update()

    def _series(self, commands, name):
        if commands is None: return []
        result=[]
        for c in commands:
            try: result.append(self._value(c, name))
            except (TypeError, ValueError): result.append(float("nan"))
        return result

    def paintEvent(self, event):
        super().paintEvent(event)
        if not self.time_s or not self.requested_commands: return
        n=min(len(self.time_s), len(self.requested_commands))
        if n < 2: return
        painter=QPainter(self); painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        top=58; bottom=self.height()-8; left=70; right=self.width()-12
        pane_h=max(30,(bottom-top)/3); t0=self.time_s[0]; t1=self.time_s[-1]
        if not math.isfinite(t1-t0) or t1<=t0: return
        for pane,name in enumerate(self.NAMES):
            rect=QRectF(left,top+pane*pane_h,right-left,pane_h-8)
            painter.setPen(QPen(Qt.GlobalColor.lightGray)); painter.drawRect(rect)
            painter.setPen(QPen(Qt.GlobalColor.black)); painter.drawText(4,int(rect.top()+15),name)
            vals=[]
            for commands in (self.requested_commands,self.applied_commands):
                values=self._series(commands,name)
                vals.extend(v for v in values if math.isfinite(v))
            if not vals: continue
            lo=min(vals); hi=max(vals); span=hi-lo or 1.0; lo-=span*.08; hi+=span*.08
            for label,commands in (("requested",self.requested_commands),("applied",self.applied_commands)):
                if commands is None: continue
                series=self._series(commands,name); count=min(n,len(series)); pen=QPen(Qt.GlobalColor.blue if label=="requested" else Qt.GlobalColor.darkYellow,2)
                pen.setColor(QColor(self.COLORS[label])); painter.setPen(pen)
                points=[]
                for i in range(count):
                    x=rect.left()+(self.time_s[i]-t0)/(t1-t0)*rect.width()
                    y=rect.bottom()-(series[i]-lo)/(hi-lo)*rect.height()
                    points.append((x,y))
                for i in range(1,len(points)):
                    # Hold samples are explicitly shown as discontinuous steps.
                    x0,y0=points[i-1]; x1,y1=points[i]
                    if self._hold_signal(commands):
                        xm=x1; painter.drawLine(int(x0),int(y0),int(xm),int(y0)); painter.drawLine(int(xm),int(y0),int(x1),int(y1))
                    else: painter.drawLine(int(x0),int(y0),int(x1),int(y1))
        painter.setPen(QPen(QColor(self.COLORS["requested"]))); painter.drawText(left,18,"Requested")
        painter.setPen(QPen(QColor(self.COLORS["applied"]))); painter.drawText(left+110,18,"Applied")
        painter.setPen(QPen(Qt.GlobalColor.darkGray)); painter.drawText(left,38,"Time (s) — shared scale")
        painter.end()

    @staticmethod
    def _hold_signal(commands):
        return bool(commands) and isinstance(commands[0], dict) and commands[0].get("interpolation") == "hold"
