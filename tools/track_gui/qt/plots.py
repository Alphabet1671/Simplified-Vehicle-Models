"""Two plain QPainter plots: the course in the XY plane and curvature against distance."""
from __future__ import annotations
import math
import numpy as np
from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import QColor, QPainter, QPainterPath, QPen
from PyQt6.QtWidgets import QWidget

MARGIN = 36
GRID = QColor(210, 210, 210)
LINE = QColor(20, 90, 170)
MARK = QColor(200, 60, 30)

def nice_step(span: float, target: int = 5) -> float:
    if not math.isfinite(span) or span <= 0.0: return 1.0
    raw = span / target
    power = 10.0 ** math.floor(math.log10(raw))
    for factor in (1.0, 2.0, 5.0, 10.0):
        if raw <= factor * power: return factor * power
    return 10.0 * power

class _Plot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent); self.setMinimumSize(240, 200); self.message = "no course"
    def set_message(self, text): self.message = text; self.update()
    def _frame(self, painter):
        painter.fillRect(self.rect(), Qt.GlobalColor.white)
        return QRectF(MARGIN, 8, max(self.width()-MARGIN-8,10), max(self.height()-MARGIN-8,10))
    def _empty(self, painter):
        painter.setPen(QColor(120,120,120)); painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, self.message)

class CoursePlot(_Plot):
    def __init__(self, parent=None):
        super().__init__(parent); self.north=np.zeros(0); self.east=np.zeros(0); self.boundary_points=[]
    def set_course(self, north, east, boundary_points=()):
        self.north,self.east=np.asarray(north,float),np.asarray(east,float); self.boundary_points=list(boundary_points)
        self.message="no course" if self.north.size==0 else ""; self.update()
    def transform(self, rect):
        e0,e1=float(self.east.min()),float(self.east.max()); n0,n1=float(self.north.min()),float(self.north.max())
        span_e,span_n=max(e1-e0,1e-6),max(n1-n0,1e-6); scale=min(rect.width()/span_e,rect.height()/span_n)
        left=rect.left()+(rect.width()-span_e*scale)/2; bottom=rect.bottom()-(rect.height()-span_n*scale)/2
        return lambda n,e: QPointF(left+(e-e0)*scale,bottom-(n-n0)*scale),scale
    def paintEvent(self, _event):
        painter=QPainter(self); painter.setRenderHint(QPainter.RenderHint.Antialiasing); rect=self._frame(painter)
        if self.north.size<2: self._empty(painter); return
        to_point,scale=self.transform(rect); painter.setPen(QPen(GRID,1)); step=nice_step(min(rect.width(),rect.height())/scale)
        n_min,e_min=float(self.north.min()),float(self.east.min()); n=math.floor(n_min/step)*step
        while to_point(n,e_min).y()>rect.top()-1:
            y=to_point(n,e_min).y()
            if rect.top()<=y<=rect.bottom(): painter.drawLine(QPointF(rect.left(),y),QPointF(rect.right(),y)); painter.drawText(QPointF(2,y+4),f"{n:g}")
            n+=step
        e=math.floor(e_min/step)*step
        while to_point(n_min,e).x()<rect.right()+1:
            x=to_point(n_min,e).x()
            if rect.left()<=x<=rect.right(): painter.drawLine(QPointF(x,rect.top()),QPointF(x,rect.bottom())); painter.drawText(QPointF(x-8,rect.bottom()+14),f"{e:g}")
            e+=step
        path=QPainterPath(to_point(self.north[0],self.east[0]))
        for n_value,e_value in zip(self.north[1:],self.east[1:]): path.lineTo(to_point(n_value,e_value))
        painter.setPen(QPen(LINE,2)); painter.drawPath(path); painter.setPen(QPen(MARK,1)); painter.setBrush(MARK)
        for n_value,e_value in self.boundary_points: painter.drawEllipse(to_point(n_value,e_value),3,3)
        painter.setBrush(Qt.GlobalColor.darkGreen); painter.drawEllipse(to_point(self.north[0],self.east[0]),5,5)
        painter.setPen(QColor(90,90,90)); painter.drawText(QPointF(rect.left()+4,rect.top()+12),"east →, north ↑ (m)")

class CurvaturePlot(_Plot):
    def __init__(self,parent=None):
        super().__init__(parent); self.distance=np.zeros(0); self.curvature=np.zeros(0); self.boundaries=[]
    def set_course(self,distance,curvature,boundaries=()):
        self.distance,self.curvature=np.asarray(distance,float),np.asarray(curvature,float); self.boundaries=[float(b) for b in boundaries]
        self.message="no course" if self.distance.size==0 else ""; self.update()
    def limits(self):
        peak=max(float(np.max(np.abs(self.curvature))),1e-3)
        return float(self.distance[0]),float(self.distance[-1]),-1.1*peak,1.1*peak
    def paintEvent(self,_event):
        painter=QPainter(self); painter.setRenderHint(QPainter.RenderHint.Antialiasing); rect=self._frame(painter)
        if self.distance.size<2: self._empty(painter); return
        x0,x1,y0,y1=self.limits()
        def to_point(x,y): return QPointF(rect.left()+(x-x0)/max(x1-x0,1e-9)*rect.width(),rect.bottom()-(y-y0)/(y1-y0)*rect.height())
        painter.setPen(QPen(GRID,1)); zero=to_point(x0,0).y(); painter.drawLine(QPointF(rect.left(),zero),QPointF(rect.right(),zero))
        for boundary in self.boundaries:
            x=to_point(boundary,0).x(); painter.drawLine(QPointF(x,rect.top()),QPointF(x,rect.bottom()))
        step=nice_step(x1-x0); x=math.ceil(x0/step)*step
        while x<=x1: painter.drawText(QPointF(to_point(x,0).x()-8,rect.bottom()+14),f"{x:g}"); x+=step
        painter.drawText(QPointF(2,rect.top()+10),f"{y1:.3g}"); painter.drawText(QPointF(2,rect.bottom()),f"{y0:.3g}")
        path=QPainterPath(to_point(self.distance[0],self.curvature[0]))
        for d,c in zip(self.distance[1:],self.curvature[1:]): path.lineTo(to_point(d,c))
        painter.setPen(QPen(LINE,2)); painter.drawPath(path); painter.setPen(QColor(90,90,90)); painter.drawText(QPointF(rect.left()+4,rect.top()+12),"curvature (1/m) against distance (m)")
