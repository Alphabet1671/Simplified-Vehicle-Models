"""Existing track viewer with safe reverse and closed-section export."""
from __future__ import annotations
from pathlib import Path
import numpy as np
from PyQt6.QtWidgets import (QWidget,QVBoxLayout,QHBoxLayout,QPushButton,QLabel,QFileDialog,QMessageBox,
    QFormLayout,QDoubleSpinBox,QSpinBox,QTableWidget,QTableWidgetItem)
from tools.track_gui.track_ops import load_track_data, reverse_track, save_track_data, save_sections
from tools.track_gui.export import SaveRefused
from tools.gui_common.files import generated_by
from tools.track_gui.qt.plots import CoursePlot, CurvaturePlot
from tools.track_gui.qt.main_window import SaveBar

class TrackViewer(QWidget):
    def __init__(self,parent=None):
        super().__init__(parent); self.track=None; self.source_path=None; self.cuts=[]
        self.map=CoursePlot(); self.profile=CurvaturePlot(); self.notice=QLabel("Load a track file or transfer a track from another tab."); self.notice.setWordWrap(True)
        self.load_button=QPushButton("Load track…"); self.load_button.clicked.connect(self.choose_load)
        self.reverse_button=QPushButton("Reverse direction"); self.reverse_button.clicked.connect(self.reverse)
        self.split_button=QPushButton("Export sections"); self.split_button.clicked.connect(self.export_sections)
        self.save_button=QPushButton("Save track"); self.save_button.clicked.connect(self.save)
        self.save_bar=SaveBar("viewed_track")
        self.cut_table=QTableWidget(0,1); self.cut_table.setHorizontalHeaderLabels(["Cut distance (m)"]); self.cut_table.itemChanged.connect(self._edited_cuts)
        self.equal_count=QSpinBox(); self.equal_count.setRange(2,100); self.equal_button=QPushButton("Equal parts")
        self.equal_button.clicked.connect(self.equal_cuts); self.add_cut=QDoubleSpinBox(); self.add_cut.setRange(0,1e9); self.add_cut.setDecimals(3)
        self.add_button=QPushButton("Add cut"); self.add_button.clicked.connect(lambda:self.add_distance_cut(self.add_cut.value()))
        self.remove_button=QPushButton("Remove selected cut"); self.remove_button.clicked.connect(self.remove_selected_cut)
        row=QHBoxLayout()
        for w in (self.load_button,self.reverse_button,self.save_button,self.split_button): row.addWidget(w)
        cutrow=QHBoxLayout(); cutrow.addWidget(self.equal_count); cutrow.addWidget(self.equal_button); cutrow.addWidget(self.add_cut); cutrow.addWidget(self.add_button); cutrow.addWidget(self.remove_button)
        layout=QVBoxLayout(self); layout.addWidget(self.notice); layout.addLayout(row); layout.addWidget(self.save_bar); layout.addLayout(cutrow); layout.addWidget(self.cut_table); plots=QHBoxLayout(); plots.addWidget(self.map); plots.addWidget(self.profile); layout.addLayout(plots)
        self.map.nodeClicked.connect(self.cut_node)
        self._update_enabled()
    def set_track(self,track,title=None):
        self.track=track; self.cuts=[]; self.source_path=None; self.cut_table.setRowCount(0)
        if title and track.title!=title:
            from dataclasses import replace
            self.track=replace(track,title=title)
        self.render()
    def load_path(self,path):
        try: track=load_track_data(path)
        except (OSError,ValueError) as exc:
            self.track=None; self.source_path=None; self.cuts=[]; self.cut_table.setRowCount(0); self.notice.setText(f"Could not load track: {exc}"); self.render(); return False
        self.track=track; self.source_path=Path(path); self.cuts=[]; self.cut_table.setRowCount(0); self.render(); return True
    def choose_load(self):
        path,_=QFileDialog.getOpenFileName(self,"Load track", "", "Track files (*.xml)")
        if path:self.load_path(path)
    def _update_enabled(self):
        valid=self.track is not None; signed=valid and self.track.x_north is not None
        self.reverse_button.setEnabled(valid); self.save_button.setEnabled(valid); self.split_button.setEnabled(bool(signed and self.track.is_closed)); self.equal_button.setEnabled(bool(signed and self.track.is_closed)); self.add_button.setEnabled(bool(signed and self.track.is_closed)); self.remove_button.setEnabled(bool(signed and self.track.is_closed and self.cuts))
    def render(self):
        if self.track is None:
            self.map.set_course([],[]); self.profile.set_course([],[]); self._update_enabled(); return
        t=self.track; marks=[]
        if t.x_north is None:
            self.notice.setText("Unsigned legacy geometry: no recoverable XY map; map and section export are unavailable.")
        else:
            self.notice.setText(("Closed" if t.is_closed else "Open")+f" track — {t.title}. Green dot is start; arrow shows running direction.")
            marks=list(zip(t.x_north[np.clip(np.searchsorted(t.distance,self.cuts),0,len(t.distance)-1)],t.y_east[np.clip(np.searchsorted(t.distance,self.cuts),0,len(t.distance)-1)]))
        self.map.set_course(t.x_north if t.x_north is not None else [],t.y_east if t.y_east is not None else [],marks)
        self.profile.set_course(t.distance,t.curvature,self.cuts); self._update_enabled()
    def reverse(self):
        if self.track is not None:
            self.track=reverse_track(self.track); self.cuts=sorted(self.track.length-c for c in self.cuts); self._refresh_cuts()
    def add_distance_cut(self,d):
        if self.track is None:return
        if not np.isfinite(d) or not 0<=d<=self.track.length: self.notice.setText("Cut must be within the track length."); return
        d=0. if np.isclose(d,self.track.length,atol=1e-10) else float(d)
        if any(np.isclose(d,c,atol=1e-8) for c in self.cuts): self.notice.setText("That cut is already present."); return
        self.cuts=sorted([*self.cuts,d]); self._refresh_cuts()
    def _edited_cuts(self,item):
        if self.track is None:return
        try:
            vals=[float(self.cut_table.item(r,0).text()) for r in range(self.cut_table.rowCount())]
            if len(set(vals))!=len(vals) or any(not np.isfinite(v) or not 0<=v<=self.track.length for v in vals):raise ValueError
        except (ValueError,AttributeError):
            self.notice.setText("Cuts must be finite, distinct distances within the track."); return
        self.cuts=sorted(vals); self._refresh_cuts()
    def _refresh_cuts(self):
        self.cut_table.blockSignals(True)
        self.cut_table.setRowCount(len(self.cuts))
        for i,d in enumerate(sorted(self.cuts)): self.cut_table.setItem(i,0,QTableWidgetItem(f"{d:.6f}"))
        self.cut_table.blockSignals(False)
        if self.track:self.profile.set_course(self.track.distance,self.track.curvature,self.cuts); self.render()
    def equal_cuts(self):
        if self.track:
            self.cuts=[self.track.length*i/self.equal_count.value() for i in range(self.equal_count.value())]; self._refresh_cuts()
    def cut_node(self,index):
        if self.track and self.track.x_north is not None:self.add_distance_cut(float(self.track.distance[index]))
    def remove_selected_cut(self):
        row=self.cut_table.currentRow()
        if row>=0 and row<len(self.cuts): self.cuts.pop(row); self._refresh_cuts()
    def _table_cuts(self):
        values=[]
        try:
            for row in range(self.cut_table.rowCount()): values.append(float(self.cut_table.item(row,0).text()))
            if len(set(values))!=len(values) or any(not np.isfinite(x) or x<0 or x>self.track.length for x in values):raise ValueError
            return sorted(values)
        except (ValueError,AttributeError): raise ValueError("Cut distances must be finite, distinct values within the track length.")
    def save(self):
        if self.track is None:return
        v=self.save_bar.values()
        try: saved=save_track_data(self.track,**v)
        except SaveRefused as e:
            message=str(e)
            if "models/" in message and not v["allow_models"]:
                QMessageBox.warning(self,"Protected folder",message+"\nEnable 'Allow writing inside models/' and retry.")
                return
            if ("already exists" in message or "generated by" in message) and not v["overwrite"]:
                QMessageBox.warning(self,"Not saved",message+"\nEnable overwrite and retry.")
                return
            if QMessageBox.question(self,"Confirm replacement",message+"\n\nContinue with this explicitly enabled overwrite?")!=QMessageBox.StandardButton.Yes:return
            retry=dict(v,overwrite=True,overwrite_generated=True)
            try: saved=save_track_data(self.track,**retry)
            except Exception as err: QMessageBox.warning(self,"Not saved",str(err)); return
        except Exception as e: QMessageBox.warning(self,"Not saved",str(e)); return
        self.notice.setText(f"Saved {saved.xml}")
    def export_sections(self):
        if self.track is None:return
        try: cuts=self._table_cuts()
        except ValueError as e: QMessageBox.warning(self,"Invalid cuts",str(e)); return
        if len(cuts)<2: QMessageBox.warning(self,"Invalid cuts","Add at least two distinct cuts (include the seam as zero or track length)."); return
        v=self.save_bar.values(); targets=[Path(v['directory'])/f"{v['stem']}_{i:02d}.{ext}" for i in range(1,len(cuts)+1) for ext in ("xml","csv")]
        existing=[p for p in targets if p.exists()]; overwrite=bool(v.pop("overwrite",False)); generated=False
        if v["allow_models"] and any("models" in p.parts for p in targets):
            if QMessageBox.question(self,"Write inside models/","Confirm writing section files inside models/?")!=QMessageBox.StandardButton.Yes:return
        existing_generated=[p for p in existing if p.suffix==".xml" and generated_by(p)]
        if existing:
            if not overwrite:
                QMessageBox.warning(self,"Not saved","Section targets already exist; enable overwrite and confirm before export."); return
            if QMessageBox.question(self,"Replace existing sections?",f"Replace {len(existing)} existing file(s)?")!=QMessageBox.StandardButton.Yes:return
            if existing_generated:
                if QMessageBox.question(self,"Generated files", "Replace generated track files after this confirmation?")!=QMessageBox.StandardButton.Yes:return
                generated=True
        try: saved=save_sections(self.track,cuts,**v,overwrite=overwrite,overwrite_generated=generated)
        except Exception as e: QMessageBox.warning(self,"Sections not saved",str(e)); return
        self.notice.setText(f"Exported {len(saved)} sections.")
