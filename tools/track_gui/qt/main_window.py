"""The track GUI main window: a design tab and a closed-track tab."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from PyQt6.QtCore import QSettings, Qt, QThread, QObject, pyqtSignal
from PyQt6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGroupBox,
                             QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMessageBox, QPushButton,
                             QSplitter, QTableView, QTabWidget, QVBoxLayout, QWidget, QPlainTextEdit, QScrollArea)

from tools.gui_common import REPOSITORY_ROOT
from tools.gui_common.files import protected_directory_problem
from tools.track_gui import closed
from tools.track_gui.design import KINDS, PRESETS, CourseDesign
from tools.track_gui.export import SaveRefused, describe_course, save_open_course
from tools.track_gui.qt.plots import CoursePlot, CurvaturePlot
from tools.track_gui.qt.section_table import SectionTableModel
from tools.track_gui.mat_channels import ChannelSelection, list_channels

DEFAULT_OUTPUT = REPOSITORY_ROOT / "tracks_out"


def _spin(low, high, value, decimals=3, step=0.1):
    box = QDoubleSpinBox()
    box.setRange(low, high)
    box.setDecimals(decimals)
    box.setSingleStep(step)
    box.setValue(value)
    return box


class SaveBar(QWidget):
    """Output folder, file name, id and title, shared by both tabs."""

    def __init__(self, stem: str, parent=None) -> None:
        super().__init__(parent)
        self.folder = QLineEdit(str(DEFAULT_OUTPUT))
        browse = QPushButton("Browse…")
        browse.clicked.connect(self._browse)
        self.stem = QLineEdit(stem)
        self.model_id = QLineEdit(stem.replace("_", "-"))
        self.title = QLineEdit(stem.replace("_", " ").title())
        self.overwrite = QCheckBox("Overwrite existing files")
        self.allow_models = QCheckBox("Allow writing inside models/")
        form = QFormLayout(self)
        row = QHBoxLayout()
        row.addWidget(self.folder)
        row.addWidget(browse)
        form.addRow("Folder", row)
        form.addRow("File name", self.stem)
        form.addRow("Model id", self.model_id)
        form.addRow("Title", self.title)
        form.addRow(self.overwrite)
        form.addRow(self.allow_models)

    def _browse(self) -> None:
        chosen = QFileDialog.getExistingDirectory(self, "Output folder", self.folder.text())
        if chosen:
            self.folder.setText(chosen)

    def values(self) -> dict:
        return dict(directory=Path(self.folder.text()).expanduser(), stem=self.stem.text().strip(),
                    model_id=self.model_id.text().strip(), title=self.title.text().strip(),
                    allow_models=self.allow_models.isChecked(), overwrite=self.overwrite.isChecked())


class DesignTab(QWidget):
    status = pyqtSignal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.table = SectionTableModel()
        self.view = QTableView()
        self.view.setModel(self.table)
        self.view.setSelectionBehavior(QTableView.SelectionBehavior.SelectRows)
        self.preset = QComboBox()
        self.preset.addItems(list(PRESETS))
        load_preset = QPushButton("Load preset")
        load_preset.clicked.connect(self.load_preset)
        buttons = {}
        for label, slot in (("Add", self.add_row), ("Delete", self.delete_row), ("Up", lambda: self.move_row(-1)),
                            ("Down", lambda: self.move_row(1))):
            button = QPushButton(label)
            button.clicked.connect(slot)
            buttons[label] = button
        self.kind = QComboBox()
        self.kind.addItems(KINDS)
        self.spacing = _spin(0.01, 100.0, 0.25, 3, 0.05)
        self.heading = _spin(-360.0, 360.0, 0.0, 3, 1.0)
        self.north = _spin(-1e6, 1e6, 0.0, 3, 1.0)
        self.east = _spin(-1e6, 1e6, 0.0, 3, 1.0)
        self.jumps = QCheckBox("Allow curvature jumps")
        self.ramp = _spin(0.001, 10.0, 0.05, 3, 0.01)
        for widget in (self.spacing, self.heading, self.north, self.east, self.ramp):
            widget.valueChanged.connect(self.refresh)
        self.jumps.toggled.connect(self.refresh)
        self.table.changed.connect(self.refresh)
        self.course_plot, self.curvature_plot = CoursePlot(), CurvaturePlot()
        self.info = QLabel()
        self.problem = QLabel()
        self.problem.setWordWrap(True)
        self.problem.setStyleSheet("color: #b00020")
        self.save_bar = SaveBar("my_track")
        self.save_button = QPushButton("Save track")
        self.save_button.clicked.connect(self.save)
        self.open_button = QPushButton("Open design…")
        self.open_button.clicked.connect(self.open_design)
        self.store_button = QPushButton("Save design…")
        self.store_button.clicked.connect(self.store_design)

        left = QVBoxLayout()
        top = QHBoxLayout()
        top.addWidget(self.preset)
        top.addWidget(load_preset)
        top.addStretch()
        left.addLayout(top)
        left.addWidget(self.view, 1)
        row = QHBoxLayout()
        row.addWidget(self.kind)
        for button in buttons.values():
            row.addWidget(button)
        left.addLayout(row)
        settings = QGroupBox("Build settings")
        form = QFormLayout(settings)
        form.addRow("Node spacing (m)", self.spacing)
        form.addRow("Start heading (deg)", self.heading)
        form.addRow("Start north (m)", self.north)
        form.addRow("Start east (m)", self.east)
        form.addRow(self.jumps)
        form.addRow("Jump ramp (m)", self.ramp)
        left.addWidget(settings)
        left_widget = QWidget()
        left_widget.setLayout(left)

        right = QVBoxLayout()
        right.addWidget(self.course_plot, 2)
        right.addWidget(self.curvature_plot, 1)
        right.addWidget(self.info)
        right.addWidget(self.problem)
        right.addWidget(self.save_bar)
        row = QHBoxLayout()
        row.addWidget(self.open_button)
        row.addWidget(self.store_button)
        row.addWidget(self.save_button)
        right.addLayout(row)
        right_widget = QWidget()
        right_widget.setLayout(right)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        layout = QVBoxLayout(self)
        layout.addWidget(splitter)
        self.course = None
        self.load_preset()

    def design(self) -> CourseDesign:
        return CourseDesign(self.table.sections(), self.spacing.value(), math.radians(self.heading.value()),
                            self.north.value(), self.east.value(), self.jumps.isChecked(), self.ramp.value())

    def set_design(self, design: CourseDesign) -> None:
        for widget in (self.spacing, self.heading, self.north, self.east, self.ramp, self.jumps):
            widget.blockSignals(True)
        self.spacing.setValue(design.maximum_spacing)
        self.heading.setValue(math.degrees(design.start_heading))
        self.north.setValue(design.start_north)
        self.east.setValue(design.start_east)
        self.jumps.setChecked(design.allow_curvature_jumps)
        self.ramp.setValue(design.jump_ramp_length)
        for widget in (self.spacing, self.heading, self.north, self.east, self.ramp, self.jumps):
            widget.blockSignals(False)
        self.table.set_sections(design.sections)

    def load_preset(self) -> None:
        self.set_design(PRESETS[self.preset.currentText()]())

    def selected_row(self) -> int | None:
        rows = self.view.selectionModel().selectedRows()
        return rows[0].row() if rows else None

    def add_row(self) -> None:
        self.table.add(self.kind.currentText(), self.selected_row())

    def delete_row(self) -> None:
        row = self.selected_row()
        if row is not None:
            self.table.remove(row)

    def move_row(self, offset: int) -> None:
        row = self.selected_row()
        if row is not None:
            new = self.table.move(row, offset)
            self.view.selectRow(new)

    def refresh(self) -> None:
        try:
            self.course = self.design().build()
        except ValueError as error:
            self.course = None
            self.problem.setText(str(error))
            self.info.setText("")
            self.course_plot.set_course([], [])
            self.curvature_plot.set_course([], [])
            self.save_button.setEnabled(False)
            if hasattr(self,"transfer_button"): self.transfer_button.setEnabled(False)
            return
        figures = describe_course(self.course)
        self.problem.setText("")
        self.info.setText(f"{figures['length_m']:.1f} m, {figures['nodes']} nodes, minimum radius "
                          f"{figures['minimum_radius_m']:.2f} m, heading change {figures['heading_change_deg']:.1f}°, "
                          f"{figures['jumps']} curvature jump(s)")
        # ``boundaries`` are distances in metres (one more than the sections, from 0 to the length).
        course = self.course
        marks = [(float(np.interp(b, course.distance, course.x_north)),
                  float(np.interp(b, course.distance, course.y_east))) for b in course.boundaries]
        self.course_plot.set_course(course.x_north, course.y_east, marks)
        self.curvature_plot.set_course(course.distance, course.curvature, course.boundaries)
        self.save_button.setEnabled(True)
        if hasattr(self,"transfer_button"): self.transfer_button.setEnabled(True)

    def save(self) -> None:
        if self.course is None:
            return
        values = self.save_bar.values()
        self.status.emit(self._attempt(values, False))

    def _attempt(self, values: dict, generated_ok: bool) -> str:
        try:
            saved = save_open_course(self.course, **values, overwrite_generated=generated_ok)
        except SaveRefused as refusal:
            text = str(refusal)
            if "generated by" in text and not generated_ok:
                if QMessageBox.question(self, "Replace a generated file?", text + "\n\nReplace it?") == QMessageBox.StandardButton.Yes:
                    return self._attempt(values, True)
            elif "models/" in text and not values["allow_models"]:
                QMessageBox.warning(self, "Protected folder", text + "\nTick 'Allow writing inside models/' to confirm.")
            else:
                QMessageBox.warning(self, "Not saved", text)
            return "not saved"
        except (OSError, ValueError) as error:
            QMessageBox.warning(self, "Not saved", str(error))
            return "not saved"
        return f"saved {saved.xml} ({saved.length:.1f} m, {saved.node_count} nodes)"

    def open_design(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Open design", str(DEFAULT_OUTPUT), "Track design (*.json)")
        if path:
            try:
                self.set_design(CourseDesign.from_json(Path(path).read_text(encoding="utf-8")))
            except (OSError, ValueError) as error:
                QMessageBox.warning(self, "Not opened", str(error))

    def store_design(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Save design", str(DEFAULT_OUTPUT / "design.json"), "Track design (*.json)")
        if path:
            target = Path(path)
            problem = protected_directory_problem(target.parent, allow_models=False)
            if problem:
                if not self.save_bar.allow_models.isChecked():
                    QMessageBox.warning(self, "Protected folder", problem + "\nTick 'Allow writing inside models/' to confirm.")
                    return
            if target.exists():
                answer = QMessageBox.question(self, "Replace existing design?", f"{target} already exists. Replace it?")
                if answer != QMessageBox.StandardButton.Yes:
                    return
            try:
                from tools.gui_common.files import atomic_write_text
                atomic_write_text(target, self.design().to_json())
            except OSError as error:
                QMessageBox.warning(self, "Not saved", str(error))


class _BuildWorker(QObject):
    done = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(self, settings) -> None:
        super().__init__()
        self.settings = settings

    def run(self) -> None:
        try:
            self.done.emit(closed.build_closed(self.settings))
        except Exception as error:  # the worker must never let an exception end the thread silently
            self.failed.emit(str(error))


class ClosedTab(QWidget):
    status = pyqtSignal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.geometry = None
        self.settings = None
        self.thread: QThread | None = None
        self.telemetry = QLineEdit()
        browse = QPushButton("Browse…")
        browse.clicked.connect(self._browse)
        self.start = _spin(0.0, 1e6, 0.0, 2, 1.0)
        self.end = _spin(0.0, 1e6, 60.0, 2, 1.0)
        self.spacing = _spin(0.05, 10.0, 0.5, 2, 0.1)
        self.smoothing = _spin(0.0, 50.0, 2.0, 2, 0.5)
        self.max_radius = _spin(1.0, 1e5, 300.0, 1, 10.0)
        self.x_channel, self.y_channel = QComboBox(), QComboBox()
        self.mode = QComboBox(); self.mode.addItems(["velocity", "position", "gps"])
        self.x_scale, self.y_scale = _spin(-1e6,1e6,1.0,6,0.1), _spin(-1e6,1e6,1.0,6,0.1)
        self.x_unknown, self.y_unknown = QCheckBox("Allow unknown x units"), QCheckBox("Allow unknown y units")
        self.channel_info = QLabel("Legacy defaults: VNorth / VEast; use Discover to inspect MAT metadata.")
        self.channel_info.setWordWrap(True)
        self.discover_button = QPushButton("Discover channels")
        self.discover_button.clicked.connect(self.discover_channels)
        self.multi_lap = QCheckBox("Detect and average multiple laps")
        self.return_radius = _spin(.1,1e4,12.,2,1.)
        self.minimum_travel = _spin(1,1e6,100.,1,10.)
        self.minimum_duration = _spin(.1,1e5,15.,1,1.)
        self.maximum_duration = _spin(.2,1e5,240.,1,5.)
        self.deviation_limit = _spin(.01,1e4,8.,2,1.)
        self.build_button = QPushButton("Build")
        self.build_button.clicked.connect(self.build)
        self.report = QPlainTextEdit()
        self.report.setReadOnly(True)
        self.notice = QLabel()
        self.notice.setWordWrap(True)
        self.course_plot = CoursePlot()
        self.save_bar = SaveBar("my_closed_track")
        self.save_button = QPushButton("Save track")
        self.save_button.setEnabled(False)
        self.transfer_button=QPushButton("Send to viewer"); self.transfer_button.setEnabled(False)
        self.save_button.clicked.connect(self.save)
        form = QFormLayout()
        row = QHBoxLayout()
        row.addWidget(self.telemetry)
        row.addWidget(browse)
        form.addRow("Telemetry (.mat)", row)
        form.addRow("Lap start (s)", self.start)
        form.addRow("Lap end (s)", self.end)
        form.addRow("Node spacing (m)", self.spacing)
        form.addRow("Smoothing length (m)", self.smoothing)
        form.addRow("Maximum radius (m)", self.max_radius)
        form.addRow(self.discover_button)
        form.addRow("North/x channel", self.x_channel); form.addRow("East/y channel", self.y_channel)
        form.addRow("Coordinate mode", self.mode); form.addRow("x scale", self.x_scale); form.addRow("y scale", self.y_scale)
        form.addRow(self.x_unknown); form.addRow(self.y_unknown); form.addRow(self.channel_info)
        form.addRow(self.multi_lap); form.addRow("Return radius (m)", self.return_radius)
        form.addRow("Minimum travel (m)", self.minimum_travel); form.addRow("Minimum lap (s)", self.minimum_duration)
        form.addRow("Maximum lap (s)", self.maximum_duration); form.addRow("Deviation limit (m)", self.deviation_limit)
        left = QVBoxLayout()
        left.addLayout(form)
        left.addWidget(self.build_button)
        left.addWidget(self.notice)
        left.addWidget(self.report, 1)
        left.addWidget(self.save_bar)
        left.addWidget(self.save_button)
        left.addWidget(self.transfer_button)
        self.transfer_button.clicked.connect(self.transfer_to_viewer)
        layout = QHBoxLayout(self)
        left_widget = QWidget()
        left_widget.setLayout(left)
        scroll=QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(left_widget); scroll.setMinimumWidth(390)
        layout.addWidget(scroll, 1)
        self.course_plot.setMinimumSize(400,360)
        layout.addWidget(self.course_plot, 1)
        self._build_controls=[self.telemetry,self.start,self.end,self.spacing,self.smoothing,self.max_radius,self.x_channel,self.y_channel,self.mode,self.x_scale,self.y_scale,self.x_unknown,self.y_unknown,self.multi_lap,self.return_radius,self.minimum_travel,self.minimum_duration,self.maximum_duration,self.deviation_limit]
        for control in self._build_controls:
            signal=getattr(control,"textChanged",None) or getattr(control,"valueChanged",None) or getattr(control,"currentIndexChanged",None) or getattr(control,"toggled",None)
            if signal is not None: signal.connect(self._invalidate_result)
        if not closed.scipy_available():
            self.build_button.setEnabled(False)
            self.notice.setText("SciPy is not installed, so telemetry tracks cannot be built. "
                                "Install it with: pip install -e .[forward]")

    def _invalidate_result(self,*_):
        if self.geometry is not None:
            self.geometry=None; self.save_button.setEnabled(False); self.transfer_button.setEnabled(False)
            self.course_plot.set_overlaps([],[]); self.course_plot.set_course([],[])
            self.report.clear(); self.notice.setText("Build settings changed. Rebuild before saving or transfer.")

    def _browse(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Telemetry", "", "MATLAB files (*.mat)")
        if path:
            self.telemetry.setText(path)

    def set_channels(self,channels):
        for combo in (self.x_channel,self.y_channel): combo.clear()
        for channel in channels:
            label=f"{channel.name} — {channel.units or 'unknown units'} ({channel.samples}, {channel.start_time:g}–{channel.end_time:g} s)"
            self.x_channel.addItem(label,channel.name); self.y_channel.addItem(label,channel.name)
        if channels:
            nx=self.x_channel.findData("VNorth"); ey=self.y_channel.findData("VEast")
            if nx>=0:self.x_channel.setCurrentIndex(nx)
            if ey>=0:self.y_channel.setCurrentIndex(ey)
        self.channel_info.setText(f"Discovered {len(channels)} usable channels. Choose axes explicitly; only known default channels are preselected.")

    def discover_channels(self):
        try: channels=list_channels(self.telemetry.text())
        except Exception as error:
            self.geometry=None; self.save_button.setEnabled(False); self.course_plot.set_course([],[]); self.report.clear(); self.notice.setText(str(error)); return
        self.set_channels(channels)
        if not channels:self.channel_info.setText("No usable channels found.")

    def _selection(self):
        # Preserve the old Value-only path only for untouched legacy defaults.
        if self.x_channel.count()==0 and self.y_channel.count()==0:
            if (self.mode.currentText()!="velocity" or self.x_scale.value()!=1.0 or self.y_scale.value()!=1.0
                    or self.x_unknown.isChecked() or self.y_unknown.isChecked()):
                raise ValueError("Discover channels before using explicit modes, scales, or unit overrides.")
            return None
        x=self.x_channel.currentData(); y=self.y_channel.currentData()
        if not x or not y: raise ValueError("Discover channels and select valid x and y channels.")
        return ChannelSelection(x,y,self.mode.currentText(),self.x_scale.value(),self.y_scale.value(),self.x_unknown.isChecked(),self.y_unknown.isChecked())

    def build(self) -> None:
        if self.thread is not None:
            return
        self.geometry = None
        self.save_button.setEnabled(False)
        self.transfer_button.setEnabled(False)
        self.course_plot.set_overlaps([],[])
        self.course_plot.set_course([], [])
        self.report.clear()
        try:
            selection=self._selection()
            self.settings = closed.ClosedSettings(Path(self.telemetry.text()), self.start.value(), self.end.value(),
                self.spacing.value(), self.smoothing.value(), self.max_radius.value(), selection,
                self.multi_lap.isChecked(), self.return_radius.value(), self.minimum_travel.value(),
                self.minimum_duration.value(), self.maximum_duration.value(), self.deviation_limit.value(), selection is None)
            self.settings.check()
        except ValueError as error:
            self.notice.setText(str(error))
            return
        self.notice.setText("building…")
        self.build_button.setEnabled(False)
        self._build_token=(self.telemetry.text(),self.start.value(),self.end.value(),self.spacing.value(),self.smoothing.value(),self.max_radius.value(),self.mode.currentText(),self.x_scale.value(),self.y_scale.value(),self.x_unknown.isChecked(),self.y_unknown.isChecked(),self.multi_lap.isChecked(),self.return_radius.value(),self.minimum_travel.value(),self.minimum_duration.value(),self.maximum_duration.value(),self.deviation_limit.value(),self.x_channel.currentData(),self.y_channel.currentData())
        self.thread = QThread(self)
        worker = _BuildWorker(self.settings)
        worker.moveToThread(self.thread)
        self.thread.started.connect(worker.run)
        worker.done.connect(self._built)
        worker.failed.connect(self._failed)
        self._worker = worker
        for control in self._build_controls: control.setEnabled(False)
        self.thread.start()

    def _release(self) -> None:
        if self.thread is not None:
            self.thread.quit()
            self.thread.wait()
            self.thread = None
        self.build_button.setEnabled(closed.scipy_available())
        for control in self._build_controls: control.setEnabled(True)

    def _built(self, geometry) -> None:
        self._release()
        current=(self.telemetry.text(),self.start.value(),self.end.value(),self.spacing.value(),self.smoothing.value(),self.max_radius.value(),self.mode.currentText(),self.x_scale.value(),self.y_scale.value(),self.x_unknown.isChecked(),self.y_unknown.isChecked(),self.multi_lap.isChecked(),self.return_radius.value(),self.minimum_travel.value(),self.minimum_duration.value(),self.maximum_duration.value(),self.deviation_limit.value(),self.x_channel.currentData(),self.y_channel.currentData())
        if current!=self._build_token:
            self.geometry=None; self.course_plot.set_course([],[]); self.save_button.setEnabled(False); self.transfer_button.setEnabled(False); self.notice.setText("Discarded build because its inputs changed."); return
        self.geometry = geometry
        figures = closed.summarize(geometry)
        self.report.setPlainText("\n".join(f"{key}: {value:.6g}" for key, value in figures.items()))
        self.course_plot.set_course(geometry["north"], geometry["east"])
        self.course_plot.set_overlaps(geometry.get("overlap_north",[]),geometry.get("overlap_east",[]))
        if geometry.get("lap_diagnostics"):
            diag=geometry["lap_diagnostics"]
            self.report.appendPlainText("\nlap_diagnostics: "+str(diag))
        self.notice.setText("built")
        self.save_button.setEnabled(True)
        self.transfer_button.setEnabled(True)

    def _failed(self, message: str) -> None:
        self._release()
        self.geometry = None
        self.save_button.setEnabled(False)
        self.course_plot.set_course([],[])
        self.transfer_button.setEnabled(False)
        self.notice.setText(message)

    def transfer_to_viewer(self):
        if self.geometry is None:return
        from tools.track_gui.track_ops import from_closed_geometry
        self.window().viewer_tab.set_track(from_closed_geometry(self.geometry,maximum_radius=self.settings.maximum_radius))
        self.window().tabs.setCurrentWidget(self.window().viewer_tab)

    def save(self) -> None:
        if self.geometry is None:
            return
        values = self.save_bar.values()
        try:
            saved = closed.save_closed(self.geometry, self.settings, **values)
        except SaveRefused as refusal:
            if "generated by" in str(refusal):
                if QMessageBox.question(self, "Replace a generated file?", str(refusal) + "\n\nReplace it?") == QMessageBox.StandardButton.Yes:
                    try:
                        retry_values = dict(values, overwrite=True, overwrite_generated=True)
                        saved = closed.save_closed(self.geometry, self.settings, **retry_values)
                    except (OSError, SaveRefused, ValueError) as error:
                        QMessageBox.warning(self, "Not saved", str(error))
                        return
                else:
                    return
            else:
                QMessageBox.warning(self, "Not saved", str(refusal))
                return
        except (OSError, ValueError) as error:
            QMessageBox.warning(self, "Not saved", str(error))
            return
        self.status.emit(f"saved {saved.xml} ({saved.length:.1f} m, {saved.node_count} nodes)")


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Track GUI")
        self.settings = QSettings("Simplified-Vehicle-Models", "track-gui")
        self.tabs = QTabWidget()
        self.design_tab = DesignTab()
        self.closed_tab = ClosedTab()
        from tools.track_gui.qt.track_viewer import TrackViewer
        self.viewer_tab=TrackViewer()
        self.design_tab.transfer_button=QPushButton("Send to viewer")
        self.design_tab.transfer_button.clicked.connect(self.transfer_design_to_viewer)
        self.design_tab.transfer_button.setEnabled(self.design_tab.course is not None)
        self.design_tab.save_button.parentWidget().layout().addWidget(self.design_tab.transfer_button)
        self.tabs.addTab(self.design_tab, "Design an open course")
        self.tabs.addTab(self.closed_tab, "Closed track from telemetry")
        self.tabs.addTab(self.viewer_tab,"Track viewer")
        self.setCentralWidget(self.tabs)
        for tab in (self.design_tab, self.closed_tab):
            tab.status.connect(self.statusBar().showMessage)
        geometry = self.settings.value("geometry")
        if geometry is not None:
            self.restoreGeometry(geometry)
        else:
            self.resize(1200, 760)

    def transfer_design_to_viewer(self):
        if self.design_tab.course is None:return
        from tools.tracks.open_course import OpenCourse
        c=self.design_tab.course
        self.viewer_tab.set_track(__import__('tools.track_gui.track_ops',fromlist=['from_open_course']).from_open_course(c))
        self.tabs.setCurrentWidget(self.viewer_tab)

    def closeEvent(self, event) -> None:
        if self.closed_tab.thread is not None:
            if QMessageBox.question(self, "Quit", "A track build is running. Quit anyway?") != QMessageBox.StandardButton.Yes:
                event.ignore()
                return
            self.closed_tab.thread.quit()
            if not self.closed_tab.thread.wait(5000):
                self.closed_tab.notice.setText("A track build is still running; wait for it to finish before quitting.")
                event.ignore()
                return
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()
