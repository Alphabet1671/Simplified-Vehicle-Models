"""The animation GUI main window."""
from __future__ import annotations

import tempfile
from dataclasses import replace
from pathlib import Path

from PyQt6.QtCore import QSettings, Qt
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGroupBox,
                             QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMessageBox, QPlainTextEdit,
                             QProgressBar, QPushButton, QSlider, QSpinBox, QSplitter, QVBoxLayout, QWidget)

from tools.anim_gui import inspect_plan, progress, records, settings as S
from tools.gui_common import REPOSITORY_ROOT
from tools.gui_common.files import protected_directory_problem
from tools.gui_common.qt.process import ProcessJob, python_command

EXPORT_TIMEOUT_SECONDS = 6 * 3600.0
PREVIEW_TIMEOUT_SECONDS = 120.0


def _unsafe_symlink_path(path: Path) -> Path | None:
    """Return the first lexical symlink component, without following links."""
    path = Path(path)
    current = Path(path.anchor) if path.is_absolute() else Path.cwd()
    parts = path.parts[1:] if path.is_absolute() else path.parts
    for part in parts:
        current /= part
        # macOS exposes the system temporary directory through /var -> /private/var;
        # permit this fixed OS alias so normal TemporaryDirectory outputs remain usable.
        if current.is_symlink() and not (current == Path("/var") and current.resolve() == Path("/private/var")):
            return current
    return None


def _spin(low, high, value, decimals, step):
    box = QDoubleSpinBox()
    box.setRange(low, high)
    box.setDecimals(decimals)
    box.setSingleStep(step)
    box.setValue(value)
    return box


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Animation GUI")
        self.settings = QSettings("Simplified-Vehicle-Models", "anim-gui")
        self.job = ProcessJob(self)
        self.job.line.connect(self._line)
        self.job.finished.connect(self._finished)
        self.mode = "idle"             # idle | export | preview
        self.errors: list[str] = []
        self.summary: inspect_plan.PlanSummary | None = None
        self.span = (0.0, 1.0)
        self._preview_folder: tempfile.TemporaryDirectory | None = None
        self._active_output: Path | None = None
        self._closing = False

        # record
        self.record = QLineEdit()
        self.record.editingFinished.connect(self.load_record)
        browse = QPushButton("Record…")
        browse.clicked.connect(self._browse_record)
        study = QPushButton("Study…")
        study.clicked.connect(self._browse_study)
        self.cases = QComboBox()
        self.cases.currentIndexChanged.connect(self._case_chosen)
        self.cases.setVisible(False)

        # options
        self.format = QComboBox()
        self.format.addItem("MP4 video (H.264)", "mp4")
        self.format.addItem("PNG sequence", "png")
        self.format.currentIndexChanged.connect(self._format_changed)
        self.output = QLineEdit()
        output_browse = QPushButton("Browse…")
        output_browse.clicked.connect(self._browse_output)
        self.rate = _spin(1.0, 1000.0, 120.0, 1, 10.0)
        self.width = QSpinBox()
        self.width.setRange(2, 7680)
        self.width.setSingleStep(2)
        self.width.setValue(1920)
        self.height = QSpinBox()
        self.height.setRange(2, 4320)
        self.height.setSingleStep(2)
        self.height.setValue(1080)
        self.crf = QSpinBox()
        self.crf.setRange(0, 51)
        self.crf.setValue(18)
        self.preset = QComboBox()
        self.preset.addItems(S.PRESETS)
        self.preset.setCurrentText("medium")
        self.use_range = QCheckBox("Render only a time range")
        self.start = _spin(0.0, 1e6, 0.0, 3, 0.1)
        self.end = _spin(0.0, 1e6, 1.0, 3, 0.1)
        self.top_view = QCheckBox("Force the top view")
        self.quasi = QCheckBox("Approximate attitude (quasi-static)")
        self.stl_body = QLineEdit()
        self.stl_wheel = QLineEdit()
        self.wheel_radius = _spin(0.01, 5.0, 0.2, 3, 0.01)
        self.wheel_width = _spin(0.01, 5.0, 0.18, 3, 0.01)
        self.scale = _spin(1e-6, 1e6, 1.0, 6, 0.001)
        for widget in (self.rate, self.start, self.end, self.wheel_radius, self.wheel_width, self.scale):
            widget.editingFinished.connect(self.refresh_plan)
        for widget in (self.width, self.height, self.crf):
            widget.editingFinished.connect(self.refresh_plan)
        for widget in (self.use_range, self.top_view, self.quasi):
            widget.toggled.connect(self.refresh_plan)
        self.preset.currentIndexChanged.connect(self.refresh_plan)
        self.stl_body.editingFinished.connect(self.refresh_plan)
        self.stl_wheel.editingFinished.connect(self.refresh_plan)

        # plan + preview
        self.plan_text = QPlainTextEdit()
        self.plan_text.setReadOnly(True)
        self.problem = QLabel()
        self.problem.setWordWrap(True)
        self.problem.setStyleSheet("color: #b00020")
        self.encoder_note = QLabel()
        self.encoder_note.setWordWrap(True)
        self.encoder_note.setStyleSheet("color: #8a5a00")
        self.time_slider = QSlider(Qt.Orientation.Horizontal)
        self.time_slider.setRange(0, 1000)
        self.time_label = QLabel("t = 0.000 s")
        self.time_slider.valueChanged.connect(self._slider_moved)
        self.preview_button = QPushButton("Preview this moment")
        self.preview_button.clicked.connect(self.start_preview)
        self.picture = QLabel("No preview yet. In-app playback is not offered; export the video to watch it.")
        self.picture.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.picture.setMinimumSize(480, 270)
        self.picture.setWordWrap(True)
        self.bar = QProgressBar()
        self.bar.setRange(0, 1000)
        self.speed = QLabel()
        self.export_button = QPushButton("Export")
        self.export_button.clicked.connect(self.start_export)
        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.clicked.connect(self.cancel)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)

        form = QFormLayout()
        top = QHBoxLayout()
        top.addWidget(self.record, 1)
        top.addWidget(browse)
        top.addWidget(study)
        form.addRow("Run record", top)
        form.addRow("Study case", self.cases)
        form.addRow("Format", self.format)
        line = QHBoxLayout()
        line.addWidget(self.output, 1)
        line.addWidget(output_browse)
        form.addRow("Output", line)
        form.addRow("Frame rate (fps)", self.rate)
        size = QHBoxLayout()
        size.addWidget(self.width)
        size.addWidget(QLabel("×"))
        size.addWidget(self.height)
        form.addRow("Size (even pixels)", size)
        form.addRow("Quality (CRF, lower is better)", self.crf)
        form.addRow("Encoder preset", self.preset)
        form.addRow(self.use_range)
        span = QHBoxLayout()
        span.addWidget(self.start)
        span.addWidget(QLabel("to"))
        span.addWidget(self.end)
        form.addRow("Time range (s)", span)
        form.addRow(self.top_view)
        form.addRow(self.quasi)
        shape = QGroupBox("Vehicle shape (display only)")
        shape_form = QFormLayout(shape)
        shape_form.addRow("Body STL", self.stl_body)
        shape_form.addRow("Wheel STL", self.stl_wheel)
        shape_form.addRow("Wheel radius (m)", self.wheel_radius)
        shape_form.addRow("Wheel width (m)", self.wheel_width)
        shape_form.addRow("STL scale", self.scale)
        left = QVBoxLayout()
        left.addLayout(form)
        left.addWidget(shape)
        left.addStretch()
        left_widget = QWidget()
        left_widget.setLayout(left)

        right = QVBoxLayout()
        right.addWidget(self.encoder_note)
        right.addWidget(self.problem)
        right.addWidget(QLabel("What the export will do"))
        right.addWidget(self.plan_text, 2)
        moment = QHBoxLayout()
        moment.addWidget(self.time_slider, 1)
        moment.addWidget(self.time_label)
        moment.addWidget(self.preview_button)
        right.addLayout(moment)
        right.addWidget(self.picture, 3)
        run = QHBoxLayout()
        run.addWidget(self.bar, 1)
        run.addWidget(self.speed)
        run.addWidget(self.export_button)
        run.addWidget(self.cancel_button)
        right.addLayout(run)
        right.addWidget(self.log, 1)
        right_widget = QWidget()
        right_widget.setLayout(right)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setStretchFactor(1, 2)
        self.setCentralWidget(splitter)
        self._apply_encoder()
        geometry = self.settings.value("geometry")
        if geometry is not None:
            self.restoreGeometry(geometry)
        else:
            self.resize(1280, 820)
        self._buttons()

    # -- state ------------------------------------------------------------------------------
    def _apply_encoder(self) -> None:
        problem = inspect_plan.encoder_problem()
        index = self.format.findData("mp4")
        self.format.model().item(index).setEnabled(problem is None)
        self.encoder_note.setText(f"MP4 export is off. {problem}" if problem else "")
        if problem:
            self.format.setCurrentIndex(self.format.findData("png"))

    def current_format(self) -> str:
        return self.format.currentData()

    def _buttons(self) -> None:
        idle = self.mode == "idle"
        ready = idle and self.summary is not None
        self.export_button.setEnabled(ready)
        self.preview_button.setEnabled(ready)
        self.cancel_button.setEnabled(not idle)
        self.record.setEnabled(idle)
        self.format.setEnabled(idle)

    def current_settings(self) -> S.ExportSettings:
        text = self.record.text().strip()
        output = self.output.text().strip()
        fmt = self.current_format()
        optional = lambda widget: Path(widget.text().strip()) if widget.text().strip() else None  # noqa: E731
        record_path = self._root_path(Path(text))
        output_path = self._root_path(Path(output)) if output else S.default_output(record_path, fmt)
        return S.ExportSettings(
            record=record_path, output=output_path, format=fmt,
            frame_rate=self.rate.value(), width=self.width.value(), height=self.height.value(), crf=self.crf.value(),
            preset=self.preset.currentText(),
            time_range=(self.start.value(), self.end.value()) if self.use_range.isChecked() else None,
            stl_body=optional(self.stl_body), stl_wheel=optional(self.stl_wheel),
            wheel_radius=self.wheel_radius.value(), wheel_width=self.wheel_width.value(),
            transform_scale=self.scale.value(), force_top_view=self.top_view.isChecked(),
            quasi_static_attitude=self.quasi.isChecked())

    # -- choosing a record -------------------------------------------------------------------
    def _browse_record(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Run record", str(REPOSITORY_ROOT), "Run record (*.npz)")
        if path:
            self.cases.setVisible(False)
            self.record.setText(path)
            self.load_record()

    def _browse_study(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Study", str(REPOSITORY_ROOT), "Study (study.json)")
        if path:
            self.open_study(Path(path))

    def open_study(self, path: Path) -> bool:
        try:
            choices = records.study_records(path)
        except ValueError as error:
            QMessageBox.warning(self, "Not opened", str(error))
            return False
        if not choices:
            QMessageBox.information(self, "No records", "None of the cases in this study wrote a run record.")
            return False
        self.cases.blockSignals(True)
        self.cases.clear()
        for choice in choices:
            self.cases.addItem(choice.label, choice.path)
        self.cases.blockSignals(False)
        self.cases.setVisible(True)
        self._case_chosen()
        return True

    def _case_chosen(self) -> None:
        path = self.cases.currentData()
        if path is not None:
            self.record.setText(str(path))
            self.load_record()

    def load_record(self) -> None:
        text = self.record.text().strip()
        self.summary = None
        if not text:
            self._buttons()
            return
        try:
            self.span = inspect_plan.record_span(self._root_path(Path(text)))
        except ValueError as error:
            self.problem.setText(str(error))
            self.plan_text.clear()
            self._buttons()
            return
        for box, value in ((self.start, self.span[0]), (self.end, self.span[1])):
            box.blockSignals(True)
            box.setRange(self.span[0], self.span[1])
            box.setValue(value)
            box.blockSignals(False)
        self.output.setText(str(S.default_output(Path(text), self.current_format())))
        self.refresh_plan()

    def _format_changed(self) -> None:
        text = self.record.text().strip()
        if text:
            self.output.setText(str(S.default_output(Path(text), self.current_format())))
        self.refresh_plan()

    def _browse_output(self) -> None:
        if self.current_format() == "png":
            chosen = QFileDialog.getExistingDirectory(self, "Frames folder", self.output.text())
        else:
            chosen, _ = QFileDialog.getSaveFileName(self, "Video file", self.output.text(), "MP4 video (*.mp4)")
        if chosen:
            self.output.setText(chosen)

    # -- the plan ---------------------------------------------------------------------------
    def refresh_plan(self) -> None:
        if not self.record.text().strip() or self.mode != "idle":
            return
        try:
            self.summary = inspect_plan.inspect(self.current_settings())
        except ValueError as error:
            self.summary = None
            self.problem.setText(str(error))
            self.plan_text.clear()
        else:
            self.problem.setText("")
            self.plan_text.setPlainText(self.summary.text)
        self._buttons()

    def _slider_moved(self) -> None:
        self.time_label.setText(f"t = {self.preview_time():.3f} s")

    def preview_time(self) -> float:
        low, high = (self.start.value(), self.end.value()) if self.use_range.isChecked() else self.span
        return low + (high - low) * self.time_slider.value() / 1000.0

    # -- running ----------------------------------------------------------------------------
    def _root_path(self, path: Path) -> Path:
        return path if path.is_absolute() else (REPOSITORY_ROOT / path).resolve(strict=False)

    def _normalized_settings(self) -> S.ExportSettings:
        settings = self.current_settings()
        return replace(settings, record=self._root_path(settings.record), output=self._root_path(settings.output),
                       stl_body=self._root_path(settings.stl_body) if settings.stl_body else None,
                       stl_wheel=self._root_path(settings.stl_wheel) if settings.stl_wheel else None)

    def _run(self, settings: S.ExportSettings, mode: str, timeout: float) -> None:
        self.mode, self.errors = mode, []
        self._active_output = settings.output
        self.bar.setValue(0)
        self.speed.setText("")
        self.log.clear()
        self._buttons()
        command = python_command(S.EXPORT_SCRIPT, *settings.arguments(progress=True))
        # ProcessJob launches the current interpreter itself; pass its argument tail.
        self.job.start(command[1:], timeout_seconds=timeout)

    def start_export(self) -> None:
        if self.mode != "idle" or self.summary is None:
            return
        settings = self._normalized_settings()
        try:
            settings.check()
        except ValueError as error:
            QMessageBox.warning(self, "Not exported", str(error))
            return
        target = settings.output
        if not target.is_absolute():
            target = (REPOSITORY_ROOT / target).resolve(strict=False)
            settings = replace(settings, output=target)
        unsafe = _unsafe_symlink_path(target)
        if unsafe is not None:
            QMessageBox.warning(self, "Not exported", f"Refusing to write through output path symlink: {unsafe}")
            return
        if settings.format == "mp4":
            directory = target.parent
        else:
            directory = target
        protected = protected_directory_problem(directory, allow_models=False)
        if protected:
            if QMessageBox.question(self, "Protected folder", protected + "\nAllow writing inside models/?") != QMessageBox.StandardButton.Yes:
                return
            protected = protected_directory_problem(directory, allow_models=True)
            if protected:
                QMessageBox.warning(self, "Not exported", protected)
                return
        if target.is_symlink():
            QMessageBox.warning(self, "Not exported", f"Refusing to write through output symlink: {target}")
            return
        if target.exists():
            if settings.format == "mp4" and not target.is_file():
                QMessageBox.warning(self, "Not exported", f"Output is not a regular file: {target}")
                return
            if settings.format == "png" and not target.is_dir():
                QMessageBox.warning(self, "Not exported", f"Output is not a directory: {target}")
                return
            if QMessageBox.question(self, "Replace?", f"{target} already exists. Replace it?") != QMessageBox.StandardButton.Yes:
                return
            if QMessageBox.question(self, "Confirm overwrite", f"Replace existing export {target} and its contents?") != QMessageBox.StandardButton.Yes:
                return
        try:
            target.parent.mkdir(parents=True, exist_ok=True) if settings.format == "mp4" else target.mkdir(parents=True, exist_ok=True)
        except OSError as error:
            QMessageBox.warning(self, "Not exported", f"Cannot prepare output {target}: {error}")
            return
        self._run(settings, "export", EXPORT_TIMEOUT_SECONDS)

    def start_preview(self) -> None:
        if self.mode != "idle" or self.summary is None:
            return
        self._drop_preview()
        self._preview_folder = tempfile.TemporaryDirectory(prefix="anim-gui-preview-")
        folder = Path(self._preview_folder.name) / "frames"
        folder.mkdir()
        last = self.span[1]
        time = min(self.preview_time(), last - 0.06 if last - self.span[0] > 0.06 else self.span[0])
        settings = S.preview_settings(self.current_settings(), max(time, self.span[0]), folder)
        settings = replace(settings, time_range=(settings.time_range[0], min(settings.time_range[1], last)))
        self._run(settings, "preview", PREVIEW_TIMEOUT_SECONDS)

    def cancel(self) -> None:
        if self.mode != "idle":
            self.job.cancel()

    def _drop_preview(self) -> None:
        if self._preview_folder is not None:
            self._preview_folder.cleanup()
            self._preview_folder = None

    def _line(self, text: str) -> None:
        value = progress.parse_progress(text)
        if value is not None:
            self.bar.setValue(int(value.fraction * 1000))
            self.speed.setText(f"{value.done}/{value.total}  {value.frames_per_second:.1f} fps  eta {progress.format_eta(value.eta_seconds)}")
            return
        error = progress.parse_error(text)
        if error:
            self.errors.append(error)
        self.log.appendPlainText(text)

    def _finished(self, code: int, state: str) -> None:
        mode, output = self.mode, self._active_output
        self.mode = "idle"
        if (code != 0 or state != "done") and mode == "export" and output is not None:
            records.remove_partials(output)
        message = progress.describe_result(code, state, self.errors)
        if mode == "preview":
            self._show_preview(code, state, message)
        else:
            self.statusBar().showMessage(message if code or state != "done" else f"Done. Wrote {output}", 15000)
            if code == 0 and state == "done":
                self.bar.setValue(1000)
        self._buttons()
        if self._closing:
            self._closing = False
            self.close()

    def _show_preview(self, code: int, state: str, message: str) -> None:
        frames = sorted((Path(self._preview_folder.name) / "frames").glob("frame_*.png")) if self._preview_folder else []
        if code == 0 and state == "done" and frames:
            pixmap = QPixmap(str(frames[0]))
            self.picture.setPixmap(pixmap.scaled(self.picture.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                                 Qt.TransformationMode.SmoothTransformation))
            self.statusBar().showMessage("Preview ready.", 5000)
        else:
            self.picture.setPixmap(QPixmap())
            self.picture.setText(f"Preview failed. {message}")
            self.statusBar().showMessage("Preview failed.", 8000)

    def closeEvent(self, event) -> None:
        if self.mode != "idle":
            if QMessageBox.question(self, "Quit", "An export is running. Cancel it and quit?") != QMessageBox.StandardButton.Yes:
                event.ignore()
                return
            self._closing = True
            self.job.cancel()
            event.ignore()
            return
        self._drop_preview()
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()
