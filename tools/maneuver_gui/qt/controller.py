"""Qt lifecycle wrapper for the simulation-only maneuver runner."""
from __future__ import annotations

import json
import tempfile
from pathlib import Path

from PyQt6.QtCore import QObject, pyqtSignal

from tools.gui_common.qt.process import ProcessJob


class ManeuverController(QObject):
    """Run exactly one validated maneuver child using the common process wrapper."""

    busyChanged = pyqtSignal(bool)
    finished = pyqtSignal(object)
    progress = pyqtSignal(object)

    def __init__(self, tab, *, output_root: Path, parent=None) -> None:
        super().__init__(parent)
        self.tab = tab
        self.output_root = Path(output_root)
        self._job = ProcessJob(self)
        self._definition_file: Path | None = None
        self._job.line.connect(self._line)
        self._job.finished.connect(self._finished)

    def is_running(self) -> bool:
        return self._job.running()

    def start(self, definition) -> None:
        if self.is_running():
            raise RuntimeError("a maneuver is already running")
        payload = definition.to_dict() if hasattr(definition, "to_dict") else dict(definition)
        self.output_root.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile("w", suffix=".maneuver.json", prefix="maneuver-",
                                         dir=self.output_root, encoding="utf-8", delete=False) as stream:
            json.dump(payload, stream, sort_keys=True)
            self._definition_file = Path(stream.name)
        self.tab.set_busy(True)
        self.busyChanged.emit(True)
        self._job.start(["-m", "tools.maneuver_gui.runner", "--definition", str(self._definition_file),
                         "--output-root", str(self.output_root)])

    def cancel(self) -> None:
        self._job.cancel()

    def _line(self, line: str) -> None:
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            return
        if isinstance(item, dict):
            self.progress.emit(item)

    def _finished(self, code: int, state: str) -> None:
        if self._definition_file is not None:
            self._definition_file.unlink(missing_ok=True)
            self._definition_file = None
        outcome = "completed" if code == 0 and state == "done" else ("cancelled" if state == "cancelled" else "failed")
        summary = {"status": outcome.title(), "execution_outcome": outcome,
                   "exit_code": code, "process_state": state}
        self.tab.set_busy(False)
        self.tab.set_result(summary)
        self.busyChanged.emit(False)
        self.finished.emit(summary)
