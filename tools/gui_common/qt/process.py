"""Run one repository child process and report its output line by line."""
from __future__ import annotations

import sys
from pathlib import Path

from PyQt6.QtCore import QObject, QProcess, QTimer, pyqtSignal

from tools.gui_common import REPOSITORY_ROOT, child_environment


class ProcessJob(QObject):
    """One ``QProcess`` with merged output, a timeout and a safe cancel.

    Signals: ``line(str)`` for each output line, ``finished(int, str)`` once with the exit
    code and a state word (``"done"``, ``"cancelled"``, ``"timed_out"`` or ``"failed_to_start"``).
    """

    line = pyqtSignal(str)
    finished = pyqtSignal(int, str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._process: QProcess | None = None
        self._buffer = ""
        self._state = "done"
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._on_timeout)

    def running(self) -> bool:
        return self._process is not None

    def start(self, arguments: list[str], *, timeout_seconds: float | None = None,
              program: str | None = None) -> None:
        """Start ``program`` (default: this Python) with ``arguments`` from the repository root."""
        if self._process is not None:
            raise RuntimeError("a job is already running")
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        process.setWorkingDirectory(str(REPOSITORY_ROOT))
        environment = child_environment()
        from PyQt6.QtCore import QProcessEnvironment
        qt_environment = QProcessEnvironment()
        for key, value in environment.items():
            qt_environment.insert(key, value)
        process.setProcessEnvironment(qt_environment)
        process.readyReadStandardOutput.connect(self._read)
        process.finished.connect(self._on_finished)
        process.errorOccurred.connect(self._on_error)
        self._process, self._buffer, self._state = process, "", "done"
        process.start(program or sys.executable, list(arguments))
        if timeout_seconds:
            self._timer.start(int(timeout_seconds * 1000))

    def cancel(self) -> None:
        if self._process is not None and self._state == "done":
            self._state = "cancelled"
            self._process.kill()

    def _on_timeout(self) -> None:
        if self._process is not None and self._state == "done":
            self._state = "timed_out"
            self._process.kill()

    def _read(self) -> None:
        if self._process is None:
            return
        self._buffer += bytes(self._process.readAllStandardOutput()).decode("utf-8", "replace")
        # A progress writer may end lines with a carriage return, so split on both.
        pieces = self._buffer.replace("\r\n", "\n").replace("\r", "\n").split("\n")
        self._buffer = pieces.pop()
        for piece in pieces:
            self.line.emit(piece)

    def _on_error(self, error) -> None:
        if error == QProcess.ProcessError.FailedToStart and self._process is not None:
            self._state = "failed_to_start"
            self._finish(-1)

    def _on_finished(self, code: int, _status) -> None:
        self._read()
        self._finish(code)

    def _finish(self, code: int) -> None:
        process, state = self._process, self._state
        if process is None:
            return
        self._timer.stop()
        if self._buffer:
            self.line.emit(self._buffer)
        self._process, self._buffer = None, ""
        process.deleteLater()
        self.finished.emit(code, state)


def python_command(script: Path, *arguments: str) -> list[str]:
    """Return the argument list that runs a repository script under this interpreter."""
    return [sys.executable, str(script), *arguments]
