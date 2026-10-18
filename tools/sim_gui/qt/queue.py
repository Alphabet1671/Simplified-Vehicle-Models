"""Qt process queue for running prepared simulation cases."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time

from PyQt6.QtCore import QProcess, QProcessEnvironment, QTimer, pyqtSignal, QObject, Qt
from PyQt6.QtWidgets import QProgressBar, QTableWidget, QTableWidgetItem

from tools.sim_gui import REPOSITORY_ROOT, child_environment
from tools.sim_gui.execute import (
    PreparedCase,
    child_command,
    finish_case,
    new_attempt,
    parse_progress,
)


_finish_case = finish_case


@dataclass
class _Running:
    prepared: PreparedCase
    process: QProcess
    attempt: Path
    command: list[str]
    timer: QTimer
    started: float
    buffer: bytes = b""
    finished: bool = False
    timed_out: bool = False
    cancelled: bool = False


class CaseQueue(QObject):
    """Run prepared cases with a bounded pool of QProcesses."""

    caseStarted = pyqtSignal(str)
    caseProgress = pyqtSignal(str, float)
    caseFinished = pyqtSignal(str, dict)
    allFinished = pyqtSignal()

    def __init__(
        self,
        study_index=None,
        update_callback=None,
        command_factory=child_command,
        finish_factory=_finish_case,
        parent=None,
    ) -> None:
        """Create a queue with optional persistent index updates."""
        super().__init__(parent)
        self.study_index = study_index
        self.update_callback = update_callback
        self.command_factory = command_factory
        self.finish_factory = finish_factory
        self._pending: list[PreparedCase] = []
        self._running: dict[str, _Running] = {}
        self._parallel = 1
        self._timeout = None
        self._stopping = False
        self._all_emitted = False

    def start(self, prepared_cases, parallel: int, timeout: float | None = None) -> None:
        """Start queued cases with at most ``parallel`` processes."""
        if parallel < 1:
            raise ValueError("parallel must be at least 1")
        self._pending = list(prepared_cases)
        self._parallel = parallel
        self._timeout = timeout
        self._stopping = False
        self._all_emitted = False
        self._fill()
        self._maybe_finished()

    def stop(self) -> None:
        """Cancel pending cases and terminate active processes."""
        if self._stopping:
            return
        self._stopping = True
        while self._pending:
            prepared = self._pending.pop(0)
            self._finish_without_process(prepared, cancelled=True)
        for running in tuple(self._running.values()):
            running.cancelled = True
            running.process.kill()
        self._maybe_finished()

    def _fill(self) -> None:
        while not self._stopping and len(self._running) < self._parallel and self._pending:
            self._launch(self._pending.pop(0))

    def _launch(self, prepared: PreparedCase) -> None:
        attempt = new_attempt(prepared)
        command = self.command_factory(prepared, attempt)
        process = QProcess(self)
        process.setWorkingDirectory(str(REPOSITORY_ROOT))
        environment = QProcessEnvironment()
        for key, value in child_environment().items():
            environment.insert(key, value)
        process.setProcessEnvironment(environment)
        process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        timer = QTimer(self)
        timer.setSingleShot(True)
        running = _Running(prepared, process, attempt, command, timer, time.perf_counter())
        self._running[prepared.directory.name] = running
        process.readyReadStandardOutput.connect(lambda key=prepared.directory.name: self._read(key))
        process.finished.connect(lambda code, status, key=prepared.directory.name: self._finished(key, code, status))
        process.errorOccurred.connect(lambda error, key=prepared.directory.name: self._error(key, error))
        timer.timeout.connect(lambda key=prepared.directory.name: self._timeout_case(key))
        timeout = self._timeout if self._timeout is not None else prepared.timeout_seconds
        timer.start(max(1, int(timeout * 1000)))
        self.caseStarted.emit(prepared.directory.name)
        process.start(command[0], command[1:])

    def _read(self, key: str) -> None:
        running = self._running.get(key)
        if running is None:
            return
        data = bytes(running.process.readAllStandardOutput())
        with (running.attempt / "run.log").open("ab") as log:
            log.write(data)
            log.flush()
        running.buffer += data
        while b"\n" in running.buffer:
            line, running.buffer = running.buffer.split(b"\n", 1)
            progress = parse_progress(
                line.decode(errors="replace"), running.prepared.duration
            )
            if progress is not None:
                self.caseProgress.emit(key, progress)

    def _timeout_case(self, key: str) -> None:
        running = self._running.get(key)
        if running is not None and not running.finished:
            running.timed_out = True
            running.process.kill()

    def _error(self, key: str, error) -> None:
        running = self._running.get(key)
        if running is not None and error == QProcess.ProcessError.FailedToStart:
            self._complete(key, -1)

    def _finished(self, key: str, code: int, status) -> None:
        self._complete(key, code)

    def _complete(self, key: str, code: int) -> None:
        running = self._running.get(key)
        if running is None or running.finished:
            return
        running.finished = True
        self._read(key)
        if running.buffer:
            with (running.attempt / "run.log").open("ab") as log:
                log.write(running.buffer)
                log.flush()
            running.buffer = b""
        running.timer.stop()
        try:
            fields = self.finish_factory(
                running.prepared,
                running.attempt,
                return_code=code,
                timed_out=running.timed_out,
                cancelled=running.cancelled,
                wall_seconds=time.perf_counter() - running.started,
                command=running.command,
            )
            self._record(key, fields)
        except Exception as error:
            fields = {"status": "failed", "error": f"{type(error).__name__}: {error}"}
            try:
                self._record(key, fields)
            except Exception:
                self.caseFinished.emit(key, fields)
        finally:
            self._running.pop(key, None)
            self._fill()
            self._maybe_finished()

    def _finish_without_process(self, prepared: PreparedCase, cancelled: bool) -> None:
        attempt = new_attempt(prepared)
        fields = finish_case(
            prepared, attempt, return_code=None, timed_out=False, cancelled=cancelled,
            wall_seconds=0.0, command=[],
        )
        self._record(prepared.directory.name, fields)

    def shutdown(self) -> None:
        """Kill active processes and wait briefly; call before application exit."""
        self._stopping = True
        for running in tuple(self._running.values()):
            running.cancelled = True
            running.process.kill()
            running.process.waitForFinished(1000)

    def _record(self, key: str, fields: dict) -> None:
        if self.study_index is not None:
            self.study_index.update(key, **fields)
        if self.update_callback is not None:
            self.update_callback(key, fields)
        self.caseFinished.emit(key, fields)

    def _maybe_finished(self) -> None:
        if not self._pending and not self._running and not self._all_emitted:
            self._all_emitted = True
            self.allFinished.emit()


class QueueTable(QTableWidget):
    """Display case queue status and progress."""

    def __init__(self, parent=None) -> None:
        """Create a queue table with the standard study columns."""
        super().__init__(0, 7, parent)
        self.setHorizontalHeaderLabels(("#", "Label", "Status", "Progress", "Wall s", "Lap time s", "Record"))

    def set_cases(self, cases) -> None:
        """Populate rows from case dictionaries."""
        self.setRowCount(0)
        for row, case in enumerate(cases):
            self.insertRow(row)
            ordinal = QTableWidgetItem(str(row + 1))
            ordinal.setData(Qt.ItemDataRole.UserRole, case.get("id", ""))
            self.setItem(row, 0, ordinal)
            self.setItem(row, 1, QTableWidgetItem(case.get("label", "")))
            self.setItem(row, 2, QTableWidgetItem(case.get("status", "pending")))
            progress = QProgressBar()
            progress.setRange(0, 100)
            self.setCellWidget(row, 3, progress)
            self.setItem(row, 4, QTableWidgetItem(str(case.get("wall_seconds", ""))))
            self.setItem(row, 5, QTableWidgetItem(str((case.get("summary") or {}).get("lap_time", ""))))
            self.setItem(row, 6, QTableWidgetItem(case.get("run_record") or ""))
