"""GUI-thread QProcess controller for automatic yaw-map preparation."""
from __future__ import annotations

import json
import shlex
import threading
from datetime import datetime
from pathlib import Path
import time

from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot, QProcess, QTimer
from PyQt6.QtWidgets import QMessageBox, QProgressDialog

from tools.sim_gui import REPOSITORY_ROOT, child_environment
from tools.sim_gui.yaw_map import MapBuildRequest, accept_yaw_map, inspect_yaw_map, map_build_command

_confirm = QMessageBox.question


class YawMapController(QObject):
    """Serialize validation-thread requests through one GUI-owned child process."""
    requestBuild = pyqtSignal(object)
    stateChanged = pyqtSignal(bool, str)

    def __init__(self, parent=None, *, command_factory=map_build_command,
                 cache_root: Path | None = None, timeout_ms: int = 1_200_000,
                 save_grace_ms: int = 60_000, cleanup_grace_ms: int = 3000):
        super().__init__(parent)
        self.command_factory = command_factory
        self.cache_root = Path(cache_root or REPOSITORY_ROOT / "runs" / "yaw_maps")
        self.timeout_ms = int(timeout_ms)
        self.save_grace_ms = max(0, int(save_grace_ms))
        self.cleanup_grace_ms = max(100, int(cleanup_grace_ms))
        self.hard_timeout_ms = self.timeout_ms + self.save_grace_ms
        self.process: QProcess | None = None
        self._lock = threading.Lock()
        self._active: dict | None = None
        self._cancelled = False
        self._consented = False
        self.requestBuild.connect(self._start_request)
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._timed_out)
        self._kill_timer = QTimer(self)
        self._kill_timer.setSingleShot(True)
        self._kill_timer.timeout.connect(self._kill_if_running)
        self._pending_terminal = None

    def resolve(self, formulation: Path, settings) -> Path:
        result = inspect_yaw_map(formulation, getattr(settings, "yaw_map", ""), self.cache_root)
        if isinstance(result, Path):
            return result
        event = threading.Event()
        request = {"build": result, "event": event, "result": None, "error": None}
        with self._lock:
            if self._cancelled:
                raise RuntimeError("yaw-map preparation cancelled")
            if self._active is not None:
                raise RuntimeError("another yaw-map build is already active")
            self._active = request
        self.requestBuild.emit(request)
        while not event.wait(.05):
            with self._lock:
                if self._cancelled:
                    break
        if not event.is_set():
            raise RuntimeError("yaw-map preparation cancelled")
        if request["error"]:
            raise RuntimeError(request["error"])
        return Path(request["result"])

    @pyqtSlot(object)
    def _start_request(self, item):
        request: MapBuildRequest = item["build"]
        with self._lock:
            cancelled = self._cancelled
        if cancelled:
            self._finish(item, error="yaw-map preparation cancelled")
            return
        if not self._consented:
            answer = _confirm(self.parent(), "Yaw map required",
                f"This vehicle needs a completed yaw map. Build it now and continue? This may take several minutes.\n\nReason: {request.reason}",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
                QMessageBox.StandardButton.Yes)
            if answer != QMessageBox.StandardButton.Yes:
                self._finish(item, error="yaw-map build cancelled by user")
                return
            self._consented = True
        if self.process is not None:
            self._finish(item, error="yaw-map builder is busy")
            return
        try:
            request.output.parent.mkdir(parents=True, exist_ok=True)
            log_path = request.output.with_suffix(".log")
            self._log_file_path = str(log_path)
            self._log = log_path.open("w", encoding="utf-8")
        except OSError as exc:
            self._finish(item, error=f"cannot create yaw-map build log: {exc}")
            return
        self._dialog = QProgressDialog("Building yaw map…", "Stop", 0, 0, self.parent())
        self._dialog.setAutoClose(False)
        self._dialog.canceled.connect(self.cancel)
        self._dialog.show()
        self._item = item
        self.process = QProcess(self)
        self._started_at = datetime.now().astimezone().isoformat()
        self._started_monotonic = time.monotonic()
        self._terminal_code = None
        self.process.setWorkingDirectory(str(REPOSITORY_ROOT))
        self.process.setProcessEnvironment(self._environment())
        process = self.process
        process.finished.connect(self._process_finished)
        process.errorOccurred.connect(self._process_error)
        process.readyReadStandardOutput.connect(lambda p=process: self._write(p.readAllStandardOutput().data()))
        process.readyReadStandardError.connect(lambda p=process: self._write(p.readAllStandardError().data()))
        self._timer.start(self.hard_timeout_ms)
        try:
            command = self.command_factory(request)
        except Exception as exc:
            self._finish(item, error=f"cannot create yaw-map builder command: {exc}")
            return
        self._command = [str(part) for part in command]
        self._log_event('launch', command=self._command, cwd=str(REPOSITORY_ROOT),
                        started_at=self._started_at, build_budget_ms=self.timeout_ms,
                        save_grace_ms=self.save_grace_ms, hard_timeout_ms=self.hard_timeout_ms,
                        cleanup_grace_ms=self.cleanup_grace_ms)
        self._dialog.setLabelText(f"Building yaw map — {self.timeout_ms // 1000}s build budget + save grace; log: {log_path}")
        self.stateChanged.emit(True, f"Building yaw map — {self.timeout_ms // 1000}s build budget + save grace; log: {log_path}")
        self.process.start(command[0], command[1:])

    @staticmethod
    def _environment():
        from PyQt6.QtCore import QProcessEnvironment
        env = QProcessEnvironment.systemEnvironment()
        for key, value in child_environment().items():
            env.insert(key, value)
        return env

    def _log_event(self, event, **fields):
        if getattr(self, "_log", None):
            payload = {'timestamp': datetime.now().astimezone().isoformat(), 'event': event, **fields}
            self._log.write(json.dumps(payload, default=str) + "\n")
            self._log.flush()

    def _write(self, data):
        if data and getattr(self, "_log", None):
            self._log.write(data.decode("utf-8", errors="replace"))
            self._log.flush()

    def _process_error(self, error):
        if self.process and error == QProcess.ProcessError.FailedToStart:
            self._item["start_failed"] = True
            self._finish(self._item, error=f"yaw-map builder failed to start; log: {self._log_path()}",
                         terminal_status='startup_failure')

    def _process_finished(self, code, status):
        process = self.process
        if process is not None and process.state() == QProcess.ProcessState.NotRunning:
            self._terminal_code = code
            process.deleteLater()
            self.process = None
        if self._pending_terminal is not None:
            terminal = self._pending_terminal
            self._pending_terminal = None
            self._release_request(*terminal, code=code)
            return
        item = getattr(self, "_item", None)
        if item is None:
            process = self.process
            if process is not None and process.state() == QProcess.ProcessState.NotRunning:
                process.deleteLater()
                self.process = None
                terminal = self._pending_terminal
                self._pending_terminal = None
                if terminal is not None:
                    self._release_request(*terminal)
            return
        if code != 0 or status != QProcess.ExitStatus.NormalExit:
            if code == 4 and status == QProcess.ExitStatus.NormalExit:
                self._finish(item, error=f"yaw-map build completed with an incomplete audit (exit 4); report: {item['build'].output.with_suffix('.json')}; log: {self._log_path()}",
                             terminal_status='incomplete')
            else:
                self._finish(item, error=f"yaw-map builder exited with status {code}; log: {self._log_path()}",
                             terminal_status='nonzero_exit')
            return
        try:
            path = accept_yaw_map(item["build"], item["build"].output)
        except Exception as exc:
            details = str(exc).lower()
            report_path = item['build'].output.with_suffix('.json')
            reported_incomplete = False
            try:
                import json
                if report_path.is_file():
                    reported_incomplete = json.loads(report_path.read_text(encoding='utf-8')).get('complete') is False
            except (OSError, ValueError, TypeError):
                pass
            incomplete = ('incomplete' in details or 'not complete' in details
                          or 'complete audit' in details or reported_incomplete)
            classification = 'incomplete' if incomplete else 'invalid_artifact'
            self._finish(item, error=f"yaw-map artifact was not accepted: {exc}; log: {self._log_path()}",
                         terminal_status=classification)
            return
        self._finish(item, result=path, terminal_status='accepted')

    def _timed_out(self):
        item = getattr(self, "_item", None)
        if item is not None:
            self._finish(item, error=f"yaw-map build timed out; log: {self._log_path()}", terminate=True,
                         terminal_status='timeout')

    def _kill_if_running(self):
        if self.process and self.process.state() != QProcess.ProcessState.NotRunning:
            self.process.kill()

    def _log_path(self):
        return str(getattr(self, "_log_file_path", "yaw-map build log"))

    def _finish(self, item, *, result=None, error=None, terminate=False, terminal_status=None):
        if item is None or item["event"].is_set():
            return
        self._timer.stop()
        process = getattr(self, "process", None)
        if process is not None and process.state() != QProcess.ProcessState.NotRunning and not item.get("start_failed"):
            self._pending_terminal = (item, result, error, terminal_status)
            process.terminate()
            self._kill_timer.start(self.cleanup_grace_ms)
            return
        if process is not None:
            process.deleteLater()
            self.process = None
        self._release_request(item, result, error, terminal_status)

    def _release_request(self, item, result, error, terminal_status=None, code=None):
        elapsed = max(0.0, time.monotonic() - getattr(self, '_started_monotonic', time.monotonic()))
        code = getattr(self, '_terminal_code', None) if code is None else code
        status = terminal_status or ('cancelled' if error and 'cancel' in error.lower() else ('error' if error else 'accepted'))
        self._log_event('terminal', status=status, exit_code=code, error=error,
                        elapsed_seconds=elapsed, finished_at=datetime.now().astimezone().isoformat())
        if getattr(self, "_log", None):
            self._log.close()
            self._log = None
        if getattr(self, "_dialog", None):
            self._dialog.canceled.disconnect(self.cancel)
            self._dialog.close()
            self._dialog = None
        with self._lock:
            item["result"], item["error"] = result, error
            if self._active is item:
                self._active = None
        item["event"].set()
        self._item = None
        self.stateChanged.emit(False, error or "")

    def cancel(self):
        with self._lock:
            self._cancelled = True
            item = self._active
        if item is not None:
            self._finish(item, error="yaw-map build cancelled", terminate=True)

    def shutdown(self, wait_ms: int = 2500) -> bool:
        self.cancel()
        process = self.process
        if process is None:
            return True
        if process.state() != QProcess.ProcessState.NotRunning:
            process.kill()
            if not process.waitForFinished(wait_ms):
                return False
        process.deleteLater()
        self.process = None
        terminal = self._pending_terminal
        self._pending_terminal = None
        if terminal is not None:
            self._release_request(*terminal)
        return True
