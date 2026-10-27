"""Qt lifecycle wrapper for simulation-only maneuver result generation."""
from __future__ import annotations

import csv
import json
import tempfile
from pathlib import Path

from PyQt6.QtCore import QObject, pyqtSignal

from tools.gui_common.qt.process import ProcessJob
from tools.maneuver_gui.artifacts import load_artifact_evidence


class ManeuverController(QObject):
    """Run one validated simulation child and expose only accepted artifact evidence."""

    busyChanged = pyqtSignal(bool)
    finished = pyqtSignal(object)
    progress = pyqtSignal(object)

    def __init__(self, tab, *, output_root: Path, parent=None) -> None:
        super().__init__(parent)
        self.tab = tab
        self.output_root = Path(output_root)
        self._job = ProcessJob(self)
        self._definition_file: Path | None = None
        self._run_root: Path | None = None
        self._log_file: Path | None = None
        self._completed_artifact: Path | None = None
        self._job.line.connect(self._line)
        self._job.finished.connect(self._finished)

    def is_running(self) -> bool:
        return self._job.running()

    def start(self, definition) -> None:
        if self.is_running():
            raise RuntimeError("a maneuver is already running")
        payload = definition.to_dict() if hasattr(definition, "to_dict") else dict(definition)
        self._run_root = Path(self.tab.export_folder() if hasattr(self.tab, "export_folder") else self.output_root)
        self._run_root.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile("w", suffix=".maneuver.json", prefix="maneuver-",
                                         dir=self._run_root, encoding="utf-8", delete=False) as stream:
            json.dump(payload, stream, sort_keys=True)
            self._definition_file = Path(stream.name)
        self._log_file = self._definition_file.with_suffix(".run.log")
        self.tab.clear_run()
        self.tab.set_busy(True)
        self.busyChanged.emit(True)
        self._job.start(["-m", "tools.maneuver_gui.runner", "--definition", str(self._definition_file),
                         "--output-root", str(self._run_root)])

    def cancel(self) -> None:
        self._job.cancel()

    def _append_log(self, line: str) -> None:
        self.tab.append_log(line)
        if self._log_file is not None:
            with self._log_file.open("a", encoding="utf-8") as stream:
                stream.write(str(line) + "\n")

    def _line(self, line: str) -> None:
        self._append_log(line)
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            return
        if isinstance(item, dict):
            self.progress.emit(item)
            fraction = item.get("fraction")
            if isinstance(fraction, (int, float)):
                self.tab.set_progress(float(fraction))
            phase, time_s = item.get("phase"), item.get("time_s")
            if phase is not None:
                self.tab.status_label.setText(f"{phase} at {float(time_s or 0.):.3f} s")

    @staticmethod
    def _commands(path: Path):
        with path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        commands = [{"swa_rad": float(row["swa_rad"]), "accelerator": float(row["accelerator"]),
                     "brake": float(row["brake"])} for row in rows]
        return commands

    def _completed_result(self):
        if self._run_root is None:
            raise ValueError("run output root unavailable")
        attempts = [path for path in self._run_root.iterdir()
                    if path.is_dir() and (path / "result").is_dir()]
        if len(attempts) != 1:
            raise ValueError("expected exactly one completed maneuver attempt")
        artifact = attempts[0] / "result"
        manifest, arrays, _channels = load_artifact_evidence(artifact)
        if manifest.get("execution_outcome") != "completed":
            raise ValueError("artifact is not completed")
        from simplified_vehicle_models.results.maneuver_run_record import load_maneuver_run_record
        load_maneuver_run_record(attempts[0] / "run_record.npz", artifact)
        commands = self._commands(artifact / "commands.csv")
        if len(commands) != len(arrays["time_s"]):
            raise ValueError("accepted command count does not match evidence time")
        return attempts[0], artifact, manifest, arrays, commands

    def _failure_summary(self, code: int, state: str) -> dict:
        summary = {"status": "Cancelled" if state == "cancelled" else "Failed",
                   "execution_outcome": "cancelled" if state == "cancelled" else "failed",
                   "exit_code": code, "process_state": state}
        if self._run_root is not None:
            attempts = [path for path in self._run_root.iterdir()
                        if path.is_dir() and (path / "summary.json").is_file()]
            if len(attempts) == 1:
                try:
                    summary.update(json.loads((attempts[0] / "summary.json").read_text(encoding="utf-8")))
                    summary["attempt"] = str(attempts[0])
                except (OSError, json.JSONDecodeError):
                    pass
        return summary

    def _finished(self, code: int, state: str) -> None:
        if self._definition_file is not None:
            self._definition_file.unlink(missing_ok=True)
            self._definition_file = None
        try:
            if code != 0 or state != "done":
                summary = self._failure_summary(code, state)
                self.tab.set_result(summary)
            else:
                attempt, artifact, manifest, arrays, commands = self._completed_result()
                self._completed_artifact = artifact
                summary = {"status": "Completed", "execution_outcome": "completed", "exit_code": code,
                           "process_state": state, "attempt": str(attempt), "artifact": str(artifact),
                           "termination_reason": manifest.get("termination_reason", "duration"),
                           "run_record": str(attempt / "run_record.npz")}
                # Only accepted/applied commands are stored as checked evidence.
                self.tab.set_completed_result(summary, artifact, arrays["time_s"], commands, commands)
        except (OSError, ValueError, KeyError) as error:
            summary = {"status": "Failed", "execution_outcome": "failed", "exit_code": code,
                       "process_state": state, "error": str(error)}
            self.tab.set_result(summary)
        self.tab.set_busy(False)
        self.busyChanged.emit(False)
        self.finished.emit(summary)
