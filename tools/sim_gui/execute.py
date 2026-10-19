"""Prepare, execute, and finish one simulation GUI case."""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import sys
import time
from typing import Any

from simplified_vehicle_models.compiler.compile import compile_formulation
from tools.run import run_iem24_forward_driver as runner

from . import REPOSITORY_ROOT
from .settings import runner_argv
from .study import CaseSpec, StudyDefinition
from .variant import write_variant
from .vehicle import VehicleConfiguration


@dataclass(frozen=True, slots=True)
class PreparedCase:
    directory: Path
    formulation: Path
    vehicle: Path
    argv: list[str]
    model: str = "planar-7dof"
    timeout_seconds: float = 300.0
    duration: float = 300.0


def _resolve(value: str | Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else REPOSITORY_ROOT / path


def _atomic_json(path: Path, data: dict[str, Any]) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, sort_keys=True, allow_nan=False) + "\n")
    temporary.replace(path)


def prepare_case(study_dir: Path, definition: StudyDefinition, case: CaseSpec) -> PreparedCase:
    """Materialize a case's isolated model files and runner arguments."""
    study_dir = Path(study_dir)
    directory = study_dir / "cases" / case.id
    model_dir = directory / "model"
    configuration = VehicleConfiguration.open(_resolve(definition.source)).with_model(case.model)
    changes = []
    for key, value in case.choices.items():
        current = configuration.choices.get(key)
        resolved = _resolve(value)
        if current is None:
            raise ValueError(f"unknown slot {key}")
        if resolved.resolve() != current.resolve():
            changes.append((key, resolved))
    for key, value in sorted(changes, key=lambda item: (item[0].count('/'), item[0])):
        configuration = configuration.with_choice(key, value)
    files = write_variant(configuration, model_dir, case.id)
    track = _resolve(case.track)
    argv = runner_argv(definition.settings, case.model, track)
    payload = {
        "id": case.id, "label": case.label, "model": case.model, "track": case.track,
        "choices": dict(case.choices), "sweep_points": dict(case.sweep_points),
        "formulation": files.formulation.relative_to(directory).as_posix(),
        "vehicle": files.vehicle.relative_to(directory).as_posix(), "argv": argv,
    }
    _atomic_json(directory / "case.json", payload)
    return PreparedCase(
        directory,
        files.formulation,
        files.vehicle,
        argv,
        model=case.model,
        timeout_seconds=definition.settings.timeout_seconds,
        duration=definition.settings.duration,
    )


def validate_case(prepared: PreparedCase) -> str | None:
    try:
        compile_formulation(str(prepared.formulation))
    except Exception as error:
        return f"{type(error).__name__}: {error}"
    return None


def new_attempt(prepared: PreparedCase) -> Path:
    attempt = prepared.directory / f"attempt_{time.time_ns()}"
    attempt.mkdir(parents=True, exist_ok=False)
    return attempt


def child_command(prepared: PreparedCase, attempt: Path) -> list[str]:
    forbidden = {"--child-attempt", "--output-root"}
    if prepared.argv.count("--record") != 1:
        raise ValueError("prepared argv must contain exactly one --record")
    if forbidden.intersection(prepared.argv):
        raise ValueError("prepared argv must not contain --child-attempt or --output-root")
    command = [sys.executable, "-u", str(Path(__file__).with_name("run_case.py").resolve()), "--case-dir",
               str(prepared.directory), *prepared.argv, "--child-attempt", str(attempt)]
    return command


def parse_progress(line: str, duration: float) -> float | None:
    try:
        value = json.loads(line)
        rhs_time = float(value["rhs_time"])
    except (ValueError, TypeError, KeyError, json.JSONDecodeError):
        return None
    if duration <= 0:
        return None
    return max(0.0, min(1.0, rhs_time / duration))


def finish_case(prepared: PreparedCase, attempt: Path, *, return_code: int | None,
                timed_out: bool, cancelled: bool, wall_seconds: float,
                command) -> dict[str, Any]:
    summary_path = Path(attempt) / "summary.json"
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else None
    if cancelled:
        status, parent_status, parent_code = "cancelled", "Cancelled", None
    elif timed_out:
        status, parent_status, parent_code = "timed_out", "Timed_Out", 124
    elif return_code == 0 and summary_path.exists():
        status, parent_status, parent_code = "completed", "Child_Completed", return_code
    elif return_code == 2:
        status, parent_status, parent_code = "rejected", "Child_Rejected", return_code
    else:
        status, parent_status, parent_code = "failed", "Child_Rejected", return_code
    timeout = prepared.timeout_seconds
    parent = {"status": parent_status, "return_code": parent_code,
              "total_wall_seconds": wall_seconds, "within_wall_limit": not timed_out,
              "timeout_seconds": timeout, "requested_wall_seconds": timeout,
              "elapsed_wall_seconds": wall_seconds,
              "solver_wall_budget_seconds": runner.solver_wall_budget(timeout, prepared.model),
              "model": prepared.model, "command": list(command),
              "attempt": str(attempt)}
    _atomic_json(Path(attempt) / "parent.json", parent)
    study_root = prepared.directory.parent.parent
    rel_attempt = Path(attempt).relative_to(study_root).as_posix()
    record = Path(attempt) / "run_record.npz"
    return {"status": status, "attempt": rel_attempt,
            "run_record": (record.relative_to(study_root).as_posix() if record.exists() else None),
            "summary": summary}


def _argument(argv: list[str], name: str) -> str:
    return argv[argv.index(name) + 1]


def _timeout_from_argv(argv: list[str]) -> float:
    return float(_argument(argv, "--timeout-seconds")) if "--timeout-seconds" in argv else 300.0


def _model_from_argv(argv: list[str]) -> str:
    return _argument(argv, "--model") if "--model" in argv else "planar-7dof"
