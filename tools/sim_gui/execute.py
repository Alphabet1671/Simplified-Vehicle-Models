"""Prepare, execute, and finish one simulation GUI case."""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import sys
import time
from typing import Any

from simplified_vehicle_models.compiler.compile import compile_formulation

from . import REPOSITORY_ROOT
from .settings import solution_model_for, yaw_runner_argv, validate
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
    solver_kind: str = "yaw-aware"


def _resolve(value: str | Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else REPOSITORY_ROOT / path


def _atomic_json(path: Path, data: dict[str, Any]) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, sort_keys=True, allow_nan=False) + "\n")
    temporary.replace(path)


def prepare_case(study_dir: Path, definition: StudyDefinition, case: CaseSpec) -> PreparedCase:
    """Prepare yaw-aware cases; selected point-mass is preflight-blocked.

    Solution mapping is a serialization contract only; this does not physically
    adapt or compile a point-mass model.
    """
    if definition.settings.solver_kind == "point-mass":
        raise ValueError("point-mass adaptation unavailable until yaw GUI runner migration")
    directory = Path(study_dir) / "cases" / case.id
    model_dir = directory / "model"
    settings = definition.settings
    solution_model = solution_model_for(settings, case.model)
    configuration = VehicleConfiguration.open(_resolve(definition.source)).with_model(case.model)
    changes = []
    for key, value in case.choices.items():
        current = configuration.choices.get(key)
        resolved = _resolve(value)
        if current is None:
            raise ValueError(f"unknown slot {key}")
        if resolved.resolve() != current.resolve():
            changes.append((key, resolved))
    for key, value in sorted(changes, key=lambda item: (item[0].count("/"), item[0])):
        configuration = configuration.with_choice(key, value)
    files = write_variant(configuration, model_dir, case.id)
    track = _resolve(case.track)
    validate(settings, case.model, track)

    argv = yaw_runner_argv(settings, track)

    payload = {
        "id": case.id, "label": case.label, "model": case.model, "track": case.track,
        "settings": settings.to_json(), "solver_kind": settings.solver_kind,
        "solution_model": solution_model,
        "resolution_mapping": {"source_model": case.model, "solver_kind": settings.solver_kind,
                               "solution_model": solution_model},
        "formulation": files.formulation.relative_to(directory).as_posix(),
        "vehicle": files.vehicle.relative_to(directory).as_posix(), "argv": argv,
    }
    _atomic_json(directory / "case.json", payload)
    timeout = settings.replay_seconds if settings.mode == "replay" else settings.profile_seconds
    timeout = min(1800.0, timeout)
    return PreparedCase(directory, files.formulation, files.vehicle, argv, model=solution_model,
                        timeout_seconds=timeout, duration=settings.duration, solver_kind=settings.solver_kind)


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
    """Build a child command; reject point-mass until physical adaptation lands."""
    if prepared.solver_kind == "point-mass" or prepared.model == "point-mass":
        raise ValueError("point-mass adaptation unavailable until yaw GUI runner migration")
    forbidden = {"--child-attempt", "--output-root"}
    if "--model" not in prepared.argv and "--yaw-map" not in prepared.argv:
        raise ValueError("prepared argv must specify an explicit solver contract")
    if "--model" in prepared.argv:
        raise ValueError("unsupported legacy point-mass command; use yaw-aware runner")
    if "--mode" not in prepared.argv:
        raise ValueError("prepared yaw-aware argv must contain mode")
    if forbidden.intersection(prepared.argv):
        raise ValueError("prepared argv must not contain child-only/output-root options")
    return [sys.executable, "-u", str(Path(__file__).parents[1] / "run" / "run_yaw_lap_suite.py"),
            *prepared.argv, "--output-dir", str(attempt)]


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
                timed_out: bool, cancelled: bool, wall_seconds: float, command) -> dict[str, Any]:
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
    parent = {"status": parent_status, "return_code": parent_code, "total_wall_seconds": wall_seconds,
              "within_wall_limit": not timed_out, "timeout_seconds": timeout,
              "requested_wall_seconds": timeout, "elapsed_wall_seconds": wall_seconds,
              "solver_wall_budget_seconds": timeout, "model": prepared.model,
              "command": list(command), "attempt": str(attempt)}
    _atomic_json(Path(attempt) / "parent.json", parent)
    study_root = prepared.directory.parent.parent
    rel_attempt = Path(attempt).relative_to(study_root).as_posix()
    record = Path(attempt) / "run_record.npz"
    return {"status": status, "attempt": rel_attempt,
            "run_record": record.relative_to(study_root).as_posix() if record.exists() else None,
            "summary": summary}


def _argument(argv: list[str], name: str) -> str:
    return argv[argv.index(name) + 1]


def _timeout_from_argv(argv: list[str]) -> float:
    return float(_argument(argv, "--timeout-seconds")) if "--timeout-seconds" in argv else 300.0


def _model_from_argv(argv: list[str]) -> str:
    return _argument(argv, "--model") if "--model" in argv else "planar-7dof"
