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
    """Materialize the selected source configuration and its solver form."""
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
    if not (settings.solver_kind == "yaw-aware" and case.model == "14dof-simple") and solution_model != case.model:
        from xml.etree import ElementTree
        tree = ElementTree.parse(files.formulation)
        tree.getroot().set("subtype", solution_model)
        source_element = tree.getroot().find("source")
        if source_element is not None:
            source_element.set("ref", "vehicle.xml")
        for required in list(tree.getroot().findall("requires")):
            tree.getroot().remove(required)
        tree.write(files.formulation, encoding="UTF-8", xml_declaration=True)
    track = _resolve(case.track)
    validate(settings, case.model, track)

    if settings.solver_kind == "yaw-aware":
        argv = yaw_runner_argv(settings, track)
    else:
        argv = []  # Point-mass options are loaded from immutable case.json.

    payload = {
        "id": case.id, "label": case.label, "model": case.model, "track": case.track,
        "settings": settings.to_json(), "solver_kind": settings.solver_kind,
        "solution_model": solution_model,
        "resolution_mapping": {"source_model": case.model, "solver_kind": settings.solver_kind,
                               "solution_model": solution_model, "reduction": ("point_mass_view_of_planar" if settings.solver_kind == "point-mass" and case.model == "planar-7dof" else "14dof.planar->point_mass_view_of_planar" if settings.solver_kind == "point-mass" and case.model == "14dof-simple" else "identity" if settings.solver_kind == "point-mass" else "14dof.planar->planar-7dof" if case.model == "14dof-simple" else "identity")},
        "source_model": case.model,
        "formulation": files.formulation.relative_to(directory).as_posix(),
        "source_formulation": files.formulation.relative_to(directory).as_posix(),
        "vehicle": files.vehicle.relative_to(directory).as_posix(), "argv": argv,
    }
    _atomic_json(directory / "case.json", payload)
    timeout = (settings.point_mass_duration if settings.solver_kind == "point-mass"
               else settings.replay_seconds if settings.mode == "replay" else settings.profile_seconds)
    timeout = min(1800.0, timeout)
    return PreparedCase(directory, files.formulation, files.vehicle, argv, model=solution_model,
                        timeout_seconds=timeout, duration=settings.duration, solver_kind=settings.solver_kind)


def validate_case(prepared: PreparedCase) -> str | None:
    try:
        compiled = compile_formulation(str(prepared.formulation)).model
        if prepared.solver_kind == "yaw-aware" and prepared.model == "planar-7dof" and not hasattr(compiled, "yaw_inertia"):
            raise ValueError("yaw-aware selected variant did not compile to planar 7DoF")
        if prepared.model == "point-mass":
            from simplified_vehicle_models.parameters.vehicle import PointMassVehicleParameters
            from simplified_vehicle_models.formulations.vertical_14dof import VerticalVehicleParameters
            if not isinstance(compiled, (PointMassVehicleParameters, VerticalVehicleParameters)) and not hasattr(compiled, "yaw_inertia"):
                raise ValueError(f"unsupported compiled source type for point-mass reduction: {type(compiled).__name__}")
    except Exception as error:
        return f"{type(error).__name__}: {error}"
    return None


def new_attempt(prepared: PreparedCase) -> Path:
    attempt = prepared.directory / f"attempt_{time.time_ns()}"
    attempt.mkdir(parents=True, exist_ok=False)
    return attempt


def child_command(prepared: PreparedCase, attempt: Path) -> list[str]:
    """Run the dedicated adapter that resolves immutable case inputs."""
    return [sys.executable, "-u", str(Path(__file__).with_name("run_yaw_case.py")),
            "--case-dir", str(prepared.directory), "--child-attempt", str(attempt)]


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
    try:
        summary = json.loads(summary_path.read_text()) if summary_path.exists() else None
    except (OSError, json.JSONDecodeError):
        summary = None
    valid_summary = isinstance(summary, dict) and isinstance(summary.get("status"), str)
    if cancelled:
        status, parent_status, parent_code = "cancelled", "Cancelled", None
    elif timed_out:
        status, parent_status, parent_code = "timed_out", "Timed_Out", 124
    elif return_code == 2:
        status, parent_status, parent_code = "rejected", "Child_Rejected", return_code
    elif return_code != 0:
        status, parent_status, parent_code = "failed", "Child_Rejected", return_code
    elif not valid_summary or not (Path(attempt) / "inputs.json").is_file():
        status, parent_status, parent_code = "failed", "Child_Invalid_Artifacts", return_code
    elif (prepared.solver_kind == "yaw-aware" and summary["status"] == "planned"
          and summary.get("lap_completed") is False and (Path(attempt) / "plan.npz").is_file()):
        status, parent_status, parent_code = "planned", "Child_Planned", return_code
    elif (prepared.solver_kind == "point-mass" and summary["status"] == "completed"
          and summary.get("converged") is True and (Path(attempt) / "point_mass_plan.npz").is_file()):
        status, parent_status, parent_code = "completed", "Child_Completed", return_code
    elif (summary["status"] == "completed" and summary.get("lap_completed") is True
          and summary.get("solver_validation_complete") is True
          and (Path(attempt) / "run_record.npz").is_file()):
        status, parent_status, parent_code = "completed", "Child_Completed", return_code
    elif summary["status"] == "failed":
        status, parent_status, parent_code = "failed", "Child_Failed", return_code
    elif summary["status"] == "incomplete":
        status, parent_status, parent_code = "failed", "Child_Incomplete", return_code
    elif summary["status"] == "rejected":
        status, parent_status, parent_code = "rejected", "Child_Rejected", return_code
    else:
        status, parent_status, parent_code = "failed", "Child_Invalid_Status", return_code
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
    plan = Path(attempt) / "plan.npz"
    point_plan = Path(attempt) / "point_mass_plan.npz"
    artifacts = {name: (Path(attempt) / name).relative_to(study_root).as_posix()
                 for name in ("inputs.json", "plan.npz", "run_record.npz", "point_mass_plan.npz", "trace.csv")
                 if (Path(attempt) / name).is_file()}
    return {"status": status, "attempt": rel_attempt,
            "run_record": record.relative_to(study_root).as_posix() if record.exists() else None,
            "plan": plan.relative_to(study_root).as_posix() if plan.exists() else None,
            "point_mass_plan": point_plan.relative_to(study_root).as_posix() if point_plan.exists() else None,
            "artifacts": artifacts, "summary": summary}


def _argument(argv: list[str], name: str) -> str:
    return argv[argv.index(name) + 1]


def _timeout_from_argv(argv: list[str]) -> float:
    return float(_argument(argv, "--timeout-seconds")) if "--timeout-seconds" in argv else 300.0


def _model_from_argv(argv: list[str]) -> str:
    return _argument(argv, "--model") if "--model" in argv else "planar-7dof"
