"""Prepare, execute, and finish one simulation GUI case."""
from __future__ import annotations

from dataclasses import dataclass, replace
import json
import hashlib
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


def prepare_case(study_dir: Path, definition: StudyDefinition, case: CaseSpec, *, yaw_map_resolver=None) -> PreparedCase:
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
    if settings.solver_kind == "yaw-aware" and yaw_map_resolver is not None:
        artifact = yaw_map_resolver(files.formulation, settings)
        settings = replace(settings, yaw_map=str(Path(artifact).resolve()))
    validate(settings, case.model, track)

    if settings.solver_kind == "yaw-aware":
        argv = yaw_runner_argv(settings, track)
    else:
        argv = []  # Point-mass options are loaded from immutable case.json.

    track_path = track.resolve()
    replay_plan = (Path(settings.plan).expanduser().resolve()
                   if settings.solver_kind == "yaw-aware" and settings.mode == "replay" else None)
    payload = {
        "id": case.id, "label": case.label, "model": case.model, "track": str(track_path),
        "settings": settings.to_json(), "solver_kind": settings.solver_kind,
        "solution_model": solution_model,
        "resolution_mapping": {"source_model": case.model, "solver_kind": settings.solver_kind,
                               "solution_model": solution_model, "reduction": ("point_mass_view_of_planar" if settings.solver_kind == "point-mass" and case.model == "planar-7dof" else "14dof.planar->point_mass_view_of_planar" if settings.solver_kind == "point-mass" and case.model == "14dof-simple" else "identity" if settings.solver_kind == "point-mass" else "14dof.planar->planar-7dof" if case.model == "14dof-simple" else "identity")},
        "source_model": case.model,
        "formulation": files.formulation.relative_to(directory).as_posix(),
        "source_formulation": files.formulation.relative_to(directory).as_posix(),
        "vehicle": files.vehicle.relative_to(directory).as_posix(), "argv": argv,
        "input_identity": {"solver_kind": settings.solver_kind, "source_model": case.model,
                           "solution_model": solution_model, "track": str(track_path),
                           "track_sha256": hashlib.sha256(track_path.read_bytes()).hexdigest(),
                           "formulation": str(files.formulation.resolve()),
                           "formulation_sha256": hashlib.sha256(files.formulation.read_bytes()).hexdigest(),
                           "yaw_map": settings.yaw_map or None,
                           "yaw_map_sha256": hashlib.sha256(Path(settings.yaw_map).read_bytes()).hexdigest() if settings.solver_kind == "yaw-aware" else None,
                            "replay_plan": str(replay_plan) if replay_plan else None,
                            "replay_plan_sha256": hashlib.sha256(replay_plan.read_bytes()).hexdigest() if replay_plan else None},
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
    identity = {}
    inputs_valid = False
    inputs = {}
    try:
        inputs = json.loads((Path(attempt) / "inputs.json").read_text())
        payload = json.loads((prepared.directory / "case.json").read_text())
        identity = payload.get("input_identity", {})
        inputs_valid = isinstance(inputs, dict) and all(inputs.get(key) == identity.get(key) for key in
            ("solver_kind", "source_model", "solution_model", "track", "track_sha256", "formulation", "formulation_sha256"))
        inputs_valid = inputs_valid and isinstance(identity, dict) and identity.get("solver_kind") == prepared.solver_kind
        inputs_valid = inputs_valid and Path(identity.get("track", "")).is_file() and hashlib.sha256(Path(identity["track"]).read_bytes()).hexdigest() == identity.get("track_sha256")
        inputs_valid = inputs_valid and Path(identity.get("formulation", "")).is_file() and hashlib.sha256(Path(identity["formulation"]).read_bytes()).hexdigest() == identity.get("formulation_sha256")
        if prepared.solver_kind == "yaw-aware":
            map_path = Path(identity.get("yaw_map", ""))
            inputs_valid = inputs_valid and inputs.get("yaw_map") == identity.get("yaw_map") and inputs.get("map_sha256") == identity.get("yaw_map_sha256")
            if "mode" in inputs:
                inputs_valid = inputs_valid and inputs.get("mode") in {"fixed", "line", "replay"}
                inputs_valid = inputs_valid and (inputs.get("mode") != "replay" or isinstance(inputs.get("plan"), str))
                inputs_valid = inputs_valid and (inputs.get("mode") == "replay" or inputs.get("plan") is None)
            inputs_valid = inputs_valid and map_path.is_file() and hashlib.sha256(map_path.read_bytes()).hexdigest() == identity.get("yaw_map_sha256")
        else:
            inputs_valid = inputs_valid and identity.get("yaw_map") is None
    except (OSError, ValueError, TypeError):
        inputs_valid = False

    def valid_npz(name, members):
        path = Path(attempt) / name
        if not path.is_file() or path.stat().st_size == 0:
            return False
        try:
            import numpy as np
            with np.load(path, allow_pickle=False) as archive:
                return set(members).issubset(archive.files) and all(np.asarray(archive[key]).size > 0 for key in members)
        except Exception:
            return False

    def matches_digest(name, digest):
        path = Path(attempt) / name
        return isinstance(digest, str) and path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == digest

    def valid_yaw_plan(path):
        try:
            import numpy as np
            from simplified_vehicle_models.planning.yaw_lap_artifact import ARRAYS, SCHEMA, _digest
            with np.load(path, allow_pickle=False) as archive:
                metadata = json.loads(str(archive["metadata"].item()))
                if metadata.get("schema") != SCHEMA:
                    return False
                arrays = {key: np.asarray(archive[key], dtype=float) for key in ARRAYS}
            return (len({array.size for array in arrays.values()}) == 1
                    and all(array.ndim == 1 and np.all(np.isfinite(array)) for array in arrays.values())
                    and _digest(arrays) == metadata.get("array_digest")
                    and metadata.get("track_identity") == inputs.get("track_identity")
                    and metadata.get("map_identity") == inputs.get("map_identity"))
        except Exception:
            return False

    def _valid_run_record(path):
        try:
            from simplified_vehicle_models.results.run_record import load_run_record
            load_run_record(path, verify_sources=False)
            return True
        except Exception:
            return False

    def valid_point_mass(path):
        try:
            import numpy as np
            with np.load(path, allow_pickle=False) as archive:
                arrays = [np.asarray(archive[k], dtype=float) for k in ("distance", "speed", "elapsed_time", "longitudinal_acceleration", "lateral_acceleration")]
            n = arrays[0].size
            return n >= 3 and all(a.ndim == 1 and a.size == n and np.all(np.isfinite(a)) for a in arrays) and np.all(np.diff(arrays[0]) > 0)
        except Exception:
            return False
    if cancelled:
        status, parent_status, parent_code = "cancelled", "Cancelled", None
    elif timed_out:
        status, parent_status, parent_code = "timed_out", "Timed_Out", 124
    elif return_code == 2:
        status, parent_status, parent_code = "rejected", "Child_Rejected", return_code
    elif return_code != 0:
        status, parent_status, parent_code = "failed", "Child_Rejected", return_code
    elif not valid_summary or not inputs_valid:
        status, parent_status, parent_code = "failed", "Child_Invalid_Artifacts", return_code
    elif (prepared.solver_kind == "yaw-aware" and summary["status"] == "planned"
          and summary.get("lap_completed") is False
          and valid_npz("plan.npz", ("metadata", "distance", "speed", "north", "east", "heading"))
          and matches_digest("plan.npz", summary.get("plan_artifact_sha256"))
          and matches_digest("plan.npz", inputs.get("plan_sha256"))
          and valid_yaw_plan(Path(attempt) / "plan.npz")):
        status, parent_status, parent_code = "planned", "Child_Planned", return_code
    elif (prepared.solver_kind == "point-mass" and summary["status"] == "completed"
          and summary.get("converged") is True and summary.get("lap_completed") is True
          and valid_npz("point_mass_plan.npz", ("distance", "speed", "elapsed_time", "longitudinal_acceleration", "lateral_acceleration"))
          and matches_digest("point_mass_plan.npz", summary.get("artifact_sha256"))
          and matches_digest("point_mass_plan.npz", inputs.get("artifact_sha256"))
          and valid_point_mass(Path(attempt) / "point_mass_plan.npz")):
        status, parent_status, parent_code = "completed", "Child_Completed", return_code
    elif (prepared.solver_kind == "yaw-aware" and summary["status"] == "completed" and summary.get("lap_completed") is True
          and summary.get("solver_validation_complete") is True
          and valid_npz("run_record.npz", ("manifest.json", "time"))
          and _valid_run_record(Path(attempt) / "run_record.npz")
          and matches_digest("run_record.npz", summary.get("artifact_sha256"))
          and matches_digest("plan.npz", summary.get("plan_artifact_sha256"))
          and matches_digest("plan.npz", inputs.get("plan_sha256"))
          and matches_digest("run_record.npz", inputs.get("run_record_sha256"))
          and valid_npz("plan.npz", ("metadata", "distance", "speed", "north", "east", "heading"))
          and valid_yaw_plan(Path(attempt) / "plan.npz")):
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
