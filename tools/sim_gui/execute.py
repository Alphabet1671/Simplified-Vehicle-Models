"""Prepare, execute, and finish one simulation GUI case."""
from __future__ import annotations

from dataclasses import dataclass
import json
import hashlib
from pathlib import Path
import sys
import time
from typing import Any

from simplified_vehicle_models.compiler.compile import compile_formulation

from . import REPOSITORY_ROOT
from .settings import solution_model_for, validate
from .source_identity import execution_source_identity, identity_digest, combined_source_closure
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
    solver_kind: str = "forward-driver"


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
    if not (settings.solver_kind == "forward-driver" and case.model == "14dof-simple") and solution_model != case.model:
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

    argv = []  # Workflows load options from immutable case.json.

    track_path = track.resolve()
    forward_source_closure = None
    if settings.solver_kind == "forward-driver":
        compiled_for_identity = compile_formulation(str(files.formulation))
        closure_paths = {Path(files.formulation).resolve(), Path(compiled_for_identity.manifest.assembly.path).resolve()}
        closure_paths.update(Path(item.path).resolve() for item in compiled_for_identity.manifest.loaded_components)
        closure_paths.add(track.resolve())
        forward_source_closure = {str(path): "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()
                                  for path in sorted(closure_paths)}
    payload = {
        "id": case.id, "label": case.label, "model": case.model, "track": str(track_path),
        "settings": settings.to_json(), "solver_kind": settings.solver_kind,
        "solution_model": solution_model,
        "resolution_mapping": {"source_model": case.model, "solver_kind": settings.solver_kind,
                               "solution_model": solution_model, "reduction": ("point_mass_view_of_planar" if settings.solver_kind == "point-mass" and case.model == "planar-7dof" else "14dof.planar->point_mass_view_of_planar" if settings.solver_kind == "point-mass" and case.model == "14dof-simple" else "identity") if settings.solver_kind == "point-mass" else "identity"},
        "source_model": case.model,
        "formulation": files.formulation.relative_to(directory).as_posix(),
        "source_formulation": files.formulation.relative_to(directory).as_posix(),
        "vehicle": files.vehicle.relative_to(directory).as_posix(), "argv": argv,
        "input_identity": {"solver_kind": settings.solver_kind, "source_model": case.model,
                           "solution_model": solution_model, "track": str(track_path),
                           "track_sha256": hashlib.sha256(track_path.read_bytes()).hexdigest(),
                           "formulation": str(files.formulation.resolve()),
                           "formulation_sha256": hashlib.sha256(files.formulation.read_bytes()).hexdigest(),

                           "component_source_hashes": forward_source_closure},
    }
    _atomic_json(directory / "case.json", payload)
    timeout = (settings.point_mass_duration if settings.solver_kind == "point-mass"
               else settings.forward_wall_seconds + 60 if settings.solver_kind == "forward-driver"
               else settings.forward_wall_seconds + 60)
    timeout = min(1800.0, timeout)
    if settings.solver_kind == "forward-driver":
        from .settings import forward_driver_config
        payload["driver_config"] = forward_driver_config(settings)
        resolved_mode = payload["driver_config"]["longitudinal_mode"]
        if resolved_mode in ("physical-reachable-v1", "hybrid-drive-brake-v1") or settings.forward_transient_corner_planning:
            payload["input_identity"]["authority_source_closure_sha256"] = identity_digest(
                payload["input_identity"]["component_source_hashes"])
            payload["input_identity"]["execution_source_hashes"] = execution_source_identity()
            payload["input_identity"]["execution_source_identity_sha256"] = identity_digest(
                payload["input_identity"]["execution_source_hashes"])
            payload["input_identity"]["complete_source_closure"] = combined_source_closure(
                payload["input_identity"]["component_source_hashes"],
                payload["input_identity"]["execution_source_hashes"])
            payload["input_identity"]["complete_source_closure_sha256"] = identity_digest(
                payload["input_identity"]["complete_source_closure"])
            payload["input_identity"]["source_identity_manifest_sha256"] = identity_digest({
                rel: "required" for rel in sorted(payload["input_identity"]["execution_source_hashes"])})

        if resolved_mode == "hybrid-drive-brake-v1" and case.model != "14dof-simple":
            raise ValueError("hybrid-drive-brake-v1 requires the full vertical 14DoF plant")
        if resolved_mode == "physical-reachable-v1" and case.model != "14dof-simple":
            raise ValueError("physical brake forecasting requires the full vertical 14DoF plant")
        payload["input_identity"].update(settings=settings.to_json(), driver_config=payload["driver_config"],
            settings_sha256=hashlib.sha256(json.dumps(settings.to_json(), sort_keys=True,
                separators=(",", ":"), allow_nan=False).encode()).hexdigest())
        _atomic_json(directory / "case.json", payload)
    duration = settings.forward_duration if settings.solver_kind == "forward-driver" else settings.point_mass_duration
    return PreparedCase(directory, files.formulation, files.vehicle, argv, model=solution_model,
                        timeout_seconds=timeout, duration=duration, solver_kind=settings.solver_kind)


def validate_case(prepared: PreparedCase) -> str | None:
    try:
        compiled = compile_formulation(str(prepared.formulation)).model
        if prepared.solver_kind == "forward-driver":
            from simplified_vehicle_models.formulations.vertical_14dof import VerticalVehicleParameters
            if prepared.model == "14dof-simple" and not isinstance(compiled, VerticalVehicleParameters):
                raise ValueError("forward-driver selected variant did not compile to vertical 14DoF")
            if prepared.model == "planar-7dof" and not hasattr(compiled, "yaw_inertia"):
                raise ValueError("forward-driver selected variant did not compile to planar 7DoF")
        if prepared.model == "point-mass":
            from simplified_vehicle_models.parameters.vehicle import PointMassVehicleParameters
            from simplified_vehicle_models.formulations.vertical_14dof import VerticalVehicleParameters
            if not isinstance(compiled, (PointMassVehicleParameters, VerticalVehicleParameters)) and not hasattr(compiled, "yaw_inertia"):
                raise ValueError(f"unsupported compiled source type for point-mass reduction: {type(compiled).__name__}")
    except Exception as error:
        return f"{type(error).__name__}: {error}"
    return None


def new_attempt(prepared: PreparedCase) -> Path:
    """Allocate the fresh attempt directory owned by the parent runner."""
    attempt = prepared.directory / f"attempt_{time.time_ns()}"
    attempt.mkdir(parents=True, exist_ok=False)
    return attempt


def child_command(prepared: PreparedCase, attempt: Path) -> list[str]:
    """Run the dedicated adapter that resolves immutable case inputs."""
    return [sys.executable, "-u", str(Path(__file__).with_name("run_case.py")),
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
        if prepared.solver_kind == "forward-driver":
            inputs_valid = inputs_valid and identity.get("settings") == payload.get("settings")
            inputs_valid = inputs_valid and inputs.get("settings") == payload.get("settings")
            inputs_valid = inputs_valid and inputs.get("driver_config") == identity.get("driver_config") == payload.get("driver_config")
            inputs_valid = inputs_valid and inputs.get("settings_sha256") == identity.get("settings_sha256")
            inputs_valid = inputs_valid and inputs.get("component_source_hashes") == identity.get("component_source_hashes")
        inputs_valid = inputs_valid and Path(identity.get("track", "")).is_file() and hashlib.sha256(Path(identity["track"]).read_bytes()).hexdigest() == identity.get("track_sha256")
        inputs_valid = inputs_valid and Path(identity.get("formulation", "")).is_file() and hashlib.sha256(Path(identity["formulation"]).read_bytes()).hexdigest() == identity.get("formulation_sha256")
        if prepared.solver_kind == "forward-driver":
            inputs_valid = inputs_valid and inputs.get("formulation") == identity.get("formulation")
            inputs_valid = inputs_valid and inputs.get("track") == identity.get("track")
            inputs_valid = inputs_valid and inputs.get("formulation_sha256") == identity.get("formulation_sha256")
            inputs_valid = inputs_valid and inputs.get("settings") == identity.get("settings")
            inputs_valid = inputs_valid and inputs.get("driver_config") == identity.get("driver_config")
            inputs_valid = inputs_valid and inputs.get("track_sha256") == identity.get("track_sha256")
            inputs_valid = inputs_valid and inputs.get("completed_forward_result") is True
            inputs_valid = inputs_valid and inputs.get("lap_completed") is True
            inputs_valid = inputs_valid and inputs.get("midpoint_sample_validation_complete") is True
            inputs_valid = inputs_valid and type(inputs.get("accepted_states_validated")) is int
            inputs_valid = inputs_valid and type(inputs.get("dense_samples_validated")) is int
            forward_summary = json.loads(summary_path.read_text())
            inputs_valid = inputs_valid and inputs.get("source_hashes") == forward_summary.get("source_hashes")
            inputs_valid = inputs_valid and inputs.get("source_hashes") == identity.get("component_source_hashes")
            inputs_valid = inputs_valid and inputs.get("accepted_policy_sha256") == forward_summary.get("accepted_policy_sha256")
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

    def valid_jsonl(name, expected_rows):
        path = Path(attempt) / name
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
            return (type(expected_rows) is int and len(lines) == expected_rows
                    and all(isinstance(json.loads(line), dict) for line in lines))
        except (OSError, UnicodeDecodeError, json.JSONDecodeError):
            return False

    def matches_digest(name, digest):
        path = Path(attempt) / name
        return isinstance(digest, str) and path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == digest

    def _valid_run_record(path, *, verify_sources=False):
        try:
            from simplified_vehicle_models.results.run_record import load_run_record
            source_root = Path(attempt).parents[2] if Path(attempt).parents[1].name == "cases" else REPOSITORY_ROOT
            record = load_run_record(path, verify_sources=verify_sources, root=source_root)
            if verify_sources:
                return (record.manifest.sources == inputs.get("source_hashes")
                        and bool(record.manifest.sources))
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
    elif (prepared.solver_kind == "forward-driver" and summary.get("status") == "completed"
          and inputs.get("lap_completed") is True
          and inputs.get("midpoint_sample_validation_complete") is True
          and inputs.get("completed_forward_result") is True
          and summary.get("lap_completed") is True and summary.get("midpoint_sample_validation_complete") is True
          and type(summary.get("accepted_steps")) is int and summary["accepted_steps"] > 0
          and type(summary.get("accepted_states_validated")) is int
          and summary.get("accepted_states_validated") == summary["accepted_steps"] + 1
          and type(summary.get("dense_samples_validated")) is int
          and summary.get("dense_samples_validated") == summary.get("midpoint_samples")
          and type(summary.get("midpoint_samples")) is int
          and isinstance(summary.get("validation_counts"), dict)
          and type(summary.get("accepted_policy_rows")) is int
          and summary.get("accepted_policy_rows") == summary.get("accepted_steps") + 1
          and type(summary["validation_counts"].get("accepted_states_validated")) is int
          and type(summary["validation_counts"].get("dense_probes_validated")) is int
          and valid_jsonl("accepted_policy.jsonl", summary.get("accepted_steps") + 1)
          and summary["validation_counts"].get("accepted_states_validated") == summary["accepted_states_validated"]
          and summary["validation_counts"].get("dense_probes_validated") == summary["dense_samples_validated"]
          and summary["midpoint_samples"] >= summary["accepted_steps"]
          and summary.get("settings_sha256") == identity.get("settings_sha256")
          and summary.get("forward_driver_config") == identity.get("driver_config")
          and valid_npz("run_record.npz", ("manifest.json", "time"))
          and _valid_run_record(Path(attempt) / "run_record.npz", verify_sources=True)
          and inputs.get("accepted_policy_file") == "accepted_policy.jsonl"
          and type(inputs.get("accepted_policy_rows")) is int
          and inputs.get("accepted_policy_rows") == summary.get("accepted_steps") + 1
          and matches_digest("accepted_policy.jsonl", inputs.get("accepted_policy_sha256"))
          and inputs.get("source_hashes") == summary.get("source_hashes")
          and inputs.get("source_hashes") == identity.get("component_source_hashes")
          and isinstance(summary.get("source_hashes"), dict)
          and len(summary["source_hashes"]) >= 3
          and matches_digest("accepted_policy.jsonl", summary.get("accepted_policy_sha256"))
          and summary.get("accepted_policy_rows") == inputs.get("accepted_policy_rows")
          and inputs.get("accepted_policy_rows") == summary.get("accepted_steps") + 1
          and matches_digest("run_record.npz", summary.get("run_record_sha256"))
          and matches_digest("run_record.npz", inputs.get("run_record_sha256"))
          and valid_npz("envelope_arrays.npz", ("distance", "curvature", "corner_speed", "entry_speed", "deceleration", "binding_index"))
          and matches_digest("envelope_arrays.npz", summary.get("envelope_arrays_sha256"))):
        status, parent_status, parent_code = "completed", "Child_Completed", return_code
    elif (prepared.solver_kind == "point-mass" and summary["status"] == "completed"
          and summary.get("converged") is True and summary.get("lap_completed") is True
          and valid_npz("point_mass_plan.npz", ("distance", "speed", "elapsed_time", "longitudinal_acceleration", "lateral_acceleration"))
          and matches_digest("point_mass_plan.npz", summary.get("artifact_sha256"))
          and matches_digest("point_mass_plan.npz", inputs.get("artifact_sha256"))
          and valid_point_mass(Path(attempt) / "point_mass_plan.npz")):
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
    envelope_arrays = Path(attempt) / "envelope_arrays.npz"
    artifacts = {name: (Path(attempt) / name).relative_to(study_root).as_posix()
                 for name in ("inputs.json", "plan.npz", "run_record.npz", "point_mass_plan.npz", "trace.csv", "envelope_arrays.npz", "accepted_policy.json", "accepted_policy.jsonl", "progress.json")
                 if (Path(attempt) / name).is_file()}
    return {"status": status, "attempt": rel_attempt,
            "run_record": record.relative_to(study_root).as_posix() if record.exists() else None,
            "plan": plan.relative_to(study_root).as_posix() if plan.exists() else None,
            "point_mass_plan": point_plan.relative_to(study_root).as_posix() if point_plan.exists() else None,
            "envelope_arrays": envelope_arrays.relative_to(study_root).as_posix() if envelope_arrays.exists() else None,
            "artifacts": artifacts, "summary": summary}


def _argument(argv: list[str], name: str) -> str:
    return argv[argv.index(name) + 1]


def _timeout_from_argv(argv: list[str]) -> float:
    return float(_argument(argv, "--timeout-seconds")) if "--timeout-seconds" in argv else 300.0


def _model_from_argv(argv: list[str]) -> str:
    return _argument(argv, "--model") if "--model" in argv else "planar-7dof"
