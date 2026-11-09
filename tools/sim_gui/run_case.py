"""GUI child adapter for forward-driver and point-mass studies.

Invoked only through the GUI deadline-managed child process. Inputs are read
from the immutable case.json; outputs belong to one fresh attempt directory.
"""
from __future__ import annotations

import argparse
import json
import os
import stat
import hashlib
from pathlib import Path

from tools.sim_gui import REPOSITORY_ROOT
from tools.sim_gui.settings import RunSettings, validate
from simplified_vehicle_models.parameters.vehicle import PlanarVehicleParameters, PointMassVehicleParameters
from simplified_vehicle_models.formulations.vertical_14dof import VerticalVehicleParameters


def _resolve(value: str | Path, *, base: Path = REPOSITORY_ROOT) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (base / path).resolve()


def _inside(path: Path, roots: tuple[Path, ...], name: str) -> Path:
    resolved = path.resolve()
    if not any(resolved == root or root in resolved.parents for root in roots):
        raise ValueError(f"{name} escapes its declared input location: {path}")
    return resolved


def _write_json(path: Path, data: dict) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=2, sort_keys=True, allow_nan=False) + "\n")
    tmp.replace(path)


def _point_mass(case: dict, attempt: Path, settings: RunSettings, case_dir: Path) -> int:
    import numpy as np
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
    from simplified_vehicle_models.results.traces import LapMode
    from simplified_vehicle_models.solvers.forward_backward import solve_lap
    from simplified_vehicle_models.tracks.profile import load_track

    track_path = _inside(_resolve(case["track"]), (REPOSITORY_ROOT,), "track")
    if not track_path.is_file():
        raise ValueError(f"track does not exist: {track_path}")
    track = load_track(track_path)
    if not track.is_closed:
        raise ValueError("point-mass GUI supports closed tracks only; open/standing-start courses are unsupported")
    if (isinstance(settings.point_mass_duration, bool) or not isinstance(settings.point_mass_duration, (int, float))
            or not np.isfinite(settings.point_mass_duration) or settings.point_mass_duration <= 0):
        raise ValueError("point-mass duration must be finite and positive")
    formulation = _inside(_resolve(case["formulation"], base=case_dir),
                           ((case_dir / "model").resolve(),), "formulation")
    compiled = compile_formulation(str(formulation)).model
    source_model = case["model"]
    if source_model in {"planar-7dof", "14dof-simple"}:
        # 14DoF's explicitly composed planar member is the exact selected
        # assembly/configuration; reduce that member, not a baseline model.
        planar = compiled.planar if isinstance(compiled, VerticalVehicleParameters) else compiled
        if not isinstance(planar, PlanarVehicleParameters):
            raise ValueError(f"full-model source lacks supported planar member: {source_model}")
        point_mass = point_mass_view_of_planar(planar)
        reduction = "14dof.planar->point_mass_view_of_planar" if isinstance(compiled, VerticalVehicleParameters) else "point_mass_view_of_planar"
    elif source_model == "point-mass" and isinstance(compiled, PointMassVehicleParameters):
        point_mass = compiled
        reduction = "identity"
    else:
        raise ValueError(f"compiled source formulation/model mismatch for {source_model}: {type(compiled).__name__}")

    from simplified_vehicle_models.tracks.types import TrackProfile
    distance = np.asarray(track.distance, dtype=float)
    radius = np.asarray(track.radius, dtype=float)
    if distance.size < 3 or np.any(np.diff(distance) <= 0) or not np.all(np.isfinite(radius)):
        raise ValueError("point-mass track must have at least three finite increasing nodes")
    if np.any(radius <= 0):
        raise ValueError("point-mass track radii must be positive finite magnitudes")
    solver_track = TrackProfile(distance=distance, segment_length=np.diff(distance), radius=radius,
                                curvature=np.asarray(track.curvature, dtype=float), is_closed=True,
                                title=track.title)
    lap = solve_lap(point_mass, solver_track, LapMode.FLYING)
    if not lap.converged:
        raise RuntimeError("point-mass forward/backward solver did not converge")
    np.savez_compressed(attempt / "point_mass_plan.npz", distance=distance, speed=lap.speed,
                        elapsed_time=lap.elapsed_time,
                        longitudinal_acceleration=lap.longitudinal_acceleration,
                        lateral_acceleration=lap.lateral_acceleration)
    track_digest = hashlib.sha256(track_path.read_bytes()).hexdigest()
    formulation_digest = hashlib.sha256(formulation.read_bytes()).hexdigest()
    plan_digest = hashlib.sha256((attempt / "point_mass_plan.npz").read_bytes()).hexdigest()
    _write_json(attempt / "inputs.json", {
        "solver_kind": "point-mass", "source_model": source_model,
        "solution_model": "point-mass", "track": str(track_path), "track_sha256": track_digest,
        "formulation": str(formulation), "formulation_sha256": formulation_digest,
        "resolution_mapping": case["resolution_mapping"],
        "reduction": reduction, "source_formulation": str(formulation),
        "source_manifest": _manifest_source_ids(getattr(compiled, "manifest", None)),
        "track_closed": True,
        "semantics": "closed flying-lap power investigation; no energy/stint semantics",
        "artifact": "point_mass_plan.npz", "artifact_sha256": plan_digest,
    })
    _write_json(attempt / "summary.json", {
        "status": "completed", "lap_time_s": float(lap.lap_time),
        "resolution_mapping": case["resolution_mapping"],
        "converged": bool(lap.converged), "iterations": int(lap.iterations),
        "lap_completed": True, "source_model": source_model,
        "solution_model": "point-mass", "reduction": reduction,
        "artifact": "point_mass_plan.npz", "artifact_sha256": plan_digest,
    })
    return 0


def _validate_child_attempt(case_dir: Path, attempt: Path) -> None:
    """Admit only a fresh attempt under its case, optionally with parent run.log."""
    case_dir = Path(case_dir).resolve()
    attempt = Path(attempt)
    if attempt.is_symlink():
        raise ValueError("child attempt must not be a symlink")
    resolved_attempt = attempt.resolve()
    try:
        resolved_attempt.relative_to(case_dir)
    except ValueError as error:
        raise ValueError("child attempt must resolve inside its case directory") from error
    attempt = resolved_attempt
    if not attempt.exists():
        attempt.mkdir(parents=True, exist_ok=False)
        return
    if not attempt.is_dir():
        raise ValueError("child attempt must be a real directory")
    entries = list(attempt.iterdir())
    if not entries:
        return
    if len(entries) != 1 or entries[0].name != "run.log":
        raise ValueError("child attempt must be fresh except for parent run.log")
    log = entries[0]
    if log.is_symlink() or not log.is_file() or not stat.S_ISREG(log.lstat().st_mode):
        raise ValueError("parent run.log must be a regular file")


def run_case(case_dir: Path, attempt: Path) -> int:
    case_dir = Path(case_dir).resolve()
    attempt = Path(os.path.abspath(Path(attempt)))
    payload = json.loads((case_dir / "case.json").read_text())
    if not isinstance(payload, dict):
        raise ValueError("invalid case.json solver contract")
    if payload.get("solver_kind") == "yaw-aware":
        raise ValueError("retired yaw workflow in case.json; select forward-driver or point-mass")
    if payload.get("solver_kind") not in {"point-mass", "forward-driver"}:
        raise ValueError("invalid case.json solver contract")
    _validate_child_attempt(case_dir, attempt)
    settings = None
    try:
        settings = RunSettings.from_json(payload["settings"])
        if payload["solver_kind"] != settings.solver_kind:
            raise ValueError("case solver_kind conflicts with serialized settings")
        if settings.solver_kind == "forward-driver":
            from tools.sim_gui.forward_execute import run as run_forward
            return run_forward(payload, attempt, settings, case_dir)
        return _point_mass(payload, attempt, settings, case_dir)
    except Exception as error:
        inputs = {"status": "failed", "case": str(case_dir)}
        if settings is not None and settings.solver_kind == "forward-driver":
            if (attempt / "summary.json").is_file():
                return 4
            inputs.update({"solver_kind": "forward-driver", "source_model": payload.get("model"),
                "solution_model": payload.get("solution_model"), "formulation": payload.get("formulation"),
                "track": payload.get("track"), "settings": settings.to_json(),
                "driver_config": payload.get("driver_config"), "input_identity": payload.get("input_identity")})
        elif settings is not None:
            inputs["source_model"] = payload.get("model")
            inputs["solution_model"] = payload.get("solution_model")
            inputs["resolution_mapping"] = payload.get("resolution_mapping")
        _write_json(attempt / "inputs.json", inputs)
        failure = {"status": "failed", "exception": type(error).__name__, "message": str(error)}
        if settings is not None and settings.solver_kind == "forward-driver":
            # The backend writes its own progress-prefix/error diagnostics; do not
            # replace a rich partial failure summary with a generic traceback.
            return 4
        _write_json(attempt / "summary.json", failure)
        return 4


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case-dir", type=Path, required=True)
    parser.add_argument("--child-attempt", type=Path, required=True)
    args = parser.parse_args(argv)
    # The GUI parent owns the sole process-group watchdog and kills this entire
    # session on timeout. Never spawn a nested watchdog/session here.
    return run_case(args.case_dir, args.child_attempt)


if __name__ == "__main__":
    raise SystemExit(main())
