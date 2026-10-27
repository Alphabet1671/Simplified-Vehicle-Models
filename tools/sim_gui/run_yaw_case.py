"""GUI child adapter for yaw-aware planning/replay and point-mass studies.

Invoked only through the GUI deadline-managed child process. Inputs are read
from the immutable case.json; outputs belong to one fresh attempt directory.
"""
from __future__ import annotations

import argparse
import json
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


def _yaw(case: dict, attempt: Path, settings: RunSettings, case_dir: Path) -> int:
    from tools.run import run_yaw_lap_suite as suite
    track = _inside(_resolve(case["track"]), (REPOSITORY_ROOT,), "track")
    if not track.is_file():
        raise ValueError(f"track does not exist: {track}")
    map_path = _inside(_resolve(settings.yaw_map), (REPOSITORY_ROOT,), "yaw map")
    if not map_path.is_file():
        raise ValueError(f"completed yaw-map artifact does not exist: {map_path}")
    from tools.sim_gui.settings import _validate_map
    _validate_map(map_path)
    plan = None
    if settings.plan:
        plan = _inside(_resolve(settings.plan, base=case_dir), (REPOSITORY_ROOT, case_dir), "replay plan")
    validate(settings, case["model"], track)
    model_root = (case_dir / "model").resolve()
    formulation = _inside(_resolve(case["formulation"], base=case_dir), (model_root,), "formulation")
    if not formulation.is_file():
        raise ValueError(f"case formulation does not exist: {formulation}")
    if settings.mode == "replay" and not plan:
        raise ValueError("replay requires a saved plan")
    if settings.mode != "replay" and plan is not None:
        raise ValueError("saved plan is only accepted in replay mode")
    argv = list(case["argv"])
    for flag, value in (("--track", str(track)), ("--yaw-map", str(map_path))):
        if flag not in argv:
            raise ValueError(f"immutable case argv is missing {flag}")
        argv[argv.index(flag) + 1] = value
    if "--output-dir" in argv:
        raise ValueError("case argv must not select an output directory")
    if settings.mode == "replay" and plan is None:
        raise ValueError("replay requires a saved plan")
    if plan is not None and not plan.is_file():
        raise ValueError(f"replay plan does not exist: {plan}")
    if plan is not None:
        if "--plan" not in argv:
            raise ValueError("replay case argv is missing --plan")
        argv[argv.index("--plan") + 1] = str(plan)
    # Compile the selected source first; 14DoF planar conversion uses its
    # composed planar member and does not substitute a baseline formulation.
    from simplified_vehicle_models.compiler.compile import compile_formulation
    compile_result = compile_formulation(str(formulation))
    compiled_source = compile_result.model
    source_model = case["model"]
    if source_model == "14dof-simple":
        if not isinstance(compiled_source, VerticalVehicleParameters):
            raise ValueError("14DoF case formulation did not compile as 14dof-simple")
        # The validated 14DoF compile composes the exact selected assembly's
        # planar data as .planar. The suite consumes that actual compiled value
        # without substituting a baseline formulation.
        argv.extend(("--solution-model", "14dof-simple"))
    else:
        if not isinstance(compiled_source, PlanarVehicleParameters):
            raise ValueError(f"yaw-aware source must compile as planar-7dof; received {source_model}")
    argv.extend(("--formulation", str(formulation), "--output-dir", str(attempt)))
    args = suite.parse_args(argv)
    result = int(suite.run(args))
    summary_path = attempt / "summary.json"
    try:
        summary = json.loads(summary_path.read_text())
    except (OSError, json.JSONDecodeError):
        summary = None
    plan_path = attempt / "plan.npz"
    plan_digest = hashlib.sha256(plan_path.read_bytes()).hexdigest() if plan_path.is_file() else None
    run_record = attempt / "run_record.npz"
    if isinstance(summary, dict):
        summary["plan_artifact_sha256"] = plan_digest
        summary["artifact_sha256"] = hashlib.sha256(run_record.read_bytes()).hexdigest() if run_record.is_file() else plan_digest
        _write_json(summary_path, summary)
    try:
        inputs = json.loads((attempt / "inputs.json").read_text())
    except (OSError, json.JSONDecodeError):
        inputs = {}
    if not isinstance(inputs, dict):
        inputs = {}
    manifest = getattr(compile_result, "manifest", None)
    inputs.update({
        "solver_kind": "yaw-aware", "source_model": source_model,
        "solution_model": "planar-7dof", "track": str(track), "source_formulation": str(formulation),
        "source_manifest": _manifest_source_ids(manifest),
        "yaw_map": str(map_path), "mode": settings.mode, "plan": str(plan) if plan else None,
        "compiled_source_subtype": "14dof-simple" if source_model == "14dof-simple" else "planar-7dof",
        "reduction": "14dof.planar->planar-7dof" if source_model == "14dof-simple" else "identity",
        "track_sha256": hashlib.sha256(track.read_bytes()).hexdigest(),
        "map_sha256": hashlib.sha256(map_path.read_bytes()).hexdigest(),
        "formulation_sha256": hashlib.sha256(formulation.read_bytes()).hexdigest(),
        "plan_sha256": plan_digest,
        "run_record_sha256": hashlib.sha256(run_record.read_bytes()).hexdigest() if run_record.is_file() else None,
    })
    if plan_path.is_file():
        try:
            import numpy as np
            with np.load(plan_path, allow_pickle=False) as archive:
                metadata = json.loads(str(archive["metadata"].item()))
            inputs.update(map_identity=metadata.get("map_identity"), track_identity=metadata.get("track_identity"))
        except Exception:
            pass
    _write_json(attempt / "inputs.json", inputs)
    return result


def _manifest_source_ids(manifest) -> list[str]:
    if manifest is None:
        return []
    assembly = getattr(manifest, "assembly", None)
    sources = [str(getattr(assembly, "path", ""))] if assembly is not None else []
    components = getattr(manifest, "loaded_components", ())
    if not isinstance(components, (tuple, list)):
        components = ()
    sources.extend(str(getattr(item, "path", "")) for item in components)
    return [source for source in sources if source]


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
    if settings.mode != "fixed" or settings.plan is not None or settings.yaw_map:
        raise ValueError("point-mass GUI accepts no yaw mode, saved plan, or yaw map")
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
        "track_closed": True, "yaw_map_used": False,
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


def run_case(case_dir: Path, attempt: Path) -> int:
    case_dir, attempt = Path(case_dir).resolve(), Path(attempt).resolve()
    payload = json.loads((case_dir / "case.json").read_text())
    if not isinstance(payload, dict) or payload.get("solver_kind") not in {"yaw-aware", "point-mass"}:
        raise ValueError("invalid case.json solver contract")
    attempt.mkdir(parents=True, exist_ok=False)
    settings = None
    try:
        settings = RunSettings.from_json(payload["settings"])
        if payload["solver_kind"] != settings.solver_kind:
            raise ValueError("case solver_kind conflicts with serialized settings")
        if settings.solver_kind == "yaw-aware":
            return _yaw(payload, attempt, settings, case_dir)
        return _point_mass(payload, attempt, settings, case_dir)
    except Exception as error:
        inputs = {"status": "failed", "case": str(case_dir)}
        if settings is not None and settings.solver_kind == "yaw-aware":
            inputs["yaw_map"] = settings.yaw_map
            inputs["track"] = payload.get("track")
            inputs["formulation"] = payload.get("formulation")
        elif settings is not None:
            inputs["source_model"] = payload.get("model")
            inputs["solution_model"] = payload.get("solution_model")
            inputs["resolution_mapping"] = payload.get("resolution_mapping")
        _write_json(attempt / "inputs.json", inputs)
        _write_json(attempt / "summary.json", {"status": "failed", "exception": type(error).__name__,
                                                   "message": str(error)})
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
