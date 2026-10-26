"""Run a diagnostic yaw-aware line plan through the planar forward driver.

This runner is diagnostic only. It does not establish minimum time, TV/FM U
validation, measured-vehicle validity, full U3 qualification, footprint or
between-step containment, periodicity, or refinement.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from collections.abc import Mapping
from pathlib import Path
import time

import numpy as np

PLANAR_FORMULATION = Path("models/formulations/iem24_murphy_real_planar_7dof.xml")
DIAGNOSTIC_NONCLAIMS = {
    "minimum_time": False,
    "TV_FMU_validation": False,
    "measured_vehicle_validity": False,
    "full_U3_qualification": False,
    "footprint_containment": False,
    "between_step_containment": False,
    "periodicity": False,
    "refinement": False,
}


def prepare_output(path):
    """Create a new output directory; never reuse/overwrite an existing one."""
    path = Path(path)
    path.mkdir(parents=True, exist_ok=False)
    return path


def anchored_axis(lower: float, upper: float, count: int) -> np.ndarray:
    """Build a finite ordered axis that preserves its base count and exact zero.

    Anchoring may add one element when the requested linspace does not already
    contain zero; callers should record both ``count`` and ``len(axis)``.
    """
    if not lower < 0 < upper or count < 2:
        raise ValueError('axis must straddle zero and count must be >= 2')
    return np.unique(np.r_[np.linspace(lower, upper, count), 0.0])


def physical_map_speed_limit(point_mass_vehicle, *, search_limit_mps=None):
    """Return a finite upper speed for the map from straight-line drive capability.

    The search uses the point-mass vehicle's actual maximum positive
    acceleration envelope at zero curvature. The drivetrain's physical speed
    limit bounds the search; a finite caller limit protects against malformed
    models and is never treated as a fallback domain.
    """
    from simplified_vehicle_models.formulations.point_mass import (
        maximum_acceleration_limit, maximum_vehicle_speed,
    )

    physical_limit = float(maximum_vehicle_speed(point_mass_vehicle))
    if search_limit_mps is not None:
        if not math.isfinite(search_limit_mps) or search_limit_mps <= 0:
            raise ValueError("search_limit_mps must be finite and positive")
        physical_limit = min(physical_limit, search_limit_mps)
    if not math.isfinite(physical_limit) or physical_limit <= 0:
        raise ValueError("point-mass drivetrain has no finite positive speed limit")

    def has_positive_drive(speed):
        return maximum_acceleration_limit(point_mass_vehicle, speed, 0.) > 1.e-7

    if not has_positive_drive(0.):
        raise ValueError("point-mass vehicle has no feasible positive-drive straight-line speed")
    if has_positive_drive(physical_limit):
        # The motor speed ceiling itself is a real physical bound, not an
        # arbitrary map cap. It also handles models whose drive remains
        # feasible up to the no-load speed limit.
        return physical_limit
    low, high = 0., physical_limit
    for _ in range(60):
        middle = (low + high) / 2.
        if has_positive_drive(middle):
            low = middle
        else:
            high = middle
    if low <= 0 or not math.isfinite(low):
        raise ValueError("could not derive a finite positive-drive speed domain")
    return low


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track", type=Path, required=True, help="signed XY track XML with left/right edges")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--model", choices=("planar-7dof", "vertical-14dof"), default="planar-7dof")
    parser.add_argument("--map", choices=("no-tv", "physical-tv"), default="no-tv")
    parser.add_argument("--yaw-speed-points", type=int, default=8)
    parser.add_argument("--yaw-ax-points", type=int, default=5)
    parser.add_argument("--yaw-ay-points", type=int, default=9)
    parser.add_argument("--maximum-speed", type=float, default=None,
                        help="optional explicit map/profile speed limiter in m/s")
    parser.add_argument("--lateral-margin", type=float, default=.15)
    parser.add_argument("--control-spacing", type=float, default=60.,
                        help="yaw-line spline control spacing in metres")
    parser.add_argument("--planning-grid-spacing", type=float, default=8.,
                        help="physical yaw-profile station spacing in metres")
    parser.add_argument("--planner-iterations", type=int, default=2,
                        help="bounded Powell iterations per corner complex")
    parser.add_argument("--planner-passes", type=int, default=1,
                        help="bounded outer line/profile passes")
    parser.add_argument("--preview-seconds", type=float, default=.15)
    parser.add_argument("--heading-gain", type=float, default=1.5)
    parser.add_argument("--lateral-gain", type=float, default=1.5)
    parser.add_argument("--speed-gain", type=float, default=2.)
    parser.add_argument("--duration", type=float, default=300.)
    parser.add_argument("--wall-seconds", type=float, default=290.)
    parser.add_argument("--maximum-step", type=float, default=.02)
    parser.add_argument("--rtol", type=float, default=1.e-6)
    parser.add_argument("--method", choices=("Radau", "BDF"), default="Radau")
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--record-precision", choices=("float32", "float64"), default="float32")
    parser.add_argument("--frame-rate-note", default="")
    from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings
    defaults = CoupledProfileSettings()
    parser.add_argument("--profile-solver", choices=("coupled", "legacy"), default="coupled")
    parser.add_argument("--profile-wall-seconds", type=float, default=defaults.max_wall_seconds)
    parser.add_argument("--profile-max-refinements", type=int, default=defaults.max_refinement_rounds)
    parser.add_argument("--profile-max-candidates", type=int, default=defaults.max_candidates_per_station)
    parser.add_argument("--profile-max-seam-anchors", type=int, default=defaults.max_seam_anchors)
    parser.add_argument("--plan-only", action="store_true", help="stop after planning; no forward simulation")
    parser.add_argument("--skip-preflight", action="store_true", help="do not block on preflight map holes")
    from tools.run.yaw_map_options import add_yaw_map_options
    add_yaw_map_options(parser)
    args = parser.parse_args(argv)
    if not math.isfinite(args.profile_wall_seconds) or args.profile_wall_seconds <= 0:
        parser.error("--profile-wall-seconds must be finite and positive")
    for name in ("profile_max_refinements", "profile_max_candidates", "profile_max_seam_anchors"):
        if getattr(args, name) < 1:
            parser.error(f"--{name.replace('_','-')} must be at least 1")
    for name in ("lateral_margin", "control_spacing", "planning_grid_spacing",
                 "duration", "wall_seconds", "maximum_step", "rtol", "preview_seconds",
                 "heading_gain", "lateral_gain", "speed_gain"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name.replace('_','-')} must be finite and positive")
    if args.maximum_speed is not None and (not math.isfinite(args.maximum_speed) or args.maximum_speed <= 0):
        parser.error("--maximum-speed must be finite and positive")
    if args.lateral_margin > 1:
        parser.error("--lateral-margin must be at most 1")
    for name in ("yaw_speed_points", "yaw_ax_points", "yaw_ay_points"):
        if getattr(args, name) < 2:
            parser.error(f"--{name.replace('_','-')} must be at least 2")
    for name in ("planner_iterations", "planner_passes"):
        if getattr(args, name) < 1:
            parser.error(f"--{name.replace('_','-')} must be at least 1")
    if args.model != "planar-7dof":
        parser.error("vertical-14dof is not implemented; this initial runner is planar only")
    if args.map != "no-tv":
        parser.error("physical-tv requires explicit calibrated TV limits; only no-tv is implemented")
    return args


def make_record_sink(plant, optimized_path, output, precision="float32"):
    """Construct the recorder against the optimized path, never the track centerline."""
    from simplified_vehicle_models.results.plant_channels import RunRecordSink
    return RunRecordSink(plant, optimized_path, output, precision=precision)


def _jsonable(value):
    """Convert to strict JSON values; nonfinite floats become null."""
    if isinstance(value, Mapping):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, (tuple, list, set, frozenset)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.generic):
        return _jsonable(value.item())
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if hasattr(value, "value") and type(value).__module__.endswith("yaw_profile_map"):
        return value.value
    if hasattr(value, "__dataclass_fields__"):
        return _jsonable(asdict(value))
    return value


def write_json(path, document):
    """Write strict JSON (no NaN/Infinity); nonfinite values are already null."""
    Path(path).write_text(json.dumps(_jsonable(document), indent=2, allow_nan=False) + "\n")


def file_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def array_sha256(*arrays):
    digest = hashlib.sha256()
    for array in arrays:
        array = np.ascontiguousarray(np.asarray(array))
        digest.update(str(array.dtype).encode()); digest.update(str(array.shape).encode())
        digest.update(array.tobytes())
    return digest.hexdigest()


def failure_summary(stage, causes, started, **extra):
    return {"status": "failed", "failure_stage": stage, "causes": causes,
            "lap_completed": False, "qualified": False, "accepted": False, "U3_qualified": False,
            "elapsed_setup_seconds": time.perf_counter() - started,
            "diagnostic_only": DIAGNOSTIC_NONCLAIMS, **extra}


def record_metadata(args, track, vehicle, capability, plan, result):
    """Build deterministic provenance metadata for a recorded diagnostic run."""
    import platform
    import scipy

    from simplified_vehicle_models.results.plant_channels import source_hashes

    def identity(path):
        path = Path(path)
        try:
            return str(path.resolve().relative_to(Path.cwd().resolve()))
        except ValueError:
            return str(path)

    formulation = PLANAR_FORMULATION
    track_path = Path(args.track)
    sources = [formulation, track_path]
    data_path = None
    try:
        import xml.etree.ElementTree as ElementTree
        root = ElementTree.parse(track_path).getroot()
        data_paths = [(track_path.parent / node.get("ref")).resolve()
                      for node in root.findall("./data") if node.get("ref")]
    except (OSError, ElementTree.ParseError):
        data_paths = []
    for candidate in (getattr(track, "source_file", None), getattr(track, "data_file", None),
                      getattr(track, "csv_path", None)):
        if candidate:
            data_paths.append(Path(candidate))
    for data_path in data_paths:
        if not data_path.is_absolute():
            data_path = track_path.parent / data_path
        sources.append(data_path)
    settings = plan.settings if getattr(plan, "settings", None) is not None else None
    speed_domain = float(capability.speed[-1])
    speed_source = ("explicit_cli_limit" if args.maximum_speed is not None
                    else "physical_straight_line_envelope")
    return _jsonable({
        "producer": {"tool": "tools/run/run_yaw_aware_forward_driver.py",
                     "python": platform.python_version(), "numpy": np.__version__,
                     "scipy": scipy.__version__},
        "model": {"model": args.model, "plant_class": "PlanarForwardPlant",
                  "formulation_xml": identity(formulation), "mechanical_dof": 7,
                  "vehicle_name": getattr(vehicle, "name", None),
                  "yaw_profile": getattr(args, "yaw_profile_provenance", None)},
        "sources": source_hashes(sources),
        "track": {"file": identity(track_path), "reference": "yaw-aware-optimized-line",
                  "source_path_identity": identity(track_path),
                  "optimized_path_identity": "line.csv",
                  "optimized_path_sha256": getattr(args, "line_csv_sha256", None)},
        "yaw_capability": {"source_kind": capability.source_kind,
                           "source_identity": capability.source_identity,
                           "yaw_inertia": float(capability.yaw_inertia),
                           "grid": {"speed": capability.speed.tolist(),
                                    "longitudinal_acceleration": capability.longitudinal_acceleration.tolist(),
                                    "lateral_acceleration": capability.lateral_acceleration.tolist()}},
        "speed_domain_source": speed_source, "speed_domain_mps": speed_domain,
        "planning": {"status": plan.status, "settings": asdict(settings) if settings is not None else None,
                     "settings_used": {"maximum_speed": speed_domain,
                         "lateral_margin": args.lateral_margin, "control_spacing": args.control_spacing,
                         "planning_grid_spacing": args.planning_grid_spacing,
                         "planner_iterations": args.planner_iterations, "planner_passes": args.planner_passes},
                     "line_shape_metrics": dict(plan.shape_metrics),
                     "predicted_lap_time_seconds": float(plan.profile.lap_time)},
        "integrator": {"method": args.method, "rtol": args.rtol, "atol": args.rtol * .01,
                       "maximum_step": args.maximum_step, "duration": args.duration,
                       "wall_seconds": args.wall_seconds, "actual_wall_seconds": float(result.wall_seconds)},
        "segment": {"duration_s": float(result.time[-1] - result.time[0]) if len(result.time) else 0.,
                    "lap_completed": bool(result.lap_completed)},
        "diagnostic_only": DIAGNOSTIC_NONCLAIMS,
    })


def run(args):
    """Compile, plan, drive and emit diagnostic artifacts into a fresh directory."""
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
    from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityGrid, build_physical_yaw_map
    from simplified_vehicle_models.planning.yaw_aware_line import YawLineSettings, optimize_yaw_aware_line
    from simplified_vehicle_models.planning.yaw_aware_record_path import path_coordinates_from_yaw_line
    from simplified_vehicle_models.solvers.forward_driver import PreviewDriver, solve_forward
    from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
    from simplified_vehicle_models.solvers.yaw_aware_driver import trajectory_corridor_containment
    from simplified_vehicle_models.tracks.corridor import SIGNED_POLICY, corridor_from_track
    from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve
    from simplified_vehicle_models.tracks.profile import load_track

    output = prepare_output(args.output_dir)
    started = time.perf_counter()
    vehicle = compile_formulation(str(PLANAR_FORMULATION)).model
    track = load_track(args.track)
    if track.path_coordinates is None:
        raise ValueError("track must provide signed XY centerline geometry")
    if not track.has_edges:
        raise ValueError("track must carry signed left and right edge widths")
    reference = ReferenceCurve(track)
    corridor = corridor_from_track(track, margin=0., policy=SIGNED_POLICY, reference=reference)
    pm = point_mass_view_of_planar(vehicle)
    derived_speed = physical_map_speed_limit(pm)
    speed_domain = min(derived_speed, args.maximum_speed) if args.maximum_speed is not None else derived_speed
    speed_domain_source = ("explicit_cli_limit" if args.maximum_speed is not None
                           else "physical_straight_line_envelope")
    speed_axis = np.linspace(0., speed_domain, args.yaw_speed_points)
    # Keep the map grid consistent with the independently probed MIS
    # low-speed forward/backward profile domain: braking needs more range
    # than the initial symmetric convenience grid supplied. Exact zero is a
    # required support slice for both signed acceleration dimensions.
    ax_axis = anchored_axis(-6., 3., args.yaw_ax_points)
    ay_axis = anchored_axis(-9.80665, 9.80665, args.yaw_ay_points)
    from tools.run.yaw_map_options import resolve_yaw_map, IncompleteYawMapError
    try:
        capability, map_report = resolve_yaw_map(pm, vehicle.yaw_inertia, speed_axis, ax_axis, args,
            limited_ay=ay_axis, save_path=output / "vehicle_limit_yaw_map.npz")
    except IncompleteYawMapError as error:
        from simplified_vehicle_models.formulations.yaw_map_artifact import json_values
        write_json(output / "map_build_failure.json", {"complete":False, "audit":json_values(error.result.audit),
            "validation_audit":json_values(error.result.validation_audit)})
        summary = failure_summary("map_build", ["incomplete_vehicle_limit_audit"], started)
        write_json(output / "summary.json", summary)
        return summary
    write_json(output / "map_build.json", map_report)
    grid = YawCapabilityGrid(capability.speed, capability.longitudinal_acceleration, capability.lateral_acceleration)
    from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings
    profile_settings = CoupledProfileSettings(
        max_wall_seconds=float(getattr(args, "profile_wall_seconds", CoupledProfileSettings().max_wall_seconds)),
        max_refinement_rounds=int(getattr(args, "profile_max_refinements", CoupledProfileSettings().max_refinement_rounds)),
        max_candidates_per_station=int(getattr(args, "profile_max_candidates", CoupledProfileSettings().max_candidates_per_station)),
        max_seam_anchors=int(getattr(args, "profile_max_seam_anchors", CoupledProfileSettings().max_seam_anchors)))
    profile_solver = getattr(args, "profile_solver", "coupled")
    map_fields = ("speed", "longitudinal_acceleration", "lateral_acceleration",
                  "feasible_mask", "yaw_moment_into_turn", "yaw_moment_out_of_turn")
    map_hash = (array_sha256(*(getattr(capability, f) for f in map_fields))
                if all(hasattr(capability, f) for f in map_fields) else None)
    if map_hash is not None:
        try:
            np.savez(output / "yaw_map.npz", speed=capability.speed, ax=capability.longitudinal_acceleration,
                     ay=capability.lateral_acceleration, feasible_mask=capability.feasible_mask,
                     yaw_moment_into_turn=capability.yaw_moment_into_turn,
                     yaw_moment_out_of_turn=capability.yaw_moment_out_of_turn)
        except (TypeError, ValueError, AttributeError):
            pass  # test doubles without real arrays
    inputs = {"track": str(args.track), "args": {k: v for k, v in vars(args).items()},
              "speed_domain_mps": speed_domain, "speed_domain_source": speed_domain_source,
              "map": {"source_kind": capability.source_kind, "source_identity": capability.source_identity,
                      "array_sha256": map_hash, "requested_counts": [args.yaw_speed_points, args.yaw_ax_points, args.yaw_ay_points],
                      "actual_counts": [len(capability.speed), len(capability.longitudinal_acceleration), len(capability.lateral_acceleration)]},
              "profile_solver": profile_solver, "profile_settings": asdict(profile_settings),
              "diagnostic_only": DIAGNOSTIC_NONCLAIMS}
    try:
        import subprocess
        inputs["git_revision"] = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True,
                                                check=False).stdout.strip() or None
    except OSError:
        inputs["git_revision"] = None
    write_json(output / "inputs.json", inputs)
    if not getattr(args, "skip_preflight", True) and hasattr(capability, "feasible_mask"):
        from simplified_vehicle_models.planning.yaw_preflight import preflight
        from simplified_vehicle_models.planning.yaw_aware_line import _sample_trajectory
        from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory
        from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
        spline0 = OffsetSpline(track, args.control_spacing)
        report = preflight(_sample_trajectory(OffsetTrajectory(reference, spline0, np.zeros(spline0.control_count)),
                                              args.planning_grid_spacing), capability, maximum_speed=speed_domain)
        write_json(output / "preflight.json", report.to_dict())
        if not report.ok:
            summary = failure_summary("preflight", list(report.blocking), started)
            write_json(output / "summary.json", summary)
            return summary
    settings = YawLineSettings(maximum_speed=speed_domain, footprint_margin=0.,
        buffer=args.lateral_margin, control_spacing=args.control_spacing,
        grid_spacing=args.planning_grid_spacing, optimizer_max_iterations=args.planner_iterations,
        maximum_outer_passes=args.planner_passes, profile_solver=profile_solver,
        profile_settings=profile_settings)
    from simplified_vehicle_models.planning.yaw_aware_line import YawLineOptimizationError
    try:
        plan = optimize_yaw_aware_line(track, corridor, capability, settings=settings)
    except YawLineOptimizationError as error:
        records = [dict(item) for item in error.diagnostics]
        causes = sorted({str(r.get("profile_status") or r.get("reason")) for r in records})
        write_json(output / "planning_failure.json", {"message": str(error), "causes": causes,
                                                      "candidate_records": records})
        summary = failure_summary("planning", causes, started, candidate_count=len(records))
        write_json(output / "summary.json", summary)
        return summary
    line = path_coordinates_from_yaw_line(plan)
    np.savetxt(output / "line.csv", np.column_stack((line.distance, line.reference_distance, line.x_north,
               line.y_east, line.heading, line.curvature)), delimiter=",", comments="",
               header="distance,reference_distance,x_north,y_east,heading,curvature")
    line_path = output / "line.csv"
    args.line_csv_sha256 = file_sha256(line_path) if line_path.exists() else None
    profile_diag = dict(getattr(plan, "diagnostics", {}) or {}).get("profile_diagnostics", {})
    args.yaw_profile_provenance = {"solver": profile_solver, "settings": asdict(profile_settings),
                                   "map_array_sha256": map_hash, "line_csv_sha256": args.line_csv_sha256,
                                   "feasibility_status": profile_diag.get("feasibility_status"),
                                   "resolution_converged": profile_diag.get("resolution_converged"),
                                   "budget_reason": profile_diag.get("budget_reason")}
    plan_doc = {
        "status": plan.status, "speed_domain_source": speed_domain_source,
        "speed_domain_mps": speed_domain, "map": {"source_kind": capability.source_kind,
        "source_identity": capability.source_identity, "yaw_inertia": capability.yaw_inertia,
        "grid": {"speed": grid.speed.tolist(), "longitudinal_acceleration": grid.longitudinal_acceleration.tolist(),
                 "lateral_acceleration": grid.lateral_acceleration.tolist()}},
        "planning": asdict(settings), "line_shape_metrics": dict(plan.shape_metrics),
        "candidate_diagnostics": _jsonable(plan.diagnostics), "diagnostic_only": DIAGNOSTIC_NONCLAIMS,
        "profile_diagnostics": _jsonable(profile_diag), "yaw_profile_provenance": args.yaw_profile_provenance,
        "predicted_lap_time_seconds": float(plan.profile.lap_time),
    }
    write_json(output / "plan.json", plan_doc)
    if getattr(args, "plan_only", False):
        summary = {"status": "planned", "plan_only": True, "lap_completed": False,
                   "predicted_time_seconds": float(plan.profile.lap_time),
                   "profile_resolution_converged": profile_diag.get("resolution_converged"),
                   "elapsed_setup_seconds": time.perf_counter() - started,
                   "qualified": False, "accepted": False, "U3_qualified": False,
                   "diagnostic_only": DIAGNOSTIC_NONCLAIMS}
        write_json(output / "summary.json", summary)
        return summary
    control = vehicle
    plant = PlanarForwardPlant(vehicle, plan.speed_plan)
    driver = PreviewDriver(plan.speed_plan, control.wheelbase, control.steering_maximum_angle,
        control.steering_maximum_rate, control.maximum_pedal_force, preview_seconds=args.preview_seconds,
        heading_gain=args.heading_gain, lateral_gain=args.lateral_gain, speed_gain=args.speed_gain,
        live_limit=False)
    initial = plant.initial_state(float(plan.speed_plan.speed[0]))
    kwargs = {}
    sink = None
    if args.record:
        sink = make_record_sink(plant, line, output / "run_record.npz", args.record_precision)
        kwargs["step_sink"] = sink
    result = solve_forward(plant, driver, initial, duration=args.duration, wall_seconds=args.wall_seconds,
        maximum_wall_seconds=300., maximum_step=args.maximum_step, rtol=args.rtol,
        atol=args.rtol * .01, method=args.method, step_sink=kwargs.get("step_sink"))
    if sink is not None:
        # A sink with no accepted steps cannot produce a legitimate record. Keep
        # the fresh output directory and leave the failure explicit for diagnosis.
        try:
            sink.close(record_metadata(args, track, vehicle, capability, plan, result))
        except RuntimeError as error:
            summary = failure_summary("record", ["no_recorded_step"], started, exception=str(error))
            write_json(output / "summary.json", summary)
            return summary
    np.savetxt(output / "trace.csv", np.column_stack((result.time, result.state.T)), delimiter=",", comments="",
               header=",".join(("time", *plant.state_names)))
    if result.pedal is not None:
        np.savetxt(output / "pedal.csv", np.column_stack((result.time, result.pedal)), delimiter=",", comments="",
                   header="time,pedal")
    observations = [plant.observe(state) for state in result.state.T]
    violation, station, contained = trajectory_corridor_containment(
        plan, [o.distance for o in observations], [o.lateral_error for o in observations])
    summary = {"status": "completed" if result.lap_completed else "incomplete", "lap_completed": bool(result.lap_completed),
        "predicted_time_seconds": plan.profile.lap_time,
        "driven_time_seconds": float(result.time[-1]) if result.lap_completed else None,
        "sampled_center_corridor_violation_m": violation, "worst_violation_station_m": station,
        "sampled_center_contained": contained, "elapsed_setup_seconds": time.perf_counter()-started,
        "qualified": False, "accepted": False, "U3_qualified": False, "diagnostic_only": DIAGNOSTIC_NONCLAIMS}
    write_json(output / "summary.json", summary)
    return summary


#: Exit codes: 0 completed lap or successful --plan-only; 3 incomplete lap;
#: 4 failure summary written (planning, preflight, record); 1 exception.
EXIT_CODES = {"completed": 0, "planned": 0, "incomplete": 3, "failed": 4}


def main(argv=None):
    args = parse_args(argv)
    try:
        summary = run(args)
    except (ValueError, FileExistsError, OSError) as error:
        raise SystemExit(f"error: {error}") from error
    return EXIT_CODES.get(summary.get("status"), 1)


if __name__ == "__main__":
    raise SystemExit(main())
