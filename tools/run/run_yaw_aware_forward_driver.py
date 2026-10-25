"""Run a diagnostic yaw-aware line plan through the planar forward driver.

This runner is diagnostic only. It does not establish minimum time, TV/FM U
validation, measured-vehicle validity, full U3 qualification, footprint or
between-step containment, periodicity, or refinement.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
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


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track", type=Path, required=True, help="signed XY track XML with left/right edges")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--model", choices=("planar-7dof", "vertical-14dof"), default="planar-7dof")
    parser.add_argument("--map", choices=("no-tv", "physical-tv"), default="no-tv")
    parser.add_argument("--yaw-speed-points", type=int, default=8)
    parser.add_argument("--yaw-ax-points", type=int, default=5)
    parser.add_argument("--yaw-ay-points", type=int, default=9)
    parser.add_argument("--maximum-speed", type=float, default=30.)
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
    args = parser.parse_args(argv)
    for name in ("maximum_speed", "lateral_margin", "control_spacing", "planning_grid_spacing",
                 "duration", "wall_seconds", "maximum_step", "rtol", "preview_seconds",
                 "heading_gain", "lateral_gain", "speed_gain"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name.replace('_','-')} must be finite and positive")
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
    if isinstance(value, Mapping):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    return value


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
    grid = YawCapabilityGrid(
        np.linspace(0., args.maximum_speed, args.yaw_speed_points),
        # Keep the map grid consistent with the independently probed MIS
        # low-speed forward/backward profile domain: braking needs more range
        # than the initial symmetric convenience grid supplied.
        np.linspace(-6., 3., args.yaw_ax_points),
        np.linspace(-9.80665, 9.80665, args.yaw_ay_points),
    )
    capability = build_physical_yaw_map(pm, grid, yaw_inertia=vehicle.yaw_inertia)
    settings = YawLineSettings(maximum_speed=args.maximum_speed, footprint_margin=0.,
        buffer=args.lateral_margin, control_spacing=args.control_spacing,
        grid_spacing=args.planning_grid_spacing, optimizer_max_iterations=args.planner_iterations,
        maximum_outer_passes=args.planner_passes)
    plan = optimize_yaw_aware_line(track, corridor, capability, settings=settings)
    line = path_coordinates_from_yaw_line(plan)
    np.savetxt(output / "line.csv", np.column_stack((line.distance, line.reference_distance, line.x_north,
               line.y_east, line.heading, line.curvature)), delimiter=",", comments="",
               header="distance,reference_distance,x_north,y_east,heading,curvature")
    plan_doc = {
        "status": plan.status, "map": {"source_kind": capability.source_kind,
        "source_identity": capability.source_identity, "yaw_inertia": capability.yaw_inertia,
        "grid": {"speed": grid.speed.tolist(), "longitudinal_acceleration": grid.longitudinal_acceleration.tolist(),
                 "lateral_acceleration": grid.lateral_acceleration.tolist()}},
        "planning": asdict(settings), "line_shape_metrics": dict(plan.shape_metrics),
        "candidate_diagnostics": _jsonable(plan.diagnostics), "diagnostic_only": DIAGNOSTIC_NONCLAIMS,
    }
    (output / "plan.json").write_text(json.dumps(_jsonable(plan_doc), indent=2) + "\n")
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
        sink.close()
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
    (output / "summary.json").write_text(json.dumps(_jsonable(summary), indent=2) + "\n")
    return summary


def main(argv=None):
    args = parse_args(argv)
    try:
        run(args)
    except (ValueError, FileExistsError, OSError) as error:
        raise SystemExit(f"error: {error}") from error
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
