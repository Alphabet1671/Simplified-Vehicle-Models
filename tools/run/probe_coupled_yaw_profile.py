"""Freeze fixed-path yaw-profile diagnostics for Task 1.

The probe writes an artifact before attempting the legacy profile so failures
remain reproducible. Physical fixture production is intentionally explicit via
--output; full-track artifacts should be stored outside version control.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
from collections.abc import Mapping
from pathlib import Path
import time

import numpy as np

from tools.run import run_yaw_aware_forward_driver as runner


def _hash_array(array):
    array = np.ascontiguousarray(array)
    return hashlib.sha256(array.view(np.uint8)).hexdigest()


def _jsonable(value):
    """Convert immutable diagnostic evidence to strict JSON-compatible values."""
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, np.ndarray):
        return [_jsonable(item) for item in value.tolist()]
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    if isinstance(value, np.generic):
        return _jsonable(value.item())
    from dataclasses import is_dataclass, asdict
    from enum import Enum
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def _sample_geometry(track, geometry, spacing):
    from simplified_vehicle_models.solvers.yaw_aware_profile import YawPath, curvature_rate_from_samples
    if geometry == "raw-centerline":
        length = float(track.distance[-1])
        distance = np.linspace(0., length, max(3, int(np.ceil(length / spacing))) + 1)
        curvature = np.interp(distance, track.distance, track.curvature)
        closed = bool(track.is_closed)
    else:
        from simplified_vehicle_models.planning.yaw_aware_line import _sample_trajectory
        from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve, OffsetTrajectory
        from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
        reference = ReferenceCurve(track)
        spline = OffsetSpline(track, max(4., spacing * 100.))
        trajectory = OffsetTrajectory(reference, spline, np.zeros(spline.control_count))
        return _sample_trajectory(trajectory, spacing)
    rate = curvature_rate_from_samples(distance, curvature, is_closed=closed)
    return YawPath(distance, curvature, rate, closed)


def run(args):
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
    from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityGrid, build_physical_yaw_map
    from simplified_vehicle_models.solvers.yaw_aware_profile import yaw_aware_profile, _MapInterpolator
    from simplified_vehicle_models.tracks.profile import load_track

    args.solver = getattr(args, "solver", "legacy")
    args.wall_seconds = getattr(args, "wall_seconds", 120.)
    vehicle = compile_formulation(str(runner.PLANAR_FORMULATION)).model
    pm = point_mass_view_of_planar(vehicle)
    speed_domain = runner.physical_map_speed_limit(pm, search_limit_mps=args.maximum_speed)
    track = load_track(args.track)
    path = _sample_geometry(track, args.geometry, args.spacing)
    ax_min, ax_max = getattr(args, 'yaw_map_ax_min', -6.), getattr(args, 'yaw_map_ax_max', 3.)
    ax = np.linspace(ax_min, ax_max, args.ax_points)
    ay = np.linspace(-9.80665, 9.80665, args.ay_points)
    if args.anchored:
        ax, ay = runner.anchored_axis(ax_min, ax_max, args.ax_points), runner.anchored_axis(-9.80665, 9.80665, args.ay_points)
    started = time.perf_counter()
    from tools.run.yaw_map_options import resolve_yaw_map
    capability, map_report = resolve_yaw_map(pm, vehicle.yaw_inertia,
        np.linspace(0., speed_domain, args.speed_points), ax, args, limited_ay=ay)
    grid = YawCapabilityGrid(capability.speed, capability.longitudinal_acceleration, capability.lateral_acceleration)
    map_seconds = time.perf_counter() - started
    interp = _MapInterpolator(capability)
    doc = {
        "map_build": map_report,
        "settings": {"track": str(args.track), "geometry": args.geometry, "spacing_m": args.spacing,
                     "speed_domain_mps": speed_domain, "requested_counts": {"speed": args.speed_points,
                     "ax": args.ax_points, "ay": args.ay_points}, "actual_counts": {
                     "speed": len(grid.speed), "ax": len(grid.longitudinal_acceleration),
                     "ay": len(grid.lateral_acceleration)}, "anchored": args.anchored},
        "path": {"distance": path.distance.tolist(), "curvature": path.curvature.tolist(),
                 "curvature_rate": path.curvature_rate.tolist(), "is_closed": path.is_closed,
                 "sha256": {"distance": _hash_array(path.distance), "curvature": _hash_array(path.curvature),
                            "curvature_rate": _hash_array(path.curvature_rate)}},
        "map": {"speed": grid.speed.tolist(), "ax": grid.longitudinal_acceleration.tolist(),
                "ay": grid.lateral_acceleration.tolist(), "feasible_mask": capability.feasible_mask.astype(int).tolist(),
                "yaw_moment_into_turn": capability.yaw_moment_into_turn.tolist(),
                "yaw_moment_out_of_turn": capability.yaw_moment_out_of_turn.tolist(),
                "yaw_inertia": float(capability.yaw_inertia), "identity": capability.source_identity,
                "build_seconds": map_seconds},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Persist a useful reproduction even if the legacy profile raises.
    args.output.write_text(json.dumps(_jsonable(doc), indent=2, allow_nan=False) + "\n")
    started = time.perf_counter()
    try:
        boundary = "flying" if path.is_closed else "standing"
        if args.solver == "coupled":
            from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings
            settings = CoupledProfileSettings(max_wall_seconds=args.wall_seconds)
            profile = yaw_aware_profile(path, capability, boundary=boundary, maximum_speed=speed_domain,
                                        solver="coupled", coupled_settings=settings)
        else:
            profile = yaw_aware_profile(path, capability, boundary=boundary, maximum_speed=speed_domain)
        doc["profile"] = {"status": "ok", "solver": args.solver, "lap_time": profile.lap_time,
                          "speed_min": float(profile.speed.min()), "speed_max": float(profile.speed.max()),
                          "diagnostics": dict(profile.diagnostics),
                          "stage_seconds": {k: profile.diagnostics.get("timings", {}).get(k) for k in
                              ("interval_seconds", "graph_build_seconds", "graph_solve_seconds", "refinement_seconds", "validation_seconds", "total_seconds")}}
    except Exception as error:
        # Preserve typed failure and the whole fixed path. Recover only the
        # explicit segment integer carried by the legacy exception message.
        message = str(error)
        match = re.search(r"\bsegment\s+(\d+)\b", message)
        diagnostics = getattr(error, "diagnostics", None)
        failed_segment = (int(diagnostics["segment_index"]) if diagnostics is not None and "segment_index" in diagnostics
                          else (int(match.group(1)) if match else None))
        doc["profile"] = {"status": "failed", "solver": args.solver, "exception": type(error).__name__,
                         "profile_status": getattr(error, "status", None), "message": message,
                         "failed_segment": failed_segment, "segment_evidence": diagnostics,
                         "segment_evidence_note": ("structured legacy transition diagnostics" if diagnostics is not None else "segment parsed from exception message; no structured transition diagnostics")}
    doc["profile_seconds"] = time.perf_counter() - started
    args.output.write_text(json.dumps(_jsonable(doc), indent=2, allow_nan=False) + "\n")
    return doc


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--geometry", choices=("raw-centerline", "zero-offset"), required=True)
    parser.add_argument("--spacing", type=float, default=.05)
    parser.add_argument("--speed-points", type=int, default=12)
    parser.add_argument("--ax-points", type=int, default=9)
    parser.add_argument("--ay-points", type=int, default=17)
    parser.add_argument("--maximum-speed", type=float)
    parser.add_argument("--anchored", action="store_true")
    parser.add_argument("--solver", choices=("legacy", "coupled"), default="legacy")
    parser.add_argument("--wall-seconds", type=float, default=120.)
    from tools.run.yaw_map_options import add_yaw_map_options
    add_yaw_map_options(parser)
    args = parser.parse_args(argv)
    if args.spacing <= 0 or not np.isfinite(args.spacing):
        parser.error("--spacing must be finite and positive")
    if min(args.speed_points, args.ax_points, args.ay_points) < 2:
        parser.error("map point counts must be at least two")
    return args


if __name__ == "__main__":
    run(parse_args())
