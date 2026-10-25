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
from pathlib import Path
import time

import numpy as np

from tools.run import run_yaw_aware_forward_driver as runner


def _hash_array(array):
    array = np.ascontiguousarray(array)
    return hashlib.sha256(array.view(np.uint8)).hexdigest()


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

    vehicle = compile_formulation(str(runner.PLANAR_FORMULATION)).model
    pm = point_mass_view_of_planar(vehicle)
    speed_domain = runner.physical_map_speed_limit(pm, search_limit_mps=args.maximum_speed)
    track = load_track(args.track)
    path = _sample_geometry(track, args.geometry, args.spacing)
    ax = np.linspace(-6., 3., args.ax_points)
    ay = np.linspace(-9.80665, 9.80665, args.ay_points)
    if args.anchored:
        ax, ay = runner.anchored_axis(-6., 3., args.ax_points), runner.anchored_axis(-9.80665, 9.80665, args.ay_points)
    grid = YawCapabilityGrid(np.linspace(0., speed_domain, args.speed_points), ax, ay)
    started = time.perf_counter()
    capability = build_physical_yaw_map(pm, grid, yaw_inertia=vehicle.yaw_inertia)
    map_seconds = time.perf_counter() - started
    interp = _MapInterpolator(capability)
    doc = {
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
    args.output.write_text(json.dumps(doc, indent=2, allow_nan=False) + "\n")
    started = time.perf_counter()
    try:
        boundary = "flying" if path.is_closed else "standing"
        profile = yaw_aware_profile(path, capability, boundary=boundary, maximum_speed=speed_domain)
        doc["profile"] = {"status": "ok", "lap_time": profile.lap_time}
    except Exception as error:
        # Preserve typed failure and the whole fixed path. Recover only the
        # explicit segment integer carried by the legacy exception message.
        message = str(error)
        match = re.search(r"\bsegment\s+(\d+)\b", message)
        failed_segment = int(match.group(1)) if match else None
        doc["profile"] = {"status": "failed", "exception": type(error).__name__, "message": message,
                         "failed_segment": failed_segment, "segment_evidence": None,
                         "segment_evidence_note": "segment parsed from exception message; legacy profile exposes no further failed-segment diagnostics"}
    doc["profile_seconds"] = time.perf_counter() - started
    args.output.write_text(json.dumps(doc, indent=2, allow_nan=False) + "\n")
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
    args = parser.parse_args(argv)
    if args.spacing <= 0 or not np.isfinite(args.spacing):
        parser.error("--spacing must be finite and positive")
    if min(args.speed_points, args.ax_points, args.ay_points) < 2:
        parser.error("map point counts must be at least two")
    return args


if __name__ == "__main__":
    run(parse_args())
