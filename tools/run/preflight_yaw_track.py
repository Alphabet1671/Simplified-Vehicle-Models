"""Preflight a track for the yaw-aware virtual driver (map coverage + geometry).

Builds the same physical no-TV yaw map as the runner, samples the fixed
zero-offset path, and reports map holes, near-holes, lateral-axis saturation
and curvature-rate noise. Exit code 2 when a blocking issue is found.

    PYTHONPATH=src:. python3 tools/run/preflight_yaw_track.py \
        --track models/tracks/mis_ev_endurance_2024_width.xml --spacing .05
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from tools.run import run_yaw_aware_forward_driver as runner
from tools.run.probe_coupled_yaw_profile import _sample_geometry, _jsonable


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track", type=Path, required=True)
    parser.add_argument("--geometry", choices=("raw-centerline", "zero-offset"), default="zero-offset")
    parser.add_argument("--spacing", type=float, default=.05)
    parser.add_argument("--speed-points", type=int, default=12)
    parser.add_argument("--ax-points", type=int, default=9)
    parser.add_argument("--ay-points", type=int, default=17)
    parser.add_argument("--output", type=Path)
    from tools.run.yaw_map_options import add_yaw_map_options
    add_yaw_map_options(parser)
    args = parser.parse_args(argv)

    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
    from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityGrid, build_physical_yaw_map
    from simplified_vehicle_models.planning.yaw_preflight import preflight
    from simplified_vehicle_models.tracks.profile import load_track

    vehicle = compile_formulation(str(runner.PLANAR_FORMULATION)).model
    pm = point_mass_view_of_planar(vehicle)
    vmax = runner.physical_map_speed_limit(pm)
    from tools.run.yaw_map_options import resolve_yaw_map
    capability, map_report = resolve_yaw_map(pm, vehicle.yaw_inertia,
        np.linspace(0., vmax, args.speed_points), runner.anchored_axis(args.yaw_map_ax_min, args.yaw_map_ax_max, args.ax_points), args,
        limited_ay=runner.anchored_axis(-9.80665, 9.80665, args.ay_points))
    track = load_track(args.track)
    path = _sample_geometry(track, args.geometry, args.spacing)
    report = preflight(path, capability, maximum_speed=vmax)
    doc = {"track": str(args.track), "geometry": args.geometry, "spacing_m": args.spacing,
           "speed_domain_mps": vmax, "map_build": map_report, **report.to_dict()}
    text = json.dumps(_jsonable(doc), indent=2, allow_nan=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n")
    print(text)
    return 0 if report.ok else 2


if __name__ == "__main__":
    sys.exit(main())
