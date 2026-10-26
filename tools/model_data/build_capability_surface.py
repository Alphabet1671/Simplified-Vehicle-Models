"""Build and cache a point-mass quasi-steady capability surface."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
from simplified_vehicle_models.formulations.point_mass_capability import (
    build_capability_surface, capability_hash, save_capability_surface,
)
from simplified_vehicle_models.tracks.profile import load_track


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--formulation", required=True, type=Path)
    parser.add_argument("--track", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--curvature-margin", type=float, default=1.5)
    args = parser.parse_args(argv)
    vehicle = point_mass_view_of_planar(compile_formulation(str(args.formulation)).model)
    track = load_track(args.track)
    peak = float(abs(track.curvature).max())
    surface = build_capability_surface(vehicle, curvature_limit=args.curvature_margin * peak)
    path = save_capability_surface(surface, args.output)
    print(f"wrote {path} hash={capability_hash(vehicle)} peak_curvature={peak:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
