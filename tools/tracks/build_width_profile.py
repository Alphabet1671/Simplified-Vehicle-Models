"""Write a lateral corridor table for a track.

The corridor is a reference-point clearance: the stored half width is the
distance from the centreline to the edge minus the car half width and any
declared margin. The solver never subtracts vehicle geometry.

Positive lateral offset is to the right of the direction of travel.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.types import WIDTH_PROVENANCE


def write_width_profile(
    track,
    destination: str | Path,
    *,
    half_width: float | None = None,
    half_width_left: float | None = None,
    half_width_right: float | None = None,
    provenance: str = "assumed",
) -> Path:
    """Write ``distance,half_width_left,half_width_right`` for a track."""

    if provenance not in WIDTH_PROVENANCE:
        raise ValueError(f"provenance must be one of {WIDTH_PROVENANCE}")
    symmetric = half_width is not None
    per_side = half_width_left is not None or half_width_right is not None
    if symmetric and per_side:
        raise ValueError("declare either half_width or the two sides, not both")
    if not symmetric and not per_side:
        raise ValueError("declare half_width or both sides")
    if symmetric:
        left = np.full(track.distance.shape, float(half_width))
        right = np.full(track.distance.shape, float(half_width))
    else:
        if half_width_left is None or half_width_right is None:
            raise ValueError("both sides are required")
        left = np.full(track.distance.shape, float(half_width_left))
        right = np.full(track.distance.shape, float(half_width_right))
    if not (np.all(np.isfinite(left)) and np.all(np.isfinite(right))):
        raise ValueError("half widths must be finite")
    if np.any(left <= 0.0) or np.any(right <= 0.0):
        raise ValueError("half widths must be positive")

    path = Path(destination)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["distance", "half_width_left", "half_width_right"])
        for index, distance in enumerate(track.distance):
            writer.writerow(
                [f"{distance:.17g}", f"{left[index]:.17g}", f"{right[index]:.17g}"]
            )
    return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--half-width", type=float, default=None)
    parser.add_argument("--half-width-left", type=float, default=None)
    parser.add_argument("--half-width-right", type=float, default=None)
    parser.add_argument("--provenance", default="assumed", choices=WIDTH_PROVENANCE)
    arguments = parser.parse_args(argv)
    track = load_track(arguments.track)
    path = write_width_profile(
        track,
        arguments.output,
        half_width=arguments.half_width,
        half_width_left=arguments.half_width_left,
        half_width_right=arguments.half_width_right,
        provenance=arguments.provenance,
    )
    print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
