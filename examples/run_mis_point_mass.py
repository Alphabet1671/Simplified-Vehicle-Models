from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from simplified_vehicle_models.results.traces import LapMode
from simplified_vehicle_models.solvers.forward_backward import solve_lap
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.xml.loader import load_point_mass_vehicle


def main() -> None:
    """Run both point-mass lap modes on the legacy MIS track."""

    repository_root = Path(__file__).resolve().parents[1]
    vehicle = load_point_mass_vehicle(
        repository_root / "models/formulations/murphy_point_mass.xml"
    )
    track = load_track(
        repository_root / "models/tracks/2024_michigan_endurance_v11.xml"
    )
    output: dict[str, object] = {
        "track": track.title,
        "track_length_m": track.lap_length,
        "node_count": len(track.distance),
        "results": {},
    }
    results = output["results"]
    assert isinstance(results, dict)
    for mode in LapMode:
        result = solve_lap(vehicle, track, mode, tolerance=1.0e-5)
        results[mode.value] = {
            "lap_time_s": result.lap_time,
            "average_speed_mps": track.lap_length / result.lap_time,
            "maximum_speed_mps": float(np.max(result.speed)),
            "minimum_speed_mps": float(np.min(result.speed)),
            "converged": result.converged,
            "iterations": result.iterations,
            "maximum_speed_change_mps": result.maximum_speed_change,
        }
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
