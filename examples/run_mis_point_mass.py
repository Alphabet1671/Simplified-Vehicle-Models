from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from simplified_vehicle_models.results.traces import LapMode, SolverProgress
from simplified_vehicle_models.solvers.forward_backward import solve_lap, solve_lap_with_energy
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.xml.loader import load_point_mass_vehicle


def main() -> None:
    """Run MIS lap modes, optionally including energy traces."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--energy", action="store_true", help="include SOC, bus-power, and voltage traces"
    )
    parser.add_argument(
        "--progress", action="store_true", help="print energy-solver progress updates"
    )
    options = parser.parse_args()

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

    def show_progress(event: SolverProgress) -> None:
        if event.stage in {"energy_iteration", "lap_complete"}:
            print(
                f"progress lap={event.lap}/{event.lap_count} stage={event.stage} "
                f"energy_iteration={event.energy_iteration} elapsed={event.elapsed_seconds:.3f}s"
            )
    for mode in LapMode:
        result = solve_lap(vehicle, track, mode, tolerance=1.0e-5)
        entry = {
            "lap_time_s": result.lap_time,
            "average_speed_mps": track.lap_length / result.lap_time,
            "maximum_speed_mps": float(np.max(result.speed)),
            "minimum_speed_mps": float(np.min(result.speed)),
            "converged": result.converged,
            "iterations": result.iterations,
            "maximum_speed_change_mps": result.maximum_speed_change,
        }
        if options.energy:
            energy = solve_lap_with_energy(
                vehicle, track, mode,
                progress_callback=show_progress if options.progress else None,
            )
            entry["energy"] = {
                "consumed_j": energy.energy.consumed_energy,
                "recovered_j": energy.energy.recovered_energy,
                "final_soc": energy.energy.final_state_of_charge,
                "minimum_bus_voltage_v": energy.energy.minimum_bus_voltage,
                "state_trace": energy.energy.state_trace.tolist(),
                "bus_power_trace_w": energy.energy.bus_power.tolist(),
                "voltage_trace_v": energy.energy.voltage.tolist(),
            }
        results[mode.value] = entry
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
