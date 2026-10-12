"""Reproducible SOC-coupled point-mass benchmarks.

Examples (from the repository root)::

    PYTHONPATH=src python examples/benchmark_soc_solver.py --mode all --progress
    PYTHONPATH=src python examples/benchmark_soc_solver.py --mode regen-22 --progress

The default mode is the inexpensive exact assembled lap.  The 22-lap run is
never selected implicitly; pass ``--mode regen-22`` explicitly.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from simplified_vehicle_models.formulations.point_mass import DriveCachePolicy
from simplified_vehicle_models.results.traces import LapMode, SolverProgress
from simplified_vehicle_models.solvers.forward_backward import (
    solve_lap,
    solve_lap_with_energy,
    solve_stint,
)
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.xml.loader import load_point_mass_vehicle


ROOT = Path(__file__).resolve().parents[1]


def progress(event: SolverProgress) -> None:
    print(
        f"progress lap={event.lap}/{event.lap_count} stage={event.stage} "
        f"energy_iteration={event.energy_iteration} "
        f"profile_iteration={event.profile_iteration} elapsed={event.elapsed_seconds:.3f}s",
        flush=True,
    )


def lap_summary(result, elapsed: float, label: str) -> dict[str, object]:
    return {
        "label": label,
        "wall_seconds": elapsed,
        "lap_time_s": result.lap_time,
        "average_speed_mps": result.lap_time and result.speed.size and float(result.speed.size),
        "maximum_speed_mps": float(np.max(result.speed)),
        "minimum_speed_mps": float(np.min(result.speed)),
        "converged": result.converged,
        "iterations": result.iterations,
        "cache_hits": result.drive_cache_hits,
        "cache_entries": result.drive_cache_entries,
        "max_speed_bucket_displacement_mps": result.drive_cache_max_speed_displacement,
        "max_soc_bucket_displacement": result.drive_cache_max_soc_displacement,
    }


def run_lap(vehicle, track, policy: DriveCachePolicy, label: str) -> dict[str, object]:
    started = time.perf_counter()
    result = solve_lap(vehicle, track, LapMode.FLYING, drive_cache_policy=policy)
    elapsed = time.perf_counter() - started
    summary = lap_summary(result, elapsed, label)
    summary["average_speed_mps"] = track.lap_length / result.lap_time
    return summary


def energy_summary(result, elapsed: float, label: str) -> dict[str, object]:
    return {
        "label": label,
        "wall_seconds": elapsed,
        "lap_time_s": result.lap_time,
        "lap_converged": result.lap.converged,
        "energy_converged": result.energy.converged,
        "energy_iterations": result.energy.energy_iterations,
        "profile_residual": result.energy.state_profile_residual,
        "final_soc": result.energy.final_state_of_charge,
        "consumed_j": result.energy.consumed_energy,
        "recovered_j": result.energy.recovered_energy,
        "net_j": result.energy.net_energy,
    }


def run_regen_one(vehicle, track, progress_callback=None) -> dict[str, object]:
    started = time.perf_counter()
    cold = solve_lap_with_energy(
        vehicle, track, LapMode.FLYING, regen_enabled=True,
        progress_callback=progress_callback,
    )
    cold_elapsed = time.perf_counter() - started
    started = time.perf_counter()
    warm = solve_lap_with_energy(
        vehicle, track, LapMode.FLYING, regen_enabled=True,
        initial_speed=cold.lap.speed, corner_speed_hint=cold.lap.speed,
        progress_callback=progress_callback,
    )
    warm_elapsed = time.perf_counter() - started
    return {
        "cold": energy_summary(cold, cold_elapsed, "regen-one-cold"),
        "warm": energy_summary(warm, warm_elapsed, "regen-one-warm"),
    }


def run_regen_22(vehicle, track, progress_callback=None) -> dict[str, object]:
    started = time.perf_counter()
    stint = solve_stint(
        vehicle, track, LapMode.FLYING, laps=22, regen_enabled=True,
        reuse_profile=False, progress_callback=progress_callback,
    )
    return {
        "wall_seconds": time.perf_counter() - started,
        "laps": len(stint.laps),
        "lap_times_s": [result.lap_time for result in stint.laps],
        "final_soc": stint.laps[-1].energy.final_state_of_charge,
        "consumed_j": stint.total_consumed_energy,
        "recovered_j": stint.total_recovered_energy,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode", choices=("exact", "quantized", "regen-one", "regen-22", "all"),
        default="exact", help="benchmark to run (default: exact; 22 laps is explicit)",
    )
    parser.add_argument("--progress", action="store_true", help="accepted for compatibility; progress is always printed")
    options = parser.parse_args()
    progress_callback = progress if options.progress else None

    vehicle = load_point_mass_vehicle(ROOT / "models/formulations/murphy_point_mass.xml")
    track = load_track(ROOT / "models/tracks/2024_michigan_endurance_v11.xml")
    output: dict[str, object] = {"mode": options.mode, "track": track.title, "results": {}}
    results = output["results"]
    assert isinstance(results, dict)
    if options.mode in {"exact", "all"}:
        results["exact"] = run_lap(vehicle, track, DriveCachePolicy(), "exact")
    if options.mode in {"quantized", "all"}:
        results["quantized"] = run_lap(
            vehicle, track, DriveCachePolicy(speed_resolution=1e-3, soc_resolution=1e-5), "quantized"
        )
    if options.mode in {"regen-one", "all"}:
        results["regen_one"] = run_regen_one(vehicle, track, progress_callback)
    if options.mode == "regen-22":
        results["regen_22"] = run_regen_22(vehicle, track, progress_callback)
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
