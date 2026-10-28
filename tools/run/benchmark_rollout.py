"""Time one closed-loop rollout window; the shared number for every planner gate.

Usage: PYTHONPATH=src:. python3 tools/run/benchmark_rollout.py --formulation F --track T \
       --duration 20 --method radau-baseline --output out.json
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.forward_driver import (
    PreviewDriver,
    build_speed_plan,
    solve_forward,
)
from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
from simplified_vehicle_models.solvers.vertical_forward import vertical_plant_class
from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
from simplified_vehicle_models.tracks.profile import load_track


def _setup(formulation: Path, track_path: Path, spacing: float = 0.5, *, tv: bool = False):
    """Return (vehicle, plan, plant, driver); ``tv`` is reserved for Task 2+."""
    vehicle = compile_formulation(str(formulation)).model
    control = getattr(vehicle, "planar", vehicle)
    track = load_track(track_path)
    count = int(np.ceil(track.lap_length / spacing))
    distance = np.linspace(0.0, track.lap_length, count + 1)
    _, _, _, curvature = evaluate_path(track.path_coordinates, distance)
    plan = build_speed_plan(distance, curvature, 60.0, None, 1.0, 2.0,
                            vehicle=control, live_limit=True)
    plant_class = PlanarForwardPlant if vehicle is control else vertical_plant_class(vehicle)
    if tv:
        from simplified_vehicle_models.evolution.cosim import tv_plant_class
        plant_class = tv_plant_class(plant_class)
    plant = plant_class(vehicle, plan)
    driver = PreviewDriver(plan, control.wheelbase, control.steering_maximum_angle,
                           control.steering_maximum_rate, control.maximum_pedal_force,
                           live_limit=True, pedal_travel_time=0.175)
    return vehicle, plan, plant, driver


def benchmark(formulation: Path, track: Path, start_distance: float, duration: float,
              method: str, wall_limit: float) -> dict:
    _, plan, plant, driver = _setup(Path(formulation), Path(track))
    if start_distance != 0.0:
        raise ValueError("baseline harness starts at the plan origin; segment starts arrive in Task 2")
    if duration <= 0.0 or wall_limit <= 0.0:
        raise ValueError("duration and wall_limit must be positive")
    initial = plant.initial_state(float(plan.speed[0]))
    started = time.perf_counter()
    failure = None
    try:
        if method != "radau-baseline":
            raise ValueError(f"unknown method {method!r}")
        result = solve_forward(plant, driver, initial, duration=duration,
                               wall_seconds=wall_limit, maximum_step=0.02, rtol=1e-6,
                               atol=1e-8, method="Radau",
                               maximum_wall_seconds=max(300.0, wall_limit))
        final = result.state[:, -1]
        simulated = float(result.time[-1])
        completed = simulated >= duration * 0.999
    except Exception as error:
        failure, final, simulated, completed = f"{type(error).__name__}: {error}", initial, 0.0, False
    wall = time.perf_counter() - started
    return {"method": method, "formulation": str(formulation), "track": str(track),
            "wall_seconds": wall, "simulated_seconds": simulated,
            "distance": float(final[0]), "rhs_evaluations": int(plant.evaluations),
            "wall_per_sim_second": wall / simulated if simulated else float("inf"),
            "completed": completed, "failure": failure}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--formulation", type=Path, required=True)
    parser.add_argument("--track", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--method", default="radau-baseline")
    parser.add_argument("--wall-limit", type=float, default=120.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    result = benchmark(args.formulation, args.track, 0.0, args.duration, args.method, args.wall_limit)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=True) + "\n")
    print(json.dumps(result, allow_nan=True))
    return 0 if result["completed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
