#!/usr/bin/env python3
"""Reconstruct driver commands from a forward driver trace."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent / "src"))

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.forward_driver import (
    PreviewDriver,
    DriverObservation,
    build_speed_plan,
)
from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.path_coordinates import evaluate_path


def main():
    trace_path = Path("docs/validation/iem24-murphy/forward_driver_trials/attempt_1792006853120683000/trace.csv")
    trace = np.loadtxt(trace_path, delimiter=",", skiprows=1)

    vehicle = compile_formulation("models/formulations/iem24_murphy_real_planar_7dof.xml").model
    track = load_track("models/tracks/2024_michigan_endurance_closed.xml")
    count = int(np.ceil(track.lap_length / 0.5))
    distance = np.linspace(0.0, track.lap_length, count + 1)
    _, _, _, curvature = evaluate_path(track.path_coordinates, distance)

    plan = build_speed_plan(distance, curvature, 60.0, 3.0, 1.0, 2.0, vehicle=vehicle, live_limit=True)
    plant = PlanarForwardPlant(vehicle, plan)
    driver = PreviewDriver(
        plan,
        vehicle.wheelbase,
        vehicle.steering_maximum_angle,
        vehicle.steering_maximum_rate,
        vehicle.maximum_pedal_force,
        live_limit=True,
    )

    print(f"{'time':>8} {'s':>8} {'v_x':>7} {'v_y':>7} {'target':>7} {'feedfwd':>8} {'throttle':>9} {'brake':>7} {'steer_rate':>11} {'dv_x/dt':>9}")

    # Sample every 1000 steps (~every 10 s)
    for i in range(0, trace.shape[0], 1000):
        state = trace[i]
        obs = plant.observe(state)
        cmd = driver.command(obs)
        target, _, feedforward = plan.sample(obs.distance)

        # Compute actual acceleration
        try:
            rates = plant.evaluate(state[0], state, cmd).solve()
            dv_x = rates[3]
        except Exception as exc:
            dv_x = float("nan")
            print(f"  step {i} error: {exc}")

        print(
            f"{state[0]:8.2f} {state[1]:8.1f} {state[4]:7.3f} {state[5]:7.3f} "
            f"{target:7.3f} {feedforward:8.3f} {cmd.throttle:9.4f} {cmd.brake_force:7.2f} "
            f"{cmd.steering_rate:11.4f} {dv_x:9.4f}"
        )

    # Find where speed is max and min
    max_idx = np.argmax(trace[:, 4])
    min_idx = np.argmin(trace[:, 4])
    print(f"\nMax speed at t={trace[max_idx, 0]:.2f}, s={trace[max_idx, 1]:.1f}, v_x={trace[max_idx, 4]:.3f}")
    print(f"Min speed at t={trace[min_idx, 0]:.2f}, s={trace[min_idx, 1]:.1f}, v_x={trace[min_idx, 4]:.3f}")


if __name__ == "__main__":
    main()
