#!/usr/bin/env python3
"""Recompute true body accelerations and yaw acceleration for a forward-driver trace.

The forward-driver trace.csv only stores the integrated STATE
(time,s,n,heading_error,v_x,v_y,yaw_rate,omega_fl,...,soc,steering). It does
NOT store the state RATE at each sample, so the longitudinal/lateral
acceleration and the yaw angular acceleration are not directly available.

A previous plotting tool mistakenly read state columns v_y (index 5) and
yaw_rate (index 6) and labeled them "ax" and "ay". Since v_y and yaw_rate are
nearly proportional in this near-steady-cornering trace (corr ~ 0.9999944),
the mislabeled "ax"/"ay" curves looked identical -- that was the bug, not a
property of the vehicle.

This script replays saved states and pedals through the current plant.evaluate(...), then
converts the velocity derivatives to physical body-axis accelerations:
ax = d(v_x)/dt - yaw_rate*v_y, ay = d(v_y)/dt + yaw_rate*v_x.
In a steady turn d(v_y)/dt can be zero even though lateral acceleration
is nonzero. It also saves the unconverted state derivatives separately.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


#: Attempts that predate --track record no settings["track"]; they ran on this
#: original track, so replaying them must not silently use the new default.
LEGACY_TRACK = "models/tracks/2024_michigan_endurance_closed.xml"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt", type=Path, required=True,
                        help="Forward-driver attempt directory (has trace.csv + summary.json)")
    parser.add_argument("--output", type=Path,
                        help="Output .npz/.csv path; defaults to <attempt>/trace_with_accel.csv")
    args = parser.parse_args()

    summary = json.loads((args.attempt / "summary.json").read_text())
    settings = summary["settings"]

    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.tracks.profile import load_track
    from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
    from simplified_vehicle_models.solvers.forward_driver import build_speed_plan, PreviewDriver
    from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant, body_accelerations

    vehicle = compile_formulation("models/formulations/iem24_murphy_real_planar_7dof.xml").model
    track = load_track(settings.get("track") or LEGACY_TRACK)
    spacing = float(settings["spacing"])
    count = int(np.ceil(track.lap_length / spacing))
    distance = np.linspace(0., track.lap_length, count + 1)
    _, _, _, curvature = evaluate_path(track.path_coordinates, distance)
    live_limit = settings["planner"] == "live"
    # Runs before the live planner lost its default cap stored a number here;
    # newer live runs store None (no cap). The conservative planner uses 2.
    lateral_cap = settings.get("lateral_acceleration")
    lateral_cap = float(lateral_cap) if lateral_cap is not None else (None if live_limit else 2.)
    plan = build_speed_plan(distance, curvature, float(settings["maximum_speed"]),
                            lateral_cap, 1., 2.,
                            vehicle=vehicle if live_limit else None, live_limit=live_limit)
    plant = PlanarForwardPlant(vehicle, plan)
    driver = PreviewDriver(plan, vehicle.wheelbase, vehicle.steering_maximum_angle,
                           vehicle.steering_maximum_rate, vehicle.maximum_pedal_force,
                           live_limit=live_limit,
                           pedal_travel_time=settings.get("pedal_travel_time"))
    # Runs with pedal dynamics store the integrated pedal position separately;
    # replay the command the plant actually saw.
    pedal_path = args.attempt / "pedal.csv"
    if driver.has_pedal_dynamics and not pedal_path.is_file():
        raise FileNotFoundError(f"pedal dynamics enabled but replay file is missing: {pedal_path}")
    pedal = (np.loadtxt(pedal_path, delimiter=",", skiprows=1)
             if driver.has_pedal_dynamics else None)

    trace = np.loadtxt(args.attempt / "trace.csv", delimiter=",", skiprows=1)
    if pedal is not None and (pedal.shape != (len(trace), 2) or
                              not np.array_equal(pedal[:, 0], trace[:, 0])):
        raise ValueError("pedal.csv must match trace.csv row count and time exactly")
    time = trace[:, 0]
    states = trace[:, 1:13]

    n = states.shape[0]
    rates = np.empty((n, plant.state_size), dtype=float)
    failures = 0
    for i in range(n):
        state = states[i]
        try:
            observation = plant.observe(state)
            command = driver.command(observation)
            if pedal is not None:
                command = driver.realized_command(float(np.interp(time[i], pedal[:, 0], pedal[:, 1])), command)
            rates[i] = plant.evaluate(time[i], state, command).solve()
        except Exception:
            rates[i] = np.nan
            failures += 1

    ax_body, ay_body = body_accelerations(states, rates)
    yaw_accel = rates[:, 5]

    output = args.output or (args.attempt / "trace_with_accel.csv")
    header = ("time,s,n,heading_error,v_x,v_y,yaw_rate,omega_fl,omega_fr,omega_rl,"
              "omega_rr,soc,steering,ax_body,ay_body,yaw_accel,v_x_dot,v_y_dot")
    np.savetxt(output, np.column_stack((trace, ax_body, ay_body, yaw_accel,
                                        rates[:, 3], rates[:, 4])),
               delimiter=",", comments="", header=header)
    print(f"Wrote {output} ({n} rows, {failures} replay failures)")
    v_y = states[:, 4]
    yaw_rate = states[:, 5]
    print(f"state v_y vs yaw_rate correlation: {np.corrcoef(v_y, yaw_rate)[0, 1]:.6f}  "
          f"(this is what the old plot mislabeled as ax/ay)")
    finite = np.isfinite(ax_body) & np.isfinite(ay_body)
    print(f"true ax vs ay correlation: {np.corrcoef(ax_body[finite], ay_body[finite])[0, 1]:.6f}")
    print(f"ax range: [{np.nanmin(ax_body):.3f}, {np.nanmax(ax_body):.3f}] m/s^2")
    print(f"ay range: [{np.nanmin(ay_body):.3f}, {np.nanmax(ay_body):.3f}] m/s^2")


if __name__ == "__main__":
    main()
