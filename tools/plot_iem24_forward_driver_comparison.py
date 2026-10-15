#!/usr/bin/env python3
"""Generate comparison figures for IEM24 Murphy forward driver trials.

The forward-driver trace.csv stores the integrated STATE at each accepted
step (time,s,n,heading_error,v_x,v_y,yaw_rate,omega_fl,...,soc,steering). It
does NOT store the state RATE at each sample, so physical body-axis
accelerations and the yaw angular acceleration are not directly available.

An earlier version of this tool read state columns v_y (index 5) and
yaw_rate (index 6) and labeled them "ax" and "ay". Since v_y and yaw_rate are
nearly proportional along this driver's trajectory, the mislabeled curves
were nearly identical -- that was a plotting bug, not a property of the
vehicle. This version replays saved states and pedals through the current
plant.evaluate(...) implementation to reconstruct the commanded rates. The physical body-axis accelerations are
ax = rate[3] - yaw_rate*v_y and ay = rate[4] + yaw_rate*v_x.
The rotating-frame terms matter: rate[4] alone omits centripetal acceleration
in a steady turn. Rates come from the plant's quasi-static load/tire solve,
not numerical differentiation.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


def _recompute_accelerations(trace_path: Path, attempt_dir: Path) -> np.ndarray:
    """Replay the saved states and pedals through the currently installed plant.

    Returns physical body-axis acceleration and yaw angular acceleration.
    Settings come from summary.json; replay of historical attempts is not
    provenance-proof unless all transitive model and code inputs still match.
    """
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.tracks.profile import load_track
    from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
    from simplified_vehicle_models.solvers.forward_driver import build_speed_plan, PreviewDriver
    from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant, body_accelerations

    summary = json.loads((attempt_dir / "summary.json").read_text())
    settings = summary["settings"]

    vehicle = compile_formulation("models/formulations/iem24_murphy_real_planar_7dof.xml").model
    track = load_track("models/tracks/2024_michigan_endurance_closed.xml")
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
    pedal_path = attempt_dir / "pedal.csv"
    if driver.has_pedal_dynamics and not pedal_path.is_file():
        raise FileNotFoundError(f"pedal dynamics enabled but replay file is missing: {pedal_path}")
    pedal = (np.loadtxt(pedal_path, delimiter=",", skiprows=1)
             if driver.has_pedal_dynamics else None)

    trace = np.loadtxt(trace_path, delimiter=",", skiprows=1)
    if pedal is not None and (pedal.shape != (len(trace), 2) or
                              not np.array_equal(pedal[:, 0], trace[:, 0])):
        raise ValueError("pedal.csv must match trace.csv row count and time exactly")
    time = trace[:, 0]
    states = trace[:, 1:13]

    n = states.shape[0]
    rates = np.empty((n, plant.state_size), dtype=float)
    for i in range(n):
        state = states[i]
        observation = plant.observe(state)
        command = driver.command(observation)
        if pedal is not None:
            command = driver.realized_command(float(np.interp(time[i], pedal[:, 0], pedal[:, 1])), command)
        rates[i] = plant.evaluate(time[i], state, command).solve()
    ax_body, ay_body = body_accelerations(states, rates)
    return np.column_stack((ax_body, ay_body, rates[:, 5]))


def trace_channels(trace: np.ndarray, accelerations: np.ndarray) -> dict[str, np.ndarray]:
    """Map the trace schema to plot channels without treating states as rates."""
    if (trace.ndim != 2 or trace.shape[1] != 13 or accelerations.shape != (len(trace), 3)):
        raise ValueError("expected 13-column trace and matching 3-column accelerations")
    return {"distance": trace[:, 1], "speed": np.hypot(trace[:, 4], trace[:, 5]),
            "ax": accelerations[:, 0], "ay": accelerations[:, 1],
            "yaw_rate": trace[:, 6], "soc": trace[:, 11]}


def plot_comparison(trace_path: Path, comparison_path: Path, accelerations: np.ndarray,
                    output_dir: Path) -> Path:
    """Generate comparison plot for forward driver vs measured data."""
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-config")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output_dir.mkdir(parents=True, exist_ok=True)

    trace_data = np.loadtxt(trace_path, delimiter=",", skiprows=1)
    channels = trace_channels(trace_data, accelerations)
    fd_distance = channels["distance"]
    fd_speed = channels["speed"]
    fd_ax = channels["ax"]
    fd_ay = channels["ay"]
    fd_yaw_rate = channels["yaw_rate"]

    # comparison.csv contains measured median and the point-mass simulation.
    comp_data = np.loadtxt(comparison_path, delimiter=",", skiprows=1)
    comp_distance = comp_data[:, 0]
    meas_speed = comp_data[:, 1]
    ocp_speed = comp_data[:, 2]
    meas_ax = comp_data[:, 3]
    ocp_ax = comp_data[:, 4]
    meas_ay = comp_data[:, 5]
    ocp_ay = comp_data[:, 6]
    meas_yaw_rate = comp_data[:, 7]
    ocp_yaw_rate = comp_data[:, 8]

    from scipy.interpolate import interp1d

    def interp(values):
        return interp1d(comp_distance, values, bounds_error=False,
                        fill_value="extrapolate")(fd_distance)

    meas_speed_fd = interp(meas_speed)
    ocp_speed_fd = interp(ocp_speed)
    meas_ax_fd = interp(meas_ax)
    ocp_ax_fd = interp(ocp_ax)
    meas_ay_fd = interp(meas_ay)
    ocp_ay_fd = interp(ocp_ay)
    meas_yaw_rate_fd = interp(meas_yaw_rate)
    ocp_yaw_rate_fd = interp(ocp_yaw_rate)

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), constrained_layout=True, sharex=True)

    colors = {
        "measured_median": "#1f4e79",
        "ocp": "#c00000",
        "forward_driver": "#2ecc71"
    }

    ax = axes[0]
    ax.plot(fd_distance, meas_speed_fd, color=colors["measured_median"], lw=1.4, label="Measured median")
    ax.plot(fd_distance, ocp_speed_fd, color=colors["ocp"], lw=1.2, ls="--", label="Point-mass simulation")
    ax.plot(fd_distance, fd_speed, color=colors["forward_driver"], lw=1.2, label="Forward driver (live-limit)")
    ax.set_ylabel("Speed (m/s)")
    ax.grid(True, alpha=0.25)
    ax.legend(frameon=False, fontsize=8)

    ax = axes[1]
    ax.plot(fd_distance, meas_ax_fd, color=colors["measured_median"], lw=1.4)
    ax.plot(fd_distance, ocp_ax_fd, color=colors["ocp"], lw=1.2, ls="--")
    ax.plot(fd_distance, fd_ax, color=colors["forward_driver"], lw=1.2)
    ax.set_ylabel("Long. accel (m/s²)")
    ax.grid(True, alpha=0.25)

    ax = axes[2]
    ax.plot(fd_distance, meas_ay_fd, color=colors["measured_median"], lw=1.4)
    ax.plot(fd_distance, ocp_ay_fd, color=colors["ocp"], lw=1.2, ls="--")
    ax.plot(fd_distance, fd_ay, color=colors["forward_driver"], lw=1.2)
    ax.set_ylabel("Lat. accel (m/s²)")
    ax.grid(True, alpha=0.25)

    ax = axes[3]
    ax.plot(fd_distance, meas_yaw_rate_fd, color=colors["measured_median"], lw=1.4)
    ax.plot(fd_distance, ocp_yaw_rate_fd, color=colors["ocp"], lw=1.2, ls="--")
    ax.plot(fd_distance, fd_yaw_rate, color=colors["forward_driver"], lw=1.2)
    ax.set_xlabel("Distance (m)")
    ax.set_ylabel("Yaw rate (rad/s)")
    ax.grid(True, alpha=0.25)

    lap_time = trace_data[-1, 0] - trace_data[0, 0]
    lap_distance = fd_distance[-1] - fd_distance[0]
    fig.suptitle("Forward-driver prototype vs measured median and point-mass (nonperiodic trace)\n"
                 f"Simulated lap: {lap_time:.2f} s | Distance: {lap_distance:.2f} m", fontsize=10)

    output_path = output_dir / "comparison.png"
    fig.savefig(output_path, dpi=180, facecolor="white")
    plt.close(fig)

    return output_path


def plot_simulated_lap(trace_path: Path, accelerations: np.ndarray, output_dir: Path) -> Path:
    """Generate simulated lap plot using real (replayed) accelerations."""
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-config")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output_dir.mkdir(parents=True, exist_ok=True)

    trace_data = np.loadtxt(trace_path, delimiter=",", skiprows=1)
    channels = trace_channels(trace_data, accelerations)
    distance = channels["distance"]
    speed = channels["speed"]
    soc = channels["soc"]
    accel_x = channels["ax"]
    accel_y = channels["ay"]
    yaw_rate = channels["yaw_rate"]

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), constrained_layout=True, sharex=True)

    colors = {
        "speed": "#1f4e79",
        "ax": "#d55e00",
        "ay": "#0072b2",
        "soc": "#955ba5",
    }

    ax = axes[0]
    ax.plot(distance, speed, color=colors["speed"], lw=1.4)
    ax.set_ylabel("Speed (m/s)")
    ax.grid(True, alpha=0.25)

    ax = axes[1]
    ax.plot(distance, accel_x, color=colors["ax"], lw=1.4)
    ax.set_ylabel("Long. accel (m/s²)")
    ax.grid(True, alpha=0.25)

    ax = axes[2]
    ax.plot(distance, accel_y, color=colors["ay"], lw=1.4)
    ax.set_ylabel("Lat. accel (m/s²)")
    ax.grid(True, alpha=0.25)

    ax = axes[3]
    ax.plot(distance, yaw_rate, color=colors["ay"], lw=1.4)
    ax.set_xlabel("Distance (m)")
    ax.set_ylabel("Yaw rate (rad/s)")
    ax.grid(True, alpha=0.25)

    ax2 = ax.twinx()
    ax2.plot(distance, soc, color=colors["soc"], lw=1.2, ls="--", label="SOC")
    ax2.set_ylabel("State of Charge", color=colors["soc"])
    ax2.tick_params(axis="y", labelcolor=colors["soc"])

    lap_time = trace_data[-1, 0] - trace_data[0, 0]
    lap_distance = distance[-1] - distance[0]
    fig.suptitle("Forward-driver prototype, MIS 2024 (nonperiodic trace)\n"
                 f"Lap time: {lap_time:.2f} s | Distance: {lap_distance:.2f} m", fontsize=10)

    output_path = output_dir / "simulated_lap.png"
    fig.savefig(output_path, dpi=180, facecolor="white")
    plt.close(fig)

    return output_path


def main():
    parser = argparse.ArgumentParser(description="Generate comparison figures for IEM24 Murphy forward driver trials")
    parser.add_argument("--trace", type=Path, required=True, help="Path to trace.csv from forward driver trial")
    parser.add_argument("--comparison", type=Path, required=True, help="Path to comparison.csv from OCP simulation")
    parser.add_argument("--attempt-dir", type=Path, required=True,
                        help="Attempt directory holding summary.json (for the exact run settings)")
    parser.add_argument("--output-dir", type=Path, required=True, help="Output directory for figures")
    args = parser.parse_args()

    if not args.trace.exists():
        print(f"Error: Trace file not found: {args.trace}")
        sys.exit(1)
    if not args.comparison.exists():
        print(f"Error: Comparison file not found: {args.comparison}")
        sys.exit(1)
    if not (args.attempt_dir / "summary.json").exists():
        print(f"Error: summary.json not found in {args.attempt_dir}")
        sys.exit(1)

    print("Recomputing body accelerations by replaying the trace through the plant...")
    accelerations = _recompute_accelerations(args.trace, args.attempt_dir)
    print("Generating comparison figure...")
    comparison_path = plot_comparison(args.trace, args.comparison, accelerations, args.output_dir)
    print(f"  Created: {comparison_path}")

    print("Generating simulated lap figure...")
    simulated_lap_path = plot_simulated_lap(args.trace, accelerations, args.output_dir)
    print(f"  Created: {simulated_lap_path}")

    print("\nDone!")


if __name__ == "__main__":
    main()
