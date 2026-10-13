#!/usr/bin/env python3
"""Rerun the matched lap solvers and plot velocity and energy traces."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.results.traces import LapMode
from simplified_vehicle_models.solvers.forward_backward import solve_lap_with_energy
from simplified_vehicle_models.solvers.planar_ocp import (
    PlanarOcpOptions,
    _build_path_data,
    solve_planar_hot_lap,
)
from simplified_vehicle_models.tracks.profile import load_track
from compare_planar_point_mass import (
    FORMULATION,
    ROOT,
    TRACK,
    _point_mass_track,
    _point_mass_vehicle,
)


def _cumulative_energy_kwh(power: np.ndarray, elapsed_time: np.ndarray) -> np.ndarray:
    """Integrate node power trapezoidally, returning a value per node."""

    power_values = np.asarray(power, dtype=np.float64)
    time_values = np.asarray(elapsed_time, dtype=np.float64)
    if power_values.shape != time_values.shape or power_values.ndim != 1:
        raise ValueError("node power and elapsed time must be matching vectors")
    increments = 0.5 * (power_values[:-1] + power_values[1:]) * np.diff(time_values)
    return np.r_[0.0, np.cumsum(increments)] / 3.6e6


def _save_plots(
    data: dict[str, np.ndarray], output_dir: Path
) -> tuple[Path, Path, Path]:
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-config")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output_dir.mkdir(parents=True, exist_ok=True)
    distance_km = data["distance"] / 1000.0
    colors = {"point_mass": "#d55e00", "planar": "#0072b2"}
    labels = {"point_mass": "Point mass", "planar": "Planar 7-DOF OCP"}

    energy_path = output_dir / "point-mass-planar-energy.png"
    fig, (soc_ax, energy_ax) = plt.subplots(
        2, 1, figsize=(11.5, 7.5), sharex=True, constrained_layout=True
    )
    for key in ("point_mass", "planar"):
        soc_ax.plot(
            distance_km,
            100.0 * data[f"{key}_soc"],
            color=colors[key],
            lw=2.0,
            label=labels[key],
        )
        energy_ax.plot(
            distance_km,
            data[f"{key}_cumulative_energy_kwh"],
            color=colors[key],
            lw=2.0,
            label=labels[key],
        )
    soc_ax.set_ylabel("State of charge (%)")
    soc_ax.set_title("Energy use along the lap")
    energy_ax.set_ylabel("Cumulative net bus energy (kWh)")
    energy_ax.set_xlabel("Distance along track (km)")
    for axis in (soc_ax, energy_ax):
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best", frameon=False)
    fig.savefig(energy_path, dpi=180, facecolor="white")
    plt.close(fig)

    velocity_path = output_dir / "point-mass-planar-velocity.png"
    delta = data["point_mass_speed"] - data["planar_speed"]
    fig, (speed_ax, delta_ax) = plt.subplots(
        2, 1, figsize=(11.5, 7.5), sharex=True, constrained_layout=True,
        gridspec_kw={"height_ratios": [2.0, 1.0]},
    )
    for key in ("point_mass", "planar"):
        speed_ax.plot(
            distance_km,
            data[f"{key}_speed"],
            color=colors[key],
            lw=1.8,
            label=labels[key],
        )
    speed_ax.set_ylabel("Vehicle speed (m/s)")
    speed_ax.set_title("Velocity profiles along the lap")
    speed_ax.grid(True, alpha=0.25)
    speed_ax.legend(loc="best", frameon=False)
    delta_ax.plot(distance_km, delta, color="#009e73", lw=1.5)
    delta_ax.axhline(0.0, color="black", lw=0.8, alpha=0.7)
    delta_ax.set_ylabel("Point mass − planar (m/s)")
    delta_ax.set_xlabel("Distance along track (km)")
    delta_ax.grid(True, alpha=0.25)
    fig.savefig(velocity_path, dpi=180, facecolor="white")
    plt.close(fig)

    power_path = output_dir / "point-mass-planar-power.png"
    segment_distance_km = data["segment_distance"] / 1000.0
    power_delta_kw = (
        data["point_mass_segment_bus_power"]
        - data["planar_segment_bus_power"]
    ) / 1000.0
    fig, (power_ax, delta_power_ax) = plt.subplots(
        2, 1, figsize=(11.5, 7.5), sharex=True, constrained_layout=True,
        gridspec_kw={"height_ratios": [2.0, 1.0]},
    )
    for key in ("point_mass", "planar"):
        power_ax.plot(
            segment_distance_km,
            data[f"{key}_segment_bus_power"] / 1000.0,
            color=colors[key],
            lw=1.7,
            label=labels[key],
        )
    power_ax.set_ylabel("Segment-average bus power (kW)")
    power_ax.set_title("Electrical power demand along the lap")
    power_ax.grid(True, alpha=0.25)
    power_ax.legend(loc="best", frameon=False)
    delta_power_ax.plot(
        segment_distance_km, power_delta_kw, color="#009e73", lw=1.4
    )
    delta_power_ax.axhline(0.0, color="black", lw=0.8, alpha=0.7)
    delta_power_ax.set_ylabel("Point mass − planar (kW)")
    delta_power_ax.set_xlabel("Distance along track (km)")
    delta_power_ax.grid(True, alpha=0.25)
    fig.savefig(power_path, dpi=180, facecolor="white")
    plt.close(fig)
    return energy_path, velocity_path, power_path


def run(spacing: float, maximum_iterations: int, output_dir: Path) -> dict[str, object]:
    started = time.perf_counter()
    planar_vehicle = compile_formulation(FORMULATION).model
    track = load_track(TRACK)
    point_mass_vehicle, _ = _point_mass_vehicle(planar_vehicle)
    point_mass_track = _point_mass_track(track, spacing)
    path = _build_path_data(track, spacing)

    point_mass = solve_lap_with_energy(
        point_mass_vehicle,
        point_mass_track,
        LapMode.FLYING,
        tolerance=1.0e-5,
        maximum_iterations=40,
        lap_tolerance=1.0e-6,
        lap_maximum_iterations=200,
        regen_enabled=False,
    )
    planar = solve_planar_hot_lap(
        planar_vehicle,
        track,
        PlanarOcpOptions(
            mesh_spacing=spacing,
            maximum_iterations=maximum_iterations,
            equilibrium_seed=True,
            exact_hessian=True,
            print_level=0,
            warm_start=False,
        ),
    )

    distance = np.asarray(path["distance"], dtype=np.float64)
    if not np.allclose(point_mass_track.distance, distance, atol=1.0e-9, rtol=0.0):
        raise RuntimeError("point-mass and planar meshes do not align")
    pm_time = np.asarray(point_mass.elapsed_time, dtype=np.float64)
    point_mass_bus_power = np.asarray(
        point_mass.energy.bus_power, dtype=np.float64
    )
    pm_energy = np.r_[
        0.0,
        np.cumsum(point_mass_bus_power * np.diff(pm_time)),
    ] / 3.6e6
    planar_time = np.asarray(planar.elapsed_time, dtype=np.float64)
    planar_bus_power = np.asarray(planar.bus_power, dtype=np.float64)
    planar_segment_bus_power = 0.5 * (
        planar_bus_power[:-1] + planar_bus_power[1:]
    )
    segment_distance = 0.5 * (distance[:-1] + distance[1:])
    pm_speed = np.asarray(point_mass.speed, dtype=np.float64)
    planar_speed = np.asarray(planar.speed, dtype=np.float64)
    if not (
        distance.shape == pm_speed.shape == planar_speed.shape
        == point_mass.energy.state_trace.shape == planar.state_of_charge.shape
        == planar.bus_power.shape
        and point_mass_bus_power.shape
        == planar_segment_bus_power.shape
        == segment_distance.shape
    ):
        raise RuntimeError("solver traces have incompatible node or segment counts")
    planar_energy = _cumulative_energy_kwh(planar_bus_power, planar_time)
    data = {
        "distance": distance,
        "segment_distance": segment_distance,
        "point_mass_speed": pm_speed,
        "planar_speed": planar_speed,
        "point_mass_soc": np.asarray(point_mass.energy.state_trace),
        "planar_soc": np.asarray(planar.state_of_charge),
        "point_mass_cumulative_energy_kwh": pm_energy,
        "planar_cumulative_energy_kwh": planar_energy,
        "point_mass_segment_bus_power": point_mass_bus_power,
        "planar_segment_bus_power": planar_segment_bus_power,
    }
    energy_path, velocity_path, power_path = _save_plots(data, output_dir)
    np.savez_compressed(output_dir / "point-mass-planar-curves.npz", **data)

    return {
        "lap_length_m": track.lap_length,
        "mesh_spacing_m": spacing,
        "nodes": int(distance.size),
        "point_mass_lap_s": float(point_mass.lap_time),
        "planar_lap_s": float(planar.lap_time),
        "point_mass_final_soc": float(point_mass.energy.final_state_of_charge),
        "planar_final_soc": float(planar.state_of_charge[-1]),
        "point_mass_net_bus_energy_kwh": float(pm_energy[-1]),
        "planar_net_bus_energy_kwh": float(planar_energy[-1]),
        "point_mass_peak_segment_bus_power_kw": float(
            np.max(point_mass_bus_power) / 1000.0
        ),
        "planar_peak_segment_bus_power_kw": float(
            np.max(planar_segment_bus_power) / 1000.0
        ),
        "point_mass_converged": bool(point_mass.converged and point_mass.energy.energy_converged),
        "planar_converged": bool(planar.converged),
        "planar_status": planar.status,
        "elapsed_wall_s": time.perf_counter() - started,
        "outputs": [
            str(energy_path.relative_to(ROOT)),
            str(velocity_path.relative_to(ROOT)),
            str(power_path.relative_to(ROOT)),
            str((output_dir / "point-mass-planar-curves.npz").relative_to(ROOT)),
            str((output_dir / "summary.json").relative_to(ROOT)),
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spacing", type=float, default=1.0)
    parser.add_argument("--iterations", type=int, default=500)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "docs/validation/point-mass-planar-plots",
    )
    args = parser.parse_args()
    report = run(args.spacing, args.iterations, args.output_dir)
    rendered = json.dumps(report, indent=2)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "summary.json").write_text(rendered + "\n", encoding="utf-8")
    print(rendered)


if __name__ == "__main__":
    main()
