#!/usr/bin/env python3
"""Run matched point-mass/planar laps and generate a full trace report."""
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
    PlanarOcpOptions, _build_path_data, solve_planar_hot_lap,
)
from simplified_vehicle_models.tracks.profile import load_track
from compare_planar_point_mass import (
    FORMULATION, ROOT, TRACK, _point_mass_track, _point_mass_vehicle,
)

SIGNALS = ("ax", "ay", "velocity", "dc-power", "cornering-radius")


def _save_plots(data: dict[str, np.ndarray], output: Path) -> list[Path]:
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-config")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=True)
    x = data["segment_distance"] / 1000.0
    colors = {"point_mass": "#d55e00", "planar": "#0072b2"}
    labels = {"point_mass": "Point mass", "planar": "Planar 7-DOF"}
    paths: list[Path] = []
    specs = [
        ("ax", "Longitudinal acceleration", "m/s²"),
        ("ay", "Lateral acceleration", "m/s²"),
        ("velocity", "Vehicle speed", "m/s"),
        ("dc-power", "Signed DC bus power", "kW"),
    ]
    for stem, title, unit in specs:
        fig, ax = plt.subplots(figsize=(12, 5), constrained_layout=True)
        field = {"ax": "ax", "ay": "ay", "velocity": "speed", "dc-power": "bus_power"}[stem]
        scale = 1e-3 if stem == "dc-power" else 1.0
        for model in colors:
            ax.plot(x, data[f"{model}_{field}"] * scale, color=colors[model], lw=1.4, label=labels[model])
        if stem == "dc-power":
            ax.axhline(0, color="black", lw=.8)
        ax.set(title=title, xlabel="Distance along track (km)", ylabel=unit)
        ax.grid(True, alpha=.25); ax.legend(frameon=False)
        path = output / f"{stem}.png"; fig.savefig(path, dpi=180, facecolor="white"); plt.close(fig); paths.append(path)

    fig, ax = plt.subplots(figsize=(12, 5), constrained_layout=True)
    for model in colors:
        ax.plot(x, data[f"{model}_reference_radius"], color=colors[model], lw=1.1, ls="--", label=f"{labels[model]} reference")
        ax.plot(x, data[f"{model}_realized_radius_plot"], color=colors[model], lw=1.5, label=f"{labels[model]} realized")
    ax.set(title="Cornering radius (straight-line values clipped at 1000 m for display)", xlabel="Distance along track (km)", ylabel="Radius (m)")
    ax.grid(True, alpha=.25); ax.legend(frameon=False, ncol=2)
    path = output / "cornering-radius.png"; fig.savefig(path, dpi=180, facecolor="white"); plt.close(fig); paths.append(path)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True, constrained_layout=True)
    order = ("FL", "FR", "RL", "RR")
    for wheel, ax in enumerate(axes.flat):
        for model in colors:
            ax.plot(x, data[f"{model}_fz"][:, wheel], color=colors[model], lw=1.3, label=labels[model])
        ax.set_title(order[wheel]); ax.set_ylabel("Normal load Fz (N)"); ax.grid(True, alpha=.25)
    axes[-1, 0].set_xlabel("Distance along track (km)"); axes[-1, 1].set_xlabel("Distance along track (km)")
    axes[0, 0].legend(frameon=False)
    path = output / "fz-wheels.png"; fig.savefig(path, dpi=180, facecolor="white"); plt.close(fig); paths.append(path)
    return paths


def run(spacing: float, maximum_iterations: int, output_dir: Path, regen: bool = True) -> dict[str, object]:
    started = time.perf_counter()
    vehicle = compile_formulation(FORMULATION).model
    track = load_track(TRACK)
    pm_vehicle, _ = _point_mass_vehicle(vehicle)
    pm_track = _point_mass_track(track, spacing)
    path = _build_path_data(track, spacing)
    point_mass = solve_lap_with_energy(
        pm_vehicle, pm_track, LapMode.FLYING, tolerance=1e-5,
        maximum_iterations=40, lap_tolerance=1e-6, lap_maximum_iterations=200,
        regen_enabled=regen,
    )
    planar = solve_planar_hot_lap(vehicle, track, PlanarOcpOptions(
        mesh_spacing=spacing, maximum_iterations=maximum_iterations,
        equilibrium_seed=True, exact_hessian=True, print_level=0, warm_start=False,
    ))
    distance = np.asarray(path["distance"], dtype=float)
    if not np.allclose(pm_track.distance, distance, atol=1e-9, rtol=0):
        raise RuntimeError("point-mass and planar meshes do not align")
    segment_distance = .5 * (distance[:-1] + distance[1:])
    pm_speed = np.asarray(point_mass.speed)
    planar_speed = np.asarray(planar.speed)
    pm_bus = np.asarray(point_mass.energy.bus_power)
    planar_bus = .5 * (np.asarray(planar.bus_power[:-1]) + np.asarray(planar.bus_power[1:]))
    pm_radius = np.divide(1.0, np.abs(pm_track.curvature), out=np.full_like(pm_track.curvature, np.inf, dtype=float), where=np.abs(pm_track.curvature)>1e-12)
    planar_realized_nodes = np.divide(np.asarray(planar.speed), np.abs(planar.yaw_rate), out=np.full_like(planar.speed, np.inf, dtype=float), where=np.abs(planar.yaw_rate)>1e-9)
    planar_radius = .5 * (planar_realized_nodes[:-1] + planar_realized_nodes[1:])
    ref_radius = np.divide(1.0, np.abs(np.asarray(path["curvature"])), out=np.full_like(distance, np.inf, dtype=float), where=np.abs(np.asarray(path["curvature"]))>1e-12)
    pm_fz = np.asarray(point_mass.normal_loads)
    planar_fz = np.asarray(planar.normal_loads)
    values = {
        "distance": distance, "segment_distance": segment_distance,
        "point_mass_ax": np.asarray(point_mass.longitudinal_acceleration),
        "point_mass_ay": np.asarray(point_mass.lateral_acceleration),
        "point_mass_speed_nodes": pm_speed, "planar_speed_nodes": planar_speed,
        "point_mass_bus_power_nodes": pm_bus,
        "planar_bus_power_nodes": np.asarray(planar.bus_power),
        "point_mass_fz_nodes": pm_fz, "planar_fz_nodes": planar_fz,
        "point_mass_reference_radius_nodes": ref_radius,
        "planar_reference_radius_nodes": ref_radius,
        "planar_ax_nodes": np.asarray(planar.longitudinal_acceleration),
        "planar_ay_nodes": np.asarray(planar.lateral_acceleration),
        "planar_realized_radius_nodes": planar_realized_nodes,
    }
    # Compare segment signals on their common 1 m midpoint mesh. Planar node
    # traces are averaged across adjacent nodes; point-mass dynamics are segment-valued.
    midpoint = lambda array: .5 * (np.asarray(array[:-1]) + np.asarray(array[1:]))
    data = {
        "distance": distance, "segment_distance": segment_distance,
        "point_mass_ax": np.asarray(point_mass.longitudinal_acceleration),
        "planar_ax": midpoint(planar.longitudinal_acceleration),
        "point_mass_ay": np.asarray(point_mass.lateral_acceleration),
        "planar_ay": midpoint(planar.lateral_acceleration),
        "point_mass_speed": midpoint(pm_speed), "planar_speed": midpoint(planar_speed),
        "point_mass_bus_power": pm_bus, "planar_bus_power": planar_bus,
        "point_mass_fz": np.asarray(point_mass.normal_loads),
        "planar_fz": midpoint(planar_fz.T),
        "point_mass_reference_radius": midpoint(ref_radius),
        "planar_reference_radius": midpoint(ref_radius),
        "point_mass_realized_radius": midpoint(ref_radius),
        "planar_realized_radius": planar_radius,
        "point_mass_realized_radius_plot": np.minimum(midpoint(ref_radius),1000),
        "planar_realized_radius_plot": np.minimum(planar_radius,1000),
    }
    if not all(len(np.asarray(data[k])) == len(segment_distance) for k in ("point_mass_ax","planar_ax","point_mass_ay","planar_ay","point_mass_speed","planar_speed","point_mass_bus_power","planar_bus_power")):
        raise RuntimeError("solver trace lengths do not align with segment mesh")
    plots = _save_plots(data, output_dir)
    np.savez_compressed(output_dir / "traces.npz", **data)
    pm_time = np.asarray(point_mass.elapsed_time)
    planar_time = np.asarray(planar.elapsed_time)
    pm_consumed = float(point_mass.energy.consumed_energy / 3.6e6)
    pm_recovered = float(point_mass.energy.recovered_energy / 3.6e6)
    planar_energy_kwh = float(np.trapezoid(planar.bus_power, planar_time) / 3.6e6)
    planar_recovered = float(-np.trapezoid(np.minimum(planar.bus_power,0), planar_time) / 3.6e6)
    planar_consumed = float(np.trapezoid(np.maximum(planar.bus_power,0), planar_time) / 3.6e6)
    summary = {
        "lap_length_m": float(track.lap_length), "mesh_spacing_m": spacing,
        "nodes": int(distance.size), "regeneration_enabled": regen,
        "point_mass": {"lap_time_s": float(point_mass.lap_time), "converged": bool(point_mass.converged and point_mass.energy.energy_converged), "iterations": int(point_mass.iterations), "initial_soc": float(point_mass.energy.state_trace[0]), "final_soc": float(point_mass.energy.final_state_of_charge), "consumed_kwh": pm_consumed, "recovered_kwh": pm_recovered, "net_kwh": float(point_mass.energy.net_energy/3.6e6), "peak_discharge_kw": float(max(0,np.max(pm_bus))/1000), "peak_regen_kw": float(min(0,np.min(pm_bus))/1000)},
        "planar": {"lap_time_s": float(planar.lap_time), "converged": bool(planar.converged), "status": planar.status, "iterations": int(planar.iterations), "initial_soc": float(planar.state_of_charge[0]), "final_soc": float(planar.state_of_charge[-1]), "consumed_kwh": planar_consumed, "recovered_kwh": planar_recovered, "net_kwh": planar_energy_kwh, "peak_discharge_kw": float(max(0,np.max(planar.bus_power))/1000), "peak_regen_kw": float(min(0,np.min(planar.bus_power))/1000)},
        "trace_definitions": {"common_x": "segment midpoint distance (m), common 1 m mesh", "ax_ay_speed_fz_power": "point-mass segment signals; planar adjacent node values averaged to segment midpoint", "dc_power_sign": "positive discharge, negative battery charging; W in archive and kW in figures", "point_mass_radius": "centerline/reference radius 1/abs(curvature)", "planar_radius": "ground speed/absolute yaw rate; infinity for straight lines; plotting clipped to 1000 m", "wheel_order": ["FL","FR","RL","RR"], "normal_load_unit":"N", "acceleration_unit":"m/s^2", "speed_unit":"m/s", "radius_unit":"m"},
        "model_limits": {"drive_wheel_shares_FL_FR_RL_RR":[0.1,0.1,0.4,0.4], "regeneration_torque_fraction":float(vehicle.motor.regeneration_torque_fraction), "charge_current_limit_A":float(vehicle.storage.charge_current_limit)},
        "elapsed_wall_s": time.perf_counter()-started,
        "outputs": [p.name for p in plots]+["traces.npz","summary.json","full-report.md"],
    }
    (output_dir/"summary.json").write_text(json.dumps(summary,indent=2)+"\n",encoding="utf-8")
    report = ["# Regenerative Full-Vehicle Comparison", "", "Matched placeholder-vehicle runs on the 2024 Michigan Endurance closed track.", "", "## Results", "", "| Metric | Point mass | Planar 7-DOF |", "|---|---:|---:|"]
    for key,label in [("lap_time_s","Lap time (s)"),("initial_soc","Initial SOC"),("final_soc","Final SOC"),("consumed_kwh","Consumed energy (kWh)"),("recovered_kwh","Recovered energy (kWh)"),("net_kwh","Net energy (kWh)"),("peak_discharge_kw","Peak discharge power (kW)"),("peak_regen_kw","Peak regeneration power (kW; negative)")]:
        report.append(f"| {label} | {summary['point_mass'][key]:.4f} | {summary['planar'][key]:.4f} |")
    report += ["", f"Convergence: point mass **{summary['point_mass']['converged']}**; planar **{summary['planar']['converged']}** ({planar.status}). Iterations: {summary['point_mass']['iterations']} / {planar.iterations}.", "", "## Trace definitions", "", "All figures share track distance in km and traces are sampled at the common 1 m segment midpoints. Planar node traces are averaged across adjacent nodes; point-mass acceleration/load/power traces are segment-valued. DC power is signed (discharge positive, accepted regeneration negative). Point-mass realized radius is the centerline radius; planar realized radius is ground speed divided by absolute yaw rate, with straight-line infinity clipped to 1000 m only in the figure. Wheel ordering is FL, FR, RL, RR.", "", "Figures: " + ", ".join(f"`{p.name}`" for p in plots), ""]
    (output_dir/"full-report.md").write_text("\n".join(report),encoding="utf-8")
    return summary


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spacing",type=float,default=1.0)
    parser.add_argument("--iterations",type=int,default=500)
    parser.add_argument("--regen",action="store_true",help="enable regeneration (enabled by default)")
    parser.add_argument("--no-regen",dest="regen",action="store_false",help="disable regeneration")
    parser.set_defaults(regen=True)
    parser.add_argument("--output-dir",type=Path,default=ROOT/"docs/validation/point-mass-planar-plots")
    args=parser.parse_args()
    print(json.dumps(run(args.spacing,args.iterations,args.output_dir,args.regen),indent=2))

if __name__ == "__main__": main()
