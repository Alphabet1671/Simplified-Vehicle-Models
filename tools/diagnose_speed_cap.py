#!/usr/bin/env python3
"""Diagnose why forward driver speed is capped below 8 m/s."""
from __future__ import annotations

import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent / "src"))

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import (
    DriveCache,
    DriveCachePolicy,
    maximum_acceleration_limit,
    maximum_corner_speed,
    maximum_deceleration_limit,
    maximum_vehicle_speed,
)
from simplified_vehicle_models.powertrain.energy_storage import StorageState
from simplified_vehicle_models.solvers.forward_driver import _build_live_limit_speed_plan


#: Canonical ten-lap MIS track (repository default).
TRACK = "models/tracks/2024_michigan_endurance_multilap_closed.xml"


def main():
    assembly = compile_formulation("models/formulations/murphy_point_mass.xml")
    vehicle = assembly.model
    storage_state = StorageState(0.8291)

    print("=" * 70)
    print("Vehicle speed ceiling")
    print("=" * 70)
    max_speed = maximum_vehicle_speed(vehicle, storage_state)
    print(f"maximum_vehicle_speed: {max_speed:.3f} m/s ({max_speed * 3.6:.1f} km/h)")
    print(f"tire effective_radius: {vehicle.tire.effective_radius:.4f} m")

    print("\n" + "=" * 70)
    print("Loading track data")
    print("=" * 70)
    # Use measured/OCP comparison grid for distances and infer curvature
    comp_path = Path("docs/validation/iem24-murphy/comparison.csv")
    with comp_path.open() as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    distance = np.array([float(r["track_distance_m"]) for r in rows])
    meas_speed = np.array([float(r["measured_speed_mps"]) for r in rows])
    meas_ay = np.array([float(r["measured_lateral_acceleration_mps2"]) for r in rows])
    meas_yaw_rate = np.array([float(r["measured_yaw_rate_rad_s"]) for r in rows])

    # Infer signed curvature from measured data
    with np.errstate(divide="ignore", invalid="ignore"):
        curvature = np.where(
            meas_speed > 1.0,
            meas_ay / (meas_speed ** 2) * np.sign(meas_yaw_rate),
            0.0,
        )
    curvature = np.nan_to_num(curvature, nan=0.0, posinf=0.0, neginf=0.0)
    # Smooth slightly to avoid spikes
    curvature = np.convolve(curvature, np.ones(5) / 5, mode="same")

    print(f"distance grid: {distance.size} points, {distance[0]:.2f} to {distance[-1]:.2f} m")
    print(f"curvature range: {curvature.min():.4f} to {curvature.max():.4f} 1/m")
    print(f"abs curvature max: {np.max(np.abs(curvature)):.4f} 1/m")
    print(f"minimum radius: {1.0 / np.max(np.abs(curvature)):.3f} m")

    print("\n" + "=" * 70)
    print("Per-node live limits")
    print("=" * 70)
    drive_cache = DriveCache(DriveCachePolicy(speed_resolution=1.0, soc_resolution=0.05))
    reference_speed = 10.0

    max_speeds = []
    max_accs = []
    max_decs = []
    for i in range(distance.size):
        abs_curv = abs(curvature[i])
        if abs_curv > 1.0e-12:
            radius = 1.0 / abs_curv
            try:
                v_corner = min(maximum_corner_speed(vehicle, radius, tolerance=1.0e-8), 60.0)
            except Exception as exc:
                print(f"  node {i}: corner speed error {exc}")
                v_corner = 8.0
        else:
            v_corner = 60.0
        max_speeds.append(v_corner)

        try:
            acc = maximum_acceleration_limit(
                vehicle, reference_speed, curvature[i], storage_state, drive_cache, tolerance=1.0e-8
            )
            dec = maximum_deceleration_limit(
                vehicle, reference_speed, curvature[i], storage_state, drive_cache, tolerance=1.0e-8
            )
        except Exception as exc:
            print(f"  node {i}: accel limit error {exc}")
            acc, dec = 2.0, 3.0
        max_accs.append(acc)
        max_decs.append(dec)

    max_speeds = np.array(max_speeds)
    max_accs = np.array(max_accs)
    max_decs = np.array(max_decs)

    print(f"corner speed range: {max_speeds.min():.3f} to {max_speeds.max():.3f} m/s")
    print(f"corner speed < 8 m/s: {np.sum(max_speeds < 8.0)} / {max_speeds.size} nodes")
    print(f"corner speed < 10 m/s: {np.sum(max_speeds < 10.0)} / {max_speeds.size} nodes")
    print(f"corner speed mean: {max_speeds.mean():.3f} m/s")
    print(f"max acceleration range: {max_accs.min():.3f} to {max_accs.max():.3f} m/s²")
    print(f"max deceleration range: {max_decs.min():.3f} to {max_decs.max():.3f} m/s²")
    print(f"non-positive accelerations: {np.sum(max_accs <= 0.0)}")
    print(f"non-positive decelerations: {np.sum(max_decs <= 0.0)}")

    # Find where corner speed is minimum
    min_idx = np.argmin(max_speeds)
    print(f"minimum corner speed {max_speeds[min_idx]:.3f} m/s at distance {distance[min_idx]:.1f} m, curvature {curvature[min_idx]:.4f} 1/m")

    print("\n" + "=" * 70)
    print("Full live-limit speed plan (MIS grid)")
    print("=" * 70)
    from simplified_vehicle_models.tracks.profile import load_track
    from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
    track = load_track(TRACK)
    spacing = 0.5
    count = int(np.ceil(track.lap_length / spacing))
    distance_grid = np.linspace(0.0, track.lap_length, count + 1)
    _, _, _, curvature_grid = evaluate_path(track.path_coordinates, distance_grid)

    print(f"track length: {track.lap_length:.3f} m")
    print(f"grid: {distance_grid.size} points, {distance_grid[0]:.2f} to {distance_grid[-1]:.2f} m")

    plan = _build_live_limit_speed_plan(
        distance_grid, curvature_grid, vehicle, storage_state, tolerance=1.0e-8
    )
    print(f"plan speed range: {plan.speed.min():.3f} to {plan.speed.max():.3f} m/s")
    print(f"plan speed < 8 m/s: {np.sum(plan.speed < 8.0)} / {plan.speed.size} nodes")
    print(f"plan speed < 10 m/s: {np.sum(plan.speed < 10.0)} / {plan.speed.size} nodes")
    print(f"plan speed mean: {plan.speed.mean():.3f} m/s")

    # Find minimum plan speed location
    plan_min_idx = np.argmin(plan.speed)
    print(f"minimum plan speed {plan.speed[plan_min_idx]:.3f} m/s at distance {plan.distance[plan_min_idx]:.1f} m")

    # Show first few plan points
    print("\nFirst 10 plan points:")
    for i in range(min(10, plan.speed.size)):
        print(f"  d={plan.distance[i]:.2f} m, v={plan.speed[i]:.3f} m/s, k={plan.curvature[i]:.4f} 1/m, a={plan.acceleration[i]:.3f} m/s²")

    print("\n" + "=" * 70)
    print("Conservative planner default limits")
    print("=" * 70)
    print("Default build_speed_plan limits: max_speed=8.0, lat_acc=3.0, drive_acc=2.0, brake_acc=3.0")


if __name__ == "__main__":
    main()
