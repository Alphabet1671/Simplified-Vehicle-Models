#!/usr/bin/env python3
"""Compare the supported real-IEM24 point-mass solver on the signed MIS path.

The planar OCP solver and point-mass-seed workflow were retired. This script
now provides a point-mass-only run summary and refuses to claim a planar
comparison; use the transient forward-driver for an opt-in planar simulation.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import platform
import time

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.parameters.vehicle import PointMassVehicleParameters
from simplified_vehicle_models.powertrain.assembly import compile_powertrain_drivetrain
from simplified_vehicle_models.results.traces import LapMode
from simplified_vehicle_models.solvers import forward_backward
from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
from simplified_vehicle_models.tires.unitire_envelope import (
    build_unitire_peak_envelope,
)
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.types import TrackProfile
from simplified_vehicle_models.xml.loader import load_model


ROOT = Path(__file__).resolve().parents[2]
FORMULATION = ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
ASSEMBLY = ROOT / "models/vehicles/iem24_murphy_real_planar.xml"
TRACK = ROOT / "models/tracks/2024_michigan_endurance_multilap_closed.xml"


def _point_mass_track(track: TrackProfile, spacing: float) -> TrackProfile:
    """Sample the same signed XY path on the requested comparison mesh."""

    if not np.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("spacing must be positive and finite")
    coordinates = track.path_coordinates
    if coordinates is None:
        raise ValueError("point-mass comparison requires signed closed-path geometry")
    intervals = max(1, int(np.ceil(track.lap_length / spacing)))
    distance = np.linspace(0.0, track.lap_length, intervals + 1)
    _, _, _, curvature = evaluate_path(coordinates, distance)
    radius = np.divide(
        1.0,
        np.abs(curvature),
        out=np.full_like(curvature, 1.0e12),
        where=np.abs(curvature) > 1.0e-12,
    )
    return TrackProfile(
        distance=distance,
        segment_length=np.diff(distance),
        radius=radius,
        curvature=curvature,
        is_closed=track.is_closed,
        title=track.title,
        path_coordinates=None,
    )


def _point_mass_vehicle(planar_vehicle) -> tuple[PointMassVehicleParameters, float]:
    """Reduce the same compiled assembly and UniTire into point-mass inputs."""

    envelope = build_unitire_peak_envelope(planar_vehicle.tire)
    runtime = compile_powertrain_drivetrain(load_model(ASSEMBLY))
    source_transmission_ratio = runtime.drivetrain.transmission.gear_ratio
    motor = runtime.powertrain.power_plant
    # The smooth motor model owns the ratio used by planar-7dof. The assembly
    # also carries a distinct transmission-ratio field (10.9) that the planar
    # equations do not use; point-mass support dispatches to the motor ratio.
    vehicle = PointMassVehicleParameters(
        total_mass=planar_vehicle.total_mass,
        wheelbase=planar_vehicle.wheelbase,
        track_width=planar_vehicle.track_width,
        cg_height=planar_vehicle.cg_height,
        front_mass_fraction=planar_vehicle.front_mass_fraction,
        lateral_load_transfer_distribution=(
            planar_vehicle.lateral_load_transfer_distribution
        ),
        brake_bias=planar_vehicle.brake_bias,
        tire=envelope,
        aero=planar_vehicle.aero,
        drivetrain=runtime,
    )
    return vehicle, source_transmission_ratio


def _result_summary(result) -> dict[str, object]:
    return {
        "lap_time_s": float(result.lap_time),
        "converged": bool(result.converged),
        "iterations": int(result.iterations),
        "min_speed_mps": float(np.min(result.speed)),
        "max_speed_mps": float(np.max(result.speed)),
        "mean_speed_mps": float(np.mean(result.speed)),
    }


def run(spacing: float, maximum_iterations: int = 200) -> dict[str, object]:
    del maximum_iterations  # Retained as a CLI compatibility argument.
    setup_start = time.perf_counter()
    vehicle = compile_formulation(FORMULATION).model
    source_track = load_track(TRACK)
    pm_vehicle, source_transmission_ratio = _point_mass_vehicle(vehicle)
    pm_track = _point_mass_track(source_track, spacing)
    setup_seconds = time.perf_counter() - setup_start

    pm_start = time.perf_counter()
    pm_result = forward_backward.solve_lap_with_energy(
        pm_vehicle,
        pm_track,
        LapMode.FLYING,
        tolerance=1.0e-5,
        maximum_iterations=40,
        lap_tolerance=1.0e-6,
        lap_maximum_iterations=200,
        regen_enabled=False,
    )
    pm_seconds = time.perf_counter() - pm_start
    pm_lap = float(pm_result.lap_time)
    path_node_count = int(pm_track.distance.size)
    envelope = pm_vehicle.tire
    lookup_loads = np.array([250.0, 660.0, 1500.0, 2500.0])
    fx, fy = envelope.force_capacities(lookup_loads)
    grip = [
        {
            "vertical_load_n": float(load),
            "longitudinal_peak_mu": float(x / load),
            "lateral_peak_mu": float(y / load),
            "longitudinal_peak_force_n": float(x),
            "lateral_peak_force_n": float(y),
        }
        for load, x, y in zip(lookup_loads, fx, fy, strict=True)
    ]
    return {
        "vehicle_formulation": str(FORMULATION.relative_to(ROOT)),
        "vehicle_assembly": str(ASSEMBLY.relative_to(ROOT)),
        "track": str(TRACK.relative_to(ROOT)),
        "track_title": source_track.title,
        "lap_distance_m": source_track.lap_length,
        "mesh_spacing_m": spacing,
        "nodes": path_node_count,
        "tire_source": "simplified UniTire pure-slip peaks",
        "tire_reference_speed_mps": envelope.reference_speed,
        "tire_lookup_nodes": int(envelope.vertical_loads.size),
        "tire_peak_samples": grip,
        "point_mass": {
            **_result_summary(pm_result.lap),
            "energy_converged": bool(pm_result.energy.energy_converged),
            "energy_iterations": int(pm_result.energy.energy_iterations),
            "final_soc": float(pm_result.energy.final_state_of_charge),
            "minimum_pack_voltage_v": float(pm_result.energy.minimum_voltage),
        },
        "planar_comparison": {
            "available": False,
            "reason": "the planar OCP and point-mass-seed APIs are retired",
            "supported_alternative": "tools/run/run_iem24_forward_driver.py",
        },
        "compute_time_seconds": {
            "input_compilation_and_lookup_setup": setup_seconds,
            "point_mass_energy_coupled_solve": pm_seconds,
        },
        "environment": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "solver_options": {
                "point_mass_lap_maximum_iterations": 200,
                "point_mass_energy_maximum_iterations": 40,
            },
            "point_mass_motor_gear_ratio_used": (
                pm_vehicle.drivetrain.powertrain.power_plant.gear_ratio
            ),
            "source_transmission_gear_ratio": source_transmission_ratio,
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spacing", type=float, default=1.0)
    parser.add_argument(
        "--iterations", type=int, default=2500,
        help="deprecated OCP option, ignored by this point-mass-only tool",
    )
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    report = run(args.spacing, args.iterations)
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
