#!/usr/bin/env python3
"""Compare point-mass and planar lap solvers on one model and one path.

The point-mass force envelope is sampled from the simplified UniTire model
referenced by the placeholder planar formulation. No legacy Matlab lap-sim
tire coefficients are read. The report separates setup, point-mass solve,
planar solve, and point-mass-seed projection/solve wall time.
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
from simplified_vehicle_models.solvers import forward_backward, planar_ocp
from simplified_vehicle_models.solvers.planar_equilibrium import equilibrium_start
from simplified_vehicle_models.solvers.point_mass_seed import (
    point_mass_planar_primal_seed,
)
from simplified_vehicle_models.tires.unitire_envelope import (
    build_unitire_peak_envelope,
)
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.types import TrackProfile
from simplified_vehicle_models.xml.loader import load_model


ROOT = Path(__file__).resolve().parents[1]
FORMULATION = ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
ASSEMBLY = ROOT / "models/vehicles/iem24_murphy_real_planar.xml"
TRACK = ROOT / "models/tracks/2024_michigan_endurance_closed.xml"


def _point_mass_track(track: TrackProfile, spacing: float) -> TrackProfile:
    """Sample the same signed XY path on the requested comparison mesh."""

    path = planar_ocp._build_path_data(track, spacing)
    curvature = path["curvature"]
    radius = np.divide(
        1.0,
        np.abs(curvature),
        out=np.full_like(curvature, 1.0e12),
        where=np.abs(curvature) > 1.0e-12,
    )
    return TrackProfile(
        distance=path["distance"],
        segment_length=np.diff(path["distance"]),
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


def run(spacing: float, maximum_iterations: int) -> dict[str, object]:
    setup_start = time.perf_counter()
    planar_vehicle = compile_formulation(FORMULATION).model
    source_track = load_track(TRACK)
    pm_vehicle, source_transmission_ratio = _point_mass_vehicle(planar_vehicle)
    pm_track = _point_mass_track(source_track, spacing)
    path = planar_ocp._build_path_data(source_track, spacing)
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

    planar_options = planar_ocp.PlanarOcpOptions(
        mesh_spacing=spacing,
        maximum_iterations=maximum_iterations,
        equilibrium_seed=True,
        exact_hessian=True,
        print_level=0,
        warm_start=False,
    )
    cold_start = time.perf_counter()
    planar_result = planar_ocp.solve_planar_hot_lap(
        planar_vehicle, source_track, planar_options
    )
    planar_seconds = time.perf_counter() - cold_start

    warm_start = time.perf_counter()
    mapped = point_mass_planar_primal_seed(
        pm_result.lap,
        pm_track.distance,
        path,
        planar_vehicle,
    )
    # Retain the algebraically balanced equilibrium seed's sideslip and
    # steering, which the point-mass model cannot provide. Use point mass only
    # for the speed/time profile and approximate longitudinal pedals.
    motor = planar_vehicle.motor
    peak_wheel = (
        motor.peak_motor_torque * motor.gear_ratio * motor.motor_efficiency
    )
    functions = planar_ocp._build_node_functions(
        planar_ocp._require_casadi(),
        planar_vehicle,
        4.0 * peak_wheel,
        planar_ocp._brake_torque_magnitudes(planar_vehicle),
    )
    equilibrium = equilibrium_start(
        planar_vehicle,
        path,
        functions,
        planar_options.equilibrium_speed,
    )
    state = equilibrium["state"].copy()
    speed = mapped["state"][2]
    side_ratio = state[3] / np.maximum(state[2], 1.0e-12)
    state[2] = speed
    state[3] = side_ratio * speed
    state[4] = path["curvature"] * np.hypot(state[2], state[3])
    state[5] = planar_vehicle.storage.initial_state_of_charge
    state[6] = mapped["state"][6]
    control = equilibrium["control"].copy()
    # Point-mass pedal fractions satisfy a different force balance. Estimate
    # only initial longitudinal pedal direction/magnitude from its speed slope,
    # matching the planar cold-start heuristic instead of copying those pedals.
    acceleration = np.gradient(speed**2, 2.0 * path["distance"])
    control[0] = np.clip(acceleration / 6.0, 0.0, 1.0)
    control[1] = np.clip(-acceleration / 12.0, 0.0, planar_options.maximum_brake)
    seed = {
        "distance": path["distance"],
        "state": state,
        "control": control,
        "algebraic": equilibrium["algebraic"],
    }
    warm_options = replace(planar_options, project_initial_guess=True)
    warm_result = planar_ocp._solve(
        planar_ocp._require_casadi(),
        planar_vehicle,
        source_track,
        warm_options,
        seed,
    )
    warm_seconds = time.perf_counter() - warm_start

    pm_lap = float(pm_result.lap_time)
    planar_lap = float(planar_result.lap_time)
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
        "nodes": int(path["distance"].size),
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
        "planar_equilibrium_seed": {
            **_result_summary(planar_result),
            "ipopt_status": planar_result.status,
            "max_algebraic_residual": planar_result.maximum_algebraic_residual,
            "max_dynamics_residual": planar_result.maximum_dynamics_residual,
            "max_inequality_violation": planar_result.maximum_inequality_violation,
            "max_power_mismatch_w": planar_result.maximum_power_mismatch,
            "max_steering_rate_rad_s": planar_result.maximum_steering_rate,
        },
        "point_mass_seeded_planar": {
            **_result_summary(warm_result),
            "ipopt_status": warm_result.status,
            "max_algebraic_residual": warm_result.maximum_algebraic_residual,
            "max_dynamics_residual": warm_result.maximum_dynamics_residual,
            "max_inequality_violation": warm_result.maximum_inequality_violation,
            "max_power_mismatch_w": warm_result.maximum_power_mismatch,
            "max_steering_rate_rad_s": warm_result.maximum_steering_rate,
            "projected_start": True,
            "initialization_strategy": (
                "point-mass speed/time; equilibrium sideslip/steering; "
                "speed-slope pedals; local algebraic projection"
            ),
        },
        "comparison": {
            "point_mass_minus_planar_seconds": pm_lap - planar_lap,
            "point_mass_minus_planar_percent": 100.0 * (pm_lap - planar_lap) / planar_lap,
            "planar_equilibrium_iterations_saved_by_point_mass_seed": (
                planar_result.iterations - warm_result.iterations
            ),
        },
        "compute_time_seconds": {
            "input_compilation_and_lookup_setup": setup_seconds,
            "point_mass_energy_coupled_solve": pm_seconds,
            "planar_equilibrium_seed_solve": planar_seconds,
            "point_mass_seed_projection_and_planar_solve": warm_seconds,
            "planar_to_point_mass_compute_speedup": planar_seconds / pm_seconds,
            "point_mass_seeded_planar_vs_equilibrium_speedup": planar_seconds / warm_seconds,
        },
        "environment": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "solver_options": {
                "maximum_iterations": maximum_iterations,
                "point_mass_lap_maximum_iterations": 200,
                "point_mass_energy_maximum_iterations": 40,
                "planar_exact_hessian": True,
                "planar_steering_rate_limit_rad_s": planar_vehicle.steering_maximum_rate,
                "maximum_lateral_error_m": planar_options.maximum_lateral_error,
                "maximum_heading_error_rad": planar_options.maximum_heading_error,
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
    parser.add_argument("--iterations", type=int, default=2500)
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
