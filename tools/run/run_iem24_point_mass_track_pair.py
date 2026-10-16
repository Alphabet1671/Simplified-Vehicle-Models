#!/usr/bin/env python3
"""Run the same supported IEM24 energy-coupled point-mass solve on two signed tracks."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import time
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import numpy as np
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.results.traces import LapMode
from simplified_vehicle_models.powertrain.energy_storage import (StorageState,
    solve_current, terminal_voltage)
from simplified_vehicle_models.solvers.forward_backward import solve_lap_with_energy
from simplified_vehicle_models.tracks.profile import load_track
from tools.reports.compare_planar_point_mass import FORMULATION, _point_mass_track, _point_mass_vehicle

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TRACKS = (Path('models/tracks/2024_michigan_endurance_closed.xml'),
                  Path('docs/validation/iem24-murphy/multilap_track_candidate.xml'))


def export_run(vehicle, track_path: Path, output: Path, spacing: float) -> dict:
    loaded = load_track(ROOT / track_path)
    track = _point_mass_track(loaded, spacing)
    start = time.perf_counter()
    solution = solve_lap_with_energy(vehicle, track, LapMode.FLYING,
        tolerance=1.e-5, maximum_iterations=40, lap_tolerance=1.e-6,
        lap_maximum_iterations=200, regen_enabled=False)
    wall = time.perf_counter() - start
    if not solution.lap.converged or not solution.energy.energy_converged:
        raise RuntimeError(f'{track_path}: solver or energy iteration failed to converge')
    lap, energy = solution.lap, solution.energy
    n = len(track.distance)
    def node_values(segments):
        return np.r_[segments, segments[-1]]
    # Consistent per-motor request computed from driven rear-wheel force; no
    # assumption that throttle fraction equals the motor's 10.5 Nm limit.
    runtime = vehicle.drivetrain
    motor = runtime.powertrain.power_plant
    gear = motor.gear_ratio
    efficiency = runtime.drivetrain.transmission.gear_efficiency
    force = np.maximum(0., np.sum(lap.longitudinal_forces, axis=1))
    torque = force * vehicle.tire.effective_radius / (motor.unit_count * gear * efficiency)
    torque = np.minimum(torque, 10.5)
    # The solver records open-circuit pack voltage for its storage trace.
    # Reconstruct *loaded terminal* voltage from each segment bus power and
    # SOC instead of mislabeling that trace as terminal sensor voltage.
    storage = runtime.powertrain.energy_storage
    terminal_segments = np.asarray([
        terminal_voltage(storage, StorageState(float(energy.state_trace[i])),
            solve_current(storage, StorageState(float(energy.state_trace[i])),
                          float(energy.bus_power[i]))[0])
        for i in range(len(energy.bus_power))], dtype=float)
    voltage = node_values(terminal_segments)
    # Energy solver's bus power is total electrical power; symmetric two-motor
    # point mass has no left/right split beyond an equal allocation.
    per_motor_power = node_values(energy.bus_power) / motor.unit_count / 1000.
    result = np.column_stack((track.distance, lap.elapsed_time, lap.speed,
        node_values(lap.longitudinal_acceleration), node_values(lap.lateral_acceleration),
        node_values(lap.lateral_acceleration / np.maximum((lap.speed[:-1] + lap.speed[1:]) / 2., 1.e-8)),
        node_values(torque), voltage, per_motor_power, per_motor_power,
        node_values(lap.throttle), node_values(lap.brake)))
    output.mkdir(parents=True, exist_ok=True)
    names = 'distance_m,elapsed_time_s,speed_mps,longitudinal_acceleration_mps2,lateral_acceleration_mps2,yaw_rate_rad_s,motor_torque_request_nm,pack_voltage_v,rl_power_kw,rr_power_kw,throttle_fraction,brake_fraction'
    np.savetxt(output / 'simulated_lap.csv', result, delimiter=',', header=names, comments='')
    data = ROOT / track_path
    csv = data.with_suffix('.csv')
    summary = dict(track=str(track_path), track_sha256=hashlib.sha256(data.read_bytes()).hexdigest(),
        geometry_sha256=hashlib.sha256(csv.read_bytes()).hexdigest(), formulation=str(FORMULATION.relative_to(ROOT)),
        track_length_m=float(loaded.lap_length), nodes=n, spacing_m=spacing, lap_time_s=float(lap.lap_time),
        converged=bool(lap.converged), iterations=int(lap.iterations),
        energy_converged=bool(energy.energy_converged), energy_iterations=int(energy.energy_iterations),
        initial_state_of_charge=float(energy.state_trace[0]), final_state_of_charge=float(energy.final_state_of_charge),
        initial_terminal_voltage_v=float(voltage[0]), final_terminal_voltage_v=float(voltage[-1]),
        regen_enabled=False, motor_count=int(motor.unit_count), gear_ratio_used=float(gear),
        maximum_motor_torque_request_nm=float(np.max(torque)), solve_wall_seconds=wall,
        method='energy-coupled minimum-time point-mass flying lap',
        motor_torque_note='positive traction wheel force reconstructed per motor, clipped at 10.5 Nm; braking request = 0')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=Path('docs/validation/iem24-murphy/point_mass_track_pair'))
    parser.add_argument('--spacing', type=float, default=.5)
    args = parser.parse_args()
    if args.spacing <= 0 or not np.isfinite(args.spacing):
        parser.error('spacing must be positive and finite')
    planar = compile_formulation(FORMULATION).model
    vehicle, _ = _point_mass_vehicle(planar)
    for name, track in zip(('original', 'experimental'), DEFAULT_TRACKS):
        summary = export_run(vehicle, track, args.output_dir / name, args.spacing)
        print(name, json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
