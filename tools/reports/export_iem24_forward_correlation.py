#!/usr/bin/env python3
"""Export a freshly integrated 7DoF forward-driver lap for correlation plots.

Replays accepted states using the current plant to recover body accelerations and
per-motor electrical power; the trace is a driven lap, not an OCP solution.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import hashlib

import numpy as np

# Attempts that predate --track carry no settings["track"]; they were all run on
# the original track, so that is the documented fallback for historical runs.
LEGACY_TRACK = "models/tracks/2024_michigan_endurance_closed.xml"
FORMULATION = "models/formulations/iem24_murphy_real_planar_7dof.xml"


def verify_track_provenance(summary: dict) -> str:
    """Return the run's track XML path after checking recorded model hashes.

    Rejects a replay whose track/tire/formulation files differ from what the
    attempt recorded, or whose recorded track disagrees with its settings.
    """
    track = str(summary["settings"].get("track") or LEGACY_TRACK)
    inputs = summary.get("model_inputs")
    if inputs is None:
        if summary["settings"].get("track") is not None:
            raise ValueError("attempt names a track but records no model_inputs hashes")
        return track  # historical attempt: fallback track, no recorded hashes
    if str(inputs.get("track_xml")) != track:
        raise ValueError("recorded track_xml does not match the run's track setting")
    if str(inputs.get("formulation_xml")) != FORMULATION:
        raise ValueError("attempt used a different formulation than this exporter replays")
    hashes = summary.get("source_hashes", {})
    for role in ("track_xml", "track_csv", "tire_xml", "formulation_xml", "vehicle_xml"):
        path = inputs.get(role)
        if path is None or path not in hashes:
            raise ValueError(f"attempt records no hash for {role}")
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != hashes[path]:
            raise ValueError(f"{role} {path} differs from the file used by the attempt")
    return track


def export(attempt: Path) -> Path:
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.tracks.profile import load_track
    from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
    from simplified_vehicle_models.solvers.forward_driver import build_speed_plan, PreviewDriver
    from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant, body_accelerations

    summary = json.loads((attempt / "summary.json").read_text(encoding="utf-8"))
    if summary.get("lap_completed") is not True or summary.get("status") != "Completed":
        raise ValueError("only a newly completed full 7DoF lap may be exported")
    settings = summary["settings"]
    track_path = verify_track_provenance(summary)
    vehicle = compile_formulation(FORMULATION).model
    track = load_track(track_path)
    if not track.has_signed_geometry:
        raise ValueError("track must carry signed geometry")
    recorded_length = summary.get("track_length")
    if recorded_length is not None and abs(float(recorded_length) - track.lap_length) > 1.e-6:
        raise ValueError("track length differs from the attempt's recorded track length")
    count = int(np.ceil(track.lap_length / float(settings["spacing"])))
    distance = np.linspace(0., track.lap_length, count + 1)
    _, _, _, curvature = evaluate_path(track.path_coordinates, distance)
    live = settings["planner"] == "live"
    lateral = settings.get("lateral_acceleration")
    if lateral is None and not live:
        lateral = 2.
    plan = build_speed_plan(
        distance, curvature, float(settings["maximum_speed"]), lateral, 1., 2.,
        vehicle=vehicle if live else None, live_limit=live,
    )
    plant = PlanarForwardPlant(vehicle, plan)
    driver = PreviewDriver(
        plan, vehicle.wheelbase, vehicle.steering_maximum_angle,
        vehicle.steering_maximum_rate, vehicle.maximum_pedal_force,
        live_limit=live, pedal_travel_time=settings.get("pedal_travel_time"),
    )
    trace = np.loadtxt(attempt / "trace.csv", delimiter=",", skiprows=1)
    if trace.ndim != 2 or trace.shape[1] != 1 + plant.state_size:
        raise ValueError(
            f"expected time and {plant.state_size} forward-driver states")
    pedal_path = attempt / "pedal.csv"
    if driver.has_pedal_dynamics:
        if not pedal_path.is_file():
            raise ValueError("pedal dynamics require a matching pedal.csv")
        pedal = np.loadtxt(pedal_path, delimiter=",", skiprows=1)
        if pedal.shape != (len(trace), 2) or not np.array_equal(pedal[:, 0], trace[:, 0]):
            raise ValueError("pedal samples must match accepted trace samples")
    else:
        pedal = None
    states = trace[:, 1:]
    # Column indices come from the plant's published layout, not literals.
    names = tuple(plant.state_names)
    wheel_speeds = slice(names.index("omega_fl"), names.index("omega_rr") + 1)
    rates = np.empty_like(states)
    powers = np.empty((len(trace), 2))
    throttle = np.empty(len(trace))
    for i, (time, state) in enumerate(zip(trace[:, 0], states)):
        command = driver.command(plant.observe(state))
        if pedal is not None:
            command = driver.realized_command(float(pedal[i, 1]), command)
        rates[i] = plant.evaluate(float(time), state, command).solve()
        operating = plant.powertrain(state, command)
        # Motor-wheel torque includes gear efficiency; shaft electrical input
        # reverses the gear and drive losses for each driven wheel.
        shaft_power = (operating.motor_wheel_torques[plant.driven]
                       * state[wheel_speeds][plant.driven])
        powers[i] = np.where(
            shaft_power >= 0.,
            shaft_power / (plant.gear_efficiency * vehicle.motor.drive_efficiency),
            shaft_power * plant.gear_efficiency * vehicle.motor.drive_efficiency,
        ) / 1000.
        throttle[i] = command.throttle
    ax, ay = body_accelerations(states, rates)
    output = attempt / "simulated_lap_7dof.csv"
    np.savetxt(
        output,
        np.column_stack((states[:, names.index("s")], trace[:, 0],
                         np.hypot(states[:, names.index("v_x")], states[:, names.index("v_y")]),
                         ax, ay, powers[:, 0], powers[:, 1], throttle)),
        delimiter=",", comments="",
        header="distance_m,elapsed_time_s,velocity_mps,longitudinal_acceleration_mps2,lateral_acceleration_mps2,rl_power_kw,rr_power_kw,apps_fraction",
    )
    export_summary = {
        "source_attempt": str(attempt),
        "track": track_path,
        "track_source_hashes": {key: summary["source_hashes"][path]
                                for key, path in summary.get("model_inputs", {}).items()},
        "method": summary["method"], "status": summary["status"],
        "lap_completed": summary["lap_completed"], "usable": summary["usable"],
        "minimum_time_optimum": summary["minimum_time_optimum"],
        "lap_time_s": summary["lap_time"],
        "accepted_steps": summary["accepted_steps"],
        "power_source": "replayed per-wheel motor torque and wheel speed, with drivetrain and motor efficiency",
        "validation_warning": "Driven non-optimal transient; not usable as validated lap until all acceptance gates pass.",
    }
    output.with_name("simulated_lap_7dof_summary.json").write_text(
        json.dumps(export_summary, indent=2) + "\n", encoding="utf-8"
    )
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("attempt", type=Path)
    args = parser.parse_args()
    print(export(args.attempt))
