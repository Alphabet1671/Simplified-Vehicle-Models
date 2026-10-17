#!/usr/bin/env python3
"""Export the six MIS 2024 comparison channels from *any* forward-driver attempt.

``tools/reports/export_iem24_forward_correlation.py`` is planar-7DoF only: it
hard-codes the planar formulation, the planar plant and the planar 12-state
layout. This module is additive and model-agnostic. Both forward plants of
``tools/run/run_iem24_forward_driver.py`` publish their state layout, share the
same driver, plan builder and powertrain closure, and expose the same
``body_accelerations`` rotation, so one replay path serves the planar 7DoF rung
and the vertical 14DoF rung:

* ``distance_m`` is the integrated path coordinate ``s``;
* ``elapsed_time_s`` is the accepted-state clock (not arc length / speed);
* ``velocity_mps`` is the body-frame velocity magnitude ``hypot(v_x, v_y)``;
* ``longitudinal_acceleration_mps2`` and ``lateral_acceleration_mps2`` are the
  **road-plane body-frame** accelerations including the rotating-frame terms
  (``v_x_dot - yaw_rate*v_y`` and ``v_y_dot + yaw_rate*v_x``). The road is flat,
  so the road-plane values are the body-axis values;
* ``yaw_rate_rad_s`` is the traced yaw-rate state;
* ``motor_torque_request_nm`` is ``command.throttle`` times the per-motor torque
  that full pedal requests -- the same definition for both forward models;
* ``pack_voltage_v`` is the plant's own terminal-voltage closure
  (``ForwardPowertrain.voltage``) at the accepted state and command -- the same
  method for both forward models;
* ``rl_power_kw``/``rr_power_kw`` are the replayed per-wheel shaft powers
  referred to the bus through the gear and motor efficiency, exactly as the
  established planar exporter computes them.

The established planar exporter's own output is not touched by this module; see
``tests/test_iem24_four_way.py``, which compares this module's planar export
against it on one synthetic attempt.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _path in (str(ROOT), str(ROOT / "src")):
    if _path not in sys.path:
        sys.path.insert(0, _path)

#: Forward models this module can replay, keyed by the attempt's ``model`` field.
SPECS: dict[str, dict[str, str]] = {
    "planar-7dof": {
        "formulation": "models/formulations/iem24_murphy_real_planar_7dof.xml",
        "plant_module": "simplified_vehicle_models.solvers.planar_forward",
        "plant_class": "PlanarForwardPlant",
        "accelerations_function": "body_accelerations",
    },
    "14dof-simple": {
        "formulation": "models/formulations/iem24_murphy_real_14dof_simple.xml",
        "plant_module": "simplified_vehicle_models.solvers.vertical_forward",
        "plant_class": "VerticalForwardPlant",
        "accelerations_function": "body_accelerations",
    },
}

#: Columns written for every model, in this order. The first eight plus
#: ``apps_fraction`` are the established planar exporter's columns plus the two
#: channels it does not carry (yaw rate and terminal voltage); ``soc`` is the
#: traced state of charge, which is an initial-condition channel of the report.
CHANNELS: tuple[str, ...] = (
    "distance_m",
    "elapsed_time_s",
    "velocity_mps",
    "longitudinal_acceleration_mps2",
    "lateral_acceleration_mps2",
    "yaw_rate_rad_s",
    "motor_torque_request_nm",
    "pack_voltage_v",
    "rl_power_kw",
    "rr_power_kw",
    "apps_fraction",
    "soc",
)

#: Wheel-speed state-name patterns, most specific first (planar then vertical).
WHEEL_SPEED_PATTERNS = ("omega_{}", "wheel_speed_{}")
WHEELS = ("fl", "fr", "rl", "rr")

#: Model-input roles every attempt records.
MODEL_ROLES = ("track_xml", "track_csv", "tire_xml", "formulation_xml", "vehicle_xml")


class ExportBlocker(RuntimeError):
    """The attempt cannot support an honest channel export."""


def sha256_file(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def plant_class(model: str):
    """Return the plant class of one forward model, imported on first use."""
    spec = SPECS[model]
    return getattr(importlib.import_module(spec["plant_module"]), spec["plant_class"])


def accelerations_function(model: str):
    """Return the plant module's body-acceleration rotation for one model."""
    spec = SPECS[model]
    return getattr(importlib.import_module(spec["plant_module"]),
                   spec["accelerations_function"])


def wheel_speed_columns(state_names) -> tuple[int, ...]:
    """Return the four wheel-speed state indices in FL, FR, RL, RR order."""
    names = tuple(state_names)
    for pattern in WHEEL_SPEED_PATTERNS:
        candidate = tuple(pattern.format(wheel) for wheel in WHEELS)
        if all(name in names for name in candidate):
            return tuple(names.index(name) for name in candidate)
    raise ExportBlocker("plant state names do not expose four wheel speeds")


def drive_torque_limit_nm(vehicle) -> float:
    """Per-motor torque that full pedal (``throttle == 1``) requests."""
    return float(min(vehicle.motor.peak_motor_torque,
                     vehicle.controller.maximum_drive_motor_torque_nm))


def verify_attempt(attempt, *, root: Path = ROOT) -> dict:
    """Prove an attempt is a completed forward lap whose inputs match the files.

    Checks the recorded status, method and optimum flag, every recorded
    model-input hash against the file on disk *and* against the attempt's own
    ``source_snapshot`` copy, the formulation recorded for the attempt's model,
    that ``settings['track']`` names the recorded track, and that ``trace.csv``
    has the plant's published header.
    """
    attempt = Path(attempt)
    try:
        summary = json.loads((attempt / "summary.json").read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise ExportBlocker(f"attempt {attempt}: cannot read summary.json ({error})") from error
    model = summary.get("model")
    if model not in SPECS:
        raise ExportBlocker(f"attempt {attempt}: model {model!r} is not one of {tuple(SPECS)}")
    if summary.get("method") != "transient_forward_driver":
        raise ExportBlocker(f"attempt {attempt}: not a transient_forward_driver attempt")
    if summary.get("status") != "Completed" or summary.get("lap_completed") is not True:
        raise ExportBlocker(f"attempt {attempt}: only a completed full lap may be exported "
                            f"(status={summary.get('status')!r}, "
                            f"lap_completed={summary.get('lap_completed')!r})")
    if summary.get("minimum_time_optimum") is not False:
        raise ExportBlocker(f"attempt {attempt}: a forward-driver lap must record "
                            "minimum_time_optimum=false")
    inputs = summary.get("model_inputs")
    if not isinstance(inputs, dict):
        raise ExportBlocker(f"attempt {attempt}: records no model_inputs hashes")
    if str(inputs.get("formulation_xml")) != SPECS[model]["formulation"]:
        raise ExportBlocker(f"attempt {attempt}: recorded formulation "
                            f"{inputs.get('formulation_xml')!r} is not "
                            f"{SPECS[model]['formulation']!r}")
    hashes = summary.get("source_hashes", {})
    recorded = {}
    for role in MODEL_ROLES:
        path = inputs.get(role)
        if path is None or path not in hashes:
            raise ExportBlocker(f"attempt {attempt}: records no hash for {role}")
        actual = sha256_file(root / path)
        if actual != hashes[path]:
            raise ExportBlocker(f"attempt {attempt}: {role} {path} differs from the file "
                                "the attempt used")
        recorded[role] = {"path": str(path), "sha256": hashes[path]}
    track = str(summary.get("settings", {}).get("track") or "")
    if track != str(inputs["track_xml"]):
        raise ExportBlocker(f"attempt {attempt}: settings.track {track!r} != "
                            f"model_inputs.track_xml {inputs['track_xml']!r}")
    snapshot = attempt / "source_snapshot"
    if not snapshot.is_dir():
        raise ExportBlocker(f"attempt {attempt}: no source_snapshot directory")
    for path, digest in hashes.items():
        copy = snapshot / Path(path).name
        if not copy.is_file() or sha256_file(copy) != digest:
            raise ExportBlocker(f"attempt {attempt}: source snapshot of {Path(path).name} "
                                "is missing or differs from the recorded hash")
    header = (attempt / "trace.csv").open(encoding="utf-8").readline().strip().split(",")
    names = tuple(plant_class(model).state_names)
    if tuple(header) != ("time",) + names:
        raise ExportBlocker(f"attempt {attempt}: trace.csv header does not match the "
                            f"{model} plant layout")
    return {"summary": summary, "model": model, "inputs": recorded, "track": track}


def build_plan(vehicle, settings: dict, track):
    """Rebuild the attempt's speed plan from its recorded driver settings."""
    from simplified_vehicle_models.solvers.forward_driver import build_speed_plan
    from simplified_vehicle_models.tracks.path_coordinates import evaluate_path

    count = int(np.ceil(track.lap_length / float(settings["spacing"])))
    distance = np.linspace(0.0, track.lap_length, count + 1)
    _, _, _, curvature = evaluate_path(track.path_coordinates, distance)
    live = settings["planner"] == "live"
    lateral = settings.get("lateral_acceleration")
    if lateral is None and not live:
        lateral = 2.0
    return build_speed_plan(
        distance, curvature, float(settings["maximum_speed"]), lateral, 1.0, 2.0,
        vehicle=vehicle if live else None, live_limit=live,
    )


def replay(attempt, *, root: Path = ROOT) -> tuple[dict[str, np.ndarray], dict]:
    """Replay the accepted states of one attempt and return its channels."""
    from simplified_vehicle_models.compiler.compile import compile_formulation
    from simplified_vehicle_models.solvers.forward_driver import PreviewDriver
    from simplified_vehicle_models.tracks.profile import load_track

    attempt = Path(attempt)
    info = verify_attempt(attempt, root=root)
    model, summary = info["model"], info["summary"]
    settings = summary["settings"]
    formulation = root / SPECS[model]["formulation"]
    vehicle = compile_formulation(str(formulation)).model
    control = getattr(vehicle, "planar", vehicle)
    track = load_track(root / info["track"])
    if not track.has_signed_geometry:
        raise ExportBlocker(f"{info['track']}: track must carry signed geometry")
    recorded_length = summary.get("track_length")
    if recorded_length is None or abs(float(recorded_length) - track.lap_length) > 1.0e-6:
        raise ExportBlocker(f"attempt {attempt}: recorded track length differs from "
                            f"{info['track']}")
    plan = build_plan(control, settings, track)
    live = settings["planner"] == "live"
    klass = plant_class(model)
    plant = klass(vehicle, plan)
    names = tuple(plant.state_names)
    driver = PreviewDriver(plan, control.wheelbase, control.steering_maximum_angle,
                           control.steering_maximum_rate, control.maximum_pedal_force,
                           live_limit=live,
                           pedal_travel_time=settings.get("pedal_travel_time"))
    trace = np.loadtxt(attempt / "trace.csv", delimiter=",", skiprows=1)
    if trace.ndim != 2 or trace.shape[1] != 1 + plant.state_size:
        raise ExportBlocker(f"attempt {attempt}: expected time and {plant.state_size} "
                            "forward-driver states")
    pedal = None
    if driver.has_pedal_dynamics:
        pedal_path = attempt / "pedal.csv"
        if not pedal_path.is_file():
            raise ExportBlocker(f"attempt {attempt}: pedal dynamics require a matching "
                                "pedal.csv")
        pedal = np.loadtxt(pedal_path, delimiter=",", skiprows=1)
        if pedal.shape != (len(trace), 2) or not np.array_equal(pedal[:, 0], trace[:, 0]):
            raise ExportBlocker(f"attempt {attempt}: pedal samples must match accepted "
                                "trace samples")
    states = trace[:, 1:]
    columns = wheel_speed_columns(names)
    accelerations = accelerations_function(model)
    yaw = names.index("yaw_rate")
    count = len(trace)
    rates = np.empty_like(states)
    powers = np.empty((count, 2))
    voltage = np.empty(count)
    throttle = np.empty(count)
    for i, (time, state) in enumerate(zip(trace[:, 0], states)):
        command = driver.command(plant.observe(state))
        if pedal is not None:
            command = driver.realized_command(float(pedal[i, 1]), command)
        rates[i] = plant.evaluate(float(time), state, command).solve()
        operating = plant.powertrain(state, command)
        shaft_power = (operating.motor_wheel_torques[plant.driven]
                       * state[list(columns)][plant.driven])
        powers[i] = np.where(
            shaft_power >= 0.0,
            shaft_power / (plant.gear_efficiency * control.motor.drive_efficiency),
            shaft_power * plant.gear_efficiency * control.motor.drive_efficiency,
        ) / 1000.0
        voltage[i] = operating.voltage
        throttle[i] = command.throttle
    ax, ay = accelerations(states, rates)
    limit = drive_torque_limit_nm(control)
    channels = {
        "distance_m": states[:, names.index("s")],
        "elapsed_time_s": trace[:, 0],
        "velocity_mps": np.hypot(states[:, names.index("v_x")], states[:, names.index("v_y")]),
        "longitudinal_acceleration_mps2": ax,
        "lateral_acceleration_mps2": ay,
        "yaw_rate_rad_s": states[:, yaw],
        "motor_torque_request_nm": throttle * limit,
        "pack_voltage_v": voltage,
        "rl_power_kw": powers[:, 0],
        "rr_power_kw": powers[:, 1],
        "apps_fraction": throttle,
        "soc": states[:, names.index("soc")],
    }
    export_summary = {
        "source_attempt": str(attempt),
        "model": model,
        "track": info["track"],
        "formulation": SPECS[model]["formulation"],
        "method": summary["method"],
        "status": summary["status"],
        "lap_completed": summary["lap_completed"],
        "usable": summary.get("usable"),
        "minimum_time_optimum": summary["minimum_time_optimum"],
        "lap_time_s": float(summary["lap_time"]),
        "accepted_steps": int(summary["accepted_steps"]),
        "state_size": int(plant.state_size),
        "drive_torque_limit_nm": limit,
        "recorded_source_hashes": {key: value["sha256"] for key, value in info["inputs"].items()},
        "channel_definitions": {
            "velocity_mps": "body-frame velocity magnitude hypot(v_x, v_y)",
            "longitudinal_acceleration_mps2": "body-frame v_x_dot - yaw_rate*v_y",
            "lateral_acceleration_mps2": "body-frame v_y_dot + yaw_rate*v_x",
            "yaw_rate_rad_s": "traced yaw-rate state",
            "motor_torque_request_nm": "command.throttle * per-motor full-pedal torque limit",
            "pack_voltage_v": "plant terminal-voltage closure (ForwardPowertrain.voltage)",
            "rl_power_kw": "replayed RL shaft power referred to the bus through gear and motor efficiency",
            "rr_power_kw": "replayed RR shaft power referred to the bus through gear and motor efficiency",
        },
        "validation_warning": ("Driven non-optimal transient; not usable as a validated lap "
                               "unless every acceptance gate passes."),
    }
    return channels, export_summary


def write_channels(channels: dict[str, np.ndarray], summary: dict, output: Path) -> tuple[Path, Path]:
    """Write the channels CSV and its JSON summary into ``output``."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    table = output / f"{summary['model']}_channels.csv"
    np.savetxt(table, np.column_stack([channels[name] for name in CHANNELS]),
               delimiter=",", header=",".join(CHANNELS), comments="")
    (output / f"{summary['model']}_channels_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return table, output / f"{summary['model']}_channels_summary.json"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("attempt", type=Path,
                        help="completed forward-driver attempt directory (trace.csv, pedal.csv, summary.json)")
    parser.add_argument("--output", type=Path, required=True,
                        help="directory for the channels CSV and its JSON summary")
    parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args(argv)
    try:
        channels, summary = replay(args.attempt, root=args.root)
        table, meta = write_channels(channels, summary, args.output)
    except ExportBlocker as blocker:
        print(f"BLOCKER: {blocker}", file=sys.stderr)
        return 2
    print(json.dumps({"channels": str(table), "summary": str(meta),
                      "model": summary["model"], "lap_time_s": summary["lap_time_s"]},
                     indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
