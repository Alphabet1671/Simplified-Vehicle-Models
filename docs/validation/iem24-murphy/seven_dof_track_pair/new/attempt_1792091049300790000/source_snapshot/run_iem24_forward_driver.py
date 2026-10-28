"""Run an isolated forward-driver attempt under a parent wall deadline."""
from __future__ import annotations

import argparse
import json
import math
from numbers import Real
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ElementTree

#: Canonical ten-lap MIS endurance track: the active default for new runs.
DEFAULT_TRACK = Path("models/tracks/mis_ev_endurance_2024_multilap_linear.xml")
#: Track of every attempt that predates --track (no settings["track"] recorded).
#: Historical warm-start matching must keep resolving to this, not DEFAULT_TRACK.
LEGACY_TRACK = Path("models/tracks/mis_ev_endurance_2024_linear.xml")
FORMULATION = Path("models/formulations/iem24_murphy_real_planar_7dof.xml")
VEHICLE = Path("models/vehicles/iem24_murphy_real_planar.xml")


def canonical_path(path):
    """Cwd-relative POSIX-style path when inside the cwd, else absolute."""
    resolved = Path(path).expanduser().resolve()
    try:
        return resolved.relative_to(Path.cwd().resolve())
    except ValueError:
        return resolved


def track_files(track_xml):
    """Return (track XML, referenced CSV) paths; raise ValueError if unusable.

    Only a signed ``closed-xy-profile`` track is accepted; a legacy
    distance-radius profile has no turn direction and cannot drive the 7DoF.
    """
    track_xml = Path(track_xml)
    try:
        root = ElementTree.parse(track_xml).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise ValueError(f"cannot read track XML {track_xml}: {error}") from error
    if root.get("type") != "track-model" or root.get("subtype") != "closed-xy-profile":
        raise ValueError(f"{track_xml}: track must be a signed closed-xy-profile track-model")
    data = root.find("./data")
    if data is None or not data.get("ref"):
        raise ValueError(f"{track_xml}: missing track data reference")
    csv_path = (track_xml.parent / data.get("ref")).resolve()
    if not csv_path.is_file():
        raise ValueError(f"{track_xml}: referenced track CSV {csv_path} not found")
    return track_xml, canonical_path(csv_path)


def tire_xml_path(vehicle_xml=VEHICLE):
    """Tire component XML referenced by the vehicle model."""
    root = ElementTree.parse(vehicle_xml).getroot()
    for component in root.findall("./components/component"):
        if component.get("role") == "tire":
            return canonical_path(Path(vehicle_xml).parent / component.get("ref"))
    raise ValueError(f"{vehicle_xml}: no tire component")


def warm_start_matches(previous, source_hashes, args):
    """True when a previous attempt may seed this run (same plan, track and sources).

    Source hashes include the track XML/CSV, tire XML and formulation, so a
    historical attempt without them cannot warm-start a track-aware run.
    """
    settings = previous.get("settings", {})
    return bool(previous.get("status") == "Completed" and previous.get("lap_completed")
                and previous.get("source_hashes") == source_hashes
                and all(settings.get(key) == getattr(args, key)
                        for key in ("planner", "maximum_speed", "lateral_acceleration",
                                    "pedal_travel_time", "spacing"))
                and canonical_path(settings.get("track") or LEGACY_TRACK)
                == canonical_path(args.track))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path,
        default=Path("docs/validation/iem24-murphy/forward_driver_trials"))
    parser.add_argument("--timeout-seconds", type=float, default=300.)
    parser.add_argument("--duration", type=float, default=300.)
    parser.add_argument("--maximum-speed", type=float, default=8.)
    parser.add_argument("--lateral-acceleration", type=float, default=None,
                        help="Cornering cap (m/s^2). Conservative planner default 2; "
                             "live planner default: no cap beyond the vehicle's own limits")
    parser.add_argument("--pedal-travel-time", type=float, default=.175,
                        help="Seconds for a full pedal travel (0-100%% throttle or 0-max brake)")
    parser.add_argument("--maximum-step", type=float, default=.02)
    parser.add_argument("--rtol", type=float, default=1.e-6)
    parser.add_argument("--method", choices=("Radau", "BDF"), default="Radau")
    parser.add_argument("--spacing", type=float, default=.5)
    parser.add_argument("--planner", choices=("conservative", "live"), default="conservative",
                        help="Speed planner: conservative (constant limits) or live (point-mass)")
    parser.add_argument("--track", type=Path, default=DEFAULT_TRACK,
                        help="Signed closed-xy-profile track XML (default: the canonical "
                             f"ten-lap MIS track {DEFAULT_TRACK})")
    parser.add_argument("--warm-start", type=Path,
                        help="Completed previous attempt whose final physical/pedal state starts this lap")
    parser.add_argument("--child-attempt", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    import math
    for name in ("timeout_seconds", "duration", "maximum_speed", "lateral_acceleration",
                 "maximum_step", "rtol", "spacing", "pedal_travel_time"):
        value = getattr(args, name)
        if value is None:
            continue
        if not math.isfinite(value) or value <= 0:
            parser.error(f"{name} must be finite and positive")
    try:
        track_files(args.track)
    except ValueError as error:
        parser.error(f"--track: {error}")
    args.track = canonical_path(args.track)
    if args.warm_start is not None and args.pedal_travel_time is None:
        parser.error("warm start requires pedal dynamics")
    if args.timeout_seconds > 300 or args.spacing > .5:
        parser.error("timeout must be <=300 s and path spacing must be <=0.5 m")
    return args


def acceptance_gate(summary):
    """Return true only when every required lap-validity gate is established."""
    delta = summary.get("periodic_state_delta")
    tolerances = summary.get("periodic_state_tolerances")
    def finite_real_vector(values):
        return (isinstance(values, (list, tuple)) and bool(values)
                and all(isinstance(value, Real) and math.isfinite(value)
                        for value in values))

    periodic = (summary.get("lap_completed") is True
                and finite_real_vector(delta)
                and finite_real_vector(tolerances)
                and len(delta) == len(tolerances)
                and all(tolerance >= 0 and abs(value) <= tolerance
                        for value, tolerance in zip(delta, tolerances)))
    return bool(periodic
                and summary.get("periodicity_valid") is True
                and summary.get("integration_completed") is True
                and summary.get("path_tracking_valid") is True
                and summary.get("resolution_valid") is True
                and summary.get("checked_rhs_physics_valid") is True
                and summary.get("between_step_validation_complete") is True
                and summary.get("refinement_validated") is True
                and summary.get("contact_valid") is True)


def child(args):
    started = time.perf_counter()
    root = args.child_attempt
    summary = {"status": "Starting", "lap_completed": False, "usable": False,
               "method": "transient_forward_driver", "minimum_time_optimum": False,
               "planner": f"{args.planner}_{'live' if args.planner == 'live' else 'conservative'}_acceleration_envelope",
               "settings": {k: str(v) if isinstance(v, Path) else v
                            for k, v in vars(args).items()}}
    plant = None
    try:
        import numpy as np
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.tracks.profile import load_track
        from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
        from simplified_vehicle_models.solvers.forward_driver import (
            build_speed_plan, PreviewDriver, solve_forward,
        )
        from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant

        import hashlib
        import platform
        import scipy
        sources = [Path(__file__),
                   Path("src/simplified_vehicle_models/solvers/forward_driver.py"),
                   Path("src/simplified_vehicle_models/solvers/planar_forward.py"),
                   Path("src/simplified_vehicle_models/formulations/planar_7dof.py"),
                   Path("src/simplified_vehicle_models/tires/unitire_simple.py"),
                   VEHICLE]
        # Track, referenced CSV, tire XML and formulation define the run's
        # physical inputs; they are hashed and snapshotted with the code.
        track_xml, track_csv = track_files(args.track)
        model_inputs = {"track_xml": track_xml, "track_csv": track_csv,
                        "tire_xml": tire_xml_path(), "formulation_xml": FORMULATION,
                        "vehicle_xml": VEHICLE}
        sources += [path for path in model_inputs.values() if path not in sources]
        names = [path.name for path in sources]
        if len(set(names)) != len(names):
            raise ValueError("source snapshot file names must be unique")
        summary["source_hashes"] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                                    for path in sources}
        summary["model_inputs"] = {key: str(path) for key, path in model_inputs.items()}
        snapshot = root / "source_snapshot"
        snapshot.mkdir()
        for path in sources:
            (snapshot / path.name).write_bytes(path.read_bytes())
        summary["environment"] = {"python": sys.version, "numpy": np.__version__,
                                  "scipy": scipy.__version__, "platform": platform.platform()}
        vehicle = compile_formulation(str(FORMULATION)).model
        track = load_track(args.track)
        count = int(np.ceil(track.lap_length / args.spacing))
        distance = np.linspace(0., track.lap_length, count + 1)
        _, _, _, curvature = evaluate_path(track.path_coordinates, distance)
        live_limit = args.planner == "live"
        lateral_cap = args.lateral_acceleration
        if lateral_cap is None and not live_limit:
            lateral_cap = 2.
        plan = build_speed_plan(distance, curvature, args.maximum_speed,
                                lateral_cap, 1., 2.,
                                vehicle=vehicle if live_limit else None,
                                live_limit=live_limit)
        class LoggedPlant(PlanarForwardPlant):
            next_log = 0.

            def evaluate(self, time_value, state, command):
                value = super().evaluate(time_value, state, command)
                elapsed = time.perf_counter() - started
                if elapsed >= self.next_log:
                    print(json.dumps({"wall_seconds": elapsed, "rhs_time": time_value,
                                      "rhs_distance": float(state[0]),
                                      "rhs_lateral_error": float(state[1]),
                                      "evaluations": self.evaluations}), flush=True)
                    self.next_log = elapsed + 5.
                return value

        plant = LoggedPlant(vehicle, plan)
        driver = PreviewDriver(plan, vehicle.wheelbase, vehicle.steering_maximum_angle,
                               vehicle.steering_maximum_rate, vehicle.maximum_pedal_force,
                               live_limit=live_limit,
                               pedal_travel_time=args.pedal_travel_time)
        initial = plant.initial_state(float(plan.speed[0]))
        initial_pedal = None
        if args.warm_start is not None:
            previous = json.loads((args.warm_start / "summary.json").read_text())
            previous_settings = previous.get("settings", {})
            if not warm_start_matches(previous, summary["source_hashes"], args):
                raise ValueError("warm start requires a completed lap with matching planner, speed cap, track and source hashes")
            trace = np.loadtxt(args.warm_start / "trace.csv", delimiter=",", skiprows=1)
            pedals = np.loadtxt(args.warm_start / "pedal.csv", delimiter=",", skiprows=1)
            if (trace.ndim != 2 or trace.shape[1] != 13 or pedals.shape != (len(trace), 2)
                    or not np.array_equal(trace[:, 0], pedals[:, 0])):
                raise ValueError("warm-start trace and pedal samples must match")
            initial = np.array(trace[-1, 1:], copy=True)
            initial[0] = 0.
            initial_pedal = float(pedals[-1, 1])
            if not np.all(np.isfinite(initial)):
                raise ValueError("warm-start final state must be finite")
            summary["warm_start_source"] = str(args.warm_start)
        summary.update(wheel_inertia=vehicle.wheel_inertia, state_size=plant.state_size,
                       track_length=track.lap_length, setup_seconds=time.perf_counter() - started)
        print(json.dumps(summary), flush=True)
        remaining = args.timeout_seconds - (time.perf_counter() - started) - 5.
        if remaining <= 0:
            raise TimeoutError("setup exhausted the wall budget")
        result = solve_forward(plant, driver, initial, duration=args.duration,
                               wall_seconds=remaining, maximum_step=args.maximum_step,
                               rtol=args.rtol, atol=args.rtol * .01, method=args.method,
                                initial_pedal=initial_pedal)
        np.savetxt(root / "trace.csv", np.column_stack((result.time, result.state.T)),
                   delimiter=",", comments="",
                   header="time,s,n,heading_error,v_x,v_y,yaw_rate,omega_fl,omega_fr,omega_rl,omega_rr,soc,steering")
        if result.pedal is not None:
            np.savetxt(root / "pedal.csv", np.column_stack((result.time, result.pedal)),
                       delimiter=",", comments="", header="time,pedal")
        maximum_spacing = float(np.max(np.diff(result.state[0])))
        path_error = float(np.max(np.abs(result.state[1])))
        closure = result.state[list(plant.periodic_indices), -1] - initial[list(plant.periodic_indices)]
        tolerances = np.array([.001, .001, .01, .01, .001, .01, .01, .01, .01, .001])
        periodic = bool(result.lap_completed and np.all(np.abs(closure) <= tolerances))
        summary.update(status="Completed" if result.lap_completed else "Duration_Limit",
                       lap_completed=result.lap_completed,
                       simulated_seconds=float(result.time[-1]),
                       distance_completed=float(result.state[0, -1]),
                       lap_time=float(result.time[-1]) if result.lap_completed else None,
                       accepted_steps=result.time.size - 1, evaluations=result.evaluations,
                       rejected_domain_trials=result.rejected_domain_trials,
                       maximum_lateral_error=path_error, path_tracking_valid=path_error <= .25,
                       maximum_accepted_spatial_step=maximum_spacing,
                       resolution_valid=maximum_spacing <= .5,
                       periodicity_valid=periodic, periodic_state_delta=closure.tolist(),
                       periodic_state_tolerances=tolerances.tolist(),
                       integration_completed=True, checked_rhs_physics_valid=True,
                       midpoint_sample_validation_complete=result.midpoint_sample_validation_complete,
                        between_step_validation_complete=False, refinement_validated=False,
                       steering_peak=float(np.max(np.abs(result.state[11]))),
                       minimum_wheel_speed=float(np.min(result.state[6:10])),
                       final_soc=float(result.state[10, -1]),
                       planned_lap_time=float(np.sum(np.diff(plan.distance)
                                                     / (.5 * (plan.speed[1:] + plan.speed[:-1])))),
                       planned_peak_lateral_acceleration=float(np.max(
                           plan.speed**2 * np.abs(plan.curvature))))
        # A completed transient is not usable unless every independent acceptance
        # gate (including periodicity and between-step/refinement checks) passes.
        # Every accepted state has been checked by solve_forward. Zero contact
        # load is permitted (wheel lift); negative loads are rejected by plant.
        summary["minimum_contact_load_n"] = plant.minimum_contact_load
        summary["contact_valid"] = bool(result.lap_completed
            and math.isfinite(plant.minimum_contact_load)
            and plant.minimum_contact_load >= 0.)
        summary["usable"] = acceptance_gate(summary)
    except Exception as error:
        import traceback
        (root / "error.txt").write_text(traceback.format_exc())
        summary.update(status="Timed_Out" if isinstance(error, TimeoutError) else "Rejected",
                       error=f"{type(error).__name__}: {error}")
    finally:
        if plant is not None:
            summary.update(last_rhs_time=plant.last_time, last_rhs_distance=plant.last_distance,
                           plant_evaluations=plant.evaluations,
                           maximum_load_residual=plant.maximum_load_residual,
                           maximum_power_residual=plant.maximum_power_residual,
                           maximum_load_iterations=plant.maximum_load_iterations,
                           lifted_wheel_evaluations=plant.lifted_wheel_evaluations,
                           speed_regulated_evaluations=plant.speed_regulated_evaluations,
                           maximum_motor_speed_ratio=plant.maximum_motor_speed_ratio)
        summary["child_wall_seconds"] = time.perf_counter() - started
        (root / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
        print(json.dumps(summary, allow_nan=False), flush=True)
    return 0 if summary["status"] in ("Completed", "Duration_Limit") else 2


def main(argv=None):
    args = parse_args(argv)
    if args.child_attempt is not None:
        return child(args)
    attempt = args.output_root / f"attempt_{time.time_ns()}"
    attempt.mkdir(parents=True, exist_ok=False)
    command = [sys.executable, "-u", str(Path(__file__).resolve()),
               *(sys.argv[1:] if argv is None else argv), "--child-attempt", str(attempt)]
    started = time.perf_counter()
    with (attempt / "run.log").open("w") as log:
        try:
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                    timeout=args.timeout_seconds)
            status = "Child_Completed" if result.returncode == 0 else "Child_Rejected"
            return_code = result.returncode
        except subprocess.TimeoutExpired:
            status, return_code = "Timed_Out", 124
    runtime = time.perf_counter() - started
    parent = {"status": status, "return_code": return_code,
              "total_wall_seconds": runtime, "within_wall_limit": runtime <= args.timeout_seconds,
              "timeout_seconds": args.timeout_seconds, "command": command,
              "attempt": str(attempt)}
    (attempt / "parent.json").write_text(json.dumps(parent, indent=2) + "\n")
    print(json.dumps(parent), flush=True)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
