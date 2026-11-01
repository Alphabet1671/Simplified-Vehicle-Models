"""Matched fixed-plan linear driver-input study; prepare artifacts, then run one arm per process.

No preparation vehicle solve occurs. Candidate settings are engineering hypotheses,
not calibrated or recommended defaults. This straight does not exercise braking or corners.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

from tools.run import compare_continuous_driver_throttle as fixed

NAMES = ("baseline", "demand_only", "slew_only", "combined")
RATES = {"throttle_apply_rate": 4.0, "throttle_release_rate": 6.0,
         "brake_apply_rate": 5.0, "brake_release_rate": 6.0}


def _map_config(candidate):
    from simplified_vehicle_models.solvers.driver_maps import parse_pedal_map_config
    acc = ([0., 6.], [0., 1.]) if candidate in ("baseline", "slew_only") else ([0., 1., 2.5], [0., .5, 1.])
    payload = {"schema_version": 1, "demand_map": {
        "units": {"acceleration_axis": "m/s^2", "deceleration_axis": "m/s^2", "pedal_output": "1"},
        "acceleration_to_throttle": {"axis": acc[0], "value": acc[1]},
        "deceleration_to_brake": {"axis": [0., 6.], "value": [0., 1.]}},
        "physical_calibration": None}
    return payload, parse_pedal_map_config(payload)


def _source_manifest(compiled_manifest):
    manifest = fixed.collect_source_manifest(compiled_manifest)
    paths = dict(manifest["hashes"])
    own = Path(__file__).resolve()
    paths[str(own)] = hashlib.sha256(own.read_bytes()).hexdigest()
    manifest["hashes"] = paths
    return manifest


def _exclusive(path, data):
    payload = (json.dumps(fixed._sanitize_json(data), sort_keys=True, indent=2, allow_nan=False) + "\n").encode()
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(fd, "wb") as f:
        f.write(payload); f.flush(); os.fsync(f.fileno())


def _arrays(plan):
    return {name: getattr(plan, name) for name in
            ("distance", "speed", "curvature", "acceleration", "precursor_distance",
             "precursor_speed", "precursor_curvature", "precursor_acceleration")}


def prepare_study(output):
    started = time.perf_counter()
    compiled = fixed.compile_selected_model()
    sources = _source_manifest(compiled.manifest)
    plan = fixed.make_plan(compiled.model)
    directory = fixed.create_output_directory(output)
    arrays = _arrays(plan)
    for name, array in arrays.items():
        if array is None: raise ValueError(f"missing complete plan source array {name}")
        np.save(directory / f"plan_{name}.npy", array, allow_pickle=False)
    base = fixed._canonical_config(compiled.manifest.as_text())
    base["plan_settings"].update({"precursor_length_m": 250., "course_length_m": 150.,
        "initial_speed_mps": 6., "initial_pedal": 0., "curvature": 0.,
        "crop": "first interpolated 6 m/s point of open live-limit standing-start profile"})
    candidates = {}
    for name in NAMES:
        payload, config = _map_config(name)
        candidates[name] = {"pedal_map": payload, "pedal_map_digest": config.sha256,
            "physical_calibration": None,
            "slew_rates": RATES if name in ("slew_only", "combined") else None,
            "pedal_travel_time_s": .175, "pedal_time_constant_s": .02,
            "initial_speed_mps": 6., "initial_pedal": 0., "mapper": None}
    full = {"shared": base, "candidates": candidates,
            "candidate_note": "explicit engineering experiment values; not safe/default recommendation"}
    data = {**full, "config_digest": fixed._config_digest(full), "sources": sources,
        "plan_hash": plan.digest,
        "plan_arrays_hashes": {n: fixed._array_digest(getattr(plan, n)) for n in
                                ("distance", "speed", "curvature", "acceleration")},
        "crop_origin_m": plan.crop_origin_m,
        "tire_law": "steady combined-slip UniTire; no transient tire relaxation",
        "geometry_claim": "straight centerline plan only; no braking/corner steering or body-containment claim",
        "setup_seconds": time.perf_counter() - started}
    data["artifact_digest"] = fixed._artifact_digest(dict(data), arrays)
    _exclusive(directory / "manifest.json", data)
    return directory


def _validate(run_dir):
    directory = Path(run_dir)
    prepared = json.loads((directory / "manifest.json").read_text())
    if set(prepared["candidates"]) != set(NAMES): raise ValueError("candidate matrix mismatch")
    arrays = {}
    for name in ("distance", "speed", "curvature", "acceleration", "precursor_distance", "precursor_speed", "precursor_curvature", "precursor_acceleration"):
        a = np.load(directory / f"plan_{name}.npy", allow_pickle=False)
        if a.ndim != 1 or not np.all(np.isfinite(a)): raise ValueError(f"invalid frozen array {name}")
        arrays[name] = a
    crop = fixed._reconstruct_crop({"precursor_" + k: arrays["precursor_" + k] for k in
        ("distance", "speed", "curvature", "acceleration")}, prepared["crop_origin_m"])
    plan = fixed.freeze_plan_arrays(*(arrays[n] for n in ("distance", "speed", "curvature", "acceleration")))
    if crop.digest != plan.digest or plan.digest != prepared["plan_hash"]: raise ValueError("frozen crop mismatch")
    if prepared["plan_arrays_hashes"] != {n: fixed._array_digest(arrays[n]) for n in ("distance", "speed", "curvature", "acceleration")}: raise ValueError("plan hashes mismatch")
    body = dict(prepared); digest = body.pop("artifact_digest", None)
    if digest != fixed._artifact_digest(body, arrays): raise ValueError("prepared artifact digest mismatch")
    compiled = fixed.compile_selected_model()
    shared, candidates = prepared["shared"], prepared["candidates"]
    expected_base = fixed._canonical_config(compiled.manifest.as_text())
    expected_base["plan_settings"].update({"precursor_length_m": 250., "course_length_m": 150.,
        "initial_speed_mps": 6., "initial_pedal": 0., "curvature": 0.,
        "crop": "first interpolated 6 m/s point of open live-limit standing-start profile"})
    if shared != expected_base: raise ValueError("shared frozen model/plan/solver configuration mismatch")
    for candidate_name in NAMES:
        payload, parsed = _map_config(candidate_name)
        expected = {"pedal_map": payload, "pedal_map_digest": parsed.sha256,
            "physical_calibration": None,
            "slew_rates": RATES if candidate_name in ("slew_only", "combined") else None,
            "pedal_travel_time_s": .175, "pedal_time_constant_s": .02,
            "initial_speed_mps": 6., "initial_pedal": 0., "mapper": None}
        if candidates[candidate_name] != expected:
            raise ValueError(f"candidate configuration mismatch: {candidate_name}")
    full = {"shared": shared, "candidates": candidates, "candidate_note": prepared["candidate_note"]}
    if prepared["config_digest"] != fixed._config_digest(full): raise ValueError("canonical candidate configuration mismatch")
    expected_sources = _source_manifest(compiled.manifest)
    fixed.require_matching_manifests(prepared["sources"], expected_sources)
    return prepared, plan, compiled


def _metrics(rows, maximum_brake_force):
    required = ("time", "current_speed", "target_speed", "pedal_lagged_applied_throttle",
                "pedal_lagged_applied_brake_force", "pedal", "distance")
    if len(rows) < 2 or any(any(key not in row for key in required) for row in rows):
        raise ValueError("accepted trace lacks required finite metric channels")
    if not np.isfinite(maximum_brake_force) or maximum_brake_force <= 0:
        raise ValueError("maximum brake force must be finite and positive")
    get = lambda key: np.asarray([r[key] for r in rows], float)
    t = get("time"); dt = np.diff(t); duration = float(dt.sum())
    distance = get("distance"); ds = np.diff(distance); station = float(ds.sum())
    if (not np.all(np.isfinite(t)) or not np.all(np.isfinite(distance)) or
        not all(np.all(np.isfinite(get(key))) for key in required[1:-1]) or
        np.any(dt <= 0) or np.any(ds < 0) or duration <= 0 or station <= 0):
        raise ValueError("metric channels require finite samples, increasing time, and nondecreasing station")
    speed, target = get("current_speed"), get("target_speed")
    throttle, brake, pedal = get("pedal_lagged_applied_throttle"), get("pedal_lagged_applied_brake_force"), get("pedal")
    deficit = target - speed
    variations = np.abs(np.diff(pedal))
    return {"weighting": "left interval; terminal sample has zero duration", "duration_s": duration,
        "station_m": station,
        "speed_error_mae_mps": float(np.sum(dt * np.abs(deficit[:-1])) / duration),
        "speed_error_rmse_mps": float(np.sqrt(np.sum(dt * deficit[:-1] ** 2) / duration)),
        "speed_error_mae_per_station_m": float(np.sum(ds * np.abs(deficit[:-1])) / station),
        "speed_error_rmse_per_station_m": float(np.sqrt(np.sum(ds * deficit[:-1] ** 2) / station)),
        "positive_deficit_fraction": float(np.sum(dt * (deficit[:-1] > 0)) / duration),
        "positive_deficit_per_station_fraction": float(np.sum(ds * (deficit[:-1] > 0)) / station),
        "mean_positive_deficit_mps": float(np.sum(dt * np.maximum(deficit[:-1], 0.)) / duration),
        "maximum_overspeed_mps": float(np.max(np.maximum(-deficit, 0.))),
        "overspeed_over_0_5mps_fraction": float(np.sum(dt * (deficit[:-1] < -.5)) / duration),
        "partial_throttle_below_target_fraction": float(np.sum(dt * ((deficit[:-1] > 1.) & (throttle[:-1] < .9) & (brake[:-1] < .01 * maximum_brake_force))) / duration),
        "full_throttle_ge_0_95_fraction": float(np.sum(dt * (throttle[:-1] >= .95)) / duration),
        "pedal_total_variation": float(variations.sum()), "variation_per_simulated_second": float(variations.sum()/duration),
        "variation_per_station_m": float(variations.sum()/station),
        "station_weighted_variation_per_station_m": float(np.sum(ds * np.abs(np.diff(pedal)) / np.maximum(dt, 1e-12)) / station),
        "threshold_transition_definition": "signed pedal transitions at +/-0.05; 0.10 s dwell; 0.05 hysteresis",
        "thresholded_transitions": _transitions(t, pedal),
        "peak_applied_pedal_rate_per_s": float(np.max(variations / dt)),
        "slew_settings_are_not_proof_of_demand_smoothness": True}


def _transitions(t, pedal):
    state = 1 if pedal[0] > .05 else -1 if pedal[0] < -.05 else 0
    pending = None; since = None; events = []
    for ti, value in zip(t[1:], pedal[1:]):
        candidate = 1 if value > .05 else -1 if value < -.05 else 0
        if candidate == state: pending = since = None; continue
        if pending != candidate: pending, since = candidate, ti
        elif ti - since >= .1:
            events.append({"time": float(since), "from": state, "to": candidate}); state = candidate; pending = since = None
    return events


def run_arm(run_dir, name):
    if name not in NAMES: raise ValueError(f"arm must be one of {NAMES}")
    d = Path(run_dir)
    for suffix in ("claim.json", "started.json", "json"):
        if (d / f"{name}.{suffix}").exists(): raise FileExistsError(f"arm evidence already exists: {name}")
    _exclusive(d / f"{name}.claim.json", {"arm": name, "status": "claimed", "time": time.time()})
    rows = []; result_path = d / f"{name}.json"; summary = None
    try:
        prepared, frozen, compiled = _validate(d)
        candidate = prepared["candidates"][name]
        from simplified_vehicle_models.solvers.forward_driver import PreviewDriver
        from simplified_vehicle_models.solvers.driver_maps import parse_pedal_map_config
        from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
        vehicle = compiled.model; plan = fixed._speed_plan(frozen); control = getattr(vehicle, "planar", vehicle)
        plant = PlanarForwardPlant(vehicle, plan)
        driver = PreviewDriver(plan, control.wheelbase, control.steering_maximum_angle,
            control.steering_maximum_rate, control.maximum_pedal_force, **prepared["shared"]["driver"],
            pedal_travel_time=.175, pedal_time_constant=.02, live_limit=True,
            demand_map=parse_pedal_map_config(candidate["pedal_map"]).demand_map,
            **(candidate["slew_rates"] or {}))
        initial = plant.initial_state(6.)
        sink = fixed.TraceSink(plant, mapping_configured=False)
        rows = sink.rows
        _exclusive(d / f"{name}.started.json", {"arm": name, "status": "starting", "plan_hash": frozen.digest})
        diagnostics = {}
        def diagnostics_sink(snapshot): diagnostics.update(snapshot)
        solve_parameters = dict(duration=prepared["shared"]["solver"]["duration_s"],
            wall_seconds=prepared["shared"]["solver"]["wall_seconds"], maximum_wall_seconds=300.,
            maximum_step=prepared["shared"]["solver"]["maximum_step"], rtol=prepared["shared"]["solver"]["rtol"],
            atol=prepared["shared"]["solver"]["atol"], method=prepared["shared"]["solver"]["method"],
            initial_pedal=0., step_sink=sink, command_mapper=None, diagnostics_sink=diagnostics_sink)
        result = fixed.solve_forward(plant, driver, initial, **solve_parameters)
        if len(sink.rows) != len(result.time): raise RuntimeError("accepted telemetry/sample count mismatch")
        for row in sink.rows:
            row["configured_slew_rates"] = candidate["slew_rates"]
            row["requested_pedal"] = row["requested_throttle"] - row["requested_brake_force"] / driver.maximum_brake_force
            row["mapped_pedal"] = row["mapped_throttle"] - row["mapped_brake_force"] / driver.maximum_brake_force
            row["applied_pedal"] = row["pedal_lagged_applied_throttle"] - row["pedal_lagged_applied_brake_force"] / driver.maximum_brake_force
            row["signed_pedal_units"] = "normalized signed command (+throttle, -brake); brake force divided by maximum_brake_force"
            row["maximum_brake_force_n"] = float(driver.maximum_brake_force)
            row["a_req_mps2"] = row["feedforward_acceleration"] + prepared["shared"]["driver"]["speed_gain"] * (row["target_speed"] - row["current_speed"])
            row["speed_gain_mps2_per_mps"] = prepared["shared"]["driver"]["speed_gain"]
            row["mapped_stage"] = "identity_no_external_mapper"
        summary = {"arm": name, "status": "completed", "plan_hash": frozen.digest,
            "manifest": prepared["sources"], "candidate": candidate, "initial_state": initial.tolist(),
            "accepted_samples": len(sink.rows), "telemetry": sink.rows,
            "diagnostics": diagnostics, "wall_seconds": float(result.wall_seconds), "metrics": _metrics(sink.rows, driver.maximum_brake_force),
            "mapper": "None; identity stage", "counter_scope": "solver diagnostics from accepted integration; no external mapper counters"}
    except Exception as exc:
        summary = {"arm": name, "status": "failed", "error": f"{type(exc).__name__}: {exc}",
            "accepted_samples": len(rows), "telemetry": rows,
            "prefix_kind": "accepted step-sink prefix if solve started; no fabricated terminal sample"}
        raise
    finally:
        if summary is not None: _exclusive(result_path, summary)
    if summary["status"] == "failed": raise RuntimeError(summary["error"])
    return summary


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__); sub = p.add_subparsers(dest="command", required=True)
    prepare = sub.add_parser("prepare"); prepare.add_argument("--output", type=Path, required=True)
    arm = sub.add_parser("arm"); arm.add_argument("run_dir", type=Path); arm.add_argument("name", choices=NAMES)
    a = p.parse_args(argv)
    if a.command == "prepare": print(prepare_study(a.output)); return 0
    result = run_arm(a.run_dir, a.name); print(json.dumps({"arm": a.name, "status": result["status"], "accepted_samples": result["accepted_samples"]})); return 0 if result["status"] == "completed" else 1

if __name__ == "__main__":
    raise SystemExit(main())
