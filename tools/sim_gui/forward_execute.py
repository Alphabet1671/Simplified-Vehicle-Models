"""Opt-in continuity-fixed braking-envelope forward-driver workflow."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.vertical_14dof import VerticalVehicleParameters
from simplified_vehicle_models.powertrain.energy_storage import StorageState
from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
from simplified_vehicle_models.parameters.vehicle import PlanarVehicleParameters
from simplified_vehicle_models.results.plant_channels import RunRecordSink
from simplified_vehicle_models.results.plant_channels import source_hashes, vehicle_geometry
from simplified_vehicle_models.solvers.forward_driver import StepSample
from simplified_vehicle_models.solvers.braking_envelope import build_braking_envelope
from simplified_vehicle_models.solvers.envelope_driver import EnvelopeDriverSettings, EnvelopePolicy
from simplified_vehicle_models.solvers.forward_driver import PreviewDriver, build_speed_plan, solve_forward
from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
from simplified_vehicle_models.tracks.profile import load_track

from .settings import RunSettings, forward_driver_config, validate


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(path: Path, payload: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n")
    temporary.replace(path)


def _policy_row(sample: StepSample) -> dict:
    def command(value):
        if value is None:
            return None
        return {"throttle": float(value.throttle), "brake_force": float(value.brake_force),
                "steering_rate": float(value.steering_rate)}
    diagnostics = sample.policy_diagnostics
    if hasattr(diagnostics, "to_dict"):
        diagnostics = diagnostics.to_dict()
    return {"index": sample.index, "time": sample.time, "pedal": sample.pedal,
        "requested_command": command(sample.requested_command),
        "mapped_command": command(sample.mapped_command), "applied_command": command(sample.command),
        "policy_diagnostics": diagnostics}


def _run_impl(case: dict, attempt: Path, settings: RunSettings, case_dir: Path) -> int:
    """Run the exact selected GUI variant in one parent-supervised attempt.

    ``attempt`` is the fresh child output directory. No nested watchdog is
    started here; the existing GUI child supervisor owns cancellation/deadline.
    """
    started = time.perf_counter()
    attempt = Path(attempt)
    if not attempt.is_dir() or any(attempt.iterdir()):
        raise ValueError("forward-driver requires a fresh parent-created attempt directory")
    progress = attempt / "progress.json"
    _write(progress, {"status": "running", "phase": "identity-validation"})
    formulation = (Path(case["formulation"]) if Path(case["formulation"]).is_absolute()
                  else Path(case_dir) / case["formulation"]).resolve()
    track_path = Path(case["track"]).resolve()
    identity = case.get("input_identity", {})
    if not formulation.is_file() or not track_path.is_file():
        raise ValueError("forward-driver input file is missing")
    expected = {
        "solver_kind": "forward-driver", "source_model": case.get("model"),
        "solution_model": "planar-7dof", "formulation": str(formulation),
        "formulation_sha256": _sha(formulation), "track": str(track_path),
        "track_sha256": _sha(track_path), "settings": settings.to_json(),
        "settings_sha256": hashlib.sha256(json.dumps(settings.to_json(), sort_keys=True,
            separators=(",", ":"), allow_nan=False).encode()).hexdigest(),
        "driver_config": forward_driver_config(settings),
    }
    frozen_sources = identity.get("component_source_hashes")
    if not isinstance(frozen_sources, dict) or not frozen_sources:
        raise ValueError("forward-driver immutable component source closure is missing")
    for source, digest in frozen_sources.items():
        path = Path(source)
        if not path.is_file() or "sha256:" + _sha(path) != digest:
            raise ValueError(f"forward-driver immutable component source mismatch: {source}")
    for key, value in expected.items():
        if identity.get(key) != value:
            raise ValueError(f"forward-driver immutable input identity mismatch: {key}")
    if case.get("argv") != []:
        raise ValueError("forward-driver immutable argv must be empty")
    validate(settings, case["model"], track_path)
    # Hashes are checked above, before compiling case-selected source.
    _write(progress, {"status": "running", "phase": "compile-and-envelope", "started_wall_seconds": time.perf_counter() - started})
    compiled_result = compile_formulation(str(formulation))
    compiled = compiled_result.model
    compiled_sources = {str(Path(item.path).resolve()): "sha256:" + _sha(Path(item.path).resolve())
                        for item in compiled_result.manifest.loaded_components}
    compiled_sources[str(formulation)] = "sha256:" + _sha(formulation)
    compiled_sources[str(Path(compiled_result.manifest.assembly.path).resolve())] = "sha256:" + _sha(Path(compiled_result.manifest.assembly.path).resolve())
    compiled_sources[str(track_path)] = "sha256:" + _sha(track_path)
    if compiled_sources != frozen_sources:
        raise ValueError("compiled source closure differs from immutable prepared identity")
    if isinstance(compiled, VerticalVehicleParameters):
        control = compiled.planar
        reduction = "14dof.planar->planar-7dof"
    elif isinstance(compiled, PlanarVehicleParameters):
        control = compiled
        reduction = "identity"
    else:
        raise ValueError(f"forward-driver requires planar/14DoF source, got {type(compiled).__name__}")
    track = load_track(track_path)
    count = max(2, int(math.ceil(track.lap_length / settings.forward_spacing)))
    distance = np.linspace(0., track.lap_length, count + 1)
    _, _, _, curvature = evaluate_path(track.path_coordinates, distance)
    plan = build_speed_plan(distance, curvature, settings.forward_maximum_speed,
        vehicle=control, live_limit=True, open_course=not track.is_closed, lateral_margin=.9)
    if time.perf_counter() - started >= settings.forward_wall_seconds:
        raise TimeoutError("forward-driver setup exhausted its wall budget")
    if not track.is_closed and plan.speed[0] != 0.:
        raise ValueError("forward open-course plan must begin at a standing start")
    if not track.is_closed and plan.is_closed:
        raise ValueError("forward open-course plan must be nonperiodic")
    if track.is_closed and not plan.is_closed:
        raise ValueError("forward closed track must produce a periodic plan")
    storage = StorageState(control.drivetrain.powertrain.energy_storage.initial_state_of_charge)
    envelope = build_braking_envelope(distance, curvature, control,
        maximum_speed=settings.forward_maximum_speed, lateral_margin=.9,
        deceleration_margin=settings.forward_deceleration_margin,
        speed_query_points=settings.forward_speed_points, storage_state=storage,
        is_closed=bool(track.is_closed))
    config = forward_driver_config(settings)
    policy_settings = EnvelopeDriverSettings(speed_gain=config["speed_gain"],
        throttle_acceleration_scale=config["throttle_acceleration_scale"],
        brake_acceleration_scale=config["brake_acceleration_scale"],
        response_allowance_s=config["response_allowance_s"],
        positive_acceleration_reserve_mps2=config["positive_acceleration_reserve_mps2"],
        feedforward_distance_m=config["feedforward_distance_m"])
    plant = PlanarForwardPlant(control, plan)
    driver = PreviewDriver(plan, control.wheelbase, control.steering_maximum_angle,
        control.steering_maximum_rate, control.maximum_pedal_force, preview_seconds=config["preview_seconds"],
        speed_gain=config["speed_gain"], heading_gain=config["heading_gain"], lateral_gain=config["lateral_gain"],
        pedal_time_constant=config["pedal_time_constant_s"], throttle_apply_rate=config["throttle_apply_rate"],
        throttle_release_rate=config["throttle_release_rate"], brake_apply_rate=config["brake_apply_rate"],
        brake_release_rate=config["brake_release_rate"], steering_time_constant=config["steering_time_constant_s"],
        envelope_policy=EnvelopePolicy(envelope, policy_settings))
    initial = plant.initial_state(float(config["initial_speed_mps"]))
    run_record = attempt / "run_record.npz"
    sink = RunRecordSink(plant, track.path_coordinates, run_record, precision="float32")
    policy_path = attempt / "accepted_policy.jsonl"
    policy_stream = policy_path.open("w", encoding="utf-8")
    source_paths = {formulation, track_path, Path(compiled_result.manifest.assembly.path).resolve()}
    source_paths.update(Path(item.path) for item in compiled_result.manifest.loaded_components)
    source_hash_map = source_hashes(source_paths)
    if source_hash_map != frozen_sources:
        raise ValueError("run source hashes differ from immutable prepared identity")
    manifest_provenance = {
        "formulation": {"model_id": compiled_result.manifest.formulation.model_id,
            "model_type": compiled_result.manifest.formulation.model_type,
            "subtype": compiled_result.manifest.formulation.subtype,
            "path": str(compiled_result.manifest.formulation.path)},
        "assembly": {"model_id": compiled_result.manifest.assembly.model_id,
            "model_type": compiled_result.manifest.assembly.model_type,
            "subtype": compiled_result.manifest.assembly.subtype,
            "path": str(compiled_result.manifest.assembly.path)},
        "loaded_components": [{"role": item.role, "model_id": item.model_id,
            "subtype": item.subtype, "path": str(item.path)}
            for item in compiled_result.manifest.loaded_components],
        "unused_roles": list(compiled_result.manifest.unused_roles()),
        "capabilities": [{"capability": item.capability, "component_role": item.component_role,
            "component_model_id": item.component_model_id,
            "adapter_names": list(item.adapter_names)} for item in compiled_result.manifest.capabilities]
    }
    def record_step(sample):
        sink(sample)
        policy_stream.write(json.dumps(_policy_row(sample), sort_keys=True, allow_nan=False) + "\n")
        policy_stream.flush()
    elapsed = time.perf_counter() - started
    remaining = settings.forward_wall_seconds - elapsed
    if remaining <= 0:
        raise TimeoutError("forward-driver setup exhausted its wall budget")
    _write(progress, {"status": "running", "phase": "integration-and-recording",
        "elapsed_wall_seconds": elapsed, "remaining_wall_seconds": remaining})
    result = solve_forward(plant, driver, initial, duration=settings.forward_duration,
        wall_seconds=remaining, maximum_wall_seconds=settings.forward_wall_seconds,
        maximum_step=config["maximum_step_s"], rtol=config["rtol"], atol=config["atol"],
        method=config["method"], initial_pedal=config["initial_pedal"],
        step_sink=record_step)
    policy_stream.flush()
    policy_stream.close()
    _write(progress, {"status": "running", "phase": "post-solve-validation",
        "accepted_policy_rows": int(sink.rows), "elapsed_wall_seconds": time.perf_counter() - started})
    record = sink.close({"solver_kind": "forward-driver", "source_model": case["model"],
        "solution_model": "planar-7dof", "reduction": reduction,
        "settings": settings.to_json(), "driver_config": config,
        "producer": {"tool": "tools/sim_gui/forward_execute.py"},
        "model": {"model": case["model"], "formulation_xml": str(formulation),
                  "plant_class": "PlanarForwardPlant", "mechanical_dof": 7},
        "sources": source_hash_map, "compile_manifest": manifest_provenance,
        "integrator": {"method": config["method"],
            "rtol": config["rtol"], "atol": config["atol"],
            "maximum_step": config["maximum_step_s"], "wall_seconds": float(result.wall_seconds)},
        "segment": {"duration_s": float(result.time[-1] - result.time[0]),
            "lap_completed": bool(result.lap_completed), "initial_pedal": config["initial_pedal"]},
        "track": {"file": str(track_path)},
        "vehicle_geometry": vehicle_geometry(control, plant),
        "semantics": "sampled forward transient; not a safety certificate"})
    digest = _sha(record)
    np.savez_compressed(attempt / "envelope_arrays.npz", distance=envelope.distance,
        curvature=envelope.curvature, corner_speed=envelope.corner_speed,
        entry_speed=envelope.entry_speed, deceleration=envelope.deceleration,
        binding_index=envelope.binding_index,
        provenance=np.asarray(json.dumps({"settings": settings.to_json(),
            "builder": "build_braking_envelope", "source_hashes": source_hash_map,
            "compile_manifest": manifest_provenance}, sort_keys=True)))
    midpoints = int(result.dense_samples)
    n_index = plant.state_names.index("n")
    accepted_n = np.asarray(result.state[n_index], dtype=float)
    midpoint_n = np.asarray(result.dense_sample_state[:, n_index], dtype=float)
    accepted_track_error = float(np.max(np.abs(accepted_n))) if accepted_n.size else 0.
    midpoint_track_error = float(np.max(np.abs(midpoint_n))) if midpoint_n.size else 0.
    summary = {"schema": "sim-gui-forward-v1", "status": "completed" if result.lap_completed else "incomplete",
        "lap_completed": bool(result.lap_completed), "simulated_seconds": float(result.time[-1]),
        "accepted_steps": int(result.time.size - 1), "midpoint_samples": midpoints,
        "midpoint_sample_validation_complete": bool(result.midpoint_sample_validation_complete),
        "sampled_tracking_pass": bool(max(accepted_track_error, midpoint_track_error) <= .25),
        "accepted_max_abs_lateral_m": accepted_track_error,
        "midpoint_max_abs_lateral_m": midpoint_track_error,
        "tracking_limit_m": .25,
        "forward_driver_config": config,
        "run_record_sha256": digest, "envelope_arrays_sha256": _sha(attempt / "envelope_arrays.npz"),
        "reduction": reduction, "source_hashes": source_hash_map,
        "accepted_policy_file": "accepted_policy.jsonl", "accepted_policy_rows": int(sink.rows),
        "accepted_policy_sha256": _sha(policy_path),
        "semantics": "sampled tracking evidence; not a safety certificate",
        "settings_sha256": expected["settings_sha256"], "total_wall_seconds": time.perf_counter() - started,
        "accepted_states_validated": int(result.time.size), "dense_samples_validated": midpoints,
        "validation_counts": {"accepted_states_validated": int(result.time.size), "dense_probes_validated": midpoints}}
    _write(attempt / "summary.json", summary)
    _write(progress, {"status": summary["status"], "phase": "finished",
        "completed_forward_result": True, "lap_completed": bool(result.lap_completed),
        "elapsed_wall_seconds": summary["total_wall_seconds"]})
    _write(attempt / "inputs.json", {**expected, "reduction": reduction,
        "run_record_sha256": digest, "run_record": "run_record.npz",
        "completed_forward_result": True, "lap_completed": bool(result.lap_completed),
        "accepted_policy_file": "accepted_policy.jsonl", "accepted_policy_rows": int(sink.rows),
        "accepted_policy_sha256": _sha(policy_path), "source_hashes": source_hash_map,
        "midpoint_sample_validation_complete": bool(result.midpoint_sample_validation_complete),
        "accepted_states_validated": int(result.time.size), "dense_samples_validated": midpoints})
    return 0


def run(case: dict, attempt: Path, settings: RunSettings, case_dir: Path) -> int:
    """Run the workflow and preserve diagnostics plus any accepted prefix."""
    attempt = Path(attempt)
    started = time.perf_counter()
    try:
        return _run_impl(case, attempt, settings, case_dir)
    except Exception as error:
        policy_path = attempt / "accepted_policy.jsonl"
        if attempt.is_dir() and not policy_path.exists():
            policy_path.touch()
        if policy_path.exists():
            with policy_path.open(encoding="utf-8") as stream:
                rows = sum(1 for _ in stream)
        else:
            rows = 0
        failure = {"schema": "sim-gui-forward-v1", "status": "failed",
            "exception": type(error).__name__, "message": str(error),
            "total_wall_seconds": time.perf_counter() - started,
            "accepted_policy_file": "accepted_policy.jsonl",
            "accepted_policy_prefix_rows": rows,
            "semantics": "incomplete sampled forward transient; not a safety certificate"}
        if hasattr(error, "diagnostics"):
            failure["solver_diagnostics"] = error.diagnostics
        if hasattr(error, "last_state") and error.last_state is not None:
            failure["last_state"] = np.asarray(error.last_state, dtype=float).tolist()
        if hasattr(error, "last_time") and error.last_time is not None:
            failure["last_time"] = float(error.last_time)
        _write(attempt / "summary.json", failure)
        _write(attempt / "progress.json", {"status": "failed", "phase": "failure",
            "exception": type(error).__name__, "elapsed_wall_seconds": failure["total_wall_seconds"],
            "accepted_policy_prefix_rows": rows})
        return 4
