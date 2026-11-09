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
from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant, vertical_plant_class
from simplified_vehicle_models.parameters.vehicle import PlanarVehicleParameters
from simplified_vehicle_models.results.plant_channels import RunRecordSink
from simplified_vehicle_models.results.plant_channels import source_hashes, vehicle_geometry
from simplified_vehicle_models.solvers.forward_driver import StepSample
from simplified_vehicle_models.solvers.braking_envelope import build_braking_envelope
from simplified_vehicle_models.solvers.yaw_transition import yaw_transition_speed_caps
from simplified_vehicle_models.solvers.braking_envelope import restrict_braking_envelope
from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
from simplified_vehicle_models.solvers.envelope_driver import EnvelopeDriverSettings, EnvelopePolicy
from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityPredictor
from simplified_vehicle_models.solvers.forward_driver import PreviewDriver, build_speed_plan, solve_forward
from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
from simplified_vehicle_models.tracks.profile import load_track

from .settings import RunSettings, forward_driver_config, validate
from .source_identity import execution_source_identity, identity_digest, combined_source_closure


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(path: Path, payload: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n")
    temporary.replace(path)


def _hybrid_steering_plan(hybrid_plan):
    """Adapt only the steering driver's sample tuple contract; preserve source plan."""
    from types import SimpleNamespace
    def sample_for_driver(station):
        item = hybrid_plan.sample(station)
        return item.speed_mps, item.curvature_inv_m, item.acceleration_mps2
    return SimpleNamespace(distance=hybrid_plan.distance_m, speed=hybrid_plan.speed_mps,
        curvature=hybrid_plan.curvature_inv_m, is_closed=hybrid_plan.is_closed,
        sample=sample_for_driver)


def _policy_row(sample: StepSample, control=None) -> dict:
    def command(value):
        if value is None:
            return None
        return {"throttle": float(value.throttle), "brake_force": float(value.brake_force),
                "steering_rate": float(value.steering_rate)}
    diagnostics = sample.policy_diagnostics
    if hasattr(sample.applied_powertrain_diagnostics, "to_dict"):
        applied_powertrain = sample.applied_powertrain_diagnostics.to_dict()
    else:
        applied_powertrain = sample.applied_powertrain_diagnostics
    if hasattr(diagnostics, "to_dict"):
        diagnostics = diagnostics.to_dict()
    row={"index": sample.index, "time": sample.time, "pedal": sample.pedal,
        "requested_command": command(sample.requested_command),
        "mapped_command": command(sample.mapped_command), "applied_command": command(sample.command),
        "policy_diagnostics": diagnostics,
        "target_pedal_position": (float(sample.requested_command.throttle - sample.requested_command.brake_force / control.maximum_pedal_force) if control is not None and sample.requested_command is not None else None),
        "applied_powertrain_diagnostics": applied_powertrain}
    if control is not None:
        from simplified_vehicle_models.solvers.forward_powertrain import shape_brake_command
        from simplified_vehicle_models.controls.brakes import brake_torques
        _, hydraulic = shape_brake_command(control, float(sample.command.brake_force))
        brake=brake_torques(hydraulic, control.maximum_pedal_force, control.pedal_curve,
            control.hydraulic_ratio_front, control.hydraulic_ratio_rear, control.maximum_brake_pressure,
            control.caliper_area, control.brake_bias, control.pad_mu_front, control.pad_mu_rear,
            control.brake_effective_radius_front, control.brake_effective_radius_rear)
        motor_torque = (applied_powertrain or {}).get("wheel_motor_torque_nm")
        row["applied_powertrain_diagnostics"]={"wheel_motor_torque_nm":motor_torque,
            "wheel_brake_torque_nm":brake.wheel_torques.tolist(),"brake_bias":float(control.brake_bias),
            "front_pressure_pa":float(brake.front_line_pressure),"rear_pressure_pa":float(brake.rear_line_pressure),
            "pressure_limited":bool(brake.is_pressure_limited)}
    return row


def _run_impl(case: dict, attempt: Path, settings: RunSettings, case_dir: Path) -> int:
    """Run the exact selected GUI variant in one parent-supervised attempt.

    ``attempt`` is the fresh child output directory. No nested watchdog is
    started here; the existing GUI child supervisor owns cancellation/deadline.
    """
    started = time.perf_counter()
    case_dir = Path(case_dir).resolve()
    attempt = Path(attempt)
    if attempt.is_symlink():
        raise ValueError("forward-driver attempt must not be a symlink")
    attempt = attempt.resolve()
    try:
        attempt.relative_to(case_dir)
    except ValueError as error:
        raise ValueError("forward-driver attempt must resolve inside its case directory") from error
    if not attempt.is_dir():
        raise ValueError("forward-driver requires a parent-created attempt directory")
    entries = list(attempt.iterdir())
    if entries and (len(entries) != 1 or entries[0].name != "run.log" or entries[0].is_symlink()
                    or not entries[0].is_file()):
        raise ValueError("forward-driver attempt must be fresh except for parent run.log")
    progress = attempt / "progress.json"
    _write(progress, {"status": "running", "phase": "identity-validation"})
    formulation = (Path(case["formulation"]) if Path(case["formulation"]).is_absolute()
                  else Path(case_dir) / case["formulation"]).resolve()
    track_path = Path(case["track"]).resolve()
    identity = case.get("input_identity", {})
    if not formulation.is_file() or not track_path.is_file():
        raise ValueError("forward-driver input file is missing")
    config = forward_driver_config(settings)
    mode = config["longitudinal_mode"]
    physical_mode = mode == "physical-reachable-v1"
    expected = {
        "solver_kind": "forward-driver", "source_model": case.get("model"),
        "solution_model": case.get("model"), "formulation": str(formulation),
        "formulation_sha256": _sha(formulation), "track": str(track_path),
        "track_sha256": _sha(track_path), "settings": settings.to_json(),
        "settings_sha256": hashlib.sha256(json.dumps(settings.to_json(), sort_keys=True,
            separators=(",", ":"), allow_nan=False).encode()).hexdigest(),
        "driver_config": forward_driver_config(settings),
    }
    execution_sources = execution_source_identity()
    hybrid_mode = mode == "hybrid-drive-brake-v1"
    historical_mode = mode == "historical-preview-v1"
    if physical_mode or settings.forward_transient_corner_planning or hybrid_mode:
        expected["execution_source_hashes"] = execution_sources
    if physical_mode or settings.forward_transient_corner_planning or hybrid_mode:
        if identity.get("execution_source_hashes") != execution_sources:
            raise ValueError("forward-driver execution source identity mismatch")
        if identity.get("execution_source_identity_sha256") != identity_digest(execution_sources):
            raise ValueError("forward-driver execution source identity digest mismatch")
        complete = combined_source_closure(identity.get("component_source_hashes"), execution_sources)
        if identity.get("complete_source_closure") != complete or identity.get("complete_source_closure_sha256") != identity_digest(complete):
            raise ValueError("forward-driver complete source closure identity mismatch")
        manifest_digest = identity_digest({rel: "required" for rel in sorted(execution_sources)})
        if identity.get("source_identity_manifest_sha256") != manifest_digest:
            raise ValueError("forward-driver source identity manifest mismatch")
        closure_digest = identity_digest(identity.get("component_source_hashes"))
        if identity.get("authority_source_closure_sha256") != closure_digest:
            raise ValueError("forward-driver authority source/calibration identity mismatch")
    elif identity.get("execution_source_hashes") not in (None, execution_sources):
        raise ValueError("forward-driver execution source identity mismatch")
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
    if hybrid_mode and case["model"] != "14dof-simple":
        raise ValueError("hybrid-drive-brake-v1 requires full vertical 14DoF plant")
    if case["model"] == "14dof-simple":
        if not isinstance(compiled, VerticalVehicleParameters):
            raise ValueError(f"vertical forward-driver requires 14DoF source, got {type(compiled).__name__}")
        control = compiled.planar
        plant_type = vertical_plant_class(compiled)
        reduction = "identity"
    elif case["model"] == "planar-7dof":
        if not isinstance(compiled, PlanarVehicleParameters):
            raise ValueError(f"planar forward-driver requires 7DoF source, got {type(compiled).__name__}")
        control = compiled
        plant_type = PlanarForwardPlant
        reduction = "identity"
    else:
        raise ValueError(f"unsupported forward-driver source model: {case['model']}")
    config = forward_driver_config(settings)
    track = load_track(track_path)
    count = max(2, int(math.ceil(track.lap_length / settings.forward_spacing)))
    distance = np.linspace(0., track.lap_length, count + 1)
    _, _, _, curvature = evaluate_path(track.path_coordinates, distance)
    plan = None
    if not hybrid_mode:
        plan = build_speed_plan(distance, curvature, settings.forward_maximum_speed,
            vehicle=control, live_limit=True, open_course=not track.is_closed, lateral_margin=.9)
    if time.perf_counter() - started >= settings.forward_wall_seconds:
        raise TimeoutError("forward-driver setup exhausted its wall budget")
    if plan is not None:
        if not track.is_closed and plan.speed[0] != 0.:
            raise ValueError("forward open-course plan must begin at a standing start")
        if not track.is_closed and plan.is_closed:
            raise ValueError("forward open-course plan must be nonperiodic")
        if track.is_closed and not plan.is_closed:
            raise ValueError("forward closed track must produce a periodic plan")
    storage = StorageState(control.drivetrain.powertrain.energy_storage.initial_state_of_charge)
    envelope = None
    if not hybrid_mode and not historical_mode:
        envelope = build_braking_envelope(distance, curvature, control,
            maximum_speed=settings.forward_maximum_speed, lateral_margin=.9,
            deceleration_margin=settings.forward_deceleration_margin,
            speed_query_points=settings.forward_speed_points, storage_state=storage,
            is_closed=bool(track.is_closed))
    hybrid_setup = None
    hybrid_plan_sample = None
    if hybrid_mode:
        if not isinstance(compiled, VerticalVehicleParameters):
            raise ValueError("hybrid-drive-brake-v1 requires the full vertical 14DoF plant")
        if settings.forward_transient_corner_planning:
            raise ValueError("hybrid mode does not support transient yaw/TV planning")
        from simplified_vehicle_models.solvers.envelope_driver import PedalPolicyContext
        from simplified_vehicle_models.solvers.hybrid_setup import (
            build_hybrid_setup, HYBRID_EXECUTION_ALLOCATOR_ID, HYBRID_HORIZON_S, HYBRID_SLABS)
        pc=PedalPolicyContext(config["throttle_release_rate"],config["brake_apply_rate"],config["pedal_time_constant_s"],
            control.maximum_pedal_force, .85, config["throttle_apply_rate"],config["brake_release_rate"])
        from simplified_vehicle_models.tires.unitire_simple import loaded_radius
        # Setup uses compiled-model rated loads only as grid radii metadata; policy
        # queries use actual current-state radii from its brake context.
        reference_loads = __import__("simplified_vehicle_models.formulations.point_mass", fromlist=["normal_loads"]).normal_loads(
            __import__("simplified_vehicle_models.formulations.point_mass", fromlist=["point_mass_view_of_planar"]).point_mass_view_of_planar(control),
            max(float(config["initial_speed_mps"]), 1.0), 0.0, 0.0)
        radii=loaded_radius(control.tire, reference_loads)
        model_hashes={}
        repository=Path(__file__).resolve().parents[2]
        for source in frozen_sources:
            source_path=Path(source).resolve()
            try: rel=source_path.relative_to(repository).as_posix()
            except ValueError: continue
            model_hashes[rel]=_sha(source_path)
        if not model_hashes:
            raise ValueError("hybrid setup requires repository-rooted compiled source hashes")
        hybrid_setup=build_hybrid_setup(control,distance_m=distance,curvature_inv_m=curvature,
            maximum_speed_mps=settings.forward_maximum_speed,initial_speed_mps=float(config["initial_speed_mps"]),
            storage_soc=float(storage.state_of_charge),wheel_radii_m=tuple(float(v) for v in radii),
            model_source_hashes=model_hashes,
            maximum_pedal_force_n=control.maximum_pedal_force,pedal_context=pc,lateral_margin=.9,
            is_closed=bool(track.is_closed),
            execution_allocator_identity=HYBRID_EXECUTION_ALLOCATOR_ID)
        hybrid_plan=hybrid_setup.plan
        plant=plant_type(compiled,hybrid_plan)
        # PreviewDriver consumes the legacy tuple sample contract for steering;
        # keep the authoritative HybridSpeedPlan on the plant and adapt only this
        # independent steering interface.
        plan=_hybrid_steering_plan(hybrid_plan)
        hybrid_plan_sample=hybrid_plan.sample
    transient_evidence = None
    if settings.forward_transient_corner_planning:
        evidence = yaw_transition_speed_caps(distance, curvature, control,
            envelope.corner_speed, is_closed=bool(track.is_closed))
        semantic_payload = {"caps": evidence.caps.tolist(),
            "curvature_gradient": evidence.curvature_gradient.tolist(),
            "provenance": dict(evidence.provenance),
            "steering_feedforward": "actuator-inverse"}
        semantic = hashlib.sha256(json.dumps(semantic_payload, sort_keys=True,
            separators=(",", ":"), allow_nan=False).encode()).hexdigest()
        pre_transient_plan = plan
        pre_transient_envelope = envelope
        envelope = restrict_braking_envelope(envelope, evidence.caps,
            provenance={"yaw_transition_semantic_sha256": semantic})
        speeds = np.minimum(plan.speed, envelope.entry_speed)
        if not plan.is_closed:
            speeds[0] = 0.0
        plan = SpeedPlan(plan.distance, speeds, plan.curvature, is_closed=plan.is_closed)
        transient_evidence = {**semantic_payload, "semantic_sha256": semantic,
            "restricted_nodes": int(np.count_nonzero(evidence.caps < pre_transient_envelope.corner_speed)),
            "plan_speed_changed_nodes": int(np.count_nonzero(plan.speed < pre_transient_plan.speed - 1e-9)),
            "envelope_corner_speed_changed_nodes": int(np.count_nonzero(envelope.corner_speed < pre_transient_envelope.corner_speed - 1e-9)),
            "steering_selection": "actuator-inverse"}
    hybrid_mode = settings.forward_longitudinal_mode == "hybrid-drive-brake-v1"
    policy_settings = None if hybrid_mode or historical_mode else EnvelopeDriverSettings(speed_gain=config["speed_gain"],
        throttle_acceleration_scale=config["throttle_acceleration_scale"],
        brake_acceleration_scale=config["brake_acceleration_scale"],
        response_allowance_s=config["response_allowance_s"],
        positive_acceleration_reserve_mps2=config["positive_acceleration_reserve_mps2"],
        feedforward_distance_m=config["feedforward_distance_m"],
        planning_transition_time_s=None, forecast_horizon_s=config["physical_brake_forecast_horizon_s"] or 3.0,
        mode=config["longitudinal_mode"] )
    plant = plant_type(compiled, plan) if plant_type is not PlanarForwardPlant else plant_type(control, plan)
    hybrid_mode = mode == "hybrid-drive-brake-v1"
    if hybrid_mode:
        if float(config["initial_speed_mps"]) <= 0.:
            raise ValueError("hybrid-drive-brake-v1 supports rolling starts only; standing rest is not qualified")
        if not isinstance(plant, VerticalForwardPlant):
            raise ValueError("hybrid-drive-brake-v1 requires the full vertical 14DoF plant")
        if settings.forward_transient_corner_planning:
            raise ValueError("hybrid mode does not support transient yaw/TV planning")
    if physical_mode and not isinstance(plant, VerticalForwardPlant):
        raise ValueError("physical brake forecasting requires the full vertical 14DoF plant; planar/point-mass contexts are unsupported")
    if physical_mode and track.is_closed and plan.speed[0] <= 0.:
        raise ValueError("physical brake forecasting requires positive initial speed within predictor domain")
    if hybrid_mode and track.is_closed and plan.speed[0] <= 0.:
        raise ValueError("hybrid mode requires positive initial speed within capability domain")
    predictor = None
    planning_factory = None
    current_factory = None
    if physical_mode:
        predictor = BrakeAuthorityPredictor(control, maximum_abs_curvature_inv_m=float(np.max(np.abs(curvature))),
                                             minimum_speed_mps=min(2.0, max(.1, float(np.min(plan.speed)))),
                                             maximum_speed_mps=max(40.0, settings.forward_maximum_speed))
        planning_factory = lambda plant, observation, pedal, state: plant.brake_planning_context(
            state, pedal, predictor=predictor)
        current_factory = lambda plant, observation, pedal, state: plant.brake_authority_context(
            state, predictor=predictor)
    if hybrid_mode:
        predictor=hybrid_setup.brake_predictor
        planning_factory=None
        current_factory=lambda p, observation, pedal, state: p.brake_authority_context(state,predictor=predictor)
    driver = PreviewDriver(plan, control.wheelbase, control.steering_maximum_angle,
        control.steering_maximum_rate, control.maximum_pedal_force, preview_seconds=config["preview_seconds"],
        speed_gain=config["speed_gain"], heading_gain=config["heading_gain"], lateral_gain=config["lateral_gain"],
        pedal_time_constant=config["pedal_time_constant_s"], throttle_apply_rate=config["throttle_apply_rate"],
        throttle_release_rate=config["throttle_release_rate"], brake_apply_rate=config["brake_apply_rate"],
        brake_release_rate=config["brake_release_rate"], steering_time_constant=config["steering_time_constant_s"],
        envelope_policy=None if hybrid_mode or historical_mode else EnvelopePolicy(envelope, __import__("dataclasses").replace(
            policy_settings, mode="physical-reachable-v1" if physical_mode else "legacy")),
        hybrid_policy=hybrid_setup.policy if hybrid_mode else None,
        brake_planning_context_factory=planning_factory,
        brake_current_context_factory=current_factory,
        brake_current_predictor=predictor,
        steering_feedforward=("actuator-inverse" if settings.forward_transient_corner_planning else "preview"))
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
    from .telemetry_schema import project as project_telemetry, schema_document
    policy_rows = []
    telemetry_rows = []
    def record_step(sample):
        sink(sample)
        row = _policy_row(sample, control)
        if historical_mode:
            row["longitudinal_mode"] = "historical-preview-v1"
        if hybrid_mode:
            raw=sample.policy_diagnostics.to_dict()
            obs=plant.observe(sample.state)
            plan_sample=hybrid_setup.plan.sample(float(obs.distance))
            row.update(longitudinal_mode="hybrid-drive-brake-v1",decision={"schema":"hybrid-telemetry-v1",**raw},
                station_m=float(obs.distance),plan_sample={"speed_mps":plan_sample.speed_mps,
                    "corner_speed_cap_mps":plan_sample.corner_speed_cap_mps,
                    "braking_speed_bound_mps":plan_sample.braking_speed_bound_mps},
                planner_horizon_s=HYBRID_HORIZON_S,rollout_slabs=HYBRID_SLABS,
                max_extra_forecasts=hybrid_setup.policy.config.max_extra_forecasts,
                bracket_resolution=hybrid_setup.policy.config.bracket_resolution,
                max_total_forecasts=3+hybrid_setup.policy.config.max_extra_forecasts,
                max_total_transitions=(3+hybrid_setup.policy.config.max_extra_forecasts)*hybrid_setup.policy.config.slabs,
                provider_identity=hybrid_setup.provider.identity,
                allocator_identity=HYBRID_EXECUTION_ALLOCATOR_ID)
        telemetry_rows.append(project_telemetry(row))
        policy_rows.append(row)
        policy_stream.write(json.dumps(row, sort_keys=True, allow_nan=False) + "\n")
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
    telemetry_payload={"schema":schema_document(config["longitudinal_mode"]),"rows":telemetry_rows}
    record = sink.close({"solver_kind": "forward-driver", "source_model": case["model"],
        "solution_model": case["model"], "reduction": reduction,
        "settings": settings.to_json(), "driver_config": config,
        "producer": {"tool": "tools/sim_gui/forward_execute.py"},
        "model": {"model": case["model"], "formulation_xml": str(formulation),
                  "plant_class": type(plant).__name__,
                  "mechanical_dof": 14 if isinstance(plant, VerticalForwardPlant) else 7},
        "task5_telemetry": telemetry_payload,
        "sources": source_hash_map, "compile_manifest": manifest_provenance,
        "execution_source_hashes": execution_sources,
        "complete_source_closure_sha256": identity.get("complete_source_closure_sha256"),
        "transient_corner_evidence": transient_evidence,
        "longitudinal_mode": config["longitudinal_mode"],
        **({"hybrid_setup_identity": hybrid_setup.setup_identity,
            "hybrid_setup_version": HYBRID_SETUP_VERSION,
            "hybrid_policy_version": __import__("simplified_vehicle_models.solvers.hybrid_driver",fromlist=["HYBRID_POLICY_VERSION"]).HYBRID_POLICY_VERSION,
            "hybrid_allocator_identity": HYBRID_EXECUTION_ALLOCATOR_ID,
            "hybrid_provider_identity": hybrid_setup.provider.identity,
            "hybrid_plan_identity": hybrid_setup.plan.identity,
            "hybrid_max_extra_forecasts": hybrid_setup.policy.config.max_extra_forecasts,
            "hybrid_bracket_resolution": hybrid_setup.policy.config.bracket_resolution,
            "hybrid_max_total_transitions": (3+hybrid_setup.policy.config.max_extra_forecasts)*hybrid_setup.policy.config.slabs,
            "hybrid_max_extra_forecasts": hybrid_setup.policy.config.max_extra_forecasts,
            "hybrid_bracket_resolution": hybrid_setup.policy.config.bracket_resolution,
            "hybrid_max_total_forecasts": 3+hybrid_setup.policy.config.max_extra_forecasts,
            "hybrid_setup_identity_payload": hybrid_setup.setup_identity_payload_json}
            if hybrid_mode else {}),
        "forecast_semantics": ("operational sampled diagnostic, version operational-diagnostic-v1; not a feasibility certificate"
                               if physical_mode else "legacy"),
        "integrator": {"method": config["method"],
            "rtol": config["rtol"], "atol": config["atol"],
            "maximum_step": config["maximum_step_s"], "wall_seconds": float(result.wall_seconds)},
        "segment": {"duration_s": float(result.time[-1] - result.time[0]),
            "lap_completed": bool(result.lap_completed), "initial_pedal": config["initial_pedal"]},
        "track": {"file": str(track_path)},
        "vehicle_geometry": vehicle_geometry(control, plant),
        "semantics": "sampled forward transient; not a safety certificate"})
    digest = _sha(record)
    if hybrid_mode:
        np.savez_compressed(attempt / "hybrid_plan_arrays.npz", distance=hybrid_setup.plan.distance_m,
            curvature=hybrid_setup.plan.curvature_inv_m,
            speed_mps=hybrid_setup.plan.speed_mps,
            corner_speed_cap_mps=hybrid_setup.plan.corner_speed_cap_mps,
            braking_speed_bound_mps=hybrid_setup.plan.braking_speed_bound_mps,
            provenance=np.asarray(json.dumps({"setup_identity": hybrid_setup.setup_identity,
                "setup_identity_payload": hybrid_setup.setup_identity_payload_json,
                "source_hashes": source_hash_map}, sort_keys=True)))
    elif historical_mode:
        np.savez_compressed(attempt / "historical_plan_arrays.npz", distance=plan.distance,
            curvature=plan.curvature, speed=plan.speed, acceleration=plan.acceleration,
            provenance=np.asarray(json.dumps({"builder": "build_speed_plan", "live_limit": True,
                "source_hashes": source_hash_map}, sort_keys=True)))
    else:
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
        "execution_source_hashes": execution_sources,
        "complete_source_closure_sha256": identity.get("complete_source_closure_sha256"),
        "transient_corner_evidence": transient_evidence,
        "task5_telemetry": telemetry_payload,
        "longitudinal_mode": config["longitudinal_mode"],
        "transient_corner_planning": settings.forward_transient_corner_planning,
        "physical_forecast_diagnostics": {
            "unknown_count": sum(row.get("policy_diagnostics", {}).get("feasible_schedule") is None for row in policy_rows),
            "true_count": sum(row.get("policy_diagnostics", {}).get("feasible_schedule") is True for row in policy_rows),
            "false_count": sum(row.get("policy_diagnostics", {}).get("feasible_schedule") is False for row in policy_rows),
            "horizon_s": config["physical_brake_forecast_horizon_s"],
            "raw_inverse_force_n": [row.get("policy_diagnostics", {}).get("raw_target_inverse_force_n") for row in policy_rows],
            "final_inverse_force_n": [row.get("policy_diagnostics", {}).get("final_inverse_force_n") for row in policy_rows],
            "target_deceleration_mps2": [row.get("policy_diagnostics", {}).get("target_inverse_deceleration_mps2") for row in policy_rows],
            "rollout_horizon_s": [row.get("policy_diagnostics", {}).get("rollout_horizon_s") for row in policy_rows],
            "forecast_reason": [row.get("policy_diagnostics", {}).get("forecast_reason") for row in policy_rows],
            "event_station_m": [row.get("policy_diagnostics", {}).get("forecast_event_station_m") for row in policy_rows],
            "crossing_time_s": [row.get("policy_diagnostics", {}).get("forecast_crossing_time_s") for row in policy_rows],
            "cap_slack_m2_s2": [row.get("policy_diagnostics", {}).get("forecast_cap_slack_mps2_squared") for row in policy_rows],
            "forecast_samples": [row.get("policy_diagnostics", {}).get("forecast_samples") for row in policy_rows],
            "slack_units": "m^2/s^2",
        } if physical_mode else None,
        "run_record_sha256": digest,
        **({"hybrid_plan_arrays_sha256": _sha(attempt / "hybrid_plan_arrays.npz"),
            "hybrid_setup_identity": hybrid_setup.setup_identity,
            "hybrid_setup_identity_payload": hybrid_setup.setup_identity_payload_json}
            if hybrid_mode else {"historical_plan_arrays_sha256": _sha(attempt / "historical_plan_arrays.npz")}
            if historical_mode else {"envelope_arrays_sha256": _sha(attempt / "envelope_arrays.npz")}),
        "reduction": reduction, "source_hashes": source_hash_map,
        "accepted_policy_file": "accepted_policy.jsonl", "accepted_policy_rows": int(sink.rows),
        "accepted_policy_sha256": _sha(policy_path),
        "task5_telemetry_schema": schema_document(config["longitudinal_mode"]),
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
