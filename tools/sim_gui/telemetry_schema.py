"""Versioned accepted-row projections for envelope and hybrid decisions."""
from __future__ import annotations

SCHEMA = "task5-telemetry-v1"
HYBRID_SCHEMA = "hybrid-telemetry-v1"

# name -> (unit, source path, availability contract)
FIELDS = {
    "time_s": ("s", "accepted_step.time", "required"),
    "station_m": ("m", "policy.distance", "required"),
    "desired_acceleration_mps2": ("m/s^2", "policy.acceleration_request", "required"),
    "authority_prediction_mps2": ("m/s^2", "policy.predicted_deceleration_mps2", "nullable: predictor/context unavailable or domain exit"),
    "authority_domain_exceeded": ("1", "policy.authority_domain_exceeded", "required"),
    "authority_error_mps2": ("m/s^2", "policy.tracking_error_mps2", "nullable: authority prediction unavailable"),
    "authority_shortfall_mps2": ("m/s^2", "policy.authority_shortfall_mps2", "nullable: authority prediction unavailable"),
    "raw_target_force_n": ("N", "policy.raw_target_inverse_force_n", "nullable: physical planner unavailable"),
    "limited_force_n": ("N", "policy.physical_brake_force_n", "nullable: physical planner unavailable"),
    "governor_factor": ("1", "policy.governor_factor", "required"),
    "governed_force_n": ("N", "policy.governed_brake_force", "required"),
    "target_pedal_position": ("1", "accepted_policy.target_pedal_position", "nullable: request unavailable"),
    "actual_pedal_position": ("1", "accepted_step.pedal", "nullable: pedal dynamics disabled"),
    "actual_transition_time_s": ("s", "policy.actual_transition_time_s", "nullable: pedal context unavailable"),
    "transition_forecast_ready": ("1", "policy.feasible_schedule", "nullable: forecast indeterminate"),
    "transition_forecast_reason": ("1", "policy.forecast_reason", "nullable: no event/context"),
    "transition_forecast_horizon_s": ("s", "policy.transition_forecast_horizon_s", "required"),
    "forecast_event_station_m": ("m", "policy.forecast_event_station_m", "nullable: no binding event"),
    "forecast_crossing_time_s": ("s", "policy.forecast_crossing_time_s", "nullable: not crossed"),
    "forecast_cap_slack_m2_s2": ("m^2/s^2", "policy.forecast_cap_slack_mps2_squared", "nullable: not evaluated"),
    "competing_current_corner_cap_mps": ("m/s", "policy.competing_corner_cap_current_mps", "required"),
    "competing_preview_entry_cap_mps": ("m/s", "policy.competing_corner_cap_preview_mps", "nullable: preview sample unavailable"),
    "selected_binding_index": ("1", "policy.binding_index", "required"),
    "selected_binding_station_m": ("m", "policy.binding_station", "required"),
    "selected_binding_distance_m": ("m", "policy.binding_distance", "required"),
    "wheel_brake_torque_nm": ("N*m", "hydraulic brake law from applied command", "required"),
    "wheel_motor_torque_nm": ("N*m", "accepted powertrain.motor_wheel_torques", "required"),
    "brake_bias": ("1", "compiled vehicle brake_bias", "required"),
    "front_pressure_pa": ("Pa", "hydraulic brake law from applied command", "required"),
    "rear_pressure_pa": ("Pa", "hydraulic brake law from applied command", "required"),
    "pressure_limited": ("1", "hydraulic brake law from applied command", "required"),
    "tire_capacity_n": ("N", "unavailable: predictor does not provide measured current tire authority", "nullable: unavailable_reason required"),
}

HYBRID_FIELDS = {
    "time_s": ("s", "accepted.time", "required"),
    "station_m": ("m", "accepted.observation.distance", "required"),
    "plan_speed_target_mps": ("m/s", "HybridSpeedPlan.sample.speed_mps", "required"),
    "corner_speed_cap_mps": ("m/s", "HybridSpeedPlan.sample.corner_speed_cap_mps", "required"),
    "backward_braking_ceiling_mps": ("m/s", "HybridSpeedPlan.sample.braking_speed_bound_mps", "required"),
    "target_progress_acceleration_mps2": ("m/s^2", "HybridDecision.target_progress_acceleration_mps2", "required"),
    "raw_acceleration_request_mps2": ("m/s^2", "HybridDecision.raw_acceleration_request_mps2", "required"),
    "target_body_vx_dot_mps2": ("m/s^2", "HybridDecision.body_vx_dot_target_mps2", "required"),
    "target_pedal_position": ("1", "selected requested command", "nullable: unavailable"),
    "actual_pedal_position": ("1", "accepted state pedal", "nullable: pedal dynamics disabled"),
    "requested_throttle": ("1", "requested command", "required"),
    "requested_brake_force_n": ("N", "requested command", "required"),
    "mapped_throttle": ("1", "mapped command", "required"),
    "mapped_brake_force_n": ("N", "mapped command", "required"),
    "applied_throttle": ("1", "accepted applied command", "required"),
    "applied_brake_force_n": ("N", "accepted applied command", "required"),
    "phase": ("1", "HybridDecision.phase", "required"),
    "status": ("1", "HybridDecision.status", "required"),
    "event_crossed": ("1", "HybridDecision.event_crossed", "nullable: no future event"),
    "event_crossing_time_s": ("s", "HybridDecision.event_crossing_time_s", "nullable: event not crossed"),
    "event_speed_cap_mps": ("m/s", "HybridDecision.event_speed_cap_mps", "nullable: no binding event"),
    "planner_horizon_s": ("s", "HybridPolicyConfig.horizon_s", "required"),
    "rollout_slabs": ("1", "HybridPolicyConfig.slabs", "required"),
    "max_extra_forecasts": ("1", "HybridPolicyConfig.max_extra_forecasts", "required"),
    "bracket_resolution": ("1", "HybridPolicyConfig.bracket_resolution", "required normalized pedal units"),
    "max_total_forecasts": ("1", "3 + max_extra_forecasts", "required"),
    "max_total_transitions": ("1", "max_total_forecasts * rollout_slabs", "hard aggregate attempted work"),
    "original_candidate_count": ("1", "HybridDecision.candidate_count", "required"),
    "extra_probe_count": ("1", "HybridDecision.extra_probe_count", "required"),
    "attempted_transitions": ("1", "HybridDecision.attempted_transitions", "required; includes invalid attempts"),
    "selected_mixing_coefficient": ("1", "HybridDecision.selected_mixing_coefficient", "nullable: no relay search"),
    "selected_event_residual_mps": ("m/s", "HybridDecision.selected_event_residual_mps", "nullable: no verified mixed endpoint"),
    "search_termination_reason": ("1", "HybridDecision.search_termination_reason", "nullable: no relay search"),
    "decision_query_count": ("count", "HybridDecision.decision_query_count", "nullable: call-local provider query delta when exposed; null means unavailable"),
    "actual_event_time_s": ("s", "accepted-state event transition", "unavailable: actual event transition is not instrumented"),
    "capability_domain_exceeded": ("1", "HybridDecision.authority_domain_exceeded", "required"),
    "provider_identity": ("1", "sampled capability provider", "required"),
    "plan_identity": ("1", "HybridDecision.plan_identity", "required"),
    "allocator_identity": ("1", "matched actual non-TV execution allocator", "required"),
    "yaw_reserve_identity": ("1", "none: nonzero yaw is rejected", "unavailable: live TV allocator absent"),
    "tire_capacity_n": ("N", "unavailable: sampled capability does not expose measured tire capacity", "nullable: unavailable_reason required"),
}


def _project_envelope(row: dict) -> dict:
    d = row.get("policy_diagnostics") or {}; applied = row.get("applied_command") or {}
    result = {
        "time_s": row.get("time"), "station_m": d.get("distance"),
        "desired_acceleration_mps2": d.get("acceleration_request"),
        "authority_prediction_mps2": d.get("predicted_deceleration_mps2"),
        "authority_domain_exceeded": d.get("authority_domain_exceeded"),
        "authority_error_mps2": d.get("tracking_error_mps2"), "authority_shortfall_mps2": d.get("authority_shortfall_mps2"),
        "raw_target_force_n": d.get("raw_target_inverse_force_n"), "limited_force_n": d.get("physical_brake_force_n"),
        "governor_factor": d.get("governor_factor"), "target_pedal_position": row.get("target_pedal_position"),
        "governed_force_n": d.get("governed_brake_force"), "actual_pedal_position": row.get("pedal"),
        "actual_transition_time_s": d.get("actual_transition_time_s"), "transition_forecast_ready": d.get("feasible_schedule"),
        "transition_forecast_reason": d.get("forecast_reason"), "transition_forecast_horizon_s": d.get("transition_forecast_horizon_s"),
        "forecast_event_station_m": d.get("forecast_event_station_m"), "forecast_crossing_time_s": d.get("forecast_crossing_time_s"),
        "forecast_cap_slack_m2_s2": d.get("forecast_cap_slack_mps2_squared"),
        "competing_current_corner_cap_mps": d.get("competing_corner_cap_current_mps"),
        "competing_preview_entry_cap_mps": d.get("competing_corner_cap_preview_mps"),
        "selected_binding_index": d.get("binding_index"), "selected_binding_station_m": d.get("binding_station"),
        "selected_binding_distance_m": d.get("binding_distance"), "wheel_motor_torque_nm": None,
        "wheel_brake_torque_nm": None, "brake_bias": None, "front_pressure_pa": None,
        "rear_pressure_pa": None, "pressure_limited": None, "tire_capacity_n": None,
    }
    physical=row.get("applied_powertrain_diagnostics")
    if physical: result.update({k: physical.get(k) for k in ("wheel_motor_torque_nm","wheel_brake_torque_nm","brake_bias","front_pressure_pa","rear_pressure_pa","pressure_limited")})
    result["unavailable_reason"]={k:("predictor does not provide measured current tire authority" if k=="tire_capacity_n" else FIELDS[k][2].split(": ",1)[1] if ": " in FIELDS[k][2] else "source did not provide this value") for k,v in result.items() if v is None and k!="unavailable_reason"}
    return result


def _project_hybrid(row: dict) -> dict:
    raw=row.get("decision")
    if not isinstance(raw,dict) or raw.get("schema") != HYBRID_SCHEMA:
        raise ValueError("hybrid telemetry requires complete tagged raw HybridDecision")
    plan=row.get("plan_sample") or {}; requested=row.get("requested_command") or {}; mapped=row.get("mapped_command") or {}; applied=row.get("applied_command") or {}
    result={"time_s":row.get("time"),"station_m":row.get("station_m"),
        "plan_speed_target_mps":plan.get("speed_mps"),"corner_speed_cap_mps":plan.get("corner_speed_cap_mps"),
        "backward_braking_ceiling_mps":plan.get("braking_speed_bound_mps"),
        "target_progress_acceleration_mps2":raw.get("target_progress_acceleration_mps2"),
        "raw_acceleration_request_mps2":raw.get("raw_acceleration_request_mps2"),
        "target_body_vx_dot_mps2":raw.get("body_vx_dot_target_mps2"),
        "target_pedal_position":row.get("target_pedal_position"),"actual_pedal_position":row.get("pedal"),
        "requested_throttle":requested.get("throttle"),"requested_brake_force_n":requested.get("brake_force"),
        "mapped_throttle":mapped.get("throttle"),"mapped_brake_force_n":mapped.get("brake_force"),
        "applied_throttle":applied.get("throttle"),"applied_brake_force_n":applied.get("brake_force"),
        "phase":raw.get("phase"),"status":raw.get("status"),"event_crossed":raw.get("event_crossed"),
        "event_crossing_time_s":raw.get("event_crossing_time_s"),"event_speed_cap_mps":raw.get("event_speed_cap_mps"),
        "planner_horizon_s":row.get("planner_horizon_s"),"rollout_slabs":row.get("rollout_slabs"),
        "original_candidate_count":raw.get("candidate_count"),"extra_probe_count":raw.get("extra_probe_count"),
        "attempted_transitions":raw.get("attempted_transitions"),"selected_mixing_coefficient":raw.get("selected_mixing_coefficient"),
        "selected_event_residual_mps":raw.get("selected_event_residual_mps"),"search_termination_reason":raw.get("search_termination_reason"),
        "decision_query_count":raw.get("decision_query_count"),
        "max_extra_forecasts":row.get("max_extra_forecasts"),"bracket_resolution":row.get("bracket_resolution"),
        "max_total_forecasts":row.get("max_total_forecasts"),"max_total_transitions":row.get("max_total_transitions"),
        "actual_event_time_s":None,"capability_domain_exceeded":raw.get("authority_domain_exceeded"),
        "provider_identity":raw.get("provider_identity"),"plan_identity":raw.get("plan_identity"),
        "allocator_identity":row.get("allocator_identity"),
        "yaw_reserve_identity":None,"tire_capacity_n":None}
    result["unavailable_reason"]={k:("live TV allocator absent" if k=="yaw_reserve_identity" else "sampled capability does not expose measured tire capacity" if k=="tire_capacity_n" else "actual event transition is not instrumented") for k,v in result.items() if v is None and k not in ("unavailable_reason","event_crossed","event_crossing_time_s","event_speed_cap_mps")}
    return result


def project(row: dict) -> dict:
    """Project a mode-tagged accepted row without inventing policy forecasts."""
    if row.get("longitudinal_mode") == "historical-preview-v1":
        decision = row.get("policy_diagnostics") or {}
        return {"time_s": row.get("time"), "station_m": decision.get("distance"),
                "target_speed_mps": decision.get("target_speed"),
                "feedforward_acceleration_mps2": decision.get("feedforward_acceleration"),
                "desired_acceleration_mps2": decision.get("acceleration_request"),
                "governor_factor": decision.get("governor_factor"),
                "governed_force_n": decision.get("governed_brake_force"),
                "target_pedal_position": row.get("target_pedal_position"),
                "actual_pedal_position": row.get("pedal")}
    return _project_hybrid(row) if row.get("longitudinal_mode") == "hybrid-drive-brake-v1" else _project_envelope(row)


def schema_document(mode="legacy") -> dict:
    if mode == "historical-preview-v1":
        fields = {name: {"unit": unit, "source": "historical preview decision", "availability": "required"}
                  for name, unit in (("time_s", "s"), ("station_m", "m"), ("target_speed_mps", "m/s"),
                  ("feedforward_acceleration_mps2", "m/s^2"), ("desired_acceleration_mps2", "m/s^2"),
                  ("governor_factor", "1"), ("governed_force_n", "N"),
                  ("target_pedal_position", "1"), ("actual_pedal_position", "1"))}
        fields["actual_pedal_position"]["availability"] = "nullable: pedal dynamics disabled"
        return {"schema": "historical-preview-v1", "fields": fields}
    hybrid=mode == "hybrid-drive-brake-v1"
    fields=HYBRID_FIELDS if hybrid else FIELDS
    return {"schema": HYBRID_SCHEMA if hybrid else SCHEMA,
            "fields": {name:{"unit":spec[0],"source":spec[1],"availability":spec[2]} for name,spec in fields.items()}}
