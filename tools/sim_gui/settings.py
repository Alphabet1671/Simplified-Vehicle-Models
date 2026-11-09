"""Serializable settings and model resolution for the GUI solver workflows."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Literal

from simplified_vehicle_models.solvers.hybrid_setup import HYBRID_SETUP_VERSION

MODEL_NAMES = frozenset({"point-mass", "planar-7dof", "14dof-simple"})
SolverKind = Literal["point-mass", "forward-driver"]


@dataclass(frozen=True, slots=True)
class RunSettings:
    """Settings for the forward-driver and point-mass QSS workflows."""
    solver_kind: SolverKind = "forward-driver"
    point_mass_duration: float = 300.0
    point_mass_spacing: float = 0.5
    point_mass_maximum_speed: float = 60.0
    point_mass_lateral_acceleration: float | None = None
    point_mass_pedal_travel_time: float = 0.175
    point_mass_preview_seconds: float = 0.15
    point_mass_heading_gain: float = 1.5
    point_mass_lateral_gain: float = 1.5
    point_mass_rtol: float = 1e-5
    point_mass_method: str = "BDF"
    point_mass_record_precision: str = "float32"
    forward_duration: float = 300.0
    forward_wall_seconds: float = 600.0
    forward_maximum_speed: float = 60.0
    forward_spacing: float = 0.5
    forward_deceleration_margin: float = 0.6
    forward_feedforward_distance: float = 2.0
    forward_speed_points: int = 9
    forward_physical_brake_planning: bool = False
    forward_transient_corner_planning: bool = False
    forward_longitudinal_mode: str = "historical-preview-v1"

    @classmethod
    def for_model(cls, model: str) -> "RunSettings":
        if model not in MODEL_NAMES:
            raise ValueError(f"unsupported GUI model: {model}")
        return cls(solver_kind="point-mass" if model == "point-mass" else "forward-driver")

    def to_json(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> "RunSettings":
        if not isinstance(data, dict):
            raise ValueError("run settings must be a JSON object")
        retired_fields = {
            "profile_seconds", "profile_rounds", "spacing", "control_spacing",
            "line_seconds", "line_evaluations", "line_outer_passes",
            "line_optimizer_iterations", "duration", "replay_seconds",
            "maximum_step", "controller", "yaw_map", "mode", "plan",
            "planner", "path_plan",
        }
        retired = retired_fields.intersection(data)
        if data.get("solver_kind") == "yaw-aware":
            retired.add("solver_kind=yaw-aware")
        if retired:
            fields = ", ".join(sorted(retired))
            raise ValueError(f"retired yaw workflow setting(s): {fields}; choose forward-driver or point-mass")
        if (data.get("forward_physical_brake_planning") is True
                and data.get("forward_longitudinal_mode") == "hybrid-drive-brake-v1"):
            raise ValueError("physical forecast checkbox contradicts hybrid longitudinal mode")
        legacy_mode = None
        if "forward_longitudinal_mode" not in data and "forward_physical_brake_planning" in data:
            legacy_mode = "physical-reachable-v1" if data["forward_physical_brake_planning"] else "legacy"
        data = dict(data)
        if legacy_mode is not None:
            data["forward_longitudinal_mode"] = legacy_mode
        unknown = set(data) - set(cls.__dataclass_fields__)
        if unknown:
            raise ValueError(f"unknown run setting(s): {', '.join(sorted(unknown))}")
        integer_fields = {"forward_speed_points"}
        boolean_fields = {"forward_physical_brake_planning", "forward_transient_corner_planning"}
        float_fields = {
            "point_mass_duration", "point_mass_spacing", "point_mass_maximum_speed",
            "point_mass_pedal_travel_time", "point_mass_preview_seconds", "point_mass_heading_gain",
            "point_mass_lateral_gain", "point_mass_rtol", "point_mass_lateral_acceleration",
            "forward_duration", "forward_wall_seconds", "forward_maximum_speed", "forward_spacing",
            "forward_deceleration_margin", "forward_feedforward_distance",
        }
        string_fields = {"solver_kind", "point_mass_method", "point_mass_record_precision", "forward_longitudinal_mode"}
        for name, value in data.items():
            if name in boolean_fields and not isinstance(value, bool):
                raise ValueError(f"{name} must be a boolean")
            if name in integer_fields and (isinstance(value, bool) or not isinstance(value, int)):
                raise ValueError(f"{name} must be an integer")
            if name in float_fields and value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))):
                raise ValueError(f"{name} must be numeric" + (" or null" if name == "point_mass_lateral_acceleration" else ""))
            if name in string_fields and not isinstance(value, str):
                raise ValueError(f"{name} must be a string")
        enums = {
            "solver_kind": {"point-mass", "forward-driver"},
            "point_mass_method": {"BDF", "LSODA", "RK45", "DOP853", "Radau"},
            "point_mass_record_precision": {"float32", "float64"},
            "forward_longitudinal_mode": {"historical-preview-v1", "legacy", "physical-reachable-v1", "hybrid-drive-brake-v1"},
        }
        for name, choices in enums.items():
            if name in data and data[name] not in choices:
                raise ValueError(f"unsupported {name}: {data[name]}")
        try:
            settings = cls(**data)
        except TypeError as error:
            raise ValueError(str(error)) from error
        for name in float_fields & data.keys():
            value = getattr(settings, name)
            if value is not None and not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        return settings


def forward_driver_config(settings: RunSettings) -> dict[str, Any]:
    """Return the fully resolved forward-policy calibration and solver settings."""
    mode = settings.forward_longitudinal_mode
    if settings.forward_physical_brake_planning:
        if mode == "hybrid-drive-brake-v1":
            raise ValueError("physical forecast checkbox contradicts hybrid longitudinal mode")
        mode = "physical-reachable-v1"
    return {
        "policy": "hybrid-longitudinal-policy-v5-coast-neutral-event-rollout" if mode == "hybrid-drive-brake-v1" else "historical-preview-v1" if mode == "historical-preview-v1" else "continuous-braking-envelope-v1", "speed_gain": 2.0,
        "longitudinal_mode": mode,
        "physical_brake_forecast_version": "operational-diagnostic-v1" if mode == "physical-reachable-v1" else None,
        "physical_brake_forecast_horizon_s": 3.0 if mode == "physical-reachable-v1" else None,
        "physical_brake_authority_source": "current-state-model-context" if mode == "physical-reachable-v1" else None,
        "hybrid_setup_version": HYBRID_SETUP_VERSION if mode == "hybrid-drive-brake-v1" else None,
        "hybrid_horizon_s": 3.0 if mode == "hybrid-drive-brake-v1" else None,
        "hybrid_slabs": 30 if mode == "hybrid-drive-brake-v1" else None,
        "transient_corner_planning": settings.forward_transient_corner_planning,
        "throttle_acceleration_scale": 2.5, "brake_acceleration_scale": 6.0,
        "response_allowance_s": 0.06, "positive_acceleration_reserve_mps2": 4.0,
        "feedforward_distance_m": settings.forward_feedforward_distance,
        "lateral_margin": 0.9, "initial_speed_mps": 6.0, "initial_pedal": 0.0,
        "preview_seconds": 0.15, "heading_gain": 1.5, "lateral_gain": 1.5,
        "steering_time_constant_s": 0.08, "pedal_time_constant_s": 0.02,
        "throttle_apply_rate": 4.0, "throttle_release_rate": 6.0,
        "brake_apply_rate": 5.0, "brake_release_rate": 6.0,
        "method": "Radau", "rtol": 1e-6, "atol": 1e-8, "maximum_step_s": 0.02,
        "record_precision": "float32",
    }


def solution_model_for(settings: RunSettings, selected_model: str) -> str:
    """Resolve source to solver representation without compiling or dispatching."""
    if selected_model not in MODEL_NAMES:
        raise ValueError(f"unsupported GUI model: {selected_model}")
    if settings.solver_kind == "point-mass":
        return "point-mass"
    if settings.solver_kind == "forward-driver":
        if selected_model == "point-mass":
            raise ValueError("forward-driver mode requires a 7DoF/14DoF source; point-mass is unsupported")
        return selected_model
    raise ValueError(f"unsupported solver kind: {settings.solver_kind}")


def validate(settings: RunSettings, model: str, track) -> None:
    """Validate selected source and workflow-specific numeric settings."""
    del track
    solution_model_for(settings, model)
    if settings.solver_kind == "point-mass":
        numeric = ("point_mass_duration", "point_mass_spacing", "point_mass_maximum_speed",
                   "point_mass_pedal_travel_time", "point_mass_preview_seconds",
                   "point_mass_heading_gain", "point_mass_lateral_gain", "point_mass_rtol")
        for name in numeric:
            value = getattr(settings, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if settings.point_mass_lateral_acceleration is not None:
            value = settings.point_mass_lateral_acceleration
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError("point_mass_lateral_acceleration must be finite and positive")
        if settings.point_mass_method not in {"BDF", "LSODA", "RK45", "DOP853", "Radau"}:
            raise ValueError(f"unsupported point-mass integration method: {settings.point_mass_method}")
        if settings.point_mass_record_precision not in {"float32", "float64"}:
            raise ValueError("point_mass_record_precision must be float32 or float64")
        return
    if not isinstance(settings.forward_physical_brake_planning, bool):
        raise ValueError("forward_physical_brake_planning must be a boolean")
    if not isinstance(settings.forward_transient_corner_planning, bool):
        raise ValueError("forward_transient_corner_planning must be a boolean")
    if settings.forward_physical_brake_planning and settings.forward_longitudinal_mode == "hybrid-drive-brake-v1":
        raise ValueError("physical forecast checkbox contradicts hybrid longitudinal mode")
    if settings.forward_longitudinal_mode not in {"historical-preview-v1", "legacy", "physical-reachable-v1", "hybrid-drive-brake-v1"}:
        raise ValueError("unsupported forward_longitudinal_mode")
    mode = forward_driver_config(settings)["longitudinal_mode"]
    if mode == "physical-reachable-v1" and model != "14dof-simple":
        raise ValueError("physical brake forecasting requires the full vertical 14DoF plant")
    if mode == "historical-preview-v1" and settings.forward_transient_corner_planning:
        raise ValueError("historical preview cannot use envelope corner restrictions; select legacy or physical mode")
    if settings.forward_longitudinal_mode == "hybrid-drive-brake-v1":
        if settings.forward_physical_brake_planning:
            raise ValueError("hybrid mode cannot be combined with physical forecast checkbox")
        if model != "14dof-simple":
            raise ValueError("hybrid-drive-brake-v1 requires the full vertical 14DoF plant")
        if settings.forward_transient_corner_planning:
            raise ValueError("hybrid mode does not support transient yaw/TV planning")
    elif settings.forward_transient_corner_planning and settings.forward_longitudinal_mode == "hybrid-drive-brake-v1":
        raise ValueError("hybrid mode does not support transient yaw/TV planning")
    for name in ("forward_duration", "forward_wall_seconds", "forward_maximum_speed", "forward_spacing",
                 "forward_deceleration_margin", "forward_feedforward_distance"):
        value = getattr(settings, name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if settings.forward_deceleration_margin > 1:
        raise ValueError("forward_deceleration_margin must be at most 1")
    if isinstance(settings.forward_speed_points, bool) or not isinstance(settings.forward_speed_points, int) or settings.forward_speed_points < 2:
        raise ValueError("forward_speed_points must be an integer >= 2")
    if settings.forward_spacing > 0.5:
        raise ValueError("forward_spacing must be <= 0.5 m")
    if settings.forward_wall_seconds > 600:
        raise ValueError("forward wall budget must be <= 600 s")
    if settings.forward_wall_seconds <= settings.forward_duration:
        raise ValueError("forward wall budget must exceed simulated duration")
