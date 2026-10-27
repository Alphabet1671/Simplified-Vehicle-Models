"""Serializable settings and model resolution for the GUI solver workflows."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from pathlib import Path
from typing import Any, Literal

MODEL_NAMES = frozenset({"point-mass", "planar-7dof", "14dof-simple"})
SolverKind = Literal["point-mass", "yaw-aware"]


@dataclass(frozen=True, slots=True)
class RunSettings:
    """Settings are explicitly partitioned by point-mass or yaw-aware solver."""
    solver_kind: SolverKind = "yaw-aware"
    yaw_map: str = ""
    mode: Literal["fixed", "line", "replay"] = "fixed"
    plan: str | None = None
    spacing: float = 2.0
    control_spacing: float = 40.0
    profile_seconds: float = 900.0
    profile_rounds: int = 16
    line_seconds: float = 900.0
    line_evaluations: int = 24
    line_outer_passes: int = 3
    line_optimizer_iterations: int = 8
    duration: float = 150.0
    replay_seconds: float = 1200.0
    maximum_step: float = 0.05
    controller: Literal["preview", "model-aware"] = "model-aware"
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

    @classmethod
    def for_model(cls, model: str) -> "RunSettings":
        if model not in MODEL_NAMES:
            raise ValueError(f"unsupported GUI model: {model}")
        return cls()

    def to_json(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> "RunSettings":
        if not isinstance(data, dict):
            raise ValueError("run settings must be a JSON object")
        unknown = set(data) - set(cls.__dataclass_fields__)
        if unknown:
            raise ValueError(f"unknown run setting(s): {', '.join(sorted(unknown))}")
        try:
            return cls(**data)
        except TypeError as error:
            raise ValueError(str(error)) from error


def solution_model_for(settings: RunSettings, selected_model: str) -> str:
    """Resolve source to solver representation without compiling or dispatching.

    Point-mass ignores yaw capability and consumes a point-mass view whether
    selected directly or adapted from either full model. Yaw-aware uses the
    planar 7DoF solution, adapting a 14DoF source to that form.
    """
    if selected_model not in MODEL_NAMES:
        raise ValueError(f"unsupported GUI model: {selected_model}")
    if settings.solver_kind == "point-mass":
        return "point-mass"
    if settings.solver_kind == "yaw-aware":
        if selected_model == "point-mass":
            raise ValueError("yaw-aware mode requires a 7DoF/14DoF source; point-mass is unsupported")
        return "planar-7dof"
    raise ValueError(f"unsupported solver kind: {settings.solver_kind}")


def _validate_map(path: Path) -> None:
    import json
    import numpy as np
    from simplified_vehicle_models.formulations.yaw_map_artifact import SCHEMA, load_yaw_map

    if not path.is_file():
        raise ValueError(f"completed yaw-map artifact does not exist: {path}")
    try:
        with np.load(path, allow_pickle=False) as archive:
            metadata = json.loads(str(archive["metadata"].item()))
        if metadata.get("schema") != SCHEMA or metadata.get("report", {}).get("complete") is not True:
            raise ValueError("yaw-map artifact is not a completed supported artifact")
        capability, _ = load_yaw_map(path, expected_identity=metadata.get("build_identity"))
        if any(np.asarray(getattr(capability, axis)).size < 2
               for axis in ("speed", "longitudinal_acceleration", "lateral_acceleration")):
            raise ValueError("yaw-map axes are unsuitable for profile planning")
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid completed yaw-map artifact {path}: {error}") from error


def validate(settings: RunSettings, model: str, track: Path) -> None:
    """Validate selected source, settings partition, and required yaw artifacts."""
    del track  # Track parsing and solver dispatch belong to the run contract.
    solution_model_for(settings, model)
    if settings.solver_kind == "point-mass":
        if settings.yaw_map or settings.mode != "fixed" or settings.plan is not None:
            raise ValueError("yaw map, mode, and plan settings do not apply to point-mass")
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
    if settings.solver_kind != "yaw-aware":
        raise ValueError(f"unsupported solver kind: {settings.solver_kind}")
    if settings.mode not in {"fixed", "line", "replay"}:
        raise ValueError(f"unsupported yaw-aware mode: {settings.mode}")
    if settings.controller not in {"preview", "model-aware"}:
        raise ValueError(f"unsupported controller: {settings.controller}")
    if settings.mode == "replay" and not settings.plan:
        raise ValueError("replay mode requires a saved plan")
    if settings.mode != "replay" and settings.plan:
        raise ValueError("plan is only accepted in replay mode")
    for name in ("spacing", "control_spacing", "profile_seconds", "line_seconds", "duration", "replay_seconds", "maximum_step"):
        value = getattr(settings, name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    for name in ("profile_rounds", "line_evaluations", "line_outer_passes", "line_optimizer_iterations"):
        value = getattr(settings, name)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError(f"{name} must be a positive integer")
    if not settings.yaw_map:
        raise ValueError("a completed yaw-map artifact is required")
    _validate_map(Path(settings.yaw_map))
    if settings.mode == "replay" and not Path(settings.plan).is_file():
        raise ValueError(f"replay plan does not exist: {settings.plan}")


def yaw_runner_argv(settings: RunSettings, track: Path) -> list[str]:
    """Map yaw-aware settings to the suite CLI; point-mass remains on old runner."""
    if settings.solver_kind != "yaw-aware":
        raise ValueError("yaw_runner_argv only accepts solver_kind='yaw-aware'")
    values = (
        ("--yaw-map", settings.yaw_map), ("--track", str(track)), ("--mode", settings.mode),
        ("--spacing", repr(settings.spacing)), ("--control-spacing", repr(settings.control_spacing)),
        ("--profile-seconds", repr(settings.profile_seconds)), ("--profile-rounds", str(settings.profile_rounds)),
        ("--line-seconds", repr(settings.line_seconds)), ("--line-evaluations", str(settings.line_evaluations)),
        ("--line-outer-passes", str(settings.line_outer_passes)),
        ("--line-optimizer-iterations", str(settings.line_optimizer_iterations)),
        ("--duration", repr(settings.duration)), ("--replay-seconds", repr(settings.replay_seconds)),
        ("--maximum-step", repr(settings.maximum_step)), ("--controller", settings.controller),
    )
    argv = [item for pair in values for item in pair]
    if settings.plan:
        argv.extend(("--plan", settings.plan))
    return argv
