"""Diagnostic exact-reference trials for serialized trajectories.

This executes a numerical driver replay, not an OCP or physical qualification.
Callers must impose an outer hard timeout: the internal wall budget is soft and
includes artifact loading and setup but cannot preempt setup itself.
"""
from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from time import perf_counter

import numpy as np

from simplified_vehicle_models.solvers.trajectory_artifact import (
    LoadedTrajectory, ExactTrajectoryDriverReference, load_trajectory_artifact,
)


def _positive_real(name, value, *, allow_zero=False):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number)):
        raise ValueError(f"{name} must be a finite real scalar")
    value = float(value)
    if not np.isfinite(value) or (value < 0 if allow_zero else value <= 0):
        raise ValueError(f"{name} must be finite and {'nonnegative' if allow_zero else 'positive'}")
    return value


@dataclass(frozen=True, slots=True)
class FrozenReplaySettings:
    duration_seconds: float = 120.
    maximum_wall_seconds: float = 90.
    maximum_step_seconds: float = .02
    rtol: float = 1e-6
    atol: float = 1e-8
    preview_seconds: float = .15

    def __post_init__(self):
        for name in ("duration_seconds", "maximum_wall_seconds", "maximum_step_seconds", "rtol", "atol"):
            object.__setattr__(self, name, _positive_real(name, getattr(self, name)))
        object.__setattr__(self, "preview_seconds", _positive_real("preview_seconds", self.preview_seconds, allow_zero=True))
        if self.maximum_wall_seconds > 300.:
            raise ValueError("maximum_wall_seconds exceeds the forward solver ceiling")


class EndpointHoldReferenceConsumer:
    """Explicit open-course hold consumer; never mutates stored trajectory data."""
    consumer_endpoint_extension = "hold_reference_only"

    def __init__(self, reference):
        self._reference = reference
        self.length = reference.length
        self.is_closed = reference.is_closed
        self.distance = reference.distance
        self.speed = reference.speed
        self.curvature = reference.curvature
        self.acceleration = reference.acceleration
        self.outside_query_count = 0
        self.maximum_outside_distance = 0.
        self.first_outside_before = None
        self.first_outside_after = None

    def sample(self, distance):
        if isinstance(distance, (bool, np.bool_)) or not isinstance(distance, (int, float, np.number)):
            raise ValueError("distance must be a finite real scalar")
        d = float(distance)
        if not np.isfinite(d):
            raise ValueError("distance must be a finite real scalar")
        if self.is_closed:
            return self._reference.sample(d)
        bounded = float(np.clip(d, 0., self.length))
        if d != bounded:
            self.outside_query_count += 1
            self.maximum_outside_distance = max(self.maximum_outside_distance, abs(d - bounded))
            if d < 0 and self.first_outside_before is None:
                self.first_outside_before = d
            if d > self.length and self.first_outside_after is None:
                self.first_outside_after = d
        return self._reference.sample(bounded)


@dataclass(frozen=True, slots=True)
class ForwardReplayEvidence:
    time: np.ndarray
    state: np.ndarray
    reference_speed_at_state_distance: np.ndarray
    metrics: object
    report: object


def _readonly(value):
    result = np.array(value, dtype=float, copy=True)
    result.setflags(write=False)
    return result


def replay_exact_trajectory(loaded_or_path, vehicle, *, settings=None, plant_factory=None,
                            driver_factory=None, forward_solver=None):
    """Return (diagnostic evidence or None, fail-closed report)."""
    start = perf_counter()
    report = {"accepted": False, "physically_validated": False, "event_qualified": False,
              "numerically_validated": False, "U3_qualified": False,
              "completed": False, "phase": "setup", "consumer_endpoint_extension": "hold_reference_only"}
    evidence = None
    try:
        settings = FrozenReplaySettings() if settings is None else settings
        if not isinstance(settings, FrozenReplaySettings):
            raise TypeError("settings must be FrozenReplaySettings")
        if isinstance(loaded_or_path, LoadedTrajectory):
            loaded = loaded_or_path
        else:
            loaded = load_trajectory_artifact(loaded_or_path)
        reference = EndpointHoldReferenceConsumer(ExactTrajectoryDriverReference(loaded))
        from simplified_vehicle_models.solvers.forward_driver import PreviewDriver
        from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
        from simplified_vehicle_models.solvers.forward_driver import solve_forward
        plant_factory = PlanarForwardPlant if plant_factory is None else plant_factory
        driver_factory = PreviewDriver if driver_factory is None else driver_factory
        forward_solver = solve_forward if forward_solver is None else forward_solver
        for name in ("steering_maximum_angle", "steering_maximum_rate", "maximum_pedal_force", "wheelbase"):
            value = _positive_real(name, getattr(vehicle, name))
            report[name] = value
        initial_speed = float(reference.speed[0])
        if initial_speed <= (1. if reference.is_closed else 0.):
            report.update(phase="unsupported_standing_start", error="reference initial speed is outside planar plant domain")
            return None, report
        report["event_mode"] = "flying" if reference.is_closed else ("rolling" if initial_speed > 0. else "standing")
        elapsed = perf_counter() - start
        remaining = settings.maximum_wall_seconds - elapsed
        if remaining <= 0:
            report.update(phase="wall_budget_before_forward", wall_overrun=True,
                          elapsed_wall_seconds=elapsed)
            return None, report
        report["phase"] = "plant_setup"
        plant = plant_factory(vehicle, reference)
        driver = driver_factory(reference, vehicle.wheelbase, vehicle.steering_maximum_angle,
                                vehicle.steering_maximum_rate, vehicle.maximum_pedal_force,
                                preview_seconds=settings.preview_seconds)
        state0 = plant.initial_state(initial_speed)
        remaining = settings.maximum_wall_seconds - (perf_counter() - start)
        if remaining <= 0:
            report.update(phase="wall_budget_before_forward", wall_overrun=True,
                          elapsed_wall_seconds=perf_counter() - start)
            return None, report
        report["phase"] = "forward"
        result = forward_solver(plant, driver, state0, duration=settings.duration_seconds,
            wall_seconds=remaining, maximum_step=settings.maximum_step_seconds,
            rtol=settings.rtol, atol=settings.atol, maximum_wall_seconds=300.)
        t = np.asarray(result.time, dtype=float)
        y = np.asarray(result.state, dtype=float)
        names = tuple(plant.state_names)
        required = ("s", "n", "heading_error", "v_x", "v_y")
        if len(names) != y.shape[0] or any(n not in names for n in required):
            raise ValueError("plant state layout is missing required named channels")
        if t.ndim != 1 or y.shape != (len(names), t.size) or t.size < 1 or not np.all(np.isfinite(t)) or not np.all(np.isfinite(y)) or np.any(np.diff(t) <= 0):
            raise ValueError("forward result time/state shape, monotonicity, or finiteness invalid")
        idx = {n: names.index(n) for n in required}
        s = y[idx["s"]]
        target = np.array([reference.sample(float(d))[0] for d in s])
        speed = np.hypot(y[idx["v_x"]], y[idx["v_y"]])
        error = speed - target
        nerr, herr = y[idx["n"]], y[idx["heading_error"]]
        length = reference.length
        solver_lap_completed = getattr(result, "lap_completed", False)
        if not isinstance(solver_lap_completed, (bool, np.bool_)):
            raise ValueError("forward lap_completed must be boolean")
        completed = bool(solver_lap_completed) and abs(s[-1] - length) <= max(1e-6, 1e-6 * length)
        report["solver_lap_completed"] = bool(solver_lap_completed)
        report["endpoint_distance_error_m"] = float(s[-1] - length)
        periodic = None
        if reference.is_closed:
            raw_indices = tuple(getattr(plant, "periodic_indices", ()))
            tolerances = np.asarray(getattr(plant, "periodic_tolerances", []), dtype=float)
            if (len(tolerances) != len(raw_indices)
                    or not np.all(np.isfinite(tolerances)) or np.any(tolerances <= 0)
                    or any(not isinstance(i, (int, np.integer)) for i in raw_indices)
                    or any(i < 0 or i >= len(names) for i in raw_indices)
                    or len(set(raw_indices)) != len(raw_indices)):
                raise ValueError("plant periodicity metadata is invalid")
            applicable = tuple((i, tolerances[j]) for j, i in enumerate(raw_indices)
                               if i != idx["s"] and names[i] != "soc")
            pidx = tuple(i for i, _ in applicable)
            periodic = ({names[i]: float(y[i, -1]-y[i, 0]) for i in pidx}
                        if pidx else None)
            limits = {names[i]: float(limit) for i, limit in applicable}
            metrics_periodic_within = (all(abs(periodic[name]) <= limits[name]
                                           for name in periodic) if periodic is not None else None)
        else:
            metrics_periodic_within = None
        max_lateral = float(np.max(np.abs(nerr)))
        maximum_spatial_step = float(np.max(np.diff(s))) if s.size > 1 else 0.
        tracking_gate = bool(max_lateral <= .25 and maximum_spatial_step <= .5)
        if loaded.physical_time is None:
            time_error_abs = time_error_rel = signed_time_error_rel = None
            signed_time_error = None
        else:
            physical_time = _positive_real("loaded physical_time", loaded.physical_time)
            signed_time_error = float(t[-1] - physical_time)
            time_error_abs = abs(signed_time_error)
            time_error_rel = float(time_error_abs / physical_time)
            signed_time_error_rel = float(signed_time_error / physical_time)
        solver_message = getattr(result, "message", None)
        metrics = {"max_abs_speed_error_mps": float(np.max(np.abs(error))), "rms_speed_error_mps": float(np.sqrt(np.mean(error**2))),
            "max_abs_lateral_error_m": max_lateral, "max_abs_heading_error_rad": float(np.max(np.abs(herr))),
            "maximum_spatial_step_m": maximum_spatial_step,
            "diagnostic_tracking_within_existing_gate": tracking_gate,
            "distance_coverage_m": float(s[-1]), "distance_overshoot_m": float(max(0., s[-1]-length)),
            "time_vs_reference_seconds_diagnostic_only": time_error_abs,
            "time_vs_reference_relative_diagnostic_only": time_error_rel,
            "time_vs_reference_signed_seconds_diagnostic_only": signed_time_error,
            "time_difference_absolute_seconds": time_error_abs,
            "time_difference_signed_relative": signed_time_error_rel,
            "forward_message": solver_message,
            "periodic_state_delta": periodic, "periodic_within_published_tolerances": metrics_periodic_within,
            "endpoint_query_count": reference.outside_query_count,
            "endpoint_max_outside_distance_m": reference.maximum_outside_distance,
            "endpoint_first_query_before_m": reference.first_outside_before,
            "endpoint_first_query_after_m": reference.first_outside_after,
            "midpoint_sample_validation_complete": bool(getattr(result, "midpoint_sample_validation_complete", False)),
            "evaluations": getattr(result, "evaluations", None), "forward_wall_seconds": getattr(result, "wall_seconds", None)}
        wall_overrun = perf_counter() - start > settings.maximum_wall_seconds
        if wall_overrun:
            completed = False
        report.update(phase="complete" if completed else "incomplete", completed=completed,
                      wall_overrun=wall_overrun,
                      midpoint_sample_validation_complete=metrics["midpoint_sample_validation_complete"])
        evidence = ForwardReplayEvidence(_readonly(t), _readonly(y), _readonly(target),
                                         MappingProxyType(metrics), MappingProxyType(report.copy()))
        return evidence, report
    except Exception as exc:
        report.update(phase=report.get("phase", "setup"), error=f"{type(exc).__name__}: {exc}",
                      elapsed_wall_seconds=perf_counter()-start)
        return evidence, report
