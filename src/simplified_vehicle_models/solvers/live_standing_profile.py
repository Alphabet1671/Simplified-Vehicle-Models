"""Bounded same-original-physics standing profile on a frozen open path.

This is a sampled forward/backward energy reference, not a global or
continuous-time feasibility certificate.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numbers
import time

import numpy as np

from simplified_vehicle_models.formulations.point_mass import (
    maximum_acceleration_limit,
    maximum_corner_speed,
    maximum_deceleration_limit,
    maximum_vehicle_speed,
)
from simplified_vehicle_models.solvers.capability_profile import capability_profile


@dataclass(frozen=True, slots=True)
class LiveStandingSettings:
    step: float = 0.5
    tolerance: float = 1.0e-7
    maximum_iterations: int = 50
    maximum_wall_seconds: float = 60.0
    maximum_queries: int = 20000

    def __post_init__(self):
        for name in ("step", "tolerance", "maximum_wall_seconds"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, numbers.Real) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be a finite positive real")
        for name in ("maximum_iterations", "maximum_queries"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, numbers.Integral) or value < 1:
                raise ValueError(f"{name} must be a positive integer")


class _LiveSurface:
    speed_floor = 0.0

    def __init__(self, vehicle, settings, limit_evaluator, speed_cap_evaluator, started):
        self.vehicle = vehicle
        self.settings = settings
        self._limit_callback = limit_evaluator
        self._cap_callback = speed_cap_evaluator
        self.started = started
        self.query_count = 0
        self.cache_hits = 0
        self.limit_cache = {}
        self.cap_cache = {}

    def _check_wall(self):
        if time.monotonic() - self.started > self.settings.maximum_wall_seconds:
            raise TimeoutError("live standing profile wall-time budget exhausted")

    def ensure_wall_budget(self):
        self._check_wall()

    def _query(self, cache, key, callback, *args):
        self._check_wall()
        if key in cache:
            self.cache_hits += 1
            return cache[key]
        if self.query_count >= self.settings.maximum_queries:
            raise RuntimeError("live standing profile query budget exhausted")
        self.query_count += 1
        try:
            value = callback(*args)
        finally:
            self._check_wall()
        return value

    @staticmethod
    def _real(value, name):
        if isinstance(value, (bool, np.bool_)) or not isinstance(value, numbers.Real):
            raise ValueError(f"{name} must be a real non-boolean number")
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    def domain_speed(self, curvature):
        k = self._real(curvature, "curvature")
        value = self._real(self._query(self.cap_cache, k, self._cap_callback, self.vehicle, k),
                           "live speed cap")
        if value <= 0:
            raise ValueError("live speed cap must be positive")
        self.cap_cache[k] = value
        return value

    def limits(self, speed, curvature):
        v = self._real(speed, "speed")
        k = self._real(curvature, "curvature")
        if v < 0:
            raise ValueError("live limits require nonnegative speed")
        key = (v, k)
        answer = self._query(self.limit_cache, key, self._limit_callback, self.vehicle, v, k)
        if not isinstance(answer, (tuple, list)) or len(answer) != 2:
            raise ValueError("live limit callback must return two real values")
        amin = self._real(answer[0], "minimum acceleration")
        amax = self._real(answer[1], "maximum acceleration")
        if amin > amax:
            raise ValueError("live acceleration limits must be ordered")
        if v > self.domain_speed(k):
            raise ValueError("live limit query exceeds same-curvature speed cap")
        self.limit_cache[key] = (amin, amax)
        return amin, amax


def _original_limits(vehicle, speed, curvature):
    return (-maximum_deceleration_limit(vehicle, speed, curvature),
            maximum_acceleration_limit(vehicle, speed, curvature))


def _original_cap(vehicle, curvature):
    top = maximum_vehicle_speed(vehicle)
    if curvature == 0.0:
        return top
    return maximum_corner_speed(vehicle, 1.0 / abs(curvature), maximum_speed=top)


def _finite_array(value):
    return value is not None and np.asarray(value).ndim == 1 and np.all(np.isfinite(value))


def _json_error(error):
    return {"type": type(error).__name__, "message": str(error)}


def solve_live_standing_profile(vehicle, trajectory, *, settings=None,
                                limit_evaluator=None, speed_cap_evaluator=None):
    """Return ``(profile_or_none, JSON-safe report)`` for one exact open path."""
    settings = LiveStandingSettings() if settings is None else settings
    report = {
        "method": "original_live_limits_full_course_energy_sweep",
        "same_physics_all_speeds": True, "chart_queried": False,
        "no_transition_join": True, "frozen_path_only": True,
        "joint_line_optimized": False, "all_accepted": False,
        "event_qualified": False, "numerical_qualified": False,
        "U3_qualified": False, "physical_qualified": False,
        "accepted": False, "numerically_validated": False,
        "physically_validated": False, "event_validated": False,
        "queries": 0, "cache_hits": 0, "elapsed_wall_seconds": 0.0,
        "phase": "input_validation", "status": "failed",
    }
    start = time.monotonic()
    surface = None
    try:
        if not isinstance(settings, LiveStandingSettings):
            raise TypeError("settings must be LiveStandingSettings")
        reference = getattr(trajectory, "reference", None)
        station_map = getattr(trajectory, "station_map", None)
        if reference is None or bool(getattr(reference, "is_closed", True)):
            raise ValueError("trajectory must have an open reference")
        if not callable(getattr(trajectory, "to_path", None)) or station_map is None:
            raise ValueError("trajectory must provide exact to_path and station_map")
        source = np.asarray(getattr(reference, "source_station", None), dtype=float)
        if source.ndim != 1 or source.size < 2 or not np.all(np.isfinite(source)) or np.any(np.diff(source) <= 0):
            raise ValueError("reference source station coverage must be finite and strictly increasing")
        fullpath = trajectory.to_path(source)
        if bool(getattr(fullpath, "is_closed", True)):
            raise ValueError("trajectory path must remain open")
        distance = np.asarray(fullpath.distance, dtype=float)
        if distance.ndim != 1 or distance.size < 2 or not np.all(np.isfinite(distance)) or np.any(np.diff(distance) <= 0):
            raise ValueError("trajectory path distance must be finite and strictly increasing")
        if not math.isclose(float(distance[0]), float(station_map.path_start), rel_tol=0, abs_tol=1e-9):
            raise ValueError("trajectory path must begin at exact map origin")
        if not math.isclose(float(distance[-1]), float(station_map.path_end), rel_tol=0, abs_tol=1e-7):
            raise ValueError("trajectory path must cover exact full map length")
        limit_evaluator = _original_limits if limit_evaluator is None else limit_evaluator
        speed_cap_evaluator = _original_cap if speed_cap_evaluator is None else speed_cap_evaluator
        if not callable(limit_evaluator) or not callable(speed_cap_evaluator):
            raise TypeError("live evaluator overrides must be callable")
        surface = _LiveSurface(vehicle, settings, limit_evaluator, speed_cap_evaluator, start)
        report["phase"] = "profile"
        profile = capability_profile(fullpath, surface, boundary="standing", step=settings.step,
                                     tolerance=settings.tolerance, maximum_iterations=settings.maximum_iterations,
                                     rest_limits=surface.limits, trajectory=trajectory)
        surface.ensure_wall_budget()
        change = float(profile.max_change)
        report.update({"iterations": int(profile.iterations),
                       "max_change": change if math.isfinite(change) else None,
                       "profile_status": profile.status})
        if not profile.converged:
            report["phase"] = "profile_failed"
            report["error"] = _json_error(RuntimeError(str(profile.diagnostics.get("detail", profile.status))))
            return None, report
        arrays = (profile.distance, profile.speed, profile.elapsed_time, profile.reference_distance)
        if any(not _finite_array(item) for item in arrays):
            raise ValueError("converged profile arrays must be finite one-dimensional arrays")
        if any(np.asarray(item).shape != profile.distance.shape for item in arrays):
            raise ValueError("profile arrays must be aligned")
        if np.any(np.diff(profile.distance) <= 0):
            raise ValueError("profile distance must be strictly increasing")
        if not math.isclose(float(profile.distance[0]), 0., abs_tol=1e-9) or not math.isclose(float(profile.distance[-1]), float(distance[-1]), abs_tol=1e-7):
            raise ValueError("profile must cover exact path from zero to full length")
        expected_ref = np.asarray(station_map.to_reference(profile.distance), dtype=float)
        if not np.allclose(profile.reference_distance, expected_ref, rtol=0, atol=1e-7):
            raise ValueError("profile source map does not match exact station map")
        if profile.speed[0] != 0 or np.any(profile.speed[1:] <= 0) or profile.elapsed_time[0] != 0 or np.any(np.diff(profile.elapsed_time) <= 0):
            raise ValueError("profile must start at rest and have positive increasing speed/time")
        if not math.isclose(float(profile.lap_time), float(profile.elapsed_time[-1]), rel_tol=1e-12, abs_tol=1e-10):
            raise ValueError("profile lap_time must equal final elapsed time")
        segment = np.diff(profile.distance)
        elapsed = 2*segment/(profile.speed[:-1]+profile.speed[1:])
        if not np.allclose(np.diff(profile.elapsed_time), elapsed, rtol=1e-10, atol=1e-10):
            raise ValueError("profile elapsed time violates exact energy-segment formula")
        report.update({"phase": "complete", "status": "converged", "all_accepted": False,
                       "profile_returned": True, "path_length": float(distance[-1]), "lap_time": float(profile.lap_time),
                       "sample_count": int(profile.distance.size), "accepted": False})
        return profile, report
    except Exception as error:
        report["phase"] = "budget_exhausted" if isinstance(error, (TimeoutError,)) or "budget exhausted" in str(error) else report["phase"]
        report["error"] = _json_error(error)
        return None, report
    finally:
        report["elapsed_wall_seconds"] = float(time.monotonic() - start)
        if surface is not None:
            report["queries"] = int(surface.query_count)
            report["cache_hits"] = int(surface.cache_hits)


__all__ = ["LiveStandingSettings", "solve_live_standing_profile"]
