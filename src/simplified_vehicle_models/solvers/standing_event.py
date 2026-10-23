"""Frozen-path standing launch plus a rolling fixed-path remainder profile.

This is a composition on an unchanged analytic trajectory, not a joint line
optimization, global minimum, or event-qualified physical validation.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType

import numpy as np

from simplified_vehicle_models.solvers.standing_launch import LaunchResult, LaunchSettings, integrate_standing_launch
from simplified_vehicle_models.tracks.types import PathCoordinates


class _RemainderStationMap:
    __slots__ = ("_original", "_origin", "_end", "reference_start", "reference_end",
                 "path_start", "path_end", "is_closed")

    def __init__(self, original, origin, end):
        self._original, self._origin, self._end = original, float(origin), float(end)
        self.path_start, self.path_end, self.is_closed = 0.0, float(end-origin), False
        self.reference_start = float(self.to_reference(0.0))
        self.reference_end = float(self.to_reference(self.path_end))

    def to_reference(self, local):
        values = np.asarray(local, dtype=float)
        if not np.all(np.isfinite(values)) or np.any(values < 0.) or np.any(values > self.path_end):
            raise ValueError("remainder arclength outside frozen remainder domain")
        result = np.asarray(self._original.to_reference(self._origin + values), dtype=float)
        return float(result) if values.ndim == 0 else result

    def to_path(self, reference):
        values = np.asarray(reference, dtype=float)
        if not np.all(np.isfinite(values)) or np.any(values < self.reference_start) or np.any(values > self.reference_end):
            raise ValueError("reference station outside frozen remainder domain")
        result = np.asarray(self._original.to_path(values), dtype=float) - self._origin
        if np.any(result < -1.e-10) or np.any(result > self.path_end + 1.e-10):
            raise ValueError("reference station maps outside frozen remainder domain")
        result = np.clip(result, 0., self.path_end)
        return float(result) if values.ndim == 0 else result


class FrozenPathRemainder:
    """Read-only local arclength view of the exact original trajectory remainder."""
    __slots__ = ("trajectory", "launch", "original_path_origin", "station_map", "path_length", "_locked")

    def __init__(self, trajectory, launch: LaunchResult):
        object.__setattr__(self, "_locked", False)
        sm = getattr(trajectory, "station_map", None)
        if sm is None or not callable(getattr(sm, "to_reference", None)) or not callable(getattr(sm, "to_path", None)):
            raise ValueError("trajectory requires a bidirectional station_map")
        lo, hi = float(sm.path_start), float(sm.path_end)
        length = hi-lo
        if not math.isfinite(length) or length <= 0 or not isinstance(launch, LaunchResult):
            raise ValueError("invalid original trajectory length or launch result")
        values = (launch.original_path_length, launch.transition_distance, launch.transition_reference_station,
                  launch.transition_speed, launch.remaining_path_length, launch.transition_time)
        if not all(math.isfinite(float(x)) for x in values) or launch.transition_speed <= 0 or launch.transition_time <= 0:
            raise ValueError("launch transition fields must be finite and positive where required")
        if not math.isclose(launch.original_path_length, length, rel_tol=1e-9, abs_tol=1e-8):
            raise ValueError("launch original length does not match trajectory")
        origin = lo + launch.transition_distance
        if not lo < origin < hi or not math.isclose(launch.remaining_path_length, hi-origin, rel_tol=1e-9, abs_tol=1e-8):
            raise ValueError("launch transition does not leave the declared remainder")
        ref = float(sm.to_reference(origin))
        if not math.isclose(ref, launch.transition_reference_station, rel_tol=1e-9, abs_tol=1e-8):
            raise ValueError("launch transition reference station does not match trajectory")
        object.__setattr__(self, "trajectory", trajectory)
        object.__setattr__(self, "launch", launch)
        object.__setattr__(self, "original_path_origin", origin)
        object.__setattr__(self, "station_map", _RemainderStationMap(sm, origin, hi))
        object.__setattr__(self, "path_length", hi-origin)
        object.__setattr__(self, "_locked", True)

    def __setattr__(self, name, value):
        if getattr(self, "_locked", False):
            raise AttributeError("FrozenPathRemainder is read-only")
        object.__setattr__(self, name, value)

    def evaluate(self, reference_station):
        ref = np.asarray(reference_station, dtype=float)
        if not np.all(np.isfinite(ref)) or np.any(ref < self.station_map.reference_start) or np.any(ref > self.station_map.reference_end):
            raise ValueError("reference station outside frozen remainder domain")
        return self.trajectory.evaluate(reference_station)

    def to_path(self, step=1.0):
        if (isinstance(step, (bool, np.bool_)) or not isinstance(step, (int, float, np.number))
                or not math.isfinite(float(step)) or float(step) <= 0.):
            raise ValueError("step must be a finite positive real number")
        step = float(step)
        distance = np.r_[np.arange(0., self.path_length, step), self.path_length]
        refs = np.asarray(self.station_map.to_reference(distance), dtype=float)
        values = self.evaluate(refs)
        heading = np.unwrap(np.asarray(values.heading, dtype=float))
        return PathCoordinates(distance, np.asarray(values.north, dtype=float), np.asarray(values.east, dtype=float),
                               heading, np.asarray(values.curvature, dtype=float), False, refs)


@dataclass(frozen=True, slots=True)
class FrozenStandingResult:
    launch: LaunchResult
    remainder_profile: object
    full_path_length: float
    launch_distance: float
    remainder_distance: float
    total_time: float
    path_distance: np.ndarray
    speed: np.ndarray
    reference_station: np.ndarray
    diagnostics: object
    frozen_path_only: bool = True
    accepted: bool = False
    physically_validated: bool = False
    event_qualified: bool = False
    joint_line_optimized: bool = False

    def __post_init__(self):
        for key in ("path_distance", "speed", "reference_station"):
            a = np.array(getattr(self, key), dtype=float, copy=True)
            a.setflags(write=False)
            object.__setattr__(self, key, a)
        object.__setattr__(self, "diagnostics", MappingProxyType(dict(self.diagnostics)))


def solve_frozen_standing_profile(vehicle, trajectory, surface, *, launch_settings=None, profile_step=1.,
                                  maximum_profile_iterations=200, maximum_profile_seconds=60.,
                                  acceleration_limit=None, profile_solver=None):
    """Integrate original-model launch, then profile only the open remainder."""
    try:
        if (isinstance(profile_step, (bool, np.bool_)) or not isinstance(profile_step, (int, float, np.number))
                or not math.isfinite(float(profile_step)) or float(profile_step) <= 0.):
            raise ValueError("profile_step must be a finite positive real number")
        step = float(profile_step)
        floor = float(surface.speed_floor)
        if not math.isfinite(floor) or floor < 0:
            raise ValueError("surface.speed_floor must be finite and nonnegative")
        settings = LaunchSettings() if launch_settings is None else launch_settings
        if not isinstance(settings, LaunchSettings) or settings.transition_speed <= floor:
            raise ValueError("launch transition_speed must exceed surface.speed_floor")
    except Exception as error:
        return None, {"phase": "settings", "error": f"{type(error).__name__}: {error}"}
    try:
        launch = integrate_standing_launch(vehicle, trajectory, settings=settings,
                                           capability_speed_floor=max(floor, np.finfo(float).tiny),
                                           acceleration_limit=acceleration_limit)
    except Exception as error:
        return None, {"phase": "launch", "error": f"{type(error).__name__}: {error}"}
    try:
        remainder = FrozenPathRemainder(trajectory, launch)
        path = remainder.to_path(step)
        solver = profile_solver
        if solver is None:
            from simplified_vehicle_models.solvers.fixed_path_energy_nlp import fixed_path_energy_profile
            solver = fixed_path_energy_profile
        profile = solver(path, surface, boundary="rolling", start_speed=launch.transition_speed,
                         trajectory=remainder, step=step, maximum_iterations=maximum_profile_iterations,
                         maximum_wall_seconds=maximum_profile_seconds)
        if not bool(profile.converged):
            raise ValueError("fixed-path remainder profile did not converge")
        distance = np.asarray(profile.distance, dtype=float)
        speed = np.asarray(profile.speed, dtype=float)
        refs = np.asarray(profile.reference_distance, dtype=float)
        expected_refs = np.asarray(remainder.station_map.to_reference(distance), dtype=float)
        if (distance.ndim != 1 or distance.size < 2 or speed.shape != distance.shape or refs.shape != distance.shape
                or not np.all(np.isfinite(distance)) or not np.all(np.isfinite(speed)) or not np.all(np.isfinite(refs))
                or np.any(speed <= 0.) or np.any(np.diff(distance) <= 0.)
                or not np.allclose(refs, expected_refs, rtol=1e-10, atol=1e-8)
                or not math.isclose(distance[0], 0., rel_tol=0., abs_tol=1e-8)
                or not math.isclose(distance[-1], remainder.path_length, rel_tol=1e-9, abs_tol=1e-7)
                or abs(speed[0] - launch.transition_speed) > 1e-8
                or not math.isclose(refs[0], launch.transition_reference_station, rel_tol=1e-9, abs_tol=1e-8)):
            raise ValueError(f"remainder profile has invalid arrays, coverage, or launch join: distance=({distance[0] if distance.size else None}, {distance[-1] if distance.size else None}) expected={remainder.path_length}; speed0={speed[0] if speed.size else None} expected={launch.transition_speed}; ref0={refs[0] if refs.size else None} expected={launch.transition_reference_station}")
        if not math.isfinite(float(profile.lap_time)) or profile.lap_time <= 0:
            raise ValueError("remainder profile time must be finite and positive")
    except Exception as error:
        return None, {"phase": "remainder", "error": f"{type(error).__name__}: {error}",
                      "launch_only": False, "frozen_path_only": True, "joint_line_optimized": False,
                      "accepted": False, "event_qualified": False}
    original_path_start = float(launch.path_distance[0])
    combined_d = np.r_[launch.path_distance - original_path_start, launch.transition_distance + distance[1:]]
    combined_v = np.r_[launch.speed, speed[1:]]
    combined_ref = np.r_[launch.reference_station, refs[1:]]
    total = launch.transition_time + float(profile.lap_time)
    diagnostics = {"phase": "complete", "launch_only": False, "frozen_path_only": True,
                   "joint_line_optimized": False, "accepted": False, "physically_validated": False,
                   "event_qualified": False, "launch_time": launch.transition_time,
                   "remainder_time": float(profile.lap_time), "total_time": total}
    return FrozenStandingResult(launch, profile, launch.original_path_length,
                                launch.transition_distance, remainder.path_length, total,
                                combined_d, combined_v, combined_ref, diagnostics), diagnostics


__all__ = ["FrozenPathRemainder", "FrozenStandingResult", "solve_frozen_standing_profile"]
