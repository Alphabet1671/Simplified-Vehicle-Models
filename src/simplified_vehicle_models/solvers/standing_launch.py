"""Original-model time-domain launch on a frozen analytic trajectory.

This component stops at the transition speed; it is not a joint standing-start
minimum-time solution for the course.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from types import MappingProxyType
from typing import Callable

import numpy as np


@dataclass(frozen=True, slots=True)
class LaunchSettings:
    transition_speed: float = 1.0
    maximum_time: float = 10.0
    maximum_step: float = 0.02
    relative_tolerance: float = 1e-8
    absolute_tolerance: float = 1e-10

    def __post_init__(self):
        for name in ("transition_speed", "maximum_time", "maximum_step", "relative_tolerance", "absolute_tolerance"):
            value = getattr(self, name)
            if (isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number))
                    or not math.isfinite(float(value)) or float(value) <= 0):
                raise ValueError(f"{name} must be a finite positive real number")
            object.__setattr__(self, name, float(value))


@dataclass(frozen=True, slots=True)
class LaunchResult:
    time: np.ndarray
    path_distance: np.ndarray
    reference_station: np.ndarray
    speed: np.ndarray
    acceleration: np.ndarray
    transition_time: float
    transition_distance: float
    transition_reference_station: float
    transition_speed: float
    original_path_length: float
    remaining_path_length: float
    diagnostics: dict = field(default_factory=dict)
    launch_only: bool = True
    event_qualified: bool = False

    def __post_init__(self):
        arrays = []
        for name in ("time", "path_distance", "reference_station", "speed", "acceleration"):
            arr = np.array(getattr(self, name), dtype=float, copy=True)
            if arr.ndim != 1 or not np.all(np.isfinite(arr)):
                raise ValueError(f"{name} must be a finite one-dimensional array")
            arrays.append(arr)
            arr.setflags(write=False)
            object.__setattr__(self, name, arr)
        if not arrays[0].size or any(a.shape != arrays[0].shape for a in arrays[1:]):
            raise ValueError("launch result arrays must be nonempty and have matching shapes")
        if np.any(np.diff(arrays[0]) <= 0) or np.any(np.diff(arrays[1]) < 0) or np.any(arrays[3] < 0):
            raise ValueError("launch time must increase and distance/speed must be nondecreasing/nonnegative")
        object.__setattr__(self, "diagnostics", MappingProxyType(dict(self.diagnostics)))


def _positive(value, name):
    if (isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number))
            or not math.isfinite(float(value)) or float(value) <= 0):
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


def integrate_standing_launch(vehicle, trajectory, *, settings=None, capability_speed_floor=0.5,
                              acceleration_limit: Callable | None = None) -> LaunchResult:
    """Integrate ds/dt=v, dv/dt=a(v,k(s)) on the original frozen path.

    An explicit acceleration callback has signature ``(vehicle, speed,
    curvature) -> acceleration``.  Otherwise the original point-mass positive
    acceleration limit is queried at every RHS evaluation.
    """
    settings = LaunchSettings() if settings is None else settings
    if not isinstance(settings, LaunchSettings):
        raise TypeError("settings must be LaunchSettings")
    floor = _positive(capability_speed_floor, "capability_speed_floor")
    if settings.transition_speed <= floor:
        raise ValueError("transition_speed must exceed capability_speed_floor")
    station_map = getattr(trajectory, "station_map", None)
    if station_map is None or not callable(getattr(station_map, "to_reference", None)):
        raise ValueError("trajectory requires station_map.to_reference")
    lo, hi = float(station_map.path_start), float(station_map.path_end)
    length = hi - lo
    if not math.isfinite(length) or length <= 0:
        raise ValueError("trajectory path must have positive finite length")
    start_ref = float(station_map.to_reference(lo))
    if not math.isfinite(start_ref):
        raise ValueError("trajectory start reference station must be finite")
    curvature_fn = getattr(trajectory, "evaluate", None)
    if not callable(curvature_fn):
        raise ValueError("trajectory must provide analytic evaluate(reference_station)")
    if acceleration_limit is None:
        from simplified_vehicle_models.formulations.point_mass import maximum_acceleration_limit
        limit = lambda v, k: maximum_acceleration_limit(vehicle, v, k)
    else:
        if not callable(acceleration_limit):
            raise TypeError("acceleration_limit must be callable")
        limit = lambda v, k: acceleration_limit(vehicle, v, k)

    def query(distance, speed):
        # The ODE solver may project internal stages barely past the finish;
        # do not wrap closed maps or evaluate analytic geometry outside coverage.
        if distance < lo - 1e-12 or distance > hi + 1e-12:
            raise RuntimeError("integration stage left the physical course domain")
        d = min(max(distance, lo), hi)
        ref = float(station_map.to_reference(d))
        if not math.isfinite(ref) or not math.isfinite(speed) or speed < 0:
            raise RuntimeError("launch state or mapped reference station is non-finite/invalid")
        geometry = curvature_fn(ref)
        k = float(geometry.curvature)
        if not math.isfinite(k):
            raise RuntimeError("analytic trajectory returned non-finite curvature")
        a = float(limit(speed, k))
        if not math.isfinite(a) or a <= 0:
            raise RuntimeError("maximum acceleration limit must remain finite and positive")
        return ref, k, a

    from scipy.integrate import solve_ivp
    def rhs(t, y):
        _, _, a = query(float(y[0]), float(y[1]))
        return (float(y[1]), a)
    def transition(t, y): return float(y[1]) - settings.transition_speed
    transition.terminal = True
    transition.direction = 1
    def finish(t, y): return hi - float(y[0])
    finish.terminal = True
    finish.direction = -1
    solution = solve_ivp(rhs, (0.0, settings.maximum_time), (lo, 0.0), method="RK45",
                         rtol=settings.relative_tolerance, atol=settings.absolute_tolerance,
                         max_step=settings.maximum_step, events=(transition, finish), dense_output=False)
    if not solution.success:
        raise RuntimeError(f"launch integration failed: {solution.message}")
    tev = solution.t_events[0]
    fev = solution.t_events[1]
    if len(tev) == 0:
        if len(fev):
            raise RuntimeError("course finish reached before transition speed")
        raise RuntimeError("maximum_time reached before transition speed")
    tt = float(tev[0]); ell = float(solution.y_events[0][0, 0]); speed = float(solution.y_events[0][0, 1])
    if ell >= hi:
        raise RuntimeError("transition reached with no remaining course")
    times = np.r_[solution.t[solution.t < tt], tt]
    distances = np.r_[solution.y[0, solution.t < tt], ell]
    speeds = np.r_[solution.y[1, solution.t < tt], speed]
    refs, accels = [], []
    for d, v in zip(distances, speeds):
        ref, _, a = query(float(d), float(v)); refs.append(ref); accels.append(a)
    return LaunchResult(times, distances, np.asarray(refs), speeds, np.asarray(accels), tt, ell-lo,
                        float(station_map.to_reference(ell)), speed, length, hi-ell,
                        {"message": solution.message, "launch_only": True,
                         "event_qualified": False, "joint_standing_min_time": False})
