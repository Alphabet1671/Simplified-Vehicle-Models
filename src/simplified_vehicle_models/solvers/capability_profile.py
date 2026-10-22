"""Independent fixed-path capability-profile reference (no torque vectoring).

This module is deliberately separate from the production lap solver.  It solves
on prescribed path arclength using a sampled capability table; it is a numerical
reference, not an empirical validation of the original vehicle plant.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Callable

import numpy as np


@dataclass(frozen=True, slots=True)
class CapabilityProfile:
    """An immutable sampled fixed-path speed reference."""
    distance: np.ndarray
    speed: np.ndarray
    elapsed_time: np.ndarray
    lap_time: float
    converged: bool
    status: str
    iterations: int
    max_change: float
    reference_distance: np.ndarray | None = None
    diagnostics: dict | None = None

    def __post_init__(self) -> None:
        for name in ("distance", "speed", "elapsed_time", "reference_distance"):
            value = getattr(self, name)
            if value is not None:
                value = np.array(value, dtype=float, copy=True)
                value.setflags(write=False)
                object.__setattr__(self, name, value)
        object.__setattr__(self, "diagnostics", MappingProxyType(dict(self.diagnostics or {})))


def _finite_positive(value, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def _strict_linear(axis, values, query: float, name: str) -> float:
    axis = np.asarray(axis, dtype=float)
    values = np.asarray(values, dtype=float)
    if not math.isfinite(query) or query < axis[0] or query > axis[-1]:
        raise ValueError(f"{name} query outside supported physical domain")
    return float(np.interp(query, axis, values))


def _legacy_limits(surface, speed: float, curvature: float) -> tuple[float, float]:
    """Bilinear legacy-table query without endpoint clipping/extrapolation."""
    speeds = np.asarray(surface.speed, dtype=float)
    curves = np.asarray(surface.curvature, dtype=float)
    if speed < speeds[0] or speed > speeds[-1] or curvature < curves[0] or curvature > curves[-1]:
        raise ValueError("capability query outside supported physical domain")
    def bilinear(values):
        values = np.asarray(values, dtype=float)
        # np.interp in curvature, then speed, is bilinear on a rectilinear grid.
        by_speed = np.asarray([np.interp(curvature, curves, row) for row in values])
        return float(np.interp(speed, speeds, by_speed))
    return bilinear(surface.acceleration_min), bilinear(surface.acceleration_max)


def _surface_interface(surface):
    """Return strict cap/limit functions and the demonstrated speed floor."""
    if hasattr(surface, "domain_speed") and hasattr(surface, "speed_floor"):
        def cap(k):
            return float(surface.domain_speed(float(k)))
        def limits(v, k):
            return tuple(float(x) for x in surface.limits(float(v), float(k)))
        return cap, limits, float(surface.speed_floor)
    required = ("speed", "curvature", "speed_ceiling", "acceleration_min", "acceleration_max")
    if not all(hasattr(surface, name) for name in required):
        raise TypeError("surface must be CapabilitySurface or SignedTransientCapability")
    def cap(k):
        return _strict_linear(surface.curvature, surface.speed_ceiling, float(k), "capability")
    return cap, lambda v, k: _legacy_limits(surface, v, k), float(np.asarray(surface.speed, dtype=float)[0])


def _path_samples(path, step: float, trajectory):
    for name in ("distance", "curvature", "is_closed"):
        if not hasattr(path, name):
            raise TypeError("path must provide distance, curvature, and is_closed")
    raw_distance = np.asarray(path.distance, dtype=float)
    raw_curvature = np.asarray(path.curvature, dtype=float)
    if raw_distance.ndim != 1 or raw_curvature.shape != raw_distance.shape or raw_distance.size < 2:
        raise ValueError("path distance and curvature must be matching one-dimensional arrays")
    if not np.all(np.isfinite(raw_distance)) or not np.all(np.isfinite(raw_curvature)) or np.any(np.diff(raw_distance) <= 0.0):
        raise ValueError("path samples must be finite and strictly increasing")
    distance = np.r_[np.arange(raw_distance[0], raw_distance[-1], step), raw_distance[-1]]
    if distance[0] != raw_distance[0]:
        distance = np.r_[raw_distance[0], distance]
    distance = np.unique(distance)
    reference = getattr(path, "reference_distance", None)
    if reference is not None:
        reference = np.asarray(reference, dtype=float)
        if reference.shape != raw_distance.shape or np.any(np.diff(reference) <= 0.0):
            raise ValueError("path reference_distance must match a strictly increasing path grid")
        mapped = np.interp(distance, raw_distance, reference)
    else:
        mapped = None
    if trajectory is None:
        curvature = np.interp(distance, raw_distance, raw_curvature)
    else:
        station_map = getattr(trajectory, "station_map", None)
        if station_map is None:
            if mapped is None:
                raise ValueError("trajectory curvature requires explicit path reference_distance")
        else:
            # The authoritative analytic map, rather than interpolation through a
            # sampled path table, establishes ell -> source-station correspondence.
            try:
                mapped = np.asarray(station_map.to_reference(distance), dtype=float)
            except ValueError as error:
                raise ValueError("sampled path arclength is outside trajectory station-map coverage") from error
        values = trajectory.evaluate(mapped)
        curvature = np.asarray(values.curvature, dtype=float)
        if curvature.shape != distance.shape or not np.all(np.isfinite(curvature)):
            raise ValueError("trajectory did not return finite curvature at every path station")
    return distance, curvature, mapped


def _closed_duplicate(path, curvature: np.ndarray) -> bool:
    if not bool(path.is_closed) or not np.isclose(curvature[0], curvature[-1], rtol=0., atol=1.e-10):
        return False
    # PathCoordinates carries positions; use them where supplied to reject an
    # open table accidentally labelled closed.
    if hasattr(path, "x_north") and hasattr(path, "y_east"):
        x, y = np.asarray(path.x_north), np.asarray(path.y_east)
        return bool(np.isclose(x[0], x[-1]) and np.isclose(y[0], y[-1]))
    return True


def capability_profile(path, surface, *, boundary: str = "flying", start_speed=None,
                       step: float = .25, tolerance: float = 1.e-7,
                       maximum_iterations: int = 200, rest_limits: Callable | None = None,
                       trajectory=None) -> CapabilityProfile:
    """Solve a bounded forward/backward energy profile on exact path arclength.

    ``rest_limits(speed, curvature)`` is an explicit original-physics low-speed
    extension returning ``(minimum_acceleration, maximum_acceleration)``.  It is
    required for standing starts below a table's demonstrated speed floor.
    """
    step = _finite_positive(step, "step")
    tolerance = _finite_positive(tolerance, "tolerance")
    if isinstance(maximum_iterations, bool) or not isinstance(maximum_iterations, (int, np.integer)) or maximum_iterations < 1:
        raise ValueError("maximum_iterations must be a positive integer")
    if boundary not in ("flying", "rolling", "standing"):
        raise ValueError("boundary must be 'flying', 'rolling', or 'standing'")
    distance, curvature, reference = _path_samples(path, step, trajectory)
    cap, chart_limits, floor = _surface_interface(surface)
    if boundary == "flying":
        if start_speed is not None:
            raise ValueError("flying boundary does not accept start_speed")
        if not _closed_duplicate(path, curvature):
            raise ValueError("flying boundary requires a closed path with an equal duplicated end")
    else:
        if bool(path.is_closed):
            raise ValueError("rolling and standing boundaries require an open path")
        if boundary == "rolling":
            if start_speed is None:
                raise ValueError("rolling boundary requires start_speed")
            start_speed = float(start_speed)
            if not math.isfinite(start_speed) or start_speed <= 0.0:
                raise ValueError("rolling start_speed must be finite and positive")
        else:
            if start_speed not in (None, 0, 0.0):
                raise ValueError("standing boundary has an exact zero start speed")
            if rest_limits is None:
                raise ValueError("standing start below chart floor requires rest_limits")
            if not callable(rest_limits):
                raise TypeError("rest_limits must be callable")
    try:
        caps = np.asarray([cap(k) for k in curvature], dtype=float)
    except ValueError as error:
        raise ValueError("path curvature is outside the declared capability domain") from error
    # A segment is queried at its midpoint curvature, so its endpoints must
    # already carry that admissible capability cap; otherwise a corner cap can
    # be crossed without a dynamically feasible braking preview.
    midpoint_caps = np.asarray([cap(.5*(curvature[i] + curvature[i+1]))
                                for i in range(curvature.size - 1)], dtype=float)
    caps[:-1] = np.minimum(caps[:-1], midpoint_caps)
    caps[1:] = np.minimum(caps[1:], midpoint_caps)
    if np.any(~np.isfinite(caps)) or np.any(caps <= 0.0):
        raise ValueError("capability speed caps must be finite and positive")
    if boundary == "rolling" and start_speed > caps[0] + tolerance:
        raise ValueError("rolling start_speed exceeds the initial capability domain")
    if boundary == "rolling" and start_speed < floor:
        raise ValueError("rolling start_speed is below the demonstrated capability domain")
    if boundary == "standing":
        # A rest extension must connect to the chart at every encountered
        # curvature rather than create an unverified discontinuity at the floor.
        for k in np.unique(curvature):
            lo, hi = rest_limits(floor, float(k))
            clo, chi = chart_limits(floor, float(k))
            if not all(math.isfinite(float(x)) for x in (lo, hi)) or lo > hi:
                raise ValueError("rest_limits returned invalid acceleration limits")
            if abs(float(lo)-clo) > 1.e-6 or abs(float(hi)-chi) > 1.e-6:
                raise ValueError("rest_limits must continuously match chart limits at its floor")
    speed = caps.copy()
    if boundary == "rolling":
        speed[0] = start_speed
    elif boundary == "standing":
        speed[0] = 0.0
    else:
        speed[-1] = speed[0] = min(speed[0], speed[-1])

    def limits(v, k):
        if v < floor:
            if boundary != "standing" or rest_limits is None:
                raise ValueError("capability query below demonstrated speed floor")
            answer = rest_limits(float(v), float(k))
            if len(answer) != 2:
                raise ValueError("rest_limits must return two acceleration limits")
            lo, hi = (float(answer[0]), float(answer[1]))
            if not math.isfinite(lo) or not math.isfinite(hi) or lo > hi:
                raise ValueError("rest_limits returned invalid acceleration limits")
            return lo, hi
        return chart_limits(float(v), float(k))

    def failed(status, iteration, change, detail):
        return CapabilityProfile(distance, speed, np.full(distance.shape, np.nan), math.nan,
                                 False, status, iteration, change, reference,
                                 {"detail": detail, "boundary": boundary, "floor": floor})

    segment = np.diff(distance)
    max_change = math.inf
    for iteration in range(1, maximum_iterations + 1):
        previous = speed.copy()
        try:
            # Forward transition: decreasing fixed point from the current cap.
            for i, ds in enumerate(segment):
                vi, candidate = float(speed[i]), float(speed[i+1])
                k = .5*(curvature[i] + curvature[i+1])
                for _ in range(32):
                    vm = .5*(vi + candidate)
                    _, amax = limits(vm, k)
                    radicand = vi*vi + 2.*amax*ds
                    if radicand < -tolerance:
                        raise RuntimeError("forward transition has no real energy state")
                    updated = min(candidate, math.sqrt(max(0., radicand)))
                    if abs(updated-candidate) <= tolerance:
                        candidate = updated; break
                    candidate = updated
                else:
                    return failed("transition-failed", iteration, max_change, "forward fixed point did not converge")
                speed[i+1] = candidate
            if boundary == "flying":
                speed[0] = speed[-1] = min(speed[0], speed[-1])
            elif boundary == "rolling": speed[0] = start_speed
            else: speed[0] = 0.0
            # Backward braking preview propagates future caps dynamically.
            for i in range(len(segment)-1, -1, -1):
                vj, candidate = float(speed[i+1]), float(speed[i])
                k = .5*(curvature[i] + curvature[i+1])
                for _ in range(32):
                    vm = .5*(candidate + vj)
                    amin, _ = limits(vm, k)
                    radicand = vj*vj - 2.*amin*segment[i]
                    if radicand < -tolerance:
                        raise RuntimeError("backward transition has no real energy state")
                    updated = min(candidate, math.sqrt(max(0., radicand)))
                    if abs(updated-candidate) <= tolerance:
                        candidate = updated; break
                    candidate = updated
                else:
                    return failed("transition-failed", iteration, max_change, "backward fixed point did not converge")
                speed[i] = candidate
        except (ValueError, RuntimeError) as error:
            return failed("transition-domain-failed", iteration, max_change, str(error))
        if boundary == "flying": speed[0] = speed[-1] = min(speed[0], speed[-1])
        elif boundary == "rolling": speed[0] = start_speed
        else: speed[0] = 0.
        max_change = float(np.max(np.abs(speed-previous)))
        if max_change <= tolerance:
            break
    else:
        return failed("not-converged", maximum_iterations, max_change, "outer profile sweeps exhausted")

    # Verify every achieved segment against the same signed physical limits.
    for i, ds in enumerate(segment):
        vi, vj = float(speed[i]), float(speed[i+1])
        if vi + vj <= 0.0:
            return failed("launch-failed", iteration, max_change, "first segment has no positive speed")
        a = (vj*vj-vi*vi)/(2.*ds)
        lo, hi = limits(.5*(vi+vj), .5*(curvature[i]+curvature[i+1]))
        if a < lo-tolerance or a > hi+tolerance:
            return failed("dynamically-infeasible", iteration, max_change, "segment acceleration outside physical limits")
    elapsed = np.r_[0., np.cumsum(2.*segment/(speed[:-1]+speed[1:]))]
    return CapabilityProfile(distance, speed, elapsed, float(elapsed[-1]), True, "converged",
                             iteration, max_change, reference,
                             {"boundary": boundary, "floor": floor, "reference_kind": "capability-table"})


def matched_profile_replay(trajectory, profile: CapabilityProfile):
    """Convert a converged profile to a replay sample with explicit station map."""
    if profile.reference_distance is None:
        raise ValueError("matched profile replay requires explicit reference_distance")
    from simplified_vehicle_models.solvers.closed_loop_line import ReplaySample
    return ReplaySample(profile.distance, profile.speed, profile.converged, profile.lap_time,
                        reference_distance=profile.reference_distance)


__all__ = ["CapabilityProfile", "capability_profile", "matched_profile_replay"]
