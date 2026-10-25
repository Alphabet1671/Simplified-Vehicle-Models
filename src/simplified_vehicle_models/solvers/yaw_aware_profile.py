"""Sampled yaw-aware fixed-path speed profiling.

This module consumes physical path arclength and a Phase-1 ``YawCapabilityMap``.
It is intentionally independent of the QSS OCP and of the trajectory geometry
implementation. Interpolation is conservative with respect to infeasible map
corners: no query is admitted if any corner with nonzero interpolation weight
is marked infeasible.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal, Mapping

import numpy as np

from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.solvers.forward_driver import SpeedPlan


@dataclass(frozen=True, slots=True)
class YawPath:
    """A sampled line on physical arclength, with signed curvature and rate."""
    distance: np.ndarray
    curvature: np.ndarray
    curvature_rate: np.ndarray
    is_closed: bool

    def __post_init__(self):
        d, k, kp = (np.array(v, dtype=float, copy=True) for v in
                    (self.distance, self.curvature, self.curvature_rate))
        if d.ndim != 1 or d.size < 3 or k.shape != d.shape or kp.shape != d.shape:
            raise ValueError("yaw path arrays must be matching one-dimensional arrays with at least three nodes")
        if not all(np.all(np.isfinite(v)) for v in (d, k, kp)):
            raise ValueError("yaw path arrays must be finite")
        if d[0] != 0.0 or np.any(np.diff(d) <= 0.0):
            raise ValueError("physical arclength must start at zero and increase strictly")
        if not isinstance(self.is_closed, (bool, np.bool_)):
            raise ValueError("is_closed must be boolean")
        if self.is_closed and not np.isclose(k[0], k[-1], rtol=0., atol=1e-8):
            raise ValueError("closed yaw path curvature must agree at duplicated seam")
        for name, value in (("distance", d), ("curvature", k), ("curvature_rate", kp)):
            value.setflags(write=False)
            object.__setattr__(self, name, value)
        object.__setattr__(self, "is_closed", bool(self.is_closed))


def curvature_rate_from_samples(distance, curvature, *, is_closed=False) -> np.ndarray:
    """Estimate dκ/dℓ on a physical-arclength grid.

    Closed inputs include a duplicated endpoint. Periodic centered differences
    use the actual nonuniform spacing on both sides of each node.
    """
    x, y = np.asarray(distance, float), np.asarray(curvature, float)
    if x.ndim != 1 or x.size < 3 or y.shape != x.shape or not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
        raise ValueError("distance and curvature must be matching finite vectors with at least three nodes")
    if x[0] != 0.0 or np.any(np.diff(x) <= 0.0):
        raise ValueError("distance must be physical arclength starting at zero")
    if is_closed:
        if not np.isclose(y[0], y[-1], rtol=0., atol=1e-8):
            raise ValueError("closed curvature samples must agree at seam")
        period = x[-1]
        unique_y = y[:-1]
        unique_x = x[:-1]
        if unique_y.size < 3:
            raise ValueError("closed path requires at least three unique samples")
        prev_x = np.r_[unique_x[-1] - period, unique_x[:-1]]
        next_x = np.r_[unique_x[1:], unique_x[0] + period]
        prev_y = np.roll(unique_y, 1)
        next_y = np.roll(unique_y, -1)
        hp, hn = unique_x - prev_x, next_x - unique_x
        deriv = (-hn / (hp * (hp + hn)) * prev_y
                 + (hn - hp) / (hp * hn) * unique_y
                 + hp / (hn * (hp + hn)) * next_y)
        return np.r_[deriv, deriv[0]]
    return np.gradient(y, x, edge_order=2)


class _MapInterpolator:
    """Strict trilinear map query; infeasible support is never interpolated through."""
    def __init__(self, capability: YawCapabilityMap):
        if not isinstance(capability, YawCapabilityMap):
            raise TypeError("capability must be a YawCapabilityMap")
        self.map = capability
        self.axes = tuple(np.asarray(v, dtype=float) for v in
                          (capability.speed, capability.longitudinal_acceleration,
                           capability.lateral_acceleration))
        self.fields = (np.asarray(capability.yaw_moment_into_turn),
                       np.asarray(capability.yaw_moment_out_of_turn))

    def query(self, speed, ax, ay):
        q = (float(speed), float(ax), float(ay))
        brackets = []
        for axis, value in zip(self.axes, q):
            if not math.isfinite(value) or value < axis[0] - 1e-12 or value > axis[-1] + 1e-12:
                return None
            value = min(float(axis[-1]), max(float(axis[0]), value))
            hi = int(np.searchsorted(axis, value, side="right"))
            if hi == 0:
                brackets.append(((0, 1.0),))
            elif hi >= axis.size:
                brackets.append(((axis.size - 1, 1.0),))
            else:
                lo = hi - 1
                f = (value - axis[lo]) / (axis[hi] - axis[lo])
                brackets.append(((lo, 1. - f), (hi, f)))
        into = out = 0.0
        mask = self.map.feasible_mask
        for i, wi in brackets[0]:
            for j, wj in brackets[1]:
                for k, wk in brackets[2]:
                    w = wi * wj * wk
                    if w <= 1e-14:
                        continue
                    if not mask[i, j, k]:
                        return None
                    into += w * self.fields[0][i, j, k]
                    out += w * self.fields[1][i, j, k]
        return into, out

    def yaw_feasible(self, speed, ax, curvature, curvature_rate, *, acceleration=None):
        ay = speed * speed * curvature if acceleration is None else float(acceleration)
        moments = self.query(speed, ax, ay)
        if moments is None:
            return False
        into, out = moments
        # Directional moments are relative to the turn. On a zero-curvature
        # point, use the curvature-rate sign for a deterministic limiting side.
        direction = math.copysign(1., curvature if abs(curvature) > 1e-12 else
                                  (curvature_rate if abs(curvature_rate) > 1e-12 else 1.))
        yaw_accel = ax * curvature + speed * speed * curvature_rate
        demand = direction * self.map.yaw_inertia * yaw_accel
        return -out - 1e-9 <= demand <= into + 1e-9


def _ax_candidates(interp, speed, curvature):
    ay = speed * speed * curvature
    axis = interp.axes[1]
    return [float(ax) for ax in axis if interp.query(speed, ax, ay) is not None]


def _max_speed_cap(interp, curvature, curvature_rate, *, maximum_speed, speed_samples=80):
    axis = interp.axes[0]
    hi = min(float(axis[-1]), float(maximum_speed))
    # A descending scan avoids assuming feasibility is monotonic in speed.
    candidates = np.linspace(hi, float(axis[0]), max(2, int(speed_samples)))
    for v in candidates:
        for ax in _ax_candidates(interp, float(v), curvature):
            if interp.yaw_feasible(float(v), ax, curvature, curvature_rate):
                return float(v)
    return None


def _segment_acceleration(interp, path, v0, v1, index, ds):
    vm = math.sqrt(max(0., .5 * (v0*v0 + v1*v1)))
    ax = (v1*v1 - v0*v0) / (2. * ds)
    k = .5 * (path.curvature[index] + path.curvature[index + 1])
    kp = .5 * (path.curvature_rate[index] + path.curvature_rate[index + 1])
    return interp.yaw_feasible(vm, ax, k, kp)


def _transition_candidates(interp, upper, fixed_speed, ds, *, forward, samples=80):
    """Descending endpoint-speed scan augmented by map-axis projections.

    A uniform speed scan can skip the narrow interval corresponding to a
    feasible longitudinal-acceleration cell when ``ds`` is small. Projecting
    each acceleration axis value through the energy equation guarantees that
    those physically meaningful transition candidates are sampled. The original
    descending scan remains in the set so nonmonotone feasibility is not assumed.
    """
    values = list(np.linspace(float(upper), 0., max(2, int(samples))))
    fixed_sq = float(fixed_speed) ** 2
    for ax in interp.axes[1]:
        endpoint_sq = fixed_sq + (2. * ds * ax if forward else -2. * ds * ax)
        if endpoint_sq > 0.:
            values.append(math.sqrt(endpoint_sq))
    # Add interval midpoints as well: map values may be feasible between axis
    # projections, and midpoint sampling improves resolution without monotonicity.
    projected = sorted(set(v for v in values if math.isfinite(v) and 0. < v <= upper), reverse=True)
    mids = [(a + b) * .5 for a, b in zip(projected[:-1], projected[1:])]
    return sorted(set(projected + mids), reverse=True)


def yaw_aware_profile(path: YawPath, capability: YawCapabilityMap, *,
                       boundary: Literal["flying", "standing"] = "flying",
                       start_speed: float | None = None, maximum_speed: float = 100.,
                       tolerance: float = 1e-5, maximum_iterations: int = 200,
                       speed_samples: int = 80) -> "YawAwareProfile":
    """Build a sampled yaw-aware profile using conservative feasibility scans.

    Segment transitions use the energy equation and require a yaw-map query at
    the segment midpoint. This is a sampled reference profile, not a continuous
    feasibility certificate.
    """
    if not isinstance(path, YawPath):
        raise TypeError("path must be a YawPath")
    if not isinstance(capability, YawCapabilityMap):
        raise TypeError("capability must be a YawCapabilityMap")
    if boundary not in ("flying", "standing"):
        raise ValueError("boundary must be 'flying' or 'standing'")
    if not math.isfinite(maximum_speed) or maximum_speed <= 0 or not math.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("maximum_speed and tolerance must be finite and positive")
    if isinstance(maximum_iterations, bool) or int(maximum_iterations) < 1:
        raise ValueError("maximum_iterations must be a positive integer")
    if isinstance(speed_samples, bool) or int(speed_samples) < 2:
        raise ValueError("speed_samples must be at least two")
    if boundary == "flying":
        if not path.is_closed or start_speed is not None:
            raise ValueError("flying boundary requires a closed path and no start_speed")
    else:
        if path.is_closed or start_speed not in (None, 0, 0.):
            raise ValueError("standing boundary requires an open path and exact zero start_speed")
        if capability.speed[0] > 0.0:
            raise ValueError("standing start requires a yaw map whose speed grid includes zero; no rest extension is defined")
    interp = _MapInterpolator(capability)
    caps = np.asarray([_max_speed_cap(interp, float(k), float(kp),
                        maximum_speed=maximum_speed, speed_samples=speed_samples)
                       for k, kp in zip(path.curvature, path.curvature_rate)], dtype=object)
    if any(v is None for v in caps):
        raise ValueError("yaw map has no feasible speed at one or more path stations")
    caps = np.asarray(caps, dtype=float)
    if path.is_closed:
        caps[0] = caps[-1] = min(caps[0], caps[-1])
    speed = caps.copy()
    if boundary == "standing":
        speed[0] = 0.
    converged = False
    change = math.inf
    for iteration in range(1, int(maximum_iterations) + 1):
        old = speed.copy()
        # Repeated forward/backward projections. Each transition is searched
        # over descending endpoint speeds, preserving safety with nonmonotone maps.
        for i, ds in enumerate(np.diff(path.distance)):
            upper = min(speed[i + 1], caps[i + 1])
            if speed[i] <= 0.:
                # Low-speed launch is outside most maps; find the smallest
                # representable positive next state and let the segment query decide.
                trial = upper
            else:
                trial = upper
            found = None
            for value in _transition_candidates(interp, trial, speed[i], float(ds), forward=True):
                if _segment_acceleration(interp, path, float(speed[i]), float(value), i, float(ds)):
                    found = float(value); break
            if found is None:
                raise ValueError(f"no forward-feasible transition on segment {i}")
            speed[i + 1] = min(speed[i + 1], found)
        if boundary == "flying":
            speed[0] = speed[-1] = min(speed[0], speed[-1])
        else:
            speed[0] = 0.
        minimum_index = 1 if boundary == "standing" else 0
        for i in range(len(speed) - 2, minimum_index - 1, -1):
            ds = float(path.distance[i + 1] - path.distance[i])
            upper = min(speed[i], caps[i])
            found = None
            for value in _transition_candidates(interp, upper, speed[i + 1], ds, forward=False):
                if _segment_acceleration(interp, path, float(value), float(speed[i + 1]), i, ds):
                    found = float(value); break
            if found is None:
                raise ValueError(f"no backward-feasible transition on segment {i}")
            speed[i] = min(speed[i], found)
        if boundary == "flying":
            speed[0] = speed[-1] = min(speed[0], speed[-1])
        else:
            speed[0] = 0.
        change = float(np.max(np.abs(speed - old)))
        if change <= tolerance:
            converged = True
            break
    ax = np.zeros_like(speed)
    ds = np.diff(path.distance)
    segment_ax = np.diff(speed * speed) / (2. * ds)
    ax[:-1] = segment_ax
    ax[-1] = segment_ax[-1]
    elapsed = np.r_[0., np.cumsum(2. * ds / np.maximum(speed[:-1] + speed[1:], 1e-12))]
    if not converged:
        raise RuntimeError("yaw-aware forward/backward profile did not converge")
    return YawAwareProfile(path.distance, speed, ax, elapsed, float(elapsed[-1]), True,
                           iteration, change, {"boundary": boundary,
                           "map_source_kind": capability.source_kind,
                           "map_source_identity": capability.source_identity,
                           "interpolation": "strict_trilinear_feasible_corners_v1",
                           "continuous_feasibility_certified": False})


@dataclass(frozen=True, slots=True)
class YawAwareProfile:
    distance: np.ndarray
    speed: np.ndarray
    longitudinal_acceleration: np.ndarray
    elapsed_time: np.ndarray
    lap_time: float
    converged: bool
    iterations: int
    maximum_speed_change: float
    diagnostics: Mapping[str, object]

    def __post_init__(self):
        arrays = {}
        for name in ("distance", "speed", "longitudinal_acceleration", "elapsed_time"):
            value = np.array(getattr(self, name), dtype=float, copy=True)
            if value.ndim != 1 or not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must be a finite vector")
            value.setflags(write=False)
            arrays[name] = value
            object.__setattr__(self, name, value)
        if len({value.shape for value in arrays.values()}) != 1:
            raise ValueError("profile arrays must have matching shapes")
        if arrays["distance"].size < 2 or arrays["distance"][0] != 0.0 or np.any(np.diff(arrays["distance"]) <= 0.0):
            raise ValueError("profile distance must start at zero and increase strictly")
        if np.any(arrays["speed"] < 0.0) or np.any(np.diff(arrays["elapsed_time"]) < 0.0):
            raise ValueError("profile speed must be nonnegative and elapsed time monotone")
        if not self.converged:
            raise ValueError("YawAwareProfile can only represent a converged profile")
        if not math.isfinite(self.lap_time) or self.lap_time <= 0:
            raise ValueError("lap_time must be finite and positive")
        from types import MappingProxyType
        object.__setattr__(self, "diagnostics", MappingProxyType(dict(self.diagnostics)))


def to_speed_plan(path: YawPath, profile: YawAwareProfile) -> SpeedPlan:
    """Project a converged physical-arclength yaw profile to tracker SpeedPlan."""
    if not isinstance(path, YawPath) or not isinstance(profile, YawAwareProfile):
        raise TypeError("path and profile must be YawPath and YawAwareProfile")
    if not profile.converged or not np.array_equal(path.distance, profile.distance):
        raise ValueError("a converged profile on the identical path grid is required")
    return SpeedPlan(path.distance, profile.speed, path.curvature, is_closed=path.is_closed)
