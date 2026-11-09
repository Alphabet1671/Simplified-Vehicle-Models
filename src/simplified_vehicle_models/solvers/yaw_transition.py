"""Sampled quasi-static yaw-transition speed screen.

The screen uses r=v*kappa and, with zero longitudinal acceleration,
rdot=v**2*dkappa/ds. Its axle-force allocation is a conservative unsigned
moment balance for both yaw-acceleration signs. It is an operational sampled
screen, not a physical guarantee or certificate: loads are planar quasi-static,
tire capacity is sampled in pure lateral slip, and combined slip, tire/vehicle
transients, and actual yaw sign are not resolved.
"""
from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Callable, Mapping
import math

import numpy as np


@dataclass(frozen=True, slots=True)
class YawTransitionScreen:
    """Immutable screened caps and the sampled demand/capacity evidence."""
    caps: np.ndarray
    curvature_gradient: np.ndarray
    provenance: Mapping[str, Any]


def _array(value: Any, name: str, *, minimum: int = 2) -> np.ndarray:
    raw = np.asarray(value)
    if raw.dtype.kind not in "iuf":
        raise TypeError(f"{name} must contain real numeric values")
    result = np.array(raw, dtype=float, copy=True)
    if result.ndim != 1 or result.size < minimum or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite one-dimensional array of length >= {minimum}")
    return result


def yaw_transition_speed_caps(distance, curvature, vehicle, original_caps, *,
                              is_closed=False, force_authority: Callable | None = None,
                              bisection_iterations=18, lateral_samples=81):
    """Return immutable speed caps screening sampled lateral/yaw force demand.

    ``force_authority(speed, lateral_acceleration)`` may be supplied for tests
    or alternate sampled authority models and must return ``(front_N,rear_N)``.
    Otherwise the vehicle's planar normal loads and UniTire model are evaluated
    over ``lateral_samples`` pure-lateral slip-angle samples in [-0.5, 0.5] rad.
    The input caps are only lowered. Closed paths include the segment joining
    the penultimate station to the duplicated endpoint and duplicate output.
    """
    if not isinstance(is_closed, (bool, np.bool_)):
        raise TypeError("is_closed must be bool")
    if isinstance(bisection_iterations, bool) or not isinstance(bisection_iterations, int) or bisection_iterations < 1:
        raise ValueError("bisection_iterations must be a positive integer")
    if isinstance(lateral_samples, bool) or not isinstance(lateral_samples, int) or lateral_samples < 2:
        raise ValueError("lateral_samples must be an integer >= 2")
    if force_authority is not None and not callable(force_authority):
        raise TypeError("force_authority must be callable")
    x, k, caps = (_array(distance, "distance", minimum=3 if is_closed else 2),
                  _array(curvature, "curvature", minimum=3 if is_closed else 2),
                  _array(original_caps, "original_caps", minimum=3 if is_closed else 2))
    if len(x) != len(k) or len(x) != len(caps) or np.any(np.diff(x) <= 0) or np.any(caps < 0):
        raise ValueError("distance, curvature, and caps must match; distance increases and caps are nonnegative")
    if is_closed and (not math.isclose(k[0], k[-1], rel_tol=0, abs_tol=1e-10)
                      or not math.isclose(caps[0], caps[-1], rel_tol=0, abs_tol=1e-10)):
        raise ValueError("closed path curvature and cap endpoints must duplicate")
    for attr in ("total_mass", "wheelbase", "front_mass_fraction", "yaw_inertia"):
        value = getattr(vehicle, attr, None)
        if isinstance(value, bool) or not isinstance(value, (int, float, np.number)) or not math.isfinite(float(value)):
            raise TypeError(f"vehicle.{attr} must be finite numeric")
    mass, wheelbase, front_fraction = float(vehicle.total_mass), float(vehicle.wheelbase), float(vehicle.front_mass_fraction)
    inertia = float(vehicle.yaw_inertia)
    if mass <= 0 or wheelbase <= 0 or inertia <= 0 or not 0 < front_fraction < 1:
        raise ValueError("vehicle mass, wheelbase, yaw inertia, and mass fraction are invalid")
    slopes = np.diff(k) / np.diff(x)
    if is_closed:
        gradient = np.r_[max(abs(slopes[-1]), abs(slopes[0])),
                         np.maximum(np.abs(slopes[:-1]), np.abs(slopes[1:])),
                         max(abs(slopes[-1]), abs(slopes[0]))]
    else:
        gradient = np.r_[abs(slopes[0]), np.maximum(np.abs(slopes[:-1]), np.abs(slopes[1:])), abs(slopes[-1])]

    if force_authority is None:
        radius = float(vehicle.tire.r_1)
        if not math.isfinite(radius) or radius <= 0.:
            raise ValueError("tire free radius must be finite and positive")
        from simplified_vehicle_models.formulations.planar_7dof import evaluate_normal_loads
        from simplified_vehicle_models.tires.unitire_simple import evaluate_unitire
        alpha = np.linspace(-0.5, 0.5, lateral_samples)[:, None]
        def authority(speed, ay):
            loads = np.asarray(evaluate_normal_loads(vehicle, speed, 0.0, ay), dtype=float)
            if loads.shape != (4,) or not np.all(np.isfinite(loads)):
                raise ValueError("normal-load oracle must return four finite loads")
            if np.any(loads < 0.):
                # Two-wheel rollover is outside this authority model. Never
                # turn a negative-load tire evaluation into available grip.
                return 0., 0.
            result = evaluate_unitire(vehicle.tire, alpha, np.zeros((lateral_samples, 4)),
                                      loads[None, :], np.full((lateral_samples, 4), speed / vehicle.tire.r_1))
            per_wheel = np.max(np.abs(result.lateral_force), axis=0)
            return float(per_wheel[:2].sum()), float(per_wheel[2:].sum())
    else:
        authority = force_authority
    lf, lr = (1.0-front_fraction)*wheelbase, front_fraction*wheelbase
    out = caps.copy()
    moving = np.flatnonzero(gradient > 0)
    nonmonotone_stations = []
    for i in moving:
        def feasible(speed):
            ay = speed * speed * abs(float(k[i]))
            front, rear = authority(speed, ay)
            front, rear = float(front), float(rear)
            if not math.isfinite(front) or not math.isfinite(rear) or front < 0 or rear < 0:
                raise ValueError("force authority must return finite nonnegative front/rear capacities")
            total = mass * ay
            yaw_moment = inertia * speed * speed * float(gradient[i])
            required_front = (lr * total + yaw_moment) / wheelbase
            required_rear = (lf * total + yaw_moment) / wheelbase
            return required_front <= front and required_rear <= rear
        lo, hi = 0.0, float(caps[i])
        # Establish a sampled first-crossing bracket before bisection rather
        # than silently assuming an arbitrary speed-dependent oracle monotone.
        grid = np.linspace(0., hi, 9)
        flags = [feasible(float(speed)) for speed in grid]
        first_failure = next((j for j, flag in enumerate(flags) if not flag), None)
        if first_failure is None:
            out[i] = hi
            continue
        if first_failure == 0:
            out[i] = 0.
            continue
        if any(flags[first_failure + 1:]):
            nonmonotone_stations.append(int(i))
        lo, hi = float(grid[first_failure - 1]), float(grid[first_failure])
        for _ in range(bisection_iterations):
            speed = (lo + hi) * 0.5
            if feasible(speed):
                lo = speed
            else:
                hi = speed
        out[i] = lo
    if is_closed:
        out[-1] = out[0]
    if np.any(out > caps) or not np.all(np.isfinite(out)):
        raise RuntimeError("yaw screen produced invalid cap")
    out.setflags(write=False)
    gradient = np.array(gradient, copy=True); gradient.setflags(write=False)
    provenance = MappingProxyType({
        "method": "sampled-quasistatic-yaw-transition-screen",
        "bisection_iterations": bisection_iterations,
        "prefix_scan_samples": 9,
        "sampled_nonmonotone_station_indices": tuple(nonmonotone_stations),
        "lateral_samples": lateral_samples if force_authority is None else None,
        "authority_source": "sampled-pure-lateral-unitire" if force_authority is None else "injected-force-authority",
        "longitudinal_acceleration_assumed_m_s2": 0.0,
        "yaw_acceleration_model": "rdot=v^2*dkappa/ds; secant worst adjacent absolute curvature slope",
        "axle_demand": "both yaw signs; (lr*m*ay+I*rdot_abs)/L front and (lf*m*ay+I*rdot_abs)/L rear",
        "caveat": "Operational sampled screen, not a physical guarantee or certificate; neglects combined slip, transients, and actual yaw sign; force maxima are sampled and may be optimistic.",
    })
    return YawTransitionScreen(out, gradient, provenance)
