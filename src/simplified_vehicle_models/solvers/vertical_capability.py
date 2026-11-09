"""Bounded sampled steady-corner speed restrictions from the vertical model.

Successful samples provide operational derating evidence only. They do not
establish a physical maximum, nor certify feasibility between samples.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

import math
import numpy as np

from simplified_vehicle_models.solvers.vertical_equilibrium import (
    MINIMUM_STEADY_CORNER_SPEED, CONTINUATION_ACCEPTANCE, _solve_steady_corner_detail,
)


@dataclass(frozen=True)
class CornerSample:
    station: float
    curvature: float
    speed: float
    lateral_acceleration: float
    status: str  # "solved" or "unresolved"
    residual_inf: float | None = None
    error: str | None = None


@dataclass(frozen=True)
class VerticalCapResult:
    caps: np.ndarray
    samples: tuple[CornerSample, ...]
    reasons: tuple[str, ...]  # planar-cap-tested, restricted-to-tested-prefix, unresolved-retained
    provenance: str
    query_count: int
    max_queries: int
    caveat: str = ("A restricted value is only the highest consecutively successful tested speed; it is "
                    "an empirical operational restriction, not a physical maximum. Unresolved solves "
                    "are not proof of infeasibility. No unsampled speed/station or interpolation is certified.")


def constrain_corner_speeds(
    vehicle: Any,
    distance,
    curvature,
    planar_caps,
    *,
    max_queries: int = 128,
    samples_per_station: int = 5,
    is_closed: bool = False,
    solver: Callable | None = None,
) -> VerticalCapResult:
    """Return opt-in restrictions no greater than planar caps, from sampled solves.

    Queries use a bounded set of deterministic stations, prioritizing the
    tightest absolute-curvature stations and then the highest curvature peaks.
    Each station gets a speed grid from 3 m/s through its planar cap (or just
    that cap if ``samples_per_station=1``). Consecutively successful samples
    below the highest tested speed may operationally restrict that node. Such
    success never proves that a higher speed is infeasible. An unresolved sample
    stops the successful prefix; no prefix means retain the planar cap.

    For a closed path provide a duplicated endpoint: its cap and curvature must
    match the first node. A bounded dictionary cache avoids repeated exact
    vehicle/speed/ay calls. The model object is never modified.

    ``solver`` is a test seam, called as ``solver(vehicle, speed, ay)`` and
    returning an object with ``residual_inf``. Only expected numerical/model
    rejection exceptions are classified as unresolved; programmer errors escape.
    """
    x = _vector(distance, "distance")
    k = _vector(curvature, "curvature")
    cap = _vector(planar_caps, "planar_caps")
    if len(x) != len(k) or len(x) != len(cap) or len(x) == 0:
        raise ValueError("distance, curvature and planar_caps must have equal nonzero length")
    if np.any(np.diff(x) <= 0) or np.any(cap < 0):
        raise ValueError("distance must increase and planar caps must be nonnegative")
    if not isinstance(is_closed, (bool, np.bool_)):
        raise TypeError("is_closed must be bool")
    if is_closed and (len(x) < 3 or not math.isclose(cap[0], cap[-1], rel_tol=0, abs_tol=1e-10)
                      or not math.isclose(k[0], k[-1], rel_tol=0, abs_tol=1e-10)):
        raise ValueError("closed path requires duplicated endpoint cap and curvature")
    if isinstance(max_queries, bool) or not isinstance(max_queries, int) or max_queries < 0:
        raise ValueError("max_queries must be a nonnegative integer")
    if isinstance(samples_per_station, bool) or not isinstance(samples_per_station, int) or samples_per_station < 1:
        raise ValueError("samples_per_station must be a positive integer")
    oracle = solver if solver is not None else _solve_steady_corner_detail
    nstations = min(len(x), max_queries // samples_per_station)
    result = cap.copy()
    reasons = ["not-queried"] * len(x)
    if nstations == 0:
        result.setflags(write=False)
        return VerticalCapResult(result, (), tuple(reasons), "vertical_14dof_tested_speed_restriction_v1", 0, max_queries)

    # Deterministic severity selection; for a closed duplicated seam use the
    # first endpoint as the representative, while retaining both in outputs.
    candidates = np.arange(len(x) - 1 if is_closed else len(x))
    order = candidates[np.argsort(-np.abs(k[candidates]), kind="stable")]
    selected = order[:nstations]
    evidence: list[CornerSample] = []
    cache: dict[tuple[int, float, float], CornerSample] = {}
    query_count = 0
    for i in selected:
        if k[i] == 0.0 or cap[i] < MINIMUM_STEADY_CORNER_SPEED:
            continue
        speeds = (np.array([cap[i]]) if samples_per_station == 1 else
                  np.linspace(MINIMUM_STEADY_CORNER_SPEED, float(cap[i]), samples_per_station))
        station_samples = []
        for speed_value in speeds:
            speed = float(speed_value)
            ay = float(speed * speed * k[i])
            key = (id(vehicle), speed, ay)
            if key in cache:
                cached = cache[key]
                sample = CornerSample(float(x[i]), float(k[i]), speed, ay, cached.status,
                                      cached.residual_inf, cached.error)
            else:
                try:
                    sol = oracle(vehicle, speed, ay)
                    residual = float(sol.residual_inf)
                    if not math.isfinite(residual) or residual < 0.0 or residual >= CONTINUATION_ACCEPTANCE:
                        raise RuntimeError(f"solver residual {residual!r} outside returned-solution acceptance {CONTINUATION_ACCEPTANCE:g}")
                    sample = CornerSample(float(x[i]), float(k[i]), speed, ay, "solved", residual)
                    cache[key] = sample
                    query_count += 1
                except (RuntimeError, ValueError, FloatingPointError, np.linalg.LinAlgError) as exc:
                    sample = CornerSample(float(x[i]), float(k[i]), speed, ay, "unresolved",
                                          error=f"{type(exc).__name__}: {exc}")
                    cache[key] = sample
                    query_count += 1
            evidence.append(sample)
            station_samples.append(sample)
        prefix = []
        for sample in station_samples:
            if sample.status != "solved":
                break
            prefix.append(sample.speed)
        if prefix:
            tested_limit = max(prefix)
            if tested_limit < cap[i]:
                result[i] = tested_limit
                reasons[i] = "restricted-to-tested-prefix"
            elif station_samples[-1].status == "solved" and station_samples[-1].speed == cap[i]:
                reasons[i] = "planar-cap-tested"
            elif any(s.status == "unresolved" for s in station_samples):
                reasons[i] = "unresolved-retained"
        elif any(s.status == "unresolved" for s in station_samples):
            reasons[i] = "unresolved-retained"

    if is_closed:
        result[-1] = result[0]
        reasons[-1] = reasons[0]
    result = np.minimum(result, cap)
    result.setflags(write=False)
    return VerticalCapResult(result, tuple(evidence), tuple(reasons),
                             "vertical_14dof_tested_speed_restriction_v1", query_count, max_queries)


def _vector(value, name):
    raw = np.asarray(value)
    if raw.dtype.kind not in "iuf":
        raise TypeError(f"{name} must be numeric")
    arr = np.array(raw, dtype=float, copy=True)
    if arr.ndim != 1 or not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must be a finite one-dimensional array")
    return arr
