"""Independent fixed-path energy NLP reference using a signed capability chart.

This is intentionally a small, prescribed-path transcription: it has no vehicle
controller or torque-vectoring decision.  It reports a locally solved NLP
candidate, never a proof of globally minimum lap time.
"""
from __future__ import annotations

import math
import time
from typing import Any

import numpy as np

from simplified_vehicle_models.solvers.capability_profile import (
    CapabilityProfile, _closed_duplicate, _finite_positive, _path_samples,
)


def _signed_surface(surface):
    """Validate the only chart representation supported by this transcription."""
    required = ("u", "curvature", "speed_floor", "domain_speed", "limits")
    if not all(hasattr(surface, name) for name in required):
        raise TypeError("fixed-path energy NLP supports SignedTransientCapability only; legacy tables are unsupported")
    floor = float(surface.speed_floor)
    if not math.isfinite(floor) or floor < 0.0:
        raise ValueError("signed capability speed_floor must be finite and nonnegative")
    return floor


def _validate_scalar(name: str, value: Any, *, positive: bool = True) -> float:
    value = float(value)
    if not math.isfinite(value) or (value <= 0.0 if positive else value < 0.0):
        raise ValueError(f"{name} must be finite and {'positive' if positive else 'nonnegative'}")
    return value


def _failure(distance, speed, reference, boundary, floor, status, detail, iterations=0, diagnostics=None):
    info = {"reference_kind": "fixed-path-energy-nlp", "boundary": boundary,
            "floor": floor, "detail": detail, "claim": "local NLP candidate only"}
    info.update(diagnostics or {})
    return CapabilityProfile(distance, speed, np.full(distance.shape, np.nan), math.nan,
                             False, status, int(iterations), math.inf, reference, info)


def fixed_path_energy_profile(path, surface, *, boundary: str = "flying", start_speed=None,
                              trajectory=None, step: float = 1., maximum_iterations: int = 200,
                              tolerance: float = 1.e-8, maximum_wall_seconds: float = 60.,
                              initial_speed=None) -> CapabilityProfile:
    """Solve a bounded fixed-path energy NLP against a signed capability chart.

    ``W=v**2`` is the decision at every prescribed arclength node.  Limits are
    imposed at nodes and energy midpoints.  The midpoint curvature comes from
    the analytic trajectory station map when ``trajectory`` is supplied.
    ``boundary='flying'`` requires a physically duplicated closed endpoint;
    ``boundary='rolling'`` pins the supplied positive start speed.  Standing
    starts are deliberately unsupported because this chart has no rest model.
    """
    step = _finite_positive(step, "step")
    tolerance = _finite_positive(tolerance, "tolerance")
    maximum_wall_seconds = _validate_scalar("maximum_wall_seconds", maximum_wall_seconds)
    if isinstance(maximum_iterations, bool) or not isinstance(maximum_iterations, (int, np.integer)) or maximum_iterations < 1:
        raise ValueError("maximum_iterations must be a positive integer")
    if boundary not in ("flying", "rolling"):
        raise ValueError("boundary must be 'flying' or 'rolling'; standing is unsupported without a rest model")
    floor = _signed_surface(surface)
    distance, curvature, reference = _path_samples(path, step, trajectory)
    segment = np.diff(distance)
    midpoint_distance = .5*(distance[:-1] + distance[1:])
    if trajectory is None:
        # Without an analytic source, interpolation is the explicit sampled-path
        # representation.  It is not presented as an analytic trajectory value.
        midpoint_curvature = np.interp(midpoint_distance, distance, curvature)
    else:
        try:
            midpoint_reference = np.asarray(trajectory.station_map.to_reference(midpoint_distance), dtype=float)
            midpoint_curvature = np.asarray(trajectory.evaluate(midpoint_reference).curvature, dtype=float)
        except (TypeError, ValueError, AttributeError) as error:
            raise ValueError("trajectory must evaluate finite midpoint curvature through station_map") from error
        if midpoint_curvature.shape != midpoint_distance.shape or not np.all(np.isfinite(midpoint_curvature)):
            raise ValueError("trajectory did not return finite midpoint curvature")
    chart_curvature = np.asarray(surface.curvature, dtype=float)
    all_curvature = np.r_[curvature, midpoint_curvature]
    if np.any(all_curvature < chart_curvature[0]) or np.any(all_curvature > chart_curvature[-1]):
        raise ValueError("path curvature is outside the declared capability domain")
    if boundary == "flying":
        if start_speed is not None:
            raise ValueError("flying boundary does not accept start_speed")
        if not _closed_duplicate(path, curvature):
            raise ValueError("flying boundary requires a closed path with equal duplicated physical XY")
    else:
        if bool(path.is_closed):
            raise ValueError("rolling boundary requires an open path")
        if start_speed is None:
            raise ValueError("rolling boundary requires start_speed")
        start_speed = _validate_scalar("rolling start_speed", start_speed)

    node_cap = np.asarray([surface.domain_speed(float(k)) for k in curvature], dtype=float)
    mid_cap = np.asarray([surface.domain_speed(float(k)) for k in midpoint_curvature], dtype=float)
    if not np.all(np.isfinite(node_cap)) or not np.all(np.isfinite(mid_cap)) or np.any(node_cap < floor) or np.any(mid_cap < floor):
        raise ValueError("signed capability has invalid physical speed domain")
    if boundary == "rolling" and not floor <= start_speed <= node_cap[0]:
        raise ValueError("rolling start_speed is outside the initial capability domain")
    if initial_speed is not None:
        initial_speed = _validate_scalar("initial_speed", initial_speed, positive=False)
        if initial_speed < floor or initial_speed > float(np.min(node_cap)):
            raise ValueError("initial_speed must lie inside every declared node capability domain")

    try:
        import casadi as ca
        from simplified_vehicle_models.formulations.signed_capability import symbolic_lookup
    except ImportError as error:
        raise ImportError("fixed_path_energy_profile requires optional CasADi; install casadi to use this NLP backend") from error

    # Explicitly bounded seed: it merely initializes IPOPT, not a frozen sweep
    # state.  The NLP is free to brake, accelerate, and change every non-pinned W.
    seed_speed = float(initial_speed) if initial_speed is not None else max(floor, min(float(np.min(node_cap)), node_cap[0]))
    seed_speed = min(seed_speed, float(np.min(node_cap)))
    initial_w = np.full(distance.size, seed_speed * seed_speed)
    if boundary == "rolling":
        initial_w[0] = start_speed * start_speed

    W = ca.MX.sym("energy", distance.size)
    lookup = symbolic_lookup(surface, "fixed_path_energy")
    constraints, lower, upper = [], [], []
    # W node domain bounds are variable bounds.  Include ordinary node lookup
    # constraints too, so physical cap/normalization is independently explicit.
    lbw = np.full(distance.size, floor * floor)
    ubw = node_cap * node_cap
    if boundary == "rolling":
        lbw[0] = ubw[0] = start_speed * start_speed
    for i, k in enumerate(curvature):
        vm = ca.sqrt(W[i])
        amin, amax, domain = lookup(vm, float(k))
        constraints += [W[i] - domain*domain, amin - amax]
        lower += [-np.inf, -np.inf]; upper += [0., 0.]
    for i, ds in enumerate(segment):
        wm = .5*(W[i] + W[i+1])
        vm = ca.sqrt(wm)
        amin, amax, domain = lookup(vm, float(midpoint_curvature[i]))
        acceleration = (W[i+1] - W[i])/(2.*float(ds))
        constraints += [wm-domain*domain, acceleration-amin, acceleration-amax]
        lower += [-np.inf, 0., -np.inf]; upper += [0., np.inf, 0.]
    if boundary == "flying":
        constraints.append(W[0]-W[-1]); lower.append(0.); upper.append(0.)

    objective = ca.sum1(ca.vertcat(*[2.*float(ds)/(ca.sqrt(W[i])+ca.sqrt(W[i+1])) for i, ds in enumerate(segment)]))
    options = {"print_time": False, "ipopt": {"print_level": 0, "max_iter": int(maximum_iterations),
               "tol": float(tolerance), "acceptable_tol": float(tolerance),
               "max_cpu_time": float(maximum_wall_seconds), "max_wall_time": float(maximum_wall_seconds)}}
    started = time.monotonic()
    try:
        solver = ca.nlpsol("fixed_path_energy_nlp", "ipopt", {"x": W, "f": objective,
                          "g": ca.vertcat(*constraints)}, options)
        solved = solver(x0=initial_w, lbx=lbw, ubx=ubw, lbg=lower, ubg=upper)
        stats = solver.stats()
    except Exception as error:
        return _failure(distance, np.sqrt(np.maximum(initial_w, 0.)), reference, boundary, floor,
                        "solver-exception", f"{type(error).__name__}: {error}", diagnostics={"wall_seconds": time.monotonic()-started})
    wall = time.monotonic()-started
    candidate_w = np.asarray(solved["x"], dtype=float).reshape(-1)
    speed = np.sqrt(np.maximum(candidate_w, 0.))
    iterations = int(stats.get("iter_count", 0) or 0)
    success = bool(stats.get("success", False)) and "succeed" in str(stats.get("return_status", "")).lower()
    # Strict independent numeric proof: no symbolic clamp is used here.
    worst = 0.
    feasible = success and np.all(np.isfinite(candidate_w)) and np.all(np.isfinite(speed))
    if feasible:
        if boundary == "flying": worst = max(worst, abs(candidate_w[0]-candidate_w[-1]))
        if boundary == "rolling": worst = max(worst, abs(speed[0]-start_speed))
        for v, k, cap in zip(speed, curvature, node_cap):
            try: lo, hi = surface.limits(float(v), float(k))
            except ValueError: feasible = False; break
            worst = max(worst, max(0., v-cap), max(0., lo-hi))
        for i, ds in enumerate(segment):
            vm = math.sqrt(.5*(candidate_w[i]+candidate_w[i+1]))
            acceleration = (candidate_w[i+1]-candidate_w[i])/(2.*ds)
            try: lo, hi = surface.limits(vm, float(midpoint_curvature[i]))
            except ValueError: feasible = False; break
            worst = max(worst, max(0., vm-mid_cap[i]), max(0., lo-acceleration), max(0., acceleration-hi))
        feasible = feasible and worst <= tolerance and wall <= maximum_wall_seconds + 1.e-3
    diagnostics = {"reference_kind": "fixed-path-energy-nlp", "claim": "local NLP candidate only",
                   "boundary": boundary, "floor": floor, "objective": float(solved["f"]),
                   "wall_seconds": wall, "return_status": str(stats.get("return_status", "")),
                   "primal_violation": worst, "iterations": iterations,
                   "dual": stats.get("dual_inf", stats.get("dual_infeasibility"))}
    if not feasible:
        return _failure(distance, speed, reference, boundary, floor, "not-strictly-feasible",
                        "IPOPT did not report success or independent physical validation failed", iterations, diagnostics)
    elapsed = np.r_[0., np.cumsum(2.*segment/(speed[:-1]+speed[1:]))]
    diagnostics["time"] = float(elapsed[-1])
    return CapabilityProfile(distance, speed, elapsed, float(elapsed[-1]), True, "converged",
                             iterations, worst, reference, diagnostics)


__all__ = ["fixed_path_energy_profile"]
