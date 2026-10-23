"""Bounded empirical physical-residual localization and fixed-mesh exchange.

Residual peaks are diagnostic sampling evidence, never a continuous certificate
or an acceptance gate. The independent line validator remains authoritative.
"""
from __future__ import annotations

from dataclasses import dataclass
import copy
import math
import time
from typing import Any

import numpy as np
from scipy.optimize import minimize_scalar


@dataclass(frozen=True)
class ResidualSearchSettings:
    samples_per_partition: int = 9
    maximum_evaluations: int = 100000
    station_tolerance: float = 1e-7
    violation_tolerance: float = 1e-7
    maximum_peaks: int = 1000

    def __post_init__(self):
        for name in ("samples_per_partition", "maximum_evaluations", "maximum_peaks"):
            value = getattr(self, name)
            minimum = 5 if name == "samples_per_partition" else 1
            if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)) or value < minimum:
                raise ValueError(f"{name} must be an integer >= {minimum}")
        for name, positive in (("station_tolerance", True), ("violation_tolerance", False)):
            value = getattr(self, name)
            if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number)):
                raise ValueError(f"{name} must be finite and {'positive' if positive else 'nonnegative'}")
            value = float(value)
            if not math.isfinite(value) or (value <= 0 if positive else value < 0):
                raise ValueError(f"{name} must be finite and {'positive' if positive else 'nonnegative'}")


def _strict_surface(surface):
    if getattr(surface, "representation", None) != "signed_transient_normalized_v1" or not all(
        callable(getattr(surface, name, None)) for name in ("domain_speed", "limits")
    ) or not all(hasattr(surface, name) for name in ("speed_floor", "curvature", "u")):
        raise TypeError("only SignedTransientCapability signed chart is supported")


def _corridor_value(corridor, station, field):
    axis = np.asarray(corridor.distance, dtype=float)
    values = np.asarray(getattr(corridor, field), dtype=float)
    if bool(getattr(corridor, "is_closed", False)):
        station = (station-axis[0]) % (axis[-1]-axis[0]) + axis[0]
    if station < axis[0] or station > axis[-1]:
        raise ValueError("corridor query outside physical domain")
    return float(np.interp(station, axis, values))


def locate_physical_residuals(solution, corridor, surface, *, settings=None) -> dict:
    """Search piecewise smooth intervals for signed physical residual peaks."""
    cfg = settings or ResidualSearchSettings()
    if not isinstance(cfg, ResidualSearchSettings):
        raise TypeError("settings must be ResidualSearchSettings")
    _strict_surface(surface)
    trajectory = getattr(solution, "trajectory", None)
    evaluator = getattr(solution, "state_evaluator", None)
    if trajectory is None or not callable(getattr(evaluator, "evaluate", None)):
        raise ValueError("solution requires trajectory and dense state evaluator")
    mesh = np.asarray(solution.distance, dtype=float)
    if mesh.ndim != 1 or mesh.size < 2 or not np.all(np.isfinite(mesh)) or np.any(np.diff(mesh) <= 0):
        raise ValueError("solution distance must be finite and strictly increasing")
    left, right = float(mesh[0]), float(mesh[-1])
    knots = [mesh, np.asarray(trajectory.reference.source_station, dtype=float),
             np.asarray(trajectory.spline.control_distance, dtype=float),
             np.asarray(corridor.distance, dtype=float)]
    partitions = np.unique(np.concatenate([x[(x >= left) & (x <= right)] for x in knots] + [[left, right]]))
    min_width = float(np.min(np.diff(partitions))) if len(partitions) > 1 else right-left
    tol = min(cfg.station_tolerance, max(np.finfo(float).eps * max(1., right-left), min_width * 1e-6))
    evaluations = 0
    cache = {}
    peaks, unresolved = [], []
    completed = True
    peak_budget_exhausted = False
    names = ("capability_domain", "speed_floor", "acceleration_max", "acceleration_min",
             "corridor_max", "corridor_min", "fold_guard")

    def residual(name, s):
        nonlocal evaluations, completed
        s = float(s)
        if s not in cache:
            if evaluations >= cfg.maximum_evaluations:
                raise _Budget
            evaluations += 1
            geo = trajectory.evaluate(s); state = evaluator.evaluate(s)
            v, a, k, offset = (float(state["speed"]), float(state["acceleration"]),
                                float(geo.curvature), float(geo.offset))
            cache[s] = (geo, v, a, k, offset)
        geo, v, a, k, offset = cache[s]
        if name == "capability_domain":
            curves = np.asarray(surface.curvature, dtype=float)
            if k < curves[0] or k > curves[-1]:
                completed = False
                unresolved.append({"station": float(s), "reason": "curvature outside capability chart"})
                return float(max(curves[0]-k, k-curves[-1], 0.0))
            if v < surface.speed_floor or v > float(surface.domain_speed(k)):
                return float(max(surface.speed_floor-v, v-float(surface.domain_speed(k))))
            return 0.0
        if name == "speed_floor": return float(surface.speed_floor-v)
        if name in ("acceleration_max", "acceleration_min"):
            if not (surface.curvature[0] <= k <= surface.curvature[-1]):
                completed = False
                unresolved.append({"station": float(s), "reason": "curvature outside capability chart"})
                return -math.inf
            if not (surface.speed_floor <= v <= float(surface.domain_speed(k))):
                unresolved.append({"station": float(s), "reason": "acceleration unevaluable outside capability chart"})
                return -math.inf
            lo, hi = surface.limits(v, k)
            return float(a-hi if name == "acceleration_max" else lo-a)
        if name == "corridor_max": return offset-_corridor_value(corridor, s, "maximum")
        if name == "corridor_min": return _corridor_value(corridor, s, "minimum")-offset
        reference_curvature = float(trajectory.reference.evaluate(s).curvature)
        return float(-(1.0-reference_curvature*offset))

    class _Budget(Exception): pass
    for l, r in zip(partitions[:-1], partitions[1:]):
        xs = np.linspace(l, r, cfg.samples_per_partition)
        for name in names:
            try:
                ys = [residual(name, float(x)) for x in xs]
                candidates = [(float(xs[i]), float(ys[i])) for i in range(len(xs))]
                for i in range(1, len(xs)-1):
                    if ys[i] >= ys[i-1] and ys[i] >= ys[i+1] and (ys[i] > ys[i-1] or ys[i] > ys[i+1]):
                        result = minimize_scalar(lambda x: -residual(name, x), bounds=(xs[i-1], xs[i+1]), method="bounded",
                                                 options={"xatol": tol, "maxiter": cfg.maximum_evaluations})
                        candidates.append((float(result.x), float(-result.fun)))
                for station, value in candidates:
                    if value > cfg.violation_tolerance:
                        if len(peaks) >= cfg.maximum_peaks:
                            peak_budget_exhausted = True
                            completed = False
                            break
                        peaks.append({"name": name, "station": station, "violation": value,
                                      "element": max(0, min(len(mesh)-2, int(np.searchsorted(mesh, station, side="right")-1)))})
            except _Budget:
                completed = False
                break
        if not completed: break
    # Unsupported speed queries do not invoke chart limits and cannot establish
    # complete acceleration feasibility, though their stations remain useful.
    unresolved_speed = any("acceleration unevaluable" in item["reason"] for item in unresolved)
    if unresolved_speed:
        completed = False
    peaks.sort(key=lambda p: (p["station"], p["name"]))
    unique = []
    for p in peaks:
        if unique and p["name"] == unique[-1]["name"] and abs(p["station"]-unique[-1]["station"]) <= tol:
            if p["violation"] > unique[-1]["violation"]: unique[-1] = p
        else: unique.append(p)
    sites = sorted({p["station"] for p in unique if left < p["station"] < right})
    unresolved_curvature = any("curvature outside" in item["reason"] for item in unresolved)
    if unresolved_curvature:
        completed = False
    return {"method": "empirical_partitioned_residual_search_v1", "continuous_certificate": False,
            "completed": completed, "stop": ("curvature_domain_unresolved" if unresolved_curvature else "capability_queries_unresolved" if unresolved_speed else "peak_budget_exhausted" if peak_budget_exhausted else "complete" if completed else "evaluation_budget_exhausted"),
            "evaluations": evaluations, "partition_count": len(partitions)-1, "peaks": unique,
            "constraint_stations": sites, "unresolved_domains": unresolved}


class _ExchangeFailure(Exception):
    def __init__(self, status, detail=None): self.status, self.detail = status, detail


def solve_constraint_exchange(track, corridor, surface, mesh, spline, settings, *, vehicle=None,
                              seed=None, constraint_stations=None, maximum_rounds=5,
                              maximum_sites=500, maximum_wall_seconds=600,
                              search_settings=None, solver=None, validator=None, on_round=None):
    """Solve/refine on fixed mesh; independent validation alone controls acceptance.

    Wall timeout is soft and checked between synchronous phases; it cannot cancel
    an active solver/validator call. Spawn a child process for a hard deadline.
    """
    from simplified_vehicle_models.solvers.line_validation import validate_line
    from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line, SplineSolveError
    solver = solver or solve_spline_line
    validator = validator or validate_line
    if isinstance(maximum_rounds, bool) or not isinstance(maximum_rounds, (int, np.integer)) or maximum_rounds < 1: raise ValueError("maximum_rounds must be positive integer")
    if isinstance(maximum_sites, bool) or not isinstance(maximum_sites, (int, np.integer)) or maximum_sites < 0: raise ValueError("maximum_sites must be nonnegative integer")
    if not math.isfinite(float(maximum_wall_seconds)) or maximum_wall_seconds <= 0: raise ValueError("maximum_wall_seconds must be positive")
    retained = sorted(set(float(x) for x in (constraint_stations or ())))
    report = {"accepted": False, "status": "running", "rounds": [], "constraint_stations": retained,
              "hard_deadline_enforced": False, "continuous_certificate": False}
    start = time.monotonic(); current_seed = seed; solution = None
    def finish(status):
        report["status"] = status
        if on_round is not None: on_round(copy.deepcopy(report))
        return solution, report
    def notify():
        if on_round is not None: on_round(copy.deepcopy(report))
    try:
        for index in range(int(maximum_rounds)):
            if time.monotonic()-start >= maximum_wall_seconds: raise _ExchangeFailure("wall_budget_exhausted")
            try:
                solution = solver(track, corridor, surface, mesh, spline, settings, seed=current_seed,
                                  constraint_stations=retained)
            except SplineSolveError as exc:
                report["solver_failure"] = {"message": str(exc), "diagnostics": copy.deepcopy(exc.diagnostics)}
                raise _ExchangeFailure("solver_failed")
            except Exception as exc: raise _ExchangeFailure("solver_exception", str(exc))
            if getattr(solution, "status", None) != "Solve_Succeeded": raise _ExchangeFailure("solver_failed")
            try: validation = validator(solution, corridor, surface, settings, vehicle=vehicle, maximum_refinements=6)
            except Exception as exc: raise _ExchangeFailure("validator_exception", str(exc))
            checks = validation.get("checks", {})
            gate_value = checks.get("geometry_consistency", validation.get("geometry_consistency"))
            dense_value = checks.get("dense_feasibility", validation.get("dense_feasibility"))
            gate = gate_value is True
            dense = dense_value is True and validation.get("sampling_converged") is True
            model = validation.get("model_feasible") is True
            round_record = {"round": index+1, "validation": copy.deepcopy(validation), "search": None,
                            "status": "validated" if gate and dense and model else "validation_incomplete"}
            report["rounds"].append(round_record)
            if gate and dense and model:
                # Local checks do not establish mesh/basis, matched baseline, or replay gates.
                round_record["status"] = "locally_validated_missing_strict_gates"
                report["validated_solution_available"] = True
                report["status"] = "locally_validated"
                return finish("locally_validated")
            if time.monotonic()-start >= maximum_wall_seconds: raise _ExchangeFailure("wall_budget_exhausted")
            try: search = locate_physical_residuals(solution, corridor, surface, settings=search_settings)
            except Exception as exc: raise _ExchangeFailure("search_failed", str(exc))
            round_record["search"] = search
            if not search["completed"]: raise _ExchangeFailure("search_incomplete")
            mesh_arr = np.asarray(mesh, dtype=float)
            old = list(retained)
            added = []
            for s in search["constraint_stations"]:
                local = min(np.diff(mesh_arr)[max(0, min(len(mesh_arr)-2, int(np.searchsorted(mesh_arr, s, side="right")-1)))] * 1e-8, 1e-8)
                if min(abs(np.asarray([*mesh_arr, *((mesh_arr[:-1]+mesh_arr[1:])/2), *old])-s), default=math.inf) > local:
                    added.append(float(s))
            if not added: raise _ExchangeFailure("no_new_stations")
            if len(old)+len(added) > maximum_sites: raise _ExchangeFailure("maximum_sites")
            retained = sorted(set(old+added)); report["constraint_stations"] = retained
            report["rounds"][-1]["added_stations"] = added
            report["rounds"][-1]["status"] = "exchange_sites_added"
            if index+1 >= maximum_rounds:
                round_record["status"] = "maximum_rounds"
                raise _ExchangeFailure("maximum_rounds")
            notify()
            current_seed = np.r_[solution.coefficients, solution.speed, solution.acceleration,
                                 solution.midpoint_acceleration]
    except _ExchangeFailure as exc:
        report["detail"] = exc.detail
        return finish(exc.status)
    return finish("maximum_rounds")


__all__ = ["ResidualSearchSettings", "locate_physical_residuals", "solve_constraint_exchange"]
