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
    for name in ("curvature", "u"):
        axis = np.asarray(getattr(surface, name), dtype=float)
        if axis.ndim != 1 or axis.size < 2 or not np.all(np.isfinite(axis)) or np.any(np.diff(axis) <= 0):
            raise ValueError(f"surface {name} axis must be finite and strictly increasing")
    floor = float(surface.speed_floor)
    if not math.isfinite(floor) or floor <= 0:
        raise ValueError("surface speed_floor must be finite and positive")


def _corridor_value(corridor, station, field):
    axis = np.asarray(corridor.distance, dtype=float)
    values = np.asarray(getattr(corridor, field), dtype=float)
    if bool(getattr(corridor, "is_closed", False)):
        station = (station-axis[0]) % (axis[-1]-axis[0]) + axis[0]
    if station < axis[0] or station > axis[-1]:
        raise ValueError("corridor query outside physical domain")
    if axis.ndim != 1 or values.shape != axis.shape or axis.size < 2 or not np.all(np.isfinite(axis)) or not np.all(np.isfinite(values)) or np.any(np.diff(axis) <= 0):
        raise ValueError("corridor arrays must be finite and strictly increasing")
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
    for axis in knots:
        if axis.ndim != 1 or not np.all(np.isfinite(axis)) or np.any(np.diff(axis) <= 0):
            raise ValueError("partition axes must be finite and strictly increasing")
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
        try:
            reference_curvature = float(trajectory.reference.evaluate(s).curvature)
        except (AttributeError, TypeError, ValueError, OverflowError):
            reference_curvature = math.nan
        if not all(math.isfinite(x) for x in (v, a, k, offset)) or (name == "fold_guard" and not math.isfinite(reference_curvature)):
            unresolved.append({"station": s, "reason": "nonfinite trajectory or state query"})
            completed = False
            return math.nan
        if name == "capability_domain":
            curves = np.asarray(surface.curvature, dtype=float)
            if k < curves[0] or k > curves[-1]:
                completed = False
                unresolved.append({"station": float(s), "reason": "curvature outside capability chart"})
                return float(max(curves[0]-k, k-curves[-1], 0.0))
            domain_speed = float(surface.domain_speed(k))
            if v < surface.speed_floor or v > domain_speed:
                unresolved.append({"station": float(s), "reason": "speed outside physical capability domain",
                                  "speed": v, "speed_floor": float(surface.speed_floor),
                                  "domain_speed": domain_speed})
                completed = False
                return float(max(surface.speed_floor-v, v-domain_speed))
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
            try:
                lo, hi = map(float, surface.limits(v, k))
            except (AttributeError, TypeError, ValueError, OverflowError) as exc:
                unresolved.append({"station": s, "reason": f"capability limit query failed: {exc}"})
                completed = False
                return math.nan
            if not (math.isfinite(lo) and math.isfinite(hi) and lo <= hi):
                unresolved.append({"station": s, "reason": "nonfinite or inconsistent capability limits"})
                completed = False
                return math.nan
            return float(a-hi if name == "acceleration_max" else lo-a)
        if name == "corridor_max": return offset-_corridor_value(corridor, s, "maximum")
        if name == "corridor_min": return _corridor_value(corridor, s, "minimum")-offset
        return float(-(1.0-reference_curvature*offset))

    class _Budget(Exception): pass
    for l, r in zip(partitions[:-1], partitions[1:]):
        xs = np.linspace(l, r, cfg.samples_per_partition)
        for name in names:
            try:
                ys = [residual(name, float(x)) for x in xs]
                if np.any(np.isnan(ys)):
                    completed = False
                    unresolved.append({"station": float(xs[int(np.flatnonzero(np.isnan(ys))[0])]),
                                       "reason": "nonfinite residual query"})
                candidates = [(float(xs[i]), float(ys[i])) for i in range(len(xs)) if math.isfinite(float(ys[i]))]
                for i in range(1, len(xs)-1):
                    if all(math.isfinite(float(y)) for y in ys[i-1:i+2]) and ys[i] >= ys[i-1] and ys[i] >= ys[i+1] and (ys[i] > ys[i-1] or ys[i] > ys[i+1]):
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
    unresolved_physical_speed = any("speed outside physical capability domain" in item["reason"] for item in unresolved)
    if unresolved_speed or unresolved_physical_speed:
        completed = False
    peaks.sort(key=lambda p: (p["station"], p["name"]))
    unique = []
    for p in peaks:
        if unique and p["name"] == unique[-1]["name"] and abs(p["station"]-unique[-1]["station"]) <= tol:
            if p["violation"] > unique[-1]["violation"]: unique[-1] = p
        else: unique.append(p)
    sites = sorted({p["station"] for p in unique if left < p["station"] < right})
    unresolved_curvature = any("curvature outside" in item["reason"] for item in unresolved)
    unresolved_nonfinite = any("nonfinite trajectory" in item["reason"] for item in unresolved)
    unresolved_queries = any("query failed" in item["reason"] or "inconsistent" in item["reason"] for item in unresolved)
    unresolved_state = any("trajectory or state" in item["reason"] for item in unresolved)
    unresolved_residual = any(item["reason"] == "nonfinite residual query" for item in unresolved)
    unresolved_limit_query = any("capability limit query failed" in item["reason"] for item in unresolved)
    unresolved_unsupported = any("unevaluable" in item["reason"] for item in unresolved)
    unsupported_without_domain_violation = any(
        "unevaluable" in item["reason"] and not any(
            violation.get("reason") == "speed outside physical capability domain"
            and abs(float(violation["station"])-float(item["station"])) <= tol
            for violation in unresolved)
        for item in unresolved)
    if unresolved_curvature or unresolved_nonfinite or unresolved_queries or unresolved_state or unresolved_residual:
        completed = False
    evaluation_budget_exhausted = evaluations >= cfg.maximum_evaluations and not completed
    stop = ("nonfinite_query_unresolved" if unresolved_nonfinite or unresolved_state or unresolved_residual and not unresolved_limit_query else
            "curvature_domain_unresolved" if unresolved_curvature else
            "evaluation_budget_exhausted" if evaluation_budget_exhausted else
            "peak_budget_exhausted" if peak_budget_exhausted else
            "capability_queries_unresolved" if unresolved_speed or unresolved_physical_speed or unresolved_queries or unresolved_unsupported or unresolved_limit_query else
            "complete" if completed else "evaluation_budget_exhausted")
    fatal_reasons = (unresolved_nonfinite or unresolved_state or unresolved_residual or unresolved_curvature
                     or unresolved_queries or evaluation_budget_exhausted or peak_budget_exhausted
                     or unsupported_without_domain_violation)
    return {"method": "empirical_partitioned_residual_search_v1", "continuous_certificate": False,
            "completed": completed, "stop": stop, "search_exhausted": evaluation_budget_exhausted or peak_budget_exhausted,
            "fatal_reasons": bool(fatal_reasons),
            "physical_domain_unresolved": unresolved_physical_speed or unresolved_speed,
            "progress_stations_available": bool(sites),
            "evaluations": evaluations, "partition_count": len(partitions)-1, "peaks": unique,
            "constraint_stations": sites, "unresolved_domains": unresolved}


def _validator_refinement_stations(validation, mesh):
    """Return finite interior stations from failed physical diagnostics only."""
    excluded = {"derivative_uncertainty", "implied_acceleration_derivative_uncertainty",
                "dynamics_defect_upper_bound", "implied_acceleration_upper_bound"}
    stations = []
    for diagnostic in validation.get("diagnostics", ()):
        if not isinstance(diagnostic, dict) or diagnostic.get("passed") is not False or diagnostic.get("name") in excluded:
            continue
        for value in [diagnostic.get("worst_station"), *diagnostic.get("refinement_stations", ())]:
            if value is None or isinstance(value, (bool, np.bool_)):
                continue
            try: station = float(value)
            except (TypeError, ValueError): continue
            if math.isfinite(station) and mesh[0] < station < mesh[-1]:
                stations.append(station)
    return sorted(set(stations))


class _ExchangeFailure(Exception):
    def __init__(self, status, detail=None): self.status, self.detail = status, detail


def solve_constraint_exchange(track, corridor, surface, mesh, spline, settings, *, vehicle=None,
                              seed=None, constraint_stations=None, maximum_rounds=5,
                              maximum_sites=500, maximum_wall_seconds=600,
                              maximum_new_sites_per_round=None,
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
    if maximum_new_sites_per_round is not None and (
        isinstance(maximum_new_sites_per_round, (bool, np.bool_))
        or not isinstance(maximum_new_sites_per_round, (int, np.integer))
        or maximum_new_sites_per_round < 1
    ):
        raise ValueError("maximum_new_sites_per_round must be None or an integer >= 1")
    if isinstance(maximum_wall_seconds, (bool, np.bool_)) or not isinstance(maximum_wall_seconds, (int, float, np.number)) or not math.isfinite(float(maximum_wall_seconds)) or maximum_wall_seconds <= 0: raise ValueError("maximum_wall_seconds must be positive")
    mesh = np.asarray(mesh, dtype=float)
    if mesh.ndim != 1 or mesh.size < 2 or not np.all(np.isfinite(mesh)) or np.any(np.diff(mesh) <= 0):
        raise ValueError("mesh must be finite and strictly increasing")
    if constraint_stations is None:
        retained = []
    else:
        requested = np.asarray(constraint_stations, dtype=float)
        if requested.ndim != 1 or not np.all(np.isfinite(requested)) or np.any(requested <= mesh[0]) or np.any(requested >= mesh[-1]):
            raise ValueError("constraint_stations must be finite interior mesh-domain stations")
        retained = sorted(set(map(float, requested)))
    if len(retained) > maximum_sites:
        raise ValueError("initial constraint_stations exceed maximum_sites")
    report = {"accepted": False, "validated_solution_available": False, "status": "running", "rounds": [], "constraint_stations": retained,
              "hard_deadline_enforced": False, "continuous_certificate": False}
    start = time.monotonic(); current_seed = seed; solution = None
    def finish(status):
        report["status"] = status
        if report["rounds"] and report["rounds"][-1]["status"] not in ("solver_failed", "solver_exception", "validator_exception"):
            report["rounds"][-1]["status"] = status
        if on_round is not None: on_round(copy.deepcopy(report))
        return (solution if status == "locally_validated" and report["validated_solution_available"] else None), report
    def notify():
        if on_round is not None: on_round(copy.deepcopy(report))
    try:
        for index in range(int(maximum_rounds)):
            if time.monotonic()-start >= maximum_wall_seconds: raise _ExchangeFailure("wall_budget_exhausted")
            round_record = {"round": index+1, "validation": None, "search": None, "status": "solving"}
            report["rounds"].append(round_record)
            try:
                solution = solver(track, corridor, surface, mesh, spline, settings, seed=current_seed,
                                  constraint_stations=retained)
            except SplineSolveError as exc:
                round_record["status"] = "solver_failed"
                report["solver_failure"] = {"message": str(exc), "diagnostics": copy.deepcopy(exc.diagnostics)}
                raise _ExchangeFailure("solver_failed")
            except Exception as exc:
                round_record["status"] = "solver_exception"
                raise _ExchangeFailure("solver_exception", str(exc))
            if time.monotonic()-start >= maximum_wall_seconds:
                raise _ExchangeFailure("wall_budget_exhausted")
            if getattr(solution, "status", None) != "Solve_Succeeded": raise _ExchangeFailure("solver_failed")
            round_record["status"] = "solved"
            try: validation = validator(solution, corridor, surface, settings, vehicle=vehicle, maximum_refinements=6)
            except Exception as exc:
                round_record["status"] = "validator_exception"
                raise _ExchangeFailure("validator_exception", str(exc))
            if time.monotonic()-start >= maximum_wall_seconds:
                raise _ExchangeFailure("wall_budget_exhausted")
            checks = validation.get("checks", {})
            gate_value = checks.get("geometry_consistency", validation.get("geometry_consistency"))
            dense_value = checks.get("dense_feasibility", validation.get("dense_feasibility"))
            gate = gate_value is True
            dense = dense_value is True and validation.get("sampling_converged") is True
            model = validation.get("model_feasible") is True
            round_record["validation"] = copy.deepcopy(validation)
            if maximum_new_sites_per_round is not None:
                # Validator evidence can legitimately carry nonfinite residuals;
                # keep its failure semantics, but make opt-in reports strict-JSON-safe.
                for diagnostic in round_record["validation"].get("diagnostics", ()):
                    if not isinstance(diagnostic, dict):
                        continue
                    nonfinite = [key for key, value in diagnostic.items()
                                 if isinstance(value, (float, np.floating)) and not math.isfinite(float(value))]
                    for key in nonfinite:
                        diagnostic[key] = None
                    if nonfinite:
                        diagnostic["nonfinite_fields"] = nonfinite
            round_record["status"] = "validated" if gate and dense and model else "validation_incomplete"
            if gate and dense and model and time.monotonic()-start < maximum_wall_seconds:
                # Local checks do not establish mesh/basis, matched baseline, or replay gates.
                round_record["status"] = "locally_validated_missing_strict_gates"
                report["validated_solution_available"] = True
                report["status"] = "locally_validated"
                return finish("locally_validated")
            if time.monotonic()-start >= maximum_wall_seconds: raise _ExchangeFailure("wall_budget_exhausted")
            try: search = locate_physical_residuals(solution, corridor, surface, settings=search_settings)
            except Exception as exc: raise _ExchangeFailure("search_failed", str(exc))
            if time.monotonic()-start >= maximum_wall_seconds:
                raise _ExchangeFailure("wall_budget_exhausted")
            round_record["search"] = search
            partial_domain_progress = (not search["completed"] and search.get("stop") == "capability_queries_unresolved"
                                       and search.get("physical_domain_unresolved") is True
                                       and not search.get("search_exhausted", False)
                                       and not search.get("fatal_reasons", True))
            if not search["completed"] and not partial_domain_progress:
                raise _ExchangeFailure("search_incomplete")
            round_record["search_complete"] = bool(search["completed"])
            round_record["search_progress_only"] = bool(partial_domain_progress)
            mesh_arr = np.asarray(mesh, dtype=float)
            old = list(retained)
            validator_records = []
            physical_diagnostics = {"corridor", "fold", "speed_domain", "capability_domain",
                                    "heading_limit", "heading_rate_limit", "acceleration_max",
                                    "acceleration_min", "speed_ceiling"}
            for diagnostic in validation.get("diagnostics", ()):
                if not isinstance(diagnostic, dict) or diagnostic.get("passed") is not False or diagnostic.get("name") not in physical_diagnostics:
                    continue
                station = diagnostic.get("worst_station")
                if station is None or isinstance(station, (bool, np.bool_)):
                    continue
                try: station = float(station)
                except (TypeError, ValueError): continue
                if math.isfinite(station) and mesh_arr[0] < station < mesh_arr[-1]:
                    validator_records.append((station, str(diagnostic.get("name", ""))))
            raw_search = search.get("constraint_stations", ())
            peak_records = []
            tolerance = validation.get("tolerances", {})
            tolerance_keys = {"acceleration_max": "acceleration_mps2", "acceleration_min": "acceleration_mps2",
                             "speed_floor": "speed_mps", "speed_ceiling": "speed_mps",
                             "speed_domain": "speed_mps", "capability_domain": "speed_mps",
                             "corridor_max": "corridor_m", "corridor_min": "corridor_m",
                             "fold_guard": "fold", "fold": "fold",
                             "heading_limit": "heading_rad", "heading_rate_limit": "curvature_per_m",
                             "corridor": "corridor_m", "dynamics_defect": "dynamics_mps_per_m",
                             "implied_acceleration": "acceleration_mps2"}
            fallback_scales = {"acceleration_max": 1., "acceleration_min": 1.,
                               "implied_acceleration": 1., "speed_floor": 1.,
                               "speed_ceiling": 1., "speed_domain": 1., "capability_domain": 1.,
                               "corridor_max": 1., "corridor_min": 1., "corridor": 1.,
                               "fold_guard": 1., "fold": 1., "heading_limit": 1.,
                               "heading_rate_limit": 1., "dynamics_defect": 1.}
            for peak in search.get("peaks", ()):
                try:
                    station, name, violation = float(peak["station"]), str(peak["name"]), float(peak["violation"])
                except (KeyError, TypeError, ValueError): continue
                if math.isfinite(station) and math.isfinite(violation) and mesh_arr[0] < station < mesh_arr[-1]:
                    tolerance_key = tolerance_keys.get(name)
                    scale_value = tolerance.get(tolerance_key) if tolerance_key and isinstance(tolerance, dict) else None
                    try: scale = float(scale_value) if scale_value is not None else math.nan
                    except (TypeError, ValueError): scale = math.nan
                    # Explicit physical-unit fallback when validator tolerance evidence
                    # is absent; this ranks sites only and cannot relax acceptance.
                    if not math.isfinite(scale) or scale <= 0:
                        scale = fallback_scales.get(name, 1.)
                    severity = violation / scale
                    peak_records.append((station, name, severity))
            def already_enforced(s):
                element = max(0, min(len(mesh_arr)-2, int(np.searchsorted(mesh_arr, s, side="right")-1)))
                local = min(np.diff(mesh_arr)[element] * 1e-8, 1e-8)
                return min(abs(np.asarray([*mesh_arr, *((mesh_arr[:-1]+mesh_arr[1:])/2), *old])-s), default=math.inf) <= local
            # Build independently ranked families: validator worst stations first,
            # then one search maximum per (mesh element, diagnostic family).
            priority = []
            for station, name in sorted(set(validator_records), key=lambda x: (x[0], x[1])):
                if not already_enforced(station): priority.append((0, station, "validator:" + name, None,
                                                                   "validator_violation_magnitude_unavailable"))
            best_family = {}
            for station, name, severity in peak_records:
                element = max(0, min(len(mesh_arr)-2, int(np.searchsorted(mesh_arr, station, side="right")-1)))
                key = (element, name)
                if key not in best_family or (severity, -station) > (best_family[key][1], -best_family[key][0]):
                    best_family[key] = (station, severity)
            priority.extend((1, station, "search:" + name, severity, None)
                            for (element, name), (station, severity) in sorted(best_family.items())
                            if not already_enforced(station))
            priority.sort(key=lambda x: (x[0], -x[3] if x[3] is not None else 0., x[1], x[2]))
            ranked = list(priority)
            for station in sorted(set(map(float, raw_search))):
                if math.isfinite(station) and mesh_arr[0] < station < mesh_arr[-1] and not already_enforced(station) and all(abs(station-x[1]) > 1e-12 for x in ranked):
                    ranked.append((2, station, "search:other", 0., None))
            for station in _validator_refinement_stations(validation, mesh_arr):
                if not already_enforced(station) and all(abs(station-x[1]) > 1e-12 for x in ranked):
                    ranked.append((3, station, "validator:refinement", 0., None))
            # Multiple failed families can identify the same physical station;
            # count/select unique stations so the per-round budget is literal.
            unique_ranked = []
            for candidate in ranked:
                if all(abs(candidate[1]-existing[1]) > 1e-12 for existing in unique_ranked):
                    unique_ranked.append(candidate)
            ranked = unique_ranked
            available = maximum_sites - len(old)
            if available <= 0: raise _ExchangeFailure("maximum_sites")
            budget = available if maximum_new_sites_per_round is None else min(int(maximum_new_sites_per_round), available)
            selected = ranked[:budget]
            added = sorted(set(x[1] for x in selected))
            if not added: raise _ExchangeFailure("no_new_stations")
            unselected = ranked[budget:]
            retained = sorted(set(old+added)); report["constraint_stations"] = retained
            record = report["rounds"][-1]
            record["added_stations"] = added
            if maximum_new_sites_per_round is not None:
                record["unselected_stations_count"] = len(unselected)
                record["truncated_additions"] = bool(unselected)
                record["planned_additions"] = [
                    {"station": candidate[1], "source": candidate[2],
                     "normalized_severity": candidate[3],
                     **({"severity_unavailable_reason": candidate[4]} if candidate[4] else {})}
                    for candidate in selected]
                record["addition_budget"] = budget
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
