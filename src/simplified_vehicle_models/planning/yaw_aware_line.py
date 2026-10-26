"""Deterministic bounded yaw-aware offset-line planning (Phase 3).

The planner evaluates the exact analytic ``OffsetTrajectory`` geometry and
uses empirical dense containment checks; it does not modify QSS OCPs.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
import time
from types import MappingProxyType
from typing import Mapping

import numpy as np
from scipy.optimize import minimize

from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
from simplified_vehicle_models.solvers.yaw_aware_profile import (
    YawPath, curvature_rate_from_samples, to_speed_plan, yaw_aware_profile,
)
from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings, YawProfileSolveError
from simplified_vehicle_models.tracks.corridor import Corridor, SIGNED_POLICY, corridor_from_track
from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory, ReferenceCurve
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.tracks.types import TrackProfile


@dataclass(frozen=True, slots=True)
class YawLineSettings:
    control_spacing: float = 18.0
    footprint_margin: float = 0.0
    buffer: float = 0.15
    grid_spacing: float = 4.0
    dense_subdivisions: int = 3
    optimizer_max_iterations: int = 8
    optimizer_x_tolerance: float = 0.08
    optimizer_f_tolerance: float = 0.003
    maximum_speed: float = 60.0
    boundary: str = "flying"
    seed_blends: tuple[float, ...] = (0.0, 0.25, 0.5, 0.75)
    maximum_outer_passes: int = 3
    lap_time_tolerance: float = 0.001
    corner_quiet_curvature: float = 0.002
    corner_min_length: float = 18.0
    corner_quiet_padding: float = 25.0
    qp_tolerance: float = 1.e-9
    profile_solver: str = "coupled"
    profile_settings: CoupledProfileSettings = field(default_factory=CoupledProfileSettings)
    maximum_profile_evaluations: int = 24
    maximum_wall_seconds: float = 900.
    minimum_improvement_seconds: float = 1.e-4

    def __post_init__(self):
        for name in ("control_spacing", "grid_spacing", "optimizer_x_tolerance", "optimizer_f_tolerance", "maximum_speed", "lap_time_tolerance", "corner_min_length", "corner_quiet_padding", "qp_tolerance", "maximum_wall_seconds", "minimum_improvement_seconds"):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("footprint_margin", "buffer", "corner_quiet_curvature"):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if self.boundary not in ("flying", "standing"):
            raise ValueError("boundary must be 'flying' or 'standing'")
        if self.profile_solver not in ("coupled", "legacy"):
            raise ValueError("profile_solver must be 'coupled' or 'legacy'")
        if not isinstance(self.profile_settings, CoupledProfileSettings):
            raise TypeError("profile_settings must be CoupledProfileSettings")
        if self.dense_subdivisions < 1 or self.optimizer_max_iterations < 1 or self.maximum_outer_passes < 1 or self.maximum_profile_evaluations < 1:
            raise ValueError("subdivisions, iterations, passes, and profile evaluations must be positive")
        blends=tuple(float(value) for value in self.seed_blends)
        if any(not np.isfinite(value) or value < 0. or value > 1. for value in blends):
            raise ValueError("seed_blends must be finite values in [0, 1]")
        object.__setattr__(self, "seed_blends", blends)


class _SearchBudgetStop(RuntimeError):
    pass


class YawLineOptimizationError(ValueError):
    """No candidate passed geometry and profile gates; diagnostics are retained."""
    def __init__(self, message, diagnostics):
        super().__init__(message)
        self.diagnostics = tuple(MappingProxyType(dict(item)) for item in diagnostics)


@dataclass(frozen=True, slots=True)
class YawAwareLinePlan:
    trajectory: OffsetTrajectory
    profile: object
    speed_plan: SpeedPlan
    corridor: Corridor
    coefficients: np.ndarray
    lap_time: float
    status: str
    diagnostics: Mapping[str, object]
    shape_metrics: Mapping[str, float]

    def __post_init__(self):
        coeff = np.array(self.coefficients, dtype=float, copy=True)
        coeff.setflags(write=False)
        object.__setattr__(self, "coefficients", coeff)
        object.__setattr__(self, "diagnostics", MappingProxyType(dict(self.diagnostics)))
        object.__setattr__(self, "shape_metrics", MappingProxyType(dict(self.shape_metrics)))


def _sample_trajectory(trajectory, spacing):
    length = float(trajectory.station_map.path_end)
    distance = np.linspace(0., length, max(20, int(np.ceil(length / spacing))) + 1)
    source = np.asarray(trajectory.station_map.to_reference(distance), dtype=float)
    curvature = np.asarray(trajectory.evaluate(source).curvature, dtype=float)
    rate = curvature_rate_from_samples(distance, curvature, is_closed=trajectory.reference.is_closed)
    return YawPath(distance, curvature, rate, trajectory.reference.is_closed)


def _shape_metrics(path, profile, trajectory):
    k, distance = path.curvature, path.distance
    increments = np.abs(.5 * (k[:-1] + k[1:]) * np.diff(distance))
    total = float(np.sum(increments))
    if total <= 1e-12:
        l50 = float(distance[-1])
    else:
        cumulative = np.r_[0., np.cumsum(increments)] / total
        l50 = float(np.interp(.75, cumulative, distance) - np.interp(.25, cumulative, distance))
    abs_k = np.abs(k)
    apex = int(np.argmax(abs_k))
    slow = int(np.argmin(profile.speed))
    source = float(trajectory.station_map.to_reference(distance[apex]))
    lateral = float(trajectory.evaluate(source).offset)
    return {"L50": l50, "kappa_peak": float(np.max(abs_k)),
            "kappa_mean": float(np.mean(abs_k)),
            "kappa_peak_mean_ratio": float(np.max(abs_k) / max(np.mean(abs_k), 1e-12)),
            "apex_distance": float(distance[apex]), "apex_lateral_position": lateral,
            "minimum_speed": float(profile.speed[slow]),
            "minimum_speed_distance": float(distance[slow])}


def _corner_complexes(track, threshold, min_length, quiet_padding=25.):
    """Group opposing turns across short straights; include braking/exit quiet runs."""
    distance = np.asarray(track.distance, dtype=float)
    curvature = np.asarray(track.curvature, dtype=float)
    active = np.abs(curvature) >= threshold
    # Join adjacent turn sections unless the intervening low-curvature straight
    # is long enough to constitute an independent braking complex.
    raw = np.flatnonzero(np.diff(np.r_[False, active, False].astype(int)))
    windows = [(float(distance[a]), float(distance[min(b, len(distance)-1)]))
               for a,b in zip(raw[::2], raw[1::2])]
    if not windows:
        return []
    grouped = [list(windows[0])]
    for start, end in windows[1:]:
        gap = start - grouped[-1][1]
        if gap <= 2.*quiet_padding:
            grouped[-1][1] = end
        else:
            grouped.append([start, end])
    padded = [(max(float(distance[0]), a-quiet_padding),
               min(float(distance[-1]), b+quiet_padding)) for a,b in grouped
              if b-a >= min_length]
    if track.is_closed and len(padded) > 1:
        if padded[0][0] == distance[0] and padded[-1][1] == distance[-1]:
            padded = [(padded[-1][0], padded[0][1]+(distance[-1]-distance[0]))] + padded[1:-1]
    return padded


def _qp_seed(spline, reference, blend, lower, upper, tolerance=1.e-9, *, linearization_cache=None):
    """Solve the deterministic convex discrete offset seed QP.

    Objective: ``(1-eps)||K(c)-k_ref||_W^2 + eps/2 ||W B1 c||^2`` where
    ``K(c)`` is the linearized offset curvature ``k_ref + A c`` about the
    reference (A is obtained by deterministic central perturbations of the
    analytic trajectory), D2 smoothness regularizes the Hessian, and the second
    term is the quadratic small-slope path-length surrogate. This yields a
    signed, reference-dependent linear term, so epsilon seeds need not collapse
    to the zero solution. Box bounds are imposed directly; SLSQP uses analytic
    gradient, fixed tolerance and fixed zero initial point.
    """
    cache_box=linearization_cache if isinstance(linearization_cache,dict) else None
    if linearization_cache is None or cache_box is not None and "data" not in cache_box:
        count = spline.control_count
        eye = np.eye(count)
        if spline.is_closed:
            D2 = np.roll(eye, 1, axis=1) - 2.*eye + np.roll(eye, -1, axis=1)
        else:
            D2 = np.diff(eye, n=2, axis=0)
        ds = float(spline.track.lap_length) / max(1, count if spline.is_closed else count-1)
        grid = np.linspace(0., spline.track.lap_length, max(40, 4*count)+1)
        _, B0, B2 = spline.basis(grid)
        k_ref = np.asarray(reference.evaluate(grid).curvature, dtype=float)
        eps_fd = 1.e-3
        A = np.empty((grid.size, count))
        for j in range(count):
            unit = np.zeros(count); unit[j] = eps_fd
            kp = np.asarray(OffsetTrajectory(reference,spline,unit,build_station_map=False).evaluate(grid).curvature)
            km = np.asarray(OffsetTrajectory(reference,spline,-unit,build_station_map=False).evaluate(grid).curvature)
            A[:,j]=(kp-km)/(2.*eps_fd)
        weights=np.full(grid.size,ds*count/max(grid.size-1,1))
        data=(count,A,weights,-k_ref,D2,B2,ds)
        if cache_box is not None: cache_box["data"]=data
        linearization_cache=data
    if cache_box is not None: linearization_cache=cache_box["data"]
    count,A,weights,target,D2,B2,ds=linearization_cache
    # The curvature map is locally quadraticized by its linearization; offset
    # second-difference smoothness is the regularizer and length is quadratic.
    curvature_Q = A.T @ (weights[:, None] * A) + D2.T @ D2 * 1.e-4
    length_Q = B2.T @ (weights[:, None] * B2) * (ds*ds)
    Q = 2.*((1.-blend)*curvature_Q + blend*length_Q) + np.eye(count)*1.e-10
    q = 2.*(1.-blend)*(A.T @ (weights*target))
    def fun(c): return .5*float(c@Q@c) + float(q@c)
    def jac(c): return Q@c + q
    result = minimize(fun, np.zeros(count), jac=jac, method="SLSQP",
        bounds=list(zip(lower, upper)), options={"ftol": tolerance, "maxiter": 100, "disp": False})
    return np.clip(np.asarray(result.x, dtype=float), lower, upper), bool(result.success), int(result.nit), float(result.fun)


def optimize_yaw_aware_line(track: TrackProfile, corridor: Corridor | None,
                            capability: YawCapabilityMap, *,
                            settings: YawLineSettings = YawLineSettings()) -> YawAwareLinePlan:
    """Run deterministic seeded segment optimization with up to three passes."""
    if not isinstance(track, TrackProfile) or track.path_coordinates is None:
        raise ValueError("track must provide signed PathCoordinates")
    if not isinstance(capability, YawCapabilityMap):
        raise TypeError("capability must be a YawCapabilityMap")
    if corridor is not None and not isinstance(corridor, Corridor):
        raise TypeError("corridor must be a Corridor or None")
    if (settings.boundary == "flying") != bool(track.is_closed):
        raise ValueError("flying requires closed track; standing requires open track")
    started=time.monotonic()
    reference = ReferenceCurve(track)
    if corridor is None:
        corridor = corridor_from_track(track, margin=0., half_width=None if track.has_edges else 5.,
                                       policy=SIGNED_POLICY, reference=reference)
    if not np.array_equal(corridor.distance, track.distance):
        raise ValueError("corridor must use the track reference-distance grid")
    # Always apply fold guard against fitted signed curvature, even for supplied legacy corridors.
    fitted = np.asarray(reference.evaluate(track.distance).curvature)
    lo = np.asarray(corridor.minimum).copy()
    hi = np.asarray(corridor.maximum).copy()
    right, left = fitted > 0., fitted < 0.
    hi[right] = np.minimum(hi[right], .8/fitted[right])
    lo[left] = np.maximum(lo[left], .8/fitted[left])
    lo += settings.footprint_margin + settings.buffer
    hi -= settings.footprint_margin + settings.buffer
    if np.any(lo > hi):
        raise ValueError("corridor is infeasible after footprint/buffer shrink")
    shrunk = Corridor(corridor.distance, lo, hi, corridor.provenance, corridor.is_closed,
                      corridor.policy, corridor.policy_descriptor)
    spline = OffsetSpline(track, settings.control_spacing)
    low = np.interp(spline.control_distance, shrunk.distance, shrunk.minimum)
    high = np.interp(spline.control_distance, shrunk.distance, shrunk.maximum)
    bounds = list(zip(low.tolist(), high.tolist()))
    seeds = [np.zeros(spline.control_count)]
    seed_solver_diagnostics = []; qp_cache={}
    for blend in settings.seed_blends:
        if time.monotonic()-started >= settings.maximum_wall_seconds:
            raise YawLineOptimizationError("global line-search wall budget exhausted before seed construction",
                                           ({"reason":"global_budget","budget_cause":"wall_seconds"},))
        qp_started=time.monotonic()
        seed, success, iterations, objective_value = _qp_seed(
            spline, reference, blend, low, high, settings.qp_tolerance,linearization_cache=qp_cache)
        seeds.append(seed)
        seed_solver_diagnostics.append({"epsilon": blend, "success": success,
                                        "iterations": iterations, "objective": objective_value,
                                        "seconds":time.monotonic()-qp_started,
                                        "method": "SLSQP_box_constrained_convex_QP"})
    corners = _corner_complexes(track, settings.corner_quiet_curvature,
                                settings.corner_min_length, settings.corner_quiet_padding)
    records = []; cache={}; profile_evaluations=[0]; cache_hits=[0]
    stop_reason=[None]
    # Accepted warm reference is projected by physical reference station, never
    # by normalized path distance.
    warm_reference = [None]

    def evaluate(coeff, seed_index, stage):
        canonical=np.ascontiguousarray(np.asarray(coeff,dtype=float).copy())
        canonical[canonical == 0.] = 0.  # normalize -0. exact-cache aliases
        key=canonical.tobytes()
        if key in cache:
            cache_hits[0]+=1
            record=dict(cache[key][0]); record.update(seed=seed_index,stage=stage,cache_hit=True)
            return record,cache[key][1]
        if time.monotonic()-started >= settings.maximum_wall_seconds:
            stop_reason[0]="wall_seconds"; raise _SearchBudgetStop()
        record = {"seed": seed_index, "stage": stage, "accepted": False, "reason": None,
                  "lap_time": None, "coefficients": np.asarray(coeff).tolist(),"cache_hit":False}
        try:
            geometry_started=time.monotonic()
            trajectory = OffsetTrajectory(reference, spline, coeff)
            path = _sample_trajectory(trajectory, settings.grid_spacing)
            dense_source = np.linspace(reference.start, reference.end,
                (len(track.distance)-1)*settings.dense_subdivisions+1)
            dense = trajectory.evaluate(dense_source)
            off = np.asarray(dense.offset)
            dlo = np.interp(dense_source, shrunk.distance, shrunk.minimum)
            dhi = np.interp(dense_source, shrunk.distance, shrunk.maximum)
            if np.any(np.asarray(dense.fold_factor) < .2-1e-8) or np.any(off < dlo-1e-8) or np.any(off > dhi+1e-8):
                record["reason"] = "geometry_or_corridor"; cache[key]=(record,None); return record, None
        except Exception as exc:
            record["reason"] = "geometry_or_corridor"; record["detail"] = str(exc); cache[key]=(record,None); return record, None
        record["geometry_seconds"]=time.monotonic()-geometry_started
        try:
            if profile_evaluations[0] >= settings.maximum_profile_evaluations:
                stop_reason[0]="profile_evaluations"; raise _SearchBudgetStop()
            remaining=settings.maximum_wall_seconds-(time.monotonic()-started)
            if remaining <= 0.:
                stop_reason[0]="wall_seconds"; raise _SearchBudgetStop()
            profile_evaluations[0]+=1
            profile_started=time.monotonic()
            if settings.profile_solver == "coupled":
                warm = None
                if warm_reference[0] is not None:
                    stations,warm_q=warm_reference[0]
                    target=np.asarray(trajectory.station_map.to_reference(path.distance),dtype=float)
                    q_suggestion=np.maximum(0.,np.interp(target,stations,warm_q))
                    if path.is_closed: q_suggestion[-1]=q_suggestion[0]
                    warm=np.sqrt(q_suggestion)
                profile_settings=replace(settings.profile_settings,max_wall_seconds=min(remaining,settings.profile_settings.max_wall_seconds))
                profile = yaw_aware_profile(path, capability, boundary=settings.boundary,
                                            maximum_speed=settings.maximum_speed, solver="coupled",
                                            coupled_settings=profile_settings,
                                            warm_start_speed=warm)
            else:
                profile = yaw_aware_profile(path, capability, boundary=settings.boundary,
                                            maximum_speed=settings.maximum_speed,
                                            speed_samples=32, maximum_iterations=80)
        except YawProfileSolveError as exc:
            # Typed coupled failure: preserve status and structured detail.
            record["reason"] = "budget_exceeded" if exc.status == "budget_exceeded" else (
                "capability_domain" if exc.status == "no_path_at_resolution" else "profile_invalid")
            record["profile_status"] = exc.status
            record["detail"] = str(exc)
            record["profile_diagnostics"] = {k: v for k, v in exc.diagnostics.items()
                                             if k in ("budget_reason", "disconnect_cut", "refinement_rounds",
                                                      "candidate_cap_rejections", "empty_station_count", "timings")}
            cache[key]=(record,None); return record, None
        except RuntimeError as exc:
            record["reason"] = "profile_nonconvergence"; record["detail"] = str(exc); cache[key]=(record,None); return record, None
        except ValueError as exc:
            text = str(exc)
            record["reason"] = "capability_domain" if ("map" in text or "feasible" in text or "transition" in text) else "profile_invalid"
            record["detail"] = text; cache[key]=(record,None); return record, None
        record["profile_seconds"]=time.monotonic()-profile_started
        record.update(accepted=True, reason="accepted", lap_time=profile.lap_time,
                      geometry_grid_nodes=len(path.distance),
                      resolution_converged=profile.diagnostics.get("resolution_converged"))
        reference_stations=np.asarray(trajectory.station_map.to_reference(path.distance),dtype=float)
        warm_reference[0]=(reference_stations,np.asarray(profile.speed,dtype=float)**2)
        item=(trajectory,path,profile)
        cache[key]=(record,item)
        return record,item

    best_coeff = np.zeros(spline.control_count)
    best_eval = None
    previous_time = None
    pass_summary = []
    selected_optimizer_success = True
    selected_optimizer_attempts = []
    # Evaluate the complete seed family before any local optimization, so the
    # first seed cannot consume the global budget before comparison.
    for idx, seed in enumerate(seeds):
        # QP box bounds constrain controls, not continuous spline overshoot.
        # Deterministically contract rejected proposals toward the centreline;
        # every repaired coefficient vector still passes the exact dense gate.
        for scale in (1.,.5,.25,.125,.0625,0.):
            candidate=seed*scale
            try:
                rec,item=evaluate(candidate,idx,"seed")
            except _SearchBudgetStop:
                break
            rec["seed_scale"]=scale; records.append(rec)
            if item:
                if best_eval is None or item[2].lap_time < best_eval[2].lap_time-settings.minimum_improvement_seconds:
                    best_coeff,best_eval=candidate.copy(),item
                break
            if rec["reason"] != "geometry_or_corridor": break
        if stop_reason[0] is not None: break
    for outer in range(1, min(3, settings.maximum_outer_passes)+1):
        if stop_reason[0] is not None or best_eval is None: break
        # Subsequent passes begin from exactly the current validated incumbent.
        for idx, seed in enumerate((best_coeff.copy(),)):
            # Optimize one corner complex at a time from the current accepted
            # incumbent; controls outside the window remain fixed.
            for ci, (start, end) in enumerate(corners):
                if stop_reason[0] is not None: break
                active = np.flatnonzero((spline.control_distance >= start) & (spline.control_distance <= min(end, track.lap_length)))
                if track.is_closed and end > track.lap_length:
                    active = np.unique(np.r_[active, np.flatnonzero(spline.control_distance <= end-track.lap_length)])
                if not active.size:
                    continue
                base = best_coeff.copy()
                def objective(values):
                    candidate = base.copy(); candidate[active] = values
                    recx, evalx = evaluate(candidate, idx, f"pass_{outer}_corner_{ci}")
                    records.append(recx)
                    return evalx[2].lap_time if evalx else 1.e8 + float(np.linalg.norm(values-seed[active]))
                try:
                    result = minimize(objective, base[active], method="Powell",
                        bounds=[bounds[j] for j in active], options={"maxiter": settings.optimizer_max_iterations,
                        "maxfev": max(1,settings.maximum_profile_evaluations-profile_evaluations[0]),
                        "xtol": settings.optimizer_x_tolerance, "ftol": settings.optimizer_f_tolerance})
                except _SearchBudgetStop:
                    break
                candidate = base.copy(); candidate[active] = result.x
                try:
                    recx, evalx = evaluate(candidate, idx, f"pass_{outer}_corner_{ci}_result")
                except _SearchBudgetStop:
                    break
                recx.update(optimizer_success=bool(result.success), optimizer_iterations=int(result.nit))
                records.append(recx)
                selected_optimizer_attempts.append(bool(result.success))
                if evalx and (best_eval is None or evalx[2].lap_time < best_eval[2].lap_time-settings.minimum_improvement_seconds):
                    best_coeff, best_eval = candidate, evalx
                    selected_optimizer_success = bool(result.success)
        current = None if best_eval is None else best_eval[2].lap_time
        pass_summary.append({"pass": outer, "lap_time": current, "corner_complexes": len(corners)})
        if previous_time is not None and current is not None and abs(previous_time-current) < settings.lap_time_tolerance:
            break
        previous_time = current
    if best_eval is None:
        if stop_reason[0] is not None:
            records.append({"reason":"global_budget","budget_cause":stop_reason[0],"accepted":False})
            raise YawLineOptimizationError("global line-search budget exhausted without validated incumbent", records)
        raise YawLineOptimizationError("all yaw-aware line candidates rejected", records)
    trajectory, path, profile = best_eval
    speed_plan = to_speed_plan(path, profile)
    if stop_reason[0] is not None:
        status = "budget_exceeded"
    elif len(pass_summary) == 1:
        status = "single_pass"
    elif selected_optimizer_attempts and not all(selected_optimizer_attempts):
        status = "iteration_limit"
    elif abs(pass_summary[-2]["lap_time"]-pass_summary[-1]["lap_time"]) < settings.lap_time_tolerance:
        status = "converged"
    else:
        status = "iteration_limit"
    shape = _shape_metrics(path, profile, trajectory)
    diagnostics = {"candidate_evaluations": tuple(records), "selected_coefficients": best_coeff.tolist(),
        "map_source_kind": capability.source_kind, "map_source_identity": capability.source_identity,
        "containment_check": "dense_empirical_v1", "optimizer": "bounded_Powell",
        "seed_family": "convex_discrete_curvature_energy_and_quadratic_length_QP",
        "seed_qp_objectives": tuple(seed_solver_diagnostics),
        "corner_complexes": tuple(corners), "outer_passes": tuple(pass_summary),
        "optimizer_success": selected_optimizer_success,
        "converged": status == "converged",
        "search_stop_reason": stop_reason[0], "search_budget_exhausted": stop_reason[0] is not None,
        "evaluated_profile_count": profile_evaluations[0], "cache_hits": cache_hits[0],
        "timings": {"wall_seconds": time.monotonic()-started},
        "minimum_improvement_seconds": settings.minimum_improvement_seconds,
        "profile_solver": settings.profile_solver,
        "profile_resolution_converged": profile.diagnostics.get("resolution_converged"),
        "profile_diagnostics": {k: profile.diagnostics[k] for k in
            ("solver", "feasibility_status", "resolution_converged", "optimality_scope", "validation",
             "budget_reason", "stopping_uncertainty", "refinement_rounds", "timings", "edge_count",
             "tested_seam_anchors", "closed_seam_rotation") if k in profile.diagnostics}}
    return YawAwareLinePlan(trajectory, profile, speed_plan, shrunk, best_coeff,
                            profile.lap_time, status, diagnostics, shape)
