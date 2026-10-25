"""Deterministic bounded yaw-aware offset-line planning (Phase 3).

The planner evaluates the exact analytic ``OffsetTrajectory`` geometry and
uses empirical dense containment checks; it does not modify QSS OCPs.
"""
from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping

import numpy as np
from scipy.optimize import minimize

from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
from simplified_vehicle_models.solvers.yaw_aware_profile import (
    YawPath, curvature_rate_from_samples, to_speed_plan, yaw_aware_profile,
)
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

    def __post_init__(self):
        for name in ("control_spacing", "grid_spacing", "optimizer_x_tolerance", "optimizer_f_tolerance", "maximum_speed", "lap_time_tolerance", "corner_min_length"):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("footprint_margin", "buffer", "corner_quiet_curvature"):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if self.boundary not in ("flying", "standing"):
            raise ValueError("boundary must be 'flying' or 'standing'")
        if self.dense_subdivisions < 1 or self.optimizer_max_iterations < 1 or self.maximum_outer_passes < 1:
            raise ValueError("subdivisions, iterations, and passes must be positive")
        blends = (0., .25, .5, .75)
        object.__setattr__(self, "seed_blends", blends)


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


def _corner_complexes(track, threshold, min_length):
    """Return contiguous non-quiet source-station windows; join across closed seam."""
    distance, curvature = np.asarray(track.distance), np.abs(np.asarray(track.curvature))
    active = curvature >= threshold
    # Fill short quiet gaps, keeping chicanes/S-bends in one complex.
    for i in range(1, len(active)-1):
        if not active[i] and active[i-1] and active[i+1]:
            active[i] = True
    edges = np.flatnonzero(np.diff(np.r_[False, active, False].astype(int)))
    windows = [(float(distance[a]), float(distance[min(b, len(distance)-1)])) for a,b in zip(edges[::2],edges[1::2])]
    windows = [(a,b) for a,b in windows if b-a >= min_length]
    if track.is_closed and len(windows) > 1 and windows[0][0] == distance[0] and windows[-1][1] == distance[-1]:
        windows = [(windows[-1][0], windows[0][1] + (distance[-1]-distance[0]))] + windows[1:-1]
    return windows


def _smooth_curvature_seed(spline, reference, blend):
    """Deterministic linearized quadratic curvature/length surrogate seed.

    Curvature is minimized by moving away from the turn (``n`` opposite κ);
    increasing epsilon favors inward offset/shorter radius.  The resulting
    stationwise target is projected into the declared bounds then smoothed by
    the existing cubic OffsetSpline basis, a constrained numerical-QP surrogate.
    """
    k = np.asarray(reference.evaluate(spline.control_distance).curvature, dtype=float)
    scale = np.maximum(np.abs(k), 1.e-5)
    curvature_target = -np.sign(k) * np.minimum(2.0, 0.25 / scale)
    length_target = np.sign(k) * np.minimum(1.0, 0.08 / scale)
    return (1.0-blend) * curvature_target + blend * length_target


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
    for blend in settings.seed_blends:
        seeds.append(np.clip(_smooth_curvature_seed(spline, reference, blend), low, high))
    corners = _corner_complexes(track, settings.corner_quiet_curvature, settings.corner_min_length)
    records = []

    def evaluate(coeff, seed_index, stage):
        record = {"seed": seed_index, "stage": stage, "accepted": False, "reason": None,
                  "lap_time": None, "coefficients": np.asarray(coeff).tolist()}
        try:
            trajectory = OffsetTrajectory(reference, spline, coeff)
            path = _sample_trajectory(trajectory, settings.grid_spacing)
            dense_source = np.linspace(reference.start, reference.end,
                (len(track.distance)-1)*settings.dense_subdivisions+1)
            dense = trajectory.evaluate(dense_source)
            off = np.asarray(dense.offset)
            dlo = np.interp(dense_source, shrunk.distance, shrunk.minimum)
            dhi = np.interp(dense_source, shrunk.distance, shrunk.maximum)
            if np.any(np.asarray(dense.fold_factor) < .2-1e-8) or np.any(off < dlo-1e-8) or np.any(off > dhi+1e-8):
                record["reason"] = "geometry_or_corridor"; return record, None
        except Exception as exc:
            record["reason"] = "geometry_or_corridor"; record["detail"] = str(exc); return record, None
        try:
            profile = yaw_aware_profile(path, capability, boundary=settings.boundary,
                                        maximum_speed=settings.maximum_speed,
                                        speed_samples=32, maximum_iterations=80)
        except RuntimeError as exc:
            record["reason"] = "profile_nonconvergence"; record["detail"] = str(exc); return record, None
        except ValueError as exc:
            text = str(exc)
            record["reason"] = "capability_domain" if ("map" in text or "feasible" in text or "transition" in text) else "profile_invalid"
            record["detail"] = text; return record, None
        record.update(accepted=True, reason="accepted", lap_time=profile.lap_time,
                      geometry_grid_nodes=len(path.distance))
        return record, (trajectory, path, profile)

    best_coeff = np.zeros(spline.control_count)
    best_eval = None
    previous_time = None
    pass_summary = []
    for outer in range(1, min(3, settings.maximum_outer_passes)+1):
        for idx, seed in enumerate(seeds):
            if settings.optimizer_max_iterations <= 1 and idx > 1:
                continue
            if outer > 1 and best_eval is not None:
                seed = best_coeff.copy()
            rec, item = evaluate(seed, idx, f"pass_{outer}_seed")
            records.append(rec)
            if item and (best_eval is None or item[2].lap_time < best_eval[2].lap_time):
                best_coeff, best_eval = seed.copy(), item
            if not item:
                continue
            # Optimize one corner complex at a time; controls outside the window
            # are fixed, with smooth fixed blending supplied by the spline basis.
            for ci, (start, end) in enumerate(corners):
                if settings.optimizer_max_iterations <= 1:
                    break
                active = np.flatnonzero((spline.control_distance >= start) & (spline.control_distance <= min(end, track.lap_length)))
                if track.is_closed and end > track.lap_length:
                    active = np.unique(np.r_[active, np.flatnonzero(spline.control_distance <= end-track.lap_length)])
                if not active.size:
                    continue
                base = seed.copy()
                def objective(values):
                    candidate = base.copy(); candidate[active] = values
                    recx, evalx = evaluate(candidate, idx, f"pass_{outer}_corner_{ci}")
                    records.append(recx)
                    return evalx[2].lap_time if evalx else 1.e8 + float(np.linalg.norm(values-seed[active]))
                result = minimize(objective, seed[active], method="Powell",
                    bounds=[bounds[j] for j in active], options={"maxiter": settings.optimizer_max_iterations,
                    "xtol": settings.optimizer_x_tolerance, "ftol": settings.optimizer_f_tolerance})
                candidate = base.copy(); candidate[active] = result.x
                recx, evalx = evaluate(candidate, idx, f"pass_{outer}_corner_{ci}_result")
                recx.update(optimizer_success=bool(result.success), optimizer_iterations=int(result.nit))
                records.append(recx)
                if evalx and (best_eval is None or evalx[2].lap_time < best_eval[2].lap_time):
                    best_coeff, best_eval = candidate, evalx
        current = None if best_eval is None else best_eval[2].lap_time
        pass_summary.append({"pass": outer, "lap_time": current, "corner_complexes": len(corners)})
        if previous_time is not None and current is not None and abs(previous_time-current) < settings.lap_time_tolerance:
            break
        previous_time = current
    if best_eval is None:
        raise YawLineOptimizationError("all yaw-aware line candidates rejected", records)
    trajectory, path, profile = best_eval
    speed_plan = to_speed_plan(path, profile)
    status = "converged" if (len(pass_summary) == 1 or (len(pass_summary) > 1 and abs(pass_summary[-2]["lap_time"]-pass_summary[-1]["lap_time"]) < settings.lap_time_tolerance)) else "iteration_limit"
    shape = _shape_metrics(path, profile, trajectory)
    diagnostics = {"candidate_evaluations": tuple(records), "selected_coefficients": best_coeff.tolist(),
        "map_source_kind": capability.source_kind, "map_source_identity": capability.source_identity,
        "containment_check": "dense_empirical_v1", "optimizer": "bounded_Powell",
        "seed_family": "curvature_energy_length_epsilon_0_0.25_0.5_0.75",
        "corner_complexes": tuple(corners), "outer_passes": tuple(pass_summary),
        "converged": status == "converged"}
    return YawAwareLinePlan(trajectory, profile, speed_plan, shrunk, best_coeff,
                            profile.lap_time, status, diagnostics, shape)
