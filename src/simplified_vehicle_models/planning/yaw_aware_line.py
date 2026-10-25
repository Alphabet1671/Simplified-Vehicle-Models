"""Deterministic bounded yaw-aware offset-line planning (Phase 3).

The optimizer is deliberately independent of QSS OCP internals. Candidate
profiles are evaluated on the exact analytic ``OffsetTrajectory`` geometry;
containment is a dense empirical gate, not a continuous certificate.
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
from simplified_vehicle_models.tracks.corridor import Corridor, corridor_from_track
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
    seed_blends: tuple[float, ...] = (0.0, 0.5, 1.0)

    def __post_init__(self):
        for name in ("control_spacing", "grid_spacing", "optimizer_x_tolerance", "optimizer_f_tolerance", "maximum_speed"):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("footprint_margin", "buffer"):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if self.boundary not in ("flying", "standing"):
            raise ValueError("boundary must be 'flying' or 'standing'")
        if self.dense_subdivisions < 1 or self.optimizer_max_iterations < 1:
            raise ValueError("subdivisions and optimizer iterations must be positive")
        blends = tuple(float(x) for x in self.seed_blends)
        if not blends or any(not 0 <= x <= 1 for x in blends):
            raise ValueError("seed_blends must be a nonempty tuple in [0, 1]")
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
    n = max(20, int(np.ceil(length / spacing)))
    distance = np.linspace(0.0, length, n + 1)
    source = np.asarray(trajectory.station_map.to_reference(distance), dtype=float)
    values = trajectory.evaluate(source)
    curvature = np.asarray(values.curvature, dtype=float)
    rate = curvature_rate_from_samples(distance, curvature, is_closed=trajectory.reference.is_closed)
    return YawPath(distance, curvature, rate, trajectory.reference.is_closed), source


def _metrics(path, profile):
    k = path.curvature
    # Length containing central half of the absolute heading-change measure.
    increments = np.abs(.5 * (k[:-1] + k[1:]) * np.diff(path.distance))
    total = float(np.sum(increments))
    if total <= 1e-12:
        l50 = float(path.distance[-1])
    else:
        cumulative = np.r_[0., np.cumsum(increments)] / total
        lo = float(np.interp(.25, cumulative, path.distance))
        hi = float(np.interp(.75, cumulative, path.distance))
        l50 = hi - lo
    abs_k = np.abs(k)
    peak = int(np.argmax(abs_k))
    min_speed = int(np.argmin(profile.speed))
    return {"L50": l50, "kappa_peak": float(np.max(abs_k)),
            "kappa_mean": float(np.mean(abs_k)), "kappa_peak_mean_ratio": float(np.max(abs_k) / max(np.mean(abs_k), 1e-12)),
            "apex_distance": float(path.distance[peak]), "apex_lateral_position": 0.0,
            "minimum_speed": float(profile.speed[min_speed]), "minimum_speed_distance": float(path.distance[min_speed])}


def optimize_yaw_aware_line(track: TrackProfile, corridor: Corridor | None,
                            capability: YawCapabilityMap, *,
                            settings: YawLineSettings = YawLineSettings()) -> YawAwareLinePlan:
    """Optimize a track offset with deterministic seed family and Powell.

    Corridor samples are expected to be in reference-station coordinates; an
    optional corridor is retained as provenance and narrowed by footprint and
    buffer without modifying the caller's object.
    """
    if not isinstance(track, TrackProfile) or track.path_coordinates is None:
        raise ValueError("track must provide signed PathCoordinates")
    if not isinstance(capability, YawCapabilityMap):
        raise TypeError("capability must be a YawCapabilityMap")
    if corridor is not None and not isinstance(corridor, Corridor):
        raise TypeError("corridor must be a Corridor or None")
    if (settings.boundary == "flying") != bool(track.is_closed):
        raise ValueError("flying requires closed track; standing requires open track")
    reference = ReferenceCurve(track)
    base = corridor or corridor_from_track(track, margin=0.0,
                                            half_width=None if track.has_edges else 5.0)
    if not np.array_equal(base.distance, track.distance):
        raise ValueError("corridor must use the track reference-distance grid")
    spline = OffsetSpline(track, settings.control_spacing)
    low = np.interp(spline.control_distance, base.distance, base.minimum) + settings.footprint_margin + settings.buffer
    high = np.interp(spline.control_distance, base.distance, base.maximum) - settings.footprint_margin - settings.buffer
    if np.any(low > high):
        raise ValueError("corridor is infeasible after footprint/buffer shrink")
    limits = list(zip(low.tolist(), high.tolist()))
    # Keep constant offsets plus smooth deterministic alternating offsets within bounds.
    span = np.minimum(-low, high)
    seeds = [np.zeros(spline.control_count), np.asarray(high, dtype=float) * 0.85,
             np.asarray(low, dtype=float) * 0.85]
    for blend in settings.seed_blends:
        phase = np.linspace(0., 2*np.pi, spline.control_count, endpoint=not track.is_closed)
        pattern = np.sin(phase) if blend < .75 else np.cos(phase)
        seeds.append(np.clip(blend * span * pattern, low, high))
    records = []

    def evaluate(coeff, seed_index, stage):
        record = {"seed": seed_index, "stage": stage, "accepted": False, "reason": None,
                  "lap_time": None, "coefficients": np.asarray(coeff).tolist()}
        try:
            trajectory = OffsetTrajectory(reference, spline, coeff)
            path, source = _sample_trajectory(trajectory, settings.grid_spacing)
            dense_source = np.linspace(reference.start, reference.end,
                                       (len(track.distance)-1)*settings.dense_subdivisions+1)
            dense = trajectory.evaluate(dense_source)
            off = np.asarray(dense.offset)
            lo = np.interp(dense_source, base.distance, base.minimum) + settings.footprint_margin + settings.buffer
            hi = np.interp(dense_source, base.distance, base.maximum) - settings.footprint_margin - settings.buffer
            if np.any(np.asarray(dense.fold_factor) <= 0) or np.any(off < lo-1e-8) or np.any(off > hi+1e-8):
                record["reason"] = "geometry_or_corridor"
                return record, None
        except Exception as exc:
            record["reason"] = "geometry_or_corridor"
            record["detail"] = str(exc)
            return record, None
        try:
            profile = yaw_aware_profile(path, capability, boundary=settings.boundary,
                                        maximum_speed=settings.maximum_speed,
                                        speed_samples=32, maximum_iterations=80)
        except RuntimeError as exc:
            record["reason"] = "profile_nonconvergence"
            record["detail"] = str(exc)
            return record, None
        except ValueError as exc:
            text = str(exc)
            record["reason"] = "capability_domain" if ("map" in text or "feasible" in text or "transition" in text) else "profile_invalid"
            record["detail"] = text
            return record, None
        record.update(accepted=True, reason="accepted", lap_time=profile.lap_time)
        record["geometry_grid_nodes"] = len(path.distance)
        return record, (trajectory, path, profile)

    best = None
    best_eval = None
    for idx, seed in enumerate(seeds):
        rec, evaluated = evaluate(seed, idx, "seed")
        records.append(rec)
        if evaluated and (best is None or evaluated[2].lap_time < best_eval[2].lap_time):
            best, best_eval = seed.copy(), evaluated
        if not evaluated:
            continue
        def objective(x):
            item, value = evaluate(x, idx, "powell")
            records.append(item)
            if item["accepted"]:
                return value[2].lap_time
            return 1e6 + float(np.linalg.norm(x-seed))
        if idx > 1:
            continue
        result = minimize(objective, seed, method="Powell", bounds=limits,
                          options={"maxiter": settings.optimizer_max_iterations,
                                   "xtol": settings.optimizer_x_tolerance,
                                   "ftol": settings.optimizer_f_tolerance})
        rec, evaluated = evaluate(result.x, idx, "optimized")
        rec["optimizer_success"] = bool(result.success)
        rec["optimizer_iterations"] = int(result.nit)
        records.append(rec)
        if evaluated and (best is None or evaluated[2].lap_time < best_eval[2].lap_time):
            best, best_eval = np.asarray(result.x).copy(), evaluated
    if best is None:
        rejected = [record["reason"] for record in records]
        raise YawLineOptimizationError(f"all yaw-aware line candidates rejected: {rejected}", records)
    trajectory, path, profile = best_eval
    speed_plan = to_speed_plan(path, profile)
    shape = _metrics(path, profile)
    shape["apex_lateral_position"] = float(np.interp(shape["apex_distance"],
        trajectory.station_map.path_station, np.asarray(trajectory.evaluate(trajectory.station_map.reference_station).offset)))
    diagnostics = {"candidate_evaluations": tuple(records), "selected_seed_coefficients": best.tolist(),
                  "map_source_kind": capability.source_kind, "map_source_identity": capability.source_identity,
                  "containment_check": "dense_empirical_v1", "optimizer": "Powell", "seed_count": len(seeds)}
    return YawAwareLinePlan(trajectory, profile, speed_plan, base, best, profile.lap_time,
                            "converged", diagnostics, shape)
