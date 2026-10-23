"""Bounded computational curvature-guard proposals (not an extrema certificate)."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import math
import numbers
import time as _time
from typing import Any

_monotonic = _time.monotonic

import numpy as np
from scipy.optimize import minimize_scalar

from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory


@dataclass(frozen=True, slots=True)
class CurvatureProposalSettings:
    maximum_wall_seconds: float = 10.0
    maximum_evaluations: int = 4096
    maximum_intervals: int = 256
    maximum_candidates: int = 4096
    scout_points: int = 17
    maximum_optimizer_iterations: int = 80
    station_tolerance: float = 1e-8


def _real(value: Any) -> bool:
    return isinstance(value, numbers.Real) and not isinstance(value, (bool, np.bool_))


def _settings(value: Any) -> CurvatureProposalSettings:
    if value is None:
        result = CurvatureProposalSettings()
    elif type(value) is CurvatureProposalSettings:
        result = value
    else:
        raise TypeError("settings must be CurvatureProposalSettings or None")
    for name in ("maximum_evaluations", "maximum_intervals", "maximum_candidates", "scout_points", "maximum_optimizer_iterations"):
        item = getattr(result, name)
        minimum = 3 if name == "scout_points" else 1
        if isinstance(item, (bool, np.bool_)) or not isinstance(item, numbers.Integral) or item < minimum:
            raise ValueError(f"{name} must be an integer >= {minimum}")
    for name in ("maximum_wall_seconds", "station_tolerance"):
        item = getattr(result, name)
        if not _real(item) or not math.isfinite(float(item)) or item <= 0:
            raise ValueError(f"{name} must be finite and positive")
    return result


def _array_digest(hasher: Any, value: Any) -> None:
    a = np.asarray(value)
    hasher.update(str(a.shape).encode())
    hasher.update(a.dtype.str.encode())
    hasher.update(np.ascontiguousarray(a).tobytes())


def _fingerprint(trajectory: OffsetTrajectory) -> str:
    h = hashlib.sha256()
    for obj in (trajectory.reference._x, trajectory.reference._y, trajectory._offset_curve,
                trajectory.station_map._forward, trajectory.station_map._inverse):
        for name in ("t", "c", "x"):
            if hasattr(obj, name):
                _array_digest(h, getattr(obj, name))
        h.update(str(getattr(obj, "k", getattr(obj, "order", None))).encode())
    _array_digest(h, trajectory.coefficients)
    for obj in (trajectory.reference.source_station, trajectory.station_map.reference_station,
                trajectory.station_map.path_station):
        _array_digest(h, obj)
    h.update(repr((trajectory.reference.start, trajectory.reference.end,
                   trajectory.station_map.path_start, trajectory.station_map.path_end,
                   trajectory.reference.is_closed, trajectory.reference.interpolation_periodic,
                   trajectory.reference._interpolation_periodic, trajectory.station_map.is_closed)).encode())
    return h.hexdigest()


def propose_curvature_guards(trajectory: OffsetTrajectory, physical_intervals: Any, *,
                              settings: CurvatureProposalSettings | None = None,
                              retained_stations: Any = ()) -> dict[str, Any]:
    """Return sampled endpoint/local-extremum candidates with bounded work.

    This numerical scout makes no claim to certified extrema, global curvature
    bounds, or vehicle feasibility.
    """
    report: dict[str, Any] = {
        "status": "not_proven", "reason": "input_error", "completed": False,
        "finite_public_eval_not_certified": True,
        "accepted": False, "all_accepted": False, "continuous_extrema_certified": False,
        "physical_feasibility_certified": False, "global_minimum_proven": False,
        "joint_line_optimized": False, "event_qualified": False,
        "U3_qualified": False, "policy_approved": False,
        "physical_global_minimum": False, "joint_line_optimum": False,
        "event_coverage": False, "u3_coverage": False,
        "policy_approval": False, "physical_guard_stations": [],
        "elapsed_wall_seconds": 0.0, "attempted_intervals": 0,
        "method": "sampled_local_extremum_proposals_not_certified",
        "proposals": [], "evaluations": 0, "optimizer_calls": 0,
        "source_fingerprint_before": None, "source_fingerprint_after": None,
        "settings": None,
    }
    call_started = _monotonic()
    try:
        if type(trajectory) is not OffsetTrajectory:
            raise TypeError("trajectory must be exact OffsetTrajectory")
        cfg = _settings(settings)
        report["settings"] = {"maximum_wall_seconds": float(cfg.maximum_wall_seconds),
            "maximum_evaluations": int(cfg.maximum_evaluations),
            "maximum_intervals": int(cfg.maximum_intervals),
            "maximum_candidates": int(cfg.maximum_candidates),
            "scout_points": int(cfg.scout_points),
            "maximum_optimizer_iterations": int(cfg.maximum_optimizer_iterations),
            "station_tolerance": float(cfg.station_tolerance)}
        raw = np.asarray(physical_intervals, dtype=object)
        if raw.ndim != 2 or raw.shape[1] != 2 or raw.shape[0] == 0:
            raise ValueError("physical_intervals must be nonempty Nx2")
        if raw.shape[0] > cfg.maximum_intervals:
            raise ValueError("maximum_intervals exceeded")
        if any(not _real(v) or not math.isfinite(float(v)) for v in raw.flat):
            raise ValueError("interval stations must be finite nonboolean reals")
        intervals = np.asarray(raw, dtype=float)
        sm = trajectory.station_map
        if np.any(intervals[:, 0] < sm.path_start) or np.any(intervals[:, 1] > sm.path_end) or np.any(intervals[:, 0] >= intervals[:, 1]):
            raise ValueError("physical intervals must be increasing and within map domain")
        rr = np.asarray(retained_stations, dtype=object)
        if rr.ndim != 1:
            raise ValueError("retained_stations must be 1D")
        if rr.size > cfg.maximum_candidates:
            raise ValueError("retained station candidate budget exceeded")
        if any(not _real(v) or not math.isfinite(float(v)) for v in rr):
            raise ValueError("retained stations must be finite nonboolean reals")
        retained = np.asarray(rr, dtype=float)
        if np.any(retained < sm.path_start) or np.any(retained > sm.path_end):
            raise ValueError("retained station outside physical domain")
        retained = np.unique(retained)
        report["source_fingerprint_before"] = _fingerprint(trajectory)
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        return report

    start = call_started
    candidates: dict[float, dict[str, Any]] = {}
    def add(station: float, source: float, curvature: float, kind: str, index: int) -> None:
        if station not in candidates and len(candidates) >= cfg.maximum_candidates:
            raise RuntimeError("candidate_budget_exceeded")
        candidates.setdefault(station, {"physical_station": station,
            "computational_source_station": source, "curvature": curvature,
            "proposal_kind": kind, "physical_interval_index": index})

    error_reason = None
    try:
        for station in retained:
            if _monotonic() - start >= cfg.maximum_wall_seconds:
                raise RuntimeError("wall_budget_exceeded")
            source = float(sm.to_reference(float(station)))
            if _monotonic() - start >= cfg.maximum_wall_seconds:
                raise RuntimeError("wall_budget_exceeded")
            if not math.isfinite(source) or source < trajectory.reference.start or source > trajectory.reference.end:
                raise RuntimeError("source_station_outside_reference_domain")
            add(float(station), source, None, "retained", -1)
        def evaluate(ell: float, index: int) -> float:
            if _monotonic() - start >= cfg.maximum_wall_seconds:
                raise RuntimeError("wall_budget_exceeded")
            if report["evaluations"] >= cfg.maximum_evaluations:
                raise RuntimeError("evaluation_budget_exceeded")
            report["evaluations"] += 1
            source = float(sm.to_reference(float(ell)))
            if _monotonic() - start >= cfg.maximum_wall_seconds:
                raise RuntimeError("wall_budget_exceeded")
            if not math.isfinite(source) or source < trajectory.reference.start or source > trajectory.reference.end:
                raise RuntimeError("source_station_outside_reference_domain")
            result = trajectory.evaluate(source)
            if _monotonic() - start >= cfg.maximum_wall_seconds:
                raise RuntimeError("wall_budget_exceeded")
            k = np.asarray(result.curvature)
            metric = np.asarray(result.metric)
            if k.ndim != 0 or metric.ndim != 0 or not np.isfinite(k.item()) or not np.isfinite(metric.item()) or metric.item() <= 0:
                raise RuntimeError("invalid_geometry_result")
            return float(k.item())

        for idx, (left, right) in enumerate(intervals):
            report["attempted_intervals"] = idx + 1
            xs = np.linspace(float(left), float(right), cfg.scout_points)
            ks = [evaluate(float(x), idx) for x in xs]
            for pos, kval in ((float(xs[0]), ks[0]), (float(xs[-1]), ks[-1])):
                if _monotonic() - start >= cfg.maximum_wall_seconds:
                    raise RuntimeError("wall_budget_exceeded")
                mapped = float(sm.to_reference(pos))
                if _monotonic() - start >= cfg.maximum_wall_seconds:
                    raise RuntimeError("wall_budget_exceeded")
                if not math.isfinite(mapped) or mapped < trajectory.reference.start or mapped > trajectory.reference.end:
                    raise RuntimeError("source_station_outside_reference_domain")
                add(pos, mapped, kval, "endpoint", idx)
            for sign, kind in ((1.0, "local_min"), (-1.0, "local_max")):
                vals = np.asarray(ks) * sign
                for j in range(1, len(xs)-1):
                    if vals[j] <= vals[j-1] and vals[j] <= vals[j+1] and not (vals[j] == vals[j-1] == vals[j+1]):
                        report["optimizer_calls"] += 1
                        def objective(x: float) -> float:
                            return sign * evaluate(float(x), idx)
                        opt = minimize_scalar(objective, bounds=(float(xs[j-1]), float(xs[j+1])), method="bounded",
                            options={"xatol": float(cfg.station_tolerance), "maxiter": int(cfg.maximum_optimizer_iterations)})
                        if (not bool(getattr(opt, "success", False)) or not _real(getattr(opt, "x", None))
                                or not math.isfinite(float(opt.x)) or not xs[j-1] <= float(opt.x) <= xs[j+1]
                                or not _real(getattr(opt, "fun", None)) or not math.isfinite(float(opt.fun))):
                            raise RuntimeError("optimizer_failed")
                        # Always evaluate final point through guarded public evaluator.
                        value = evaluate(float(opt.x), idx)
                        if _monotonic() - start >= cfg.maximum_wall_seconds:
                            raise RuntimeError("wall_budget_exceeded")
                        mapped = float(sm.to_reference(float(opt.x)))
                        if _monotonic() - start >= cfg.maximum_wall_seconds:
                            raise RuntimeError("wall_budget_exceeded")
                        if not math.isfinite(mapped) or mapped < trajectory.reference.start or mapped > trajectory.reference.end:
                            raise RuntimeError("source_station_outside_reference_domain")
                        add(float(opt.x), mapped, value, kind, idx)
        report["source_fingerprint_after"] = _fingerprint(trajectory)
        if report["source_fingerprint_after"] != report["source_fingerprint_before"]:
            raise RuntimeError("source_fingerprint_mismatch")
        if _monotonic() - start >= cfg.maximum_wall_seconds:
            raise RuntimeError("wall_budget_exceeded")
        report["completed"] = True
        report["status"] = "proposed"
        report["reason"] = None
    except Exception as exc:
        error_reason = str(exc) or type(exc).__name__
        report["reason"] = error_reason
        try:
            report["source_fingerprint_after"] = _fingerprint(trajectory)
            if report["source_fingerprint_after"] != report["source_fingerprint_before"]:
                report["reason"] = "source_fingerprint_mismatch"
        except Exception:
            pass
    report["proposals"] = sorted(candidates.values(), key=lambda row: row["physical_station"])
    report["physical_guard_stations"] = [row["physical_station"] for row in report["proposals"]]
    report["elapsed_wall_seconds"] = max(0.0, float(_monotonic() - start))
    if report["completed"] and _monotonic() - start >= cfg.maximum_wall_seconds:
        report["completed"] = False
        report["status"] = "not_proven"
        report["reason"] = "wall_budget_exceeded"
    return report
