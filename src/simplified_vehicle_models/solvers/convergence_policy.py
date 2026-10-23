"""Proposed, versioned numerical-reference accuracy policy and comparator.

This module measures disagreement between supplied trajectory and state
 evaluators; it does not certify physical prediction, control quality,
 feasibility, or readiness.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from types import MappingProxyType
from typing import Any, Mapping

import numpy as np



@dataclass(frozen=True, slots=True)
class AccuracyPolicy:
    schema: str
    version: str
    identifier: str
    claim: str
    status: str
    requirements_status: str
    operating_envelope: Mapping[str, Any]
    thresholds: Mapping[str, Any]
    derivation: Mapping[str, Any]

    def canonical_bytes(self) -> bytes:
        return json.dumps(_plain(self), sort_keys=True, separators=(",", ":"),
                          ensure_ascii=True, allow_nan=False).encode("ascii")

    @property
    def sha256(self) -> str:
        return hashlib.sha256(self.canonical_bytes()).hexdigest()


def _plain(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_plain(v) for v in value]
    if hasattr(value, "__dataclass_fields__"):
        return {name: _plain(getattr(value, name)) for name in value.__dataclass_fields__}
    return value


def _frozen_map(value: dict[str, Any]) -> Mapping[str, Any]:
    # Nested values are recursively made immutable as tuples/mapping proxies.
    def freeze(v):
        if isinstance(v, dict):
            return MappingProxyType({k: freeze(x) for k, x in v.items()})
        if isinstance(v, list):
            return tuple(freeze(x) for x in v)
        return v
    return freeze(value)


NUMERICAL_POLICY_V1 = AccuracyPolicy(
    schema="simplified_vehicle_models.numerical_accuracy_policy",
    version="1.0.0", identifier="no-tv-numerical-convergence-v1",
    claim="numerical_reference_accuracy", status="proposed_pending_independent_review",
    requirements_status="unapproved_assumptions_for_review",
    operating_envelope=_frozen_map({"speed_max_m_s": 30.0, "abs_curvature_max_per_m": 0.24,
                                   "wheelbase_m": 1.535, "outside_envelope": "not_qualified_no_clipping"}),
    thresholds=_frozen_map({
        "xy_max_m": 0.02, "offset_max_m": 0.02, "xy_rms_m": 0.005, "offset_rms_m": 0.005,
        "metric_max": 1e-4, "metric_rms": 2e-5,
        "relative_pathlength_max": 1e-4,
        "speed_max_m_s": 0.001, "speed_rms_m_s": 0.00025,
        "curvature_max_per_m": 1e-5, "curvature_rms_per_m": 2.5e-6,
        "time_absolute_s": 0.01, "time_relative": 0.001,
        "pathlength_absolute_m": 0.05, "pathlength_relative": 1e-4,
        "comparison_uncertainty_fraction": 0.1}),
    derivation=_frozen_map({
        "lateral_acceleration_budget_m_s2": 0.025,
        "speed_contribution_m_s2": 0.01440024,
        "curvature_contribution_m_s2": 0.00900060001,
        "combined_contribution_m_s2": 0.02340084001,
        "reserved_margin_m_s2": 0.00159915999,
        "speed_formula": "2*30*0.24*0.001 + 0.24*0.001^2",
        "curvature_formula": "(30.001)^2*1e-5",
        "clearance_context_m": 2.0,
        "geometry_tolerance_context": "1% of declared reference clearance; not a physical corridor waiver"}),
)


def _get(obj: Any, name: str, default=None):
    return getattr(obj, name, default)


def _source_fingerprint(reference: Any) -> str | None:
    """Fingerprint only the exact trusted repository ReferenceCurve type.

    Knot XY samples from arbitrary interpolators cannot prove equality between
    knots, so custom references require object identity in the caller.
    """
    from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve

    if type(reference) is not ReferenceCurve:
        return None
    frame = "world_X_north_Y_east_Z_down_v1"  # PathCoordinates convention.

    station = np.asarray(getattr(reference, "source_station", ()), dtype="<f8")
    if station.ndim != 1 or station.size < 2 or not np.all(np.isfinite(station)) or np.any(np.diff(station) <= 0):
        return None
    try:
        geom = reference.evaluate(station)
        north = np.asarray(geom.north, dtype="<f8")
        east = np.asarray(geom.east, dtype="<f8")
    except (AttributeError, TypeError, ValueError):
        return None
    if north.shape != station.shape or east.shape != station.shape or not np.all(np.isfinite(north)) or not np.all(np.isfinite(east)):
        return None
    digest = hashlib.sha256()
    digest.update(b"reference-source-v2\\0")
    digest.update(frame.encode("ascii"))
    digest.update(b"\\0closed=" + (b"1" if bool(getattr(reference, "is_closed", False)) else b"0"))
    digest.update(b"\\0interpolation_periodic=" +
                  (b"1" if bool(getattr(reference, "interpolation_periodic", False)) else b"0"))
    for array in (station, north, east):
        digest.update(np.asarray(array, dtype="<f8").tobytes(order="C"))
    return digest.hexdigest()


def _bad(reason: str, *, known_violation: bool = False,
         policy=NUMERICAL_POLICY_V1) -> dict[str, Any]:
    return {"policy_id": policy.identifier, "policy_sha256": policy.sha256,
            "policy_status": policy.status, "requirements_status": policy.requirements_status,
            "accepted": False, "maximum_accuracy_certified": False,
            "meets_accuracy_budget": False if known_violation else None,
            "complete": False, "reason": reason,
            "metrics": {}, "estimated_sampling_error": {}}


def compare_trajectory_states(left: Any, right: Any, *, maximum_refinements: int = 5,
                              samples_per_interval: int = 5) -> dict[str, Any]:
    """Compare vectorized exact evaluators over common source-station refinements.

    Evaluators expose ``evaluate(station_array)`` and return fields north, east,
    offset, metric, speed, curvature. Inputs also expose distance knots,
    ``trajectory.reference.source_station``, ``trajectory.spline.control_distance``,
    lap_time, and an exact station_map.to_path(station) for path length.
    """
    if isinstance(maximum_refinements, bool) or not isinstance(maximum_refinements, int) or maximum_refinements < 2:
        raise ValueError("maximum_refinements must be an integer >= 2")
    if isinstance(samples_per_interval, bool) or not isinstance(samples_per_interval, int) or samples_per_interval < 1:
        raise ValueError("samples_per_interval must be a positive integer")
    try:
        le, re = _get(left, "state_evaluator"), _get(right, "state_evaluator")
        lt = _get(left, "trajectory", _get(le, "trajectory"))
        rt = _get(right, "trajectory", _get(re, "trajectory"))
        if le is None or re is None or lt is None or rt is None:
            return _bad("missing_exact_trajectory_or_state_evaluator")
        lr, rr = lt.reference, rt.reference
        start, end = float(lr.start), float(lr.end)
        if not (math.isfinite(start) and math.isfinite(end) and end > start):
            return _bad("invalid_source_domain")
        if not math.isclose(start, float(rr.start), rel_tol=0, abs_tol=1e-10) or not math.isclose(end, float(rr.end), rel_tol=0, abs_tol=1e-10):
            return _bad("source_domain_mismatch")
        if bool(lr.is_closed) != bool(rr.is_closed):
            return _bad("periodicity_mismatch")
        if lr is not rr:
            left_source = _source_fingerprint(lr)
            right_source = _source_fingerprint(rr)
            if left_source is None or right_source is None:
                return _bad("source_identity_unverifiable_or_untrusted_interpolant")
            if left_source != right_source:
                return _bad("source_reference_mismatch")
        else:
            left_source = _source_fingerprint(lr)

        source_fingerprint = left_source if left_source is not None else "same_reference_object_unfingerprinted"
        def knots(sol, traj):
            seq = [np.asarray(sol.distance, dtype=float), np.asarray(traj.reference.source_station, dtype=float),
                   np.asarray(traj.spline.control_distance, dtype=float)]
            for a in seq:
                if a.ndim != 1 or a.size < 2 or not np.all(np.isfinite(a)):
                    raise ValueError("invalid source knots")
            return np.unique(np.concatenate(seq))
        base = np.unique(np.r_[knots(left, lt), knots(right, rt), start, end])
        base = base[(base >= start) & (base <= end)]
        if base.size < 2 or base[0] != start or base[-1] != end:
            return _bad("incomplete_common_source_domain")
        lap_times = (float(left.lap_time), float(right.lap_time))
        if not all(math.isfinite(v) and v > 0 for v in lap_times):
            return _bad("invalid_physical_lap_time")
        path_lengths = []
        for traj in (lt, rt):
            sm = getattr(traj, "station_map", None)
            if sm is None:
                return _bad("missing_exact_station_map")
            p = np.asarray(sm.to_path(np.array([start, end])), dtype=float)
            if p.shape != (2,) or not np.all(np.isfinite(p)) or p[1] <= p[0]:
                return _bad("invalid_exact_path_length")
            path_lengths.append(float(p[1] - p[0]))

        names = ("xy", "offset", "metric", "speed", "curvature")
        policy_limits = NUMERICAL_POLICY_V1.thresholds
        limits = {"xy": (policy_limits["xy_max_m"], policy_limits["xy_rms_m"]),
                  "offset": (policy_limits["offset_max_m"], policy_limits["offset_rms_m"]),
                  "metric": (policy_limits["metric_max"], policy_limits["metric_rms"]),
                  "speed": (policy_limits["speed_max_m_s"], policy_limits["speed_rms_m_s"]),
                  "curvature": (policy_limits["curvature_max_per_m"], policy_limits["curvature_rms_per_m"])}
        level_results = []
        for level in range(maximum_refinements + 1):
            subdivisions = samples_per_interval * (2 ** level)
            pieces = [np.linspace(a, b, subdivisions + 1)[:-1] for a, b in zip(base[:-1], base[1:])]
            stations = np.r_[np.concatenate(pieces), end]
            a, b = le.evaluate(stations), re.evaluate(stations)
            ga, gb = lt.evaluate(stations), rt.evaluate(stations)
            sa, sb = le.evaluate(stations), re.evaluate(stations)
            fields = {"north": (ga.north, gb.north), "east": (ga.east, gb.east),
                      "offset": (ga.offset, gb.offset), "metric": (ga.metric, gb.metric),
                      "speed": (sa["speed"], sb["speed"]),
                      "curvature": (ga.curvature, gb.curvature)}
            arrays = {}
            for key, pair in fields.items():
                x, y = (np.asarray(q, dtype=float) for q in pair)
                if x.shape != stations.shape or y.shape != stations.shape or not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
                    return _bad("nonfinite_or_malformed_evaluator_output")
                arrays[key] = (x, y)
            metric_left, metric_right = arrays["metric"]
            if np.any(metric_left <= 0) or np.any(metric_right <= 0):
                return _bad("nonpositive_metric")
            metric_mean = .5 * (metric_left + metric_right)
            speed_left, speed_right = arrays["speed"]
            curvature_left, curvature_right = arrays["curvature"]
            envelope = NUMERICAL_POLICY_V1.operating_envelope
            if (np.any(speed_left <= 0) or np.any(speed_right <= 0)
                    or np.any(speed_left > envelope["speed_max_m_s"])
                    or np.any(speed_right > envelope["speed_max_m_s"])
                    or np.any(np.abs(curvature_left) > envelope["abs_curvature_max_per_m"])
                    or np.any(np.abs(curvature_right) > envelope["abs_curvature_max_per_m"])):
                return _bad("outside_consumer_operating_envelope_not_qualified", known_violation=True)
            weight = metric_mean
            ds = np.diff(stations)
            metrics = {}
            diffs = {"xy": np.hypot(arrays["north"][0]-arrays["north"][1], arrays["east"][0]-arrays["east"][1]),
                     **{k: np.abs(arrays[k][0]-arrays[k][1]) for k in ("offset", "metric", "speed", "curvature")}}
            for name, d in diffs.items():
                denominator = float(np.sum(.5 * (weight[:-1]+weight[1:]) * ds))
                numerator = float(np.sum(.5 * (d[:-1]**2*weight[:-1] + d[1:]**2*weight[1:]) * ds))
                if not (math.isfinite(denominator) and denominator > 0 and math.isfinite(numerator)):
                    return _bad("nonfinite_integrated_metric")
                rms = math.sqrt(numerator / denominator)
                maximum = float(np.max(d))
                if not (math.isfinite(rms) and math.isfinite(maximum)):
                    return _bad("nonfinite_comparison_metric")
                metrics[name] = {"rms": rms, "max": maximum}
            level_results.append(metrics)
        last = level_results[-1]
        thresholds = {name: (limits[name][1], limits[name][0]) for name in names}
        uncertainty = {}
        changes = []
        for previous, current in zip(level_results[:-1], level_results[1:]):
            changes.append({n: {kind: abs(current[n][kind]-previous[n][kind]) for kind in ("rms", "max")} for n in names})
        # Require two successive changes to be <=10% budget for every statistic;
        # empirical residual is twice the larger of those two level changes.
        stable = len(changes) >= 2
        if stable:
            for n in names:
                for kind, idx in (("rms", 0), ("max", 1)):
                    budget = thresholds[n][idx]
                    if changes[-1][n][kind] > .1*budget or changes[-2][n][kind] > .1*budget:
                        stable = False
                    uncertainty[f"{n}_{kind}"] = 2*max(changes[-1][n][kind], changes[-2][n][kind])
        sampled_over = any(last[n][kind] > thresholds[n][idx]
                           for n in names for kind, idx in (("rms", 0), ("max", 1)))
        uncertainty_ok = stable and all(uncertainty.get(f"{n}_{kind}", math.inf) <= .1*thresholds[n][idx]
                                        and last[n][kind] + uncertainty.get(f"{n}_{kind}", math.inf) <= thresholds[n][idx]
                                        for n in names for kind, idx in (("rms", 0), ("max", 1)))
        time_abs = abs(lap_times[0]-lap_times[1]); time_rel = time_abs / max(lap_times)
        length_abs = abs(path_lengths[0]-path_lengths[1]); length_rel = length_abs / max(path_lengths)
        time_ok = time_abs <= .01 and time_rel <= .001
        length_ok = length_abs <= .05 and length_rel <= 1e-4
        non_sampling_ok = time_ok and length_ok
        known_violation = sampled_over or not non_sampling_ok
        budget_result = False if known_violation else None
        return {"policy_id": NUMERICAL_POLICY_V1.identifier, "policy_sha256": NUMERICAL_POLICY_V1.sha256,
                "policy_status": NUMERICAL_POLICY_V1.status,
                "requirements_status": NUMERICAL_POLICY_V1.requirements_status,
                "accepted": False, "maximum_accuracy_certified": False,
                "meets_accuracy_budget": budget_result,
                "sampled_metrics_within_budget": not sampled_over,
                "complete": bool(uncertainty_ok),
                "reason": ("known_sampled_or_integral_budget_violation" if known_violation else
                           "unsampled_maximum_not_certified"),
                "sampling_uncertainty_kind": "empirical_refinement_stability_diagnostic_not_formal_unsampled_bound",
                "metrics": last, "estimated_sampling_error": uncertainty,
                "adjacent_level_changes": changes[-2:], "stable_two_levels": stable,
                "lap_time": {"left_s": lap_times[0], "right_s": lap_times[1], "absolute_s": time_abs, "relative": time_rel,
                             "passes": time_ok},
                "pathlength": {"left_m": path_lengths[0], "right_m": path_lengths[1], "absolute_m": length_abs,
                               "relative": length_rel, "passes": length_ok},
                "source_interval_count": int(base.size-1), "refinement_levels": maximum_refinements+1,
                "source_fingerprint": source_fingerprint,
                "operating_envelope": dict(NUMERICAL_POLICY_V1.operating_envelope),
                "sampled_envelope_checked": True, "continuous_envelope_certified": False,
                "qualification_note": "sampled envelope only; unsampled envelope and maxima are uncertified"}
    except (AttributeError, TypeError, ValueError, OverflowError, FloatingPointError) as exc:
        return _bad(f"invalid_comparison_input:{type(exc).__name__}:{exc}")
