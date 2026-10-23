"""Bounded whole-artifact aggregation for the cached-polynomial chart check.

This audits stored chart mathematics over every represented source element. It is
not physical, vehicle, event, solver-artifact, or policy qualification.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import hashlib
import json
import math
from fractions import Fraction
import time
from numbers import Real
from pathlib import Path

import numpy as np

from simplified_vehicle_models.formulations.signed_capability import SignedTransientCapability
from simplified_vehicle_models.solvers.artifact_chart_check import (
    _snapshot, check_artifact_chart_element,
)
from simplified_vehicle_models.solvers.chart_element_certificate import ChartElementSettings
from simplified_vehicle_models.solvers.trajectory_artifact import LoadedTrajectory, load_trajectory_artifact

_monotonic = time.monotonic


@dataclass(frozen=True, slots=True)
class ArtifactAuditSettings:
    maximum_wall_seconds: float = 180.0
    maximum_elements: int = 4096
    stop_on_unproven: bool = False


def _chart_fingerprint(surface):
    h = hashlib.sha256()
    h.update(str(surface.representation).encode()+b"\0")
    h.update(str(surface.provenance_hash).encode()+b"\0")
    h.update(json.dumps({"speed_floor": float(surface.speed_floor),
                         "build_settings": surface.build_settings,
                         "validation": surface.validation}, sort_keys=True,
                         separators=(",", ":"), allow_nan=False, default=str).encode())
    for name in ("u", "curvature", "acceleration_max", "acceleration_min", "domain_speed_values"):
        a = np.asarray(getattr(surface, name))
        if a.dtype != np.dtype("float64") or not np.all(np.isfinite(a)):
            raise ValueError(f"chart {name} must remain finite binary64")
        h.update(name.encode()+b"\0")
        h.update(np.asarray(a.shape, dtype="<i8").tobytes())
        h.update(np.asarray(a, dtype="<f8").tobytes(order="C"))
    return h.hexdigest()


def _jsonsafe(value):
    if isinstance(value, dict):
        return {str(k): _jsonsafe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonsafe(v) for v in value]
    if isinstance(value, np.generic):
        return _jsonsafe(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def _report(reason, started, n=0, rows=(), unattempted=(), *, clock=None, wall_limit=None, **extra):
    rows = list(rows)
    clock = time.monotonic if clock is None else clock
    proven = sum(bool(r.get("chart_constraints_proven")) for r in rows)
    ok = reason == "all_elements_chart_constraints_proven" and n > 0 and proven == n and not unattempted
    out = {"status": "all_elements_chart_constraints_proven" if ok else "not_proven",
           "all_elements_chart_constraints_proven": bool(ok),
           "reason": reason, "binding_semantics": "binary64_cached_node_Hermite_Lagrange_real_extension_v1",
           "scope": "stored_chart_mathematics_over_all_source_elements_only",
           "finite_public_eval_not_certified": True,
           "requested_element_count": int(n), "attempted_element_count": len(rows),
           "proven_element_count": int(proven),
           "unproven_element_indices": [int(r["element_index"]) for r in rows if not r.get("chart_constraints_proven")],
           "unattempted_element_indices": list(unattempted), "element_results": rows,
           "elapsed_wall_seconds": float(max(0., clock()-started)),
           "accepted": False, "solver_artifact_qualified": False,
           "physical_feasibility_certified": False, "original_vehicle_validated": False,
           "event_qualified": False, "U3_qualified": False, "policy_approved": False}
    out.update(extra)
    result = _jsonsafe(out)
    # Include report assembly and JSON normalization in the global deadline.
    elapsed = max(0., clock()-started)
    result["elapsed_wall_seconds"] = float(elapsed)
    if (result.get("all_elements_chart_constraints_proven") is True
            and wall_limit is not None and elapsed >= wall_limit):
        result.update(status="not_proven", all_elements_chart_constraints_proven=False,
                      reason="wall_budget_exhausted")
    return result


def _exact_ratio(value):
    q = value if isinstance(value, Fraction) else Fraction.from_float(float(value))
    return {"numerator": q.numerator, "denominator": q.denominator}


def audit_artifact_chart(loaded_or_path, surface, *, settings=None, element_settings=None,
                         speed_domain_margin_mps=0., minimum_fold=.2,
                         maximum_speed_rate_defect=1e-6) -> dict:
    """Audit all source elements using the canonical single-element checker."""
    clock = _monotonic
    started = clock()
    cfg = ArtifactAuditSettings() if settings is None else settings
    if type(cfg) is not ArtifactAuditSettings:
        return _report("invalid_settings", started, clock=clock)
    try:
        wall_limit = float(cfg.maximum_wall_seconds)
    except (TypeError, ValueError, OverflowError):
        wall_limit = math.nan
    if (isinstance(cfg.maximum_elements, bool) or not isinstance(cfg.maximum_elements, int)
            or cfg.maximum_elements <= 0 or type(cfg.stop_on_unproven) is not bool
            or isinstance(cfg.maximum_wall_seconds, bool) or not isinstance(cfg.maximum_wall_seconds, Real)
            or not math.isfinite(wall_limit) or wall_limit <= 0):
        return _report("invalid_settings", started, clock=clock)
    if element_settings is not None and type(element_settings) is not ChartElementSettings:
        return _report("invalid_element_settings", started, clock=clock)
    try:
        for name, value in (("speed_domain_margin_mps", speed_domain_margin_mps),
                            ("minimum_fold", minimum_fold),
                            ("maximum_speed_rate_defect", maximum_speed_rate_defect)):
            if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(float(value)):
                raise ValueError(f"{name} must be a finite real non-bool")
        if speed_domain_margin_mps < 0 or minimum_fold < 0 or maximum_speed_rate_defect < 0:
            raise ValueError("criteria must be nonnegative")
        if isinstance(loaded_or_path, (str, Path)):
            loaded = load_trajectory_artifact(loaded_or_path)
        elif type(loaded_or_path) is LoadedTrajectory:
            loaded = loaded_or_path
        else:
            raise TypeError("expected a LoadedTrajectory or artifact path")
        if type(surface) is not SignedTransientCapability:
            raise TypeError("unsupported chart class")
        source_arrays, source_fp = _snapshot(loaded)
        chart_fp = _chart_fingerprint(surface)
        mesh = source_arrays["node_station"]
        n = int(len(mesh)-1)
        if n <= 0:
            return _report("empty_source_mesh", started, max(n, 0), source_fingerprint_sha256=source_fp,
                           chart_fingerprint_sha256=chart_fp, clock=clock)
        if n > cfg.maximum_elements:
            return _report("source_element_budget", started, n,
                           unattempted=range(n), source_fingerprint_sha256=source_fp,
                           chart_fingerprint_sha256=chart_fp, maximum_elements=cfg.maximum_elements, clock=clock)
        base = ChartElementSettings() if element_settings is None else element_settings
        # Validate settings through the exact checker settings validator before dispatch.
        from simplified_vehicle_models.solvers.chart_element_certificate import _settings
        _settings(base)
    except Exception as exc:
        return _report(f"input_error:{type(exc).__name__}:{exc}", started, clock=clock)

    rows = []
    failure_reason = None
    for i in range(n):
        if clock() - started >= wall_limit:
            failure_reason = "wall_budget_exhausted"
            break
        try:
            _, current_source = _snapshot(loaded)
            current_chart = _chart_fingerprint(surface)
            if current_source != source_fp:
                failure_reason = "source_changed"
                break
            if current_chart != chart_fp:
                failure_reason = "chart_changed"
                break
            remaining = wall_limit - (clock()-started)
            if remaining <= 0:
                failure_reason = "wall_budget_exhausted"
                break
            child_settings = replace(base, maximum_wall_seconds=min(float(base.maximum_wall_seconds), remaining))
            child = check_artifact_chart_element(loaded, surface, i, settings=child_settings,
                speed_domain_margin_mps=speed_domain_margin_mps, minimum_fold=minimum_fold,
                maximum_speed_rate_defect=maximum_speed_rate_defect)
            _, after_source = _snapshot(loaded)
            after_chart = _chart_fingerprint(surface)
            mutated_source = after_source != source_fp
            mutated_chart = after_chart != chart_fp
            elapsed = clock()-started
            row = dict(child) if isinstance(child, dict) else {"status": "not_proven", "reason": "invalid_element_result"}
            row["element_index"] = i
            if row.get("chart_constraints_proven") is True and row.get("input_fingerprint_sha256") != source_fp:
                row.update(chart_constraints_proven=False, status="not_proven", reason="element_source_fingerprint_mismatch")
                failure_reason = "element_source_fingerprint_mismatch"
            elif row.get("chart_constraints_proven") is not True:
                row.update(chart_constraints_proven=False, status="not_proven")
            if mutated_source:
                row.update(chart_constraints_proven=False, status="not_proven", reason="source_changed")
                failure_reason = "source_changed"
            elif mutated_chart:
                row.update(chart_constraints_proven=False, status="not_proven", reason="chart_changed")
                failure_reason = "chart_changed"
            elif elapsed >= wall_limit:
                row.update(chart_constraints_proven=False, status="not_proven", reason="wall_budget_exhausted")
                failure_reason = "wall_budget_exhausted"
            elif not row.get("chart_constraints_proven") and failure_reason is None:
                failure_reason = "elements_not_proven"
            rows.append(row)
            if failure_reason in ("source_changed", "chart_changed", "wall_budget_exhausted"):
                break
            if failure_reason and cfg.stop_on_unproven:
                break
        except Exception as exc:
            rows.append({"element_index": i, "status": "not_proven", "chart_constraints_proven": False,
                         "reason": f"element_exception:{type(exc).__name__}:{exc}"})
            failure_reason = "elements_not_proven"
            if cfg.stop_on_unproven:
                break
    try:
        _, final_source = _snapshot(loaded)
        final_chart = _chart_fingerprint(surface)
        if final_source != source_fp:
            failure_reason = "source_changed"
        elif final_chart != chart_fp:
            failure_reason = "chart_changed"
        elif clock()-started >= wall_limit:
            failure_reason = "wall_budget_exhausted"
    except Exception as exc:
        failure_reason = f"final_identity_error:{type(exc).__name__}:{exc}"
    unattempted = list(range(len(rows), n))
    if failure_reason is None and (len(rows) != n or any(not r.get("chart_constraints_proven") for r in rows)):
        failure_reason = "elements_not_proven"
    reason = "all_elements_chart_constraints_proven" if failure_reason is None else failure_reason
    return _report(reason, started, n, rows, unattempted, clock=clock, wall_limit=wall_limit,
                   source_fingerprint_sha256=source_fp, chart_fingerprint_sha256=chart_fp,
                   artifact_sha256=str(loaded.source_sha256),
                   artifact_file_identity_reverified=isinstance(loaded_or_path, (str, Path)),
                   criteria_exact={"speed_domain_margin_mps": _exact_ratio(speed_domain_margin_mps),
                                   "minimum_fold": _exact_ratio(minimum_fold),
                                   "maximum_speed_rate_defect": _exact_ratio(maximum_speed_rate_defect)},
                   maximum_wall_seconds=wall_limit, maximum_elements=cfg.maximum_elements)
