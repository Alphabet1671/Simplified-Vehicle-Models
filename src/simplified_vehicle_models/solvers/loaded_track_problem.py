"""Loaded-data binding for the versioned track problem contract.

This is sampled execution preflight, not a continuous geometry certificate.
"""
from __future__ import annotations

import hashlib
import numpy as np

from simplified_vehicle_models.solvers.track_problem_contract import validate_problem_contract


CORRIDOR_POLICY = "legacy-abs-source-curvature-v1"


def preflight_loaded_problem(validation, *, track, reference, corridor, surface, settings,
                             mesh, constraint_sites):
    """Bind a pure contract to actual loaded track, chart and solver settings."""
    if validation is None:
        return {"status": "legacy_diagnostic", "reasons": [], "normalized_contract": None,
                "contract_hash": None, "enforcement": "historical_solver_boundary",
                "corridor_policy": CORRIDOR_POLICY,
                "sampling": "diagnostic_only_not_continuous_certificate"}
    if not isinstance(validation, dict):
        validation = validate_problem_contract(validation)
    else:
        checked = validate_problem_contract(validation.get("normalized_contract"))
        if (checked["status"] == "invalid" or checked["normalized_hash"] != validation.get("normalized_hash")
                or checked["status"] != validation.get("status")):
            validation = checked
        else:
            validation = checked
    if validation.get("status") == "invalid":
        return {"status": "invalid", "reasons": list(validation.get("reasons", ["invalid_contract"])),
                "normalized_contract": None, "contract_hash": None,
                "corridor_policy": CORRIDOR_POLICY,
                "sampling": "diagnostic_only_not_continuous_certificate"}
    normalized = validation.get("normalized_contract")
    reasons = []
    status = validation.get("status")
    if status != "supported":
        reasons.extend(validation.get("reasons", []))
    try:
        c = normalized
        if c["topology"] != bool(getattr(track, "is_closed", False)) and c["topology"] != ("closed" if getattr(track, "is_closed", False) else "open"):
            reasons.append("topology_mismatch")
        start = end = None
        start, end = float(reference.start), float(reference.end)
        source = np.asarray(reference.source_station, dtype=float)
        q = np.asarray(c["source_stations"], dtype=float)
        if (not np.all(np.isfinite(source)) or not np.all(np.isfinite(q)) or q[0] < start or q[-1] > end):
            reasons.append("source_domain_mismatch")
        if (q.shape != source.shape or not np.array_equal(q, source)):
            reasons.append("source_grid_mismatch")
        stations = np.asarray(corridor.distance, dtype=float)
        lower = np.asarray(corridor.minimum, dtype=float)
        upper = np.asarray(corridor.maximum, dtype=float)
        if (stations.ndim != 1 or lower.shape != stations.shape or upper.shape != stations.shape
                or len(stations) < 2 or not all(np.all(np.isfinite(v)) for v in (stations, lower, upper))
                or np.any(np.diff(stations) <= 0)):
            reasons.append("loaded_corridor_invalid")
        else:
            if q[0] < stations[0] or q[-1] > stations[-1]:
                reasons.append("corridor_domain_mismatch")
            else:
                actual_lo, actual_hi = np.interp(q, stations, lower), np.interp(q, stations, upper)
                if not (np.array_equal(actual_lo, np.asarray(c["lower_offset"]))
                        and np.array_equal(actual_hi, np.asarray(c["upper_offset"]))):
                    reasons.append("corridor_contract_mismatch")
                if (np.any(np.asarray(c["lower_offset"]) > 0) or np.any(np.asarray(c["upper_offset"]) < 0)
                        or np.any(np.asarray(c["upper_offset"]) <= np.asarray(c["lower_offset"]))):
                    reasons.append("invalid_corridor_width_or_centreline")
        if bool(getattr(corridor, "is_closed", False)) != bool(getattr(track, "is_closed", False)):
            reasons.append("corridor_topology_mismatch")
        declared = float(c["boundary"]["start_speed_mps"])
        if c["topology"] == "closed":
            # Current OCP only enforces periodicity; settings.start_speed is ignored.
            reasons.append("requested_boundary_not_enforced")
        else:
            floor = max(float(getattr(surface, "speed_floor", settings.minimum_speed)),
                        float(settings.minimum_speed))
            if declared < floor:
                reasons.append("start_speed_below_model_floor")
            elif float(settings.start_speed) != declared:
                reasons.append("solver_start_speed_mismatch")
            if c["boundary"]["finish_mode"] != "free":
                reasons.append("finish_boundary_not_enforced")
        for name, values in (("mesh", mesh), ("constraint_sites", constraint_sites)):
            array = np.asarray(values, dtype=float)
            if array.ndim != 1 or not np.all(np.isfinite(array)) or np.any(array < start) or np.any(array > end):
                reasons.append(f"{name}_outside_reference_domain")
        # Evaluate reference at every actual source knot and request station; this
        # catches nonfinite fit/query data without claiming between-sample regularity.
        samples = np.unique(np.r_[source, q, np.asarray(mesh, dtype=float), np.asarray(constraint_sites, dtype=float)])
        evaluation = reference.evaluate(samples)
        for field in ("north", "east", "metric", "heading", "curvature", "curvature_s"):
            if not np.all(np.isfinite(np.asarray(getattr(evaluation, field), dtype=float))):
                reasons.append("reference_nonfinite_" + field)
        if np.any(np.asarray(evaluation.metric, dtype=float) <= 0):
            reasons.append("reference_degenerate_metric")
        # Confirm the chart's actual curvature domain covers sampled reference queries.
        chart_k = np.asarray(surface.curvature, dtype=float)
        k = np.asarray(evaluation.curvature, dtype=float)
        if (chart_k.ndim != 1 or len(chart_k) < 2 or not np.all(np.isfinite(chart_k))
                or np.any(np.diff(chart_k) <= 0) or np.min(k) < chart_k[0] or np.max(k) > chart_k[-1]):
            reasons.append("sampled_reference_chart_curvature_domain_exclusion")
        chart_floor = float(getattr(surface, "speed_floor", settings.minimum_speed))
        if (not np.isfinite(chart_floor) or declared < max(chart_floor, float(settings.minimum_speed))):
            reasons.append("start_speed_below_model_floor")
        if callable(getattr(surface, "domain_speed", None)):
            try:
                start_ceiling = float(surface.domain_speed(float(k[0])))
                if not np.isfinite(start_ceiling) or declared > start_ceiling:
                    reasons.append("start_speed_above_chart_domain")
            except (ValueError, IndexError):
                reasons.append("chart_start_query_outside_domain")
        elif hasattr(surface, "speed_ceiling"):
            legacy_ceiling = np.asarray(surface.speed_ceiling, dtype=float)
            if (legacy_ceiling.ndim != 1 or legacy_ceiling.shape != chart_k.shape
                    or not np.all(np.isfinite(legacy_ceiling))):
                reasons.append("chart_speed_domain_invalid")
            elif len(k) and declared > float(np.interp(float(k[0]), chart_k, legacy_ceiling)):
                reasons.append("start_speed_above_chart_domain")
        else:
            # The unsigned legacy surface has a single global speed axis rather
            # than a curvature-dependent start-speed ceiling.
            axis = np.asarray(surface.speed, dtype=float)
            if axis.ndim != 1 or len(axis) < 2 or not np.all(np.isfinite(axis)) or np.any(np.diff(axis) <= 0):
                reasons.append("chart_speed_domain_invalid")
            elif declared > float(axis[-1]):
                reasons.append("start_speed_above_chart_domain")
    except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError) as error:
        reasons.append("loaded_preflight_error:" + type(error).__name__)
    reasons = sorted(set(reasons))
    invalid_reasons = {"source_grid_mismatch", "corridor_domain_mismatch", "corridor_contract_mismatch",
                       "topology_mismatch", "corridor_topology_mismatch", "loaded_corridor_invalid",
                       "source_domain_mismatch", "invalid_corridor_width_or_centreline",
                       "chart_speed_domain_invalid", "chart_start_query_outside_domain"}
    invalid = any(reason in invalid_reasons or reason.startswith("loaded_preflight_error") for reason in reasons)
    status_out = "invalid" if invalid else ("unsupported" if reasons else "supported")
    safe_domain = [start, end] if start is not None and end is not None else None
    return {"status": status_out,
            "reasons": reasons, "normalized_contract": normalized,
            "contract_hash": validation.get("normalized_hash"),
            "enforcement": "requested_boundary_enforced" if not reasons else "not_enforced",
            "corridor_policy": str(getattr(corridor, "source", CORRIDOR_POLICY)),
            "reference_fit": dict(getattr(reference, "fit_discrepancy", {})),
            "source_domain_m": safe_domain,
            "sampling": "diagnostic_only_not_continuous_certificate",
            "sample_count": int(len(samples)) if "samples" in locals() else 0,
            "domain_check_scope": "sampled_reference_curvature_only_not_offset_path_chart_certificate",
            "corridor_identity": {"policy": str(getattr(corridor, "provenance", CORRIDOR_POLICY)),
                                  "source_station_start_m": float(stations[0]) if "stations" in locals() and len(stations) else None,
                                  "source_station_end_m": float(stations[-1]) if "stations" in locals() and len(stations) else None,
                                  "source_station_sha256": hashlib.sha256(np.asarray(stations, dtype=np.float64).tobytes()).hexdigest() if "stations" in locals() and len(stations) else None}}
