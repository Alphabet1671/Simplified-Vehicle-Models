"""Sampled, diagnostic-only comparison of loaded track and fitted geometry."""
from __future__ import annotations

import math

import numpy as np


_MAX_SAMPLES = 1000
_DEFAULT_PAIR_BUDGET = 50_000


def _finite(value):
    """Convert a scalar to JSON-safe float, or explicit null."""
    value = float(value)
    return value if math.isfinite(value) else None


def _range(values, unit):
    values = np.asarray(values, dtype=float)
    finite = values[np.isfinite(values)]
    if not finite.size:
        return {"min_" + unit: None, "max_" + unit: None, "unavailable": "no finite samples"}
    return {"min_" + unit: _finite(np.min(finite)), "max_" + unit: _finite(np.max(finite)),
            "nonfinite_samples": int(values.size - finite.size)}


def _errors(values, stations, unit):
    values = np.asarray(values, dtype=float)
    finite = np.isfinite(values)
    if not np.any(finite):
        return {"max_abs_" + unit: None, "max_" + unit: None, "rms_" + unit: None, "worst_station_m": None,
                "unavailable": "no finite comparison samples"}
    ids = np.flatnonzero(finite)
    worst = ids[int(np.argmax(np.abs(values[finite])))]
    return {"max_abs_" + unit: _finite(np.max(np.abs(values[finite]))),
            "max_" + unit: _finite(np.max(np.abs(values[finite]))),
            "rms_" + unit: _finite(np.sqrt(np.mean(values[finite] ** 2))),
            "worst_station_m": _finite(stations[worst]),
            "nonfinite_samples": int(values.size - ids.size)}


def _segments_cross(a, b, c, d):
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    return o1 * o2 < 0.0 and o3 * o4 < 0.0


def audit_track_geometry(track, reference, corridor, *, sample_budget=_MAX_SAMPLES,
                         crossing_pair_budget=_DEFAULT_PAIR_BUDGET) -> dict:
    """Compare source nodes and the actual loaded fit; never an acceptance test.

    Units: station/XY/width in metres, heading in radians, curvature in 1/m,
    metric in metres per source metre. Pair testing is capped independently.
    """
    if isinstance(sample_budget, bool) or not isinstance(sample_budget, int) or sample_budget < 2:
        raise ValueError("sample_budget must be an integer >= 2")
    if isinstance(crossing_pair_budget, bool) or not isinstance(crossing_pair_budget, int) or crossing_pair_budget < 0:
        raise ValueError("crossing_pair_budget must be a nonnegative integer")
    coordinates = getattr(track, "path_coordinates", None)
    start, end = float(reference.start), float(reference.end)
    source_s = np.asarray(getattr(track, "distance", ()), dtype=float)
    source_k = np.asarray(getattr(track, "curvature", ()), dtype=float)
    sample_count = min(max(int(sample_budget), 2), _MAX_SAMPLES)
    stations = np.linspace(start, end, sample_count)
    evaluation = reference.evaluate(stations)
    fit_xy = np.column_stack((evaluation.north, evaluation.east))
    fit_k = np.asarray(evaluation.curvature, dtype=float)
    fit_h = np.asarray(evaluation.heading, dtype=float)
    source_xy_error = source_heading_error = source_curvature_error = None
    source_heading = None
    if coordinates is not None:
        source_x = np.asarray(coordinates.x_north, dtype=float)
        source_y = np.asarray(coordinates.y_east, dtype=float)
        source_heading = np.asarray(getattr(coordinates, "heading", ()), dtype=float)
        if source_x.shape == source_s.shape and source_y.shape == source_s.shape:
            at_nodes = reference.evaluate(source_s)
            source_xy_error = np.hypot(np.asarray(at_nodes.north) - source_x, np.asarray(at_nodes.east) - source_y)
            fit_node_heading = np.asarray(at_nodes.heading)
            fit_node_k = np.asarray(at_nodes.curvature)
            if source_heading.shape == source_s.shape:
                source_heading_error = (fit_node_heading - source_heading + np.pi) % (2*np.pi) - np.pi
            if source_k.shape == source_s.shape:
                source_curvature_error = fit_node_k - source_k
    gaps = np.diff(source_s) if source_s.size > 1 else np.array([])
    # Keep the terminal sample to include the final closing segment; exclude only
    # its trivial adjacency with the first segment during pair enumeration.
    xy = fit_xy
    nseg = max(0, len(xy) - 1)
    pairs_examined = crossing_count = 0
    crossing_records = []
    truncated = False
    for i in range(nseg):
        for j in range(i + 2, nseg):
            if bool(getattr(track, "is_closed", False)) and i == 0 and j == nseg - 1:
                continue
            if pairs_examined >= crossing_pair_budget:
                truncated = True
                break
            pairs_examined += 1
            proper = _segments_cross(xy[i], xy[i + 1], xy[j], xy[j + 1])
            shared_node = any(np.array_equal(p, q) for p in (xy[i], xy[i + 1]) for q in (xy[j], xy[j + 1]))
            if proper or shared_node:
                crossing_count += 1
                if len(crossing_records) < 100:
                    crossing_records.append({"station_a_m": _finite(stations[i]), "station_b_m": _finite(stations[j]),
                                             "kind": "proper_segment_crossing" if proper else "coincident_sampled_node"})
        if truncated:
            break
    width = {}
    if corridor is None:
        width = {"available": False, "provenance": None, "minimum_m": None, "maximum_m": None}
    else:
        width = {"available": True, "provenance": str(corridor.provenance),
                 "minimum_m": float(np.min(corridor.minimum)),
                 "maximum_m": float(np.max(corridor.maximum)),
                 "narrowest_left_width_m": float(-np.max(corridor.minimum)),
                 "narrowest_right_width_m": float(np.min(corridor.maximum)),
                 "pinch_station_left_m": float(corridor.distance[int(np.argmax(corridor.minimum))]),
                 "pinch_station_right_m": float(corridor.distance[int(np.argmin(corridor.maximum))]),
                 "interpretation": "point-path bounds; not vehicle-footprint containment or measured clearance"}
    seam = {"query_topology_closed": bool(getattr(reference, "is_closed", False)),
            "source_interpolation_periodic": bool(getattr(reference, "interpolation_periodic", False)),
            "position_jump_m": None, "tangent_jump_m_per_m": None,
            "heading_jump_rad": None, "curvature_jump_per_m": None,
            "unavailable": "one-sided seam values require explicit one-sided fit support"}
    if bool(getattr(reference, "is_closed", False)):
        eps = max((end - start) * 1e-8, 1e-9)
        left = reference.evaluate(end - eps)
        right = reference.evaluate(start + eps)
        seam = {"query_topology_closed": True,
                "source_interpolation_periodic": bool(getattr(reference, "interpolation_periodic", False)),
                "position_jump_m": _finite(math.hypot(left.north - right.north, left.east - right.east)),
                "tangent_jump_m_per_m": _finite(math.hypot(left.north_s - right.north_s, left.east_s - right.east_s)),
                "heading_jump_rad": _finite((left.heading - right.heading + np.pi) % (2*np.pi) - np.pi),
                "curvature_jump_per_m": _finite(left.curvature - right.curvature),
                "one_sided_probe_offset_m": _finite(eps)}
    return {
        "claim": "diagnostic_only",
        "limitations": {"sampled_not_continuous_certificate": True, "source_and_fit_not_ranked_as_truth": True,
                        "sample_budget_requested": int(sample_budget), "sample_count": int(sample_count),
                        "sample_budget_capped": sample_budget > _MAX_SAMPLES,
                        "crossing_polyline_approximation": True},
        "source": {"available": coordinates is not None,
                   "xy_fit_error": None if source_xy_error is None else _errors(source_xy_error, source_s, "m"),
                   "heading_fit_error": None if source_heading_error is None else _errors(source_heading_error, source_s, "rad"),
                   "curvature_fit_error": None if source_curvature_error is None else _errors(source_curvature_error, source_s, "per_m"),
                   "curvature": {"range": _range(source_k, "per_m")},

                   "spacing": {"minimum_m": _finite(np.min(gaps)) if gaps.size and np.all(np.isfinite(gaps)) else None,
                               "maximum_m": _finite(np.max(gaps)) if gaps.size and np.all(np.isfinite(gaps)) else None,
                               "ratio": _finite(np.max(gaps)/np.min(gaps)) if gaps.size and np.all(np.isfinite(gaps)) and np.min(gaps)>0 else None}},
        "fit": {"metric": _range(evaluation.metric, "m_per_m"), "curvature": _range(fit_k, "per_m")},
        "source_vs_fit": {"xy": None if source_xy_error is None else _errors(source_xy_error, source_s, "m"),
                          "heading_wrapped": None if source_heading_error is None else _errors(source_heading_error, source_s, "rad"),
                          "signed_curvature": None if source_curvature_error is None else _errors(source_curvature_error, source_s, "per_m")},
        "seam": seam,
        "crossings": {"count": crossing_count, "records": crossing_records,
                      "policy": "diagnostic only; same-XY distinct route stations are retained, not rejected",
                      "pairs_examined": pairs_examined, "pair_budget": int(crossing_pair_budget),
                      "truncated": truncated, "unresolved": bool(truncated),
                      "clearance_min_m": None, "clearance_unavailable": "not computed; crossing scan is not a continuous clearance proof"},
        "corridor": width,
    }
