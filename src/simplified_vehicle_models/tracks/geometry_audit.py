"""Sampled, diagnostic-only comparison of loaded track and fitted geometry."""
from __future__ import annotations

import math

import numpy as np

_MAX_SAMPLES = 1000
_DEFAULT_PAIR_BUDGET = 50_000


def _finite(value):
    value = float(value)
    return value if math.isfinite(value) else None


def _unavailable(reason, **extra):
    return {"available": False, "unavailable": str(reason), **extra}


def _range(values, unit):
    values = np.asarray(values, dtype=float)
    finite = values[np.isfinite(values)]
    if not finite.size:
        return {"min_" + unit: None, "max_" + unit: None,
                "nonfinite_samples": int(values.size), "unavailable": "no finite samples"}
    return {"min_" + unit: _finite(np.min(finite)), "max_" + unit: _finite(np.max(finite)),
            "nonfinite_samples": int(values.size - finite.size)}


def _errors(values, stations, unit):
    values, stations = np.asarray(values, dtype=float), np.asarray(stations, dtype=float)
    if values.shape != stations.shape:
        return _unavailable("comparison values and stations have mismatched shapes")
    finite = np.isfinite(values) & np.isfinite(stations)
    bad_count = int(values.size - np.count_nonzero(finite))
    if bad_count:
        return _unavailable("one or more comparison samples are nonfinite", nonfinite_samples=bad_count)
    if not values.size:
        return _unavailable("no comparison samples", nonfinite_samples=0)
    worst = int(np.argmax(np.abs(values)))
    return {"max_abs_" + unit: _finite(np.max(np.abs(values))),
            "rms_" + unit: _finite(np.sqrt(np.mean(values ** 2))),
            "worst_station_m": _finite(stations[worst]), "nonfinite_samples": 0}


def _segment_event(a, b, c, d):
    """Return intersection point and segment fractions, including endpoint touches."""
    r, s = b-a, d-c
    cross = lambda u, v: u[0]*v[1] - u[1]*v[0]
    den = cross(r, s)
    q = c-a
    scale = max(float(np.linalg.norm(r)), float(np.linalg.norm(s)), 1.0)
    if abs(den) > np.finfo(float).eps * scale * scale * 16:
        u, v = cross(q, s)/den, cross(q, r)/den
        if 0 <= u <= 1 and 0 <= v <= 1:
            return a + u*r, float(u), float(v)
        return None
    # Parallel segments intersect only if they are exactly collinear. Project
    # onto a nonzero direction and intersect closed scalar intervals; no spatial
    # tolerance is introduced. Degenerate point segments are handled explicitly.
    rr=float(r@r); ss=float(s@s)
    if rr == 0.0 and ss == 0.0:
        return (a.copy(),0.0,0.0) if np.array_equal(a,c) else None
    if rr == 0.0:
        if cross(a-c,s) != 0.0:
            return None
        v=float(np.clip((a-c)@s/ss,0.0,1.0))
        return (a.copy(),0.0,v) if np.array_equal(a,c+v*s) else None
    if cross(q,r) != 0.0:
        return None
    t0=float(q@r/rr); t1=t0+float(s@r/rr)
    lo=max(0.0,min(t0,t1)); hi=min(1.0,max(t0,t1))
    if lo > hi:
        return None
    point=a+lo*r
    v=0.0 if ss == 0.0 else float(np.clip((point-c)@s/ss,0.0,1.0))
    return point,float(lo),v


def _segment_distance(a, b, c, d):
    event = _segment_event(a, b, c, d)
    if event is not None:
        return 0.0
    def point_segment(p, x, y):
        v=y-x; den=float(v@v)
        u=0.0 if den == 0.0 else float(np.clip((p-x)@v/den,0.0,1.0))
        return float(np.linalg.norm(p-(x+u*v)))
    return min(point_segment(a,c,d), point_segment(b,c,d),
               point_segment(c,a,b), point_segment(d,a,b))


def _corridor_diagnostic(corridor):
    if corridor is None:
        return {"available": False, "provenance": None, "minimum_m": None, "maximum_m": None}
    try:
        lo=np.asarray(corridor.minimum,dtype=float); hi=np.asarray(corridor.maximum,dtype=float)
        ds=np.asarray(corridor.distance,dtype=float); provenance=str(corridor.provenance)
        if lo.ndim != 1 or hi.shape != lo.shape or ds.shape != lo.shape or not lo.size:
            return _unavailable("corridor arrays must be matching nonempty vectors", provenance=provenance)
        finite=np.isfinite(lo)&np.isfinite(hi)&np.isfinite(ds)
        if not np.all(finite):
            return _unavailable("corridor contains nonfinite samples", provenance=provenance,
                                nonfinite_samples=int(np.count_nonzero(~finite)))
        il=int(np.argmax(lo)); ir=int(np.argmin(hi))
        return {"available": True, "provenance": provenance,
                "minimum_m": _finite(np.min(lo)), "maximum_m": _finite(np.max(hi)),
                "narrowest_left_width_m": _finite(-lo[il]),
                "narrowest_right_width_m": _finite(hi[ir]),
                "pinch_station_left_m": _finite(ds[il]), "pinch_station_right_m": _finite(ds[ir]),
                "interpretation": "point-path bounds; not vehicle-footprint containment or measured clearance"}
    except (AttributeError, TypeError, ValueError, IndexError) as exc:
        return _unavailable(f"invalid corridor diagnostic input: {type(exc).__name__}")


def audit_track_geometry(track, reference, corridor, *, sample_budget=_MAX_SAMPLES,
                         crossing_pair_budget=_DEFAULT_PAIR_BUDGET) -> dict:
    """Compare source nodes and loaded fit; all outputs are diagnostic only.

    XY/stations/width/separation are metres, heading radians, curvature 1/m,
    and metric metres per source metre. Pair work is bounded; sampled chord
    clearance is not continuous, footprint, or approved physical clearance.
    """
    if isinstance(sample_budget, bool) or not isinstance(sample_budget, int) or sample_budget < 2:
        raise ValueError("sample_budget must be an integer >= 2")
    if isinstance(crossing_pair_budget, bool) or not isinstance(crossing_pair_budget, int) or crossing_pair_budget < 0:
        raise ValueError("crossing_pair_budget must be a nonnegative integer")
    start, end = float(reference.start), float(reference.end)
    sample_count = min(sample_budget, _MAX_SAMPLES)
    stations = np.linspace(start, end, sample_count)
    evaluation = reference.evaluate(stations)
    fit_xy = np.column_stack((evaluation.north, evaluation.east))
    fit_k, fit_metric = np.asarray(evaluation.curvature, float), np.asarray(evaluation.metric, float)
    coords = getattr(track, "path_coordinates", None)
    try:
        source_s = np.asarray(getattr(track, "distance", ()), dtype=float)
        source_k = np.asarray(getattr(track, "curvature", ()), dtype=float)
    except (TypeError, ValueError):
        source_s = np.array([], dtype=float)
        source_k = np.array([], dtype=float)
    source_compare = {}
    if coords is None:
        source_xy_error = source_heading_error = source_curvature_error = None
        source_compare = {key: _unavailable("source signed geometry unavailable") for key in ("xy", "heading_wrapped", "signed_curvature")}
    else:
        try:
            sx, sy = np.asarray(coords.x_north, float), np.asarray(coords.y_east, float)
            sh = np.asarray(coords.heading, float)
            shape_ok = (source_s.ndim == 1 and source_s.size >= 2 and sx.shape == sy.shape == sh.shape == source_s.shape
                        and source_k.shape == source_s.shape)
            all_finite = shape_ok and all(np.all(np.isfinite(a)) for a in (source_s,sx,sy,sh,source_k))
            if not shape_ok or not all_finite:
                reason = "source geometry arrays have mismatched shapes" if not shape_ok else "source geometry contains nonfinite samples"
                source_compare = {key: _unavailable(reason, nonfinite_samples=(int(sum(np.count_nonzero(~np.isfinite(a)) for a in (source_s,sx,sy,sh,source_k))) if shape_ok else None))
                                  for key in ("xy", "heading_wrapped", "signed_curvature")}
                source_xy_error = source_heading_error = source_curvature_error = None
            else:
                node_eval = reference.evaluate(source_s)
                source_xy_error=np.hypot(np.asarray(node_eval.north)-sx,np.asarray(node_eval.east)-sy)
                source_heading_error=(np.asarray(node_eval.heading)-sh+np.pi)%(2*np.pi)-np.pi
                source_curvature_error=np.asarray(node_eval.curvature)-source_k
                source_compare={"xy":_errors(source_xy_error,source_s,"m"),
                                "heading_wrapped":_errors(source_heading_error,source_s,"rad"),
                                "signed_curvature":_errors(source_curvature_error,source_s,"per_m")}
        except (AttributeError, TypeError, ValueError, IndexError) as exc:
            source_xy_error = source_heading_error = source_curvature_error = None
            source_compare={key:_unavailable(f"source comparison failed: {type(exc).__name__}") for key in ("xy","heading_wrapped","signed_curvature")}
    gaps=np.diff(source_s) if source_s.ndim == 1 and source_s.size>1 else np.array([])
    spacing={"minimum_m":None,"maximum_m":None,"ratio":None}
    if gaps.size and np.all(np.isfinite(gaps)) and np.all(gaps>0):
        spacing={"minimum_m":_finite(np.min(gaps)),"maximum_m":_finite(np.max(gaps)),"ratio":_finite(np.max(gaps)/np.min(gaps))}
    elif gaps.size:
        spacing={"minimum_m":None,"maximum_m":None,"ratio":None,"unavailable":"source station gaps are nonfinite or nonpositive"}
    # Pairwise sampled-chord separation and crossings share a hard pair budget.
    closed=bool(getattr(track,"is_closed",False))
    xy=fit_xy
    nseg=max(0,len(xy)-1)
    examined=0; truncated=False; events=[]; total_event_count=0; min_sep=None; min_pair=None
    # Exclude nearby route brackets cyclically for closed routes, linearly for open routes.
    local_exclusion=max(2, int(math.ceil((sample_count-1)*0.01)))
    for i in range(nseg):
        for j in range(i+2,nseg):
            route_gap = min(j-i, nseg-(j-i)) if closed else j-i
            if route_gap < local_exclusion:
                continue
            if examined>=crossing_pair_budget:
                truncated=True; break
            examined+=1
            dist=_segment_distance(xy[i],xy[i+1],xy[j],xy[j+1])
            if min_sep is None or dist<min_sep:
                min_sep=dist; min_pair=(i,j)
            event=_segment_event(xy[i],xy[i+1],xy[j],xy[j+1])
            if event is not None:
                total_event_count += 1
                point,u,v=event
                si=float(stations[i]+u*(stations[i+1]-stations[i]))
                sj=float(stations[j]+v*(stations[j+1]-stations[j]))
                if len(events) < 100:
                    events.append({"station_a_m":_finite(si),"station_b_m":_finite(sj),
                               "station_a_bracket_m":[_finite(stations[i]),_finite(stations[i+1])],
                               "station_b_bracket_m":[_finite(stations[j]),_finite(stations[j+1])],
                               "north_m":_finite(point[0]),"east_m":_finite(point[1]),
                               "fraction_a":_finite(u),"fraction_b":_finite(v),
                               "kind":"sampled_polyline_intersection"})
        if truncated: break
    if min_pair is None: separation=_unavailable("no nonlocal segment pairs tested",pairs_examined=examined)
    else:
        i,j=min_pair
        separation={"available":True,"minimum_m":_finite(min_sep),
                    "segment_a_station_bracket_m":[_finite(stations[i]),_finite(stations[i+1])],
                    "segment_b_station_bracket_m":[_finite(stations[j]),_finite(stations[j+1])],
                    "pairs_examined":examined,"not_continuous_or_footprint_clearance":True}
    width=_corridor_diagnostic(corridor)
    seam={"query_topology_closed":bool(getattr(reference,"is_closed",False)),
          "source_interpolation_periodic":bool(getattr(reference,"interpolation_periodic",False)),
          "position_jump_m":None,"tangent_jump_m_per_m":None,"heading_jump_rad":None,
          "curvature_jump_per_m":None,"unavailable":"one-sided seam probe unavailable"}
    if bool(getattr(reference,"is_closed",False)):
        eps=max((end-start)*1e-8,1e-9); left=reference.evaluate(end-eps); right=reference.evaluate(start+eps)
        seam={"query_topology_closed":True,"source_interpolation_periodic":bool(getattr(reference,"interpolation_periodic",False)),
              "position_jump_m":_finite(math.hypot(left.north-right.north,left.east-right.east)),
              "tangent_jump_m_per_m":_finite(math.hypot(left.north_s-right.north_s,left.east_s-right.east_s)),
              "heading_jump_rad":_finite((left.heading-right.heading+np.pi)%(2*np.pi)-np.pi),
              "curvature_jump_per_m":_finite(left.curvature-right.curvature),
              "probe_kind":"finite_offset_difference, not one-sided limit", "one_sided_probe_offset_m":_finite(eps)}
    return {"claim":"diagnostic_only",
            "limitations":{"sampled_not_continuous_certificate":True,"source_and_fit_not_ranked_as_truth":True,
                           "sample_budget_requested":sample_budget,"sample_count":sample_count,
                           "sample_budget_capped":sample_budget>_MAX_SAMPLES,
                           "crossing_polyline_approximation":True,"nonlocal_pair_budget_truncated":truncated,
                           "nearby_route_exclusion_fraction":local_exclusion/(sample_count-1)},
            "source":{"available":coords is not None,
                      "curvature":{"range":_range(source_k,"per_m")},"spacing":spacing},
            "fit":{"metric":_range(fit_metric,"m_per_m"),"curvature":_range(fit_k,"per_m")},
            "source_vs_fit":source_compare,"seam":seam,
            "crossings":{"count":total_event_count,"records":events,
                          "records_truncated":total_event_count>len(events),
                         "policy":"diagnostic only; distinct route stations at shared XY are retained",
                         "route_order_policy":{
                             "identity":"source_station",
                             "nonadjacent_xy_coincidence_or_crossing":"diagnostic_information_only",
                             "chart_rejection":"not_inferred",
                             "branch_switch":"not_inferred",
                             "physical_conclusion":"not_inferred",
                             "qualification_conclusion":"not_inferred",
                             "sampling":"sampled_polyline_approximation_not_continuous_geometry",
                             "pair_budget":"bounded_pair_scan_may_truncate_and_mark_unresolved",
                         },
                         "pairs_examined":examined,"pair_budget":crossing_pair_budget,
                         "truncated":truncated,"unresolved":truncated,
                         "minimum_nonlocal_separation_m":separation,
                         "clearance_min_m":None,"clearance_unavailable":"separation is sampled chord geometry, not continuous or footprint clearance"},
            "corridor":width}
