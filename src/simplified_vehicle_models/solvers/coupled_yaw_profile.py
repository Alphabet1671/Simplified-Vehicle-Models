"""Adaptive sampled candidate generation and cyclic yaw-profile solve."""
from __future__ import annotations

import math
import numpy as np

from simplified_vehicle_models.solvers.yaw_aware_profile import YawPath
from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings, YawProfileSolveError
from simplified_vehicle_models.solvers.yaw_profile_map import _MapInterpolator
from simplified_vehicle_models.solvers.yaw_profile_transition import transition_intervals, edge_evidence
from simplified_vehicle_models.solvers.layered_q_graph import pack_layered_q_graph, solve_open_graph, solve_closed_graph
from simplified_vehicle_models.solvers.yaw_profile_validation import validate_profile


def _domain(path, capability, maximum_speed):
    interp = _MapInterpolator(capability)
    vmax = min(float(maximum_speed), float(capability.speed[-1]))
    lo, hi = float(capability.speed[0])**2, vmax*vmax
    bounds=[]
    for k in path.curvature:
        a,b=lo,hi
        if k > 0: b=min(b,float(capability.lateral_acceleration[-1])/k)
        elif k < 0: a=max(a,float(capability.lateral_acceleration[0])/k)
        bounds.append((max(lo,a),max(max(lo,a),b)))
    return bounds,interp


def solve_coupled_profile(path, capability, *, boundary, maximum_speed,
                          settings: CoupledProfileSettings, warm_start_q=None) -> dict:
    """Find a strictly validated sampled profile; graph optimality is over generated edges."""
    if not isinstance(path,YawPath): raise TypeError("path must be YawPath")
    if not isinstance(settings,CoupledProfileSettings): raise TypeError("settings must be CoupledProfileSettings")
    if boundary not in ("standing","flying"): raise ValueError("boundary must be standing or flying")
    if boundary == "standing" and (path.is_closed or capability.speed[0] != 0):
        raise YawProfileSolveError("invalid_boundary","standing requires open path and map speed zero",{})
    if boundary == "flying" and not path.is_closed:
        raise YawProfileSolveError("invalid_boundary","flying requires closed path",{})
    if not math.isfinite(maximum_speed) or maximum_speed <= 0: raise ValueError("maximum_speed must be positive")
    n=len(path.distance); segs=n-1
    domains,interp=_domain(path,capability,maximum_speed)
    warm=None if warm_start_q is None else np.asarray(warm_start_q,float)
    if warm is not None and (warm.shape!=(n,) or not np.all(np.isfinite(warm))): raise ValueError("warm_start_q must match path stations")
    layers=[]
    for i,(lo,hi) in enumerate(domains):
        qs=list(np.linspace(lo,hi,settings.q_seed_count))
        qs.extend(float(v*v) for v in capability.speed if lo <= v*v <= hi and v <= maximum_speed)
        if warm is not None and lo <= warm[i]**2 <= hi: qs.append(float(warm[i]**2))
        k=float(path.curvature[i])
        if abs(k)>0:
            qs.extend(float(y/k) for y in capability.lateral_acceleration if lo <= y/k <= hi)
        if boundary == "standing" and i==0: qs=[0.]
        qs=sorted(set(q for q in qs if lo<=q<=hi))
        layers.append(qs)
    if boundary == "flying":
        # The duplicated endpoint represents one periodic station; intersect its domain/candidates.
        common=sorted(set(layers[0]).intersection(layers[-1]))
        if not common:
            low=max(domains[0][0],domains[-1][0]); high=min(domains[0][1],domains[-1][1])
            common=list(np.linspace(low,high,min(8,settings.max_seam_anchors))) if high>=low else []
        layers[0]=layers[-1]=common
    refinement_rounds=0; incumbent=None; previous_time=None; stable=0; budget_reason=None; graph_result=None
    for round_index in range(settings.max_refinement_rounds):
        refinement_rounds=round_index+1
        counts=np.array([len(x) for x in layers],dtype=np.int64)
        if np.any(counts>settings.max_candidates_per_station) or int(counts.sum())>settings.max_total_nodes:
            budget_reason="candidate_budget"; break
        edges=[]
        for i in range(segs):
            src_layer=layers[i]; dst=layers[i+1]
            for j,q0 in enumerate(src_layer):
                try:
                    intervals=transition_intervals(path,capability,i,q0,"forward",domains[i+1],settings=settings)
                except ValueError as exc:
                    if "nonnegative" in str(exc):
                        continue
                    raise
                for iv in intervals:
                    if iv.upper < 0 or iv.lower > domains[i+1][1]:
                        continue
                    a=np.searchsorted(dst,max(0.,iv.lower),side="left"); b=np.searchsorted(dst,min(domains[i+1][1],iv.upper),side="right")
                    for target in range(a,b):
                        q1=dst[target]
                        ev=edge_evidence(path,capability,i,q0,q1,settings=settings)
                        if ev["feasible"]:
                            dt=2*(path.distance[i+1]-path.distance[i])/(math.sqrt(q0)+math.sqrt(q1)) if q0+q1>0 else math.inf
                            if math.isfinite(dt) and dt>0: edges.append((int(sum(counts[:i])+j),int(sum(counts[:i+1])+target),dt))
        if len(edges)>settings.max_total_edges:
            budget_reason="edge_budget"; break
        packed=pack_layered_q_graph(counts,np.concatenate([np.asarray(x,float) for x in layers]),
            np.asarray([e[0] for e in edges],dtype=np.int64),np.asarray([e[1] for e in edges],dtype=np.int64),
            np.asarray([e[2] for e in edges],float),max_nodes=settings.max_total_nodes,max_edges=settings.max_total_edges)
        if isinstance(packed,dict): budget_reason="graph_budget"; break
        offsets=packed.node_offsets
        if boundary=="standing":
            graph_result=solve_open_graph(packed,int(offsets[0]),np.arange(offsets[-2],offsets[-1]),max_nodes=settings.max_total_nodes,max_edges=settings.max_total_edges)
        else:
            pairs=np.array([(int(offsets[0]+j),int(offsets[-2]+j)) for j in range(min(len(layers[0]),len(layers[-1])))],dtype=np.int64)
            graph_result=solve_closed_graph(packed,pairs,max_nodes=settings.max_total_nodes,max_edges=settings.max_total_edges)
        if graph_result["status"]=="ok":
            ids=graph_result["node_ids"]; candidate=packed.q[ids]
            # For closed traversal, graph includes duplicated seam; for open, direct.
            candidate_speed=np.sqrt(candidate)
            # Graph endpoints are q; public validator consumes speed. Do not
            # allow a zero-to-zero segment or any graph traceback mismatch.
            if len(candidate_speed) != n:
                graph_result={**graph_result,"traceback_node_count":len(candidate_speed)}
                budget_reason="strict_validation_failed"
                break
            validation=validate_profile(path,capability,candidate_speed,maximum_speed=maximum_speed,boundary=boundary,settings=settings)
            if validation.valid:
                incumbent=(candidate_speed.copy(),float(validation.recomputed_time),validation)
                if previous_time is not None and abs(previous_time-incumbent[1]) <= settings.refinement_time_relative_tolerance*max(1.,previous_time) and np.max(np.abs(candidate_speed-prev_speed)) < settings.refinement_speed_tolerance: stable+=1
                else: stable=0
                previous_time=incumbent[1]; prev_speed=candidate_speed.copy()
                # Adaptive insertions around incumbent transitions (strictly rechecked).
                changed=False
                for i in range(segs):
                    q0,q1=float(candidate[i]**2),float(candidate[i+1]**2)
                    iv=transition_intervals(path,capability,i,q0,"forward",domains[i+1],settings=settings)
                    for x in iv:
                        for q in (x.lower,(x.lower+x.upper)*.5,x.upper):
                            if edge_evidence(path,capability,i,q0,q,settings=settings)["feasible"] and q not in layers[i+1]: layers[i+1].append(q); changed=True
                if not changed or stable>=2: break
            else:
                graph_result={**graph_result,"validation":validation}
                budget_reason="strict_validation_failed"; break
        else:
            # Refine edges around reachable frontier via interval witnesses in both directions.
            reach=set(map(int,graph_result.get("reachable_nodes",[]))); coreach=set(map(int,graph_result.get("co_reachable_nodes",[])))
            changed=False; offsets=packed.node_offsets
            for i in range(segs):
                loid,hiid=int(offsets[i]),int(offsets[i+1]); nlo,nhi=int(offsets[i+1]),int(offsets[i+2])
                upstream=[packed.q[x] for x in range(loid,hiid) if x in reach]
                downstream=[packed.q[x] for x in range(nlo,nhi) if x in coreach]
                for q0 in upstream:
                    for iv in transition_intervals(path,capability,i,float(q0),"forward",domains[i+1],settings=settings):
                        for q in (iv.lower,(iv.lower+iv.upper)/2,iv.upper):
                            if q not in layers[i+1]: layers[i+1].append(q); changed=True
                for q1 in downstream:
                    for iv in transition_intervals(path,capability,i,float(q1),"backward",domains[i][0:2],settings=settings):
                        for q in (iv.lower,(iv.lower+iv.upper)/2,iv.upper):
                            if q not in layers[i]: layers[i].append(q); changed=True
            for x in layers: x.sort()
            if not changed: break
    if incumbent is None:
        status="budget_exceeded" if budget_reason else "no_path_at_resolution"
        raise YawProfileSolveError(status,"coupled yaw profile has no validated sampled path",{
            "graph_status":None if graph_result is None else graph_result.get("status"),"refinement_rounds":refinement_rounds,
            "candidate_counts":tuple(map(len,layers)),"budget_reason":budget_reason,"stopping_uncertainty":"candidate resolution not converged"})
    q,time_seconds,validation=incumbent
    return {"q":q*q,"time_seconds":time_seconds,"refinement_rounds":refinement_rounds,
            "diagnostics":{"status":"feasible_sampled","resolution_converged":stable>=2,"budget_reason":budget_reason,
                "candidate_counts":tuple(map(len,layers)),"graph_status":"ok","validation":validation,
                "stopping_uncertainty":None if stable>=2 else "sampled candidate resolution remains unproven",
                "edge_density":None if packed is None else len(packed.targets)/max(1,packed.n_nodes),"closed_seam_rotation":0}}
