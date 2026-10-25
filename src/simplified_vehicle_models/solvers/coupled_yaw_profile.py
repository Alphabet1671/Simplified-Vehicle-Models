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


def _rotate_closed_path(path, rotation):
    """Rotate unique closed stations and preserve every original edge length."""
    m=len(path.distance)-1
    order=np.r_[np.arange(rotation,m),np.arange(0,rotation+1)]
    edge=np.diff(path.distance)
    lengths=np.r_[edge[rotation:],edge[:rotation]]
    distance=np.r_[0.,np.cumsum(lengths)]
    return YawPath(distance,path.curvature[order],path.curvature_rate[order],True),order


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
    original_path=path
    rotation=0
    rotate_order=np.arange(len(path.distance))
    if boundary=="flying":
        original_domains,_=_domain(path,capability,maximum_speed)
        rotation=min(range(len(original_domains)-1),key=lambda i:(original_domains[i][1],i))
        path,rotate_order=_rotate_closed_path(path,rotation)
        if warm_start_q is not None:
            warm_start_q=np.asarray(warm_start_q,float)[rotate_order]
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
    tested_anchors=[]
    if boundary == "flying":
        low=max(domains[0][0],domains[-1][0]); high=min(domains[0][1],domains[-1][1])
        if high<low:
            raise YawProfileSolveError("no_path_at_resolution","closed seam domains do not overlap",{"seam_rotation":rotation})
        initial_anchor_cap=min(8,settings.max_seam_anchors)
        anchor_set=set(np.linspace(low,high,initial_anchor_cap).tolist())
        anchor_set.update(float(v*v) for v in capability.speed if low<=v*v<=high and v<=maximum_speed)
        anchors=sorted(anchor_set)[:initial_anchor_cap]
        tested_anchors.extend(anchors)
        layers[0]=layers[-1]=anchors.copy()
    refinement_rounds=0; incumbent=None; previous_time=None; stable=0; budget_reason=None; graph_result=None
    interval_cache={}; work_queue=[]; refinement_radius=32
    cache_identity=(tuple(map(float,path.distance)),tuple(map(float,path.curvature)),tuple(map(float,path.curvature_rate)),
                    capability.source_identity,tuple(map(float,capability.speed)),tuple(map(float,capability.longitudinal_acceleration)),
                    tuple(map(float,capability.lateral_acceleration)),settings)
    def intervals_for(i,fixed,direction,bounds):
        key=(cache_identity,int(i),direction,float(fixed).hex())
        if key not in interval_cache:
            interval_cache[key]=transition_intervals(path,capability,i,float(fixed),direction,bounds,settings=settings)
        return interval_cache[key]
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
                    intervals=intervals_for(i,q0,"forward",domains[i+1])
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
                if incumbent is None or float(validation.recomputed_time) <= incumbent[1]*(1.+1e-10):
                    incumbent=(candidate_speed.copy(),float(validation.recomputed_time),validation)
                else:
                    candidate_speed=incumbent[0].copy()
                    validation=incumbent[2]
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
                    for iv in intervals_for(i,q1,"backward",domains[i][0:2]):
                        for q in (iv.lower,(iv.lower+iv.upper)/2,iv.upper):
                            if q not in layers[i]: layers[i].append(q); changed=True
            if boundary=="flying" and round_index < settings.max_refinement_rounds-1:
                # Add deterministic anchors adjacent to current best seam or failed frontiers.
                current=sorted(set(layers[0]))
                if current and len(current)<settings.max_seam_anchors:
                    center=(incumbent[0][0]**2 if incumbent is not None else current[len(current)//2])
                    neighbors=sorted(current,key=lambda a:(abs(a-center),a))[:2]
                    additions=[]
                    for a in neighbors:
                        pos=int(np.searchsorted(current,a))
                        if pos>0: additions.append((current[pos-1]+a)*.5)
                        if pos+1<len(current): additions.append((a+current[pos+1])*.5)
                    for a in sorted(set(additions)):
                        if len(current)>=settings.max_seam_anchors: break
                        if a not in current: current.append(a); tested_anchors.append(a)
                    layers[0]=layers[-1]=sorted(current)
                    changed=True
            # Localize candidate additions to a deterministic window around the
            # first reachability/co-reachability frontier; wrap on flying laps.
            failed=graph_result.get("failed_layer") if graph_result else None
            if failed is not None:
                center=min(segs-1,max(0,int(failed)))
                selected={(center+offset)%segs for offset in range(-refinement_radius,refinement_radius+1)} if boundary=="flying" else set(range(max(0,center-refinement_radius),min(segs,center+refinement_radius+1)))
                for i in range(segs):
                    if i not in selected:
                        # Retain only initial/previous nodes outside the active window.
                        pass
                if refinement_radius < segs: refinement_radius=min(segs,refinement_radius*2)
            work_queue=sorted({(i,d,float(q).hex()) for i,d,q in ((i,"forward",q) for i,layer in enumerate(layers[:-1]) for q in layer)})
            for x in layers: x.sort()
            if not changed: break
    if incumbent is None:
        status = ("strict_validation_failed" if budget_reason == "strict_validation_failed"
                  else "budget_exceeded" if budget_reason
                  else "no_path_at_resolution")
        raise YawProfileSolveError(status,"coupled yaw profile has no validated sampled path",{
            "graph_status":None if graph_result is None else graph_result.get("status"),"refinement_rounds":refinement_rounds,
            "candidate_counts":tuple(map(len,layers)),"budget_reason":budget_reason,
            "closed_seam_rotation":rotation,"tested_seam_anchors":tuple(tested_anchors),
            "interval_cache_entries":len(interval_cache),"interval_work_queue_entries":len(work_queue),
            "refinement_radius":refinement_radius,
            "stopping_uncertainty":"candidate resolution not converged"})
    q,time_seconds,validation=incumbent
    if rotation:
        restored=np.empty_like(q)
        # rotated order maps each rotated station to its original unique station
        for rotated,original in enumerate(rotate_order[:-1]): restored[original]=q[rotated]
        restored[-1]=restored[0]
        q=restored
    return {"q":q*q,"time_seconds":time_seconds,"refinement_rounds":refinement_rounds,
            "diagnostics":{"status":"feasible_sampled","resolution_converged":stable>=2,"budget_reason":budget_reason,
                "candidate_counts":tuple(map(len,layers)),"graph_status":"ok","validation":validation,
                "stopping_uncertainty":None if stable>=2 else "sampled candidate resolution remains unproven",
                "edge_density":None if packed is None else len(packed.targets)/max(1,packed.n_nodes),"closed_seam_rotation":rotation,"tested_seam_anchors":tuple(tested_anchors),"interval_cache_entries":len(interval_cache),"interval_work_queue_entries":len(work_queue),"refinement_radius":refinement_radius}}
