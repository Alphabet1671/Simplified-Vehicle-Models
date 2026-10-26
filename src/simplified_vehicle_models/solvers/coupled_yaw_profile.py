"""Adaptive sampled candidate generation and cyclic yaw-profile solve.

Refinement is organised as bounded *work waves*.  After every graph solve the
solver schedules transition-interval requests ``(segment, direction, fixed_q)``
at the actual reachability/co-reachability frontier, processes one snapshot of
them, inserts strictly evidenced witnesses, and re-solves the graph before any
request created by those insertions is processed.  After a disconnected graph,
bounded directional sweeps run from the real graph cut.  Every interval
component returned for a swept value contributes its midpoint; optional
endpoint/projection witnesses beyond the per-layer cap are counted in
``omitted_optional_witnesses``.  Insertions never exceed
``max_candidates_per_station``; a layer at its cap reports a visible
``candidate_budget`` stop instead of silently dropping a component midpoint.
"""
from __future__ import annotations

import bisect
import math
import time

import numpy as np

from simplified_vehicle_models.solvers.yaw_aware_profile import YawPath
from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings, YawProfileSolveError
from simplified_vehicle_models.solvers.yaw_profile_map import _MapInterpolator
from simplified_vehicle_models.solvers.yaw_profile_transition import transition_intervals, edge_evidence, edge_feasibility_matrix
from simplified_vehicle_models.solvers.layered_q_graph import pack_layered_q_graph, solve_open_graph, solve_closed_graph
from simplified_vehicle_models.solvers.yaw_profile_validation import validate_profile

#: Fixed-q requests processed per (segment, direction) in one wave.  Extra
#: requests are deferred to later waves; they are never dropped.
REQUESTS_PER_FRONTIER = 8
#: Fixed values carried from one layer to the next in a directional sweep.
SWEEP_CARRY = 4
#: Maximum witnesses inserted into one layer by one sweep step.  Every
#: returned interval component always contributes its midpoint first.
SWEEP_INSERT_PER_LAYER = 16
#: Number of witness records retained verbatim in diagnostics.
WITNESS_SAMPLE_LIMIT = 256
#: Values inserted per layer by one full-path incumbent sweep.
CHAIN_INSERT_PER_LAYER = 6
#: Initial half-width, in segments, of the failed-frontier refinement window.
INITIAL_REFINEMENT_RADIUS = 32


def _domain(path, capability, maximum_speed):
    interp = _MapInterpolator(capability)
    vmax = min(float(maximum_speed), float(capability.speed[-1]))
    lo, hi = float(capability.speed[0])**2, vmax*vmax
    bounds=[]
    ay_lo,ay_hi=float(capability.lateral_acceleration[0]),float(capability.lateral_acceleration[-1])
    for k in path.curvature:
        # Lateral acceleration is v^2*kappa; both turn directions bound v^2 above.
        b=hi
        if k > 0 and ay_hi > 0: b=min(b,ay_hi/k)
        elif k < 0 and ay_lo < 0: b=min(b,ay_lo/k)
        elif k != 0: b=lo if lo == 0. else -1.
        bounds.append((lo,b))
    return bounds,interp


def _rotate_closed_path(path, rotation):
    """Rotate unique closed stations and preserve every original edge length."""
    m=len(path.distance)-1
    order=np.r_[np.arange(rotation,m),np.arange(0,rotation+1)]
    edge=np.diff(path.distance)
    lengths=np.r_[edge[rotation:],edge[:rotation]]
    distance=np.r_[0.,np.cumsum(lengths)]
    return YawPath(distance,path.curvature[order],path.curvature_rate[order],True),order


def _snap(q):
    """Return a squared speed that survives the ``v=sqrt(q); v*v`` round trip.

    Profiles leave the solver as speeds and the independent validator
    recomputes ``v*v``. Candidates are stored as a value ``q`` with
    ``s*s == q`` for ``s = sqrt(q)`` exactly (note ``s*s`` and ``s**2`` can
    round differently; the validator uses ``v*v``), so solver edge checks and
    validation see identical numbers. Iterating reaches a fixed point within a
    couple of steps for finite nonnegative floats.
    """
    q = float(q)
    for _ in range(8):
        root = math.sqrt(q)
        nxt = root * root
        if nxt == q:
            return q
        q = nxt
    raise ArithmeticError("squared-speed snap did not reach a fixed point")


def _spread(items, count):
    """Deterministically pick up to ``count`` items spanning a sorted list."""
    if len(items) <= count:
        return list(items)
    picks=sorted(set(int(round(x)) for x in np.linspace(0,len(items)-1,count)))
    return [items[p] for p in picks]


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
    started=time.monotonic()
    timings={"graph_build_seconds":0.,"graph_solve_seconds":0.,"refinement_seconds":0.,"validation_seconds":0.}
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
        if hi < lo:
            layers.append([]); continue
        qs=list(np.linspace(lo,hi,settings.q_seed_count))
        qs.extend(float(v*v) for v in capability.speed if lo <= v*v <= hi and v <= maximum_speed)
        if warm is not None and lo <= warm[i]**2 <= hi: qs.append(float(warm[i]**2))
        k=float(path.curvature[i])
        if abs(k)>0:
            qs.extend(float(y/k) for y in capability.lateral_acceleration if lo <= y/k <= hi)
        if boundary == "standing" and i==0: qs=[0.]
        qs=sorted(set(_snap(q) for q in qs if lo<=q<=hi))
        qs=[q for q in qs if lo<=q<=hi]
        layers.append(qs)
    oversize=[i for i,layer in enumerate(layers) if len(layer)>settings.max_candidates_per_station]
    if oversize or sum(map(len,layers))>settings.max_total_nodes:
        raise YawProfileSolveError("budget_exceeded","initial candidate seeds exceed the candidate budget",
            {"budget_reason":"candidate_budget","oversize_stations":tuple(oversize[:64]),
             "candidate_counts":tuple(map(len,layers))})
    empty=[i for i,layer in enumerate(layers) if not layer]
    if empty:
        raise YawProfileSolveError("no_path_at_resolution","station squared-speed domain admits no candidate",
            {"empty_stations":tuple(empty[:64]),"empty_station_count":len(empty),"boundary":boundary})
    tested_anchors=[]
    if boundary == "flying":
        low=max(domains[0][0],domains[-1][0]); high=min(domains[0][1],domains[-1][1])
        if high<low:
            raise YawProfileSolveError("no_path_at_resolution","closed seam domains do not overlap",{"seam_rotation":rotation})
        initial_anchor_cap=min(8,settings.max_seam_anchors)
        # Uniform anchors span the whole seam domain (including its top, which
        # sorting-and-truncating previously discarded); map speed knots are
        # added only while the anchor cap allows.
        anchor_set=set(_snap(a) for a in np.linspace(low,high,initial_anchor_cap))
        for v in capability.speed:
            if len(anchor_set)>=initial_anchor_cap: break
            if low<=v*v<=high and v<=maximum_speed: anchor_set.add(_snap(v*v))
        anchors=sorted(a for a in anchor_set if low<=a<=high)
        tested_anchors.extend(anchors)
        layers[0]=anchors.copy(); layers[-1]=anchors.copy()

    tol=float(settings.interval_q_tolerance)
    seam_layers={0,n-1} if boundary=="flying" else ({0} if boundary=="standing" else set())
    interval_cache={}
    cache_identity=(tuple(map(float,path.distance)),tuple(map(float,path.curvature)),tuple(map(float,path.curvature_rate)),
                    capability.source_identity,tuple(map(float,capability.speed)),tuple(map(float,capability.longitudinal_acceleration)),
                    tuple(map(float,capability.lateral_acceleration)),capability.feasible_mask.tobytes(),
                    capability.yaw_moment_into_turn.tobytes(),capability.yaw_moment_out_of_turn.tobytes(),settings)
    queued=set(); pending=[]
    cap_hits={"layers":set(),"rejected":0}
    total_nodes=[sum(map(len,layers))]
    def room(layer_index):
        return (len(layers[layer_index])<settings.max_candidates_per_station
                and total_nodes[0]<settings.max_total_nodes)
    refinement_waves=[]; refinement_windows=[]; inserted_witnesses=[]; witness_counts={}
    omitted=[0]
    refinement_radius=min(segs,INITIAL_REFINEMENT_RADIUS)

    def deadline_reached():
        return time.monotonic()-started >= settings.max_wall_seconds

    def intervals_for(i,fixed,direction,bounds):
        key=(cache_identity,int(i),direction,float(fixed).hex())
        if key not in interval_cache:
            interval_cache[key]=transition_intervals(path,capability,i,float(fixed),direction,bounds,settings=settings)
        return interval_cache[key]

    def add_candidate(layer_index,q):
        """Insert a strictly evidenced witness; seam/standing layers are controlled separately."""
        if layer_index in seam_layers: return False
        q=float(q); lo,hi=domains[layer_index]
        if not math.isfinite(q) or q < lo-tol or q > hi+tol: return False
        q=_snap(min(hi,max(lo,q)))
        if q < lo or q > hi: return False
        layer=layers[layer_index]
        pos=bisect.bisect_left(layer,q)
        if (pos<len(layer) and abs(layer[pos]-q)<=tol) or (pos>0 and abs(layer[pos-1]-q)<=tol): return False
        if not room(layer_index):
            cap_hits["layers"].add(layer_index); cap_hits["rejected"]+=1; return False
        layer.insert(pos,q); total_nodes[0]+=1; return True

    def node_for(layer_index,q):
        """Return the layer value representing ``q`` (inserting it if new), or None."""
        q=float(q); lo,hi=domains[layer_index]
        if not math.isfinite(q) or q < lo-tol or q > hi+tol: return None
        q=_snap(min(hi,max(lo,q)))
        if q < lo or q > hi: return None
        layer=layers[layer_index]
        pos=bisect.bisect_left(layer,q)
        for p_ in (pos,pos-1):
            if 0<=p_<len(layer) and abs(layer[p_]-q)<=tol: return layer[p_]
        if layer_index in seam_layers: return None
        if not room(layer_index):
            cap_hits["layers"].add(layer_index); cap_hits["rejected"]+=1; return None
        layer.insert(pos,q); total_nodes[0]+=1; return q

    def sweep(start,direction,fixed_values,in_window,source):
        """Propagate strict witnesses layer by layer from a frontier, bounded per layer."""
        step=1 if direction=="forward" else -1
        sign=float(step)
        i=start; carry=_spread(sorted(set(fixed_values)),SWEEP_CARRY)
        steps=0; inserted_total=0; dead_end=None
        while carry and 0<=i<segs and in_window(i):
            if deadline_reached(): return inserted_total,True,steps,dead_end
            target=i+1 if direction=="forward" else i
            ds=float(path.distance[i+1]-path.distance[i])
            mandatory=[]; optional=[]
            for fixed in carry:
                for iv in intervals_for(i,fixed,direction,domains[target]):
                    cands=[("interval_midpoint",(iv.lower+iv.upper)*.5,True),("interval_lower",iv.lower,False),("interval_upper",iv.upper,False)]
                    for knot in capability.longitudinal_acceleration:
                        projected=fixed+sign*2.0*ds*float(knot)
                        if iv.lower < projected < iv.upper:
                            cands.append(("acceleration_knot_projection",projected,False))
                    for kind,qfree,must in cands:
                        qfree=float(qfree)
                        if not math.isfinite(qfree) or qfree < 0.: continue
                        qleft,qright=(fixed,qfree) if direction=="forward" else (qfree,fixed)
                        feasible=bool(edge_evidence(path,capability,i,qleft,qright,settings=settings)["feasible"])
                        witness_counts[kind]=witness_counts.get(kind,0)+1
                        record=None
                        if len(inserted_witnesses)<WITNESS_SAMPLE_LIMIT:
                            record={"kind":kind,"segment":i,"direction":direction,"fixed_q":fixed,
                                    "free_q":qfree,"strict_feasible":feasible,"source":source,
                                    "target_layer":target,"inserted":False}
                            inserted_witnesses.append(record)
                        if feasible: (mandatory if must else optional).append((qfree,record))
            spare=max(0,SWEEP_INSERT_PER_LAYER-len(mandatory))
            sampled=_spread(sorted(optional,key=lambda x:x[0]),spare)
            omitted[0]+=len(optional)-len(sampled)
            chosen=list(mandatory)+sampled
            nodes=[]
            for q,record in sorted(chosen,key=lambda x:x[0]):
                before=len(layers[target])
                value=node_for(target,q)
                if value is None: continue
                nodes.append(value)
                if len(layers[target])>before:
                    inserted_total+=1
                    if record is not None: record["inserted"]=True
            steps+=1
            if not nodes:
                dead_end=i; break
            carry=_spread(sorted(set(nodes)),SWEEP_CARRY)
            i+=step
        return inserted_total,False,steps,dead_end

    def enqueue(i,direction,fixed):
        fixed=float(fixed)
        if not (0<=i<segs and math.isfinite(fixed) and fixed>=0.): return
        key=(int(i),direction,fixed.hex())
        if key not in queued:
            queued.add(key); pending.append(key)

    def process_wave(source):
        """Process one deterministic snapshot; insertions only take effect at the next graph solve."""
        groups={}
        for key in sorted(pending):
            groups.setdefault((key[0],key[1]),[]).append(key)
        snapshot=[]
        for group in sorted(groups):
            ordered=sorted(groups[group],key=lambda k:float.fromhex(k[2]))
            snapshot.extend(_spread(ordered,REQUESTS_PER_FRONTIER))
        chosen=set(snapshot)
        pending[:]=[key for key in pending if key not in chosen]
        queued.difference_update(chosen)
        processed=0; inserted=0; deadline_stop=False
        for i,direction,fixed_hex in snapshot:
            if deadline_reached():
                deadline_stop=True
                for key in snapshot[processed:]:
                    if key not in queued: queued.add(key); pending.append(key)
                break
            processed+=1
            fixed=float.fromhex(fixed_hex)
            target=i+1 if direction=="forward" else i
            ds=float(path.distance[i+1]-path.distance[i])
            sign=1.0 if direction=="forward" else -1.0
            for iv in intervals_for(i,fixed,direction,domains[target]):
                witnesses=[("interval_lower",iv.lower),("interval_midpoint",(iv.lower+iv.upper)*.5),("interval_upper",iv.upper)]
                for knot in capability.longitudinal_acceleration:
                    projected=fixed+sign*2.0*ds*float(knot)
                    if iv.lower < projected < iv.upper:
                        witnesses.append(("acceleration_knot_projection",projected))
                for kind,qfree in witnesses:
                    qfree=float(qfree)
                    if not math.isfinite(qfree) or qfree < 0.: continue
                    qleft,qright=(fixed,qfree) if direction=="forward" else (qfree,fixed)
                    feasible=bool(edge_evidence(path,capability,i,qleft,qright,settings=settings)["feasible"])
                    added=feasible and add_candidate(target,qfree)
                    inserted+=int(added)
                    witness_counts[kind]=witness_counts.get(kind,0)+1
                    if len(inserted_witnesses)<WITNESS_SAMPLE_LIMIT:
                        inserted_witnesses.append({"kind":kind,"segment":i,"direction":direction,"fixed_q":fixed,
                                                   "free_q":qfree,"strict_feasible":feasible,"inserted":added})
        refinement_waves.append({"source":source,"snapshot_requests":len(snapshot),"processed_requests":processed,
                                 "deferred_next_wave":len(pending),"inserted_candidates":inserted,
                                 "deadline_stop":deadline_stop,"max_candidates_after":max(map(len,layers))})
        return inserted,deadline_stop

    accel_axis=np.asarray(capability.longitudinal_acceleration,float)
    chain_accels=np.unique(np.r_[accel_axis,(accel_axis[:-1]+accel_axis[1:])*.5])

    def chain_sweep(direction,incumbent_q,source,envelope):
        """Full-path strict acceleration/braking chains seeded from the incumbent.

        Each step projects every carried value by ``2*ds*a`` for map
        acceleration knots and their midpoints, keeps only pairs passing the
        vectorised strict oracle (identical to ``edge_evidence``), and inserts
        a bounded spread that always includes the fastest and slowest feasible
        value. A dead end reseeds from the incumbent so the sweep covers the
        whole path. Candidates are additive seeds; the graph re-checks every
        edge.
        """
        order=range(segs) if direction=="forward" else range(segs-1,-1,-1)
        sign=1.0 if direction=="forward" else -1.0
        start=0 if direction=="forward" else n-1
        carry=np.array([incumbent_q[start]])
        inserted=0; dead_ends=0
        for step,i in enumerate(order):
            if step % 256 == 255 and deadline_reached(): return inserted,True,dead_ends
            target=i+1 if direction=="forward" else i
            ds=float(path.distance[i+1]-path.distance[i])
            lo,hi=domains[target]
            fixed=np.repeat(carry,chain_accels.size)
            free=np.clip((carry[:,None]+sign*2.*ds*chain_accels[None,:]).ravel(),lo,hi)
            free=np.array([_snap(x) for x in free])
            keep=(free>=lo)&(free<=hi)&((fixed+free)>0)
            fixed,free=fixed[keep],free[keep]
            if free.size:
                left,right=(fixed,free) if direction=="forward" else (free,fixed)
                ok=edge_feasibility_matrix(path,capability,i,left,right,pairwise=True)
                free=np.unique(free[ok])
            if free.size==0:
                dead_ends+=1
                carry=np.array([incumbent_q[target]]); continue
            # Fastest strict-feasible value reached from the carried set: the
            # discrete forward (or backward) speed envelope at this layer.
            envelope[target]=float(free[-1])
            picks=_spread(list(free),CHAIN_INSERT_PER_LAYER)
            nodes=[]
            for q in picks:
                before=len(layers[target]); v=node_for(target,float(q))
                if v is not None:
                    nodes.append(v); inserted+=len(layers[target])-before
            if not nodes:
                carry=np.array([incumbent_q[target]]); continue
            nodes=sorted(set(nodes))
            carry=np.array(_spread(nodes,SWEEP_CARRY))
        witness_counts[source]=witness_counts.get(source,0)+segs
        return inserted,False,dead_ends

    def refine_frontier(graph_result,packed):
        """Sweep forward from the reach frontier and backward from the co-reach frontier."""
        offsets=packed.node_offsets
        reach=np.zeros(packed.n_nodes,bool); reach[np.asarray(graph_result.get("reachable_nodes",[]),dtype=np.int64)]=True
        coreach=np.zeros(packed.n_nodes,bool); coreach[np.asarray(graph_result.get("co_reachable_nodes",[]),dtype=np.int64)]=True
        reach_layers=[k for k in range(n) if reach[offsets[k]:offsets[k+1]].any()]
        coreach_layers=[k for k in range(n) if coreach[offsets[k]:offsets[k+1]].any()]
        if not reach_layers or not coreach_layers:
            return 0,False,True
        left,right=max(reach_layers),min(coreach_layers)
        center=min(left,segs-1)
        def in_window(i):
            if boundary=="flying":
                gap=abs(i-center)%segs
                return min(gap,segs-gap)<=refinement_radius
            return abs(i-center)<=refinement_radius
        values=lambda layer,mask:[float(packed.q[x]) for x in range(int(offsets[layer]),int(offsets[layer+1])) if mask[x]]
        inserted=0; record={"cut":(int(left),int(right)),"center":int(center),"radius":int(refinement_radius)}
        fwd_start=left if left<segs else None
        bwd_start=right-1 if right>0 else None
        deadline=False
        if fwd_start is not None:
            got,deadline,steps,dead=sweep(fwd_start,"forward",values(left,reach),in_window,"frontier_forward")
            inserted+=got; record.update(forward_steps=steps,forward_dead_end=dead)
        if bwd_start is not None and not deadline:
            got,deadline,steps,dead=sweep(bwd_start,"backward",values(right,coreach),in_window,"frontier_backward")
            inserted+=got; record.update(backward_steps=steps,backward_dead_end=dead)
        starts=[x for x in (fwd_start,bwd_start) if x is not None]
        if not starts:
            refinement_windows.append({**record,"note":"cut at path ends; no interior frontier"})
            return 0,False,True
        lo_seg=min(starts); hi_seg=max(starts)
        record["window_segments"]=((int(center-refinement_radius),int(center+refinement_radius)) if boundary=="flying"
                                   else (int(max(0,center-refinement_radius)),int(min(segs-1,center+refinement_radius))))
        record["inserted_candidates"]=inserted
        window_covers=all(in_window(i) for i in range(lo_seg,hi_seg+1)) if boundary!="flying" else refinement_radius>=segs//2
        refinement_windows.append(record)
        refinement_waves.append({"source":"frontier","snapshot_requests":2,"processed_requests":2,
                                 "deferred_next_wave":len(pending),"inserted_candidates":inserted,
                                 "deadline_stop":deadline,"max_candidates_after":max(map(len,layers))})
        return inserted,deadline,window_covers

    refinement_rounds=0; incumbent=None; budget_reason=None; graph_result=None; packed=None; last_packed=None
    stable=0; previous_candidate=None
    for round_index in range(settings.max_refinement_rounds):
        if deadline_reached():
            budget_reason="wall_deadline"; break
        refinement_rounds=round_index+1
        if cap_hits["rejected"]:
            budget_reason="candidate_budget"; break
        counts=np.array([len(x) for x in layers],dtype=np.int64)
        if np.any(counts>settings.max_candidates_per_station) or int(counts.sum())>settings.max_total_nodes:
            budget_reason="candidate_budget"; break
        t0=time.monotonic()
        node_offsets=np.r_[0,np.cumsum(counts)]
        src_ids=[]; dst_ids=[]; costs=[]; edge_total=0
        for i in range(segs):
            # Every candidate pair is checked by the vectorised strict oracle,
            # which reproduces edge_evidence exactly; no edge is inferred from
            # interval membership alone.
            ds=float(path.distance[i+1]-path.distance[i])
            base_i=int(node_offsets[i]); base_j=int(node_offsets[i+1])
            q0=np.asarray(layers[i],float); q1=np.asarray(layers[i+1],float)
            feasible=edge_feasibility_matrix(path,capability,i,q0,q1)
            feasible&=(q0[:,None]+q1[None,:])>0
            a,b=np.nonzero(feasible)
            if a.size:
                src_ids.append(base_i+a); dst_ids.append(base_j+b)
                costs.append(2*ds/(np.sqrt(q0[a])+np.sqrt(q1[b])))
                edge_total+=a.size
            if edge_total>settings.max_total_edges: break
            if i % 64 == 63 and deadline_reached(): break
        timings["graph_build_seconds"]+=time.monotonic()-t0
        if deadline_reached():
            budget_reason="wall_deadline"; break
        if edge_total>settings.max_total_edges:
            budget_reason="edge_budget"; break
        cat=lambda parts,dtype: np.concatenate(parts).astype(dtype) if parts else np.zeros(0,dtype)
        packed=pack_layered_q_graph(counts,np.concatenate([np.asarray(x,float) for x in layers]),
            cat(src_ids,np.int64),cat(dst_ids,np.int64),cat(costs,float),
            max_nodes=settings.max_total_nodes,max_edges=settings.max_total_edges)
        if isinstance(packed,dict):
            packed=last_packed; budget_reason="graph_budget"; break
        last_packed=packed
        offsets=packed.node_offsets
        t0=time.monotonic()
        if boundary=="standing":
            graph_result=solve_open_graph(packed,int(offsets[0]),np.arange(offsets[-2],offsets[-1]),max_nodes=settings.max_total_nodes,max_edges=settings.max_total_edges)
        else:
            pairs=np.array([(int(offsets[0]+j),int(offsets[-2]+j)) for j in range(min(len(layers[0]),len(layers[-1])))],dtype=np.int64)
            graph_result=solve_closed_graph(packed,pairs,max_nodes=settings.max_total_nodes,max_edges=settings.max_total_edges)
        timings["graph_solve_seconds"]+=time.monotonic()-t0
        if graph_result["status"]=="ok":
            candidate=packed.q[graph_result["node_ids"]]
            candidate_speed=np.sqrt(candidate)
            if len(candidate_speed) != n:
                graph_result={**graph_result,"traceback_node_count":len(candidate_speed)}
                budget_reason="strict_validation_failed"
                break
            t0=time.monotonic()
            validation=validate_profile(path,capability,candidate_speed,maximum_speed=maximum_speed,boundary=boundary,settings=settings)
            timings["validation_seconds"]+=time.monotonic()-t0
            if not validation.valid:
                graph_result={**graph_result,"validation":validation}
                budget_reason="strict_validation_failed"; break
            candidate_time=float(validation.recomputed_time)
            if incumbent is None or candidate_time <= incumbent[1]*(1.+1e-10):
                incumbent=(candidate_speed.copy(),candidate_time,validation)
            # Convergence compares consecutive *graph candidates*, never the
            # incumbent with itself.
            if previous_candidate is not None:
                prev_speed,prev_time=previous_candidate
                if (abs(candidate_time-prev_time) <= settings.refinement_time_relative_tolerance*max(1.,prev_time)
                        and float(np.max(np.abs(candidate_speed-prev_speed))) < settings.refinement_speed_tolerance):
                    stable+=1
                else:
                    stable=0
            previous_candidate=(candidate_speed.copy(),candidate_time)
            if stable>=2 or round_index==settings.max_refinement_rounds-1: break
            t0=time.monotonic()
            env_f=np.full(n,np.nan); env_b=np.full(n,np.nan)
            got_f,stop_f,dead_f=chain_sweep("forward",candidate,"incumbent_forward_chain",env_f)
            got_b,stop_b,dead_b=(0,False,0) if stop_f else chain_sweep("backward",candidate,"incumbent_backward_chain",env_b)
            inserted=got_f+got_b
            # The time-optimal profile in the unconstrained-yaw limit follows
            # min(forward envelope, backward envelope); seed it so the two
            # chains can meet. Every edge is still strictly checked.
            got_e=0
            if not (stop_f or stop_b):
                both=np.isfinite(env_f)&np.isfinite(env_b)
                for i in np.flatnonzero(both):
                    before=len(layers[i]); node_for(int(i),float(min(env_f[i],env_b[i])))
                    got_e+=len(layers[i])-before
            inserted+=got_e
            # Seam anchors: bracket the incumbent seam speed and add the two
            # chain envelopes' seam values (the faster lap's anchor lies there).
            if boundary=="flying" and not (stop_f or stop_b):
                current=sorted(set(layers[0])); seam=float(candidate[0])
                extra=[x for x in (env_f[-1],env_b[0]) if np.isfinite(x)]
                pos=bisect.bisect_left(current,seam)
                if pos>0: extra.append((current[pos-1]+seam)*.5)
                if pos+1<len(current): extra.append((seam+current[pos+1])*.5)
                lo_s,hi_s=max(domains[0][0],domains[-1][0]),min(domains[0][1],domains[-1][1])
                for a in sorted(set(_snap(x) for x in extra)):
                    if len(current)>=settings.max_seam_anchors: break
                    if lo_s<=a<=hi_s and all(abs(a-c)>tol for c in current):
                        current.append(a); tested_anchors.append(a); inserted+=1
                layers[0]=sorted(current); layers[-1]=sorted(current)
            refinement_waves.append({"source":"incumbent_chains","snapshot_requests":2,"processed_requests":2,
                                     "deferred_next_wave":len(pending),"inserted_candidates":inserted,
                                     "forward_dead_ends":dead_f,"backward_dead_ends":dead_b,
                                     "envelope_inserted":got_e,
                                     "deadline_stop":stop_f or stop_b,"max_candidates_after":max(map(len,layers))})
            timings["refinement_seconds"]+=time.monotonic()-t0
            if stop_f or stop_b: budget_reason="wall_deadline"; break
            if inserted==0 and not pending: break
        elif graph_result["status"]=="disconnected":
            if round_index==settings.max_refinement_rounds-1: break
            t0=time.monotonic()
            changed=False
            if boundary=="flying":
                current=sorted(set(layers[0]))
                if current and len(current)<settings.max_seam_anchors:
                    center=current[len(current)//2]
                    neighbors=sorted(current,key=lambda a:(abs(a-center),a))[:2]
                    additions=[]
                    for a in neighbors:
                        pos=int(np.searchsorted(current,a))
                        if pos>0: additions.append((current[pos-1]+a)*.5)
                        if pos+1<len(current): additions.append((a+current[pos+1])*.5)
                    for a in sorted(set(additions)):
                        if len(current)>=settings.max_seam_anchors: break
                        a=_snap(a)
                        if a not in current: current.append(a); tested_anchors.append(a); changed=True
                    layers[0]=sorted(current); layers[-1]=sorted(current)
            inserted,deadline_stop,window_covers=refine_frontier(graph_result,packed)
            timings["refinement_seconds"]+=time.monotonic()-t0
            if deadline_stop: budget_reason="wall_deadline"; break
            changed=changed or inserted>0
            # An unresolved local window doubles for the next round.
            if refinement_radius < segs and (not window_covers or not changed):
                refinement_radius=min(segs,refinement_radius*2)
            elif not changed:
                break
        else:
            budget_reason="graph_budget"; break
    diagnostics_common={
        "refinement_rounds":refinement_rounds,"candidate_counts":tuple(map(len,layers)),"budget_reason":budget_reason,
        "closed_seam_rotation":rotation,"tested_seam_anchors":tuple(tested_anchors),
        "interval_cache_entries":len(interval_cache),"interval_work_queue_entries":len(pending),
        "refinement_radius":refinement_radius,"refinement_waves":tuple(refinement_waves),
        "refinement_windows":tuple(refinement_windows),"inserted_witnesses":tuple(inserted_witnesses),
        "witness_counts":dict(witness_counts),"omitted_optional_witnesses":omitted[0],
        "candidate_cap_rejections":cap_hits["rejected"],"candidate_cap_layers":tuple(sorted(cap_hits["layers"]))[:64],"timings":{**timings,"total_seconds":time.monotonic()-started},
        "edge_count":None if packed is None else len(packed.targets),
        "edge_density":None if packed is None else len(packed.targets)/max(1,packed.n_nodes)}
    if incumbent is None:
        status = ("strict_validation_failed" if budget_reason == "strict_validation_failed"
                  else "budget_exceeded" if budget_reason
                  else "no_path_at_resolution")
        raise YawProfileSolveError(status,"coupled yaw profile has no validated sampled path",{
            **diagnostics_common,"graph_status":None if graph_result is None else graph_result.get("status"),
            "disconnect_cut":None if graph_result is None else graph_result.get("disconnect_cut"),
            "stopping_uncertainty":"candidate resolution not converged"})
    q,time_seconds,validation=incumbent
    if rotation:
        restored=np.empty_like(q)
        for rotated,original in enumerate(rotate_order[:-1]): restored[original]=q[rotated]
        restored[-1]=restored[0]
        q=restored
    converged=stable>=2 and budget_reason is None
    return {"q":q*q,"time_seconds":time_seconds,"refinement_rounds":refinement_rounds,
            "diagnostics":{**diagnostics_common,"status":"feasible_sampled","resolution_converged":converged,
                "graph_status":"ok","validation":validation,
                "stopping_uncertainty":(None if converged else
                     (f"sampling stopped at {budget_reason}" if budget_reason is not None else
                      "sampled candidate resolution remains unproven"))}}
