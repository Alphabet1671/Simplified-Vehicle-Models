"""Immutable sampled forward-drive/backward-brake speed plan.

The trajectory is a quasi-steady sampled estimate, not a transient feasibility
certificate. Capability acceleration values are signed path-progress quantities.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

import numpy as np

from simplified_vehicle_models.solvers.longitudinal_capability import (
    CapabilityQuery, _canonical, _digest, _source_closure_hash, validate_execution_allocator,
)

HYBRID_PLAN_VERSION = "hybrid-speed-plan-v6-cap-bounded-trial-search"


def _copy_array(value, name, *, integer=False):
    raw = np.asarray(value)
    kind = "iu" if integer else "iuf"
    if raw.ndim != 1 or raw.dtype.kind not in kind:
        raise TypeError(f"{name} must be a one-dimensional {'integer' if integer else 'numeric'} array")
    owned = np.array(raw, dtype=int if integer else float, copy=True)
    if not np.all(np.isfinite(owned)):
        raise ValueError(f"{name} must be finite")
    # Immutable bytes backing prevents callers from re-enabling ndarray writes.
    out = np.frombuffer(owned.tobytes(order="C"), dtype=owned.dtype).reshape(owned.shape)
    out.setflags(write=False)
    return out


def _scalar(value, name, *, nonnegative=False):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number)):
        raise TypeError(f"{name} must be numeric")
    out = float(value)
    if not math.isfinite(out) or (nonnegative and out < 0):
        raise ValueError(f"{name} must be finite" + (" and nonnegative" if nonnegative else ""))
    return out


@dataclass(frozen=True, slots=True)
class HybridSpeedPlanSample:
    station_m: float
    segment_index: int
    speed_mps: float
    acceleration_mps2: float
    corner_speed_cap_mps: float
    braking_speed_bound_mps: float
    binding_index: int
    binding_event_station_m: float
    distance_to_binding_event_m: float
    braking_onset_station_m: float | None
    distance_to_braking_onset_m: float | None
    braking_region: bool
    active_constraint: str
    status: str
    curvature_inv_m: float
    future_event_index: int | None
    future_event_station_m: float | None
    distance_to_future_event_m: float | None
    future_event_speed_cap_mps: float | None


@dataclass(frozen=True, slots=True)
class HybridSpeedPlan:
    distance_m: np.ndarray
    speed_mps: np.ndarray
    acceleration_mps2: np.ndarray
    corner_speed_cap_mps: np.ndarray
    braking_speed_bound_mps: np.ndarray
    binding_index: np.ndarray
    curvature_inv_m: np.ndarray
    is_closed: bool
    storage_soc: float
    brake_margin: float
    provider_identity: str
    source_closure_hash: str
    identity: str
    status: str
    iterations: int
    limiting_constraints: tuple[str, ...]
    algorithm_version: str
    max_iterations: int
    tolerance_q_m2_s2: float
    candidate_edge_evaluations: int = 0
    _next_braking_onset: tuple[int | None, ...] = field(init=False, repr=False, compare=False)
    _future_candidates: tuple[tuple[int, ...], ...] = field(init=False, repr=False, compare=False)

    def __post_init__(self):
        for n in ("distance_m", "speed_mps", "acceleration_mps2", "corner_speed_cap_mps",
                  "braking_speed_bound_mps", "curvature_inv_m"):
            object.__setattr__(self, n, _copy_array(getattr(self, n), n))
        object.__setattr__(self, "binding_index", _copy_array(self.binding_index, "binding_index", integer=True))
        n = len(self.distance_m)
        if n < 2 or any(len(getattr(self, k)) != n for k in ("speed_mps", "acceleration_mps2", "corner_speed_cap_mps", "braking_speed_bound_mps", "binding_index", "curvature_inv_m")):
            raise ValueError("plan arrays must have matching length >= 2")
        if self.distance_m[0] != 0 or np.any(np.diff(self.distance_m) <= 0):
            raise ValueError("distance must start at zero and increase strictly")
        if np.any(self.speed_mps < 0) or np.any(self.corner_speed_cap_mps < 0) or np.any(self.speed_mps > self.corner_speed_cap_mps + 1e-8):
            raise ValueError("trajectory exceeds nonnegative corner caps")
        if np.any(self.binding_index < 0) or np.any(self.binding_index >= n):
            raise ValueError("binding index out of range")
        unique = n - 1
        # Ordered immutable event tables leave all station arithmetic, including
        # epsilon-sensitive candidate admission, in sample() unchanged.
        braking = tuple(j for j in range(unique) if self.acceleration_mps2[j] < -1e-10)
        onsets = [None] * n
        candidates = [()] * n
        for i in range(n):
            offsets = tuple(sorted((j for j in braking
                            if ((j-i) % unique if self.is_closed else j-i) > 0),
                            key=lambda j: ((j-i) % unique if self.is_closed else j-i)))
            onsets[i] = offsets[0] if offsets else None
            # Baseline closed scan traverses ``unique`` offsets, so current
            # braking segment appears last after one complete ring.
            if self.is_closed and i < unique and i in braking:
                offsets = offsets + (i,)
            ordered = []
            seen = set()
            for j in offsets:
                candidate = int(self.binding_index[j])
                if candidate != j and candidate not in seen:
                    ordered.append(j)
                    seen.add(candidate)
            candidates[i] = tuple(ordered)
        object.__setattr__(self, "_next_braking_onset", tuple(onsets))
        object.__setattr__(self, "_future_candidates", tuple(candidates))

    def sample(self, station_m: float) -> HybridSpeedPlanSample:
        s = _scalar(station_m, "station")
        length = float(self.distance_m[-1])
        if self.is_closed:
            s %= length
        else:
            s = min(max(s, 0.), length)
        i = min(max(int(np.searchsorted(self.distance_m, s, side="right") - 1), 0), len(self.distance_m)-2)
        dx = s - self.distance_m[i]
        ds = self.distance_m[i+1] - self.distance_m[i]
        q0, q1 = self.speed_mps[i]**2, self.speed_mps[i+1]**2
        slope = (q1-q0)/ds
        q = max(0., q0+slope*dx)
        speed = math.sqrt(q)
        accel = slope/2.
        bi = int(self.binding_index[i])
        binding_station = float(self.distance_m[bi])
        if bi == i:
            event_distance = 0.
        elif self.is_closed:
            event_distance = (binding_station-s) % length
        else:
            event_distance = max(0., binding_station-s)
        braking = accel < -1e-10
        active = "braking" if braking else ("drive" if accel > 1e-10 else "coast_or_cap")
        status = "binding_brake_event" if bi != i else ("braking_region" if braking else "nominal")
        cap = float(self.corner_speed_cap_mps[i] + (self.corner_speed_cap_mps[i+1]-self.corner_speed_cap_mps[i])*dx/ds)
        bound = float(self.braking_speed_bound_mps[i] + (self.braking_speed_bound_mps[i+1]-self.braking_speed_bound_mps[i])*dx/ds)
        onset=None
        if braking:
            # The active braking segment begins at its upstream node; callers
            # use this only as metadata, never as a future lead time.
            onset=float(self.distance_m[i])
        else:
            onset_index = self._next_braking_onset[i]
            if onset_index is not None:
                onset=float(self.distance_m[onset_index])
        if onset is None:
            onset_station=None; onset_distance=None
        else:
            onset_station=onset
            onset_distance=((onset-s) % length if self.is_closed else max(0., onset-s))
        curvature=float(self.curvature_inv_m[i] + (self.curvature_inv_m[i+1]-self.curvature_inv_m[i])*dx/ds)
        future_index=None
        for braking_segment in self._future_candidates[i]:
            candidate = int(self.binding_index[braking_segment])
            candidate_station=float(self.distance_m[candidate])
            candidate_distance=((candidate_station-s) % length if self.is_closed else candidate_station-s)
            if candidate_distance > 1e-10:
                future_index=candidate; break
        if future_index is None:
            future_station=future_distance=future_cap=None
        else:
            future_station=float(self.distance_m[future_index])
            future_distance=((future_station-s) % length if self.is_closed else max(0.,future_station-s))
            future_cap=float(min(self.speed_mps[future_index],self.corner_speed_cap_mps[future_index]))
        return HybridSpeedPlanSample(s, i, speed, accel, cap, bound, bi, binding_station,
                                    event_distance, onset_station, onset_distance, braking, active, status, curvature,
                                    future_index, future_station, future_distance, future_cap)


def build_hybrid_speed_plan(distance_m, curvature_inv_m, corner_speed_cap_mps,
                            capability_provider, *, initial_speed_mps, storage_soc,
                            is_closed, brake_margin=.60, yaw_moment_request_nm=0.,
                            max_iterations=None, tolerance_q_m2_s2=1e-8,
                            execution_allocator_identity=None) -> HybridSpeedPlan:
    """Build a sampled plan; authority is bounded only at segment endpoints/midpoint.

    Drive is the signed net progress acceleration. Each forward edge uses a
    deterministic bounded scan over reachable squared-speed candidates followed
    by local refinement. This is a sampled estimate, not a global maximum;
    separated nonmonotone feasible branches may be missed. Negative drive is
    never clamped to zero, and infeasible continuation fails closed.
    """
    x = _copy_array(distance_m, "distance_m")
    k = _copy_array(curvature_inv_m, "curvature_inv_m")
    cap = _copy_array(corner_speed_cap_mps, "corner_speed_cap_mps")
    if len(x) < (3 if is_closed else 2) or len(k) != len(x) or len(cap) != len(x):
        raise ValueError("distance, curvature and corner caps need matching valid lengths")
    if x[0] != 0. or np.any(np.diff(x) <= 0) or np.any(cap < 0):
        raise ValueError("distance must start at zero and increase; corner caps must be nonnegative")
    if not isinstance(is_closed, (bool, np.bool_)):
        raise TypeError("is_closed must be bool")
    soc = _scalar(storage_soc, "storage_soc")
    if not 0 <= soc <= 1: raise ValueError("storage_soc must be in [0, 1]")
    init = _scalar(initial_speed_mps, "initial_speed_mps", nonnegative=True)
    margin = _scalar(brake_margin, "brake_margin", nonnegative=True)
    if margin > 1: raise ValueError("brake_margin must be in [0, 1]")
    yaw = _scalar(yaw_moment_request_nm, "yaw_moment_request_nm")
    if not hasattr(capability_provider, "evaluate") or not getattr(capability_provider, "identity", None):
        raise TypeError("capability_provider must expose evaluate and stable identity")
    n = len(x)
    if is_closed:
        if not math.isclose(k[0], k[-1], rel_tol=0, abs_tol=1e-8) or not math.isclose(cap[0], cap[-1], rel_tol=0, abs_tol=1e-8):
            raise ValueError("closed endpoint curvature and caps must duplicate seam")
        unique = n-1
    else:
        unique = n
        if init > cap[0] + 1e-10: raise ValueError("infeasible initial speed exceeds initial cap")
    maxit = 4*n+8 if max_iterations is None else max_iterations
    if isinstance(maxit, bool) or not isinstance(maxit, int) or maxit < 1: raise ValueError("max_iterations must be positive integer")
    allocator_identity = getattr(capability_provider, "allocator_identity", capability_provider.identity)
    if yaw != 0:
        if execution_allocator_identity is None:
            raise ValueError("nonzero yaw requires an explicit matching execution allocator identity")
        validate_execution_allocator(allocator_identity, execution_allocator_identity, yaw,
                                     capability_provider if hasattr(capability_provider, "allocator_identity") else None)
    elif execution_allocator_identity is not None and execution_allocator_identity != allocator_identity:
        raise ValueError("capability and execution allocator identities do not match")
    tol = _scalar(tolerance_q_m2_s2, "tolerance_q_m2_s2", nonnegative=True)
    if tol <= 0: raise ValueError("tolerance_q_m2_s2 must be positive")
    qcap = cap[:unique]**2
    q = qcap.copy()
    binding = np.arange(unique, dtype=int)
    if not is_closed: q[0] = init**2
    queries = {}
    limiting = set()
    candidate_edge_evaluations = 0
    def evaluate(v, kval):
        vv = math.sqrt(max(0., float(v)))
        # v is squared speed here. Query endpoint and representative midpoint; use
        # the minimum authority to remain conservative over this sampled segment.
        key = (vv.hex(), float(kval).hex())
        if key not in queries:
            result = capability_provider.evaluate(CapabilityQuery(vv, float(kval), soc, yaw))
            if not result.in_domain:
                raise ValueError("capability query outside domain: " + ",".join(result.limiting_constraints))
            if result.allocator_identity != capability_provider.identity:
                raise ValueError("capability allocator identity mismatch")
            vals = (result.drive_net_progress_acceleration_mps2,
                    result.coast_net_progress_acceleration_mps2,
                    result.brake_net_progress_acceleration_mps2)
            if not all(math.isfinite(float(vv0)) for vv0 in vals): raise ValueError("non-finite capability")
            queries[key] = result
            limiting.update(result.limiting_constraints)
        return queries[key]
    def edge_authority(i, q0, q1):
        j=(i+1)%unique
        kmid=(float(k[i])+float(k[j]))/2.
        qs=(q0, (q0+q1)/2., q1)
        ks=(float(k[i]), kmid, float(k[j]))
        results=[evaluate(v, kval) for v,kval in zip(qs,ks)]
        # The declared sampled contract uses the minimum signed drive across
        # start/mid/end; no absolute-curvature substitution or zero clamp.
        drive=min(float(r.drive_net_progress_acceleration_mps2) for r in results)
        brake=min(max(0.,-float(r.brake_net_progress_acceleration_mps2)) for r in results)
        return drive, brake*margin
    def reachable_forward_q(i, q0, ds, q_limit):
        """Bounded sampled endpoint search over the explicit finite cap range.

        Candidate authority is evaluated at same-provider start/mid/end points.
        Forty-eight uniform candidates span [0,q_limit], with attained q_i and
        the start-only constant-acceleration proposal added as extra candidates
        when in range. Eight deterministic interior samples refine next to the
        best feasible sample. Nonmonotone islands may be missed; this routine
        does not claim a global maximum or continuous reachability certificate.
        """
        nonlocal candidate_edge_evaluations
        start = evaluate(q0, float(k[i])).drive_net_progress_acceleration_mps2
        q_start = min(max(float(q0), 0.), float(q_limit))
        proposal = min(float(q_limit), max(0., float(q0) + 2.*float(start)*ds))
        grid = np.unique(np.r_[np.linspace(0., float(q_limit), 48), q_start, proposal])
        best = None
        best_idx = -1
        values = []
        first_domain_error = None
        for idx, trial in enumerate(grid):
            candidate_edge_evaluations += 1
            try:
                drive, _ = edge_authority(i, q0, float(trial))
            except ValueError as exc:
                if not str(exc).startswith("capability query outside domain:"):
                    raise
                if first_domain_error is None:
                    first_domain_error = exc
                values.append(False)
                continue
            feasible = float(trial) <= float(q0) + 2.*drive*ds + tol
            values.append(feasible)
            if feasible and (best is None or trial > best):
                best, best_idx = float(trial), idx
        if best is None or best <= 0.:
            if first_domain_error is not None and best is None:
                raise first_domain_error
            raise ValueError("infeasible positive-speed continuation under negative net drive")
        # Add eight deterministic interior samples between the best feasible
        # grid point and its next grid point. Do not use sign-based bisection:
        # feasibility is not known to be monotone.
        lo = best
        hi = float(grid[min(best_idx + 1, len(grid)-1)])
        for step in range(1, 9):
            trial = lo + (hi-lo) * step / 9.
            candidate_edge_evaluations += 1
            try:
                drive, _ = edge_authority(i, q0, trial)
            except ValueError as exc:
                if str(exc).startswith("capability query outside domain:"):
                    continue
                raise
            if trial <= q0 + 2.*drive*ds + tol and trial > best:
                best = float(trial)
        # Same-provider verification at the exact accepted endpoint/midpoint
        # is an additional counted evaluation (58 trials + this verification).
        candidate_edge_evaluations += 1
        drive, _ = edge_authority(i, q0, best)
        if best > q0 + 2.*drive*ds + 1e-7:
            raise RuntimeError("reachable forward candidate failed same-provider verification")
        return best

    converged=False
    for iteration in range(1,maxit+1):
        before=q.copy()
        edges=range(unique) if is_closed else range(unique-1)
        for i in edges:
            j=(i+1)%unique; ds=(x[i+1]-x[i]) if i < unique-1 else (x[-1]-x[-2])
            candidate=reachable_forward_q(i, q[i], ds, qcap[j])
            if candidate < q[j]: q[j]=candidate; binding[j]=binding[i]
            q[j]=min(q[j],qcap[j])
        # Backward brake pass: explicit downstream event identity propagates.
        for i in (range(unique-1,-1,-1) if is_closed else range(unique-2,-1,-1)):
            j=(i+1)%unique; ds=x[i+1]-x[i]
            _,b= edge_authority(i,q[i],q[j])
            candidate=q[j]+2.*b*ds
            if candidate < q[i]-1e-12:
                q[i]=max(0.,candidate); binding[i]=binding[j]
            q[i]=min(q[i],qcap[i])
        if not is_closed: q[0]=init**2
        delta=float(np.max(np.abs(q-before)))
        if delta <= tol:
            converged=True; break
    if not converged: raise RuntimeError("hybrid speed plan failed to converge within bounded iterations")
    # Preserve a separate backward-only ceiling for downstream policy logic.
    ceiling=qcap.copy()
    ceiling_binding=np.arange(unique,dtype=int)
    for _ in range(maxit):
        prior=ceiling.copy()
        for i in (range(unique-1,-1,-1) if is_closed else range(unique-2,-1,-1)):
            j=(i+1)%unique
            ds=x[i+1]-x[i]
            _, authority=edge_authority(i,ceiling[i],ceiling[j])
            candidate=ceiling[j]+2.*authority*ds
            if candidate < ceiling[i]-1e-12:
                ceiling[i]=max(0.,candidate); ceiling_binding[i]=ceiling_binding[j]
        if float(np.max(np.abs(ceiling-prior)))<=tol: break
    else: raise RuntimeError("backward braking ceiling failed to converge")
    ceilingout=np.r_[ceiling,ceiling[0]] if is_closed else ceiling
    ceiling_bindout=np.r_[ceiling_binding,ceiling_binding[0]] if is_closed else ceiling_binding
    # Independently verify every ceiling segment with the same sampled provider.
    for i in range(n-1):
        ds=x[i+1]-x[i]
        dq=ceilingout[i+1]-ceilingout[i]
        _,available=edge_authority(i,ceilingout[i],ceilingout[i+1])
        if ceilingout[i] > ceilingout[i+1] + 2*available*ds + 1e-7:
            raise RuntimeError("backward ceiling failed independent braking verification")
        if ceilingout[i] > cap[i]**2+tol or ceilingout[i+1] > cap[i+1]**2+tol:
            raise RuntimeError("backward ceiling exceeds corner cap")
    if not is_closed and init*init > ceiling[0]+tol:
        raise ValueError("infeasible initial speed exceeds backward braking ceiling")
    qout=np.r_[q,q[0]] if is_closed else q
    bindout=np.r_[binding,binding[0]] if is_closed else binding
    speed=np.sqrt(np.maximum(qout,0.))
    # Segment acceleration is exact for the piecewise-linear q trajectory.
    segment_accel=np.diff(qout)/(2*np.diff(x))
    accel=np.empty(n, dtype=float)
    accel[:-1]=segment_accel
    accel[-1]=segment_accel[-1]
    if is_closed and not math.isclose(speed[0],speed[-1],rel_tol=0,abs_tol=1e-10): raise RuntimeError("closed seam trajectory mismatch")
    # Independently verify each accepted segment against same provider and margin.
    for i in range(n-1):
        ds=x[i+1]-x[i]
        drive,b=edge_authority(i,speed[i]**2,speed[i+1]**2)
        dq=speed[i+1]**2-speed[i]**2
        if dq > 2*drive*ds+1e-7 or -dq > 2*b*ds+1e-7:
            raise RuntimeError("accepted segment failed provider drive/brake verification")
    # Event binding indices identify the actual downstream limiting node.
    closure = getattr(capability_provider, "source_closure_hash", None) or _source_closure_hash()
    payload={"distance":x,"curvature":k,"caps":cap,"speed":speed,"acceleration":accel,
             "braking_ceiling":np.sqrt(np.maximum(ceilingout,0.)),"binding":ceiling_bindout,
             "provider_identity":capability_provider.identity,"source_closure_hash":closure,
             "storage_soc":soc,"brake_margin":margin,"yaw_request":yaw,"closed":is_closed,
             "iterations":iteration,"max_iterations":maxit,"tolerance_q_m2_s2":tol,
             "algorithm_version":HYBRID_PLAN_VERSION,
             "forward_search_contract":"48-uniform+2-extra+8-refinement+1-final-verify;domain-trial-skip;not-global-maximum",
              "forward_search_trial_budget_per_edge_iteration":58,
              "forward_search_total_budget_per_edge_iteration":59,
              "candidate_edge_evaluations":candidate_edge_evaluations,
              "query_metadata":sorted((key, r.in_domain, r.limiting_constraints, r.allocator_identity) for key,r in queries.items())}
    identity=_digest(payload)
    return HybridSpeedPlan(x,speed,accel,cap,np.sqrt(np.maximum(ceilingout,0.)),ceiling_bindout,k,is_closed,
                           soc,margin,str(capability_provider.identity),str(closure),identity,
                           "converged_sampled_quasisteady",iteration,tuple(sorted(limiting)),
                           HYBRID_PLAN_VERSION,maxit,tol,candidate_edge_evaluations)
