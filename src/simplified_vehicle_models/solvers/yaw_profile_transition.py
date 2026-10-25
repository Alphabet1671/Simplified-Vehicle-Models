"""Cell-aware one-dimensional feasible transition intervals."""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np
from numpy.polynomial import Polynomial

from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings, YawProfileSolveError
from simplified_vehicle_models.solvers.yaw_profile_map import _MapInterpolator


@dataclass(frozen=True, slots=True)
class QInterval:
    lower: float
    upper: float
    lower_included: bool
    upper_included: bool
    support_id: tuple[int, ...]


def _segment(path, capability, segment_index):
    if not isinstance(segment_index, int) or isinstance(segment_index, bool) or not 0 <= segment_index < len(path.distance)-1:
        raise ValueError("segment_index out of range")
    ds = float(path.distance[segment_index+1]-path.distance[segment_index])
    k = .5*float(path.curvature[segment_index]+path.curvature[segment_index+1])
    kp = .5*float(path.curvature_rate[segment_index]+path.curvature_rate[segment_index+1])
    sign = math.copysign(1., k if abs(k)>1e-12 else (kp if abs(kp)>1e-12 else 1.))
    return ds,k,kp,sign


def transition_intervals(path, capability, segment_index: int, fixed_q: float, direction: str,
                         free_q_bounds: tuple[float,float], *, settings: CoupledProfileSettings) -> tuple[QInterval,...]:
    if direction not in ("forward", "backward"):
        raise ValueError("direction must be 'forward' or 'backward'")
    if not isinstance(settings, CoupledProfileSettings): raise TypeError("settings must be CoupledProfileSettings")
    ds,k,kp,sign = _segment(path,capability,segment_index)
    if not math.isfinite(fixed_q) or fixed_q < 0 or len(free_q_bounds)!=2: raise ValueError("invalid q domain")
    lo,hi=map(float,free_q_bounds)
    if not (math.isfinite(lo) and math.isfinite(hi)) or lo<0 or hi<lo: raise ValueError("invalid free_q_bounds")
    axes=(np.asarray(capability.speed),np.asarray(capability.longitudinal_acceleration),np.asarray(capability.lateral_acceleration))
    interp=_MapInterpolator(capability)
    # Map free endpoint q to midpoint-speed squared. Independently intersect
    # endpoint speed domain: both endpoint q values must lie in the map domain.
    ulo2=(fixed_q+lo)/2.; uhi2=(fixed_q+hi)/2.
    speed_lo, speed_hi = map(float, axes[0][[0,-1]])
    endpoint_lo, endpoint_hi = speed_lo**2, speed_hi**2
    if fixed_q < endpoint_lo or fixed_q > endpoint_hi:
        return ()
    lo=max(lo, endpoint_lo); hi=min(hi, endpoint_hi)
    if lo > hi: return ()
    ulo=max(speed_lo,math.sqrt(max(0.,(fixed_q+lo)/2.)))
    uhi=min(speed_hi,math.sqrt(max(0.,(fixed_q+hi)/2.)))
    # Also intersect acceleration/lateral domains via partitioned strict queries.
    if uhi<ulo: return ()
    cuts=[ulo,uhi]
    for knot in axes[0]:
        if ulo<knot<uhi: cuts.append(float(knot))
    for a in axes[1]:
        sq=fixed_q+ds*float(a) if direction=='forward' else fixed_q-ds*float(a)
        if sq>=0:
            u=math.sqrt(sq)
            if ulo<u<uhi: cuts.append(u)
    if k:
        for y in axes[2]:
            sq=float(y)/k
            if sq>=0:
                u=math.sqrt(sq)
                if ulo<u<uhi: cuts.append(u)
    cuts=sorted(cuts)
    tol=settings.interval_q_tolerance
    cuts=[cuts[0]]+[x for i,x in enumerate(cuts[1:],1) if x-cuts[i-1]>tol]
    def query(u):
        ax=(u*u-fixed_q)/ds if direction=='forward' else (fixed_q-u*u)/ds
        return interp.query_evidence(u,ax,k*u*u)
    def feasible(u):
        e=query(u)
        if e['reason'].value!='supported': return False
        into,out=e['moments']; demand=sign*capability.yaw_inertia*( ((u*u-fixed_q)/ds if direction=='forward' else (fixed_q-u*u)/ds)*k+u*u*kp)
        return into-demand >= -1e-9 and out+demand >= -1e-9
    # Build exact composed polynomial for each support cell by multilinear corner weights.
    pieces=[]
    for a,b in zip(cuts[:-1],cuts[1:]):
        mid=(a+b)/2; axmid=(mid*mid-fixed_q)/ds if direction=='forward' else (fixed_q-mid*mid)/ds
        indices=[]
        for axis,val in zip(axes,(mid,axmid,k*mid*mid)):
            j=int(np.searchsorted(axis,val,side='right')-1); j=max(0,min(j,len(axis)-2)); indices.append(j)
        i,j,l=indices
        # Build the true strict-oracle support. Knot-pinned axes have a single
        # active corner face; do not require masked zero-weight neighbors.
        active=[]
        for axis,val,ix in zip(axes,(mid,axmid,k*mid*mid),indices):
            if val == axis[ix]: active.append(((ix,Polynomial([1.])),))
            elif val == axis[ix+1]: active.append(((ix+1,Polynomial([1.])),))
            else: active.append(((ix,None),(ix+1,None)))
        if not query(mid)['reason'].value == 'supported': continue
        U=Polynomial([0.,1.]); vals=(U, (U*U-fixed_q)/ds if direction=='forward' else (fixed_q-U*U)/ds, k*U*U)
        weights=[]
        for axis,val,ix in zip(axes,vals,indices):
            weights.append(((axis[ix+1]-val)/(axis[ix+1]-axis[ix]),(val-axis[ix])/(axis[ix+1]-axis[ix])))
        moments=[]
        for field in (capability.yaw_moment_into_turn,capability.yaw_moment_out_of_turn):
            p=Polynomial([0.])
            for di in (0,1):
              for dj in (0,1):
               for dk in (0,1):
                p += weights[0][di]*weights[1][dj]*weights[2][dk]*float(field[i+di,j+dj,l+dk])
            moments.append(p)
        axp=vals[1]; dem=sign*capability.yaw_inertia*(axp*k+U*U*kp)
        margins=(moments[0]-dem,moments[1]+dem)
        roots=[a,b]
        for margin_index,p in enumerate(margins):
            # Roots over finite cell; normalize for conditioning. A numerically
            # unresolved near-real root is a hard failure, never an omitted cut.
            z=Polynomial([a,b-a]); pn=p(z)
            for r in pn.roots():
                if abs(r.imag)<1e-8:
                    if 0 < r.real < 1:
                        rr=float(a+(b-a)*r.real)
                        residual=abs(float(p(rr)))
                        scale=max(1.,float(np.max(np.abs(p.coef))))
                        if residual > settings.yaw_moment_tolerance*scale:
                            raise YawProfileSolveError("interval_oracle_unresolved", "root residual exceeds tolerance", {
                                "segment_index":segment_index,"cell":tuple(indices),"margin_index":margin_index,
                                "root":rr,"residual":residual,"capability_identity":capability.source_identity})
                        roots.append(rr)
                elif abs(r.imag) <= 1e-5 and -1e-8 <= r.real <= 1+1e-8:
                    raise YawProfileSolveError("interval_oracle_unresolved", "ambiguous near-real margin root", {
                        "segment_index":segment_index,"cell":tuple(indices),"margin_index":margin_index,
                        "root":(float(r.real),float(r.imag)),"capability_identity":capability.source_identity})
        roots=sorted(set(roots))
        for witness in [a,b,*roots,*[(x+y)/2 for x,y in zip(roots[:-1],roots[1:])]]:
            ev=query(witness)
            if ev['reason'].value != 'supported':
                continue
            exact=ev['moments']
            approx=(float(margins[0](witness)+sign*capability.yaw_inertia*(((witness*witness-fixed_q)/ds if direction=='forward' else (fixed_q-witness*witness)/ds)*k+witness*witness*kp)),
                    float(margins[1](witness)-sign*capability.yaw_inertia*(((witness*witness-fixed_q)/ds if direction=='forward' else (fixed_q-witness*witness)/ds)*k+witness*witness*kp)))
            err=max(abs(exact[0]-approx[0]),abs(exact[1]-approx[1]))
            if err > settings.yaw_moment_tolerance*max(1.,*map(abs,exact)):
                raise YawProfileSolveError("interval_oracle_unresolved", "polynomial and strict evidence disagree", {
                    "segment_index":segment_index,"cell":tuple(indices),"query":(witness,),"error":err,
                    "capability_identity":capability.source_identity})
        for x,y in zip(roots[:-1],roots[1:]):
            if feasible((x+y)/2): pieces.append((x,y,tuple(indices)))
        for r in roots:
            if feasible(r): pieces.append((r,r,tuple(indices)))
    # Boundaries are queried independently, including exact map faces.
    for u in cuts:
        if feasible(u): pieces.append((u,u,()))
    result=[]
    for a,b,support in sorted(pieces):
        # Mapping u back to q can round an exact map face just outside its
        # domain (e.g. ax=-2 becomes -2.000000000001). Move only that
        # numerical endpoint toward the feasible interior and re-query.
        aa, bb = a, b
        if aa < bb and not feasible(aa):
            aa = float(np.nextafter(aa, bb))
            for _ in range(16):
                if feasible(aa): break
                aa = float(np.nextafter(aa, bb))
        if aa < bb and not feasible(bb):
            bb = float(np.nextafter(bb, aa))
            for _ in range(16):
                if feasible(bb): break
                bb = float(np.nextafter(bb, aa))
        qa=2*aa*aa-fixed_q; qb=2*bb*bb-fixed_q
        if direction=='backward': qa=2*aa*aa-fixed_q; qb=2*bb*bb-fixed_q
        low,high=sorted((qa,qb)); low=max(lo,low); high=min(hi,high)
        amin,amax=float(axes[1][0]),float(axes[1][-1])
        if direction == 'forward':
            low=max(low,fixed_q+2*ds*amin); high=min(high,fixed_q+2*ds*amax)
        else:
            low=max(low,fixed_q-2*ds*amax); high=min(high,fixed_q-2*ds*amin)
        def q_feasible(q):
            ev = edge_evidence(path, capability, segment_index,
                               fixed_q if direction == 'forward' else q,
                               q if direction == 'forward' else fixed_q,
                               settings=settings)
            return bool(ev['feasible'])
        # Guard against cancellation in inverse q->u mapping: the algebraic
        # endpoint may be a few ulps inside although its strict query is out.
        for _ in range(16):
            if low >= high or q_feasible(low): break
            low = float(np.nextafter(low, high))
        for _ in range(16):
            if low >= high or q_feasible(high): break
            high = float(np.nextafter(high, low))
        if low>high+tol: continue
        item=QInterval(low,high,q_feasible(low),q_feasible(high),support)
        if result and item.lower <= result[-1].upper+tol and feasible(math.sqrt((fixed_q+max(item.lower,result[-1].upper))/2)):
            prev=result.pop(); result.append(QInterval(prev.lower,max(prev.upper,item.upper),prev.lower_included,item.upper_included,prev.support_id if prev.support_id==item.support_id else ()))
        else: result.append(item)
    return tuple(result)


def edge_evidence(path, capability, segment_index: int, q_left: float, q_right: float, *, settings: CoupledProfileSettings) -> dict:
    if not isinstance(settings, CoupledProfileSettings):
        raise TypeError("settings must be CoupledProfileSettings")
    ds,k,kp,sign=_segment(path,capability,segment_index)
    if min(q_left,q_right)<0 or not all(map(math.isfinite,(q_left,q_right))): raise ValueError("q endpoints must be finite and nonnegative")
    u=math.sqrt((q_left+q_right)/2); ax=(q_right-q_left)/(2*ds); ay=k*u*u
    evidence=_MapInterpolator(capability).query_evidence(u,ax,ay)
    out={"segment_index":segment_index,"q_left":float(q_left),"q_right":float(q_right),"midpoint_speed":u,"longitudinal_acceleration":ax,"lateral_acceleration":ay,**evidence}
    out['capability_identity'] = capability.source_identity
    if evidence.get('moments') is not None:
        into,outer=evidence['moments']; demand=sign*capability.yaw_inertia*(ax*k+u*u*kp)
        out.update(yaw_demand=demand,into_margin=into-demand,out_margin=outer+demand,feasible=(into-demand>=-1e-9 and outer+demand>=-1e-9))
    else: out.update(feasible=False)
    return out
