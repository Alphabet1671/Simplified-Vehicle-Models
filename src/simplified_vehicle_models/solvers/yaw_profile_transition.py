"""Cell-aware one-dimensional feasible transition intervals."""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np


class Polynomial:
    """Minimal power-series polynomial (ascending coefficients).

    Mirrors the subset of ``numpy.polynomial.Polynomial`` used by the interval
    oracle with plain Python floats; numpy's object overhead dominated runtime.
    Arithmetic is exact-order identical: no trimming happens implicitly.
    """
    __slots__ = ("c",)

    def __init__(self, coef):
        self.c = [float(x) for x in coef] or [0.]

    @property
    def coef(self):
        return np.asarray(self.c, dtype=float)

    def degree(self):
        return len(self.c) - 1

    def deriv(self):
        if len(self.c) <= 1:
            return Polynomial([0.])
        return Polynomial([i * self.c[i] for i in range(1, len(self.c))])

    @staticmethod
    def _wrap(other):
        return other if isinstance(other, Polynomial) else Polynomial([other])

    def __add__(self, other):
        o = self._wrap(other).c; a = self.c
        n = max(len(a), len(o))
        return Polynomial([(a[i] if i < len(a) else 0.) + (o[i] if i < len(o) else 0.) for i in range(n)])
    __radd__ = __add__

    def __neg__(self):
        return Polynomial([-x for x in self.c])

    def __sub__(self, other):
        return self + (-self._wrap(other))

    def __rsub__(self, other):
        return self._wrap(other) - self

    def __mul__(self, other):
        if not isinstance(other, Polynomial):
            f = float(other); return Polynomial([x * f for x in self.c])
        a, b = self.c, other.c
        out = [0.] * (len(a) + len(b) - 1)
        for i, x in enumerate(a):
            if x == 0.: continue
            for j, y in enumerate(b):
                out[i + j] += x * y
        return Polynomial(out)
    __rmul__ = __mul__

    def __truediv__(self, other):
        f = float(other); return Polynomial([x / f for x in self.c])

    def __call__(self, x):
        if isinstance(x, Polynomial):
            out = Polynomial([0.])
            for coefficient in reversed(self.c):
                out = out * x + coefficient
            return out
        x = float(x); acc = 0.
        for coefficient in reversed(self.c):
            acc = acc * x + coefficient
        return acc

from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings, YawProfileSolveError
from simplified_vehicle_models.solvers.yaw_profile_map import _MapInterpolator, interpolator_for


@dataclass(frozen=True, slots=True)
class QInterval:
    lower: float
    upper: float
    lower_included: bool
    upper_included: bool
    support_id: tuple[int, ...]


def _real_roots_unit(poly, *, scale, tolerance, diagnostics):
    """Isolate all real roots on [0,1] by recursively splitting at derivative roots."""
    p = Polynomial(poly.coef)
    while p.degree() > 0 and abs(p.coef[-1]) <= np.finfo(float).eps * max(1., np.max(np.abs(p.coef))):
        p = Polynomial(p.coef[:-1])
    if p.degree() <= 0:
        return []
    if p.degree() == 1:
        root = -p.coef[0] / p.coef[1]
        if -tolerance <= root <= 1+tolerance:
            root = min(1., max(0., float(root)))
            if abs(float(p(root))) > tolerance * scale:
                raise YawProfileSolveError("interval_oracle_unresolved", "linear root residual unresolved", diagnostics)
            return [root]
        return []
    critical = _real_roots_unit(p.deriv(), scale=max(1., np.max(np.abs(p.deriv().coef))),
                                tolerance=tolerance, diagnostics=diagnostics)
    knots = sorted(set([0., *critical, 1.]))
    roots = []
    for x in knots:
        fx = float(p(x))
        if abs(fx) <= tolerance * scale:
            roots.append(x)  # includes even-multiplicity/tangent roots
    for a, b in zip(knots[:-1], knots[1:]):
        fa, fb = float(p(a)), float(p(b))
        if fa == 0. or fb == 0. or fa * fb > 0.:
            continue
        left, right = a, b
        for _ in range(100):
            mid = left + (right-left)*.5
            fm = float(p(mid))
            if abs(fm) <= tolerance*scale or np.nextafter(left,right) >= right:
                left = right = mid
                break
            if fa*fm <= 0.:
                right, fb = mid, fm
            else:
                left, fa = mid, fm
        root = (left+right)*.5
        if abs(float(p(root))) > tolerance*scale:
            raise YawProfileSolveError("interval_oracle_unresolved", "bracketed root residual unresolved", diagnostics)
        roots.append(root)
    return sorted(set(roots))


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
    interp=interpolator_for(capability)
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
        # Query strict support directly; at exact pinned faces, the strict
        # interpolator correctly ignores zero-weight neighboring corners.
        if query(mid)['reason'].value != 'supported': continue
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
            # Normalize each finite cell to [0,1]. Derivative-root partitioning
            # brackets sign changes and explicitly captures repeated/tangent roots.
            z=Polynomial([a,b-a]); pn=p(z)
            rdiag={"segment_index":segment_index,"cell":tuple(indices),"margin_index":margin_index,
                   "polynomial_coefficients":tuple(map(float,pn.coef)),"capability_identity":capability.source_identity}
            for r in _real_roots_unit(pn, scale=max(1.,float(np.max(np.abs(pn.coef)))),
                                      tolerance=max(settings.yaw_moment_tolerance,1e-12), diagnostics=rdiag):
                if 0 < r < 1: roots.append(float(a+(b-a)*r))
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
        # Keep the strict oracle domain aligned with the normalized interval
        # endpoints for the exact-zero roundoff case.
        q_domain_lo, q_domain_hi = lo, hi
        def q_feasible(q):
            q=float(q)
            q_left=fixed_q if direction == 'forward' else q
            q_right=q if direction == 'forward' else fixed_q
            if not math.isfinite(q) or q < 0.0 or min(q_left,q_right) < 0.0:
                return False
            if not (q_domain_lo <= q <= q_domain_hi):
                return False
            ev = edge_evidence(path, capability, segment_index,q_left,q_right,settings=settings)
            return bool(ev['feasible'])
        # A negative high bound can only be repaired when its magnitude is
        # consistent with floating-point cancellation at q=0. Do not use the
        # interval-merging tolerance here: it is not a physical-domain tolerance.
        q_roundoff = 8.0 * np.finfo(float).eps * max(
            abs(fixed_q), abs(ds * amin), abs(ds * amax), np.finfo(float).tiny)
        if high < 0.0:
            if high >= -q_roundoff:
                high = 0.0
            else:
                continue
        low = max(low, 0.0)
        q_domain_lo, q_domain_hi = max(q_domain_lo, 0.0), high
        if low > high:
            continue
        if low == high:
            if q_feasible(low):
                item=QInterval(low,high,True,True,support)
                if result and item.lower <= result[-1].upper and q_feasible(item.lower):
                    prev=result[-1]
                    result[-1]=QInterval(prev.lower,max(prev.upper,item.upper),True,True,
                                         prev.support_id if prev.support_id==item.support_id else ())
                else:
                    result.append(item)
            continue
        # A returned numeric bound itself must be strict-feasible. If root
        # rounding produced an infeasible endpoint, isolate the nearest
        # representable feasible value toward a known feasible interior point.
        qmid = low+(high-low)*.5
        if not q_feasible(qmid):
            # The u-midpoint and q-midpoint are not identical under the
            # nonlinear transform; locate an actual strict witness before
            # retaining this piece, or fail closed as unresolved.
            candidates=np.linspace(low,high,33)[1:-1]
            qmid=next((float(q) for q in candidates if q_feasible(float(q))), math.nan)
            if not math.isfinite(qmid):
                probe_evidence = []
                for probe in candidates:
                    probe=float(probe)
                    q_left=fixed_q if direction == 'forward' else probe
                    q_right=probe if direction == 'forward' else fixed_q
                    if not math.isfinite(probe) or min(q_left,q_right)<0 or not (lo <= probe <= hi):
                        probe_evidence.append({"q_free":probe,"feasible":False,"reason":"outside_nonnegative_free_domain"})
                        continue
                    ev = edge_evidence(path, capability, segment_index,q_left,q_right,settings=settings)
                    probe_evidence.append({"q_free": probe, "feasible": bool(ev['feasible']),
                                           "reason": ev['reason'].value,
                                           "midpoint_speed": ev['midpoint_speed'],
                                           "longitudinal_acceleration": ev['longitudinal_acceleration'],
                                           "lateral_acceleration": ev['lateral_acceleration'],
                                           "into_margin": ev.get('into_margin'),
                                           "out_margin": ev.get('out_margin')})
                raise YawProfileSolveError("interval_oracle_unresolved",
                    "candidate piece has no feasible midpoint or fixed probes; a narrow feasible component may be missed",
                    {"segment_index": segment_index, "cell": support,
                     "capability_identity": capability.source_identity,
                     "direction": direction, "fixed_q": float(fixed_q),
                     "candidate_q_interval": (float(low), float(high)),
                     "candidate_probes": tuple(probe_evidence)})
        if not q_feasible(low):
            bad, good = low, qmid
            for _ in range(128):
                midq = bad+(good-bad)*.5
                if midq == bad or midq == good: break
                if q_feasible(midq): good=midq
                else: bad=midq
            low=good
        if not q_feasible(high):
            good, bad = qmid, high
            for _ in range(128):
                midq = good+(bad-good)*.5
                if midq == good or midq == bad: break
                if q_feasible(midq): good=midq
                else: bad=midq
            high=good
        if low>high: continue
        item=QInterval(low,high,True,True,support)
        # Merge only on actual numeric overlap and only when the connecting
        # endpoint itself passes the strict edge oracle. Tolerance-near gaps
        # may be masked holes and must remain disconnected.
        if result and item.lower <= result[-1].upper:
            prev=result[-1]
            if q_feasible(item.lower):
                result[-1]=QInterval(prev.lower,max(prev.upper,item.upper),True,True,
                                     prev.support_id if prev.support_id==item.support_id else ())
            else:
                result.append(item)
        else: result.append(item)
    return tuple(result)


def edge_evidence(path, capability, segment_index: int, q_left: float, q_right: float, *, settings: CoupledProfileSettings) -> dict:
    if not isinstance(settings, CoupledProfileSettings):
        raise TypeError("settings must be CoupledProfileSettings")
    ds,k,kp,sign=_segment(path,capability,segment_index)
    if min(q_left,q_right)<0 or not all(map(math.isfinite,(q_left,q_right))): raise ValueError("q endpoints must be finite and nonnegative")
    u=math.sqrt((q_left+q_right)/2); ax=(q_right-q_left)/(2*ds); ay=k*u*u
    evidence=interpolator_for(capability).query_evidence(u,ax,ay)
    out={"segment_index":segment_index,"q_left":float(q_left),"q_right":float(q_right),"midpoint_speed":u,"longitudinal_acceleration":ax,"lateral_acceleration":ay,**evidence}
    out['capability_identity'] = capability.source_identity
    out['strict_query_model'] = "_MapInterpolator.query_evidence; zero-weight masked corners excluded"
    out['strict_feasibility_tolerance'] = 1e-9
    if evidence.get('moments') is not None:
        into,outer=evidence['moments']; demand=sign*capability.yaw_inertia*(ax*k+u*u*kp)
        out.update(yaw_demand=demand,into_margin=into-demand,out_margin=outer+demand,feasible=(into-demand>=-1e-9 and outer+demand>=-1e-9))
    else: out.update(feasible=False)
    return out
