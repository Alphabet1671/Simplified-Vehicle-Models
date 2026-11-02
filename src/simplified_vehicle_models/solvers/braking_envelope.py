"""Offline sampled quasi-static braking envelope; no runtime oracle calls."""
from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping
import math

import numpy as np


def _array(value: Any, name: str, *, minimum: int = 1) -> np.ndarray:
    raw = np.asarray(value)
    if raw.dtype.kind not in "iuf":
        raise TypeError(f"{name} must contain real numeric values")
    result = np.array(raw, dtype=float, copy=True)
    if result.ndim != 1 or result.size < minimum or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite one-dimensional array of length >= {minimum}")
    result.setflags(write=False)
    return result


def _positive(value: Any, name: str, *, zero: bool = False) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number)):
        raise TypeError(f"{name} must be numeric")
    value = float(value)
    if not math.isfinite(value) or (value < 0 if zero else value <= 0):
        raise ValueError(f"{name} must be finite and {'nonnegative' if zero else 'positive'}")
    return value


def _closed_ring(distance, caps, decel, tolerance):
    """Validate duplicated endpoint and return unique-node edge lengths."""
    if len(distance) < 3 or not math.isclose(float(caps[0]), float(caps[-1]), rel_tol=0, abs_tol=tolerance):
        raise ValueError("closed path requires at least two unique nodes and duplicated endpoint cap")
    return np.diff(distance)


def propagate_braking_caps(distance, corner_speed, deceleration, *, is_closed,
                           tolerance=1e-8, max_iterations=None):
    """Backward relax squared-speed caps; closed arrays duplicate their first endpoint."""
    if not isinstance(is_closed, (bool, np.bool_)):
        raise TypeError("is_closed must be bool")
    x = _array(distance, "distance", minimum=2)
    cap = _array(corner_speed, "corner_speed", minimum=2)
    b = _array(deceleration, "deceleration", minimum=1)
    if len(x) != len(cap) or len(b) != len(x) - 1:
        raise ValueError("expected N distances/corner speeds and N-1 segment decelerations")
    if np.any(np.diff(x) <= 0) or np.any(cap < 0) or np.any(b < 0):
        raise ValueError("distance must increase; speed and deceleration must be nonnegative")
    tol = _positive(tolerance, "tolerance")
    if is_closed:
        _closed_ring(x, cap, b, tol)
    if max_iterations is None:
        max_iterations = len(x) + 1
    if isinstance(max_iterations, bool) or not isinstance(max_iterations, int) or max_iterations < 1:
        raise ValueError("max_iterations must be a positive integer")
    # Check intermediate range before squaring or multiplying.
    with np.errstate(over="ignore", invalid="ignore"):
        q = cap * cap
        edge_gain = 2.0 * b * np.diff(x)
    if not np.all(np.isfinite(q)) or not np.all(np.isfinite(edge_gain)):
        raise ValueError("finite inputs overflow braking-envelope calculation domain")
    n = len(x)
    unique_n = n - 1 if is_closed else n
    qwork = np.array(q[:unique_n], copy=True)
    binding = np.arange(unique_n, dtype=int)
    iterations = 0
    changed = True
    while changed and iterations < max_iterations:
        changed = False
        iterations += 1
        # Each directed forward edge u->v imposes a braking cap on its upstream u.
        for i in (range(unique_n) if is_closed else range(unique_n - 2, -1, -1)):
            j = (i + 1) % unique_n if is_closed else i + 1
            gain = edge_gain[i]
            candidate = qwork[j] + gain
            if not math.isfinite(float(candidate)):
                raise ValueError("braking propagation overflow")
            # Deterministic tie: prefer lower-index binding identity except for its own cap.
            if candidate < qwork[i] - tol:
                qwork[i] = max(0.0, candidate)
                binding[i] = binding[j]
                changed = True
            elif abs(candidate-qwork[i]) <= tol and binding[j] < binding[i] and binding[j] != i:
                binding[i] = binding[j]
    if changed:
        raise RuntimeError("braking-cap propagation did not converge within max_iterations")
    outq = np.r_[qwork, qwork[0]] if is_closed else qwork
    outbind = np.r_[binding, binding[0]] if is_closed else binding
    result = np.sqrt(np.maximum(outq, 0.0))
    if not np.all(np.isfinite(result)):
        raise ValueError("non-finite propagated speed")
    return result, outbind, iterations


@dataclass(frozen=True, slots=True)
class EnvelopeSample:
    station: float
    segment_index: int
    corner_speed: float
    entry_speed: float
    squared_speed: float
    squared_speed_slope: float
    deceleration: float
    binding_index: int
    binding_station: float
    binding_distance: float
    speed_domain_max: float


@dataclass(frozen=True, slots=True)
class BrakingEnvelope:
    distance: np.ndarray
    curvature: np.ndarray
    corner_speed: np.ndarray
    entry_speed: np.ndarray
    deceleration: np.ndarray
    binding_index: np.ndarray
    is_closed: bool
    maximum_speed: float
    provenance: Mapping[str, Any]

    def __post_init__(self):
        arrays = {}
        for name in ("distance", "curvature", "corner_speed", "entry_speed", "deceleration"):
            arrays[name] = _array(getattr(self, name), name, minimum=1)
            object.__setattr__(self, name, arrays[name])
        raw_binding = np.asarray(self.binding_index)
        if raw_binding.ndim != 1 or raw_binding.dtype.kind not in "iu":
            raise TypeError("binding_index must be an integer array")
        bindings = np.array(raw_binding, dtype=int, copy=True)
        bindings.setflags(write=False)
        object.__setattr__(self, "binding_index", bindings)
        if not isinstance(self.is_closed, (bool, np.bool_)):
            raise TypeError("is_closed must be bool")
        top = _positive(self.maximum_speed, "maximum_speed")
        object.__setattr__(self, "maximum_speed", top)
        x, k, c, e, d = (arrays[n] for n in ("distance", "curvature", "corner_speed", "entry_speed", "deceleration"))
        if len(x) < 2 or len(k)!=len(x) or len(c)!=len(x) or len(e)!=len(x) or len(bindings)!=len(x) or len(d)!=len(x)-1:
            raise ValueError("envelope arrays have inconsistent lengths")
        if x[0] != 0.0 or np.any(np.diff(x)<=0):
            raise ValueError("distance must start at zero and increase strictly")
        if np.any(c<0) or np.any(e<0) or np.any(d<0) or np.any(c>top) or np.any(e>c+1e-8):
            raise ValueError("speed/deceleration bounds are inconsistent")
        if np.any(bindings<0) or np.any(bindings>=len(x)):
            raise ValueError("binding indices out of range")
        if self.is_closed:
            if len(x)<3:
                raise ValueError("closed envelope needs at least two unique nodes")
            if not math.isclose(c[0],c[-1],rel_tol=0,abs_tol=1e-8) or not math.isclose(k[0],k[-1],rel_tol=0,abs_tol=1e-8) or not math.isclose(e[0],e[-1],rel_tol=0,abs_tol=1e-8):
                raise ValueError("closed envelope endpoint values must duplicate first node")
        prov = dict(self.provenance)
        for key, value in list(prov.items()):
            if isinstance(value, np.ndarray):
                copied=np.array(value, copy=True); copied.setflags(write=False); prov[key]=copied
            elif isinstance(value, (list, dict)):
                raise TypeError("provenance nested mutable values are not allowed")
        object.__setattr__(self, "provenance", MappingProxyType(prov))

    def sample(self, station: float) -> EnvelopeSample:
        if isinstance(station, bool) or not isinstance(station, (int, float, np.number)) or not math.isfinite(float(station)):
            raise ValueError("station must be finite numeric")
        x, s = self.distance, float(station)
        if self.is_closed:
            s = s % x[-1]
        elif s < x[0] or s >= x[-1]:
            i = 0 if s < x[0] else len(x)-1
            return EnvelopeSample(s,max(0,min(i,len(x)-2)),float(self.corner_speed[i]),float(self.entry_speed[i]),float(self.entry_speed[i]**2),0.,float(self.deceleration[min(i,len(self.deceleration)-1)]),int(self.binding_index[i]),float(x[self.binding_index[i]]),0.,float(self.provenance.get("segment_speed_domain_max",self.corner_speed)[min(i,len(self.deceleration)-1)] if len(self.deceleration) else self.corner_speed[i]))
        i=min(int(np.searchsorted(x,s,side="right")-1),len(x)-2)
        dx=s-x[i]
        slope=(self.entry_speed[i+1]**2-self.entry_speed[i]**2)/(x[i+1]-x[i])
        q=max(0.,self.entry_speed[i]**2+slope*dx)
        corner_q0=float(self.corner_speed[i])**2
        corner_q1=float(self.corner_speed[i+1])**2
        localcap=math.sqrt(max(0.,corner_q0+(corner_q1-corner_q0)*dx/(x[i+1]-x[i])))
        domain=self.provenance.get("segment_speed_domain_max",[localcap]*len(self.deceleration))[i]
        bi=int(self.binding_index[i])
        if bi == i: bd=0.
        else:
            target=float(x[bi] % x[-1]) if self.is_closed else float(x[bi])
            bd=(target-s) % x[-1] if self.is_closed else max(0.,target-s)
        return EnvelopeSample(s,i,localcap,math.sqrt(q),q,float(slope),float(self.deceleration[i]),bi,float(x[bi]),float(bd),float(domain))


def build_braking_envelope(distance, curvature, vehicle, *, maximum_speed=60.0,
                           lateral_margin=.9, deceleration_margin=.6,
                           speed_query_points=9, storage_state=None, tolerance=1e-8,
                           is_closed=True, max_iterations=None):
    """Build an immutable offline sampled quasi-static braking estimate."""
    from simplified_vehicle_models.formulations.point_mass import (
        point_mass_view_of_planar, maximum_corner_speed, maximum_deceleration_limit,
    )
    from simplified_vehicle_models.parameters.vehicle import PointMassVehicleParameters
    x=_array(distance,"distance",minimum=2); k=_array(curvature,"curvature",minimum=2)
    if len(x)!=len(k) or np.any(np.diff(x)<=0) or x[0]!=0.: raise ValueError("distance/curvature grids mismatch or invalid")
    top=_positive(maximum_speed,"maximum_speed"); lm=_positive(lateral_margin,"lateral_margin"); dm=_positive(deceleration_margin,"deceleration_margin")
    if lm>1.0 or dm>1.0: raise ValueError("lateral_margin and deceleration_margin must be in (0, 1]")
    if isinstance(speed_query_points,bool) or not isinstance(speed_query_points,int) or speed_query_points<2: raise ValueError("speed_query_points must be integer >=2")
    pm=vehicle if isinstance(vehicle,PointMassVehicleParameters) else point_mass_view_of_planar(vehicle)
    corner=np.array([min(top,maximum_corner_speed(pm,lm/max(abs(v),1e-4),maximum_speed=top,storage_state=storage_state,tolerance=tolerance)) for v in k])
    if not np.all(np.isfinite(corner)) or np.any(corner<0): raise ValueError("corner-speed oracle returned invalid limit")
    if is_closed and not math.isclose(corner[0],corner[-1],rel_tol=0,abs_tol=tolerance): raise ValueError("closed path endpoint caps must agree")
    b=[]; cache={}; domains=[]; calls=0
    for i in range(len(x)-1):
        kval=max(abs(k[i]),abs(k[i+1])); domain=min(corner[i],corner[i+1]); key=(kval,domain)
        if key not in cache:
            vals=[]
            for v in np.linspace(0.,domain,speed_query_points):
                value=maximum_deceleration_limit(pm,float(v),math.copysign(kval,k[i] if k[i] else 1),storage_state=storage_state,tolerance=tolerance)
                if not math.isfinite(value) or value<0: raise ValueError("deceleration oracle returned invalid authority")
                vals.append(value); calls+=1
            cache[key]=min(vals)*dm
        b.append(cache[key]); domains.append(domain)
    entry,bind,iterations=propagate_braking_caps(x,corner,np.asarray(b),is_closed=is_closed,tolerance=tolerance,max_iterations=max_iterations)
    prov={"method":"sampled-quasistatic-conservative-estimate","speed_query_points":speed_query_points,"segment_speed_domain_max":tuple(domains),"oracle_calls":calls,"unique_oracle_domains":len(cache),"lateral_margin":lm,"deceleration_margin":dm,"storage_state":repr(storage_state),"iterations":iterations,"caveat":"Not a physical feasibility guarantee; sampled quasi-static authority may be optimistic."}
    return BrakingEnvelope(x,k,corner,entry,np.asarray(b),bind,is_closed,top,prov)
