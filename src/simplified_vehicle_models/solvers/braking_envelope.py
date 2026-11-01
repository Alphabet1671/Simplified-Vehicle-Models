"""Offline sampled quasi-static braking envelope; no runtime oracle calls."""
from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping
import math

import numpy as np


def _array(value: Any, name: str, *, minimum: int = 1) -> np.ndarray:
    raw = np.asarray(value)
    if raw.dtype.kind not in "iuf" or raw.dtype.kind == "b":
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


def propagate_braking_caps(distance, corner_speed, deceleration, *, is_closed,
                           tolerance=1e-8, max_iterations=None):
    """Propagate sampled speed caps backward using q=v² and q_i<=q_j+2*b*ds."""
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
    if is_closed and (len(x) < 3 or not math.isclose(x[-1]-x[0], x[-2]-x[-1], rel_tol=tol, abs_tol=tol)):
        raise ValueError("closed distance grid must duplicate its first node at the endpoint")
    if max_iterations is None:
        max_iterations = len(x) + 1
    if isinstance(max_iterations, bool) or not isinstance(max_iterations, int) or max_iterations < 1:
        raise ValueError("max_iterations must be a positive integer")
    q = cap * cap
    binding = np.arange(len(x), dtype=int)
    # For closed paths the final sample duplicates the first; each segment including
    # the seam is represented by b[-1], with the last node constrained to q[0].
    if is_closed:
        if not math.isclose(cap[0], cap[-1], rel_tol=0.0, abs_tol=tol):
            raise ValueError("closed path endpoint corner caps must agree")
        q[-1] = q[0]
        binding[-1] = binding[0]
    iterations = 0
    changed = True
    while changed and iterations < max_iterations:
        changed = False
        iterations += 1
        # Every edge relaxes its upstream node from the downstream constraint.
        edges = range(len(b) - 1, -1, -1)
        for i in edges:
            j = i + 1
            ds = x[j] - x[i]
            candidate = q[j] + 2.0 * b[i] * ds
            if candidate < q[i] - tol:
                q[i] = max(0.0, candidate)
                binding[i] = binding[j]
                changed = True
        if is_closed:
            ds = (x[-1] - x[-2])
            candidate = q[0] + 2.0 * b[-1] * ds
            if candidate < q[-2] - tol:
                q[-2] = max(0.0, candidate)
                binding[-2] = binding[0]
                changed = True
            q[-1], binding[-1] = q[0], binding[0]
    if changed:
        raise RuntimeError("braking-cap propagation did not converge within max_iterations")
    return np.sqrt(np.maximum(q, 0.0)), binding, iterations


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
        for name in ("distance", "curvature", "corner_speed", "entry_speed", "deceleration"):
            arr = _array(getattr(self, name), name, minimum=1)
            object.__setattr__(self, name, arr)
        raw_binding = np.asarray(self.binding_index)
        if raw_binding.ndim != 1 or raw_binding.dtype.kind not in "iu" or len(raw_binding) != len(self.distance):
            raise ValueError("binding_index must be a same-length integer array")
        bindings = np.array(raw_binding, dtype=int, copy=True)
        bindings.setflags(write=False)
        object.__setattr__(self, "binding_index", bindings)
        if not isinstance(self.is_closed, (bool, np.bool_)):
            raise TypeError("is_closed must be bool")
        _positive(self.maximum_speed, "maximum_speed")
        if len(self.curvature) != len(self.distance) or len(self.corner_speed) != len(self.distance) or len(self.entry_speed) != len(self.distance) or len(self.deceleration) != len(self.distance)-1:
            raise ValueError("envelope arrays have inconsistent lengths")
        object.__setattr__(self, "provenance", MappingProxyType(dict(self.provenance)))

    def sample(self, station: float) -> EnvelopeSample:
        if isinstance(station, bool) or not isinstance(station, (int, float, np.number)) or not math.isfinite(float(station)):
            raise ValueError("station must be finite numeric")
        x = self.distance
        s = float(station)
        if self.is_closed:
            s = x[0] + ((s - x[0]) % (x[-1] - x[0]))
        elif s < x[0] or s >= x[-1]:
            i = 0 if s < x[0] else len(x)-1
            return EnvelopeSample(s, max(0, min(i, len(x)-2)), float(self.corner_speed[i]), float(self.entry_speed[i]), float(self.entry_speed[i]**2), 0.0, float(self.deceleration[min(i,len(self.deceleration)-1)]), int(self.binding_index[i]), float(x[self.binding_index[i]]), max(0.0,float(x[self.binding_index[i]]-s)), float(self.corner_speed[i]))
        i = min(int(np.searchsorted(x, s, side="right")-1), len(x)-2)
        dx = s-x[i]
        slope = (self.entry_speed[i+1]**2-self.entry_speed[i]**2)/(x[i+1]-x[i])
        q = max(0.0, self.entry_speed[i]**2+slope*dx)
        bi=int(self.binding_index[i]); bd=max(0.0,float(x[bi]-s))
        if self.is_closed and bi <= i: bd=max(0.0,float((x[-1]-s)+(x[bi]-x[0])))
        return EnvelopeSample(s,i,float(self.corner_speed[i]),math.sqrt(q),q,float(slope),float(self.deceleration[i]),bi,float(x[bi]),bd,float(self.corner_speed[i]))


def build_braking_envelope(distance, curvature, vehicle, *, maximum_speed=60.0,
                           lateral_margin=.9, deceleration_margin=.6,
                           speed_query_points=9, storage_state=None, tolerance=1e-8,
                           is_closed=True, max_iterations=None):
    """Build an immutable offline envelope from sampled point-mass oracle queries."""
    from simplified_vehicle_models.formulations.point_mass import (
        point_mass_view_of_planar, maximum_corner_speed, maximum_deceleration_limit,
    )
    from simplified_vehicle_models.parameters.vehicle import PointMassVehicleParameters
    x = _array(distance, "distance", minimum=2); k = _array(curvature, "curvature", minimum=2)
    if len(x)!=len(k) or np.any(np.diff(x)<=0): raise ValueError("distance/curvature grids mismatch or unordered")
    top=_positive(maximum_speed,"maximum_speed"); lm=_positive(lateral_margin,"lateral_margin"); dm=_positive(deceleration_margin,"deceleration_margin")
    if isinstance(speed_query_points,bool) or not isinstance(speed_query_points,int) or speed_query_points<2: raise ValueError("speed_query_points must be integer >=2")
    pm=vehicle if isinstance(vehicle,PointMassVehicleParameters) else point_mass_view_of_planar(vehicle)
    corner=np.array([min(top,maximum_corner_speed(pm,1.0/max(abs(v),1e-4),maximum_speed=top,storage_state=storage_state,tolerance=tolerance)*math.sqrt(lm)) for v in k])
    if not np.all(np.isfinite(corner)) or np.any(corner < 0): raise ValueError("corner-speed oracle returned invalid limit")
    if is_closed and not math.isclose(corner[0],corner[-1],rel_tol=0,abs_tol=tolerance): raise ValueError("closed path endpoint caps must agree")
    b=[]; cache={}; domains=[]; calls=0
    for i in range(len(x)-1):
        kval=max(abs(k[i]),abs(k[i+1])); domain=min(corner[i],corner[i+1]); key=(kval,domain)
        if key not in cache:
            speeds=np.linspace(0.0,domain,speed_query_points)
            vals=[]
            for v in speeds:
                value=maximum_deceleration_limit(pm,float(v),math.copysign(kval,k[i] if k[i] else 1),storage_state=storage_state,tolerance=tolerance)
                if not math.isfinite(value) or value<0: raise ValueError("deceleration oracle returned invalid authority")
                vals.append(value); calls+=1
            cache[key]=min(vals)*dm
        b.append(cache[key]); domains.append(domain)
    entry, bind, iterations=propagate_braking_caps(x,corner,np.asarray(b),is_closed=is_closed,tolerance=tolerance,max_iterations=max_iterations)
    prov={"method":"sampled-quasistatic-conservative-estimate","speed_query_points":speed_query_points,"segment_speed_domain_max":tuple(domains),"oracle_calls":calls,"unique_oracle_domains":len(cache),"lateral_margin":lm,"deceleration_margin":dm,"storage_state":repr(storage_state),"iterations":iterations,"caveat":"Not a physical feasibility guarantee; sampled quasi-static authority may be optimistic."}
    return BrakingEnvelope(x,k,corner,entry,np.asarray(b),bind,is_closed,top,prov)
