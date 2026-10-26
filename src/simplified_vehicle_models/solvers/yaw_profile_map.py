"""Strict feasible-corner interpolation and compact map-query evidence."""
from __future__ import annotations
import bisect
import math
from enum import Enum
import numpy as np
from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap

SUPPORT_WEIGHT_TOLERANCE = 1e-14

class MapQueryReason(str, Enum):
    SUPPORTED = "supported"
    OUT_OF_DOMAIN = "out_of_domain"
    MASKED_SUPPORT = "masked_support"

class _MapInterpolator:
    """Strict trilinear map query; infeasible support is never interpolated through."""
    def __init__(self, capability: YawCapabilityMap):
        if not isinstance(capability, YawCapabilityMap):
            raise TypeError("capability must be a YawCapabilityMap")
        self.map = capability
        self.axes = tuple(np.asarray(v, dtype=float) for v in
                          (capability.speed, capability.longitudinal_acceleration, capability.lateral_acceleration))
        self.fields = (np.asarray(capability.yaw_moment_into_turn), np.asarray(capability.yaw_moment_out_of_turn))
        # Plain-Python mirrors for the scalar hot path; values are identical.
        self._axis_lists = tuple(a.tolist() for a in self.axes)
        self._bounds = tuple((float(a[0]), float(a[-1])) for a in self.axes)
        self._mask = capability.feasible_mask.tolist()
        self._into = self.fields[0].tolist()
        self._out = self.fields[1].tolist()

    def _query(self, speed, ax, ay, evidence=False):
        q = (float(speed), float(ax), float(ay))
        bounds = self._bounds
        brackets = []
        for axis, value in zip(self._axis_lists, q):
            first, last = axis[0], axis[-1]
            if not math.isfinite(value) or value < first - 1e-12 or value > last + 1e-12:
                if evidence:
                    return {"query": q, "in_domain": False, "reason": MapQueryReason.OUT_OF_DOMAIN,
                            "axes_bounds": bounds, "support": (), "masked_corners": (),
                            "support_weight_tolerance": SUPPORT_WEIGHT_TOLERANCE}
                return None
            value = min(last, max(first, value))
            hi = bisect.bisect_right(axis, value)
            if hi == 0: brackets.append(((0, 1.0),))
            elif hi >= len(axis): brackets.append(((len(axis) - 1, 1.0),))
            else:
                lo = hi - 1; f = (value-axis[lo])/(axis[hi]-axis[lo])
                brackets.append(((lo, 1.-f), (hi, f)))
        mask, f_into, f_out = self._mask, self._into, self._out
        into = out = 0.0; support=[]; masked=[]
        for i, wi in brackets[0]:
            for j, wj in brackets[1]:
                for k, wk in brackets[2]:
                    weight = wi*wj*wk
                    if weight <= SUPPORT_WEIGHT_TOLERANCE: continue
                    feasible = bool(mask[i][j][k])
                    if evidence: support.append((i,j,k,weight,feasible))
                    if not feasible:
                        if not evidence: return None
                        masked.append((i,j,k,weight,False)); continue
                    into += weight*f_into[i][j][k]; out += weight*f_out[i][j][k]
        if evidence:
            reason = MapQueryReason.MASKED_SUPPORT if masked else MapQueryReason.SUPPORTED
            return {"query": q, "in_domain": True, "reason": reason, "axes_bounds": bounds,
                    "support": tuple(support), "masked_corners": tuple(masked),
                    "support_weight_tolerance": SUPPORT_WEIGHT_TOLERANCE,
                    "moments": None if masked else (float(into),float(out))}
        return into, out

    def query(self, speed, ax, ay): return self._query(speed, ax, ay)
    def query_evidence(self, speed, ax, ay): return self._query(speed, ax, ay, True)

    def yaw_feasible(self, speed, ax, curvature, curvature_rate, *, acceleration=None):
        ay = speed*speed*curvature if acceleration is None else float(acceleration)
        moments = self.query(speed, ax, ay)
        if moments is None: return False
        into,out=moments
        direction=math.copysign(1., curvature if abs(curvature)>1e-12 else (curvature_rate if abs(curvature_rate)>1e-12 else 1.))
        demand=direction*self.map.yaw_inertia*(ax*curvature+speed*speed*curvature_rate)
        return -out-1e-9 <= demand <= into+1e-9


_INTERPOLATORS: "dict[int, tuple[YawCapabilityMap, _MapInterpolator]]" = {}


def interpolator_for(capability: YawCapabilityMap) -> _MapInterpolator:
    """Return a cached interpolator for this (immutable) capability object."""
    key = id(capability)
    hit = _INTERPOLATORS.get(key)
    if hit is not None and hit[0] is capability:
        return hit[1]
    if len(_INTERPOLATORS) > 64:
        _INTERPOLATORS.clear()
    interp = _MapInterpolator(capability)
    _INTERPOLATORS[key] = (capability, interp)
    return interp
