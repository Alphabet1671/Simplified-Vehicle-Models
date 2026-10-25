"""Strict feasible-corner interpolation and compact map-query evidence."""
from __future__ import annotations
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

    def _query(self, speed, ax, ay, evidence=False):
        q = tuple(float(v) for v in (speed, ax, ay))
        bounds = tuple((float(a[0]), float(a[-1])) for a in self.axes)
        brackets = []
        for axis, value in zip(self.axes, q):
            if not math.isfinite(value) or value < axis[0] - 1e-12 or value > axis[-1] + 1e-12:
                if evidence:
                    return {"query": q, "in_domain": False, "reason": MapQueryReason.OUT_OF_DOMAIN,
                            "axes_bounds": bounds, "support": (), "masked_corners": (),
                            "support_weight_tolerance": SUPPORT_WEIGHT_TOLERANCE}
                return None
            value = min(float(axis[-1]), max(float(axis[0]), value))
            hi = int(np.searchsorted(axis, value, side="right"))
            if hi == 0: brackets.append(((0, 1.0),))
            elif hi >= axis.size: brackets.append(((axis.size - 1, 1.0),))
            else:
                lo = hi - 1; f = (value-axis[lo])/(axis[hi]-axis[lo])
                brackets.append(((lo, 1.-f), (hi, f)))
        into = out = 0.0; support=[]; masked=[]
        for i, wi in brackets[0]:
            for j, wj in brackets[1]:
                for k, wk in brackets[2]:
                    weight = wi*wj*wk
                    if weight <= SUPPORT_WEIGHT_TOLERANCE: continue
                    feasible = bool(self.map.feasible_mask[i,j,k])
                    if evidence: support.append((int(i),int(j),int(k),float(weight),feasible))
                    if not feasible:
                        if not evidence: return None
                        masked.append((int(i),int(j),int(k),float(weight),False)); continue
                    into += weight*self.fields[0][i,j,k]; out += weight*self.fields[1][i,j,k]
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
