"""Independent strict validator for sampled yaw-aware profiles."""
from __future__ import annotations

import math
from typing import Literal

import numpy as np

from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.solvers.yaw_aware_profile import YawAwareProfile, YawPath
from simplified_vehicle_models.solvers.yaw_profile_contracts import (
    CoupledProfileSettings, ProfileValidation,
)
from simplified_vehicle_models.solvers.yaw_profile_map import _MapInterpolator


def validate_profile(path: YawPath, capability: YawCapabilityMap, q,
                     *, maximum_speed: float, boundary: Literal["flying", "standing"],
                     settings: CoupledProfileSettings, dense: bool = False) -> ProfileValidation:
    """Recompute each segment's kinematics and yaw demand from q itself.

    Stored acceleration, elapsed-time, graph cost and edge certificates are never
    inputs to the feasibility decision. ``q`` is the node-speed vector.
    """
    if not isinstance(path, YawPath) or not isinstance(capability, YawCapabilityMap):
        raise TypeError("path and capability must be YawPath and YawCapabilityMap")
    if not isinstance(settings, CoupledProfileSettings):
        raise TypeError("settings must be CoupledProfileSettings")
    if not isinstance(dense, bool):
        raise ValueError("dense must be boolean")
    if boundary not in ("flying", "standing"):
        raise ValueError("boundary must be 'flying' or 'standing'")
    if not isinstance(maximum_speed, (int, float)) or isinstance(maximum_speed, bool) or not math.isfinite(maximum_speed) or maximum_speed <= 0:
        raise ValueError("maximum_speed must be finite and positive")
    speed = np.asarray(q, dtype=float)
    if speed.ndim != 1 or speed.shape != path.distance.shape:
        raise ValueError("q must be a speed vector matching path nodes")
    if not np.all(np.isfinite(speed)):
        return ProfileValidation(False, math.inf, math.inf, None, tuple(range(len(path.distance)-1)),
                                 0, math.inf, None, False if dense else None)
    nseg = len(speed)-1
    bad = []
    max_ax_violation = max_yaw_violation = 0.0
    min_margin = math.inf
    invalid_support = 0
    dense_valid = True if dense else None
    total_time = 0.0
    interp = _MapInterpolator(capability)
    for i, ds in enumerate(path.segment_lengths):
        v0, v1 = float(speed[i]), float(speed[i+1])
        if v0 < 0 or v1 < 0 or v0 > maximum_speed or v1 > maximum_speed:
            bad.append(i)
            max_ax_violation = max(max_ax_violation, max(0., -v0, -v1, v0-maximum_speed, v1-maximum_speed))
            continue
        if v0 + v1 <= 0.0:
            bad.append(i)
            continue
        # Same arithmetic as the edge oracle (q = v*v, then midpoint and
        # difference in q), so a value on a map knot face is classified
        # identically by solver and validator.
        q0, q1 = v0*v0, v1*v1
        vm = math.sqrt((q0 + q1)/2)
        ax = (q1 - q0)/(2*float(ds))
        ax_violation = max(0., float(capability.longitudinal_acceleration[0])-ax,
                           ax-float(capability.longitudinal_acceleration[-1]))
        max_ax_violation = max(max_ax_violation, ax_violation)
        if ax_violation > settings.acceleration_tolerance:
            bad.append(i)
        k = .5*(float(path.curvature[i])+float(path.curvature[i+1]))
        kp = .5*(float(path.curvature_rate[i])+float(path.curvature_rate[i+1]))
        ay = vm*vm*k
        evidence = interp.query_evidence(vm, ax, ay)
        if evidence["reason"].value != "supported":
            invalid_support += 1
            bad.append(i)
            continue
        into, out = evidence["moments"]
        sign = math.copysign(1., k if abs(k)>1e-12 else (kp if abs(kp)>1e-12 else 1.))
        demand = sign*capability.yaw_inertia*(ax*k+vm*vm*kp)
        yaw_violation = max(0., demand-into, -out-demand)
        max_yaw_violation = max(max_yaw_violation, yaw_violation)
        min_margin = min(min_margin, into-demand, out+demand)
        if ax_violation > settings.acceleration_tolerance or yaw_violation > settings.yaw_moment_tolerance:
            bad.append(i)
        if dense:
            for fraction in (0.0, 0.25, 0.75, 1.0):
                v = (1.-fraction)*v0 + fraction*v1
                kf = (1.-fraction)*float(path.curvature[i]) + fraction*float(path.curvature[i+1])
                kpf = (1.-fraction)*float(path.curvature_rate[i]) + fraction*float(path.curvature_rate[i+1])
                ayf = v*v*kf
                audit = interp.query_evidence(v, ax, ayf)
                if audit["reason"].value != "supported":
                    dense_valid = False
                    continue
                fi, fo = audit["moments"]
                fs = math.copysign(1., kf if abs(kf)>1e-12 else (kpf if abs(kpf)>1e-12 else 1.))
                fdemand = fs*capability.yaw_inertia*(ax*kf+v*v*kpf)
                fax_violation = max(0., float(capability.longitudinal_acceleration[0])-ax,
                                    ax-float(capability.longitudinal_acceleration[-1]))
                fyaw_violation = max(0., fdemand-fi, -fo-fdemand)
                if fax_violation > settings.acceleration_tolerance or fyaw_violation > settings.yaw_moment_tolerance:
                    dense_valid = False
        if v0+v1 > 0:
            total_time += 2.*float(ds)/(v0+v1)
    seam_error = abs(float(speed[0]-speed[-1])) if path.is_closed else 0.0
    if boundary == "standing" and (not path.is_closed and speed[0] != 0.0):
        bad.append(0)
    if boundary == "flying" and (not path.is_closed or seam_error > 1e-10):
        bad.append(max(0,nseg-1))
    if dense and not dense_valid:
        bad.extend(range(nseg))
    valid = not bad and dense_valid is not False
    return ProfileValidation(valid, max_ax_violation, max_yaw_violation,
        None if min_margin == math.inf else float(min_margin), tuple(sorted(set(bad))),
        invalid_support, seam_error, float(total_time), dense_valid)


def validate_profile_object(path: YawPath, capability: YawCapabilityMap, profile: YawAwareProfile,
                            *, maximum_speed: float, boundary: Literal["flying", "standing"],
                            settings: CoupledProfileSettings, dense: bool = False) -> ProfileValidation:
    """Validate a profile and its redundant stored time/acceleration fields."""
    result = validate_profile(path, capability, profile.speed, maximum_speed=maximum_speed,
                              boundary=boundary, settings=settings, dense=dense)
    expected = float(np.sum(2.*path.segment_lengths/(profile.speed[:-1]+profile.speed[1:]))) if np.all(profile.speed[:-1]+profile.speed[1:] > 0) else math.inf
    if (not math.isfinite(expected) or not math.isclose(profile.lap_time, expected, rel_tol=0., abs_tol=1e-9*max(1., expected))
            or not math.isclose(float(profile.elapsed_time[-1]), expected, rel_tol=0., abs_tol=1e-9*max(1., expected))):
        return ProfileValidation(False, result.maximum_acceleration_domain_violation,
            result.maximum_yaw_moment_violation, result.minimum_yaw_margin,
            result.invalid_segment_indices, result.invalid_support_count,
            result.seam_speed_error, result.recomputed_time, result.dense_sample_valid)
    return result
