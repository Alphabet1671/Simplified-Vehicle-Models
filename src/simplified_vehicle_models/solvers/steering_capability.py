"""Kinematic screening speed caps for finite-rate steering actuators.

This is a conservative feedforward actuator screen, not a certificate of
closed-loop or full vehicle tracking performance.
"""
from __future__ import annotations

import math

import numpy as np


def steering_transition_speed_cap(distance, curvature, wheelbase, maximum_steering,
                                  maximum_steering_rate, steering_time_constant, *,
                                  original_cap=None, is_closed=False):
    """Lower sampled speed caps for steering-angle transition feasibility.

    Curvature is converted to bicycle-model steering angle ``atan(L*kappa)``.
    Each segment's secant steering-angle derivative is imposed at both of its
    endpoints. The first-order feedforward condition is
    ``abs(delta)+tau*v*abs(d(delta)/ds) <= delta_max``; the actuator slew
    condition is ``v*abs(d(delta)/ds) <= rate_max``. On a closed path the
    arrays include a duplicated endpoint and the seam segment is included.

    Raises ValueError if the nominal no-slip bicycle geometry requires more
    than the steering limit. Lowering speed cannot remove that *kinematic*
    violation, but sideslip and transient motion mean it is not proof that the
    full vehicle cannot traverse the path within a lateral-error tolerance.
    """
    def scalar(value, name, *, zero=False):
        if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number)):
            raise TypeError(f"{name} must be numeric")
        result = float(value)
        if not math.isfinite(result) or (result < 0 if zero else result <= 0):
            raise ValueError(f"{name} must be finite and {'nonnegative' if zero else 'positive'}")
        return result

    def array(value, name):
        raw = np.asarray(value)
        if raw.dtype.kind not in "iuf":
            raise TypeError(f"{name} must be real numeric")
        result = np.asarray(raw, dtype=float)
        if result.ndim != 1 or result.size < (3 if is_closed else 2) or not np.all(np.isfinite(result)):
            raise ValueError(f"{name} must be a finite one-dimensional array")
        return result

    if not isinstance(is_closed, (bool, np.bool_)):
        raise TypeError("is_closed must be bool")
    x, k = array(distance, "distance"), array(curvature, "curvature")
    if len(x) != len(k) or np.any(np.diff(x) <= 0):
        raise ValueError("distance/curvature lengths must match and distance increase")
    L = scalar(wheelbase, "wheelbase")
    delta_max = scalar(maximum_steering, "maximum_steering")
    rate_max = scalar(maximum_steering_rate, "maximum_steering_rate")
    tau = scalar(steering_time_constant, "steering_time_constant", zero=True)
    if is_closed and not (math.isclose(k[0], k[-1], rel_tol=0, abs_tol=1e-10)):
        raise ValueError("closed path curvature must duplicate the endpoint")
    delta = np.arctan(L * k)
    if not np.all(np.isfinite(delta)):
        raise ValueError("steering-angle conversion overflow")
    if np.any(np.abs(delta) > delta_max + 1e-12):
        raise ValueError("nominal bicycle infeasible geometry: required steering exceeds maximum_steering; full-vehicle feasibility unresolved")
    caps = np.full(len(x), math.inf) if original_cap is None else array(original_cap, "original_cap").copy()
    if len(caps) != len(x) or np.any(caps < 0):
        raise ValueError("original_cap must match distance and be nonnegative")
    slopes = np.diff(delta) / np.diff(x)
    if is_closed:
        # Input includes the duplicated station; its last segment reaches seam.
        if original_cap is not None and not math.isclose(caps[0], caps[-1], rel_tol=0, abs_tol=1e-10):
            raise ValueError("closed path original_cap must duplicate the endpoint")
        node_slope = np.r_[max(abs(slopes[-1]), abs(slopes[0])),
                           np.maximum(np.abs(slopes[:-1]), np.abs(slopes[1:])),
                           max(abs(slopes[-1]), abs(slopes[0]))]
    else:
        node_slope = np.r_[np.abs(slopes[0]),
                           np.maximum(np.abs(slopes[:-1]), np.abs(slopes[1:])),
                           np.abs(slopes[-1])]
    moving = node_slope > 0
    rate_bound = np.full(len(x), math.inf)
    rate_bound[moving] = rate_max / node_slope[moving]
    caps = np.minimum(caps, rate_bound)
    if tau > 0:
        remaining = delta_max - np.abs(delta)
        lag_bound = np.full(len(x), math.inf)
        lag_bound[moving] = remaining[moving] / (tau * node_slope[moving])
        caps = np.minimum(caps, lag_bound)
    # Infinity is meaningful only when no original cap was supplied and a
    # constant-curvature station imposes no actuator bound.
    if np.any(np.isnan(caps)) or np.any(caps < 0):
        raise ValueError("invalid steering screening cap")
    caps.setflags(write=False)
    return caps
