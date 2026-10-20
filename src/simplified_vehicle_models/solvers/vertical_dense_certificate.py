"""Conservative state-domain certificate for vertical 14-DoF dense output.

This module certifies only inequalities that are meaningful for the reported BDF
interpolant.  It is deliberately not a tire, damper, table, load, powertrain,
or true-ODE certificate.
"""
from __future__ import annotations

import numpy as np

from .planar_dense_certificate import (
    MIN_DENOMINATOR,
    MIN_PROGRESS,
    MIN_SPEED,
    STEERING_TOLERANCE,
    plan_domain_limits,
    _cos_range,
    _interval_product,
    _sin_range,
    curvature_enclosure,
)

# VerticalForwardPlant layout: path (0:3), VerticalState (3:24), SOC, steering.
_S, _N, _PSI = 0, 1, 2
_VX = 3
_OMEGA = slice(12, 16)       # mechanical wheel speeds 9:13
_JOUNCE = slice(16, 20)       # mechanical jounce 13:17
_SOC, _STEER = 24, 25


def _failure(reason, **extra):
    result = {"certified": False, "failed": [], "scope": _SCOPE, "reason": reason}
    result.update(extra)
    return result


_SCOPE = (
    "BDF dense interpolant state-domain certificate only: interpolant-wide SOC, "
    "forward-speed, wheel-speed, steering, physical jounce-travel, curvature "
    "path-denominator, and path-progress inequalities; not a full between-step "
    "certificate, true ODE trajectory, table/damper/tire/load/powertrain validation"
)


def certify_vertical_dense_domain(vehicle, plan, bounds, summary=None, pedal_dynamics=False):
    """Certify explicit vertical state-domain inequalities over dense intervals.

    ``bounds`` is ``(intervals, 38, 2)`` with per-component ``[min, max]``
    enclosures.  The supplied jounce limits are the physical suspension limits,
    not the vertical solver's +/-10 mm Newton-trial allowance.
    """
    bounds = np.asarray(bounds, dtype=float)
    expected = 39 if pedal_dynamics else 38
    if bounds.ndim != 3 or bounds.shape[2] != 2 or bounds.shape[1] != expected:
        return _failure("enclosure shape does not match the 14DoF plant")
    if bounds.shape[0] == 0 or not np.all(np.isfinite(bounds)):
        return _failure("enclosure is empty or non-finite")
    lo, hi = bounds[:, :, 0], bounds[:, :, 1]
    if np.any(lo > hi):
        return _failure("enclosure lower bound exceeds upper bound")

    try:
        suspension = vehicle.suspension
        if hasattr(suspension, "kinematics"):
            # The compiled vehicle stores one (corner, [lower, upper]) array.
            jounce_limits = np.array(suspension.kinematics.jounce_limits, dtype=float)
            if jounce_limits.shape != (4, 2):
                raise ValueError("jounce limits must be four corners by lower and upper")
        else:
            lower, upper = tuple(float(x) for x in suspension.front.jounce_limits)
            rear_lower, rear_upper = tuple(float(x) for x in suspension.rear.jounce_limits)
            # Each corner has an axle-specific limit. Keep per-corner margins.
            jounce_limits = np.array([[lower, upper], [lower, upper],
                                      [rear_lower, rear_upper], [rear_lower, rear_upper]])
    except (AttributeError, TypeError, ValueError):
        return _failure("vehicle has no finite suspension jounce limits")
    if not np.all(np.isfinite(jounce_limits)):
        return _failure("vehicle jounce limits are non-finite")

    # An open plan starts from rest; see ``plan_domain_limits``.
    enclosure, minimum_speed, minimum_progress, strict_limits, roundoff = plan_domain_limits(plan)
    margins = {
        "soc_floor": float(np.min(lo[:, _SOC])),
        "soc_ceiling": float(np.min(1.0 - hi[:, _SOC])),
        "speed_over_minimum": float(np.min(lo[:, _VX] - minimum_speed)),
        "wheel_speed_floor": float(np.min(lo[:, _OMEGA])),
        "steering_headroom": float(np.min(
            float(vehicle.planar.steering_maximum_angle) + STEERING_TOLERANCE
            - np.maximum(np.abs(lo[:, _STEER]), np.abs(hi[:, _STEER])))),
        "jounce_lower_margin": float(np.min(lo[:, _JOUNCE] - jounce_limits[:, 0])),
        "jounce_upper_margin": float(np.min(jounce_limits[:, 1] - hi[:, _JOUNCE])),
    }
    if pedal_dynamics:
        margins["pedal_overshoot_beyond_travel"] = float(np.max(
            np.maximum(np.abs(lo[:, 38]), np.abs(hi[:, 38])) - 1.0))
    if plan is None or not hasattr(plan, "distance") or not hasattr(plan, "curvature"):
        return _failure("track plan is required for path-domain checks", margins=margins)
    try:
        kappa_lo, kappa_hi = enclosure(plan.distance, plan.curvature,
                                       lo[:, _S], hi[:, _S])
        product_lo, product_hi = _interval_product(kappa_lo, kappa_hi,
                                                    lo[:, _N], hi[:, _N])
        den_lo, den_hi = 1.0 - product_hi, 1.0 - product_lo
        margins["path_denominator_over_minimum"] = float(np.min(den_lo - MIN_DENOMINATOR))
        cos_lo, cos_hi = _cos_range(lo[:, _PSI], hi[:, _PSI])
        sin_lo, sin_hi = _sin_range(lo[:, _PSI], hi[:, _PSI])
        a_lo, a_hi = _interval_product(lo[:, _VX], hi[:, _VX], cos_lo, cos_hi)
        # Vertical plant's lateral velocity is VerticalState.v_y, global index 4.
        b_lo, b_hi = _interval_product(lo[:, 4], hi[:, 4], sin_lo, sin_hi)
        numerator_lo = a_lo - b_hi
        # Avoid evaluating the unused branch when a denominator enclosure reaches
        # zero; such intervals fail closed via the non-finite margin below.
        positive = numerator_lo >= 0.0
        worst = np.full_like(numerator_lo, np.nan, dtype=float)
        np.divide(numerator_lo, den_hi, out=worst, where=positive & (den_hi != 0.0))
        np.divide(numerator_lo, den_lo, out=worst, where=(~positive) & (den_lo != 0.0))
        margins["path_progress_over_minimum"] = float(np.min(worst - minimum_progress))
    except (AttributeError, IndexError, TypeError, ValueError, FloatingPointError):
        return _failure("track plan path enclosure could not be evaluated", margins=margins)

    strict = ({"speed_over_minimum", "path_progress_over_minimum"} if strict_limits else set())
    diagnostic = {"pedal_overshoot_beyond_travel"}
    slack = {"speed_over_minimum", "wheel_speed_floor", "path_progress_over_minimum"}
    failed = sorted(name for name, value in margins.items()
                    if name not in diagnostic and
                    (not np.isfinite(value)
                     or ((value + (roundoff if name in slack else 0.0)) <= 0.0 if name in strict
                         else (value + (roundoff if name in slack else 0.0)) < 0.0)))
    return {
        "certified": not failed,
        "failed": failed,
        "intervals": int(bounds.shape[0]),
        "margins": margins,
        "jounce_travel_limits_m": jounce_limits.tolist(),
        "scope": _SCOPE,
        "model": "14dof-simple",
        "full_between_step_certificate": False,
    }


# Naming used by callers that treat the dense-domain result as a trajectory cert.
certify_vertical_dense_trajectory = certify_vertical_dense_domain

__all__ = ["certify_vertical_dense_domain", "certify_vertical_dense_trajectory"]
