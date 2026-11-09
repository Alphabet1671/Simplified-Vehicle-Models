"""Bounded scalar inverse of the forward hydraulic brake command chain.

This solves total hydraulic wheel-torque magnitude, not acceleration or a
four-wheel allocator. Regenerative braking is deliberately outside this API.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from simplified_vehicle_models.controls.brakes import brake_torques
from simplified_vehicle_models.solvers.forward_powertrain import shape_brake_command


@dataclass(frozen=True, slots=True)
class BrakeInverseResult:
    pedal_force_n: float
    wheel_torques_nm: np.ndarray
    achieved_effort_nm: float
    residual_nm: float
    feasible: bool
    constraint: str

    def __post_init__(self):
        values = np.array(self.wheel_torques_nm, dtype=float, copy=True)
        if values.shape != (4,) or not np.all(np.isfinite(values)):
            raise ValueError("wheel_torques_nm must be a finite four-vector")
        values.setflags(write=False)
        object.__setattr__(self, "wheel_torques_nm", values)


def _map(vehicle, force: float):
    controller = vehicle.controller
    _, effective = shape_brake_command(vehicle, force)
    return brake_torques(
        effective, vehicle.maximum_pedal_force, vehicle.pedal_curve,
        vehicle.hydraulic_ratio_front, vehicle.hydraulic_ratio_rear,
        vehicle.maximum_brake_pressure, vehicle.caliper_area, vehicle.brake_bias,
        vehicle.pad_mu_front, vehicle.pad_mu_rear,
        vehicle.brake_effective_radius_front, vehicle.brake_effective_radius_rear,
    )


def _raw_peak_pressure(vehicle, command_force: float) -> float:
    """Peak circuit pressure before brake_torques applies pressure clipping."""
    _, hydraulic_force = shape_brake_command(vehicle, command_force)
    curve = vehicle.pedal_curve
    pushrod = hydraulic_force * curve.ratio(hydraulic_force)
    share = 0.5 * (1.0 + vehicle.brake_bias)
    front = pushrod * share * vehicle.hydraulic_ratio_front / vehicle.caliper_area
    rear = pushrod * (1.0 - share) * vehicle.hydraulic_ratio_rear / vehicle.caliper_area
    return max(front, rear)


def _first_pressure_limit_force(vehicle) -> float:
    """Return first command force reaching pressure cap via monotone bisection."""
    cap = float(vehicle.maximum_brake_pressure)
    lo, hi = 0.0, float(vehicle.maximum_pedal_force)
    if _raw_peak_pressure(vehicle, hi) < cap:
        return hi
    # The returned point is the first representable command whose uncapped
    # pressure reaches the cap; hydraulic torque finalization still delegates
    # to the shared forward ``brake_torques`` implementation.
    for _ in range(100):
        mid = lo + 0.5 * (hi - lo)
        if mid == lo or mid == hi:
            break
        if _raw_peak_pressure(vehicle, mid) >= cap:
            hi = mid
        else:
            lo = mid
    return hi


def _validate_monotonic(vehicle) -> None:
    """Certify each affine-ratio segment's force*ratio product derivative.

    On a segment r(f)=a+b*f, d(f*r)/df=a+2*b*f is affine, so checking
    both segment endpoints proves its sign throughout. Pressure clipping is a
    monotone min with a constant, and calibration/deadband are nondecreasing.
    """
    curve = vehicle.pedal_curve
    maximum = float(vehicle.maximum_pedal_force)
    points = [0.0, maximum]
    if curve.pedal_force.size:
        points.extend(float(x) for x in curve.pedal_force if 0.0 < x < maximum)
    points = sorted(set(points))
    for lo, hi in zip(points, points[1:]):
        r0, r1 = curve.ratio(lo), curve.ratio(hi)
        slope = (r1 - r0) / (hi - lo)
        intercept = r0 - slope * lo
        if min(intercept + 2 * slope * lo, intercept + 2 * slope * hi) < -1e-12:
            raise ValueError("pedal curve force*ratio product is not monotone")
    calibration = getattr(vehicle.controller, "actuator_calibration", None)
    if calibration is not None:
        table = calibration.brake_force_fraction
        for a, b, va, vb in zip(table.axis, table.axis[1:], table.value, table.value[1:]):
            if vb < va:
                raise ValueError("brake actuator calibration must be monotone")


def invert_brake_effort(vehicle, requested_effort_nm: float) -> BrakeInverseResult:
    """Invert summed hydraulic torque magnitudes over pedal force [0,max]."""
    if not math.isfinite(requested_effort_nm) or requested_effort_nm < 0:
        raise ValueError("requested_effort_nm must be finite and nonnegative")
    controller = vehicle.controller
    if controller.regen_enabled:
        raise ValueError("combined regenerative braking is unsupported; disable regen for hydraulic inversion")
    _validate_monotonic(vehicle)
    maximum = float(vehicle.maximum_pedal_force)

    def evaluate(force):
        result = _map(vehicle, force)
        return float(np.sum(result.wheel_torque_magnitudes)), result

    cap, cap_result = evaluate(maximum)
    if requested_effort_nm > cap:
        force, mapped = maximum, cap_result
        achieved = cap
        feasible, constraint = False, "maximum_pedal_force"
    elif requested_effort_nm == 0:
        force, mapped = 0.0, _map(vehicle, 0.0)
        achieved, feasible, constraint = 0.0, True, "none"
    elif requested_effort_nm == cap and cap_result.is_pressure_limited:
        # At the overall capped maximum, the pressure transition is the first
        # physical reaching force. Avoid sub-ULP torque-sum equality before
        # brake_torques toggles its pressure-limited flag.
        force = _first_pressure_limit_force(vehicle)
        mapped = _map(vehicle, force)
        achieved, feasible, constraint = cap, True, "none"
    else:
        # Find the first positive region that reaches target, selecting the
        # smallest force across any initial deadband/flat calibration interval.
        knots = [0.0, maximum]
        curve = vehicle.pedal_curve
        knots.extend(float(x) for x in curve.pedal_force if 0 < x < maximum)
        cal = getattr(controller, "actuator_calibration", None)
        if cal is not None:
            knots.extend(float(x) * maximum for x in cal.brake_force_fraction.axis)
        knots = sorted(set(knots))
        left = 0.0
        right = None
        for point in knots[1:]:
            if evaluate(point)[0] >= requested_effort_nm:
                right = point
                break
            left = point
        if right is None:
            force, mapped, achieved, feasible, constraint = maximum, cap_result, cap, True, "none"
        elif evaluate(left)[0] >= requested_effort_nm:
            force, mapped = left, _map(vehicle, left)
            achieved, feasible, constraint = evaluate(force)[0], True, "none"
        else:
            # Lower-bound bisection, not a zero-of-residual solver: when the
            # target is exactly the level of a flat cap, all plateau points
            # are roots and ordinary brentq need not return the first one.
            low, high = left, right
            for _ in range(100):
                midpoint = low + 0.5 * (high - low)
                if midpoint == low or midpoint == high:
                    break
                if evaluate(midpoint)[0] >= requested_effort_nm:
                    high = midpoint
                else:
                    low = midpoint
            force = high
            achieved, mapped = evaluate(force)
            feasible, constraint = True, "none"
    return BrakeInverseResult(force, mapped.wheel_torques,
                              float(achieved), float(achieved-requested_effort_nm),
                              feasible, constraint)
