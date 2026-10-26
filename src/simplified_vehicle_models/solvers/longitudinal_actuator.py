"""Invert the existing fixed-split forward powertrain for total wheel torque.

This is a local actuator primitive at fixed wheel speeds and SOC. It does not
allocate per-wheel torque or certify tire forces, body acceleration, or tracking.
"""
from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Callable, Mapping
import math

import numpy as np

from simplified_vehicle_models.solvers.forward_driver import DriverCommand


@dataclass(frozen=True)
class ActuatorInversionSettings:
    torque_tolerance_nm: float = 1.e-5
    maximum_iterations: int = 60
    root_method: str = "bisection"

    def __post_init__(self):
        if (not isinstance(self.root_method, str)
                or self.root_method not in ("bisection", "safeguarded_interpolation")):
            raise ValueError("root_method must be 'bisection' or 'safeguarded_interpolation'")
        if (isinstance(self.torque_tolerance_nm, (bool, np.bool_))
                or not isinstance(self.torque_tolerance_nm, (int, float, np.number))
                or not math.isfinite(float(self.torque_tolerance_nm))
                or self.torque_tolerance_nm <= 0):
            raise ValueError("torque_tolerance_nm must be finite and positive")
        if (isinstance(self.maximum_iterations, (bool, np.bool_))
                or not isinstance(self.maximum_iterations, (int, np.integer))
                or self.maximum_iterations <= 0):
            raise ValueError("maximum_iterations must be a positive integer")


@dataclass(frozen=True)
class InversionResult:
    throttle: float
    brake_force: float
    achieved_total_wheel_torque_nm: float
    requested_total_wheel_torque_nm: float
    torque_error_nm: float
    saturated: bool
    lower_reachable_torque_nm: float
    upper_reachable_torque_nm: float
    evaluations: int
    diagnostics: Mapping[str, object]


def _finite_real(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.number)):
        raise ValueError(f"{name} must be a finite real number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def invert_longitudinal_actuators(vehicle, wheel_speed, soc,
                                  requested_total_wheel_torque_nm, *,
                                  settings=None, evaluator=None) -> InversionResult:
    """Find exclusive throttle/brake command matching requested net wheel torque.

    ``maximum_iterations`` bounds the root-refinement loop. With opt-in
    safeguarded interpolation, one additional pre-loop secant candidate may be
    evaluated after the coarse monotone samples; that candidate is independently
    checked against the actual evaluator residual and bracket monotonicity.
    """
    requested = _finite_real(requested_total_wheel_torque_nm, "requested torque")
    soc = _finite_real(soc, "soc")
    if not 0. <= soc <= 1.:
        raise ValueError("soc must be in [0, 1]")
    try:
        speeds = np.array(wheel_speed, dtype=float, copy=True)
    except (TypeError, ValueError) as error:
        raise ValueError("wheel_speed must be a finite four-vector") from error
    if speeds.shape != (4,) or not np.all(np.isfinite(speeds)) or np.any(speeds < 0.):
        raise ValueError("wheel_speed must be a finite nonnegative four-vector")
    settings = ActuatorInversionSettings() if settings is None else settings
    if not isinstance(settings, ActuatorInversionSettings):
        raise ValueError("settings must be ActuatorInversionSettings")
    if vehicle.controller.regen_enabled:
        raise NotImplementedError("longitudinal inversion does not support regen-enabled controllers")
    transmission = vehicle.drivetrain.drivetrain.transmission
    index = {name: i for i, name in enumerate(("fl", "fr", "rl", "rr"))}
    try:
        driven = np.array([index[name] for name in transmission.driven_wheels], dtype=int)
    except (KeyError, TypeError) as error:
        raise ValueError("transmission has invalid driven_wheels") from error
    if driven.size != vehicle.motor.unit_count or driven.size == 0 or len(set(driven.tolist())) != driven.size:
        raise ValueError("motor count must match distinct driven wheels")
    gearing = vehicle.motor.gear_ratio * transmission.gear_efficiency
    if evaluator is None:
        from simplified_vehicle_models.solvers.forward_powertrain import evaluate_forward_powertrain
        evaluator = evaluate_forward_powertrain
    if not callable(evaluator):
        raise ValueError("evaluator must be callable")
    maximum_brake = _finite_real(vehicle.maximum_pedal_force, "maximum pedal force")
    if maximum_brake <= 0:
        raise ValueError("maximum pedal force must be positive")
    cache = {}

    def query(throttle, brake):
        key = (float(throttle), float(brake))
        if key not in cache:
            output = evaluator(vehicle, driven, gearing, transmission.gear_efficiency,
                              speeds, soc, DriverCommand(key[0], key[1], 0.))
            torques = np.asarray(output.wheel_torques, dtype=float)
            if torques.shape != (4,) or not np.all(np.isfinite(torques)):
                raise RuntimeError("powertrain evaluator returned invalid wheel torques")
            if hasattr(output, "power_residual") and not math.isfinite(float(output.power_residual)):
                raise RuntimeError("powertrain evaluator returned invalid power residual")
            cache[key] = (float(np.sum(torques)), output)
        return cache[key]

    coast = query(0., 0.)[0]
    if requested >= coast:
        end = query(1., 0.)[0]
        low_command, high_command = 0., 1.
        low_torque, high_torque = coast, end
        pedal = "throttle"
        brake_endpoint = query(0., maximum_brake)[0]
        reachable_min, reachable_max = min(brake_endpoint, coast), max(end, coast)
    else:
        end = query(0., maximum_brake)[0]
        low_command, high_command = 0., maximum_brake
        low_torque, high_torque = coast, end
        pedal = "brake_force"
        drive_endpoint = query(1., 0.)[0]
        reachable_min, reachable_max = min(end, coast), max(drive_endpoint, coast)
    increasing = pedal == "throttle"
    if (increasing and end < coast) or (not increasing and end > coast):
        raise RuntimeError("actuator endpoint torque map is not monotone")
    # Endpoints alone cannot establish monotonicity. Sample the active pedal
    # branch before deciding feasibility, including out-of-range requests.
    prior = coast
    for fraction in (.25, .5, .75):
        probe = high_command * fraction
        sampled = query(probe, 0.)[0] if pedal == "throttle" else query(0., probe)[0]
        if (increasing and sampled < prior - settings.torque_tolerance_nm) or (
                not increasing and sampled > prior + settings.torque_tolerance_nm):
            raise RuntimeError("actuator torque map is not monotone")
        prior = sampled
    if (increasing and prior > end + settings.torque_tolerance_nm) or (
            not increasing and prior < end - settings.torque_tolerance_nm):
        raise RuntimeError("actuator torque map is not monotone")
    if requested < reachable_min or requested > reachable_max:
        command = low_command if abs(requested-low_torque) <= abs(requested-high_torque) else high_command
        torque = query(command, 0.)[0] if pedal == "throttle" else query(0., command)[0]
        saturated = True
    else:
        left, right = low_command, high_command
        left_torque, right_torque = low_torque, high_torque
        command = torque = None
        if settings.root_method == "safeguarded_interpolation":
            samples = sorted((key[0] if pedal == "throttle" else key[1], value[0])
                             for key, value in cache.items()
                             if ((key[1] == 0.) if pedal == "throttle" else (key[0] == 0.)))
            for (command_a, torque_a), (command_b, torque_b) in zip(samples, samples[1:]):
                if min(torque_a, torque_b) <= requested <= max(torque_a, torque_b):
                    left, left_torque = command_a, torque_a
                    right, right_torque = command_b, torque_b
                    break
            command = torque = None
            saturated = False
            if (abs(requested - left_torque) > settings.torque_tolerance_nm
                    and abs(requested - right_torque) > settings.torque_tolerance_nm):
                denominator = right_torque - left_torque
                if denominator != 0.:
                    first = left + (requested - left_torque) * (right - left) / denominator
                    if left < first < right:
                        first_torque = query(first, 0.)[0] if pedal == "throttle" else query(0., first)[0]
                        if (first_torque < min(left_torque, right_torque) - settings.torque_tolerance_nm
                                or first_torque > max(left_torque, right_torque) + settings.torque_tolerance_nm):
                            raise RuntimeError("actuator torque map is not monotone")
                        if abs(first_torque - requested) <= settings.torque_tolerance_nm:
                            command, torque, saturated = first, first_torque, False
                        elif (first_torque < requested) == increasing:
                            left, left_torque = first, first_torque
                        else:
                            right, right_torque = first, first_torque
        if command is None and abs(requested-left_torque) <= settings.torque_tolerance_nm:
            command, torque, saturated = left, left_torque, False
        elif command is None and abs(requested-right_torque) <= settings.torque_tolerance_nm:
            command, torque, saturated = right, right_torque, False
        elif command is None:
            saturated = False
            for _ in range(settings.maximum_iterations):
                width = right - left
                middle = (left + right) / 2.
                candidate = middle
                if settings.root_method == "safeguarded_interpolation":
                    denominator = right_torque - left_torque
                    if math.isfinite(denominator) and denominator != 0.:
                        secant = left + (requested - left_torque) * width / denominator
                        margin = .05 * width
                        if math.isfinite(secant) and left + margin < secant < right - margin:
                            candidate = secant
                result_torque = query(candidate, 0.)[0] if pedal == "throttle" else query(0., candidate)[0]
                if ((increasing and (result_torque < left_torque - settings.torque_tolerance_nm
                                     or result_torque > right_torque + settings.torque_tolerance_nm))
                        or (not increasing and (result_torque > left_torque + settings.torque_tolerance_nm
                                                or result_torque < right_torque - settings.torque_tolerance_nm))):
                    raise RuntimeError("actuator torque map is not monotone")
                if abs(result_torque-requested) <= settings.torque_tolerance_nm:
                    command, torque = candidate, result_torque
                    break
                if (right - left) <= 2. * np.finfo(float).eps * max(abs(left), abs(right), 1.):
                    best_error = min(abs(left_torque - requested), abs(right_torque - requested))
                    if best_error > settings.torque_tolerance_nm:
                        raise RuntimeError("actuator map has no torque-matching solution at requested tolerance")
                    command, torque = ((left, left_torque) if abs(left_torque-requested) <= abs(right_torque-requested)
                                       else (right, right_torque))
                    break
                if (result_torque < requested) == increasing:
                    left, left_torque = candidate, result_torque
                else:
                    right, right_torque = candidate, result_torque
            if command is None:
                raise RuntimeError("actuator inversion did not converge within maximum_iterations")
    throttle, brake = (command, 0.) if pedal == "throttle" else (0., command)
    if throttle * brake > 1.e-12:
        raise RuntimeError("inversion produced overlapping pedals")
    error = torque - requested
    diagnostics = MappingProxyType({
        "claim": "actuator torque inversion only; tire forces/body acceleration/tracking not certified",
        "regen_supported": False, "no_tv": True,
        "powertrain_limited": bool(getattr(query(throttle, brake)[1], "limited", False)),
    })
    return InversionResult(float(throttle), float(brake), float(torque), requested,
                           float(error), bool(saturated), float(reachable_min),
                           float(reachable_max), len(cache), diagnostics)
