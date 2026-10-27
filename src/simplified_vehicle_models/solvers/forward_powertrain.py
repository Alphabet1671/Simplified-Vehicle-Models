"""Shared forward-driver powertrain closure.

The scalar voltage closure below was the body of
``PlanarForwardPlant.powertrain`` (``solvers/planar_forward.py``). Task 7 moved
it here unchanged so the planar and vertical forward plants share one
implementation; the arithmetic and its order are preserved bitwise so recorded
planar outputs do not move.

``ForwardPowertrain`` is re-exported from ``planar_forward`` for compatibility.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import brentq

from simplified_vehicle_models.controls.brakes import brake_torques
from simplified_vehicle_models.powertrain.energy_storage import (
    StorageState, evaluate_storage, solve_current, terminal_voltage,
)
from simplified_vehicle_models.powertrain.motor_envelope import speed_regulated_torque_limit
from simplified_vehicle_models.solvers.forward_driver import DriverCommand


@dataclass(frozen=True)
class ForwardPowertrain:
    wheel_torques: np.ndarray
    motor_wheel_torques: np.ndarray
    bus_power: float
    voltage: float
    current: float
    power_residual: float
    limited: bool


def evaluate_forward_powertrain(vehicle, driven, gearing, gear_efficiency,
                                wheel_speed, soc, command: DriverCommand) -> ForwardPowertrain:
    """Return the wheel torques and electrical operating point of one command.

    ``wheel_speed`` is the four wheel angular speeds in FL, FR, RL, RR order and
    ``soc`` the battery state of charge. ``gearing`` is the motor gear ratio
    already multiplied by ``gear_efficiency`` (the historic plant attribute), and
    ``driven`` the indices of the driven wheels.
    """
    motor = vehicle.motor
    storage = vehicle.storage
    controller = vehicle.controller
    inputs = [command.throttle, command.brake_force, command.steering_rate]
    if (not np.all(np.isfinite(inputs)) or not 0. <= command.throttle <= 1.
            or not 0. <= command.brake_force <= vehicle.maximum_pedal_force
            or abs(command.steering_rate) > vehicle.steering_maximum_rate + 1.e-10
            or command.throttle * command.brake_force > 1.e-12):
        raise ValueError("driver command exceeds physical bounds or overlaps pedals")
    charge_state = StorageState(float(soc))
    limits = evaluate_storage(storage, charge_state)
    resistance = storage.series_count / storage.parallel_count * float(np.interp(
        soc, storage.state_of_charge_axis, storage.resistance_axis))
    minimum_voltage = storage.series_count * storage.minimum_cell_voltage
    maximum_voltage = min(storage.series_count * storage.maximum_cell_voltage,
                          motor.maximum_bus_voltage)
    discharge_current = min(storage.discharge_current_limit,
                            max((limits.bus_voltage - minimum_voltage) / resistance, 0.),
                            limits.bus_voltage / (2 * resistance))
    charge_current = min(storage.charge_current_limit,
                         max((maximum_voltage - limits.bus_voltage) / resistance, 0.))
    maximum_power = (limits.bus_voltage - resistance * discharge_current) * discharge_current
    minimum_power = -(limits.bus_voltage + resistance * charge_current) * charge_current
    wheel_speed = np.asarray(wheel_speed)
    motor_speed = motor.gear_ratio * wheel_speed[driven]
    requested = command.throttle * min(motor.peak_motor_torque,
                                       controller.maximum_drive_motor_torque_nm)
    regen_fraction = (min(command.brake_force / controller.regen_pedal_force_limit, 1.)
                      if controller.regen_enabled else 0.)
    requested_regen = regen_fraction * controller.regen_motor_torque_limit

    def operating_point(voltage):
        # Speed regulation tapers positive drive only. Regeneration has its
        # own torque rating and is constrained by battery charge-current
        # and power feasibility below.
        available = speed_regulated_torque_limit(motor, motor_speed, voltage)
        drive = np.minimum(requested, available)
        regen_available = motor.peak_motor_torque * motor.regeneration_torque_fraction
        regen = np.minimum(requested_regen, regen_available)
        torque = np.zeros(4)
        torque[driven] = (drive - regen) * gearing
        wheel_power = torque * wheel_speed
        shaft_power = np.where(wheel_power >= 0., wheel_power / gear_efficiency,
                               wheel_power * gear_efficiency)
        demand = float(np.sum(np.where(shaft_power >= 0., shaft_power / motor.drive_efficiency,
                                      shaft_power * motor.drive_efficiency)))
        scale = 1.
        if demand > maximum_power:
            scale = maximum_power / demand
        elif demand < minimum_power:
            scale = minimum_power / demand
        if scale < 1.:
            scale *= (1. - 1.e-12)
            torque = torque * scale
            demand *= scale
        current, feasible = solve_current(storage, charge_state, demand)
        if not feasible:
            raise RuntimeError("battery current solve is infeasible")
        limited = bool(scale < 1. or np.any(drive < requested)
                       or np.any(regen < requested_regen))
        return torque, demand, current, terminal_voltage(storage, charge_state, current), limited

    # The terminal voltage closes a scalar loop: voltage -> motor envelope
    # -> bus power -> current -> terminal voltage. The residual
    # ``voltage - terminal(voltage)`` is continuous and nondecreasing in
    # voltage (more voltage never lowers the torque envelope, so it never
    # lowers the discharge current), and it is nonpositive at the minimum
    # voltage and nonnegative at the maximum voltage because the current
    # limits above keep the terminal voltage inside that window. Solve it
    # with a bracketed root instead of a damped fixed point, which fails to
    # contract when a spinning wheel puts the motor in its speed-regulation
    # band.
    def residual(voltage):
        return voltage - operating_point(voltage)[3]

    lower = minimum_voltage
    upper = max(maximum_voltage, limits.bus_voltage)
    lower_residual = residual(lower)
    upper_residual = residual(upper)
    if lower_residual >= 0.:
        voltage = lower
    elif upper_residual <= 0.:
        voltage = upper
    else:
        voltage = brentq(residual, lower, upper, xtol=1.e-10, rtol=1.e-15, maxiter=200)
    torque, demand, current, next_voltage, limited = operating_point(voltage)
    if abs(next_voltage - voltage) > 1.e-6:
        raise RuntimeError("motor and battery voltage closure did not converge")
    voltage = next_voltage
    if voltage < minimum_voltage - 1.e-7 or voltage > maximum_voltage + 1.e-7:
        raise RuntimeError("battery voltage constraint violated")
    hydraulic_force = max(command.brake_force - controller.hydraulic_deadband_n, 0.)
    # The hydraulic chain takes the physical pedal force in newtons.
    brake = brake_torques(hydraulic_force,
        vehicle.maximum_pedal_force, vehicle.pedal_curve,
        vehicle.hydraulic_ratio_front, vehicle.hydraulic_ratio_rear,
        vehicle.maximum_brake_pressure, vehicle.caliper_area, vehicle.brake_bias,
        vehicle.pad_mu_front, vehicle.pad_mu_rear,
        vehicle.brake_effective_radius_front, vehicle.brake_effective_radius_rear)
    residual = abs(demand - voltage * current)
    if residual > 1.e-6:
        raise RuntimeError("electrical power balance failed")
    return ForwardPowertrain(torque + brake.wheel_torques, torque, demand,
                             voltage, current, residual, limited)
