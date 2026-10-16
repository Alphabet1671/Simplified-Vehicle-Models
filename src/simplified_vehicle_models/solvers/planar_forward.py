"""Adapt the transient planar vehicle to the shared forward driver.

Wheel speeds remain dynamic states. Tire forces use the current coupled state.
The adapter supports forward rolling and the configured fixed motor layout.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.controls.brakes import brake_torques
from simplified_vehicle_models.formulations.planar_7dof import (
    PlanarState, evaluate_normal_loads, evaluate_planar_dynamics,
    contact_velocities, slip_quantities, wheel_positions, wheel_steering_angles,
)
from simplified_vehicle_models.powertrain.energy_storage import (
    StorageState, evaluate_storage, solve_current, terminal_voltage,
)
from scipy.optimize import brentq

from simplified_vehicle_models.powertrain.motor_envelope import (
    INVERTER_SPEED_REGULATION_BAND, motor_speed_limit, speed_regulated_torque_limit,
)
from simplified_vehicle_models.tires.unitire_simple import loaded_radius
from simplified_vehicle_models.solvers.forward_driver import (
    DriverCommand, DriverObservation, RateSystem, SpeedPlan, TrialStateError,
)


def body_accelerations(state: np.ndarray, rates: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return physical body-axis accelerations from the planar state rates.

    Velocity derivatives alone omit the rotating-frame terms. In particular,
    a steady corner has ``v_y_dot == 0`` but ``a_y == yaw_rate * v_x``.
    Supports a single 12-state row or arrays of rows.
    """
    state = np.asarray(state, dtype=float)
    rates = np.asarray(rates, dtype=float)
    if state.shape != rates.shape or state.shape[-1] != 12:
        raise ValueError("planar states and rates must have matching 12-state layouts")
    return (rates[..., 3] - state[..., 5] * state[..., 4],
            rates[..., 4] + state[..., 5] * state[..., 3])


@dataclass(frozen=True)
class ForwardPowertrain:
    wheel_torques: np.ndarray
    motor_wheel_torques: np.ndarray
    bus_power: float
    voltage: float
    current: float
    power_residual: float
    limited: bool


class PlanarForwardPlant:
    """Own twelve states, including four wheel speeds and steering.

    Layout: s, n, heading error, v_x, v_y, yaw rate, four wheel speeds,
    state of charge, and road-wheel steering angle.
    """

    state_size = 12
    periodic_indices = (1, 2, 3, 4, 5, 6, 7, 8, 9, 11)

    def __init__(self, vehicle, plan: SpeedPlan):
        self.vehicle = vehicle
        self.plan = plan
        transmission = vehicle.drivetrain.drivetrain.transmission
        wheel_index = {name: i for i, name in enumerate(("fl", "fr", "rl", "rr"))}
        self.driven = np.array([wheel_index[name] for name in transmission.driven_wheels])
        if len(self.driven) != vehicle.motor.unit_count:
            raise ValueError("motor count must match driven wheels")
        self.gear_efficiency = transmission.gear_efficiency
        self.gearing = vehicle.motor.gear_ratio * self.gear_efficiency
        self.maximum_load_residual = 0.
        self.minimum_contact_load = float("inf")
        self.maximum_power_residual = 0.
        self.maximum_load_iterations = 0
        #: Load-closure diagnostics: solves finished by each method and the
        #: smallest ``|det(I - J)|`` of the load map seen at a Newton solve.
        self.load_method_counts = {"newton": 0, "fixed-point": 0}
        self.minimum_load_jacobian_determinant = float("inf")
        #: Diagnostics: evaluations with a lifted (zero-load) wheel, with a
        #: driven motor inside its inverter speed-regulation band, and the
        #: largest motor speed as a fraction of its voltage-scaled limit.
        self.lifted_wheel_evaluations = 0
        self.speed_regulated_evaluations = 0
        self.maximum_motor_speed_ratio = 0.
        self.evaluations = 0
        self.last_time = 0.
        self.last_distance = 0.

    def mechanical_state(self, state):
        return PlanarState(float(state[3]), float(state[4]), float(state[5]),
                           np.asarray(state[6:10]), float(state[10]))

    def initial_state(self, speed: float) -> np.ndarray:
        if not np.isfinite(speed) or speed <= 1.:
            raise ValueError("initial forward speed must exceed 1 m/s")
        _, curvature, _ = self.plan.sample(0.)
        state = np.zeros(self.state_size)
        state[3] = speed
        state[5] = speed * curvature
        # Align rear contact velocity before the dynamic tire forces develop.
        state[4] = self.vehicle.front_mass_fraction * self.vehicle.wheelbase * state[5]
        state[2] = -np.arctan2(state[4], state[3])
        state[10] = self.vehicle.storage.initial_state_of_charge
        state[11] = np.arctan(self.vehicle.wheelbase * curvature)
        if abs(state[11]) > self.vehicle.steering_maximum_angle:
            raise ValueError("initial curvature exceeds steering authority")
        loads = evaluate_normal_loads(self.vehicle, speed, 0., speed**2 * curvature)
        radius = loaded_radius(self.vehicle.tire, loads)
        longitudinal, _ = contact_velocities(self.mechanical_state(state),
            wheel_positions(self.vehicle), wheel_steering_angles(state[11]))
        state[6:10] = longitudinal / radius
        return state

    def observe(self, state) -> DriverObservation:
        # Report how close the most-locked wheel's braking slip ratio is to
        # full lock (slip ratio -1, wheel stationary while the car still
        # moves). PreviewDriver uses this to back its brake command off
        # before a wheel actually locks; it would otherwise command the
        # open-loop hydraulic brake torque needed for the full point-mass
        # envelope deceleration, which this low-wheel-inertia car cannot
        # absorb without instantly spinning a wheel backward (see
        # TrialStateError below). Slip ratio (not a wheel-speed-minus-body-
        # speed proxy) is used because each wheel's own loaded radius varies
        # with load transfer; computing a speed-domain margin from a single
        # nominal radius was swamped by that several-percent radius spread
        # and misfired during ordinary driving, not just near lock.
        mechanical = self.mechanical_state(state)
        loads = evaluate_normal_loads(self.vehicle, float(np.hypot(state[3], state[4])), 0., 0.)
        radii = loaded_radius(self.vehicle.tire, loads)
        longitudinal, lateral = contact_velocities(
            mechanical, wheel_positions(self.vehicle), wheel_steering_angles(state[11]))
        slip_ratio, _ = slip_quantities(np.asarray(state[6:10]), radii, longitudinal, lateral, 1.0)
        # slip_ratio is negative under braking and -1 at full lock; margin
        # above lock is the distance from that floor, clamped so an already
        # driving (positive-slip) wheel reports a large, saturating margin.
        wheel_speed_margin = float(np.min(slip_ratio)) + 1.0
        return DriverObservation(float(state[0]), float(state[1]), float(state[2]),
                                 float(state[3]), float(state[5]), float(state[11]), float(state[4]),
                                 wheel_speed_margin)

    def powertrain(self, state, command: DriverCommand) -> ForwardPowertrain:
        vehicle = self.vehicle
        motor = vehicle.motor
        storage = vehicle.storage
        controller = vehicle.controller
        inputs = [command.throttle, command.brake_force, command.steering_rate]
        if (not np.all(np.isfinite(inputs)) or not 0. <= command.throttle <= 1.
                or not 0. <= command.brake_force <= vehicle.maximum_pedal_force
                or abs(command.steering_rate) > vehicle.steering_maximum_rate + 1.e-10
                or command.throttle * command.brake_force > 1.e-12):
            raise ValueError("driver command exceeds physical bounds or overlaps pedals")
        charge_state = StorageState(float(state[10]))
        limits = evaluate_storage(storage, charge_state)
        resistance = storage.series_count / storage.parallel_count * float(np.interp(
            state[10], storage.state_of_charge_axis, storage.resistance_axis))
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
        wheel_speed = np.asarray(state[6:10])
        motor_speed = motor.gear_ratio * wheel_speed[self.driven]
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
            torque[self.driven] = (drive - regen) * self.gearing
            wheel_power = torque * wheel_speed
            shaft_power = np.where(wheel_power >= 0., wheel_power / self.gear_efficiency,
                                   wheel_power * self.gear_efficiency)
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
        brake = brake_torques(hydraulic_force / vehicle.maximum_pedal_force,
            vehicle.maximum_pedal_force, vehicle.pedal_curve, vehicle.hydraulic_ratio,
            vehicle.maximum_brake_pressure, vehicle.caliper_area, vehicle.brake_bias,
            vehicle.pad_mu_front, vehicle.pad_mu_rear,
            vehicle.brake_effective_radius_front, vehicle.brake_effective_radius_rear)
        residual = abs(demand - voltage * current)
        if residual > 1.e-6:
            raise RuntimeError("electrical power balance failed")
        return ForwardPowertrain(torque + brake.wheel_torques, torque, demand,
                                 voltage, current, residual, limited)

    def evaluate(self, time, state, command) -> RateSystem:
        state = np.asarray(state, dtype=float)
        if state.shape != (self.state_size,) or not np.all(np.isfinite(state)):
            raise ValueError("invalid planar forward state")
        self.last_time = float(time)
        self.last_distance = float(state[0])
        if not 0. <= state[10] <= 1.:
            raise TrialStateError(f"trial state of charge outside bounds: {state[10]:.6g}")
        if state[3] <= 0.25 or np.any(state[6:10] < 0.):
            raise TrialStateError("prototype requires forward body and wheel motion")
        if abs(state[11]) > self.vehicle.steering_maximum_angle + 1.e-7:
            raise ValueError("steering angle exceeds the physical bound")
        point = self.powertrain(state, command)
        dynamics = evaluate_planar_dynamics(self.vehicle, self.mechanical_state(state),
                                            point.wheel_torques, state[11])
        if not dynamics.load_converged:
            raise RuntimeError(f"load closure failed: residual={dynamics.load_residual:.6g}")
        # A lifted wheel carries exactly zero load and is a valid physical
        # state: its tire produces no force and a driven lifted wheel spins
        # up into the inverter's speed regulation. Only a negative load,
        # which the load-transfer model reserves for a two-wheel (rollover)
        # state, is outside the plant domain.
        if np.any(dynamics.normal_loads < 0.):
            raise TrialStateError("planar contact load is negative: both inside wheels lifted (rollover)")
        self.minimum_contact_load = min(self.minimum_contact_load, float(np.min(dynamics.normal_loads)))
        _, curvature, _ = self.plan.sample(state[0])
        denominator = 1. - curvature * state[1]
        if denominator < 0.5:
            raise RuntimeError("path coordinate denominator is invalid")
        progress = (state[3] * np.cos(state[2]) - state[4] * np.sin(state[2])) / denominator
        if progress <= 0.25:
            raise RuntimeError("positive path progress constraint failed")
        rate = np.zeros(self.state_size)
        rate[0] = progress
        rate[1] = state[3] * np.sin(state[2]) + state[4] * np.cos(state[2])
        rate[2] = state[5] - curvature * progress
        rate[3:6] = [dynamics.longitudinal_acceleration, dynamics.lateral_acceleration,
                     dynamics.yaw_acceleration]
        rate[6:10] = dynamics.wheel_accelerations
        rate[10] = -point.current / self.vehicle.storage.pack_capacity
        rate[11] = command.steering_rate
        self.maximum_load_residual = max(self.maximum_load_residual, dynamics.load_residual)
        self.maximum_power_residual = max(self.maximum_power_residual, point.power_residual)
        self.maximum_load_iterations = max(self.maximum_load_iterations, dynamics.load_iterations)
        self.load_method_counts[dynamics.load_method] += 1
        if np.isfinite(dynamics.load_jacobian_determinant):
            self.minimum_load_jacobian_determinant = min(
                self.minimum_load_jacobian_determinant, dynamics.load_jacobian_determinant)
        if np.any(dynamics.normal_loads == 0.):
            self.lifted_wheel_evaluations += 1
        speed_ratio = float(np.max(self.vehicle.motor.gear_ratio * state[6:10][self.driven])
                            / motor_speed_limit(self.vehicle.motor, point.voltage))
        if speed_ratio > 1. - INVERTER_SPEED_REGULATION_BAND:
            self.speed_regulated_evaluations += 1
        self.maximum_motor_speed_ratio = max(self.maximum_motor_speed_ratio, speed_ratio)
        self.evaluations += 1
        return RateSystem(rate)
