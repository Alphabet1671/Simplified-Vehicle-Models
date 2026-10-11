from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math

import numpy as np

from simplified_vehicle_models.core.conventions import GRAVITY
from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.parameters.vehicle import PointMassVehicleParameters


@dataclass(frozen=True, slots=True)
class EquilibriumState:
    """Store one prescribed-acceleration force equilibrium."""

    longitudinal_acceleration: float
    lateral_acceleration: float
    normal_loads: FloatArray
    longitudinal_capacities: FloatArray
    lateral_capacities: FloatArray
    longitudinal_forces: FloatArray
    lateral_forces: FloatArray
    combined_utilization: FloatArray
    drive_force_limit: float
    brake_force_limit: float
    drag_force: float
    active_longitudinal_constraint: str
    force_balance_residual: float
    yaw_moment_residual: float
    contact_feasible: bool
    lateral_feasible: bool
    longitudinal_feasible: bool

    @property
    def is_feasible(self) -> bool:
        """Return true when all force constraints are feasible."""

        return (
            self.contact_feasible
            and self.lateral_feasible
            and self.longitudinal_feasible
        )


@dataclass(frozen=True, slots=True)
class AccelerationLimits:
    """Store live limits and their solved equilibrium states."""

    maximum_acceleration: float
    maximum_deceleration: float
    lateral_acceleration: float
    acceleration_state: EquilibriumState
    braking_state: EquilibriumState
    coasting_state: EquilibriumState


def aerodynamic_forces(
    vehicle: PointMassVehicleParameters,
    speed: float,
) -> tuple[float, float]:
    """Calculate downforce and drag at the specified speed."""

    dynamic_pressure = 0.5 * vehicle.aero.air_density * speed * speed
    return (
        dynamic_pressure * vehicle.aero.downforce_area,
        dynamic_pressure * vehicle.aero.drag_area,
    )


def normal_loads(
    vehicle: PointMassVehicleParameters,
    speed: float,
    longitudinal_acceleration: float,
    lateral_acceleration: float,
) -> FloatArray:
    """Calculate the rigid vertical load solution for each tire."""

    mass = vehicle.total_mass
    downforce, _ = aerodynamic_forces(vehicle, speed)
    front_axle = (
        mass * GRAVITY * vehicle.front_mass_fraction
        + downforce * vehicle.aero.aero_balance
        - mass
        * longitudinal_acceleration
        * vehicle.cg_height
        / vehicle.wheelbase
    )
    rear_axle = mass * GRAVITY + downforce - front_axle
    total_transfer = (
        mass * abs(lateral_acceleration) * vehicle.cg_height / vehicle.track_width
    )
    front_transfer = (
        total_transfer * vehicle.lateral_load_transfer_distribution
    )
    rear_transfer = total_transfer - front_transfer
    return np.asarray(
        [
            front_axle * 0.5 - front_transfer,
            front_axle * 0.5 + front_transfer,
            rear_axle * 0.5 - rear_transfer,
            rear_axle * 0.5 + rear_transfer,
        ],
        dtype=np.float64,
    )


def tire_force_capacities(
    vehicle: PointMassVehicleParameters,
    loads: FloatArray,
) -> tuple[FloatArray, FloatArray]:
    """Calculate pure force capacities for nonnegative contact loads."""

    contact_loads = np.maximum(loads, 0.0)
    tire = vehicle.tire
    load_factor = (
        tire.load_sensitivity_slope * contact_loads
        + tire.load_sensitivity_intercept
    )
    longitudinal_mu = np.maximum(tire.longitudinal_mu_scale * load_factor, 0.0)
    lateral_mu = np.maximum(tire.lateral_mu_scale * load_factor, 0.0)
    return longitudinal_mu * contact_loads, lateral_mu * contact_loads


def maximum_vehicle_speed(vehicle: PointMassVehicleParameters) -> float:
    """Calculate the speed limit from the maximum motor speed."""

    motor_speed = vehicle.drivetrain.maximum_motor_speed * 2.0 * math.pi / 60.0
    return motor_speed * vehicle.tire.effective_radius / vehicle.drivetrain.gear_ratio


def evaluate_equilibrium(
    vehicle: PointMassVehicleParameters,
    speed: float,
    curvature: float,
    longitudinal_acceleration: float,
) -> EquilibriumState:
    """Evaluate wheel forces for one prescribed vehicle acceleration."""

    lateral_acceleration = speed * speed * abs(curvature)
    loads = normal_loads(
        vehicle,
        speed,
        longitudinal_acceleration,
        lateral_acceleration,
    )
    contact_feasible = bool(np.all(loads >= -1.0e-9))
    longitudinal_capacity, lateral_capacity = tire_force_capacities(vehicle, loads)

    total_lateral_force = vehicle.total_mass * lateral_acceleration
    front_target = total_lateral_force * vehicle.front_mass_fraction
    rear_target = total_lateral_force - front_target
    lateral_forces = np.zeros(4, dtype=np.float64)
    front_ok = _allocate_axle_force(
        lateral_forces,
        lateral_capacity,
        slice(0, 2),
        front_target,
    )
    rear_ok = _allocate_axle_force(
        lateral_forces,
        lateral_capacity,
        slice(2, 4),
        rear_target,
    )
    lateral_feasible = front_ok and rear_ok and contact_feasible

    lateral_ratio = np.divide(
        lateral_forces,
        lateral_capacity,
        out=np.zeros(4, dtype=np.float64),
        where=lateral_capacity > 0.0,
    )
    remaining = longitudinal_capacity * np.sqrt(
        np.maximum(1.0 - lateral_ratio * lateral_ratio, 0.0)
    )
    drive_limit, drive_constraint = _available_drive_force(
        vehicle,
        speed,
        remaining,
    )
    brake_limit, brake_constraint = _available_brake_force(vehicle, remaining)
    _, drag = aerodynamic_forces(vehicle, speed)
    required_force = vehicle.total_mass * longitudinal_acceleration + drag
    longitudinal_forces = np.zeros(4, dtype=np.float64)
    if required_force >= 0.0:
        _allocate_total_force(longitudinal_forces, remaining, required_force)
        longitudinal_feasible = required_force <= drive_limit + 1.0e-8
        active_constraint = drive_constraint
    else:
        brake_demand = -required_force
        _allocate_brake_force(
            longitudinal_forces,
            remaining,
            brake_demand,
            vehicle.brake_bias,
        )
        longitudinal_feasible = brake_demand <= brake_limit + 1.0e-8
        active_constraint = brake_constraint

    longitudinal_ratio = np.divide(
        longitudinal_forces,
        longitudinal_capacity,
        out=np.zeros(4, dtype=np.float64),
        where=longitudinal_capacity > 0.0,
    )
    combined_utilization = np.sqrt(
        longitudinal_ratio * longitudinal_ratio
        + lateral_ratio * lateral_ratio
    )
    force_residual = (
        float(np.sum(longitudinal_forces))
        - drag
        - vehicle.total_mass * longitudinal_acceleration
    )
    distance_to_front = vehicle.wheelbase * (
        1.0 - vehicle.front_mass_fraction
    )
    distance_to_rear = vehicle.wheelbase * vehicle.front_mass_fraction
    lateral_moment = (
        float(np.sum(lateral_forces[:2])) * distance_to_front
        - float(np.sum(lateral_forces[2:])) * distance_to_rear
    )
    left_longitudinal_force = longitudinal_forces[0] + longitudinal_forces[2]
    right_longitudinal_force = longitudinal_forces[1] + longitudinal_forces[3]
    longitudinal_moment = (
        right_longitudinal_force - left_longitudinal_force
    ) * vehicle.track_width * 0.5
    yaw_residual = lateral_moment + longitudinal_moment
    arrays = (
        loads,
        longitudinal_capacity,
        lateral_capacity,
        longitudinal_forces,
        lateral_forces,
        combined_utilization,
    )
    for array in arrays:
        array.setflags(write=False)
    return EquilibriumState(
        longitudinal_acceleration=longitudinal_acceleration,
        lateral_acceleration=lateral_acceleration,
        normal_loads=loads,
        longitudinal_capacities=longitudinal_capacity,
        lateral_capacities=lateral_capacity,
        longitudinal_forces=longitudinal_forces,
        lateral_forces=lateral_forces,
        combined_utilization=combined_utilization,
        drive_force_limit=drive_limit,
        brake_force_limit=brake_limit,
        drag_force=drag,
        active_longitudinal_constraint=active_constraint,
        force_balance_residual=force_residual,
        yaw_moment_residual=yaw_residual,
        contact_feasible=contact_feasible,
        lateral_feasible=lateral_feasible,
        longitudinal_feasible=(
            longitudinal_feasible
            and lateral_feasible
            and float(np.max(combined_utilization)) <= 1.0 + 1.0e-8
        ),
    )


def maximum_corner_speed(
    vehicle: PointMassVehicleParameters,
    radius: float,
    maximum_speed: float = 60.0,
) -> float:
    """Solve the maximum steady speed for one unsigned radius."""

    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError("radius must be finite and positive")
    upper = min(maximum_speed, maximum_vehicle_speed(vehicle))
    lower = 0.0
    if evaluate_equilibrium(vehicle, upper, 1.0 / radius, 0.0).is_feasible:
        return upper
    for _ in range(32):
        trial = 0.5 * (lower + upper)
        state = evaluate_equilibrium(vehicle, trial, 1.0 / radius, 0.0)
        if state.is_feasible:
            lower = trial
        else:
            upper = trial
    return lower


def acceleration_limits(
    vehicle: PointMassVehicleParameters,
    speed: float,
    curvature: float,
) -> AccelerationLimits:
    """Solve live acceleration limits and their equilibrium states."""

    if not math.isfinite(speed) or speed < 0.0:
        raise ValueError("speed must be finite and nonnegative")
    if not math.isfinite(curvature):
        raise ValueError("curvature must be finite")
    curvature = abs(curvature)
    coasting = evaluate_equilibrium(vehicle, speed, curvature, 0.0)

    def drive_residual(acceleration: float) -> float:
        state = evaluate_equilibrium(vehicle, speed, curvature, acceleration)
        required = vehicle.total_mass * acceleration + state.drag_force
        if not state.contact_feasible or not state.lateral_feasible:
            return -required
        return state.drive_force_limit - required

    def brake_residual(deceleration: float) -> float:
        state = evaluate_equilibrium(vehicle, speed, curvature, -deceleration)
        required = vehicle.total_mass * deceleration - state.drag_force
        if not state.contact_feasible or not state.lateral_feasible:
            return -max(required, 0.0)
        return state.brake_force_limit - max(required, 0.0)

    maximum_acceleration = (
        _positive_root(drive_residual, 30.0) if coasting.lateral_feasible else 0.0
    )
    maximum_deceleration = (
        _positive_root(brake_residual, 30.0) if coasting.lateral_feasible else 0.0
    )
    acceleration_state = evaluate_equilibrium(
        vehicle,
        speed,
        curvature,
        maximum_acceleration,
    )
    braking_state = evaluate_equilibrium(
        vehicle,
        speed,
        curvature,
        -maximum_deceleration,
    )
    return AccelerationLimits(
        maximum_acceleration=maximum_acceleration,
        maximum_deceleration=maximum_deceleration,
        lateral_acceleration=coasting.lateral_acceleration,
        acceleration_state=acceleration_state,
        braking_state=braking_state,
        coasting_state=coasting,
    )


def _allocate_axle_force(
    output: FloatArray,
    capacity: FloatArray,
    indices: slice,
    target: float,
) -> bool:
    axle_capacity = float(np.sum(capacity[indices]))
    if target <= 1.0e-12:
        return True
    if axle_capacity <= 0.0:
        return False
    output[indices] = capacity[indices] * (target / axle_capacity)
    return target <= axle_capacity + 1.0e-8


def _allocate_total_force(
    output: FloatArray,
    capacity: FloatArray,
    target: float,
) -> None:
    side_target = target * 0.5
    left_capacity = capacity[0] + capacity[2]
    right_capacity = capacity[1] + capacity[3]
    if left_capacity > 0.0:
        output[[0, 2]] = capacity[[0, 2]] * (side_target / left_capacity)
    if right_capacity > 0.0:
        output[[1, 3]] = capacity[[1, 3]] * (side_target / right_capacity)


def _allocate_brake_force(
    output: FloatArray,
    _capacity: FloatArray,
    target: float,
    front_bias: float,
) -> None:
    front_wheel_target = target * front_bias * 0.5
    rear_wheel_target = target * (1.0 - front_bias) * 0.5
    output[:] = -np.asarray(
        [
            front_wheel_target,
            front_wheel_target,
            rear_wheel_target,
            rear_wheel_target,
        ],
        dtype=np.float64,
    )


def _available_drive_force(
    vehicle: PointMassVehicleParameters,
    speed: float,
    tire_limits: FloatArray,
) -> tuple[float, str]:
    if speed > maximum_vehicle_speed(vehicle) + 1.0e-9:
        return 0.0, "motor_speed"
    drivetrain = vehicle.drivetrain
    torque_force = (
        drivetrain.motor_count
        * drivetrain.peak_motor_torque
        * drivetrain.gear_ratio
        * drivetrain.efficiency
        / vehicle.tire.effective_radius
    )
    power_force = (
        math.inf
        if speed <= 1.0e-9
        else drivetrain.power_limit * drivetrain.efficiency / speed
    )
    left_limit = float(tire_limits[0] + tire_limits[2])
    right_limit = float(tire_limits[1] + tire_limits[3])
    limits = {
        "tire": 2.0 * min(left_limit, right_limit),
        "motor_torque": torque_force,
        "power": power_force,
    }
    constraint = min(limits, key=limits.get)
    return limits[constraint], constraint


def _available_brake_force(
    vehicle: PointMassVehicleParameters,
    tire_limits: FloatArray,
) -> tuple[float, str]:
    bias = vehicle.brake_bias
    limits = {
        "front_left_tire": 2.0 * float(tire_limits[0]) / bias,
        "front_right_tire": 2.0 * float(tire_limits[1]) / bias,
        "rear_left_tire": 2.0 * float(tire_limits[2]) / (1.0 - bias),
        "rear_right_tire": 2.0 * float(tire_limits[3]) / (1.0 - bias),
    }
    constraint = min(limits, key=limits.get)
    return limits[constraint], constraint


def _positive_root(function: Callable[[float], float], upper: float) -> float:
    lower = 0.0
    if function(lower) <= 0.0:
        return 0.0
    if function(upper) > 0.0:
        return upper
    for _ in range(32):
        trial = 0.5 * (lower + upper)
        if function(trial) > 0.0:
            lower = trial
        else:
            upper = trial
    return lower
