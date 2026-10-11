from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math

import numpy as np
import numpy.typing as npt

from simplified_vehicle_models.parameters import PointMassVehicleParameters


GRAVITY = 9.81
FloatArray = npt.NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class AccelerationLimits:
    """Store the live acceleration limits and wheel diagnostics."""

    maximum_acceleration: float
    maximum_deceleration: float
    lateral_acceleration: float
    drive_force_limit: float
    brake_force_limit: float
    drag_force: float
    normal_loads: FloatArray
    longitudinal_capacities: FloatArray
    lateral_capacities: FloatArray
    lateral_forces: FloatArray
    active_drive_constraint: str
    is_feasible: bool


def aerodynamic_forces(
    vehicle: PointMassVehicleParameters,
    speed: float,
) -> tuple[float, float]:
    """Calculate downforce and drag at the specified speed."""

    dynamic_pressure = 0.5 * vehicle.aero.air_density * speed * speed
    downforce = dynamic_pressure * vehicle.aero.downforce_area
    drag = dynamic_pressure * vehicle.aero.drag_area
    return downforce, drag


def normal_loads(
    vehicle: PointMassVehicleParameters,
    speed: float,
    longitudinal_acceleration: float,
    lateral_acceleration: float,
) -> FloatArray:
    """Calculate the vertical load for each tire."""

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

    total_lateral_transfer = (
        mass * abs(lateral_acceleration) * vehicle.cg_height / vehicle.track_width
    )
    front_transfer = (
        total_lateral_transfer * vehicle.lateral_load_transfer_distribution
    )
    rear_transfer = total_lateral_transfer - front_transfer
    loads = np.asarray(
        [
            front_axle * 0.5 - front_transfer,
            front_axle * 0.5 + front_transfer,
            rear_axle * 0.5 - rear_transfer,
            rear_axle * 0.5 + rear_transfer,
        ],
        dtype=np.float64,
    )
    return np.maximum(loads, 0.0)


def tire_force_capacities(
    vehicle: PointMassVehicleParameters,
    loads: FloatArray,
) -> tuple[FloatArray, FloatArray]:
    """Calculate pure longitudinal and lateral force capacities."""

    tire = vehicle.tire
    load_factor = tire.load_sensitivity_slope * loads + tire.load_sensitivity_intercept
    longitudinal_mu = np.maximum(tire.longitudinal_mu_scale * load_factor, 0.0)
    lateral_mu = np.maximum(tire.lateral_mu_scale * load_factor, 0.0)
    return longitudinal_mu * loads, lateral_mu * loads


def maximum_vehicle_speed(vehicle: PointMassVehicleParameters) -> float:
    """Calculate the speed limit from the maximum motor speed."""

    motor_speed = vehicle.drivetrain.maximum_motor_speed * 2.0 * math.pi / 60.0
    return motor_speed * vehicle.tire.effective_radius / vehicle.drivetrain.gear_ratio


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
    if _corner_is_feasible(vehicle, upper, radius):
        return upper
    for _ in range(32):
        trial = 0.5 * (lower + upper)
        if _corner_is_feasible(vehicle, trial, radius):
            lower = trial
        else:
            upper = trial
    return lower


def acceleration_limits(
    vehicle: PointMassVehicleParameters,
    speed: float,
    curvature: float,
) -> AccelerationLimits:
    """Solve the live acceleration limits at one path condition."""

    if not math.isfinite(speed) or speed < 0.0:
        raise ValueError("speed must be finite and nonnegative")
    if not math.isfinite(curvature):
        raise ValueError("curvature must be finite")

    lateral_acceleration = speed * speed * abs(curvature)
    base = _force_state(vehicle, speed, 0.0, lateral_acceleration)
    if not base[-1]:
        return AccelerationLimits(
            maximum_acceleration=0.0,
            maximum_deceleration=0.0,
            lateral_acceleration=lateral_acceleration,
            drive_force_limit=0.0,
            brake_force_limit=0.0,
            drag_force=aerodynamic_forces(vehicle, speed)[1],
            normal_loads=base[0],
            longitudinal_capacities=base[1],
            lateral_capacities=base[2],
            lateral_forces=base[3],
            active_drive_constraint="lateral",
            is_feasible=False,
        )

    maximum_acceleration = _solve_drive_acceleration(
        vehicle,
        speed,
        lateral_acceleration,
    )
    maximum_deceleration = _solve_braking_deceleration(
        vehicle,
        speed,
        lateral_acceleration,
    )
    drive_force, constraint = _available_drive_force(
        vehicle,
        speed,
        base[4],
    )
    brake_force = _available_brake_force(vehicle, base[4])
    _, drag = aerodynamic_forces(vehicle, speed)
    return AccelerationLimits(
        maximum_acceleration=maximum_acceleration,
        maximum_deceleration=maximum_deceleration,
        lateral_acceleration=lateral_acceleration,
        drive_force_limit=drive_force,
        brake_force_limit=brake_force,
        drag_force=drag,
        normal_loads=base[0],
        longitudinal_capacities=base[1],
        lateral_capacities=base[2],
        lateral_forces=base[3],
        active_drive_constraint=constraint,
        is_feasible=True,
    )


def _corner_is_feasible(
    vehicle: PointMassVehicleParameters,
    speed: float,
    radius: float,
) -> bool:
    lateral_acceleration = speed * speed / radius
    state = _force_state(vehicle, speed, 0.0, lateral_acceleration)
    if not state[-1]:
        return False
    remaining = state[4]
    _, drag = aerodynamic_forces(vehicle, speed)
    drive_force, _ = _available_drive_force(vehicle, speed, remaining)
    return drive_force + 1.0e-9 >= drag


def _force_state(
    vehicle: PointMassVehicleParameters,
    speed: float,
    longitudinal_acceleration: float,
    lateral_acceleration: float,
) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray, FloatArray, bool]:
    loads = normal_loads(
        vehicle,
        speed,
        longitudinal_acceleration,
        lateral_acceleration,
    )
    longitudinal, lateral = tire_force_capacities(vehicle, loads)
    demand = vehicle.total_mass * abs(lateral_acceleration)
    total_lateral_capacity = float(np.sum(lateral))
    feasible = demand <= total_lateral_capacity + 1.0e-9
    if total_lateral_capacity <= 0.0:
        lateral_forces = np.zeros(4, dtype=np.float64)
        feasible = demand <= 1.0e-9
    else:
        lateral_forces = lateral * min(demand / total_lateral_capacity, 1.0)
    utilization = np.divide(
        lateral_forces,
        lateral,
        out=np.zeros(4, dtype=np.float64),
        where=lateral > 0.0,
    )
    remaining = longitudinal * np.sqrt(np.maximum(1.0 - utilization * utilization, 0.0))
    return loads, longitudinal, lateral, lateral_forces, remaining, feasible


def _available_drive_force(
    vehicle: PointMassVehicleParameters,
    speed: float,
    tire_limits: FloatArray,
) -> tuple[float, str]:
    tire_force = float(np.sum(tire_limits))
    drivetrain = vehicle.drivetrain
    speed_limit = maximum_vehicle_speed(vehicle)
    if speed > speed_limit + 1.0e-9:
        return 0.0, "motor_speed"
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
    limits = {"tire": tire_force, "motor_torque": torque_force, "power": power_force}
    constraint = min(limits, key=limits.get)
    return limits[constraint], constraint


def _available_brake_force(
    vehicle: PointMassVehicleParameters,
    tire_limits: FloatArray,
) -> float:
    front_limit = float(np.sum(tire_limits[:2]))
    rear_limit = float(np.sum(tire_limits[2:]))
    bias = vehicle.brake_bias
    return min(front_limit / bias, rear_limit / (1.0 - bias))


def _solve_drive_acceleration(
    vehicle: PointMassVehicleParameters,
    speed: float,
    lateral_acceleration: float,
) -> float:
    _, drag = aerodynamic_forces(vehicle, speed)

    def residual(acceleration: float) -> float:
        state = _force_state(vehicle, speed, acceleration, lateral_acceleration)
        if not state[-1]:
            return -vehicle.total_mass * acceleration - drag
        drive_force, _ = _available_drive_force(vehicle, speed, state[4])
        return drive_force - drag - vehicle.total_mass * acceleration

    if residual(0.0) <= 0.0:
        return 0.0
    return _positive_root(residual, 30.0)


def _solve_braking_deceleration(
    vehicle: PointMassVehicleParameters,
    speed: float,
    lateral_acceleration: float,
) -> float:
    _, drag = aerodynamic_forces(vehicle, speed)

    def residual(deceleration: float) -> float:
        state = _force_state(vehicle, speed, -deceleration, lateral_acceleration)
        if not state[-1]:
            return drag - vehicle.total_mass * deceleration
        brake_force = _available_brake_force(vehicle, state[4])
        return brake_force + drag - vehicle.total_mass * deceleration

    return _positive_root(residual, 30.0)


def _positive_root(function: Callable[[float], float], upper: float) -> float:
    lower = 0.0
    if function(upper) > 0.0:
        return upper
    for _ in range(32):
        trial = 0.5 * (lower + upper)
        if function(trial) > 0.0:
            lower = trial
        else:
            upper = trial
    return lower
