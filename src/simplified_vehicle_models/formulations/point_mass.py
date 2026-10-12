from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
import math
from typing import TypeVar, cast

import numpy as np

from simplified_vehicle_models.compiler.adapters import ResolutionResult
from simplified_vehicle_models.compiler.capabilities import (
    AERO_LOADS,
    DRIVE_FORCE_LIMIT,
    TIRE_FORCE_ENVELOPE,
)
from simplified_vehicle_models.core.conventions import GRAVITY
from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.parameters.aero import AeroParameters
from simplified_vehicle_models.parameters.drivetrain import DrivetrainParameters
from simplified_vehicle_models.parameters.tire import TireEnvelopeParameters
from simplified_vehicle_models.parameters.vehicle import PointMassVehicleParameters
from simplified_vehicle_models.powertrain.assembly import (
    FullDrivetrainRuntime,
    maximum_drive_force,
)
from simplified_vehicle_models.powertrain.energy_storage import StorageState
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    require_closed_unit_interval,
    require_nonnegative,
    require_open_unit_interval,
    require_positive,
    required_parameters,
)


POINT_MASS_REQUESTED_CAPABILITIES: tuple[str, ...] = (
    TIRE_FORCE_ENVELOPE,
    AERO_LOADS,
    DRIVE_FORCE_LIMIT,
)

_CapabilityValue = TypeVar("_CapabilityValue")


@dataclass(frozen=True, slots=True)
class DriveCachePolicy:
    """Control solver-local drive-force cache key approximation."""

    speed_resolution: float = 0.0
    soc_resolution: float = 0.0

    def __post_init__(self) -> None:
        for name, value in (("speed_resolution", self.speed_resolution), ("soc_resolution", self.soc_resolution)):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and nonnegative")


class DriveCache:
    """Per-solve cache and approximation diagnostics for assembled drive limits."""

    def __init__(self, policy: DriveCachePolicy) -> None:
        self.policy = policy
        self.values: dict[tuple[int, int] | tuple[float, float], tuple[float, float, object]] = {}
        self.hits = 0
        self.max_speed_displacement = 0.0
        self.max_soc_displacement = 0.0

    def key(self, speed: float, soc: float) -> tuple[int, int] | tuple[float, float]:
        if self.policy.speed_resolution > 0.0:
            speed_key: int | float = round(speed / self.policy.speed_resolution)
        else:
            speed_key = speed
        if self.policy.soc_resolution > 0.0:
            soc_key: int | float = round(soc / self.policy.soc_resolution)
        else:
            soc_key = soc
        return speed_key, soc_key

    def get(self, speed: float, soc: float) -> object | None:
        item = self.values.get(self.key(speed, soc))
        if item is None:
            return None
        actual_speed, actual_soc, value = item
        self.hits += 1
        self.max_speed_displacement = max(self.max_speed_displacement, abs(speed - actual_speed))
        self.max_soc_displacement = max(self.max_soc_displacement, abs(soc - actual_soc))
        return value

    def put(self, speed: float, soc: float, value: object) -> None:
        self.values.setdefault(self.key(speed, soc), (speed, soc, value))

POINT_MASS_VEHICLE_PARAMETER_UNITS: dict[str, str] = {
    "total_mass": "kg",
    "wheelbase": "m",
    "track_width": "m",
    "cg_height": "m",
    "front_mass_fraction": "1",
    "lateral_load_transfer_distribution": "1",
    "brake_bias": "1",
}


def compile_point_mass_vehicle(
    assembly: Model,
    resolutions: Mapping[str, ResolutionResult],
) -> PointMassVehicleParameters:
    """Compile one fidelity-neutral assembly for the point-mass formulation.

    Consume the assembly parameters that the point-mass model needs.
    Resolve each requested capability through the capability resolutions.
    """

    values, _ = required_parameters(assembly, POINT_MASS_VEHICLE_PARAMETER_UNITS)
    require_positive(assembly, values, "total_mass", "wheelbase", "track_width")
    require_nonnegative(assembly, values, "cg_height")
    require_open_unit_interval(assembly, values, "front_mass_fraction", "brake_bias")
    require_closed_unit_interval(
        assembly,
        values,
        "lateral_load_transfer_distribution",
    )
    return PointMassVehicleParameters(
        total_mass=values["total_mass"],
        wheelbase=values["wheelbase"],
        track_width=values["track_width"],
        cg_height=values["cg_height"],
        front_mass_fraction=values["front_mass_fraction"],
        lateral_load_transfer_distribution=values[
            "lateral_load_transfer_distribution"
        ],
        brake_bias=values["brake_bias"],
        tire=_capability_value(
            assembly,
            resolutions,
            TIRE_FORCE_ENVELOPE,
            TireEnvelopeParameters,
        ),
        aero=_capability_value(
            assembly,
            resolutions,
            AERO_LOADS,
            AeroParameters,
        ),
        drivetrain=_capability_value(
            assembly,
            resolutions,
            DRIVE_FORCE_LIMIT,
            DrivetrainParameters,
        ),
    )


def _capability_value(
    assembly: Model,
    resolutions: Mapping[str, ResolutionResult],
    capability: str,
    expected_type: type[_CapabilityValue],
) -> _CapabilityValue:
    resolution = resolutions.get(capability)
    if resolution is None:
        raise ValueError(
            f"{assembly.path}: the compiler did not resolve capability {capability!r}"
        )
    if not isinstance(resolution.value, expected_type):
        raise ValueError(
            f"{assembly.path}: capability {capability!r} produced "
            f"{type(resolution.value).__name__}, expected "
            f"{expected_type.__name__}"
        )
    return cast(_CapabilityValue, resolution.value)


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
    storage_state: StorageState | None = None,
    drive_cache: DriveCache | None = None,
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
        storage_state,
        drive_cache,
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


def _equilibrium_limits(
    vehicle: PointMassVehicleParameters,
    speed: float,
    curvature: float,
    longitudinal_acceleration: float,
    storage_state: StorageState | None = None,
    drive_cache: DriveCache | None = None,
) -> tuple[bool, bool, float, float, float]:
    """Evaluate only scalar feasibility and force limits for root solves."""

    lateral_acceleration = speed * speed * abs(curvature)
    downforce, drag = aerodynamic_forces(vehicle, speed)
    mass = vehicle.total_mass
    front_axle = (
        mass * GRAVITY * vehicle.front_mass_fraction
        + downforce * vehicle.aero.aero_balance
        - mass * longitudinal_acceleration * vehicle.cg_height / vehicle.wheelbase
    )
    rear_axle = mass * GRAVITY + downforce - front_axle
    total_transfer = mass * abs(lateral_acceleration) * vehicle.cg_height / vehicle.track_width
    front_transfer = total_transfer * vehicle.lateral_load_transfer_distribution
    rear_transfer = total_transfer - front_transfer
    loads = (
        front_axle * 0.5 - front_transfer,
        front_axle * 0.5 + front_transfer,
        rear_axle * 0.5 - rear_transfer,
        rear_axle * 0.5 + rear_transfer,
    )
    contact_feasible = all(load >= -1.0e-9 for load in loads)
    tire = vehicle.tire
    capacities = tuple(
        (
            max(tire.longitudinal_mu_scale * load_factor, 0.0) * contact_load,
            max(tire.lateral_mu_scale * load_factor, 0.0) * contact_load,
        )
        for load in loads
        for contact_load in (max(load, 0.0),)
        for load_factor in (
            tire.load_sensitivity_slope * contact_load
            + tire.load_sensitivity_intercept,
        )
    )
    longitudinal_capacities = tuple(item[0] for item in capacities)
    lateral_capacities = tuple(item[1] for item in capacities)

    total_lateral_force = mass * lateral_acceleration
    front_target = total_lateral_force * vehicle.front_mass_fraction
    rear_target = total_lateral_force - front_target
    front_capacity = lateral_capacities[0] + lateral_capacities[1]
    rear_capacity = lateral_capacities[2] + lateral_capacities[3]
    front_ok = front_target <= 1.0e-12 or (
        front_capacity > 0.0 and front_target <= front_capacity + 1.0e-8
    )
    rear_ok = rear_target <= 1.0e-12 or (
        rear_capacity > 0.0 and rear_target <= rear_capacity + 1.0e-8
    )
    front_scale = front_target / front_capacity if front_target > 1.0e-12 and front_capacity > 0.0 else 0.0
    rear_scale = rear_target / rear_capacity if rear_target > 1.0e-12 and rear_capacity > 0.0 else 0.0
    lateral_forces = (
        lateral_capacities[0] * front_scale,
        lateral_capacities[1] * front_scale,
        lateral_capacities[2] * rear_scale,
        lateral_capacities[3] * rear_scale,
    )
    lateral_feasible = front_ok and rear_ok and contact_feasible
    remaining = tuple(
        longitudinal_capacity * math.sqrt(
            max(
                1.0
                - (lateral_force / lateral_capacity if lateral_capacity > 0.0 else 0.0) ** 2,
                0.0,
            )
        )
        for longitudinal_capacity, lateral_capacity, lateral_force in zip(
            longitudinal_capacities, lateral_capacities, lateral_forces
        )
    )
    drive_limit, _ = _available_drive_force(vehicle, speed, remaining, storage_state, drive_cache)
    brake_limit, _ = _available_brake_force(vehicle, remaining)
    return contact_feasible, lateral_feasible, drag, drive_limit, brake_limit


def _validate_limit_inputs(speed: float, curvature: float) -> float:
    if not math.isfinite(speed) or speed < 0.0:
        raise ValueError("speed must be finite and nonnegative")
    if not math.isfinite(curvature):
        raise ValueError("curvature must be finite")
    return abs(curvature)


def maximum_acceleration_limit(
    vehicle: PointMassVehicleParameters,
    speed: float,
    curvature: float,
    storage_state: StorageState | None = None,
    drive_cache: DriveCache | None = None,
    tolerance: float = 1.0e-8,
) -> float:
    """Solve only the positive longitudinal acceleration limit."""

    curvature = _validate_limit_inputs(speed, curvature)
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("tolerance must be finite and positive")
    if not _equilibrium_limits(vehicle, speed, curvature, 0.0, storage_state, drive_cache)[1]:
        return 0.0

    def residual(acceleration: float) -> float:
        contact, lateral, drag, drive, _ = _equilibrium_limits(
            vehicle, speed, curvature, acceleration, storage_state, drive_cache
        )
        required = vehicle.total_mass * acceleration + drag
        if not contact or not lateral:
            return -required
        return drive - required

    return _positive_root(residual, 30.0, tolerance=tolerance)


def maximum_deceleration_limit(
    vehicle: PointMassVehicleParameters,
    speed: float,
    curvature: float,
    storage_state: StorageState | None = None,
    drive_cache: DriveCache | None = None,
    tolerance: float = 1.0e-8,
) -> float:
    """Solve only the positive longitudinal deceleration limit."""

    curvature = _validate_limit_inputs(speed, curvature)
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("tolerance must be finite and positive")
    if not _equilibrium_limits(vehicle, speed, curvature, 0.0, storage_state, drive_cache)[1]:
        return 0.0

    def residual(deceleration: float) -> float:
        contact, lateral, drag, _, brake = _equilibrium_limits(
            vehicle, speed, curvature, -deceleration, storage_state, drive_cache
        )
        required = vehicle.total_mass * deceleration - drag
        if not contact or not lateral:
            return -max(required, 0.0)
        return brake - max(required, 0.0)

    return _positive_root(residual, 30.0, tolerance=tolerance)


def maximum_corner_speed(
    vehicle: PointMassVehicleParameters,
    radius: float,
    maximum_speed: float = 60.0,
    storage_state: StorageState | None = None,
    tolerance: float = 1.0e-8,
    hint: float | None = None,
    drive_cache: DriveCache | None = None,
) -> float:
    """Solve the maximum steady speed for one unsigned radius."""

    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError("radius must be finite and positive")
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("tolerance must be finite and positive")
    upper = min(maximum_speed, maximum_vehicle_speed(vehicle))
    lower = 0.0
    if evaluate_equilibrium(vehicle, upper, 1.0 / radius, 0.0, storage_state, drive_cache).is_feasible:
        return upper
    if hint is not None and math.isfinite(hint) and 0.0 < hint < upper:
        if evaluate_equilibrium(vehicle, hint, 1.0 / radius, 0.0, storage_state, drive_cache).is_feasible:
            lower = hint
        else:
            upper = hint
    while upper - lower > tolerance:
        trial = 0.5 * (lower + upper)
        state = evaluate_equilibrium(vehicle, trial, 1.0 / radius, 0.0, storage_state, drive_cache)
        if state.is_feasible:
            lower = trial
        else:
            upper = trial
    return lower


def acceleration_limits(
    vehicle: PointMassVehicleParameters,
    speed: float,
    curvature: float,
    storage_state: StorageState | None = None,
) -> AccelerationLimits:
    """Solve live acceleration limits and their equilibrium states."""

    curvature = _validate_limit_inputs(speed, curvature)
    coasting = evaluate_equilibrium(vehicle, speed, curvature, 0.0, storage_state)
    maximum_acceleration = maximum_acceleration_limit(
        vehicle, speed, curvature, storage_state
    )
    maximum_deceleration = maximum_deceleration_limit(
        vehicle, speed, curvature, storage_state
    )
    acceleration_state = evaluate_equilibrium(
        vehicle,
        speed,
        curvature,
        maximum_acceleration,
        storage_state,
    )
    braking_state = evaluate_equilibrium(
        vehicle,
        speed,
        curvature,
        -maximum_deceleration,
        storage_state,
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
    storage_state: StorageState | None = None,
    drive_cache: DriveCache | None = None,
) -> tuple[float, str]:
    if speed > maximum_vehicle_speed(vehicle) + 1.0e-9:
        return 0.0, "motor_speed"
    left_limit = float(tire_limits[0] + tire_limits[2])
    right_limit = float(tire_limits[1] + tire_limits[3])
    tire_force = 2.0 * min(left_limit, right_limit)
    drivetrain = vehicle.drivetrain
    if isinstance(drivetrain, FullDrivetrainRuntime):
        # The assembled capability owns the motor, storage, and transmission
        # coupling.  Point mass supplies only the operating speed and tire cap;
        # it does not reproduce the powertrain equations here.
        state = storage_state or StorageState(drivetrain.powertrain.energy_storage.initial_state_of_charge)
        if drive_cache is not None:
            drive = drive_cache.get(speed, state.state_of_charge)
        else:
            drive = None
        if drive is None:
            drive = maximum_drive_force(
                drivetrain.powertrain,
                drivetrain.drivetrain,
                speed,
                state,
                effective_radius=vehicle.tire.effective_radius,
            )
            if drive_cache is not None:
                drive_cache.put(speed, state.state_of_charge, drive)
        limits = {"tire": tire_force, drive.active_constraint: drive.maximum_drive_force}
        constraint = min(limits, key=limits.get)
        return limits[constraint], constraint

    # Legacy lumped drivetrain reduction.  Keep this path byte-for-byte in
    # behavior so existing MIS golden values remain a migration guard.
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
    limits = {
        "tire": tire_force,
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


def _positive_root(
    function: Callable[[float], float],
    upper: float,
    tolerance: float = 1.0e-8,
    hint: float | None = None,
) -> float:
    if not math.isfinite(upper) or upper < 0.0:
        raise ValueError("upper must be finite and nonnegative")
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("tolerance must be finite and positive")
    lower = 0.0
    if function(lower) <= 0.0:
        return 0.0
    if function(upper) > 0.0:
        return upper
    if hint is not None and math.isfinite(hint) and 0.0 < hint < upper:
        if function(hint) > 0.0:
            lower = hint
        else:
            upper = hint
    while upper - lower > tolerance:
        trial = 0.5 * (lower + upper)
        if function(trial) > 0.0:
            lower = trial
        else:
            upper = trial
    return lower
