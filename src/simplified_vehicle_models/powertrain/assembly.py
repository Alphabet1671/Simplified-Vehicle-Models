"""Compose the electric powertrain and drivetrain assemblies.

The assembly keeps storage, motor, and transmission models separate.  The
runtime reduction exposes the drive-force limit used by low-fidelity models.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

from simplified_vehicle_models.compiler.adapters import register_adapter
from simplified_vehicle_models.compiler.capabilities import DRIVE_FORCE_LIMIT
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.drivetrain.transmission import FixedGearParameters
from simplified_vehicle_models.parameters.drivetrain import DrivetrainParameters
from simplified_vehicle_models.xml.loader import load_model
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.powertrain.energy_storage import (
    EnergyStorageParameters,
    StorageLimits,
    StorageState,
    evaluate_storage,
)
from simplified_vehicle_models.powertrain.power_plant import (
    PowerPlantParameters,
    evaluate_shaft_power,
    maximum_motor_torque,
)

POWERTRAIN_ASSEMBLY_TYPE = "powertrain-assembly"
POWERTRAIN_SUBTYPE = "electric"
DRIVETRAIN_ASSEMBLY_TYPE = "drivetrain-assembly"
DRIVETRAIN_SUBTYPE = "electric-four-wheel"
ASSEMBLY_SOURCE_CAPABILITY = "powertrain-and-drivetrain"


@dataclass(frozen=True, slots=True)
class PowertrainParameters:
    power_plant: PowerPlantParameters
    energy_storage: EnergyStorageParameters


@dataclass(frozen=True, slots=True)
class DrivetrainAssemblyParameters:
    transmission: FixedGearParameters
    differential_declared: bool = False


@dataclass(frozen=True, slots=True)
class RegenLimit:
    force_limit: float
    force_from_torque: float
    force_from_charge_power: float
    charge_power_limit: float
    crossover_speed: float
    active_constraint: str
    residual: float = 0.0
    converged: bool = True
    iterations: int = 0


@dataclass(frozen=True, slots=True)
class DriveLimit:
    maximum_drive_force: float
    maximum_wheel_torque: float
    bus_voltage: float
    bus_power_at_limit: float
    active_constraint: str
    storage_state_rate: float
    shaft_torque_residual: float = 0.0
    converged: bool = True
    iterations: int = 0


@dataclass(frozen=True, slots=True)
class FixedPointResult:
    """Report an implicit torque update without hiding convergence."""

    torque: float
    residual: float
    converged: bool
    iterations: int


def _references(model: Model, tag: str) -> dict[str, Model]:
    result = {reference.role: reference.model for reference in model.references if reference.tag == tag}
    if any(reference.role is None for reference in model.references if reference.tag == tag):
        raise ValueError(f"{model.path}: every {tag} reference needs a role")
    return result


def compile_powertrain_assembly(model: Model) -> PowertrainParameters:
    """Compile a power plant and an energy buffer from one assembly."""
    if (model.model_type, model.subtype) != (POWERTRAIN_ASSEMBLY_TYPE, POWERTRAIN_SUBTYPE):
        raise ValueError(f"{model.path}: expected powertrain-assembly/electric")
    references = _references(model, "component")
    if set(references) != {"power-plant", "energy-storage"}:
        raise ValueError(f"{model.path}: powertrain needs power-plant and energy-storage")
    from simplified_vehicle_models.powertrain.energy_storage import compile_battery_pack
    from simplified_vehicle_models.powertrain.power_plant import compile_electric_motor
    return PowertrainParameters(
        compile_electric_motor(load_model(references["power-plant"].path)),
        compile_battery_pack(load_model(references["energy-storage"].path)),
    )


def compile_drivetrain_assembly(model: Model) -> DrivetrainAssemblyParameters:
    """Compile the transmission in one drivetrain assembly."""
    if (model.model_type, model.subtype) != (DRIVETRAIN_ASSEMBLY_TYPE, DRIVETRAIN_SUBTYPE):
        raise ValueError(f"{model.path}: expected drivetrain-assembly/electric-four-wheel")
    references = _references(model, "component")
    if "transmission" not in references or set(references) - {"transmission", "differential"}:
        raise ValueError(f"{model.path}: drivetrain needs transmission and optional differential")
    from simplified_vehicle_models.drivetrain.transmission import compile_fixed_gear_transmission
    return DrivetrainAssemblyParameters(
        compile_fixed_gear_transmission(load_model(references["transmission"].path)),
        differential_declared="differential" in references,
    )


def compile_powertrain_drivetrain(model: Model) -> DrivetrainParameters:
    """Compile nested assemblies to the legacy drive-force contract.

    The point-mass formulation still consumes the compact contract.  The
    assembled values are derived from the reviewed motor and transmission, not
    from a hard-coded adapter chain.
    """
    nested = {role: reference for role, reference in _references(model, "component").items() if role in {"powertrain", "drivetrain"}}
    if set(nested) != {"powertrain", "drivetrain"}:
        raise ValueError(f"{model.path}: vehicle needs powertrain and drivetrain assemblies")
    powertrain = compile_powertrain_assembly(load_model(nested["powertrain"].path))
    drivetrain = compile_drivetrain_assembly(load_model(nested["drivetrain"].path))
    motor = powertrain.power_plant
    transmission = drivetrain.transmission
    if (
        motor.unit_count != len(transmission.driven_wheels)
        and not drivetrain.differential_declared
    ):
        raise ValueError(
            f"{model.path}: motor count {motor.unit_count} does not match driven wheels "
            f"{len(transmission.driven_wheels)} without a declared differential"
        )
    state = StorageState(powertrain.energy_storage.initial_state_of_charge)
    limits = evaluate_storage(powertrain.energy_storage, state)
    torque = maximum_motor_torque(motor, 1.0, limits.bus_voltage / math.sqrt(3.0))
    return DrivetrainParameters(
        motor_count=motor.unit_count,
        gear_ratio=transmission.gear_ratio,
        peak_motor_torque=torque,
        power_limit=motor.unit_count * motor.unit_power_limit,
        efficiency=transmission.gear_efficiency,
        maximum_motor_speed=motor.maximum_motor_speed,
    )


def solve_shaft_torque_fixed_point(
    motor: PowerPlantParameters,
    motor_speed: float,
    bus_power: float,
    bus_voltage: float,
    unit_count: int = 1,
    tolerance: float = 1.0e-9,
    max_iterations: int = 100,
) -> FixedPointResult:
    """Solve the implicit shaft-torque/loss equation.

    ``bus_power`` is the total inverter-side power.  The update evaluates the
    motor loss at the current torque, then inverts the directional inverter
    efficiency to obtain the next shaft torque.  The iteration limit is an
    explicit input and a result with ``converged=False`` is returned when it is
    reached.
    """
    if motor_speed < 0.0 or unit_count < 1 or tolerance <= 0.0 or max_iterations < 1:
        raise ValueError("motor speed, unit count, tolerance, and max_iterations are invalid")
    omega = motor_speed * 2.0 * math.pi / 60.0
    if omega == 0.0:
        residual = abs(bus_power)
        return FixedPointResult(0.0, residual, residual <= tolerance, 0)
    torque = bus_power / max(unit_count, 1) / omega
    residual = math.inf
    for iteration in range(1, max_iterations + 1):
        result = evaluate_shaft_power(motor, motor_speed, torque, bus_voltage)
        if bus_power >= 0.0:
            target_shaft_power = bus_power / unit_count * motor.inverter_efficiency - result.loss_power
        else:
            target_shaft_power = bus_power / unit_count / motor.inverter_efficiency - result.loss_power
        target = target_shaft_power / omega
        residual = abs(target - torque)
        if residual <= tolerance:
            return FixedPointResult(target, residual, True, iteration)
        # Loss maps can be stiff near the charge-power limit. Under-relax the
        # update while retaining the residual of the actual fixed-point map.
        # The factor is explicit so iteration behavior is reproducible.
        relaxation = 0.25
        torque = (1.0 - relaxation) * torque + relaxation * target
    return FixedPointResult(torque, residual, False, max_iterations)


def maximum_drive_force(powertrain: PowertrainParameters, drivetrain: DrivetrainAssemblyParameters, speed: float, state: StorageState, effective_radius: float = 1.0) -> DriveLimit:
    """Return the motoring force and its active constraint."""
    if speed < 0.0 or effective_radius <= 0.0:
        raise ValueError("speed must be nonnegative and radius must be positive")
    storage = evaluate_storage(powertrain.energy_storage, state)
    motor = powertrain.power_plant
    transmission = drivetrain.transmission
    motor_speed = transmission.motor_speed(speed / effective_radius)
    if motor_speed > motor.maximum_motor_speed:
        return DriveLimit(0.0, 0.0, storage.bus_voltage, 0.0, "motor_speed", 0.0)
    envelope = maximum_motor_torque(motor, motor_speed, storage.bus_voltage / math.sqrt(3.0))
    torque = envelope * motor.unit_count
    wheel_torque = torque * transmission.gear_ratio * transmission.gear_efficiency
    force = wheel_torque / effective_radius
    power_force = math.inf if speed == 0.0 else storage.discharge_power_limit * transmission.gear_efficiency * motor.inverter_efficiency / speed
    if power_force < force:
        force = power_force
        active = "storage_power"
    else:
        active = "motor_torque"
    shaft = force * effective_radius / (transmission.gear_ratio * transmission.gear_efficiency)
    fixed_point = solve_shaft_torque_fixed_point(
        motor, motor_speed, storage.discharge_power_limit, storage.bus_voltage,
        unit_count=motor.unit_count,
    )
    bus = evaluate_shaft_power(
        motor, motor_speed, shaft / motor.unit_count, storage.bus_voltage
    ).bus_power * motor.unit_count
    return DriveLimit(
        force, force * effective_radius, storage.bus_voltage, bus, active, 0.0,
        fixed_point.residual, fixed_point.converged, fixed_point.iterations,
    )


def maximum_regen_force(
    powertrain: PowertrainParameters,
    drivetrain: DrivetrainAssemblyParameters,
    speed: float,
    state: StorageState,
    effective_radius: float = 1.0,
    tolerance: float = 1.0e-9,
    max_iterations: int = 100,
) -> RegenLimit:
    """Resolve torque and charge-power regeneration limits.

    The charge-power branch solves the motor-loss fixed point.  Its residual,
    iteration count, and convergence status are part of the returned report.
    """
    if speed < 0.0 or effective_radius <= 0.0:
        raise ValueError("speed must be nonnegative and radius must be positive")
    storage = evaluate_storage(powertrain.energy_storage, state)
    motor = powertrain.power_plant
    transmission = drivetrain.transmission
    motor_speed = transmission.motor_speed(speed / effective_radius)
    if motor_speed > motor.maximum_motor_speed:
        return RegenLimit(
            0.0, 0.0, 0.0, storage.charge_power_limit, math.inf,
            "motor_speed", 0.0, True, 0,
        )
    envelope = maximum_motor_torque(motor, motor_speed, storage.bus_voltage / math.sqrt(3.0))
    torque_force = envelope * motor.unit_count * transmission.gear_ratio * transmission.gear_efficiency / effective_radius
    power_force = math.inf if speed == 0.0 else abs(storage.charge_power_limit) * motor.inverter_efficiency * transmission.gear_efficiency / speed
    limit = min(torque_force, power_force)
    active = "motor_torque" if torque_force <= power_force else "storage_power"
    crossover = math.inf if torque_force == 0.0 else abs(storage.charge_power_limit) * motor.inverter_efficiency * transmission.gear_efficiency / torque_force
    if speed == 0.0:
        return RegenLimit(limit, torque_force, power_force, storage.charge_power_limit, crossover, active, 0.0, True, 0)
    fixed_point = solve_shaft_torque_fixed_point(
        motor, motor_speed, -abs(storage.charge_power_limit), storage.bus_voltage,
        unit_count=motor.unit_count, tolerance=tolerance, max_iterations=max_iterations,
    )
    loss_limited_force = abs(fixed_point.torque) * motor.unit_count * transmission.gear_ratio * transmission.gear_efficiency / effective_radius
    if active == "storage_power":
        limit = min(limit, loss_limited_force)
    return RegenLimit(
        limit, torque_force, power_force, storage.charge_power_limit, crossover, active,
        fixed_point.residual, fixed_point.converged, fixed_point.iterations,
    )


register_component(POWERTRAIN_ASSEMBLY_TYPE, POWERTRAIN_SUBTYPE, kind="assembly")
register_component(DRIVETRAIN_ASSEMBLY_TYPE, DRIVETRAIN_SUBTYPE, kind="assembly")


def reduce_powertrain_and_drivetrain(value: object) -> object:
    """Return the assembled drive-force provider."""
    return value


register_adapter(ASSEMBLY_SOURCE_CAPABILITY, DRIVE_FORCE_LIMIT, reduce_powertrain_and_drivetrain)
