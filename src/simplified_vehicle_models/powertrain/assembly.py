"""Compose the electric powertrain and drivetrain assemblies.

The assembly keeps storage, motor, and transmission models separate.  The
runtime reduction exposes the drive-force limit used by low-fidelity models.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
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
    solve_current,
    terminal_voltage,
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


@dataclass(frozen=True, slots=True, eq=False)
class FullDrivetrainRuntime(DrivetrainParameters):
    """Keep nested powertrain components available at runtime."""

    powertrain: PowertrainParameters
    drivetrain: DrivetrainAssemblyParameters



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
    storage_current: float = 0.0
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
    result = {
        reference.role: reference.model
        for reference in model.references
        if reference.tag == tag
    }
    if any(
        reference.role is None
        for reference in model.references
        if reference.tag == tag
    ):
        raise ValueError(f"{model.path}: every {tag} reference needs a role")
    return result


def compile_powertrain_assembly(model: Model) -> PowertrainParameters:
    """Compile a power plant and an energy buffer from one assembly."""
    if (model.model_type, model.subtype) != (
        POWERTRAIN_ASSEMBLY_TYPE,
        POWERTRAIN_SUBTYPE,
    ):
        raise ValueError(f"{model.path}: expected powertrain-assembly/electric")
    references = _references(model, "component")
    if set(references) != {"power-plant", "energy-storage"}:
        raise ValueError(
            f"{model.path}: powertrain needs power-plant and energy-storage"
        )
    from simplified_vehicle_models.powertrain.energy_storage import compile_battery_pack
    from simplified_vehicle_models.powertrain.power_plant import compile_electric_motor
    return PowertrainParameters(
        compile_electric_motor(load_model(references["power-plant"].path)),
        compile_battery_pack(load_model(references["energy-storage"].path)),
    )


def compile_drivetrain_assembly(model: Model) -> DrivetrainAssemblyParameters:
    """Compile the transmission in one drivetrain assembly."""
    if (model.model_type, model.subtype) != (
        DRIVETRAIN_ASSEMBLY_TYPE,
        DRIVETRAIN_SUBTYPE,
    ):
        raise ValueError(
            f"{model.path}: expected drivetrain-assembly/electric-four-wheel"
        )
    references = _references(model, "component")
    if "transmission" not in references or set(references) - {
        "transmission",
        "differential",
    }:
        raise ValueError(
            f"{model.path}: drivetrain needs transmission and optional differential"
        )
    from simplified_vehicle_models.drivetrain.transmission import (
        compile_fixed_gear_transmission,
    )
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
    nested = {
        role: reference
        for role, reference in _references(model, "component").items()
        if role in {"powertrain", "drivetrain"}
    }
    if set(nested) != {"powertrain", "drivetrain"}:
        raise ValueError(
            f"{model.path}: vehicle needs powertrain and drivetrain assemblies"
        )
    powertrain = compile_powertrain_assembly(load_model(nested["powertrain"].path))
    drivetrain = compile_drivetrain_assembly(load_model(nested["drivetrain"].path))
    motor = powertrain.power_plant
    transmission = drivetrain.transmission
    if (
        motor.unit_count != len(transmission.driven_wheels)
        and not drivetrain.differential_declared
    ):
        raise ValueError(
            f"{model.path}: motor count {motor.unit_count} does not match "
            f"driven wheels {len(transmission.driven_wheels)} without a "
            "declared differential"
        )
    state = StorageState(powertrain.energy_storage.initial_state_of_charge)
    limits = evaluate_storage(powertrain.energy_storage, state)
    torque = maximum_motor_torque(motor, 1.0, limits.bus_voltage / math.sqrt(3.0))
    return FullDrivetrainRuntime(
        motor_count=motor.unit_count,
        gear_ratio=transmission.gear_ratio,
        peak_motor_torque=torque,
        power_limit=motor.unit_count * motor.unit_power_limit,
        efficiency=transmission.gear_efficiency,
        maximum_motor_speed=motor.maximum_motor_speed,
        powertrain=powertrain,
        drivetrain=drivetrain,
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
        raise ValueError(
            "motor speed, unit count, tolerance, and max_iterations are invalid"
        )
    omega = motor_speed * 2.0 * math.pi / 60.0
    if omega == 0.0:
        residual = abs(bus_power)
        return FixedPointResult(0.0, residual, residual <= tolerance, 0)
    torque = bus_power / max(unit_count, 1) / omega
    residual = math.inf
    def map_target(current_torque: float) -> float:
        result = evaluate_shaft_power(motor, motor_speed, current_torque, bus_voltage)
        if bus_power >= 0.0:
            target_shaft_power = (
                bus_power / unit_count * motor.inverter_efficiency
                - result.loss_power
            )
        else:
            target_shaft_power = (
                bus_power / unit_count / motor.inverter_efficiency
                - result.loss_power
            )
        return target_shaft_power / omega

    for iteration in range(1, max_iterations + 1):
        target = map_target(torque)
        residual = abs(target - torque)
        if residual <= tolerance:
            return FixedPointResult(target, residual, True, iteration)
        # Loss maps can be stiff near the charge-power limit. Under-relax the
        # update while retaining the residual of the actual fixed-point map.
        relaxation = 0.25
        torque = (1.0 - relaxation) * torque + relaxation * target
    # Report the residual at the torque that is actually returned, not at the
    # pre-relaxation iterate used to produce it.
    residual = abs(map_target(torque) - torque)
    return FixedPointResult(torque, residual, False, max_iterations)


def maximum_drive_force(
    powertrain: PowertrainParameters,
    drivetrain: DrivetrainAssemblyParameters,
    speed: float,
    state: StorageState,
    effective_radius: float = 1.0,
) -> DriveLimit:
    """Solve the coupled motor, inverter, and storage drive limit."""
    if speed < 0.0 or effective_radius <= 0.0:
        raise ValueError("speed must be nonnegative and radius must be positive")
    storage = evaluate_storage(powertrain.energy_storage, state)
    motor = powertrain.power_plant
    transmission = drivetrain.transmission
    motor_speed = transmission.motor_speed(speed / effective_radius)
    if motor_speed > motor.maximum_motor_speed:
        return DriveLimit(
            0.0, 0.0, storage.bus_voltage, 0.0, "motor_speed", 0.0,
            0.0, True, 0, 0.0,
        )
    voltage = storage.bus_voltage
    torque = 0.0
    operating = evaluate_shaft_power(motor, motor_speed, 0.0, voltage)
    current = 0.0
    converged = False
    iterations = 0
    residual = math.inf
    tolerance = 1.0e-9
    for iterations in range(1, 51):
        envelope = maximum_motor_torque(
            motor, motor_speed, voltage / math.sqrt(3.0)
        )

        def evaluate(candidate: float):
            return evaluate_shaft_power(motor, motor_speed, candidate, voltage)

        lower = 0.0
        upper = envelope
        lower_result = evaluate(lower)
        upper_result = evaluate(upper)
        if lower_result.feasible and lower_result.bus_power >= 0.0:
            if upper_result.feasible and upper_result.bus_power <= storage.discharge_power_limit:
                torque = upper
                operating = upper_result
            else:
                for _ in range(60):
                    midpoint = 0.5 * (lower + upper)
                    midpoint_result = evaluate(midpoint)
                    if midpoint_result.feasible and 0.0 <= midpoint_result.bus_power <= storage.discharge_power_limit:
                        lower = midpoint
                        lower_result = midpoint_result
                    else:
                        upper = midpoint
                torque = lower
                operating = lower_result
        else:
            torque = 0.0
            operating = lower_result
        bus = max(0.0, operating.bus_power * motor.unit_count)
        current, feasible = solve_current(powertrain.energy_storage, state, bus)
        new_voltage = terminal_voltage(powertrain.energy_storage, state, current)
        residual = abs(new_voltage - voltage)
        voltage = 0.5 * (voltage + new_voltage)
        if residual <= tolerance:
            converged = feasible and operating.feasible
            break
    bus = max(0.0, operating.bus_power * motor.unit_count)
    wheel_torque = (
        torque
        * motor.unit_count
        * transmission.gear_ratio
        * transmission.gear_efficiency
    )
    force = wheel_torque / effective_radius
    active = "motor_torque" if torque >= envelope - tolerance else "storage_power"
    if not feasible:
        active = "storage_power"
    state_rate = -current / powertrain.energy_storage.pack_capacity_joules
    return DriveLimit(
        force,
        force * effective_radius,
        voltage,
        bus,
        active,
        state_rate,
        current,
        residual,
        converged and feasible,
        iterations,
    )


def maximum_drive_force_lumped(
    powertrain: PowertrainParameters,
    lumped: DrivetrainParameters,
    speed: float,
    state: StorageState,
    effective_radius: float = 1.0,
) -> DriveLimit:
    """Evaluate the independent legacy scalar drivetrain reduction."""
    if speed < 0.0 or effective_radius <= 0.0:
        raise ValueError("speed must be nonnegative and radius must be positive")
    storage = evaluate_storage(powertrain.energy_storage, state)
    motor_speed = lumped.gear_ratio * speed / effective_radius
    if motor_speed > lumped.maximum_motor_speed:
        return DriveLimit(
            0.0, 0.0, storage.bus_voltage, 0.0, "motor_speed", 0.0
        )
    torque_force = (
        lumped.motor_count
        * lumped.peak_motor_torque
        * lumped.gear_ratio
        * lumped.efficiency
        / effective_radius
    )
    power_force = (
        math.inf
        if speed == 0.0
        else lumped.power_limit * powertrain.power_plant.inverter_efficiency
        * lumped.efficiency / speed
    )
    force = min(torque_force, power_force)
    active = "motor_torque" if torque_force <= power_force else "storage_power"
    bus = 0.0 if speed == 0.0 else force * speed / (
        lumped.efficiency * powertrain.power_plant.inverter_efficiency
    )
    bus = min(bus, storage.discharge_power_limit)
    current, feasible = solve_current(powertrain.energy_storage, state, bus)
    return DriveLimit(
        force,
        force * effective_radius,
        terminal_voltage(powertrain.energy_storage, state, current),
        bus,
        active,
        -current / powertrain.energy_storage.pack_capacity_joules,
        current,
        0.0,
        feasible,
        0,
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
    envelope = maximum_motor_torque(
        motor, motor_speed, storage.bus_voltage / math.sqrt(3.0)
    )
    # Regeneration is directional: wheel power is reduced by the gear and
    # inverter losses before it reaches the battery.  The wheel force allowed
    # by a battery charge-power limit therefore uses reciprocal efficiencies.
    regen_gear_factor = 1.0 / transmission.gear_efficiency
    regen_inverter_factor = 1.0 / motor.inverter_efficiency
    torque_force = (
        envelope * motor.unit_count * transmission.gear_ratio
        * regen_gear_factor / effective_radius
    )
    power_force = (
        math.inf if speed == 0.0 else abs(storage.charge_power_limit)
        * regen_gear_factor * regen_inverter_factor / speed
    )
    limit = min(torque_force, power_force)
    active = "motor_torque" if torque_force <= power_force else "storage_power"
    crossover = (
        math.inf if torque_force == 0.0 else abs(storage.charge_power_limit)
        * regen_gear_factor * regen_inverter_factor / torque_force
    )
    if speed == 0.0:
        return RegenLimit(
            limit,
            torque_force,
            power_force,
            storage.charge_power_limit,
            crossover,
            active,
            0.0,
            True,
            0,
        )
    fixed_point = solve_shaft_torque_fixed_point(
        motor, motor_speed, -abs(storage.charge_power_limit), storage.bus_voltage,
        unit_count=motor.unit_count,
        tolerance=tolerance,
        max_iterations=max_iterations,
    )
    loss_limited_force = (
        abs(fixed_point.torque) * motor.unit_count * transmission.gear_ratio
        * regen_gear_factor / effective_radius
    )
    if active == "storage_power":
        limit = min(limit, loss_limited_force)
    return RegenLimit(
        limit, torque_force, power_force, storage.charge_power_limit, crossover, active,
        fixed_point.residual, fixed_point.converged, fixed_point.iterations,
    )


@dataclass(frozen=True, slots=True)
class BrakingResult:
    """Report regenerative and friction braking allocation."""

    regen_force_used: float
    friction_force_used: float
    total_force_limit: float
    recovered_bus_power: float
    active_constraint: str
    regen_limit: RegenLimit

    # Legacy aliases retained for callers of the initial P7 contract.
    @property
    def regen_force(self) -> float:
        return self.regen_force_used

    @property
    def friction_force(self) -> float:
        return self.friction_force_used

    @property
    def total_force(self) -> float:
        return self.total_force_limit

    @property
    def recovered_power(self) -> float:
        return self.recovered_bus_power

    @property
    def force_limit(self) -> float:
        """Return the available regenerative force."""
        return self.regen_limit.force_limit

    @property
    def force_from_charge_power(self) -> float:
        """Return the charge-power-limited regenerative force."""
        return self.regen_limit.force_from_charge_power


def resolve_braking(
    powertrain: PowertrainParameters,
    drivetrain: DrivetrainAssemblyParameters,
    speed: float,
    state: StorageState,
    braking_demand: float = math.inf,
    tire_force_capacity: float = math.inf,
    effective_radius: float = 1.0,
) -> BrakingResult:
    """Allocate a braking demand between regen and friction brakes."""
    if braking_demand < 0.0 or tire_force_capacity < 0.0:
        raise ValueError("braking demand and tire capacity must be nonnegative")
    regen_limit = maximum_regen_force(
        powertrain, drivetrain, speed, state, effective_radius=effective_radius
    )
    demand = min(braking_demand, tire_force_capacity)
    if not math.isfinite(demand):
        demand = regen_limit.force_limit
    regen_force = min(demand, regen_limit.force_limit)
    friction_force = demand - regen_force
    active = regen_limit.active_constraint
    motor = powertrain.power_plant
    transmission = drivetrain.transmission
    if speed == 0.0 or regen_limit.active_constraint == "motor_speed":
        recovered_bus_power = 0.0
    else:
        motor_speed = transmission.motor_speed(speed / effective_radius)
        motor_torque = (
            regen_force * effective_radius
            / (motor.unit_count * transmission.gear_ratio)
            * transmission.gear_efficiency
        )
        evaluated = evaluate_shaft_power(
            motor,
            motor_speed,
            -abs(motor_torque),
            evaluate_storage(powertrain.energy_storage, state).bus_voltage,
        )
        recovered_bus_power = abs(evaluated.bus_power) * motor.unit_count
    return BrakingResult(
        regen_force_used=regen_force,
        friction_force_used=friction_force,
        total_force_limit=demand,
        recovered_bus_power=recovered_bus_power,
        active_constraint=active,
        regen_limit=regen_limit,
    )


register_component(POWERTRAIN_ASSEMBLY_TYPE, POWERTRAIN_SUBTYPE, kind="assembly")
register_component(DRIVETRAIN_ASSEMBLY_TYPE, DRIVETRAIN_SUBTYPE, kind="assembly")


def reduce_powertrain_and_drivetrain(value: object) -> object:
    """Return the assembled drive-force provider."""
    return value


register_adapter(
    ASSEMBLY_SOURCE_CAPABILITY,
    DRIVE_FORCE_LIMIT,
    reduce_powertrain_and_drivetrain,
)
