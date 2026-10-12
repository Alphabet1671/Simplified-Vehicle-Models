"""Model a battery pack from exported cell curves."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from simplified_vehicle_models.compiler.capabilities import ENERGY_BUFFER
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import closed_parameters, require_no_references

_DATA = Path(__file__).resolve().parents[3] / "models" / "data" / "cosmx_cell_curves.csv"


@dataclass(frozen=True, slots=True)
class StorageState:
    """Store the battery state of charge."""

    state_of_charge: float


@dataclass(frozen=True, slots=True)
class StorageLimits:
    """Report battery terminal limits in SI units."""

    bus_voltage: float
    discharge_power_limit: float
    charge_power_limit: float
    usable_energy: float


@dataclass(frozen=True, slots=True)
class EnergyStorageParameters:
    """Store battery parameters and monotone interpolation axes."""

    series_count: int
    parallel_count: int
    discharge_current_limit: float
    charge_current_limit: float
    cell_capacity: float
    minimum_cell_voltage: float
    maximum_cell_voltage: float
    initial_state_of_charge: float
    state_of_charge_axis: tuple[float, ...]
    open_circuit_voltage_axis: tuple[float, ...]
    resistance_axis: tuple[float, ...]
    energy_axis: tuple[float, ...]

    @property
    def pack_capacity_ah(self) -> float:
        """Return nominal pack capacity in ampere-hours."""

        return self.parallel_count * self.cell_capacity

    @property
    def pack_capacity_joules(self) -> float:
        """Return nominal pack capacity in ampere-seconds (joules)."""

        return self.pack_capacity_ah * 3600.0

    @property
    def pack_capacity(self) -> float:
        """Return capacity in ampere-seconds for legacy callers."""

        return self.pack_capacity_joules


def _curve(path: Path = _DATA):
    rows = []
    with path.open(newline="") as stream:
        for row in csv.reader(stream):
            if not row or row[0].startswith("#") or row[0] == "state_of_charge":
                continue
            rows.append(tuple(float(value) for value in row))
    raw = np.asarray(rows, dtype=float)[::-1]
    return tuple(tuple(raw[:, index]) for index in range(4))


def _interp(axis, values, state_of_charge):
    return float(np.interp(state_of_charge, axis, values))


def _validate_state(state):
    state_of_charge = (
        state.state_of_charge
        if isinstance(state, StorageState)
        else float(state)
    )
    if not 0.0 <= state_of_charge <= 1.0:
        raise ValueError("state_of_charge must be from zero through one")
    return state_of_charge


def _pack_values(parameters, state_of_charge):
    voltage = parameters.series_count * _interp(
        parameters.state_of_charge_axis,
        parameters.open_circuit_voltage_axis,
        state_of_charge,
    )
    resistance = parameters.series_count * _interp(
        parameters.state_of_charge_axis,
        parameters.resistance_axis,
        state_of_charge,
    ) / parameters.parallel_count
    if resistance <= 0.0:
        raise ValueError("pack resistance must be positive")
    return voltage, resistance


def evaluate_storage(parameters: EnergyStorageParameters, state: StorageState):
    """Return terminal voltage, power limits, and usable energy."""

    state_of_charge = _validate_state(state)
    voltage, resistance = _pack_values(parameters, state_of_charge)
    discharge = (
        voltage * parameters.discharge_current_limit
        - parameters.discharge_current_limit**2 * resistance
    )
    charge = -(
        voltage * parameters.charge_current_limit
        + parameters.charge_current_limit**2 * resistance
    )
    energy = (
        parameters.energy_axis[-1] - parameters.energy_axis[0]
    ) * 3600.0 * parameters.parallel_count
    return StorageLimits(voltage, discharge, charge, energy)


def terminal_voltage(
    parameters: EnergyStorageParameters,
    state: StorageState,
    current: float,
) -> float:
    """Return terminal voltage after internal resistance drop."""
    voltage, resistance = _pack_values(parameters, _validate_state(state))
    return voltage - resistance * current


def solve_current(
    parameters: EnergyStorageParameters,
    state: StorageState,
    terminal_power: float,
) -> tuple[float, bool]:
    """Solve terminal power for current and report discriminant feasibility."""

    state_of_charge = _validate_state(state)
    open_circuit_voltage, resistance = _pack_values(parameters, state_of_charge)
    discharge_limit = (
        open_circuit_voltage * parameters.discharge_current_limit
        - parameters.discharge_current_limit**2 * resistance
    )
    charge_limit = -(
        open_circuit_voltage * parameters.charge_current_limit
        + parameters.charge_current_limit**2 * resistance
    )
    feasible = charge_limit <= terminal_power <= discharge_limit
    terminal_power = max(charge_limit, min(discharge_limit, terminal_power))
    discriminant = open_circuit_voltage**2 - 4.0 * resistance * terminal_power
    if discriminant < 0.0:
        feasible = False
        terminal_power = open_circuit_voltage**2 / (4.0 * resistance)
        discriminant = 0.0
    current = (
        open_circuit_voltage - math.sqrt(discriminant)
    ) / (2.0 * resistance)
    return current, feasible


def integrate_storage(
    parameters: EnergyStorageParameters,
    state: StorageState,
    bus_power: float,
    duration: float,
) -> StorageState:
    """Advance state of charge using current in ampere-seconds."""

    current, _ = solve_current(parameters, state, bus_power)
    state_of_charge = (
        _validate_state(state) - current * duration / parameters.pack_capacity_joules
    )
    return StorageState(max(0.0, min(1.0, state_of_charge)))


_STORAGE_UNITS = {
    "series_count": "1",
    "parallel_count": "1",
    "discharge_current_limit": "A",
    "charge_current_limit": "A",
    "cell_capacity": "A*h",
    "minimum_cell_voltage": "V",
    "maximum_cell_voltage": "V",
    "initial_state_of_charge": "1",
}


def compile_battery_pack(model: Model) -> EnergyStorageParameters:
    """Compile a battery XML model and validate its source curves."""

    values = closed_parameters(model, _STORAGE_UNITS)
    require_no_references(model)
    for name in ("series_count", "parallel_count"):
        value = values[name]
        if not value.is_integer() or value < 1:
            raise ValueError(f"{name} must be a positive integer")
    for name in ("discharge_current_limit", "charge_current_limit", "cell_capacity"):
        if values[name] <= 0.0:
            raise ValueError(f"{name} must be positive")
    if (
        values["minimum_cell_voltage"] <= 0.0
        or values["maximum_cell_voltage"] <= values["minimum_cell_voltage"]
    ):
        raise ValueError("cell voltage limits are invalid")
    if not 0.0 <= values["initial_state_of_charge"] <= 1.0:
        raise ValueError("initial_state_of_charge must be from zero through one")
    axis, open_circuit_voltage, resistance, energy = _curve()
    if any(value <= 0.0 for value in resistance):
        raise ValueError("source resistance must be positive")
    return EnergyStorageParameters(
        int(values.pop("series_count")),
        int(values.pop("parallel_count")),
        state_of_charge_axis=axis,
        open_circuit_voltage_axis=open_circuit_voltage,
        resistance_axis=resistance,
        energy_axis=energy,
        **values,
    )


register_component(
    "energy-storage",
    "battery-pack",
    capability=ENERGY_BUFFER,
    compile_function=compile_battery_pack,
)
