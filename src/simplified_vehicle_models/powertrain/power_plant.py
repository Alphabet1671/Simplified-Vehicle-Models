"""Model a voltage-dependent electric motor.

The identification voltages are line-to-line peak values. Runtime equations
use per-phase peak values, so fitted flux and inductance values are divided by
square root of three before the voltage solve. Resistance remains physical.
"""

from __future__ import annotations

import csv
import hashlib
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from simplified_vehicle_models.compiler.capabilities import TORQUE_SOURCE
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    closed_parameters,
    require_no_references,
)

_ROOT = Path(__file__).resolve().parents[3]
_DATA = _ROOT / "models" / "data"
_SQRT_THREE = math.sqrt(3.0)


@dataclass(frozen=True, slots=True)
class DirectCurrentResult:
    """Report the direct current and voltage feasibility."""

    direct_current: float
    feasible: bool


@dataclass(frozen=True, slots=True)
class ShaftPowerResult:
    """Report shaft power, losses, and the active operating limit."""

    bus_power: float
    loss_power: float
    field_weakening_correction: float
    direct_current: float
    efficiency: float
    active_limit: str
    feasible: bool = True


@dataclass(frozen=True, slots=True)
class MotorOperatingMap:
    """Validated immutable normalized-load motor operating table."""

    voltage: np.ndarray
    speed: np.ndarray
    load: np.ndarray
    drive_torque: np.ndarray
    regen_torque: np.ndarray
    bus_power: np.ndarray
    loss_power: np.ndarray


@dataclass(frozen=True, slots=True)
class PowerPlantParameters:
    """Store motor maps and identified machine parameters."""

    unit_count: int
    maximum_motor_speed: float
    unit_power_limit: float
    inverter_efficiency: float
    device_drop: float
    pole_pairs: int
    reference_bus_voltage: float
    resistance: float
    flux_linkage: float
    direct_inductance: float
    quadrature_inductance: float
    torque_speed: tuple[float, ...]
    torque_values: tuple[float, ...]
    loss_speed: tuple[float, ...]
    loss_torque: tuple[float, ...]
    loss_values: tuple[float, ...]
    measured_reference_speed: float
    measured_reference_direct_current: float
    measured_reference_quadrature_current: float
    measured_reference_direct_voltage: float
    measured_reference_quadrature_voltage: float
    measured_reference_voltage: float
    operating_map: MotorOperatingMap | None = None

    @property
    def phase_flux_linkage(self) -> float:
        """Return flux linkage on the per-phase voltage basis."""

        return self.flux_linkage / _SQRT_THREE

    @property
    def phase_direct_inductance(self) -> float:
        """Return direct inductance on the per-phase voltage basis."""

        return self.direct_inductance / _SQRT_THREE

    @property
    def phase_quadrature_inductance(self) -> float:
        """Return quadrature inductance on the per-phase voltage basis."""

        return self.quadrature_inductance / _SQRT_THREE


def _read(name):
    with (_DATA / name).open(newline="") as stream:
        return [
            row
            for row in csv.reader(stream)
            if row and not row[0].startswith("#")
            and row[0] not in {"name", "speed_rpm"}
        ]


def _parameter_file():
    return {
        row[0]: float(row[1])
        for row in _read("amk_dd5_100c_machine_parameters.csv")
    }


def _interp(x_axis, y_axis, value):
    if value < x_axis[0] or value > x_axis[-1]:
        raise ValueError("motor speed is outside the map")
    return float(np.interp(value, x_axis, y_axis))


_LOSS_GRID_CACHE = {}


def _loss_grid(parameters):
    key = id(parameters)
    grid = _LOSS_GRID_CACHE.get(key)
    if grid is not None:
        return grid
    rows = {}
    for row_speed, torque_value, loss_value in zip(
        parameters.loss_speed, parameters.loss_torque, parameters.loss_values
    ):
        rows.setdefault(float(row_speed), []).append((float(torque_value), float(loss_value)))
    speeds = np.asarray(sorted(rows), dtype=float)
    torque_rows = []
    loss_rows = []
    for speed in speeds:
        pairs = sorted(rows[float(speed)])
        torque_rows.append(np.asarray([pair[0] for pair in pairs], dtype=float))
        loss_rows.append(np.asarray([pair[1] for pair in pairs], dtype=float))
    grid = (speeds, tuple(torque_rows), tuple(loss_rows))
    _LOSS_GRID_CACHE[key] = grid
    return grid


def _loss_interpolate(parameters, speed, torque):
    speeds, torque_rows, loss_rows = _loss_grid(parameters)
    speed = min(float(speeds[-1]), max(float(speeds[0]), speed))
    speed_index = int(np.searchsorted(speeds, speed, side="right"))
    upper = min(len(speeds) - 1, max(1, speed_index))
    lower = upper - 1
    lower_value = float(np.interp(torque, torque_rows[lower], loss_rows[lower]))
    upper_value = float(np.interp(torque, torque_rows[upper], loss_rows[upper]))
    if upper == lower:
        return lower_value
    fraction = (speed - speeds[lower]) / (speeds[upper] - speeds[lower])
    return lower_value + fraction * (upper_value - lower_value)


def _map_values(speed, torque, parameters):
    if speed > parameters.maximum_motor_speed:
        raise ValueError("motor speed exceeds maximum_motor_speed")
    speed = max(speed, parameters.torque_speed[0])
    envelope = _interp(parameters.torque_speed, parameters.torque_values, speed)
    loss = _loss_interpolate(
        parameters, max(speed, parameters.loss_speed[0]), abs(torque)
    )
    return envelope, loss


def calculate_available_voltage(parameters, bus_voltage, current):
    """Return available per-phase peak voltage.

    ``current`` is the signed q-axis current estimate. It provides the
    resistive drop before the direct-axis solve and is refined by that solve.
    """

    return (
        bus_voltage / _SQRT_THREE
        - parameters.resistance * current
        - parameters.device_drop
    )


def maximum_motor_torque(parameters, motor_speed, available_voltage):
    """Scale the map envelope along its speed axis."""

    _map_values(motor_speed, 0.0, parameters)
    if available_voltage <= 0.0:
        return 0.0
    reference_speed = (
        motor_speed * parameters.reference_bus_voltage / _SQRT_THREE / available_voltage
    )
    if reference_speed > parameters.torque_speed[-1]:
        return 0.0
    return _interp(
        parameters.torque_speed,
        parameters.torque_values,
        max(reference_speed, parameters.torque_speed[0]),
    )


def solve_direct_current(
    parameters,
    motor_speed,
    quadrature_current,
    available_voltage,
):
    """Solve the d-axis voltage equation and report infeasibility."""

    electrical_speed = (
        parameters.pole_pairs * 2.0 * math.pi * max(motor_speed, 0.0) / 60.0
    )
    resistance = parameters.resistance
    direct_inductance = parameters.phase_direct_inductance
    quadrature_inductance = parameters.phase_quadrature_inductance
    flux_linkage = parameters.phase_flux_linkage
    direct_term = electrical_speed * direct_inductance
    quadrature_term = electrical_speed * quadrature_inductance
    a = resistance**2 + direct_term**2
    b = (
        -2.0 * quadrature_term * quadrature_current * resistance
        + 2.0 * direct_term * (
            resistance * quadrature_current + electrical_speed * flux_linkage
        )
    )
    c = (
        (-quadrature_term * quadrature_current) ** 2
        + (resistance * quadrature_current + electrical_speed * flux_linkage) ** 2
        - available_voltage**2
    )
    if abs(a) < 1.0e-30:
        return DirectCurrentResult(0.0, c <= 0.0)
    discriminant = b**2 - 4.0 * a * c
    if discriminant < 0.0:
        return DirectCurrentResult(0.0, False)
    roots = (
        (-b - math.sqrt(discriminant)) / (2.0 * a),
        (-b + math.sqrt(discriminant)) / (2.0 * a),
    )
    return DirectCurrentResult(min(roots, key=abs), True)


def evaluate_shaft_power_exact(parameters, motor_speed, motor_torque, bus_voltage):
    """Analytical motor oracle: evaluate power or report voltage infeasibility."""

    quadrature_current = motor_torque / (
        1.5 * parameters.pole_pairs * parameters.phase_flux_linkage
    )
    available_voltage = calculate_available_voltage(
        parameters, bus_voltage, quadrature_current
    )
    current = solve_direct_current(
        parameters, motor_speed, quadrature_current, available_voltage
    )
    _, map_loss = _map_values(motor_speed, motor_torque, parameters)
    shaft_power = motor_speed * 2.0 * math.pi / 60.0 * motor_torque
    if not current.feasible:
        return ShaftPowerResult(
            0.0,
            map_loss,
            0.0,
            current.direct_current,
            0.0,
            "voltage",
            False,
        )
    reference_voltage = calculate_available_voltage(
        parameters, parameters.reference_bus_voltage, quadrature_current
    )
    reference = solve_direct_current(
        parameters, motor_speed, quadrature_current, reference_voltage
    )
    if not reference.feasible:
        return ShaftPowerResult(
            0.0,
            map_loss,
            0.0,
            current.direct_current,
            0.0,
            "voltage",
            False,
        )
    correction = (
        0.0
        if abs(bus_voltage - parameters.reference_bus_voltage) < 1.0e-12
        else parameters.resistance
        * 1.5
        * (current.direct_current**2 - reference.direct_current**2)
    )
    # Voltage correction represents an added physical loss. A lower current
    # estimate cannot make total loss negative.
    loss_power = map_loss + max(correction, 0.0)
    raw_bus_power = shaft_power + loss_power
    if raw_bus_power >= 0.0:
        bus_power = raw_bus_power / parameters.inverter_efficiency
        efficiency = shaft_power / bus_power if bus_power else 1.0
    else:
        bus_power = raw_bus_power * parameters.inverter_efficiency
        efficiency = bus_power / shaft_power if shaft_power else 1.0
    active_limit = (
        "power" if abs(shaft_power) >= parameters.unit_power_limit else "torque"
    )
    return ShaftPowerResult(
        bus_power,
        loss_power,
        correction,
        current.direct_current,
        efficiency,
        active_limit,
        True,
    )


# Backward-compatible name retained until LUT-first queries are introduced.
evaluate_shaft_power = evaluate_shaft_power_exact


def scalar_efficiency_bus_power(shaft_power, efficiency):
    """Convert shaft power to bus power using one scalar efficiency."""

    if not 0.0 < efficiency <= 1.0:
        raise ValueError("efficiency must be above zero and no greater than one")
    if shaft_power >= 0.0:
        return shaft_power / efficiency
    return shaft_power * efficiency


def _reference_row():
    rows = _read("amk_dd5_100c_identification.csv")
    candidates = [row for row in rows if abs(float(row[0]) - 16300.0) < 1.0e-9]
    return max(candidates, key=lambda row: math.hypot(float(row[2]), float(row[3])))


_UNITS = {
    "unit_count": "1",
    "maximum_motor_speed": "rpm",
    "unit_power_limit": "W",
    "inverter_efficiency": "1",
    "device_drop": "V",
}


def _compile_motor_source(model: Model, operating_map=None) -> PowerPlantParameters:
    values = closed_parameters(model, _UNITS)
    require_no_references(model)
    if not values["unit_count"].is_integer() or values["unit_count"] < 1:
        raise ValueError("unit_count must be a positive integer")
    for name in ("maximum_motor_speed", "unit_power_limit"):
        if values[name] <= 0.0:
            raise ValueError(f"{name} must be positive")
    if not 0.0 < values["inverter_efficiency"] <= 1.0:
        raise ValueError("inverter_efficiency must be from zero to one")
    parameters = _parameter_file()
    if parameters["resistance"] <= 0.0:
        raise ValueError("motor resistance must be positive")
    torque_map = _read("amk_dd5_100c_torque_map.csv")
    loss_map = _read("amk_dd5_100c_loss_map.csv")
    reference = _reference_row()
    return PowerPlantParameters(
        int(values["unit_count"]), values["maximum_motor_speed"], values["unit_power_limit"],
        values["inverter_efficiency"], values["device_drop"], int(parameters["pole_pairs"]),
        parameters["reference_bus_voltage"], parameters["resistance"], parameters["flux_linkage"],
        parameters["direct_inductance"], parameters["quadrature_inductance"],
        tuple(float(row[0]) for row in torque_map), tuple(float(row[1]) for row in torque_map),
        tuple(float(row[0]) for row in loss_map), tuple(float(row[1]) for row in loss_map),
        tuple(float(row[2]) for row in loss_map), float(reference[0]), float(reference[2]),
        float(reference[3]), float(reference[4]), float(reference[5]),
        math.hypot(float(reference[4]), float(reference[5])), operating_map,
    )


def compile_analytical_motor_source(model: Model) -> PowerPlantParameters:
    """Private generator-only construction path without a persisted LUT."""
    return _compile_motor_source(model)


def _validate_operating_map(data: dict[str, np.ndarray]) -> MotorOperatingMap:
    names = ("voltage", "speed", "load", "drive_torque", "regen_torque", "bus_power", "loss_power")
    if any(name not in data for name in names):
        raise ValueError("operating map is missing required arrays")
    arrays = {name: np.array(data[name], dtype=np.float64, order="C", copy=True) for name in names}
    v, s, l = arrays["voltage"], arrays["speed"], arrays["load"]
    if v.ndim != 1 or s.ndim != 1 or l.ndim != 1 or v.size < 2 or s.size < 2 or l.size < 3:
        raise ValueError("operating map axes have invalid shape")
    if not all(np.all(np.isfinite(a)) for a in arrays.values()):
        raise ValueError("operating map contains nonfinite values")
    if not np.all(np.diff(v) > 0) or not np.all(np.diff(s) > 0) or not np.all(np.diff(l) > 0):
        raise ValueError("operating map axes must be strictly increasing")
    if l[0] != -1.0 or l[-1] != 1.0:
        raise ValueError("operating map load axis must span -1 to 1")
    shape2, shape3 = (v.size, s.size), (v.size, s.size, l.size)
    if arrays["drive_torque"].shape != shape2 or arrays["regen_torque"].shape != shape2:
        raise ValueError("operating map torque envelope shape mismatch")
    if arrays["bus_power"].shape != shape3 or arrays["loss_power"].shape != shape3:
        raise ValueError("operating map power shape mismatch")
    if np.any(arrays["drive_torque"] < 0) or np.any(arrays["regen_torque"] < 0):
        raise ValueError("operating map torque envelopes must be nonnegative")
    if np.any(arrays["loss_power"] < 0):
        raise ValueError("operating map loss must be nonnegative")
    for row in arrays["bus_power"].reshape(-1, l.size):
        if np.any(np.diff(row[: l.size // 2 + 1]) < -1e-9) or np.any(np.diff(row[l.size // 2 :]) < -1e-9):
            raise ValueError("operating map bus power is not monotone by direction")
    for array in arrays.values():
        array.setflags(write=False)
    return MotorOperatingMap(**arrays)


def _load_operating_map(model: Model) -> MotorOperatingMap:
    references = [item for item in model.data if item.role == "operating-map"]
    if len(references) != 1 or references[0].format != "npz":
        raise ValueError("electric motor requires one operating-map NPZ declaration")
    reference = references[0]
    if not reference.sha256:
        raise ValueError("operating-map declaration requires sha256")
    digest = hashlib.sha256(reference.path.read_bytes()).hexdigest()
    if digest.lower() != reference.sha256.lower():
        raise ValueError("operating-map digest mismatch")
    with np.load(reference.path, allow_pickle=False) as archive:
        return _validate_operating_map({name: archive[name] for name in archive.files})


def compile_electric_motor(model: Model) -> PowerPlantParameters:
    """Compile the XML motor and require its declared, validated LUT."""
    return _compile_motor_source(model, _load_operating_map(model))


register_component(
    "power-plant",
    "electric-motor",
    capability=TORQUE_SOURCE,
    compile_function=compile_electric_motor,
)
