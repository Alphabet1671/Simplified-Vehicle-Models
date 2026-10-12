"""Model a voltage-dependent electric motor.

The identification voltages are line-to-line peak values. Runtime equations
use per-phase peak values, so fitted flux and inductance values are divided by
square root of three before the voltage solve. Resistance remains physical.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from simplified_vehicle_models.compiler.capabilities import TORQUE_SOURCE
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import closed_parameters, require_no_references

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
    measured_reference_voltage: float

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
    return {row[0]: float(row[1]) for row in _read("amk_dd5_100c_machine_parameters.csv")}


def _interp(x_axis, y_axis, value):
    if value < x_axis[0] or value > x_axis[-1]:
        raise ValueError("motor speed is outside the map")
    return float(np.interp(value, x_axis, y_axis))


def _loss_interpolate(parameters, speed, torque):
    speeds = sorted(set(parameters.loss_speed))
    minimum_torque = min(parameters.loss_torque)
    maximum_torque = max(parameters.loss_torque)
    clamped_torque = max(minimum_torque, min(maximum_torque, torque))
    values = []
    for speed_value in speeds:
        pairs = [
            (torque_value, loss_value)
            for row_speed, torque_value, loss_value in zip(
                parameters.loss_speed,
                parameters.loss_torque,
                parameters.loss_values,
            )
            if row_speed == speed_value
        ]
        pairs.sort()
        values.append(
            float(
                np.interp(
                    clamped_torque,
                    [pair[0] for pair in pairs],
                    [pair[1] for pair in pairs],
                )
            )
        )
    return float(np.interp(speed, speeds, values))


def _map_values(speed, torque, parameters):
    if speed > parameters.maximum_motor_speed:
        raise ValueError("motor speed exceeds maximum_motor_speed")
    speed = max(speed, parameters.torque_speed[0])
    envelope = _interp(parameters.torque_speed, parameters.torque_values, speed)
    loss = _loss_interpolate(parameters, max(speed, parameters.loss_speed[0]), abs(torque))
    return envelope, loss


def calculate_available_voltage(parameters, bus_voltage, current):
    """Return available per-phase peak voltage.

    ``current`` is the signed q-axis current estimate. It provides the
    resistive drop before the direct-axis solve and is refined by that solve.
    """

    return bus_voltage / _SQRT_THREE - parameters.resistance * current - parameters.device_drop


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


def evaluate_shaft_power(parameters, motor_speed, motor_torque, bus_voltage):
    """Evaluate shaft and bus power, or report an infeasible voltage demand."""

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
    loss_power = map_loss + correction
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


def compile_electric_motor(model: Model) -> PowerPlantParameters:
    """Compile the motor XML model and validate its data."""

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
        int(values["unit_count"]),
        values["maximum_motor_speed"],
        values["unit_power_limit"],
        values["inverter_efficiency"],
        values["device_drop"],
        int(parameters["pole_pairs"]),
        parameters["reference_bus_voltage"],
        parameters["resistance"],
        parameters["flux_linkage"],
        parameters["direct_inductance"],
        parameters["quadrature_inductance"],
        tuple(float(row[0]) for row in torque_map),
        tuple(float(row[1]) for row in torque_map),
        tuple(float(row[0]) for row in loss_map),
        tuple(float(row[1]) for row in loss_map),
        tuple(float(row[2]) for row in loss_map),
        float(reference[0]),
        float(reference[2]),
        float(reference[3]),
        math.hypot(float(reference[4]), float(reference[5])),
    )


register_component(
    "power-plant",
    "electric-motor",
    capability=TORQUE_SOURCE,
    compile_function=compile_electric_motor,
)
