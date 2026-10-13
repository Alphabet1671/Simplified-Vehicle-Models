"""Compile and evaluate a smooth motor torque, speed, and power envelope.

The fitted motor map in ``powertrain/power_plant.py`` is an interpolated table
with branch fallbacks. An optimal-control formulation cannot differentiate it.
This module supplies a smooth envelope instead, so the same motor limits can be
used by a simulation and by a nonlinear program.

The envelope has three bounds per motor:

- a torque bound, which is the peak or the continuous value;
- a power bound, which falls as ``power / speed`` above the base speed;
- a speed bound.

A caller takes the smallest. A nonlinear program should impose the bounds as
separate inequality constraints instead, because the smallest of two smooth
functions has a kink at the corner.

The voltage scales the base speed. A lower bus voltage reaches the power bound
sooner, which is the field-weakening limit in a simple form.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.compiler.capabilities import TORQUE_SOURCE
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    closed_parameters,
    require_model_type,
    require_no_references,
    require_open_unit_interval,
    require_positive,
)


POWER_PLANT_MODEL_TYPE = "power-plant"
TORQUE_ENVELOPE_SUBTYPE = "torque-speed-envelope"

_PARAMETER_UNITS = {
    "unit_count": "1",
    "gear_ratio": "1",
    "motor_efficiency": "1",
    "peak_motor_torque": "N*m",
    "continuous_motor_torque": "N*m",
    "peak_power": "W",
    "continuous_power": "W",
    "maximum_motor_speed": "rpm",
    "nominal_bus_voltage": "V",
    "maximum_bus_voltage": "V",
    "regeneration_torque_fraction": "1",
    "inverter_efficiency": "1",
}

#: Revolutions per minute to radians per second.
_RPM_TO_RADIANS_PER_SECOND = 2.0 * math.pi / 60.0


@dataclass(frozen=True, slots=True)
class MotorEnvelopeParameters:
    """Store one smooth motor envelope and its gearing.

    The speed values use radians per second in the object, even though the XML
    declares revolutions per minute.
    """

    unit_count: int
    gear_ratio: float
    motor_efficiency: float
    peak_motor_torque: float
    continuous_motor_torque: float
    peak_power: float
    continuous_power: float
    maximum_motor_speed: float
    nominal_bus_voltage: float
    maximum_bus_voltage: float
    regeneration_torque_fraction: float
    inverter_efficiency: float

    @property
    def base_motor_speed(self) -> float:
        """Return the speed where the power bound meets the torque bound."""

        return self.peak_power / self.peak_motor_torque

    @property
    def drive_efficiency(self) -> float:
        """Return the combined motor and inverter efficiency."""

        return self.motor_efficiency * self.inverter_efficiency

    def motor_speed(self, wheel_speed: float) -> float:
        """Return the motor speed for one wheel speed."""

        return self.gear_ratio * wheel_speed


def compile_motor_envelope(model: Model) -> MotorEnvelopeParameters:
    """Compile one smooth motor envelope power plant."""

    require_model_type(model, POWER_PLANT_MODEL_TYPE, TORQUE_ENVELOPE_SUBTYPE)
    values = closed_parameters(model, _PARAMETER_UNITS)
    require_no_references(model)
    require_positive(
        model,
        values,
        "gear_ratio",
        "peak_motor_torque",
        "continuous_motor_torque",
        "peak_power",
        "continuous_power",
        "maximum_motor_speed",
        "nominal_bus_voltage",
        "maximum_bus_voltage",
        "inverter_efficiency",
    )
    require_open_unit_interval(
        model,
        values,
        "motor_efficiency",
        "inverter_efficiency",
        "regeneration_torque_fraction",
    )
    if values["continuous_motor_torque"] > values["peak_motor_torque"]:
        raise ValueError(
            f"{model.path}: the continuous torque must not exceed the peak torque"
        )
    if values["continuous_power"] > values["peak_power"]:
        raise ValueError(
            f"{model.path}: the continuous power must not exceed the peak power"
        )
    if values["maximum_bus_voltage"] < values["nominal_bus_voltage"]:
        raise ValueError(
            f"{model.path}: the maximum bus voltage must cover the nominal value"
        )
    count = values["unit_count"]
    if count != round(count) or count < 1:
        raise ValueError(f"{model.path}: unit_count must be a positive whole number")
    return MotorEnvelopeParameters(
        unit_count=int(round(count)),
        gear_ratio=values["gear_ratio"],
        motor_efficiency=values["motor_efficiency"],
        peak_motor_torque=values["peak_motor_torque"],
        continuous_motor_torque=values["continuous_motor_torque"],
        peak_power=values["peak_power"],
        continuous_power=values["continuous_power"],
        maximum_motor_speed=values["maximum_motor_speed"]
        * _RPM_TO_RADIANS_PER_SECOND,
        nominal_bus_voltage=values["nominal_bus_voltage"],
        maximum_bus_voltage=values["maximum_bus_voltage"],
        regeneration_torque_fraction=values["regeneration_torque_fraction"],
        inverter_efficiency=values["inverter_efficiency"],
    )


def voltage_scaled_speed(
    parameters: MotorEnvelopeParameters,
    motor_speed: FloatArray,
    bus_voltage: float,
) -> FloatArray:
    """Return the speed that the voltage makes the motor see.

    A lower voltage makes the motor reach the power bound at a lower speed.
    Clamp the voltage to the nominal value, so a high voltage gives no free
    torque.
    """

    if not np.isfinite(bus_voltage) or bus_voltage <= 0.0:
        raise ValueError("bus_voltage must be finite and positive")
    effective = min(float(bus_voltage), parameters.nominal_bus_voltage)
    ratio = parameters.nominal_bus_voltage / effective
    return np.abs(np.asarray(motor_speed, dtype=np.float64)) * ratio


def motor_speed_limit(
    parameters: MotorEnvelopeParameters,
    bus_voltage: float,
) -> float:
    """Return the greatest motor speed that the voltage allows.

    A lower voltage reaches the speed limit sooner.
    """

    if not np.isfinite(bus_voltage) or bus_voltage <= 0.0:
        raise ValueError("bus_voltage must be finite and positive")
    effective = min(float(bus_voltage), parameters.nominal_bus_voltage)
    return parameters.maximum_motor_speed * effective / parameters.nominal_bus_voltage


def wheel_speed_limit(
    parameters: MotorEnvelopeParameters,
    bus_voltage: float,
) -> float:
    """Return the greatest wheel **angular** speed that the motor allows.

    The value uses radians per second. Multiply it by the tire radius for a
    ground speed.
    """

    return motor_speed_limit(parameters, bus_voltage) / parameters.gear_ratio


def motor_torque_bounds(
    parameters: MotorEnvelopeParameters,
    motor_speed: FloatArray,
    bus_voltage: float,
    peak: bool = True,
) -> tuple[FloatArray, FloatArray, FloatArray]:
    """Return the torque bound, the power bound, and the speed bound.

    Return the bounds separately, so a nonlinear program can impose each one as
    its own constraint. Take the smallest for a simulation.

    The speed bound is the greatest motor speed that the voltage allows. Above
    it the motor produces no torque. A caller should bound the speed as a
    variable instead, because the bound is a limit on the state.
    """

    speed = voltage_scaled_speed(parameters, motor_speed, bus_voltage)
    torque_rating = (
        parameters.peak_motor_torque if peak else parameters.continuous_motor_torque
    )
    power_rating = parameters.peak_power if peak else parameters.continuous_power
    torque_bound = np.full(np.shape(speed), torque_rating, dtype=np.float64)
    power_bound = power_rating / np.maximum(speed, 1.0e-9)
    speed_bound = np.full(
        np.shape(speed), parameters.maximum_motor_speed, dtype=np.float64
    )
    return torque_bound, power_bound, speed_bound


def motor_torque_limit(
    parameters: MotorEnvelopeParameters,
    motor_speed: FloatArray,
    bus_voltage: float,
    peak: bool = True,
) -> FloatArray:
    """Return the smaller of the torque bound and the power bound.

    A simulation uses this function. A nonlinear program uses
    ``motor_torque_bounds`` and imposes both bounds as constraints.
    """

    speed = np.abs(np.asarray(motor_speed, dtype=np.float64))
    torque, power, speed_bound = motor_torque_bounds(
        parameters, motor_speed, bus_voltage, peak
    )
    limit = np.minimum(torque, power)
    # Above the speed limit the motor cannot turn, so it makes no torque.
    return np.where(speed <= speed_bound, limit, 0.0)


def wheel_torque_limit(
    parameters: MotorEnvelopeParameters,
    wheel_speed: FloatArray,
    bus_voltage: float,
    peak: bool = True,
) -> FloatArray:
    """Return the drive torque that one wheel can accept.

    The gearing multiplies the motor torque. The gear and motor efficiency
    reduce the wheel torque for a given motor torque.
    """

    speed = np.asarray(wheel_speed, dtype=np.float64)
    motor_speed = parameters.gear_ratio * speed
    limit = motor_torque_limit(parameters, motor_speed, bus_voltage, peak)
    return limit * parameters.gear_ratio * parameters.motor_efficiency


def wheel_torque_bounds(
    parameters: MotorEnvelopeParameters,
    wheel_speed: FloatArray,
    bus_voltage: float,
) -> tuple[FloatArray, FloatArray]:
    """Return the wheel torque bound and the wheel power bound."""

    speed = np.asarray(wheel_speed, dtype=np.float64)
    torque, power, _ = motor_torque_bounds(
        parameters, parameters.gear_ratio * speed, bus_voltage
    )
    gearing = parameters.gear_ratio * parameters.motor_efficiency
    return torque * gearing, power * gearing


def shaft_power(
    parameters: MotorEnvelopeParameters,
    wheel_speed: FloatArray,
    wheel_torque: FloatArray,
) -> FloatArray:
    """Return the mechanical shaft power of one motor.

    A positive torque at a positive speed draws power. The power is negative in
    regeneration.
    """

    wheel_power = np.asarray(wheel_torque, dtype=np.float64) * np.asarray(
        wheel_speed, dtype=np.float64
    )
    # Motoring loses power through the drive; regeneration reverses its flow.
    return np.where(
        wheel_power >= 0.0,
        wheel_power / parameters.motor_efficiency,
        wheel_power * parameters.motor_efficiency,
    )


def bus_power(
    parameters: MotorEnvelopeParameters,
    wheel_speed: FloatArray,
    wheel_torque: FloatArray,
) -> FloatArray:
    """Return the bus power of one motor for a wheel torque.

    Motoring divides by the efficiency. Regeneration multiplies by it.
    """

    mechanical = shaft_power(parameters, wheel_speed, wheel_torque)
    efficiency = parameters.inverter_efficiency
    return np.where(mechanical >= 0.0, mechanical / efficiency, mechanical * efficiency)


def total_bus_power(
    parameters: MotorEnvelopeParameters,
    wheel_speeds: FloatArray,
    wheel_torques: FloatArray,
) -> float:
    """Return the bus power of every motor together."""

    return float(np.sum(bus_power(parameters, wheel_speeds, wheel_torques)))


register_component(
    POWER_PLANT_MODEL_TYPE,
    TORQUE_ENVELOPE_SUBTYPE,
    capability=TORQUE_SOURCE,
    compile_function=compile_motor_envelope,
)
