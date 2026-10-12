"""Compile and evaluate an independent fixed-ratio transmission."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.compiler.capabilities import TORQUE_TRANSMISSION
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    closed_parameters,
    require_closed_unit_interval,
    require_model_type,
    require_no_references,
    require_open_closed_unit_interval,
    require_positive,
)


TRANSMISSION_MODEL_TYPE = "transmission"
FIXED_GEAR_SUBTYPE = "fixed-gear"
_WHEEL_POSITIONS = ("fl", "fr", "rl", "rr")
_UNITS = {
    "gear_ratio": "1",
    "gear_efficiency": "1",
    "front_torque_fraction": "1",
}


@dataclass(frozen=True, slots=True)
class FixedGearParameters:
    """Store fixed-ratio transmission parameters and driven wheel positions."""

    gear_ratio: float
    gear_efficiency: float
    front_torque_fraction: float
    driven_wheels: tuple[str, ...]


def motor_speed_from_wheel_speed(
    transmission: FixedGearParameters, wheel_speed: float
) -> float:
    """Convert wheel speed to motor speed with the fixed ratio."""

    return transmission.gear_ratio * wheel_speed


def wheel_torque_from_motor_torque(
    transmission: FixedGearParameters, motor_torque: float
) -> FloatArray:
    """Convert motor torque to equal-split torque at each driven wheel.

    Positive torque is motoring and applies efficiency as a loss. Negative
    torque is regenerative and applies the reciprocal efficiency.
    """

    efficiency = transmission.gear_efficiency
    directional_factor = efficiency if motor_torque >= 0.0 else 1.0 / efficiency
    total_wheel_torque = transmission.gear_ratio * motor_torque * directional_factor
    front_torque = total_wheel_torque * transmission.front_torque_fraction / 2.0
    rear_torque = total_wheel_torque * (1.0 - transmission.front_torque_fraction) / 2.0
    values = {
        "fl": front_torque,
        "fr": front_torque,
        "rl": rear_torque,
        "rr": rear_torque,
    }
    return np.asarray([values[position] for position in transmission.driven_wheels], dtype=float)


def wheel_speed_to_motor_speed(
    transmission: FixedGearParameters, wheel_speed: float
) -> float:
    """Convert wheel speed to motor speed."""

    return motor_speed_from_wheel_speed(transmission, wheel_speed)


def motor_torque_to_wheel_torque(
    transmission: FixedGearParameters, motor_torque: float
) -> FloatArray:
    """Convert motor torque to per-wheel torque."""

    return wheel_torque_from_motor_torque(transmission, motor_torque)


def compile_fixed_gear_transmission(model: Model) -> FixedGearParameters:
    """Compile one fixed-ratio transmission XML model."""

    require_model_type(model, TRANSMISSION_MODEL_TYPE, FIXED_GEAR_SUBTYPE)
    values = closed_parameters(model, _UNITS)
    require_no_references(model)
    require_positive(model, values, "gear_ratio")
    require_open_closed_unit_interval(model, values, "gear_efficiency")
    require_closed_unit_interval(model, values, "front_torque_fraction")
    driven_wheels = model.driven_wheels
    if len(driven_wheels) != 4 or set(driven_wheels) != set(_WHEEL_POSITIONS):
        raise ValueError(
            f"{model.path}: driven wheels must contain fl, fr, rl, and rr exactly once"
        )
    return FixedGearParameters(driven_wheels=driven_wheels, **values)


register_component(
    TRANSMISSION_MODEL_TYPE,
    FIXED_GEAR_SUBTYPE,
    capability=TORQUE_TRANSMISSION,
    compile_function=compile_fixed_gear_transmission,
)
