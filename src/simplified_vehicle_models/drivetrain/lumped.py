"""Compile the lumped four-wheel-electric drivetrain component.

The component declares the drive-force-limit capability.
The behavior functions stay in the point-mass formulation until Task P0C.
Task P7 registers the full powertrain reduction into the same capability.
"""

from __future__ import annotations

from simplified_vehicle_models.compiler.capabilities import DRIVE_FORCE_LIMIT
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.parameters.drivetrain import DrivetrainParameters
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    closed_parameters,
    require_model_type,
    require_no_references,
    require_open_closed_unit_interval,
    require_positive,
)


DRIVETRAIN_MODEL_TYPE = "drivetrain-model"
LUMPED_SUBTYPE = "four-wheel-electric"

_PARAMETER_UNITS = {
    "motor_count": "1",
    "gear_ratio": "1",
    "peak_motor_torque": "N*m",
    "power_limit": "W",
    "efficiency": "1",
    "maximum_motor_speed": "rpm",
}


def compile_lumped_drivetrain(model: Model) -> DrivetrainParameters:
    """Compile one lumped four-wheel-electric drivetrain component."""

    require_model_type(model, DRIVETRAIN_MODEL_TYPE, LUMPED_SUBTYPE)
    values = closed_parameters(model, _PARAMETER_UNITS)
    require_no_references(model)
    motor_count = values.pop("motor_count")
    if not motor_count.is_integer() or motor_count < 1:
        raise ValueError(f"{model.path}: motor_count must be a positive integer")
    require_positive(
        model,
        values,
        "gear_ratio",
        "peak_motor_torque",
        "power_limit",
        "maximum_motor_speed",
    )
    require_open_closed_unit_interval(model, values, "efficiency")
    return DrivetrainParameters(motor_count=int(motor_count), **values)


register_component(
    DRIVETRAIN_MODEL_TYPE,
    LUMPED_SUBTYPE,
    capability=DRIVE_FORCE_LIMIT,
    compile_function=compile_lumped_drivetrain,
)
