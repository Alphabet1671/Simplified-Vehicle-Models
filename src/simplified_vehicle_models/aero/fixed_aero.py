"""Compile the fixed-aero component.

The component declares the aero-loads capability.
The behavior functions stay in the point-mass formulation until Task P0C.
"""

from __future__ import annotations

from simplified_vehicle_models.compiler.capabilities import AERO_LOADS
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.parameters.aero import AeroParameters
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    closed_parameters,
    require_closed_unit_interval,
    require_model_type,
    require_nonnegative,
    require_no_references,
    require_positive,
)


AERO_MODEL_TYPE = "aero-model"
FIXED_AERO_SUBTYPE = "fixed-aero"

_PARAMETER_UNITS = {
    "air_density": "kg/m^3",
    "downforce_area": "m^2",
    "drag_area": "m^2",
    "aero_balance": "1",
}


def compile_fixed_aero(model: Model) -> AeroParameters:
    """Compile one fixed-aero component."""

    require_model_type(model, AERO_MODEL_TYPE, FIXED_AERO_SUBTYPE)
    values = closed_parameters(model, _PARAMETER_UNITS)
    require_no_references(model)
    require_positive(model, values, "air_density")
    require_nonnegative(model, values, "downforce_area", "drag_area")
    require_closed_unit_interval(model, values, "aero_balance")
    return AeroParameters(**values)


register_component(
    AERO_MODEL_TYPE,
    FIXED_AERO_SUBTYPE,
    capability=AERO_LOADS,
    compile_function=compile_fixed_aero,
)
