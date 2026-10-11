"""Compile the transient tire component.

This component is a fixture for the cross-fidelity promises.
It provides the transient-combined-slip capability.
The point-mass formulation does not request that capability.
Task P0C adds the reduction adapter from transient-combined-slip.
This module implements the schema only. It implements no tire physics.
"""

from __future__ import annotations

from dataclasses import dataclass

from simplified_vehicle_models.compiler.capabilities import TRANSIENT_COMBINED_SLIP
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    closed_parameters,
    require_model_type,
    require_no_references,
    require_positive,
)


TIRE_MODEL_TYPE = "tire-model"
TRANSIENT_SUBTYPE = "transient-combined-slip"

_PARAMETER_UNITS = {
    "relaxation_length": "m",
    "cornering_stiffness": "N/rad",
    "longitudinal_slip_stiffness": "N",
    "effective_radius": "m",
}


@dataclass(frozen=True, slots=True)
class TransientTireParameters:
    """Store the parameters of a transient tire model."""

    relaxation_length: float
    cornering_stiffness: float
    longitudinal_slip_stiffness: float
    effective_radius: float


def compile_transient_tire(model: Model) -> TransientTireParameters:
    """Compile one transient tire component."""

    require_model_type(model, TIRE_MODEL_TYPE, TRANSIENT_SUBTYPE)
    values = closed_parameters(model, _PARAMETER_UNITS)
    require_no_references(model)
    require_positive(
        model,
        values,
        "relaxation_length",
        "cornering_stiffness",
        "longitudinal_slip_stiffness",
        "effective_radius",
    )
    return TransientTireParameters(**values)


register_component(
    TIRE_MODEL_TYPE,
    TRANSIENT_SUBTYPE,
    capability=TRANSIENT_COMBINED_SLIP,
    compile_function=compile_transient_tire,
)
