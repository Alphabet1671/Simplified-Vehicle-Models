"""Compile the suspension lookup-table component.

This component is a fixture for the cross-fidelity promises.
It provides the suspension-load-transfer capability.
No current formulation requests that capability.
This module implements the schema only. It implements no suspension physics.
"""

from __future__ import annotations

from dataclasses import dataclass

from simplified_vehicle_models.compiler.capabilities import SUSPENSION_LOAD_TRANSFER
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    closed_parameters,
    require_model_type,
    require_nonnegative,
    require_no_references,
    require_positive,
)


SUSPENSION_MODEL_TYPE = "suspension-model"
LOOKUP_TABLE_SUBTYPE = "lookup-table"

_PARAMETER_UNITS = {
    "front_roll_stiffness": "N*m/rad",
    "rear_roll_stiffness": "N*m/rad",
    "front_ride_height": "m",
    "rear_ride_height": "m",
    "motion_ratio": "1",
}


@dataclass(frozen=True, slots=True)
class SuspensionLookupParameters:
    """Store the parameters of a suspension lookup table."""

    front_roll_stiffness: float
    rear_roll_stiffness: float
    front_ride_height: float
    rear_ride_height: float
    motion_ratio: float


def compile_suspension_lookup(model: Model) -> SuspensionLookupParameters:
    """Compile one suspension lookup-table component."""

    require_model_type(model, SUSPENSION_MODEL_TYPE, LOOKUP_TABLE_SUBTYPE)
    values = closed_parameters(model, _PARAMETER_UNITS)
    require_no_references(model)
    require_positive(
        model,
        values,
        "front_roll_stiffness",
        "rear_roll_stiffness",
        "motion_ratio",
    )
    require_nonnegative(model, values, "front_ride_height", "rear_ride_height")
    return SuspensionLookupParameters(**values)


register_component(
    SUSPENSION_MODEL_TYPE,
    LOOKUP_TABLE_SUBTYPE,
    capability=SUSPENSION_LOAD_TRANSFER,
    compile_function=compile_suspension_lookup,
)
