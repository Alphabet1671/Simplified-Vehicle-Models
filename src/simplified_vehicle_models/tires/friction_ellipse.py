"""Compile the load-sensitive friction-ellipse tire component.

The component declares the tire-force-envelope capability.
The behavior functions stay in the point-mass formulation until Task P0C.
"""

from __future__ import annotations

from simplified_vehicle_models.compiler.capabilities import TIRE_FORCE_ENVELOPE
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.parameters.tire import TireEnvelopeParameters
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    closed_parameters,
    require_model_type,
    require_no_references,
    require_positive,
)


TIRE_MODEL_TYPE = "tire-model"
FRICTION_ELLIPSE_SUBTYPE = "load-sensitive-friction-ellipse"

_PARAMETER_UNITS = {
    "longitudinal_mu_scale": "1",
    "lateral_mu_scale": "1",
    "load_sensitivity_slope": "1/N",
    "load_sensitivity_intercept": "1",
    "effective_radius": "m",
}


def compile_tire_envelope(model: Model) -> TireEnvelopeParameters:
    """Compile one load-sensitive friction-ellipse tire component."""

    require_model_type(model, TIRE_MODEL_TYPE, FRICTION_ELLIPSE_SUBTYPE)
    values = closed_parameters(model, _PARAMETER_UNITS)
    require_no_references(model)
    require_positive(
        model,
        values,
        "longitudinal_mu_scale",
        "lateral_mu_scale",
        "load_sensitivity_intercept",
        "effective_radius",
    )
    if values["load_sensitivity_slope"] > 0.0:
        raise ValueError(
            f"{model.path}: load_sensitivity_slope must be nonpositive"
        )
    return TireEnvelopeParameters(**values)


register_component(
    TIRE_MODEL_TYPE,
    FRICTION_ELLIPSE_SUBTYPE,
    capability=TIRE_FORCE_ENVELOPE,
    compile_function=compile_tire_envelope,
)
