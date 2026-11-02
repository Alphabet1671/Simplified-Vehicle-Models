"""XML compilers for physical struts and additive suspension assemblies."""
from __future__ import annotations

from collections.abc import Mapping

import numpy as np

from simplified_vehicle_models.compiler.capabilities import SUSPENSION_FORCES
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.suspension.components import CompiledDamper, _child
from simplified_vehicle_models.suspension.kinematics import SuspensionKinematics
from simplified_vehicle_models.suspension.layouts.parallel import STRUT_NAMES, build_parallel_suspension
from simplified_vehicle_models.suspension.physical import LinearDamper, PhysicalStrut
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import closed_parameters, require_no_references, require_nonnegative


class CompiledLinearDamper(LinearDamper):
    """Linear damper accepted through the common force(velocity) interface."""


def compile_linear_damper(model: Model) -> LinearDamper:
    require_no_references(model)
    values = closed_parameters(model, {"coefficient": "N*s/m"})
    require_nonnegative(model, values, "coefficient")
    if not np.isfinite(values["coefficient"]):
        raise ValueError(f"{model.path}: coefficient must be finite and nonnegative")
    return LinearDamper(values["coefficient"])


_STRUT_UNITS = {"spring_rate": "N/m", "preload": "N", "inertance": "kg",
                "damper_scale": "1", "motion_ratio": "1", "orientation": "1"}


def compile_physical_strut(model: Model, children: Mapping[str, object]) -> PhysicalStrut:
    values = closed_parameters(model, _STRUT_UNITS)
    require_nonnegative(model, values, "spring_rate", "preload", "inertance", "damper_scale")
    if values["motion_ratio"] <= 0:
        raise ValueError(f"{model.path}: motion_ratio must be finite and positive")
    if values["orientation"] not in (-1., 1.):
        raise ValueError(f"{model.path}: orientation must be exactly +1 or -1")
    damper = children["damper"]
    # Explicitly accept either registered damper family, without coupling struts
    # to table-only CompiledDamper: custom force(velocity) elements remain valid.
    if not (isinstance(damper, (CompiledDamper, LinearDamper))
            or callable(getattr(damper, "force", None))):
        raise ValueError(f"{model.path}: child 'damper' must provide force(velocity)")
    try:
        return PhysicalStrut(values["spring_rate"], values["preload"], values["inertance"], damper,
                             values["damper_scale"], values["motion_ratio"], values["orientation"])
    except ValueError as error:
        raise ValueError(f"{model.path}: {error}") from error


def compile_parallel_corner_axle(model: Model, children: Mapping[str, object]):
    closed_parameters(model, {})
    try:
        kinematics = _child(model, children, "kinematics", SuspensionKinematics)
        struts = tuple(_child(model, children, role, PhysicalStrut) for role in STRUT_NAMES)
        return build_parallel_suspension(kinematics=kinematics, struts=struts)
    except ValueError as error:
        raise ValueError(f"{model.path}: {error}") from error


register_component("damper-model", "linear", compile_function=compile_linear_damper)
register_component("suspension-strut", "physical", compile_function=compile_physical_strut,
                   nests_components=True, child_roles=("damper",))
register_component("suspension-assembly", "parallel-corner-axle", capability=SUSPENSION_FORCES,
                   compile_function=compile_parallel_corner_axle, nests_components=True,
                   child_roles=("kinematics", *STRUT_NAMES))
