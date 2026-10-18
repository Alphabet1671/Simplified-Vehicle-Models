"""Register the hydraulic suspension components.

``hydraulic-circuit/npz`` loads one ``hydraulic-circuit-v1`` NPZ (sha256 pinned)
and has one scalar parameter, ``line_scale`` (unit ``1``): a multiplier on every
orifice resistance, for sweeps and for the stiff-circuit limit.

``suspension-assembly/hydraulic`` nests five children and compiles to a
``CompiledSuspension`` whose network carries the circuit gas fractions as states:

* ``kinematics`` (``suspension-kinematics/corner-table``),
* ``spring`` (``spring-model/corner-table``): the mechanical corner springs,
* ``circuit`` (``hydraulic-circuit/npz``),
* ``damper-front`` and ``damper-rear`` (``damper-model``).
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace

from simplified_vehicle_models.compiler.capabilities import SUSPENSION_FORCES
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.hydraulics.circuit import HydraulicCircuit
from simplified_vehicle_models.hydraulics.elements import OrificeSet
from simplified_vehicle_models.hydraulics.table import load_circuit_npz
from simplified_vehicle_models.suspension.components import CompiledDamper, _child
from simplified_vehicle_models.suspension.elements import CornerSprings
from simplified_vehicle_models.suspension.kinematics import SuspensionKinematics
from simplified_vehicle_models.suspension.layouts.hydraulic import build_hydraulic
from simplified_vehicle_models.suspension.network import CompiledSuspension
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    closed_parameters,
    require_no_references,
)

CIRCUIT_ROLE = "circuit-table"
HYDRAULIC_ROLES = ("kinematics", "spring", "circuit", "damper-front", "damper-rear")
_CIRCUIT_UNITS = {"line_scale": "1"}


def compile_circuit(model: Model) -> HydraulicCircuit:
    """Compile a hydraulic circuit: load the pinned NPZ and scale its line resistances."""

    require_no_references(model)
    values = closed_parameters(model, _CIRCUIT_UNITS)
    scale = values["line_scale"]
    if not scale > 0.0:
        raise ValueError(f"{model.path}: line_scale must be positive")
    references = [item for item in model.data if item.role == CIRCUIT_ROLE]
    if len(references) != 1 or len(model.data) != 1 or references[0].format != "npz":
        raise ValueError(f"{model.path}: requires one circuit-table NPZ declaration")
    reference = references[0]
    if not reference.sha256:
        raise ValueError(f"{model.path}: circuit-table declaration requires sha256")
    try:
        circuit = load_circuit_npz(reference.path, reference.sha256)
    except ValueError as error:
        raise ValueError(f"{model.path}: {error}") from error
    if scale != 1.0:
        orifices = circuit.orifices
        circuit = replace(circuit, orifices=OrificeSet(
            linear=orifices.linear * scale, quadratic=orifices.quadratic * scale))
    return circuit


def compile_hydraulic(model: Model, children: Mapping[str, object]) -> CompiledSuspension:
    """Compile the hydraulic suspension from its five compiled children."""

    closed_parameters(model, {})
    try:
        return build_hydraulic(
            kinematics=_child(model, children, "kinematics", SuspensionKinematics),
            springs=_child(model, children, "spring", CornerSprings),
            circuit=_child(model, children, "circuit", HydraulicCircuit),
            damper_front=_child(model, children, "damper-front", CompiledDamper),
            damper_rear=_child(model, children, "damper-rear", CompiledDamper),
        )
    except ValueError as error:
        raise ValueError(f"{model.path}: {error}") from error


register_component("hydraulic-circuit", "npz", compile_function=compile_circuit)
register_component(
    "suspension-assembly",
    "hydraulic",
    capability=SUSPENSION_FORCES,
    compile_function=compile_hydraulic,
    nests_components=True,
    child_roles=HYDRAULIC_ROLES,
)
