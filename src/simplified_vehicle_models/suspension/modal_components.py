"""Register the modal suspension assemblies and their mode-rate spring.

Two ``suspension-assembly`` subtypes nest their parts as child components:

* ``axle-roll-heave``: ``kinematics`` and, for each axle, a heave spring, a heave
  damper, a roll spring and a roll damper. Roles are ``heave-spring-front``,
  ``heave-damper-front``, ``roll-spring-front``, ``roll-damper-front`` and the
  same four with ``-rear``.
* ``four-mode``: ``kinematics``, ``heave-spring``, ``roll-spring``,
  ``pitch-spring``, ``warp-spring`` and the four matching ``-damper`` roles.

``spring-model/mode-rate`` holds one wheel-referred mode rate [N/m]. A mode
damper is a ``damper-model/table-mode``: the same damper table and click
parameters as a corner damper. Axle roll/heave mode dampers apply the configured
DSS wheel/shaft ratio and the orthonormal-mode travel Jacobian; four-mode dampers
retain direct wheel-referred mode velocity pending an explicit ratio definition.

The assemblies provide the single ``suspension-forces`` capability.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from simplified_vehicle_models.compiler.capabilities import SUSPENSION_FORCES
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.suspension.components import (
    CompiledDamper,
    _child,
    compile_damper,
)
from simplified_vehicle_models.suspension.kinematics import SuspensionKinematics
from simplified_vehicle_models.suspension.layouts.axle_roll_heave import (
    build_axle_roll_heave,
)
from simplified_vehicle_models.suspension.layouts.four_mode import build_four_mode
from simplified_vehicle_models.suspension.network import CompiledSuspension
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    closed_parameters,
    require_no_references,
    require_nonnegative,
)

_RATE_UNITS = {"rate": "N/m"}

_AXLE_MOTION_UNITS = {
    "motion_ratio_heave_front": "1", "motion_ratio_roll_front": "1",
    "motion_ratio_heave_rear": "1", "motion_ratio_roll_rear": "1",
}

AXLE_ROLES = (
    "kinematics",
    "heave-spring-front", "heave-damper-front", "roll-spring-front", "roll-damper-front",
    "heave-spring-rear", "heave-damper-rear", "roll-spring-rear", "roll-damper-rear",
)
FOUR_MODE_ROLES = (
    "kinematics",
    "heave-spring", "heave-damper", "roll-spring", "roll-damper",
    "pitch-spring", "pitch-damper", "warp-spring", "warp-damper",
)


@dataclass(frozen=True)
class ModeRate:
    """One wheel-referred mode spring rate [N/m]."""

    rate: float


def compile_mode_rate(model: Model) -> ModeRate:
    """Compile a mode spring given as one wheel-referred rate."""

    require_no_references(model)
    values = closed_parameters(model, _RATE_UNITS)
    require_nonnegative(model, values, "rate")
    return ModeRate(rate=values["rate"])


def _rate(model: Model, children: Mapping[str, object], role: str) -> float:
    return _child(model, children, role, ModeRate).rate


def _damper(model: Model, children: Mapping[str, object], role: str) -> CompiledDamper:
    return _child(model, children, role, CompiledDamper)


def compile_axle_roll_heave(model: Model, children: Mapping[str, object]) -> CompiledSuspension:
    """Compile the axle roll/heave suspension from its nine compiled children."""

    values = closed_parameters(model, _AXLE_MOTION_UNITS)
    if any(value <= 0.0 for value in values.values()):
        raise ValueError(f"{model.path}: all heave/roll motion ratios must be positive")
    try:
        return build_axle_roll_heave(
            kinematics=_child(model, children, "kinematics", SuspensionKinematics),
            heave_rate_front=_rate(model, children, "heave-spring-front"),
            roll_rate_front=_rate(model, children, "roll-spring-front"),
            heave_rate_rear=_rate(model, children, "heave-spring-rear"),
            roll_rate_rear=_rate(model, children, "roll-spring-rear"),
            heave_damper_front=_damper(model, children, "heave-damper-front"),
            roll_damper_front=_damper(model, children, "roll-damper-front"),
            heave_damper_rear=_damper(model, children, "heave-damper-rear"),
            roll_damper_rear=_damper(model, children, "roll-damper-rear"),
            motion_ratios=(values["motion_ratio_heave_front"], values["motion_ratio_roll_front"],
                           values["motion_ratio_heave_rear"], values["motion_ratio_roll_rear"]),
        )
    except ValueError as error:
        raise ValueError(f"{model.path}: {error}") from error


def compile_four_mode(model: Model, children: Mapping[str, object]) -> CompiledSuspension:
    """Compile the four-mode suspension from its nine compiled children."""

    closed_parameters(model, {})
    try:
        return build_four_mode(
            kinematics=_child(model, children, "kinematics", SuspensionKinematics),
            heave_rate=_rate(model, children, "heave-spring"),
            roll_rate=_rate(model, children, "roll-spring"),
            pitch_rate=_rate(model, children, "pitch-spring"),
            warp_rate=_rate(model, children, "warp-spring"),
            heave_damper=_damper(model, children, "heave-damper"),
            roll_damper=_damper(model, children, "roll-damper"),
            pitch_damper=_damper(model, children, "pitch-damper"),
            warp_damper=_damper(model, children, "warp-damper"),
        )
    except ValueError as error:
        raise ValueError(f"{model.path}: {error}") from error


register_component("spring-model", "mode-rate", compile_function=compile_mode_rate)
register_component("damper-model", "table-mode", compile_function=compile_damper)
register_component(
    "suspension-assembly",
    "axle-roll-heave",
    capability=SUSPENSION_FORCES,
    compile_function=compile_axle_roll_heave,
    nests_components=True,
    child_roles=AXLE_ROLES,
)
register_component(
    "suspension-assembly",
    "four-mode",
    capability=SUSPENSION_FORCES,
    compile_function=compile_four_mode,
    nests_components=True,
    child_roles=FOUR_MODE_ROLES,
)
