"""Register the suspension model components: the table-based damper and the corner suspension.

A damper XML names one damper-table NPZ (sha256 pinned) and one integer click
per valve. A 2-valve damper (Multimatic DSSV VC01) uses ``click_compression``
and ``click_rebound``. A damper with more valves uses ``click_<valve_name>``.
The compiled value is a ``CompiledDamper`` that evaluates ``force(velocity)``.

A suspension is a ``suspension-assembly`` that nests its parts. The
``independent-corner`` layout nests five children:

* ``kinematics`` (``suspension-kinematics/corner-table``): the geometry from one
  corner-table NPZ (sha256 pinned) and the static toe of both axles.
* ``spring`` (``spring-model/corner-table``): the spring curves from the same NPZ.
* ``arb`` (``arb-model/axle-rate``): one anti-roll-bar wheel rate per axle.
* ``damper-front`` and ``damper-rear`` (``damper-model``).

The inertances of both axles are scalars of the assembly. The compiled value is a
``CompiledSuspension`` (kinematics plus force network).
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from simplified_vehicle_models.compiler.capabilities import (
    FRONT_DAMPER_FORCE,
    REAR_DAMPER_FORCE,
    SUSPENSION_FORCES,
)
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.suspension.corner import (
    geometry_from_arrays,
    load_corner_arrays,
    spring_from_arrays,
)
from simplified_vehicle_models.suspension.elements import AxleArbRates, CornerSprings
from simplified_vehicle_models.suspension.kinematics import SuspensionKinematics
from simplified_vehicle_models.suspension.layouts.independent_corner import (
    build_independent_corner,
)
from simplified_vehicle_models.suspension.network import CompiledSuspension
from simplified_vehicle_models.suspension.damper_table import (
    DamperForce,
    DamperSetting,
    DamperTable,
    damper_force,
    load_damper_npz,
)
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    closed_parameters,
    require_no_references,
    require_nonnegative,
)

DAMPER_TABLE_ROLE = "damper-table"
CORNER_TABLE_ROLE = "corner-table"
_CLICK_PREFIX = "click_"
_KINEMATICS_UNITS = {"static_toe_front": "rad", "static_toe_rear": "rad"}
_ARB_UNITS = {"arb_wheel_rate_front": "N/m", "arb_wheel_rate_rear": "N/m"}
_INDEPENDENT_UNITS = {"inertance_front": "kg", "inertance_rear": "kg"}


@dataclass(frozen=True, slots=True)
class CompiledDamper:
    """Pair one damper table with the validated click setting of one damper."""

    table: DamperTable
    setting: DamperSetting

    def force(self, velocity) -> DamperForce:
        """Return force (N), slope (N s/m), and range flag at ``velocity`` (m/s)."""

        return damper_force(self.table, self.setting, velocity)


def compile_damper(model: Model) -> CompiledDamper:
    """Compile a table damper: load the pinned NPZ and map clicks to valves."""

    require_no_references(model)
    references = [item for item in model.data if item.role == DAMPER_TABLE_ROLE]
    if len(references) != 1 or len(model.data) != 1 or references[0].format != "npz":
        raise ValueError(f"{model.path}: damper requires one damper-table NPZ declaration")
    reference = references[0]
    if not reference.sha256:
        raise ValueError(f"{model.path}: damper-table declaration requires sha256")
    table = load_damper_npz(reference.path, reference.sha256)
    for name, unit in model.units.items():
        if unit != "1":
            raise ValueError(f"{model.path}: parameter {name!r} must use unit '1'")
    expected = {_CLICK_PREFIX + valve.name for valve in table.valves}
    actual = set(model.parameters)
    missing, extra = sorted(expected - actual), sorted(actual - expected)
    if missing or extra:
        raise ValueError(
            f"{model.path}: damper click parameters do not match valves; "
            f"missing={missing}, unknown={extra}"
        )
    setting = DamperSetting(
        {name[len(_CLICK_PREFIX):]: value for name, value in model.parameters.items()}
    )
    try:
        setting.validate(table)
    except ValueError as error:
        raise ValueError(f"{model.path}: {error}") from error
    return CompiledDamper(table=table, setting=setting)


def _corner_arrays(model: Model) -> dict:
    """Load the one corner-table NPZ that ``model`` declares."""

    require_no_references(model)
    references = [item for item in model.data if item.role == CORNER_TABLE_ROLE]
    if len(references) != 1 or len(model.data) != 1 or references[0].format != "npz":
        raise ValueError(f"{model.path}: requires one corner-table NPZ declaration")
    reference = references[0]
    if not reference.sha256:
        raise ValueError(f"{model.path}: corner-table declaration requires sha256")
    return load_corner_arrays(reference.path, reference.sha256)


def compile_suspension_kinematics(model: Model) -> SuspensionKinematics:
    """Compile corner-table kinematics: geometry from the pinned NPZ plus static toe."""

    values = closed_parameters(model, _KINEMATICS_UNITS)
    arrays = _corner_arrays(model)
    try:
        return SuspensionKinematics(
            front=geometry_from_arrays(arrays, "front", static_toe=values["static_toe_front"]),
            rear=geometry_from_arrays(arrays, "rear", static_toe=values["static_toe_rear"]),
        )
    except ValueError as error:
        raise ValueError(f"{model.path}: {error}") from error


def compile_corner_springs(model: Model) -> CornerSprings:
    """Compile corner-table springs: the wheel-referred spring curve of each axle."""

    closed_parameters(model, {})
    arrays = _corner_arrays(model)
    try:
        return CornerSprings(
            front=spring_from_arrays(arrays, "front"),
            rear=spring_from_arrays(arrays, "rear"),
        )
    except ValueError as error:
        raise ValueError(f"{model.path}: {error}") from error


def compile_axle_arb(model: Model) -> AxleArbRates:
    """Compile an anti-roll bar given as one wheel rate per axle."""

    require_no_references(model)
    values = closed_parameters(model, _ARB_UNITS)
    require_nonnegative(model, values, *_ARB_UNITS)
    return AxleArbRates(front=values["arb_wheel_rate_front"], rear=values["arb_wheel_rate_rear"])


def compile_independent_corner(model: Model, children: Mapping[str, object]) -> CompiledSuspension:
    """Compile the independent-corner suspension from its five compiled children."""

    values = closed_parameters(model, _INDEPENDENT_UNITS)
    require_nonnegative(model, values, *_INDEPENDENT_UNITS)
    try:
        return build_independent_corner(
            kinematics=_child(model, children, "kinematics", SuspensionKinematics),
            springs=_child(model, children, "spring", CornerSprings),
            arb=_child(model, children, "arb", AxleArbRates),
            damper_front=_child(model, children, "damper-front", CompiledDamper),
            damper_rear=_child(model, children, "damper-rear", CompiledDamper),
            inertance_front=values["inertance_front"],
            inertance_rear=values["inertance_rear"],
        )
    except ValueError as error:
        raise ValueError(f"{model.path}: {error}") from error


def _child(model: Model, children: Mapping[str, object], role: str, expected: type):
    value = children[role]
    if not isinstance(value, expected):
        raise ValueError(
            f"{model.path}: child {role!r} must compile to {expected.__name__}, "
            f"received {type(value).__name__}"
        )
    return value


register_component(
    "suspension-kinematics",
    "corner-table",
    compile_function=compile_suspension_kinematics,
)
register_component(
    "spring-model",
    "corner-table",
    compile_function=compile_corner_springs,
)
register_component(
    "arb-model",
    "axle-rate",
    compile_function=compile_axle_arb,
)
register_component(
    "suspension-assembly",
    "independent-corner",
    capability=SUSPENSION_FORCES,
    compile_function=compile_independent_corner,
    nests_components=True,
    child_roles=("kinematics", "spring", "arb", "damper-front", "damper-rear"),
)
register_component(
    "damper-model",
    "table-front",
    capability=FRONT_DAMPER_FORCE,
    compile_function=compile_damper,
)
register_component(
    "damper-model",
    "table-rear",
    capability=REAR_DAMPER_FORCE,
    compile_function=compile_damper,
)
