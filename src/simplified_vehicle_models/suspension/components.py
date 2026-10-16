"""Register the suspension model components: the table-based damper.

A damper XML names one damper-table NPZ (sha256 pinned) and one integer click
per valve. A 2-valve damper (Multimatic DSSV VC01) uses ``click_compression``
and ``click_rebound``. A damper with more valves uses ``click_<valve_name>``.
The compiled value is a ``CompiledDamper`` that evaluates ``force(velocity)``.
"""

from __future__ import annotations

from dataclasses import dataclass

from simplified_vehicle_models.compiler.capabilities import (
    FRONT_DAMPER_FORCE,
    REAR_DAMPER_FORCE,
)
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.suspension.damper_table import (
    DamperForce,
    DamperSetting,
    DamperTable,
    damper_force,
    load_damper_npz,
)
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import require_no_references

DAMPER_TABLE_ROLE = "damper-table"
_CLICK_PREFIX = "click_"


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
