"""Compile the vertical 14-DOF (``14dof-simple``) vehicle parameters.

The 14 mechanical degrees of freedom are the 7 planar ones plus heave, roll,
pitch, and four wheel-hub vertical motions. This module holds the parameters and
the compile function only. The equations of motion come in a later module.

The compiled value composes the planar 7-DOF vehicle (field ``planar``) and adds:

- the per-corner unsprung masses, the sprung-body radii of gyration, and the tire
  vertical damping (assembly parameters);
- the corner suspension, the front and rear dampers, and the transient tire
  coefficients (components);
- derived sprung-body values: ``sprung_mass``, ``sprung_cg_height``,
  ``sprung_front_fraction``, ``roll_inertia``, and ``pitch_inertia``.

Derivation. The planar vehicle gives the total mass, the total CG height and the
front static fraction. Each wheel assembly (hub, upright, wheel, tire, and the
unsprung share of the suspension) has its CG at the free tire radius, ``r_1`` of
the steady tire, so the sprung CG height and front fraction follow from moment
balances about the ground and the front axle::

    m_s       = m_total - 2 * (m_uf + m_ur)
    h_s       = (m_total * h - 2 * (m_uf + m_ur) * r_free) / m_s
    f_s_front = (m_total * f_front - 2 * m_uf) / m_s

``m_uf`` and ``m_ur`` are the unsprung mass of one front and one rear corner.

The tire vertical law requires the steady tire to have zero lateral loaded-radius
corrections, and its relaxation lengths to stay positive up to the largest
tire load that the model supports (``TIRE_OPERATING_MAXIMUM_LOAD``).
"""

from __future__ import annotations

import dataclasses
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from simplified_vehicle_models.compiler.adapters import ResolutionResult
from simplified_vehicle_models.compiler.capabilities import (
    FRONT_DAMPER_FORCE,
    REAR_DAMPER_FORCE,
    SUSPENSION_CORNER,
    TRANSIENT_COMBINED_SLIP,
)
from simplified_vehicle_models.formulations.planar_7dof import (
    PLANAR_7DOF_REQUESTED_CAPABILITIES,
    PLANAR_7DOF_VEHICLE_PARAMETER_UNITS,
    _capability_value,
    compile_planar_7dof_vehicle,
)
from simplified_vehicle_models.parameters.vehicle import PlanarVehicleParameters
from simplified_vehicle_models.suspension.components import CompiledDamper
from simplified_vehicle_models.suspension.corner import SuspensionModel
from simplified_vehicle_models.tires.unitire_transient import (
    TransientTireParameters,
    check_relaxation_lengths,
)
from simplified_vehicle_models.tires.vertical import require_exact_inverse_supported
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import require_positive, required_parameters

if TYPE_CHECKING:
    from simplified_vehicle_models.compiler.manifest import CompileManifest


VERTICAL_14DOF_REQUESTED_CAPABILITIES: tuple[str, ...] = (
    *PLANAR_7DOF_REQUESTED_CAPABILITIES,
    SUSPENSION_CORNER,
    TRANSIENT_COMBINED_SLIP,
    FRONT_DAMPER_FORCE,
    REAR_DAMPER_FORCE,
)

#: Assembly parameters that only the vertical model consumes.
VERTICAL_14DOF_ONLY_PARAMETER_UNITS: dict[str, str] = {
    "unsprung_mass_front": "kg",
    "unsprung_mass_rear": "kg",
    "roll_radius_of_gyration": "m",
    "pitch_radius_of_gyration": "m",
    "tire_vertical_damping": "N*s/m",
}

VERTICAL_14DOF_VEHICLE_PARAMETER_UNITS: dict[str, str] = {
    **PLANAR_7DOF_VEHICLE_PARAMETER_UNITS,
    **VERTICAL_14DOF_ONLY_PARAMETER_UNITS,
}

#: Largest tire load [N] over which the transient relaxation lengths must stay positive.
TIRE_OPERATING_MAXIMUM_LOAD = 3000.0


@dataclass(frozen=True, slots=True)
class VerticalVehicleParameters:
    """Store the compiled parameters of the vertical 14-DOF vehicle."""

    planar: PlanarVehicleParameters
    unsprung_mass_front: float
    unsprung_mass_rear: float
    roll_radius_of_gyration: float
    pitch_radius_of_gyration: float
    tire_vertical_damping: float
    suspension: SuspensionModel
    dampers: tuple[CompiledDamper, CompiledDamper]
    transient_tire: TransientTireParameters
    sprung_mass: float
    sprung_cg_height: float
    sprung_front_fraction: float
    roll_inertia: float
    pitch_inertia: float
    manifest: CompileManifest | None = field(
        default=None,
        compare=False,
        repr=False,
        hash=False,
    )

    @property
    def unsprung_cg_height(self) -> float:
        """Return the unsprung CG height: the free tire radius."""

        return self.planar.tire.r_1

    @property
    def total_unsprung_mass(self) -> float:
        """Return the mass of the four wheel assemblies."""

        return 2.0 * (self.unsprung_mass_front + self.unsprung_mass_rear)

    def with_manifest(self, manifest: CompileManifest) -> VerticalVehicleParameters:
        """Return a copy that holds the manifest here and in the composed planar model."""

        return dataclasses.replace(
            self,
            planar=dataclasses.replace(self.planar, manifest=manifest),
            manifest=manifest,
        )


def compile_vertical_14dof_vehicle(
    assembly: Model,
    resolutions: Mapping[str, ResolutionResult],
) -> VerticalVehicleParameters:
    """Compile one assembly for the vertical 14-DOF formulation."""

    planar = compile_planar_7dof_vehicle(assembly, resolutions)
    values, _ = required_parameters(assembly, VERTICAL_14DOF_ONLY_PARAMETER_UNITS)
    require_positive(assembly, values, *VERTICAL_14DOF_ONLY_PARAMETER_UNITS)

    suspension = _capability_value(assembly, resolutions, SUSPENSION_CORNER, SuspensionModel)
    front_damper = _capability_value(assembly, resolutions, FRONT_DAMPER_FORCE, CompiledDamper)
    rear_damper = _capability_value(assembly, resolutions, REAR_DAMPER_FORCE, CompiledDamper)
    transient = _capability_value(
        assembly, resolutions, TRANSIENT_COMBINED_SLIP, TransientTireParameters
    )

    try:
        require_exact_inverse_supported(planar.tire)
        check_relaxation_lengths(planar.tire, transient, TIRE_OPERATING_MAXIMUM_LOAD)
    except ValueError as error:
        raise ValueError(f"{assembly.path}: {error}") from error

    unsprung = 2.0 * (values["unsprung_mass_front"] + values["unsprung_mass_rear"])
    sprung_mass = planar.total_mass - unsprung
    if not sprung_mass > 0.0:
        raise ValueError(
            f"{assembly.path}: the four unsprung masses ({unsprung} kg) must be "
            f"smaller than total_mass ({planar.total_mass} kg)"
        )
    sprung_cg_height = (
        planar.total_mass * planar.cg_height - unsprung * planar.tire.r_1
    ) / sprung_mass
    sprung_front_fraction = (
        planar.total_mass * planar.front_mass_fraction
        - 2.0 * values["unsprung_mass_front"]
    ) / sprung_mass
    if not sprung_cg_height > 0.0:
        raise ValueError(
            f"{assembly.path}: the derived sprung CG height {sprung_cg_height} m "
            "must be positive"
        )
    if not 0.0 < sprung_front_fraction < 1.0:
        raise ValueError(
            f"{assembly.path}: the derived sprung front fraction "
            f"{sprung_front_fraction} must lie between zero and one"
        )
    return VerticalVehicleParameters(
        planar=planar,
        unsprung_mass_front=values["unsprung_mass_front"],
        unsprung_mass_rear=values["unsprung_mass_rear"],
        roll_radius_of_gyration=values["roll_radius_of_gyration"],
        pitch_radius_of_gyration=values["pitch_radius_of_gyration"],
        tire_vertical_damping=values["tire_vertical_damping"],
        suspension=suspension,
        dampers=(front_damper, rear_damper),
        transient_tire=transient,
        sprung_mass=sprung_mass,
        sprung_cg_height=sprung_cg_height,
        sprung_front_fraction=sprung_front_fraction,
        roll_inertia=sprung_mass * values["roll_radius_of_gyration"] ** 2,
        pitch_inertia=sprung_mass * values["pitch_radius_of_gyration"] ** 2,
    )
