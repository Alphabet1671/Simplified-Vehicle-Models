from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from simplified_vehicle_models.parameters.aero import AeroParameters
from simplified_vehicle_models.parameters.drivetrain import DrivetrainParameters
from simplified_vehicle_models.parameters.tire import TireEnvelopeParameters

if TYPE_CHECKING:
    from simplified_vehicle_models.compiler.manifest import CompileManifest


@dataclass(frozen=True, slots=True)
class PointMassVehicleParameters:
    """Store the compiled point-mass vehicle parameters."""

    total_mass: float
    wheelbase: float
    track_width: float
    cg_height: float
    front_mass_fraction: float
    lateral_load_transfer_distribution: float
    brake_bias: float
    tire: TireEnvelopeParameters
    aero: AeroParameters
    drivetrain: DrivetrainParameters
    manifest: CompileManifest | None = field(
        default=None,
        compare=False,
        repr=False,
        hash=False,
    )
