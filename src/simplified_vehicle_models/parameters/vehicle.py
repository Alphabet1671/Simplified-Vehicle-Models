from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from simplified_vehicle_models.parameters.aero import AeroParameters
from simplified_vehicle_models.parameters.drivetrain import DrivetrainParameters
from simplified_vehicle_models.parameters.tire import (
    TireEnvelopeParameters,
    UniTireSimpleParameters,
)

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


@dataclass(frozen=True, slots=True)
class PlanarVehicleParameters:
    """Store the compiled parameters of the planar 7-DOF vehicle.

    The vehicle has seven mechanical degrees of freedom: longitudinal motion,
    lateral motion, yaw, and four wheel rotations. It has no heave, roll, or
    pitch state.

    The lateral load transfer is quasi static. It uses the yaw inertia and the
    lateral load transfer distribution, and it holds no roll state.

    The yaw inertia is derived from ``total_mass * yaw_radius_of_gyration**2``,
    so it follows the loaded mass. The property ``yaw_inertia`` returns it.
    """

    total_mass: float
    mass_without_driver: float
    driver_mass: float
    wheelbase: float
    track_width: float
    cg_height: float
    front_mass_fraction: float
    lateral_load_transfer_distribution: float
    yaw_radius_of_gyration: float
    wheel_inertia: float
    steer_ratio: float
    steering_maximum_angle: float
    steering_maximum_rate: float
    caster_angle: float
    kingpin_inclination_angle: float
    brake_bias: float
    maximum_brake_torque_front: float
    maximum_brake_torque_rear: float
    tire: UniTireSimpleParameters
    aero: AeroParameters
    drivetrain: DrivetrainParameters
    manifest: CompileManifest | None = field(
        default=None,
        compare=False,
        repr=False,
        hash=False,
    )

    @property
    def yaw_inertia(self) -> float:
        """Return the yaw inertia in kilogram metre squared.

        The value follows the loaded mass through the radius of gyration.
        """

        return self.total_mass * self.yaw_radius_of_gyration**2

    @property
    def front_axle_to_cg(self) -> float:
        """Return the distance from the front axle to the centre of gravity."""

        return self.wheelbase * (1.0 - self.front_mass_fraction)

    @property
    def rear_axle_to_cg(self) -> float:
        """Return the distance from the rear axle to the centre of gravity."""

        return self.wheelbase * self.front_mass_fraction

    @property
    def half_track(self) -> float:
        """Return the half track width."""

        return 0.5 * self.track_width
