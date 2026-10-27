from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from simplified_vehicle_models.controls.brakes import (
    PedalRatioCurve,
    front_brake_torque_share,
    maximum_brake_torques,
)
from simplified_vehicle_models.controls.steering import AckermannTable
from simplified_vehicle_models.drivetrain.controller import PedalControllerParameters
from simplified_vehicle_models.parameters.aero import AeroParameters
from simplified_vehicle_models.parameters.drivetrain import DrivetrainParameters
from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.powertrain.energy_storage import (
    EnergyStorageParameters,
)
from simplified_vehicle_models.powertrain.motor_envelope import (
    MotorEnvelopeParameters,
)
from simplified_vehicle_models.parameters.tire import (
    TireEnvelopeParameters,
    UniTirePeakEnvelope,
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
    #: Front axle share of the hydraulic brake torque, derived from the
    #: hydraulic brake chain (signed balance bar, front/rear hydraulic ratios,
    #: pad mu and effective radii). Not the signed balance-bar setting.
    front_brake_share: float
    tire: TireEnvelopeParameters | UniTirePeakEnvelope
    aero: AeroParameters
    drivetrain: DrivetrainParameters
    #: Let an inside wheel lift at zero load and move the unreacted roll
    #: moment to the other axle (see ``quasi_static.lateral_transfer_loads``).
    #: False keeps the historic fixed-distribution transfer.
    wheel_lift: bool = False
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
    rack_travel_per_revolution: float
    steer_arm_length: float
    caster_angle: float
    kingpin_inclination_angle: float
    ackermann: AckermannTable
    #: Signed balance-bar setting: 0 neutral, +1 all force on the front
    #: master cylinder, -1 all force on the rear master cylinder.
    brake_bias: float
    maximum_pedal_force: float
    hydraulic_ratio_front: float
    hydraulic_ratio_rear: float
    maximum_brake_pressure: float
    caliper_area: float
    pad_mu_front: float
    pad_mu_rear: float
    brake_effective_radius_front: float
    brake_effective_radius_rear: float
    pedal_curve: PedalRatioCurve
    tire: UniTireSimpleParameters
    motor: MotorEnvelopeParameters
    storage: EnergyStorageParameters
    aero: AeroParameters
    drivetrain: DrivetrainParameters
    controller: PedalControllerParameters = field(
        default_factory=PedalControllerParameters
    )
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

    @property
    def front_brake_share(self) -> float:
        """Return the front axle share of the hydraulic brake torque."""

        return front_brake_torque_share(
            self.brake_bias,
            self.hydraulic_ratio_front,
            self.hydraulic_ratio_rear,
            self.pad_mu_front,
            self.pad_mu_rear,
            self.brake_effective_radius_front,
            self.brake_effective_radius_rear,
        )

    def maximum_brake_torques(self) -> FloatArray:
        """Return the brake torque magnitude at full pedal.

        This value follows the driver force limit and the pedal chain. It is
        not a fixed brake limit.
        """

        return maximum_brake_torques(
            maximum_pedal_force=self.maximum_pedal_force,
            pedal_curve=self.pedal_curve,
            hydraulic_ratio_front=self.hydraulic_ratio_front,
            hydraulic_ratio_rear=self.hydraulic_ratio_rear,
            maximum_brake_pressure=self.maximum_brake_pressure,
            caliper_area=self.caliper_area,
            brake_bias=self.brake_bias,
            pad_mu_front=self.pad_mu_front,
            pad_mu_rear=self.pad_mu_rear,
            effective_radius_front=self.brake_effective_radius_front,
            effective_radius_rear=self.brake_effective_radius_rear,
        )
