from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class TireEnvelopeParameters:
    """Store the parameters for a tire force envelope."""

    longitudinal_mu_scale: float
    lateral_mu_scale: float
    load_sensitivity_slope: float
    load_sensitivity_intercept: float
    effective_radius: float


@dataclass(frozen=True, slots=True)
class AeroParameters:
    """Store the aerodynamic parameters."""

    air_density: float
    downforce_area: float
    drag_area: float
    aero_balance: float


@dataclass(frozen=True, slots=True)
class DrivetrainParameters:
    """Store the electric drivetrain parameters."""

    motor_count: int
    gear_ratio: float
    peak_motor_torque: float
    power_limit: float
    efficiency: float
    maximum_motor_speed: float


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
