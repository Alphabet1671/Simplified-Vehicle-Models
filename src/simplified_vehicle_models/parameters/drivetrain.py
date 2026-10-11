from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class DrivetrainParameters:
    """Store the electric drivetrain parameters."""

    motor_count: int
    gear_ratio: float
    peak_motor_torque: float
    power_limit: float
    efficiency: float
    maximum_motor_speed: float
