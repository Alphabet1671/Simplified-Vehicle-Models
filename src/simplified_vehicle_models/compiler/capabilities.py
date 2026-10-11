"""Declare the capability names and their Protocols.

A capability is a contract between a component and a formulation.
A component provides a capability. A formulation requests a capability.
The compiler joins the two contracts through the registry and the adapters.

Task P0B declares the capabilities of the existing components only.
Task P0C and Task P7 add the tire and powertrain reductions.

A Protocol describes the compiled value of one capability.
Task P0C moves the behavior behind the Protocols.
A component is type-checked against the Protocol of the capability that it provides.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable


TIRE_FORCE_ENVELOPE = "tire-force-envelope"
AERO_LOADS = "aero-loads"
DRIVE_FORCE_LIMIT = "drive-force-limit"
BRAKE_LIMITS = "brake-limits"
TRANSIENT_COMBINED_SLIP = "transient-combined-slip"
SUSPENSION_LOAD_TRANSFER = "suspension-load-transfer"

CAPABILITY_NAMES = (
    TIRE_FORCE_ENVELOPE,
    AERO_LOADS,
    DRIVE_FORCE_LIMIT,
    BRAKE_LIMITS,
    TRANSIENT_COMBINED_SLIP,
    SUSPENSION_LOAD_TRANSFER,
)


@runtime_checkable
class TireForceEnvelope(Protocol):
    """Provide the steady force envelope of four tire contact patches."""

    longitudinal_mu_scale: float
    lateral_mu_scale: float
    load_sensitivity_slope: float
    load_sensitivity_intercept: float
    effective_radius: float


@runtime_checkable
class AeroLoads(Protocol):
    """Provide the downforce and the drag of the car body."""

    air_density: float
    downforce_area: float
    drag_area: float
    aero_balance: float


@runtime_checkable
class DriveForceLimit(Protocol):
    """Provide the longitudinal force limit of the drivetrain."""

    motor_count: int
    gear_ratio: float
    peak_motor_torque: float
    power_limit: float
    efficiency: float
    maximum_motor_speed: float
