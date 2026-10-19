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

from simplified_vehicle_models.core.types import FloatArray


TIRE_FORCE_ENVELOPE = "tire-force-envelope"
STEADY_COMBINED_SLIP = "steady-combined-slip"
AERO_LOADS = "aero-loads"
DRIVE_FORCE_LIMIT = "drive-force-limit"
BRAKE_LIMITS = "brake-limits"
TRANSIENT_COMBINED_SLIP = "transient-combined-slip"
SUSPENSION_LOAD_TRANSFER = "suspension-load-transfer"
ENERGY_BUFFER = "energy-buffer"
TORQUE_SOURCE = "torque-source"
TORQUE_TRANSMISSION = "torque-transmission"
DAMPER_FORCE = "damper-force"
FRONT_DAMPER_FORCE = "front-damper-force"
REAR_DAMPER_FORCE = "rear-damper-force"
SUSPENSION_FORCES = "suspension-forces"


@runtime_checkable
class TireForceEnvelope(Protocol):
    """Provide the steady force envelope of four tire contact patches."""

    longitudinal_mu_scale: float
    lateral_mu_scale: float
    load_sensitivity_slope: float
    load_sensitivity_intercept: float
    effective_radius: float


@runtime_checkable
class SteadyCombinedSlip(Protocol):
    """Provide a steady combined-slip tire law with load sensitivity.

    The value holds fitted coefficients and carries no state.
    Pass the value to the tire evaluation function of its subtype.
    The envelope capability cannot become this capability.
    A steady tire cannot invent slip or relaxation behavior.
    """

    f_z_rated: float
    k_x: tuple[float, ...]
    k_y: tuple[float, ...]
    mu_0_x: float
    mu_0_y: float


@runtime_checkable
class TransientCombinedSlip(Protocol):
    """Provide the scalar coefficients of a transient layer over a steady tire.

    The value holds only PROPOSED dimensionless coefficients and carries no
    state. Pass the value, a steady tire value, and a state to the evaluation
    function of its subtype. The state is held by the solver, not here.
    """

    longitudinal_length_scale: float
    lateral_length_scale: float
    trail_pole_ratio: float
    trail_lead_ratio: float


@runtime_checkable
class AeroLoads(Protocol):
    """Provide the downforce and the drag of the car body."""

    air_density: float
    downforce_area: float
    drag_area: float
    aero_balance: float


@runtime_checkable
class TorqueTransmission(Protocol):
    """Provide fixed-ratio speed and torque transmission."""

    driven_wheels: tuple[str, ...]

    def motor_speed(self, wheel_speed: float) -> float:
        """Convert one wheel speed to motor speed."""

    def wheel_torque(self, motor_torque: float) -> FloatArray:
        """Convert motor torque to per-wheel torque."""


@runtime_checkable
class DriveForceLimit(Protocol):
    """Provide the longitudinal force limit of the drivetrain."""

    motor_count: int
    gear_ratio: float
    peak_motor_torque: float
    power_limit: float
    efficiency: float
    maximum_motor_speed: float


@runtime_checkable
class DamperForceSource(Protocol):
    """Provide the force of one damper as a function of its velocity.

    Positive velocity is compression and positive force is compression force.
    DAMPER_FORCE is the generic contract. A formulation that needs the front
    and the rear damper requests FRONT_DAMPER_FORCE and REAR_DAMPER_FORCE, so
    that two providers of one capability cannot be confused.
    """

    def force(self, velocity):
        """Return the damper force, slope, and out-of-range flag at ``velocity`` (m/s)."""


@runtime_checkable
class SuspensionSource(Protocol):
    """Provide the suspension of a vertical vehicle model: kinematics plus a force network.

    ``kinematics`` holds the per-wheel geometry (motion ratio, roll centre, anti,
    toe, jounce steer, travel limits). ``network`` maps jounces, jounce rates and
    internal states to four wheel-referred corner forces. ``layout`` names the
    suspension layout. Zero jounce is the static equilibrium at the design ride
    height, named by ``kinematics.preload_reference``.
    """

    kinematics: object
    network: object
    layout: str
