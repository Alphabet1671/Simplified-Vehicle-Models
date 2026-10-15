"""Solve the quasi-static vertical tire loads of a four-wheel vehicle.

Both the point-mass model and the planar model use this module. The solver is
algebraic, so it needs no state. It holds no roll, pitch, or heave state.

Every lateral quantity uses the SAE vehicle frame. A positive lateral
acceleration points to the right. A right turn therefore moves vertical load to
the left wheels.

The load transfer is quasi static. It uses a fixed lateral load transfer
distribution in place of a roll stiffness calculation. The distribution is the
share of the total lateral load transfer that reaches the front axle.
"""

from __future__ import annotations

from simplified_vehicle_models.core.conventions import GRAVITY
from simplified_vehicle_models.core.types import FloatArray

import numpy as np


def axle_loads(
    *,
    total_mass: float,
    wheelbase: float,
    cg_height: float,
    front_mass_fraction: float,
    downforce: float,
    aero_balance: float,
    longitudinal_acceleration: float,
) -> tuple[float, float]:
    """Return the front and rear axle loads.

    The static load, the aerodynamic load, and the longitudinal transfer add
    together. A positive longitudinal acceleration moves load to the rear.
    """

    front_axle = (
        total_mass * GRAVITY * front_mass_fraction
        + downforce * aero_balance
        - total_mass * longitudinal_acceleration * cg_height / wheelbase
    )
    rear_axle = total_mass * GRAVITY + downforce - front_axle
    return front_axle, rear_axle


def _saturate_axle_transfer(
    front_transfer: float,
    rear_transfer: float,
    front_axle: float,
    rear_axle: float,
) -> tuple[float, float]:
    """Move transfer that would lift an inside wheel onto the other axle."""

    def cap(axle: float) -> float:
        return max(0.5 * axle, 0.0)

    for _ in range(2):
        front_cap = cap(front_axle)
        excess = abs(front_transfer) - front_cap
        if excess > 0.0:
            sign = 1.0 if front_transfer >= 0.0 else -1.0
            front_transfer = sign * front_cap
            rear_transfer += sign * excess
        rear_cap = cap(rear_axle)
        excess = abs(rear_transfer) - rear_cap
        if excess > 0.0:
            sign = 1.0 if rear_transfer >= 0.0 else -1.0
            rear_transfer = sign * rear_cap
            front_transfer += sign * excess
    # If both axles are saturated, the leftover sits on the axle it was last
    # moved to and drives its inside wheel negative: a rollover.
    return front_transfer, rear_transfer


def lateral_transfer_loads(
    front_axle: float,
    rear_axle: float,
    *,
    total_mass: float,
    cg_height: float,
    track_width: float,
    lateral_transfer_distribution: float,
    lateral_acceleration: float,
    wheel_lift: bool = False,
) -> tuple[float, float, float, float]:
    """Split the two axle loads over the four wheels.

    Return the loads in the order FL, FR, RL, RR.
    A positive lateral acceleration is a right turn, so the left wheels gain
    load and the right wheels lose it.

    With ``wheel_lift=False`` (the historic behavior) the fixed distribution is
    applied at every lateral acceleration, so an inside wheel can be given a
    negative load.

    With ``wheel_lift=True`` an axle cannot transfer more than half its own
    load: once its inside wheel reaches zero load it lifts, and the remaining
    roll moment is reacted by the other axle, which is what a rigid chassis
    does once one axle's roll stiffness can no longer act. The total lateral
    transfer is unchanged. When both axles saturate the vehicle is on two
    wheels; the unreacted moment is left on the inside wheels as a negative
    load so that a caller's contact check flags the rollover.
    """

    total_transfer = (
        total_mass * lateral_acceleration * cg_height / track_width
    )
    front_transfer = total_transfer * lateral_transfer_distribution
    rear_transfer = total_transfer - front_transfer
    if wheel_lift:
        front_transfer, rear_transfer = _saturate_axle_transfer(
            front_transfer, rear_transfer, front_axle, rear_axle
        )
    return (
        front_axle * 0.5 + front_transfer,
        front_axle * 0.5 - front_transfer,
        rear_axle * 0.5 + rear_transfer,
        rear_axle * 0.5 - rear_transfer,
    )


def normal_loads(
    *,
    total_mass: float,
    wheelbase: float,
    track_width: float,
    cg_height: float,
    front_mass_fraction: float,
    lateral_transfer_distribution: float,
    downforce: float,
    aero_balance: float,
    longitudinal_acceleration: float,
    lateral_acceleration: float,
    wheel_lift: bool = False,
) -> FloatArray:
    """Return the quasi-static vertical load of each tire.

    ``wheel_lift`` selects the lifted-wheel transfer of
    ``lateral_transfer_loads``.
    Return the loads in the order FL, FR, RL, RR.
    The longitudinal acceleration is positive forward.
    The lateral acceleration is positive to the right.
    """

    front_axle, rear_axle = axle_loads(
        total_mass=total_mass,
        wheelbase=wheelbase,
        cg_height=cg_height,
        front_mass_fraction=front_mass_fraction,
        downforce=downforce,
        aero_balance=aero_balance,
        longitudinal_acceleration=longitudinal_acceleration,
    )
    return np.asarray(
        lateral_transfer_loads(
            front_axle,
            rear_axle,
            total_mass=total_mass,
            cg_height=cg_height,
            track_width=track_width,
            lateral_transfer_distribution=lateral_transfer_distribution,
            lateral_acceleration=lateral_acceleration,
            wheel_lift=wheel_lift,
        ),
        dtype=np.float64,
    )
