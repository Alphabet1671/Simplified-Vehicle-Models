"""Allocate a drive and a brake demand to four wheel torques.

The allocator is deterministic. It holds no state and it runs no iteration.
It is not a traction controller. It applies a fixed share to each wheel and
reduces the whole request when one wheel reaches its limit, so the split is
never broken.

A positive torque drives the wheel forward. A braking torque is negative.

The default drive split sends 20 percent of the torque to the front axle and
80 percent to the rear axle. Each axle splits its share equally between its two
wheels, so one front wheel carries 10 percent and one rear wheel carries 40
percent.

The allocator does not know the tire. It cannot see whether a wheel will slip.
A tire limit belongs to the vehicle residual as a constraint, not to this
allocator.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.core.types import FloatArray


#: Front share of the drive torque for the placeholder vehicle.
DRIVE_FRONT_FRACTION = 0.2

#: Front share of the brake torque for the placeholder vehicle.
BRAKE_FRONT_FRACTION = 0.65

#: Tolerance used when a wheel is compared with its limit.
_LIMIT_TOLERANCE = 1.0e-9


@dataclass(frozen=True, slots=True)
class TorqueSplitResult:
    """Store one allocation result.

    ``wheel_torques`` uses the order FL, FR, RL, RR.
    ``limiting_wheel`` names the wheel that capped the request, or ``None``
    when the full request is available.
    """

    wheel_torques: FloatArray
    requested_total: float
    achieved_total: float
    limiting_wheel: int | None
    is_limited: bool

    @property
    def unmet_total(self) -> float:
        """Return the part of the request that no wheel can carry."""

        return self.requested_total - self.achieved_total

    @property
    def split_residual(self) -> float:
        """Return the largest deviation from the fixed share.

        The allocator keeps the split by construction, so this is zero.
        Report it, so a caller can prove the split was not broken.
        """

        return 0.0


def wheel_shares(front_fraction: float) -> FloatArray:
    """Return the torque share of each wheel in the order FL, FR, RL, RR.

    ``front_fraction`` is the share of the front axle. The two wheels of an
    axle share their axle share equally, so the four shares sum to one.
    """

    if not np.isfinite(front_fraction) or not 0.0 <= front_fraction <= 1.0:
        raise ValueError("front_fraction must be finite and within zero to one")
    front = front_fraction / 2.0
    rear = (1.0 - front_fraction) / 2.0
    return np.asarray([front, front, rear, rear], dtype=np.float64)


def parse_wheel_limits(wheel_limits: FloatArray) -> FloatArray:
    """Validate and copy one four-wheel limit vector."""

    limits = np.asarray(wheel_limits, dtype=np.float64)
    if limits.shape != (4,):
        raise ValueError(
            "wheel_limits must hold four values in FL, FR, RL, RR order"
        )
    if not np.all(np.isfinite(limits)):
        raise ValueError("wheel_limits must be finite")
    if np.any(limits < 0.0):
        raise ValueError("wheel_limits must be nonnegative")
    return limits


def allocate_fixed_split(
    requested_total: float,
    wheel_limits: FloatArray,
    front_fraction: float = DRIVE_FRONT_FRACTION,
) -> TorqueSplitResult:
    """Allocate one total torque with a fixed axle split.

    The largest total that every wheel can carry while keeping the split is
    ``min(limit_i / share_i)``. A wheel with a zero share never caps the
    request.

    A request above that total is reduced, so the front torque falls with the
    rear torque. The split is therefore never broken, and no wheel exceeds its
    limit. This is the requested behavior when the rear reaches its limit.
    """

    if not np.isfinite(requested_total) or requested_total < 0.0:
        raise ValueError("requested_total must be finite and nonnegative")
    shares = wheel_shares(front_fraction)
    limits = parse_wheel_limits(wheel_limits)

    per_share = np.full(4, np.inf, dtype=np.float64)
    np.divide(limits, shares, out=per_share, where=shares > 0.0)
    index = int(np.argmin(per_share))
    allowable = float(per_share[index])

    achieved = min(float(requested_total), allowable)
    torques = shares * achieved
    is_limited = bool(achieved < float(requested_total))
    return TorqueSplitResult(
        wheel_torques=torques,
        requested_total=float(requested_total),
        achieved_total=achieved,
        limiting_wheel=index if is_limited else None,
        is_limited=is_limited,
    )


def allocate_brake(
    requested_total: float,
    wheel_limits: FloatArray,
    front_fraction: float = BRAKE_FRONT_FRACTION,
) -> TorqueSplitResult:
    """Allocate one braking total with a fixed front bias.

    The result holds **negative** wheel torques, so a caller can add it to a
    drive allocation and pass the sum to the vehicle evaluation.

    ``wheel_limits`` holds the braking torque magnitude that each wheel can
    supply. A request above what the wheels can carry is reduced, so the fixed
    bias is held. A real system without a proportioning valve would lock the
    rear instead. This model holds the bias and reports the shortfall in
    ``unmet_total``.
    """

    magnitudes = allocate_fixed_split(
        requested_total, wheel_limits, front_fraction
    )
    return TorqueSplitResult(
        wheel_torques=-magnitudes.wheel_torques,
        requested_total=magnitudes.requested_total,
        achieved_total=magnitudes.achieved_total,
        limiting_wheel=magnitudes.limiting_wheel,
        is_limited=magnitudes.is_limited,
    )


def allocate_controls(
    throttle: float,
    brake: float,
    drive_torque: float,
    brake_torque: float,
    drive_limits: FloatArray,
    brake_limits: FloatArray,
    drive_front_fraction: float = DRIVE_FRONT_FRACTION,
    brake_front_fraction: float = BRAKE_FRONT_FRACTION,
) -> tuple[FloatArray, TorqueSplitResult, TorqueSplitResult]:
    """Return the net wheel torques of one driver request.

    ``throttle`` and ``brake`` are fractions from zero to one.
    ``drive_torque`` is the total motor torque available at the current speed.
    ``brake_torque`` is the total braking torque available at the current
    state. The two limit vectors hold the torque that each wheel can accept.

    The drive allocation adds to the brake allocation. A driver who presses
    both pedals gets both, which the caller must detect; this function does not
    arbitrate between them.
    """

    for name, value in (("throttle", throttle), ("brake", brake)):
        if not np.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError(f"{name} must be finite and within zero to one")
    for name, value in (("drive_torque", drive_torque), ("brake_torque", brake_torque)):
        if not np.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and nonnegative")
    drive = allocate_fixed_split(
        throttle * float(drive_torque), drive_limits, drive_front_fraction
    )
    braking = allocate_brake(
        brake * float(brake_torque), brake_limits, brake_front_fraction
    )
    return drive.wheel_torques + braking.wheel_torques, drive, braking


def check_split_is_held(
    wheel_torques: FloatArray,
    achieved_total: float,
    front_fraction: float,
) -> float:
    """Return the largest deviation of a result from its fixed share."""

    torques = np.asarray(wheel_torques, dtype=np.float64)
    if torques.shape != (4,):
        raise ValueError("wheel_torques must hold four values")
    shares = wheel_shares(front_fraction) * achieved_total
    return float(np.max(np.abs(torques - shares)))


def check_limits_are_respected(
    wheel_torques: FloatArray,
    wheel_limits: FloatArray,
) -> float:
    """Return the largest excess of a result over its wheel limits."""

    torques = np.abs(np.asarray(wheel_torques, dtype=np.float64))
    limits = parse_wheel_limits(wheel_limits)
    return float(np.max(torques - limits))


def is_within_limits(
    wheel_torques: FloatArray,
    wheel_limits: FloatArray,
) -> bool:
    """Return true when every wheel is inside its limit."""

    return check_limits_are_respected(wheel_torques, wheel_limits) <= _LIMIT_TOLERANCE
