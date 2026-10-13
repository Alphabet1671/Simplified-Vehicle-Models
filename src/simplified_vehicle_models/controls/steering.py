"""Convert a steering-wheel angle into the four road-wheel angles.

The chain is:

1. The steering wheel turns and the rack moves.
2. The rack travel turns each front upright by the steer-arm geometry.
3. An Ackermann table corrects the outer wheel against the inner wheel.
4. The maximum road-wheel angle bounds the result.

Every angle in this module is a road-wheel angle in radians.
A positive angle turns the wheel to the right, which follows the SAE frame.

The Ackermann table holds the angle that the outer wheel gives up against the
inner wheel, indexed by the magnitude of the inner angle. A table of zeros
gives parallel steering, which is the current placeholder.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.core.types import FloatArray


#: Wheel order for every four-wheel array in this project.
WHEEL_ORDER = ("fl", "fr", "rl", "rr")


@dataclass(frozen=True, slots=True)
class AckermannTable:
    """Store the outer-wheel correction against the inner-wheel angle.

    ``inner_angle`` holds the magnitude of the inner road-wheel angle in
    radians and must increase. ``outer_offset`` holds the angle that the outer
    wheel gives up, in radians, and must be nonnegative.

    An empty table gives parallel steering. The table is symmetric, so it is
    indexed by the magnitude of the inner angle.
    """

    inner_angle: FloatArray
    outer_offset: FloatArray

    def __post_init__(self) -> None:
        angles = np.asarray(self.inner_angle, dtype=np.float64)
        offsets = np.asarray(self.outer_offset, dtype=np.float64)
        if angles.shape != offsets.shape:
            raise ValueError("the Ackermann table arrays must have the same shape")
        if angles.ndim != 1:
            raise ValueError("the Ackermann table arrays must be one dimensional")
        if angles.size and np.any(np.diff(angles) <= 0.0):
            raise ValueError("the Ackermann inner angles must increase")
        if np.any(offsets < 0.0):
            raise ValueError("the Ackermann outer offsets must be nonnegative")
        if not (np.all(np.isfinite(angles)) and np.all(np.isfinite(offsets))):
            raise ValueError("the Ackermann table must be finite")

    def offset(self, inner_angle: float) -> float:
        """Return the outer-wheel correction at one inner-wheel angle."""

        magnitude = abs(float(inner_angle))
        if self.inner_angle.size == 0:
            return 0.0
        return float(
            np.interp(
                magnitude,
                self.inner_angle,
                self.outer_offset,
                left=float(self.outer_offset[0]),
                right=float(self.outer_offset[-1]),
            )
        )

    @property
    def is_parallel(self) -> bool:
        """Return true when the table holds no correction."""

        return bool(np.all(self.outer_offset == 0.0))


def rack_travel(
    steering_wheel_angle: float,
    rack_travel_per_revolution: float,
) -> float:
    """Return the rack travel for one steering-wheel angle.

    ``steering_wheel_angle`` uses radians. ``rack_travel_per_revolution`` uses
    metres per steering-wheel revolution.
    """

    if not np.isfinite(steering_wheel_angle):
        raise ValueError("steering_wheel_angle must be finite")
    if not np.isfinite(rack_travel_per_revolution) or rack_travel_per_revolution <= 0.0:
        raise ValueError("rack_travel_per_revolution must be finite and positive")
    revolutions = steering_wheel_angle / (2.0 * np.pi)
    return float(revolutions * rack_travel_per_revolution)


@dataclass(frozen=True, slots=True)
class SteeringResult:
    """Store one steering result.

    ``wheel_angles`` uses the order FL, FR, RL, RR.
    ``reference_angle`` is the inner road-wheel angle before the Ackermann
    correction and before the maximum-angle bound.
    """

    wheel_angles: FloatArray
    rack_travel: float
    reference_angle: float
    is_limited: bool


def steering_angles(
    steering_wheel_angle: float,
    rack_travel_per_revolution: float,
    steer_arm_length: float,
    maximum_wheel_angle: float,
    ackermann: AckermannTable,
) -> SteeringResult:
    """Return the four road-wheel angles for one steering-wheel angle.

    The front wheels steer. The rear wheels stay straight.

    A positive steering-wheel angle turns the vehicle to the right, which
    follows the SAE frame. The right wheel is then the inner wheel, so it turns
    by the reference angle and the left wheel gives up the Ackermann offset.
    """

    for name, value in (
        ("steer_arm_length", steer_arm_length),
        ("maximum_wheel_angle", maximum_wheel_angle),
    ):
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")

    travelled = rack_travel(steering_wheel_angle, rack_travel_per_revolution)
    reference = float(np.arctan2(travelled, steer_arm_length))
    magnitude = abs(reference)
    is_limited = magnitude > maximum_wheel_angle
    bounded = float(np.clip(reference, -maximum_wheel_angle, maximum_wheel_angle))

    offset = ackermann.offset(bounded)
    if bounded > 0.0:
        # A right turn puts the inner wheel on the right.
        left = bounded - offset
        right = bounded
    elif bounded < 0.0:
        # A left turn puts the inner wheel on the left.
        left = bounded
        right = bounded + offset
    else:
        left = right = 0.0

    return SteeringResult(
        wheel_angles=np.asarray([left, right, 0.0, 0.0], dtype=np.float64),
        rack_travel=travelled,
        reference_angle=reference,
        is_limited=is_limited,
    )
