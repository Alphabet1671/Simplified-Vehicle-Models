"""Convert a driver brake-pedal force into four wheel brake torques.

The input is the **physical pedal force in newtons** that the driver applies.
Any 0-100 % brake position belongs to the driver model (the open-loop driver
robot scales its brake request by ``maximum_pedal_force``); the vehicle's
hydraulic chain never sees a percentage.

The chain is:

1. The pedal ratio curve turns the pedal force into the total master-cylinder
   pushrod force: ``pushrod = pedal_force * pedal_ratio(pedal_force)``. A real
   pedal box may have a rising ratio, so the curve is a table; a constant
   curve holds one ratio.
2. The balance bar splits the pushrod force between the front and the rear
   master cylinder. ``brake_bias`` is signed: ``0`` is neutral (50/50),
   ``+1`` puts all force on the front master cylinder and ``-1`` all force on
   the rear one, so the front share is ``(1 + brake_bias) / 2``.
3. Each circuit's hydraulic ratio turns its master-cylinder force into the
   clamp force of **one** caliper on that axle (``hydraulic_ratio_front`` and
   ``hydraulic_ratio_rear``; caliper piston area over master-cylinder area).
4. Each circuit pressure is its clamp force over the caliper area. The system
   pressure limit bounds the higher of the two circuit pressures and scales
   both circuits together.
5. Each wheel torque is ``2 * pad_mu * clamp * effective_radius``. The factor
   of two covers the two friction faces of the disc.

The result holds **negative** torques, so a caller can add a drive allocation
and pass the sum to the vehicle evaluation.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.core.types import FloatArray


@dataclass(frozen=True, slots=True)
class PedalRatioCurve:
    """Store the pedal ratio against the pedal force.

    ``pedal_force`` uses newtons and must increase.
    ``pedal_ratio`` is dimensionless and must be positive: one newton at the
    pedal makes ``pedal_ratio`` newtons of total master-cylinder force.

    An empty curve keeps the ratio constant at ``constant_ratio``.
    """

    pedal_force: FloatArray
    pedal_ratio: FloatArray
    constant_ratio: float = 1.0

    def __post_init__(self) -> None:
        forces = np.asarray(self.pedal_force, dtype=np.float64)
        ratios = np.asarray(self.pedal_ratio, dtype=np.float64)
        if forces.shape != ratios.shape:
            raise ValueError("the pedal curve arrays must have the same shape")
        if forces.ndim != 1:
            raise ValueError("the pedal curve arrays must be one dimensional")
        if forces.size and np.any(np.diff(forces) <= 0.0):
            raise ValueError("the pedal curve forces must increase")
        if np.any(ratios <= 0.0) or not np.all(np.isfinite(ratios)):
            raise ValueError("the pedal ratios must be finite and positive")
        if not np.isfinite(self.constant_ratio) or self.constant_ratio <= 0.0:
            raise ValueError("the constant pedal ratio must be finite and positive")

    def ratio(self, pedal_force: float) -> float:
        """Return the pedal ratio at one pedal force."""

        if self.pedal_force.size == 0:
            return float(self.constant_ratio)
        return float(
            np.interp(
                float(pedal_force),
                self.pedal_force,
                self.pedal_ratio,
                left=float(self.pedal_ratio[0]),
                right=float(self.pedal_ratio[-1]),
            )
        )

    @property
    def is_constant(self) -> bool:
        """Return true when the curve holds no variation."""

        return self.pedal_force.size == 0


def front_master_cylinder_share(brake_bias: float) -> float:
    """Return the front master-cylinder share of the pushrod force.

    ``brake_bias`` runs from ``-1`` (all rear) through ``0`` (neutral) to
    ``+1`` (all front).
    """

    _check_bias(brake_bias)
    return 0.5 * (1.0 + float(brake_bias))


def front_brake_torque_share(
    brake_bias: float,
    hydraulic_ratio_front: float,
    hydraulic_ratio_rear: float,
    pad_mu_front: float,
    pad_mu_rear: float,
    effective_radius_front: float,
    effective_radius_rear: float,
) -> float:
    """Return the front axle's share of the total hydraulic brake torque.

    The share does not depend on the pedal force or the pedal ratio while the
    pressure limit is inactive, because both circuits scale with the same
    pushrod force (and the pressure limit scales them together).
    """

    _check_positive(
        hydraulic_ratio_front=hydraulic_ratio_front,
        hydraulic_ratio_rear=hydraulic_ratio_rear,
        pad_mu_front=pad_mu_front,
        pad_mu_rear=pad_mu_rear,
        effective_radius_front=effective_radius_front,
        effective_radius_rear=effective_radius_rear,
    )
    share = front_master_cylinder_share(brake_bias)
    front = share * hydraulic_ratio_front * pad_mu_front * effective_radius_front
    rear = (1.0 - share) * hydraulic_ratio_rear * pad_mu_rear * effective_radius_rear
    return float(front / (front + rear))


@dataclass(frozen=True, slots=True)
class BrakeResult:
    """Store one brake allocation.

    ``wheel_torques`` uses the order FL, FR, RL, RR and holds negative values.
    ``wheel_torque_magnitudes`` holds magnitudes and is convenient for a limit
    check.
    """

    wheel_torques: FloatArray
    wheel_torque_magnitudes: FloatArray
    pedal_force: float
    pedal_ratio: float
    pushrod_force: float
    front_master_cylinder_force: float
    rear_master_cylinder_force: float
    front_clamp_force: float
    rear_clamp_force: float
    front_line_pressure: float
    rear_line_pressure: float
    is_pressure_limited: bool
    is_driver_limited: bool

    @property
    def clamp_force(self) -> float:
        """Return the sum of one front and one rear caliper clamp force."""

        return self.front_clamp_force + self.rear_clamp_force

    @property
    def line_pressure(self) -> float:
        """Return the higher of the two circuit pressures."""

        return max(self.front_line_pressure, self.rear_line_pressure)


def brake_torques(
    pedal_force: float,
    maximum_pedal_force: float,
    pedal_curve: PedalRatioCurve,
    hydraulic_ratio_front: float,
    hydraulic_ratio_rear: float,
    maximum_brake_pressure: float,
    caliper_area: float,
    brake_bias: float,
    pad_mu_front: float,
    pad_mu_rear: float,
    effective_radius_front: float,
    effective_radius_rear: float,
) -> BrakeResult:
    """Return the four brake torques of one driver pedal force in newtons.

    ``pedal_force`` must lie in ``[0, maximum_pedal_force]``; the maximum is
    the driver's physical force limit. The driver force limit and the system
    pressure limit both bound the result, and the result reports which binds.
    """

    _check_positive(
        maximum_pedal_force=maximum_pedal_force,
        hydraulic_ratio_front=hydraulic_ratio_front,
        hydraulic_ratio_rear=hydraulic_ratio_rear,
        maximum_brake_pressure=maximum_brake_pressure,
        caliper_area=caliper_area,
        pad_mu_front=pad_mu_front,
        pad_mu_rear=pad_mu_rear,
        effective_radius_front=effective_radius_front,
        effective_radius_rear=effective_radius_rear,
    )
    if not np.isfinite(pedal_force) or not 0.0 <= pedal_force <= maximum_pedal_force:
        raise ValueError(
            "pedal_force must be finite and within zero and the maximum pedal force"
        )
    share = front_master_cylinder_share(brake_bias)

    pedal_force = float(pedal_force)
    pedal_ratio = pedal_curve.ratio(pedal_force)
    pushrod_force = pedal_force * pedal_ratio
    front_master = pushrod_force * share
    rear_master = pushrod_force - front_master
    front_clamp = front_master * float(hydraulic_ratio_front)
    rear_clamp = rear_master * float(hydraulic_ratio_rear)

    front_pressure = front_clamp / float(caliper_area)
    rear_pressure = rear_clamp / float(caliper_area)
    peak_pressure = max(front_pressure, rear_pressure)
    is_pressure_limited = peak_pressure > float(maximum_brake_pressure)
    if is_pressure_limited:
        scale = float(maximum_brake_pressure) / peak_pressure
        front_clamp *= scale
        rear_clamp *= scale
        front_pressure *= scale
        rear_pressure *= scale

    front_wheel = 2.0 * float(pad_mu_front) * front_clamp * float(effective_radius_front)
    rear_wheel = 2.0 * float(pad_mu_rear) * rear_clamp * float(effective_radius_rear)
    magnitudes = np.asarray(
        [front_wheel, front_wheel, rear_wheel, rear_wheel], dtype=np.float64
    )
    return BrakeResult(
        wheel_torques=-magnitudes,
        wheel_torque_magnitudes=magnitudes,
        pedal_force=pedal_force,
        pedal_ratio=pedal_ratio,
        pushrod_force=pushrod_force,
        front_master_cylinder_force=front_master,
        rear_master_cylinder_force=rear_master,
        front_clamp_force=front_clamp,
        rear_clamp_force=rear_clamp,
        front_line_pressure=front_pressure,
        rear_line_pressure=rear_pressure,
        is_pressure_limited=bool(is_pressure_limited),
        is_driver_limited=bool(
            pedal_force >= float(maximum_pedal_force) and not is_pressure_limited
        ),
    )


def maximum_brake_torques(
    maximum_pedal_force: float,
    pedal_curve: PedalRatioCurve,
    hydraulic_ratio_front: float,
    hydraulic_ratio_rear: float,
    maximum_brake_pressure: float,
    caliper_area: float,
    brake_bias: float,
    pad_mu_front: float,
    pad_mu_rear: float,
    effective_radius_front: float,
    effective_radius_rear: float,
) -> FloatArray:
    """Return the four brake torque magnitudes at the driver's force limit.

    Use this for a bound or a diagnostic. The value is what the driver can
    command, not a fixed brake limit.
    """

    return brake_torques(
        maximum_pedal_force,
        maximum_pedal_force,
        pedal_curve,
        hydraulic_ratio_front,
        hydraulic_ratio_rear,
        maximum_brake_pressure,
        caliper_area,
        brake_bias,
        pad_mu_front,
        pad_mu_rear,
        effective_radius_front,
        effective_radius_rear,
    ).wheel_torque_magnitudes


def _check_bias(brake_bias: float) -> None:
    if not np.isfinite(brake_bias) or not -1.0 <= brake_bias <= 1.0:
        raise ValueError("brake_bias must be finite and within -1 and +1")


def _check_positive(**values: float) -> None:
    for name, value in values.items():
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
