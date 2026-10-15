"""Convert a brake pedal request into four wheel brake torques.

The brake torque is limited by the **driver**, not by a fixed torque. A driver
who pushes harder makes more pressure and more torque, until the driver force
limit or the system pressure limit is reached.

The chain is:

1. The pedal force is the brake control times the driver force limit.
2. The pedal ratio curve turns the pedal force into a pushrod force. A real
   pedal box has a rising ratio, so the curve is a table.
3. The hydraulic ratio turns the pushrod force into the caliper clamp force.
   Its test value is 2 to 1, piston to pedal.
4. The line pressure is the clamp force over the caliper area. The system
   pressure limit bounds it.
5. The balance bar splits the pushrod force between the two circuits by the
   brake bias, and each circuit develops its own pressure.
6. Each caliper clamp force is its circuit force times the hydraulic ratio.
   The clamp force is **per caliper**, so it is not divided again.
7. Each wheel torque is ``2 * mu * clamp * effective_radius``. The factor of
   two covers the two friction faces of the disc.

The reported line pressure is the **front** circuit pressure, because the sheet
states the front pressure at one g. The model lumps the two master cylinders
into one hydraulic ratio, so it cannot reproduce the two circuit pressures at
once. The torque calibration removes that limit at the stated operating point.

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
    ``pedal_ratio`` is dimensionless and must be positive.

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


@dataclass(frozen=True, slots=True)
class BrakeResult:
    """Store one brake allocation.

    ``wheel_torques`` uses the order FL, FR, RL, RR and holds negative values.
    ``wheel_torques`` holds magnitudes and is convenient for a limit check.
    """

    wheel_torques: FloatArray
    wheel_torque_magnitudes: FloatArray
    pedal_force: float
    pedal_ratio: float
    pushrod_force: float
    clamp_force: float
    line_pressure: float
    is_pressure_limited: bool
    is_driver_limited: bool


def brake_torques(
    brake_control: float,
    maximum_pedal_force: float,
    pedal_curve: PedalRatioCurve,
    hydraulic_ratio: float,
    maximum_brake_pressure: float,
    caliper_area: float,
    brake_bias: float,
    pad_mu_front: float,
    pad_mu_rear: float,
    effective_radius_front: float,
    effective_radius_rear: float,
) -> BrakeResult:
    """Return the four brake torques of one pedal request.

    ``brake_control`` runs from zero to one and scales the pedal force.

    The driver force limit and the system pressure limit both bound the result.
    Report which one binds; never hide it.
    """

    if not np.isfinite(brake_control) or not 0.0 <= brake_control <= 1.0:
        raise ValueError("brake_control must be finite and within zero to one")
    for name, value in (
        ("maximum_pedal_force", maximum_pedal_force),
        ("maximum_brake_pressure", maximum_brake_pressure),
        ("caliper_area", caliper_area),
        ("effective_radius_front", effective_radius_front),
        ("effective_radius_rear", effective_radius_rear),
    ):
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    if not np.isfinite(hydraulic_ratio) or hydraulic_ratio <= 0.0:
        raise ValueError("hydraulic_ratio must be finite and positive")
    if not np.isfinite(brake_bias) or not 0.0 < brake_bias < 1.0:
        raise ValueError("brake_bias must be finite and within zero and one")
    for name, value in (("pad_mu_front", pad_mu_front), ("pad_mu_rear", pad_mu_rear)):
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")

    pedal_force = float(brake_control) * float(maximum_pedal_force)
    pedal_ratio = pedal_curve.ratio(pedal_force)
    pushrod_force = pedal_force * pedal_ratio

    # The balance bar splits the pushrod force between the two circuits.
    front_pushrod = pushrod_force * float(brake_bias)
    rear_pushrod = pushrod_force - front_pushrod
    front_clamp = front_pushrod * float(hydraulic_ratio)
    rear_clamp = rear_pushrod * float(hydraulic_ratio)
    clamp_force = front_clamp + rear_clamp

    line_pressure = front_clamp / float(caliper_area)
    is_pressure_limited = line_pressure > float(maximum_brake_pressure)
    if is_pressure_limited:
        scale = float(maximum_brake_pressure) / line_pressure
        front_clamp *= scale
        rear_clamp *= scale
        clamp_force *= scale
        line_pressure = float(maximum_brake_pressure)

    # The clamp force is per caliper, so it is not divided again.
    front_wheel = (
        2.0 * float(pad_mu_front) * front_clamp * float(effective_radius_front)
    )
    rear_wheel = (
        2.0 * float(pad_mu_rear) * rear_clamp * float(effective_radius_rear)
    )
    magnitudes = np.asarray(
        [front_wheel, front_wheel, rear_wheel, rear_wheel], dtype=np.float64
    )
    return BrakeResult(
        wheel_torques=-magnitudes,
        wheel_torque_magnitudes=magnitudes,
        pedal_force=pedal_force,
        pedal_ratio=pedal_ratio,
        pushrod_force=pushrod_force,
        clamp_force=clamp_force,
        line_pressure=line_pressure,
        is_pressure_limited=bool(is_pressure_limited),
        is_driver_limited=bool(brake_control >= 1.0 and not is_pressure_limited),
    )


def maximum_brake_torques(
    maximum_pedal_force: float,
    pedal_curve: PedalRatioCurve,
    hydraulic_ratio: float,
    maximum_brake_pressure: float,
    caliper_area: float,
    brake_bias: float,
    pad_mu_front: float,
    pad_mu_rear: float,
    effective_radius_front: float,
    effective_radius_rear: float,
) -> FloatArray:
    """Return the four brake torque magnitudes at full pedal.

    Use this for a bound or a diagnostic. The value is what the driver can
    command, not a fixed brake limit.
    """

    return brake_torques(
        1.0,
        maximum_pedal_force,
        pedal_curve,
        hydraulic_ratio,
        maximum_brake_pressure,
        caliper_area,
        brake_bias,
        pad_mu_front,
        pad_mu_rear,
        effective_radius_front,
        effective_radius_rear,
    ).wheel_torque_magnitudes
