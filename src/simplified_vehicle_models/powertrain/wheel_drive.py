"""Bound the drive torque of each wheel by the motor and the battery.

The motor bounds each wheel through its torque, power, and speed limits. The
battery bounds the four motors together through the discharge power limit at
the present state of charge.

The battery bound scales every wheel torque by one factor, so it preserves the
axle split. The factor is exact while the efficiency is constant, because the
bus power is then proportional to the torque.

This module is not a traction controller. It does not look at the tire. A tire
limit belongs to the vehicle residual as a constraint.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.powertrain.energy_storage import (
    EnergyStorageParameters,
    StorageState,
    evaluate_storage,
)
from simplified_vehicle_models.powertrain.motor_envelope import (
    MotorEnvelopeParameters,
    total_bus_power,
    wheel_torque_limit,
)


#: Names the bound that set the limit.
MOTOR_CONSTRAINT = "motor"
BATTERY_CONSTRAINT = "battery"


@dataclass(frozen=True, slots=True)
class DriveLimitResult:
    """Store one drive limit result.

    ``wheel_torque_limits`` uses the order FL, FR, RL, RR and holds the torque
    that each wheel can accept, including the battery reduction.
    """

    wheel_torque_limits: FloatArray
    total_torque_limit: float
    motor_torque_limits: FloatArray
    bus_power_at_limit: float
    bus_power_limit: float
    battery_scale: float
    active_constraint: str

    @property
    def is_battery_limited(self) -> bool:
        """Return true when the battery reduced the motor limit."""

        return self.active_constraint == BATTERY_CONSTRAINT


def evaluate_drive_limits(
    motor: MotorEnvelopeParameters,
    wheel_speeds: FloatArray,
    bus_voltage: float,
    storage: EnergyStorageParameters | None = None,
    storage_state: StorageState | None = None,
    peak: bool = True,
) -> DriveLimitResult:
    """Return the drive torque that each wheel can accept.

    ``bus_voltage`` is the terminal voltage that the motors see.
    ``storage`` and ``storage_state`` add the battery bound. Omit them to model
    the motors alone.
    """

    speeds = np.asarray(wheel_speeds, dtype=np.float64)
    if speeds.shape != (4,):
        raise ValueError("wheel_speeds must hold four values in FL, FR, RL, RR order")
    if not np.all(np.isfinite(speeds)):
        raise ValueError("wheel_speeds must be finite")

    motor_limits = wheel_torque_limit(motor, speeds, bus_voltage, peak)
    if storage is None:
        return DriveLimitResult(
            wheel_torque_limits=motor_limits,
            total_torque_limit=float(np.sum(motor_limits)),
            motor_torque_limits=motor_limits,
            bus_power_at_limit=total_bus_power(motor, speeds, motor_limits),
            bus_power_limit=float("inf"),
            battery_scale=1.0,
            active_constraint=MOTOR_CONSTRAINT,
        )

    limits = evaluate_storage(storage, storage_state)
    power_limit = limits.discharge_power_limit
    power_at_full = total_bus_power(motor, speeds, motor_limits)
    if power_at_full <= power_limit:
        return DriveLimitResult(
            wheel_torque_limits=motor_limits,
            total_torque_limit=float(np.sum(motor_limits)),
            motor_torque_limits=motor_limits,
            bus_power_at_limit=power_at_full,
            bus_power_limit=power_limit,
            battery_scale=1.0,
            active_constraint=MOTOR_CONSTRAINT,
        )

    # Every motor motors here, so the bus power is proportional to the torque.
    scale = float(power_limit) / power_at_full
    scaled = motor_limits * scale
    return DriveLimitResult(
        wheel_torque_limits=scaled,
        total_torque_limit=float(np.sum(scaled)),
        motor_torque_limits=motor_limits,
        bus_power_at_limit=total_bus_power(motor, speeds, scaled),
        bus_power_limit=power_limit,
        battery_scale=scale,
        active_constraint=BATTERY_CONSTRAINT,
    )
