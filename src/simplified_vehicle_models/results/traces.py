from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from simplified_vehicle_models.core.types import FloatArray


class LapMode(Enum):
    """Identify the boundary condition for a lap."""

    FLYING = "flying"
    STANDING_START = "standing_start"


@dataclass(frozen=True, slots=True)
class LapResult:
    """Store the calculated lap state and control traces."""

    speed: FloatArray
    elapsed_time: FloatArray
    longitudinal_acceleration: FloatArray
    lateral_acceleration: FloatArray
    throttle: FloatArray
    brake: FloatArray
    normal_loads: FloatArray
    longitudinal_forces: FloatArray
    lateral_forces: FloatArray
    combined_utilization: FloatArray
    drive_force_limit: FloatArray
    brake_force_limit: FloatArray
    force_balance_residual: FloatArray
    yaw_moment_residual: FloatArray
    active_longitudinal_constraint: tuple[str, ...]
    lap_time: float
    converged: bool
    iterations: int
    maximum_speed_change: float
