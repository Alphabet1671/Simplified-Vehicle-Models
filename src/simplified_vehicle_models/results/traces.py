from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from simplified_vehicle_models.core.types import FloatArray


@dataclass(frozen=True, slots=True)
class SolverProgress:
    """Immutable observational progress update from a solver."""

    stage: str
    lap: int
    lap_count: int
    energy_iteration: int
    profile_iteration: int
    elapsed_seconds: float


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
    drive_cache_hits: int = 0
    drive_cache_entries: int = 0
    drive_cache_max_speed_displacement: float = 0.0
    drive_cache_max_soc_displacement: float = 0.0
    motor_forward_queries: int = 0
    motor_inverse_queries: int = 0
    motor_fallback_queries: int = 0


@dataclass(frozen=True, slots=True)
class EnergyReport:
    """Energy and outer-loop diagnostics for one lap."""
    state_trace: FloatArray
    bus_power: FloatArray
    shaft_power: FloatArray
    bus_voltage: FloatArray
    active_constraint: tuple[str, ...]
    consumed_energy: float
    recovered_energy: float
    net_energy: float
    final_state_of_charge: float
    minimum_voltage: float
    energy_converged: bool
    energy_iterations: int
    maximum_state_change: float
    # Maximum SOC mismatch between the profile input and its integrated trace.
    state_profile_residual: float = 0.0
    # True when the speed/force profile was intentionally reused rather than
    # re-solved at the carried SOC.
    profile_reused: bool = False


@dataclass(frozen=True, slots=True)
class EnergyLapResult:
    """Pair the legacy lap result with energy diagnostics."""
    lap: LapResult
    energy: EnergyReport

    def __getattr__(self, name: str):
        return getattr(self.lap, name)


@dataclass(frozen=True, slots=True)
class StintReport:
    """Per-lap endurance results with carried-over battery state."""
    laps: tuple[EnergyLapResult, ...]
    lap_times: FloatArray
    lap_energy: FloatArray
    final_state_of_charge: float
    total_consumed_energy: float
    total_recovered_energy: float
    total_net_energy: float
    usable_energy_lap_count: float = 0.0
