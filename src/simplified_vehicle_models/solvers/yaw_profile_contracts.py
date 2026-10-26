"""Shared contracts for the coupled yaw-profile solver."""
from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping


@dataclass(frozen=True, slots=True)
class CoupledProfileSettings:
    q_seed_count: int = 33
    max_refinement_rounds: int = 8
    max_candidates_per_station: int = 256
    max_total_nodes: int = 4_000_000
    max_total_edges: int = 16_000_000
    max_seam_anchors: int = 32
    max_wall_seconds: float = 120.0
    interval_q_tolerance: float = 1e-8
    acceleration_tolerance: float = 1e-7
    yaw_moment_tolerance: float = 1e-7
    refinement_time_relative_tolerance: float = 1e-3
    refinement_speed_tolerance: float = 2e-2
    # Appended to preserve all previously positional constructor arguments.
    initial_optional_seed_cap: int = 32
    refinement_reserve_per_station: int = 32

    def __post_init__(self):
        integer_fields = ("q_seed_count", "initial_optional_seed_cap", "refinement_reserve_per_station",
                          "max_refinement_rounds", "max_candidates_per_station", "max_total_nodes",
                          "max_total_edges", "max_seam_anchors")
        for name in integer_fields:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"{name} must be an integer")
            minimum = 0 if name in ("initial_optional_seed_cap", "refinement_reserve_per_station") else (2 if name == "q_seed_count" else 1)
            if value < minimum:
                raise ValueError(f"{name} must be at least {minimum}")
        for name in ("max_wall_seconds", "interval_q_tolerance", "acceleration_tolerance",
                     "yaw_moment_tolerance", "refinement_time_relative_tolerance",
                     "refinement_speed_tolerance"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")


class YawProfileSolveError(ValueError):
    STATUSES = frozenset({"no_path_at_resolution", "budget_exceeded", "interval_oracle_unresolved",
                          "strict_validation_failed", "invalid_boundary"})

    def __init__(self, status: str, message: str, diagnostics: Mapping[str, object]):
        if status not in self.STATUSES:
            raise ValueError(f"unknown yaw profile solve status: {status}")
        super().__init__(message)
        self.status = status
        self.diagnostics = MappingProxyType(dict(diagnostics))


@dataclass(frozen=True, slots=True)
class ProfileValidation:
    valid: bool
    maximum_acceleration_domain_violation: float
    maximum_yaw_moment_violation: float
    minimum_yaw_margin: float | None
    invalid_segment_indices: tuple[int, ...]
    invalid_support_count: int
    seam_speed_error: float
    recomputed_time: float | None
    dense_sample_valid: bool | None
