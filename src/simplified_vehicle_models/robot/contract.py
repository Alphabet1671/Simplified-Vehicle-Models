"""Vendor-neutral robot execution contract; no physical transport implementation."""
from __future__ import annotations
from dataclasses import dataclass
from typing import Protocol
from simplified_vehicle_models.controls.maneuvers.types import RobotCommand


@dataclass(frozen=True, slots=True)
class RobotCapabilities:
    backend_id: str
    calibration_id: str | None
    command_modes: tuple[str, ...]
    telemetry_fields: tuple[str, ...]
    max_swa_rad: float
    max_swa_rate_rad_s: float
    max_accelerator: float
    max_brake: float
    tick_ns: int
    deadline_ns: int
    freshness_ns: int
    stop_modes: tuple[str, ...]
    commissioned: bool = False


@dataclass(frozen=True, slots=True)
class Telemetry:
    monotonic_ns: int
    swa_rad: float | None
    accelerator: float | None
    brake: float | None
    speed_mps: float | None
    x_m: float | None
    y_m: float | None
    heading_rad: float | None
    ay_mps2: float | None
    enabled: bool
    interlock_ok: bool
    sequence_id: int


class RobotBackend(Protocol):
    def capabilities(self) -> RobotCapabilities: ...
    def connect(self) -> None: ...
    def telemetry(self) -> Telemetry: ...
    def arm(self, definition_sha256: str) -> None: ...
    def send(self, command: RobotCommand, deadline_ns: int) -> None: ...
    def request_stop(self, reason: str) -> None: ...
    def disconnect(self) -> None: ...
