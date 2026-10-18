"""Run settings mapping to the forward runner command line."""
from __future__ import annotations

from contextlib import redirect_stderr
from dataclasses import asdict, dataclass
import io
from pathlib import Path
from typing import Any

from tools.run import run_iem24_forward_driver as runner


@dataclass(frozen=True, slots=True)
class RunSettings:
    duration: float = 300.0
    timeout_seconds: float = 300.0
    maximum_speed: float = 60.0
    lateral_acceleration: float | None = None
    pedal_travel_time: float = 0.175
    preview_seconds: float = 0.15
    heading_gain: float = 1.5
    lateral_gain: float = 1.5
    maximum_step: float = 0.008
    rtol: float = 1e-5
    method: str = "BDF"
    spacing: float = 0.5
    planner: str = "live"
    record_precision: str = "float32"
    quasi_static_attitude: bool = False

    @classmethod
    def for_model(cls, model: str) -> "RunSettings":
        return cls(timeout_seconds=300.0 if model == "planar-7dof" else 3600.0)

    def to_json(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> "RunSettings":
        return cls(**data)


def runner_argv(settings: RunSettings, model: str, track: Path) -> list[str]:
    values = [
        ("--model", model), ("--track", str(track)), ("--planner", settings.planner),
        ("--duration", repr(settings.duration)), ("--timeout-seconds", repr(settings.timeout_seconds)),
        ("--maximum-speed", repr(settings.maximum_speed)),
        ("--pedal-travel-time", repr(settings.pedal_travel_time)),
        ("--preview-seconds", repr(settings.preview_seconds)),
        ("--heading-gain", repr(settings.heading_gain)), ("--lateral-gain", repr(settings.lateral_gain)),
        ("--maximum-step", repr(settings.maximum_step)), ("--rtol", repr(settings.rtol)),
        ("--method", settings.method), ("--spacing", repr(settings.spacing)),
        ("--record-precision", settings.record_precision),
    ]
    argv: list[str] = []
    for key, value in values:
        argv.extend((key, value))
    if settings.lateral_acceleration is not None:
        argv.extend(("--lateral-acceleration", repr(settings.lateral_acceleration)))
    argv.append("--record")
    if settings.quasi_static_attitude:
        argv.append("--quasi-static-attitude")
    return argv


def validate(settings: RunSettings, model: str, track: Path) -> None:
    if settings.quasi_static_attitude and model != "planar-7dof":
        raise ValueError("quasi-static attitude applies to planar-7dof only")
    stderr = io.StringIO()
    try:
        with redirect_stderr(stderr):
            runner.parse_args(runner_argv(settings, model, track))
    except SystemExit as error:
        message = stderr.getvalue().strip()
        raise ValueError(message or f"invalid runner arguments (exit {error.code})") from None


def _json_value(value: Any) -> Any:
    return value


def to_json(settings: RunSettings) -> dict[str, Any]:
    return asdict(settings)


def from_json(data: dict[str, Any]) -> RunSettings:
    return RunSettings(**data)

