"""Read a record and describe what an export would do. Cheap: no matplotlib, no ffmpeg."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

from tools.anim import export_animation as cli
from tools.anim.encode import EncoderUnavailable, find_ffmpeg
from tools.anim_gui.animation_gui_settings import ExportSettings


@dataclass(frozen=True, slots=True)
class PlanSummary:
    text: str                       # the same text as ``--dry-run`` prints
    frames: int
    first_time: float
    last_time: float
    visible: tuple[str, ...]
    hidden: tuple[str, ...]
    warnings: tuple[str, ...]
    model: str
    has_wheel_data: bool


def encoder_problem() -> str | None:
    """None when ffmpeg can be started, else the CLI's own message."""
    try:
        find_ffmpeg()
    except EncoderUnavailable as error:
        return str(error)
    return None


def namespace(settings: ExportSettings) -> argparse.Namespace:
    """Parse the settings with the CLI's own parser, so the GUI cannot drift from it."""
    return cli.build_parser().parse_args(settings.arguments(dry_run=True))


def inspect(settings: ExportSettings) -> PlanSummary:
    """Raise ``ValueError`` with the CLI's message when the record or the options are not usable."""
    settings.check()
    try:
        args = namespace(settings)
    except SystemExit as error:  # argparse prints to stderr and exits; turn it into a message
        raise ValueError("the options were rejected by the exporter") from error
    try:
        plan = cli.Plan(args)
    except cli.CliError as error:
        raise ValueError(str(error)) from error
    has_wheel_data = any(name.startswith(("wheel-", "tire-"))
                         for name in plan.source.channel_names)
    return PlanSummary(cli.describe(plan), plan.source.count, float(plan.record.time[0]),
                       float(plan.record.time[-1]), tuple(plan.visibility.visible), tuple(plan.visibility.hidden),
                       tuple(plan.warning_lines()), plan.source.model_name, has_wheel_data)


def record_span(path: Path) -> tuple[float, float]:
    """The first and last time of a record, for the range controls."""
    from simplified_vehicle_models.results.run_record import RunRecordError, load_run_record
    try:
        record = load_run_record(Path(path), verify_sources=False)
    except (RunRecordError, OSError, ValueError) as error:
        raise ValueError(f"{path} is not a valid run record: {error}") from error
    if len(record.time) == 0:
        raise ValueError(f"{path} is not a valid run record: it has no time samples")
    return float(record.time[0]), float(record.time[-1])
