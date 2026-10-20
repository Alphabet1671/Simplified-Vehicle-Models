"""Export settings and the command line they make. No Qt, no rendering."""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path

from tools.gui_common import REPOSITORY_ROOT
from tools.gui_common.files import check_name

EXPORT_SCRIPT = REPOSITORY_ROOT / "tools" / "anim" / "export_animation.py"
VIDEO_DIRECTORY = REPOSITORY_ROOT / "videos"
PRESETS = ("ultrafast", "superfast", "veryfast", "faster", "fast", "medium", "slow", "slower", "veryslow")
FORMATS = ("mp4", "png")


@dataclass(frozen=True, slots=True)
class ExportSettings:
    record: Path
    output: Path
    format: str = "mp4"
    frame_rate: float = 120.0
    width: int = 1920
    height: int = 1080
    crf: int = 18
    preset: str = "medium"
    time_range: tuple[float, float] | None = None
    stl_body: Path | None = None
    stl_wheel: Path | None = None
    body_length: float | None = None
    body_width: float | None = None
    body_height: float | None = None
    wheel_radius: float = 0.2
    wheel_width: float = 0.18
    transform_scale: float = 1.0
    transform_x: float = 0.0
    transform_z: float = 0.0
    force_top_view: bool = False
    quasi_static_attitude: bool = False

    def check(self) -> None:
        """Raise ``ValueError`` with a message the user can act on. The CLI checks again."""
        if self.format not in FORMATS:
            raise ValueError(f"unknown format {self.format!r}")
        if not self.record.is_file():
            raise ValueError(f"the record does not exist: {self.record}")
        for label, value in (("frame rate", self.frame_rate), ("wheel radius", self.wheel_radius),
                             ("wheel width", self.wheel_width), ("scale", self.transform_scale)):
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{label} must be a positive number")
        for label, value in (("width", self.width), ("height", self.height)):
            if value <= 0 or value % 2:
                raise ValueError(f"{label} must be a positive even number of pixels (video needs even sizes)")
        if not 0 <= self.crf <= 51:
            raise ValueError("quality (CRF) must be 0 to 51")
        if self.preset not in PRESETS:
            raise ValueError(f"unknown encoder preset {self.preset!r}")
        if self.time_range is not None:
            start, end = self.time_range
            if not (math.isfinite(start) and math.isfinite(end) and start < end):
                raise ValueError("the time range must have a start that is less than its end")
        for label, value in (("body length", self.body_length), ("body width", self.body_width),
                             ("body height", self.body_height)):
            if value is not None and not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{label} must be a positive number")
        if self.format == "mp4" and self.output.suffix.lower() != ".mp4":
            raise ValueError("the video file name must end in .mp4")

    def arguments(self, *, dry_run: bool = False, progress: bool = False) -> list[str]:
        """The arguments after the script name."""
        args = ["--record", str(self.record)]
        if not dry_run:
            args += ["--png-sequence" if self.format == "png" else "--output", str(self.output)]
        args += ["--frame-rate", repr(self.frame_rate), "--width", str(self.width), "--height", str(self.height),
                 "--crf", str(self.crf), "--preset", self.preset]
        if self.time_range is not None:
            args += ["--time-range", repr(self.time_range[0]), repr(self.time_range[1])]
        for flag, value in (("--stl-body", self.stl_body), ("--stl-wheel", self.stl_wheel),
                            ("--body-length", self.body_length), ("--body-width", self.body_width),
                            ("--body-height", self.body_height)):
            if value is not None:
                args += [flag, str(value)]
        args += ["--wheel-radius", repr(self.wheel_radius), "--wheel-width", repr(self.wheel_width),
                 "--transform-scale", repr(self.transform_scale), "--transform-x", repr(self.transform_x),
                 "--transform-z", repr(self.transform_z)]
        if self.force_top_view:
            args.append("--force-top-view")
        if self.quasi_static_attitude:
            args.append("--quasi-static-attitude")
        if dry_run:
            args.append("--dry-run")
        if progress:
            args.append("--progress")
        return args


def default_output(record: Path, fmt: str, directory: Path = VIDEO_DIRECTORY) -> Path:
    """A name under ``videos/`` built from the record's folder and file name."""
    parent = re.sub(r"[^A-Za-z0-9_.-]+", "_", Path(record).parent.name).strip("_.-") or "record"
    stem = f"{parent}_{Path(record).stem}".strip("_.-")
    check_name(stem, "file name")
    return directory / (f"{stem}_frames" if fmt == "png" else f"{stem}.mp4")


def preview_settings(settings: ExportSettings, time: float, folder: Path, *, width: int = 640,
                     height: int = 360, window: float = 0.05) -> ExportSettings:
    """Settings that render a few frames around ``time`` into ``folder`` as PNG (a still preview)."""
    from dataclasses import replace
    return replace(settings, format="png", output=Path(folder), width=width - width % 2, height=height - height % 2,
                   time_range=(time, time + window))
