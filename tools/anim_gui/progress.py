"""Read the exporter's output lines."""
from __future__ import annotations

import math
import re
from dataclasses import dataclass

_PROGRESS = re.compile(r"frame\s+(\d+)/(\d+)\s+([\d.]+|nan)\s+frames/s\s+eta\s+(\d+|nan)\s+s")
_ERROR = re.compile(r"^error:\s*(.*)$")

EXIT_MESSAGES = {
    2: "The exporter refused the request.",
    3: "No video encoder is available. Install ffmpeg, or export a PNG sequence.",
}


@dataclass(frozen=True, slots=True)
class Progress:
    done: int
    total: int
    frames_per_second: float
    eta_seconds: float          # nan when unknown

    @property
    def fraction(self) -> float:
        return self.done / self.total if self.total else 0.0


def parse_progress(line: str) -> Progress | None:
    match = _PROGRESS.search(line)
    if match is None:
        return None
    done, total = int(match.group(1)), int(match.group(2))
    rate = float(match.group(3)) if match.group(3) != "nan" else math.nan
    eta = float(match.group(4)) if match.group(4) != "nan" else math.nan
    return Progress(done, total, rate, eta)


def parse_error(line: str) -> str | None:
    match = _ERROR.match(line.strip())
    return match.group(1) if match else None


def describe_result(code: int, state: str, errors: list[str]) -> str:
    """One sentence for the status bar."""
    if state == "cancelled":
        return "Cancelled. The unfinished file was removed."
    if state == "timed_out":
        return "Stopped: the export ran longer than the time limit."
    if state == "failed_to_start":
        return "The exporter could not be started."
    if code == 0:
        return "Done."
    detail = errors[-1] if errors else ""
    return f"{EXIT_MESSAGES.get(code, f'The exporter failed with exit code {code}.')} {detail}".strip()


def format_eta(seconds: float) -> str:
    if not math.isfinite(seconds):
        return "estimating…"
    minutes, rest = divmod(int(seconds + 0.5), 60)
    return f"{minutes} min {rest:02d} s" if minutes else f"{rest} s"
