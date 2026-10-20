"""Build a closed track from MAT telemetry, for the track GUI.

This needs SciPy, which is an optional dependency (``forward`` extra).
"""
from __future__ import annotations

import math
import tempfile
from dataclasses import dataclass
from pathlib import Path

from simplified_vehicle_models.tracks.profile import load_track
from tools.gui_common.files import check_name
from tools.track_gui.export import SavedTrack, SaveRefused, _guard_targets, _install, check_model_id, check_title


def scipy_available() -> bool:
    try:
        import scipy.io  # noqa: F401
    except ImportError:
        return False
    return True


@dataclass(frozen=True, slots=True)
class ClosedSettings:
    telemetry: Path
    start_time: float
    end_time: float
    spacing: float = 0.5
    smoothing: float = 2.0
    maximum_radius: float = 300.0

    def check(self) -> None:
        values = (self.start_time, self.end_time, self.spacing, self.smoothing, self.maximum_radius)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("every number must be finite")
        if not self.start_time < self.end_time:
            raise ValueError("the lap start time must be before the end time")
        if self.spacing <= 0.0 or self.smoothing < 0.0 or self.maximum_radius <= 0.0:
            raise ValueError("spacing and maximum radius must be positive, and smoothing must not be negative")
        if not Path(self.telemetry).is_file():
            raise ValueError(f"no such telemetry file: {self.telemetry}")


def build_closed(settings: ClosedSettings) -> dict:
    """Return the geometry dict of ``build_closed_track.build_geometry``."""
    if not scipy_available():
        raise ValueError("SciPy is not installed; install it with: pip install -e .[forward]")
    from tools.tracks.build_closed_track import build_geometry

    settings.check()
    return build_geometry(Path(settings.telemetry).expanduser().resolve(), settings.start_time,
                          settings.end_time, settings.spacing, settings.smoothing)


def summarize(geometry: dict) -> dict[str, float]:
    """Return the report figures as numbers, for a table. ``build_closed_track.report`` prints them."""
    return {
        "length_m": float(geometry["length"]),
        "nodes": int(len(geometry["distance"])),
        "closure_gap_m": float(geometry["closure_gap"]),
        "winding_error_rad": abs(abs(float(geometry["winding"])) - 2.0 * math.pi),
        "winding_rad": float(geometry["winding"]),
        "minimum_radius_m": float(geometry["minimum_radius"]),
        "minimum_self_distance_m": float(geometry["minimum_self_distance"]),
        "peak_smoothing_move_m": float(geometry["peak_smoothing_correction"]),
    }


def save_closed(geometry: dict, settings: ClosedSettings, directory: Path, stem: str, model_id: str,
                title: str, *, allow_models: bool = False, overwrite: bool = False,
                overwrite_generated: bool = False) -> SavedTrack:
    """Write a closed track and read it back with ``load_track`` before it reaches ``directory``."""
    from tools.tracks.build_closed_track import write_csv, write_xml

    directory = Path(directory)
    check_name(stem, "file name")
    check_model_id(model_id)
    check_title(title)
    _guard_targets(directory, stem, allow_models=allow_models, overwrite=overwrite,
                   overwrite_generated=overwrite_generated)
    telemetry = Path(settings.telemetry).expanduser().resolve()
    with tempfile.TemporaryDirectory() as folder:
        scratch = Path(folder)
        write_csv(scratch / f"{stem}.csv", geometry)
        write_xml(scratch / f"{stem}.xml", f"{stem}.csv", geometry, telemetry,
                  settings.maximum_radius, settings.spacing, settings.smoothing,
                  model_id=model_id, title=title)
        profile = load_track(scratch / f"{stem}.xml")
        if not profile.is_closed:
            raise ValueError("the written track loaded as an open track")
        xml_target, csv_target = _install(scratch, directory, stem)
    return SavedTrack(xml_target, csv_target, float(profile.lap_length), True, int(profile.distance.size))
