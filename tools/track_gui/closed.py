"""Build a closed track from MAT telemetry, for the track GUI.

This needs SciPy, which is an optional dependency (``forward`` extra).
"""
from __future__ import annotations

import math
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from simplified_vehicle_models.tracks.profile import load_track
from tools.gui_common.files import check_name
from tools.track_gui.export import SavedTrack, _guard_targets, _install, check_model_id, check_title
from tools.track_gui.mat_channels import ChannelSelection, load_coordinates


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
    channel_selection: ChannelSelection | None = None
    multi_lap: bool = False
    return_radius: float = 12.0
    minimum_travel: float = 100.0
    minimum_duration: float = 15.0
    maximum_duration: float = 240.0
    deviation_limit: float = 8.0
    legacy_unknown_units: bool = True

    def check(self) -> None:
        values = (
            self.start_time,
            self.end_time,
            self.spacing,
            self.smoothing,
            self.maximum_radius,
            self.return_radius,
            self.minimum_travel,
            self.minimum_duration,
            self.maximum_duration,
            self.deviation_limit,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("every number must be finite")
        if not self.start_time < self.end_time:
            raise ValueError("the lap start time must be before the end time")
        if self.spacing <= 0 or self.smoothing < 0 or self.maximum_radius <= 0:
            raise ValueError(
                "spacing and maximum radius must be positive, and smoothing must not be negative"
            )
        detection_limits = (
            self.return_radius,
            self.minimum_travel,
            self.minimum_duration,
            self.maximum_duration,
            self.deviation_limit,
        )
        if min(detection_limits) <= 0 or self.minimum_duration >= self.maximum_duration:
            raise ValueError("lap detection limits must be positive, finite, and ordered")
        if not Path(self.telemetry).is_file():
            raise ValueError(f"no such telemetry file: {self.telemetry}")


def _provenance(settings, selection):
    fields = (
        "start_time",
        "end_time",
        "spacing",
        "smoothing",
        "maximum_radius",
        "multi_lap",
        "return_radius",
        "minimum_travel",
        "minimum_duration",
        "maximum_duration",
        "deviation_limit",
    )
    channel_selection = {
        "x_channel": selection.x_channel,
        "y_channel": selection.y_channel,
        "mode": selection.mode,
        "x_scale": selection.x_scale,
        "y_scale": selection.y_scale,
        "x_allow_unknown_units": selection.x_allow_unknown_units,
        "y_allow_unknown_units": selection.y_allow_unknown_units,
    }
    return {
        "channels": {"x": selection.x_channel, "y": selection.y_channel},
        "mode": selection.mode,
        "settings": {
            **{name: getattr(settings, name) for name in fields},
            "channel_selection": channel_selection,
        },
    }


def build_closed(settings: ClosedSettings) -> dict:
    """Build geometry from a synchronized selected signal pair."""
    settings.check()
    if not scipy_available():
        raise ValueError("SciPy is not installed; install it with: pip install -e .[forward]")

    from tools.tracks.build_closed_track import geometry_from_arrays

    path = Path(settings.telemetry).expanduser().resolve()
    selection = settings.channel_selection or ChannelSelection()
    if settings.channel_selection is None and settings.legacy_unknown_units:
        selection = ChannelSelection(
            x_allow_unknown_units=True,
            y_allow_unknown_units=True,
        )
    try:
        time, north, east = load_coordinates(
            path, selection, settings.start_time, settings.end_time
        )
    except ValueError as exc:
        if "no common interval" in str(exc):
            raise ValueError(
                "selected window contains too few samples or no common interval: "
                f"{exc}"
            ) from exc
        raise

    if settings.multi_lap:
        from tools.track_gui.lap_average import average_laps

        north_nodes, east_nodes, diagnostics, overlaps = average_laps(
            time,
            north,
            east,
            spacing=settings.spacing,
            smoothing=settings.smoothing,
            return_radius=settings.return_radius,
            minimum_travel=settings.minimum_travel,
            minimum_duration=settings.minimum_duration,
            maximum_duration=settings.maximum_duration,
            deviation_limit=settings.deviation_limit,
        )
        if np.hypot(
            north_nodes[0] - north_nodes[-1], east_nodes[0] - east_nodes[-1]
        ) > 1e-9:
            north_nodes = np.append(north_nodes, north_nodes[0])
            east_nodes = np.append(east_nodes, east_nodes[0])
        geometry = geometry_from_arrays(
            north_nodes, east_nodes, settings.spacing, 0.0
        )
        provenance = _provenance(settings, selection)
        provenance["lap_diagnostics"] = diagnostics
        geometry.update(
            {
                "lap_diagnostics": diagnostics,
                "overlap_north": overlaps[:, :, 0],
                "overlap_east": overlaps[:, :, 1],
                "provenance": provenance,
            }
        )
        return geometry

    geometry = geometry_from_arrays(
        north, east, settings.spacing, settings.smoothing, stamps=time
    )
    geometry["provenance"] = _provenance(settings, selection)
    return geometry


def summarize(geometry: dict) -> dict[str, float]:
    return {
        "length_m": float(geometry["length"]),
        "nodes": int(len(geometry["distance"])),
        "closure_gap_m": float(geometry["closure_gap"]),
        "winding_error_rad": abs(
            abs(float(geometry["winding"])) - 2 * math.pi
        ),
        "winding_rad": float(geometry["winding"]),
        "minimum_radius_m": float(geometry["minimum_radius"]),
        "minimum_self_distance_m": float(geometry["minimum_self_distance"]),
        "peak_smoothing_move_m": float(geometry["peak_smoothing_correction"]),
    }


def save_closed(geometry: dict, settings: ClosedSettings, directory: Path, stem: str, model_id: str,
                title: str, *, allow_models: bool=False, overwrite: bool=False, overwrite_generated: bool=False) -> SavedTrack:
    """Write a closed track and read it back before installation."""
    from tools.tracks.build_closed_track import write_csv, write_xml

    directory = Path(directory)
    check_name(stem, "file name")
    check_model_id(model_id)
    check_title(title)
    _guard_targets(
        directory,
        stem,
        allow_models=allow_models,
        overwrite=overwrite,
        overwrite_generated=overwrite_generated,
    )
    telemetry = Path(settings.telemetry).expanduser().resolve()
    with tempfile.TemporaryDirectory() as folder:
        scratch = Path(folder)
        write_csv(scratch / f"{stem}.csv", geometry)
        write_xml(
            scratch / f"{stem}.xml",
            f"{stem}.csv",
            geometry,
            telemetry,
            settings.maximum_radius,
            settings.spacing,
            settings.smoothing,
            model_id=model_id,
            title=title,
        )
        profile = load_track(scratch / f"{stem}.xml")
        if not profile.is_closed:
            raise ValueError("the written track loaded as an open track")
        xml_target, csv_target = _install(scratch, directory, stem)
    return SavedTrack(
        xml_target,
        csv_target,
        float(profile.lap_length),
        True,
        int(profile.distance.size),
    )
