"""Qt-free MAT telemetry channel discovery and synchronized coordinate loading."""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Literal

import numpy as np


@dataclass(frozen=True, slots=True)
class ChannelInfo:
    name: str
    units: str
    samples: int
    start_time: float
    end_time: float


@dataclass(frozen=True, slots=True)
class ChannelSelection:
    x_channel: str = "VNorth"
    y_channel: str = "VEast"
    mode: Literal["velocity", "position", "gps"] = "velocity"
    x_scale: float = 1.0
    y_scale: float = 1.0
    x_allow_unknown_units: bool = False
    y_allow_unknown_units: bool = False


def _scipy():
    try:
        from scipy.io import loadmat
    except ImportError as exc:
        raise ValueError("SciPy is required to read MAT telemetry; install the forward extra") from exc
    return loadmat


def _metadata(path: str | Path, *, names=None):
    data = _scipy()(str(path), variable_names=names, squeeze_me=True, struct_as_record=False)
    return {key: value for key, value in data.items() if not key.startswith("__")}


def _field(record, name, channel):
    try:
        value = getattr(record, name) if hasattr(record, name) else record[name]
    except (AttributeError, KeyError, IndexError, TypeError, ValueError) as exc:
        raise ValueError(f"channel {channel!r} is missing required {name} field") from exc
    return value


def _channel(record, name, legacy_time=None):
    values = np.asarray(_field(record, "Value", name), dtype=float)
    if values.ndim != 1 or values.size < 2:
        raise ValueError(f"channel {name!r} Value must be a one-dimensional array with at least two samples")
    try:
        raw_clock = _field(record, "Time", name)
    except ValueError:
        if legacy_time is None:
            raise
        raw_clock = legacy_time
    clock = np.asarray(raw_clock, dtype=float)
    if clock.ndim != 1 or clock.size != values.size:
        raise ValueError(f"channel {name!r} Time must be one-dimensional and match Value")
    if not np.all(np.isfinite(clock)) or not np.all(np.isfinite(values)):
        raise ValueError(f"channel {name!r} contains nonfinite time or values")
    if np.any(np.diff(clock) <= 0):
        raise ValueError(f"channel {name!r} Time must be strictly increasing without duplicates")
    units = ""
    try:
        units = str(np.asarray(_field(record, "Units", name)).item()).strip()
    except ValueError:
        pass
    return clock, values, units


def list_channels(path: str | Path) -> list[ChannelInfo]:
    """List signal channels; metadata inspection deliberately loads MAT variables."""
    data = _metadata(path)
    result = []
    for name, record in data.items():
        try:
            fallback = None
            if "xtime" in data:
                fallback = _field(data["xtime"], "Value", "xtime")
            t, _, units = _channel(record, name, fallback)
        except ValueError:
            continue
        result.append(ChannelInfo(name, units, len(t), float(t[0]), float(t[-1])))
    return result


def _conversion(units: str, mode: str, scale: float, name: str, allow_unknown: bool = False) -> float:
    if not math.isfinite(scale) or scale == 0:
        raise ValueError(f"channel {name!r} scale must be finite and nonzero")
    unit = units.strip().lower().replace(" ", "").replace("°", "deg")
    if mode == "velocity":
        factors = {"m/s": 1.0, "mps": 1.0, "km/h": 1 / 3.6, "kph": 1 / 3.6, "mph": 0.44704, "ft/s": 0.3048}
        if unit in factors:
            return factors[unit] * scale
        if not unit and allow_unknown:
            return scale  # Explicit opt-in authorizes scaling unspecified units only.
        if not unit:
            raise ValueError(f"channel {name!r} has unknown velocity units; explicitly allow unknown units and provide a scale")
        raise ValueError(f"channel {name!r} has unsupported velocity units {units!r}; explicit scale cannot override a known unit label")
    if mode == "position":
        factors = {"m": 1.0, "meter": 1.0, "meters": 1.0, "metre": 1.0, "metres": 1.0, "km": 1000.0, "cm": .01, "mm": .001, "ft": .3048}
        if unit in factors:
            return factors[unit] * scale
        if not unit and allow_unknown:
            return scale
        raise ValueError(f"channel {name!r} has unknown position units; explicitly allow unknown units and provide a scale")
    if mode == "gps":
        if unit not in {"deg", "degree", "degrees", ""}:
            raise ValueError(f"GPS channel {name!r} must be expressed in degrees")
        if not unit and not allow_unknown:
            raise ValueError(f"GPS channel {name!r} has unknown units; explicitly allow unknown units")
        return scale
    raise ValueError(f"unsupported coordinate mode {mode!r}")


def load_coordinates(path: str | Path, selection: ChannelSelection, start_time: float, end_time: float):
    """Synchronize selected signals over their common interval; never extrapolate."""
    if selection.x_channel == selection.y_channel:
        raise ValueError("choose two different channels for north/X and east/Y; a track needs distinct axes")
    if selection.mode not in {"velocity", "position", "gps"}:
        raise ValueError(f"unsupported coordinate mode {selection.mode!r}")
    if not math.isfinite(start_time) or not math.isfinite(end_time) or start_time >= end_time:
        raise ValueError("start_time and end_time must be finite and start before end")
    data = _metadata(path, names=[selection.x_channel, selection.y_channel, "xtime"])
    try:
        xrec, yrec = data[selection.x_channel], data[selection.y_channel]
    except KeyError as exc:
        raise ValueError(f"selected channel {exc.args[0]!r} was not found in MAT file") from exc
    fallback = None
    if "xtime" in data:
        try:
            fallback = _field(data["xtime"], "Value", "xtime")
        except ValueError:
            pass
    tx, x, ux = _channel(xrec, selection.x_channel, fallback)
    ty, y, uy = _channel(yrec, selection.y_channel, fallback)
    fx = _conversion(ux, selection.mode, selection.x_scale, selection.x_channel, selection.x_allow_unknown_units)
    fy = _conversion(uy, selection.mode, selection.y_scale, selection.y_channel, selection.y_allow_unknown_units)
    lo, hi = max(start_time, tx[0], ty[0]), min(end_time, tx[-1], ty[-1])
    if not lo < hi:
        raise ValueError("selected channels and requested time range have no common interval")
    # Include exact clipped boundaries even when neither lands on an x sample.
    time = np.unique(np.concatenate(([lo], tx[(tx > lo) & (tx < hi)], ty[(ty > lo) & (ty < hi)], [hi])))
    if time.size < 2:
        raise ValueError("common interval contains fewer than two samples")
    xv = np.interp(time, tx, x) * fx
    yv = np.interp(time, ty, y) * fy
    if selection.mode == "velocity":
        north = np.concatenate(([0.0], np.cumsum((xv[1:] + xv[:-1]) * np.diff(time) / 2)))
        east = np.concatenate(([0.0], np.cumsum((yv[1:] + yv[:-1]) * np.diff(time) / 2)))
    elif selection.mode == "position":
        north, east = xv - xv[0], yv - yv[0]
    else:
        if np.any(np.abs(xv) > 90) or np.any(np.abs(yv) > 180):
            raise ValueError("GPS latitude must be within [-90, 90] and longitude within [-180, 180] degrees")
        lat0, lon0 = np.deg2rad(xv[0]), np.deg2rad(yv[0])
        north = 6371008.8 * np.deg2rad(xv - xv[0])
        east = 6371008.8 * np.cos(lat0) * np.deg2rad(yv - yv[0])
        if not np.any(np.hypot(north, east) > 1e-6):
            raise ValueError("GPS path is degenerate: all coordinates are identical")
    if not np.all(np.isfinite(north)) or not np.all(np.isfinite(east)):
        raise ValueError("converted coordinates contain nonfinite values")
    if not np.any(np.hypot(north - north[0], east - east[0]) > 1e-6):
        raise ValueError("coordinate path is degenerate: all coordinates are identical")
    return time, north, east
