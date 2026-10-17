"""N-valve damper force table: monotone velocity spline x multilinear clicks.

Sign convention: positive velocity is compression and compression force is
positive. The rebound half is stored signed (negative velocity, negative
force) on its own axis, so it is interpolated directly and never flipped.
"""
from __future__ import annotations

import hashlib
import itertools
import json
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Literal, Mapping

import numpy as np

from .errors import DomainError  # re-exported for callers
from .monotone_spline import MonotoneSpline

__all__ = [
    "DamperForce",
    "DamperSetting",
    "DamperTable",
    "DamperValve",
    "DomainError",
    "SCHEMA",
    "damper_force",
    "load_damper_npz",
]

SCHEMA = "damper-table-v1"
_HALVES = ("compression", "rebound")
_BANDS = ("low", "high", "full")


@dataclass(frozen=True)
class DamperValve:
    name: str
    half: Literal["compression", "rebound"]
    speed_band: Literal["low", "high", "full"]
    clicks: int

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name:
            raise ValueError("valve name must be a non-empty string")
        if self.half not in _HALVES:
            raise ValueError(f"valve {self.name!r}: half must be one of {_HALVES}")
        if self.speed_band not in _BANDS:
            raise ValueError(f"valve {self.name!r}: speed_band must be one of {_BANDS}")
        if int(self.clicks) != self.clicks or self.clicks < 1:
            raise ValueError(f"valve {self.name!r}: clicks must be a positive integer")
        object.__setattr__(self, "clicks", int(self.clicks))


def _frozen(array, name: str, ndim: int | None = None) -> np.ndarray:
    out = np.array(array, dtype=float)
    if ndim is not None and out.ndim != ndim:
        raise ValueError(f"{name} must be {ndim}-D")
    if not np.all(np.isfinite(out)):
        raise ValueError(f"{name} must be finite")
    out.setflags(write=False)
    return out


@dataclass(frozen=True, eq=False)
class DamperTable:
    velocity_axis_compression: np.ndarray  # m/s, strictly increasing from 0
    velocity_axis_rebound: np.ndarray      # m/s, strictly increasing to 0
    valves: tuple[DamperValve, ...]
    compression_force: np.ndarray          # (n_v,) + compression click counts, N
    rebound_force: np.ndarray              # (n_v,) + rebound click counts, N (<= 0)
    metadata: Mapping[str, str]
    _splines: dict = field(init=False, repr=False, compare=False, default_factory=dict)

    def __post_init__(self) -> None:
        vc = _frozen(self.velocity_axis_compression, "velocity_axis_compression", 1)
        vr = _frozen(self.velocity_axis_rebound, "velocity_axis_rebound", 1)
        if vc.size < 2 or vr.size < 2:
            raise ValueError("velocity axes need at least two knots")
        if vc[0] != 0.0 or np.any(np.diff(vc) <= 0.0):
            raise ValueError("compression velocity axis must start at 0 and increase")
        if vr[-1] != 0.0 or np.any(np.diff(vr) <= 0.0):
            raise ValueError("rebound velocity axis must end at 0 and increase")
        valves = tuple(self.valves)
        names = [v.name for v in valves]
        if len(set(names)) != len(names):
            raise ValueError("valve names must be unique")
        if not all(isinstance(v, DamperValve) for v in valves):
            raise ValueError("valves must be DamperValve instances")
        fc = _frozen(self.compression_force, "compression_force")
        fr = _frozen(self.rebound_force, "rebound_force")
        for half, force, axis in (("compression", fc, vc), ("rebound", fr, vr)):
            clicks = tuple(v.clicks for v in valves if v.half == half)
            if not clicks:
                raise ValueError(f"table has no {half} valve")
            if force.shape != (axis.size,) + clicks:
                raise ValueError(
                    f"{half}_force shape {force.shape} does not match "
                    f"velocity knots and valve clicks {(axis.size,) + clicks}"
                )
        if np.any(fc[0] != 0.0) or np.any(fr[-1] != 0.0):
            raise ValueError("force at zero velocity must be zero for every click")
        if np.any(fc < 0.0):
            raise ValueError("compression force must be non-negative")
        if np.any(fr > 0.0):
            raise ValueError("rebound force must be signed (non-positive)")
        meta = {str(k): str(v) for k, v in dict(self.metadata).items()}
        object.__setattr__(self, "velocity_axis_compression", vc)
        object.__setattr__(self, "velocity_axis_rebound", vr)
        object.__setattr__(self, "valves", valves)
        object.__setattr__(self, "compression_force", fc)
        object.__setattr__(self, "rebound_force", fr)
        object.__setattr__(self, "metadata", MappingProxyType(meta))

    def valves_of(self, half: str) -> tuple[DamperValve, ...]:
        return tuple(v for v in self.valves if v.half == half)

    def _spline(self, half: str, index: tuple[int, ...]) -> MonotoneSpline:
        key = (half, index)
        spline = self._splines.get(key)
        if spline is None:
            if half == "compression":
                spline = MonotoneSpline(
                    self.velocity_axis_compression,
                    self.compression_force[(slice(None),) + index],
                )
            else:
                spline = MonotoneSpline(
                    self.velocity_axis_rebound,
                    self.rebound_force[(slice(None),) + index],
                )
            self._splines[key] = spline
        return spline


@dataclass(frozen=True)
class DamperSetting:
    """Click position per valve name; may be fractional for optimisation."""

    clicks: Mapping[str, float]

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "clicks", MappingProxyType({str(k): float(v) for k, v in dict(self.clicks).items()})
        )

    def validate(self, table: DamperTable, allow_fractional: bool = False) -> None:
        expected = {v.name: v for v in table.valves}
        if set(self.clicks) != set(expected):
            raise ValueError(
                f"damper setting valves {sorted(self.clicks)} do not match table "
                f"valves {sorted(expected)}"
            )
        for name, value in self.clicks.items():
            if not np.isfinite(value):
                raise ValueError(f"click value for {name!r} must be finite")
            if not 0.0 <= value <= expected[name].clicks - 1:
                raise ValueError(
                    f"click value {value} for {name!r} outside 0..{expected[name].clicks - 1}"
                )
            if not allow_fractional and value != int(value):
                raise ValueError(f"click value {value} for {name!r} must be an integer")


@dataclass(frozen=True)
class DamperForce:
    force: np.ndarray         # N, positive in compression
    slope: np.ndarray         # dF/dv, N s/m
    out_of_range: np.ndarray  # bool, velocity beyond the table limit


def _corner_weights(values: list[float], counts: list[int]):
    """Yield (index tuple, weight) for the multilinear corners."""
    choices = []
    for value, count in zip(values, counts):
        lo = min(int(np.floor(value)), count - 1)
        frac = value - lo
        if frac <= 0.0 or lo + 1 >= count:
            choices.append(((lo, 1.0),))
        else:
            choices.append(((lo, 1.0 - frac), (lo + 1, frac)))
    for combo in itertools.product(*choices):
        weight = 1.0
        for _, w in combo:
            weight *= w
        yield tuple(i for i, _ in combo), weight


def _half_force(table: DamperTable, half: str, setting: DamperSetting, v: np.ndarray):
    valves = table.valves_of(half)
    axis = table.velocity_axis_compression if half == "compression" else table.velocity_axis_rebound
    lo, hi = axis[0], axis[-1]
    vq = np.clip(v, lo, hi)
    force = np.zeros_like(v)
    slope = np.zeros_like(v)
    values = [setting.clicks[x.name] for x in valves]
    counts = [x.clicks for x in valves]
    for index, weight in _corner_weights(values, counts):
        spline = table._spline(half, index)
        force += weight * spline(vq)
        slope += weight * spline.derivative(vq)
    # Linear extension with the end slope (vq == v inside the table).
    force = force + slope * (v - vq)
    return force, slope, v != vq


def damper_force(table: DamperTable, setting: DamperSetting, velocity) -> DamperForce:
    """Damper force for ``velocity`` (m/s, positive = compression)."""
    v = np.asarray(velocity, dtype=float)
    if not np.all(np.isfinite(v)):
        raise ValueError("damper velocity must be finite")
    setting.validate(table, allow_fractional=True)
    scalar = v.ndim == 0
    v = np.atleast_1d(v)
    force = np.zeros_like(v)
    slope = np.zeros_like(v)
    flag = np.zeros(v.shape, dtype=bool)
    for half, mask in (("compression", v >= 0.0), ("rebound", v < 0.0)):
        if np.any(mask):
            f, s, o = _half_force(table, half, setting, v[mask])
            force[mask], slope[mask], flag[mask] = f, s, o
    if scalar:
        return DamperForce(force[0], slope[0], flag[0])
    return DamperForce(force, slope, flag)


_REQUIRED_KEYS = (
    "schema", "velocity_compression", "velocity_rebound", "valve_names",
    "valve_halves", "valve_bands", "valve_clicks", "force_compression",
    "force_rebound", "source_sha256", "generator_version", "corrections",
)


def load_damper_npz(path, expected_sha256: str) -> DamperTable:
    """Load a ``damper-table-v1`` NPZ after checking the file's SHA-256."""
    path = Path(path)
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != str(expected_sha256).strip().lower():
        raise ValueError(f"damper table sha256 mismatch for {path}: got {digest}")
    with np.load(path, allow_pickle=False) as data:
        missing = [k for k in _REQUIRED_KEYS if k not in data.files]
        if missing:
            raise ValueError(f"damper table {path} is missing keys {missing}")
        arrays = {k: data[k] for k in _REQUIRED_KEYS}
    if str(arrays["schema"]) != SCHEMA:
        raise ValueError(f"unsupported damper table schema {arrays['schema']!s}")
    names = [str(x) for x in arrays["valve_names"]]
    halves = [str(x) for x in arrays["valve_halves"]]
    bands = [str(x) for x in arrays["valve_bands"]]
    clicks = arrays["valve_clicks"]
    if not (len(names) == len(halves) == len(bands) == clicks.size) or clicks.ndim != 1:
        raise ValueError("valve arrays must be 1-D with equal length")
    valves = tuple(DamperValve(n, h, b, int(c)) for n, h, b, c in zip(names, halves, bands, clicks))
    corrections = str(arrays["corrections"])
    try:
        json.loads(corrections)
    except json.JSONDecodeError as exc:
        raise ValueError("corrections must be a JSON string") from exc
    return DamperTable(
        velocity_axis_compression=arrays["velocity_compression"],
        velocity_axis_rebound=arrays["velocity_rebound"],
        valves=valves,
        compression_force=arrays["force_compression"],
        rebound_force=arrays["force_rebound"],
        metadata={
            "schema": SCHEMA,
            "source_sha256": str(arrays["source_sha256"]),
            "generator_version": str(arrays["generator_version"]),
            "corrections": corrections,
            "file_sha256": digest,
        },
    )
