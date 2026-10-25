"""Immutable exact station mapping and sampled views for periodic source fits."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from typing import Any

import numpy as np


@dataclass(frozen=True, slots=True)
class SourcePanel:
    view_start: float
    view_end: float
    source_start: float
    source_end: float
    side: str


@dataclass(frozen=True, slots=True)
class MappedWidths:
    view_station: np.ndarray
    source_station: np.ndarray
    half_width_left: np.ndarray
    half_width_right: np.ndarray
    provenance: str


class UnsupportedWidthSeam(ValueError):
    """Widths disagree at periodic seam; no approved discontinuity policy exists."""


class PeriodicCutMapping:
    """Strict descriptor-backed map from open view stations to a periodic source."""
    __slots__ = ("_descriptor", "source_identity", "source_start", "source_end", "period", "cut_source_station", "view_start", "view_end", "canonical_json", "sha256")
    _keys = frozenset(("mapping_version", "source_start", "source_end", "cut_source_station", "view_start", "view_end", "source_identity"))

    def __init__(self, descriptor: dict[str, Any]):
        if type(descriptor) is not dict or set(descriptor) != self._keys:
            raise ValueError("descriptor must contain exactly the versioned mapping keys")
        if descriptor["mapping_version"] != "periodic_cut_v1":
            raise ValueError("unsupported mapping_version")
        vals = {}
        def freeze_identity(value):
            if value is None or type(value) in (str, bool, int): return value
            if type(value) is float and math.isfinite(value): return value
            if type(value) is list: return tuple(freeze_identity(item) for item in value)
            if type(value) is dict and all(type(key) is str for key in value):
                return tuple((key, freeze_identity(value[key])) for key in sorted(value))
            raise ValueError("source_identity must contain only finite JSON scalars, lists, and string-keyed objects")
        identity = freeze_identity(descriptor["source_identity"])
        # Thaw immutable tuple encoding to canonical JSON object/array form.
        def thaw_identity(value):
            if isinstance(value, tuple):
                if value and all(isinstance(item, tuple) and len(item) == 2 and isinstance(item[0], str) for item in value):
                    return {key: thaw_identity(item) for key, item in value}
                return [thaw_identity(item) for item in value]
            return value
        identity_json = thaw_identity(identity)
        for name in self._keys - {"mapping_version", "source_identity"}:
            value = descriptor[name]
            if isinstance(value, (bool, np.bool_)):
                raise ValueError(f"{name} must be a finite real number")
            try: value = float(value)
            except (TypeError, ValueError, OverflowError) as exc: raise ValueError(f"{name} must be finite") from exc
            if not math.isfinite(value): raise ValueError(f"{name} must be finite")
            vals[name] = value
        start, end, cut = vals["source_start"], vals["source_end"], vals["cut_source_station"]
        period = end - start
        if not period > 0 or not start <= cut < end:
            raise ValueError("source period must be positive and cut inside [source_start, source_end)")
        if vals["view_start"] != 0. or vals["view_end"] != period:
            raise ValueError("view domain must be [0, period]")
        frozen = {"mapping_version": "periodic_cut_v1", **vals, "source_identity": identity_json}
        canonical = json.dumps(frozen, sort_keys=True, separators=(",", ":"), allow_nan=False)
        object.__setattr__(self, "_descriptor", ("periodic_cut_v1", vals["source_start"], vals["source_end"], vals["cut_source_station"], vals["view_start"], vals["view_end"], identity))
        object.__setattr__(self, "source_identity", identity)
        for key, value in vals.items(): object.__setattr__(self, key, value)
        object.__setattr__(self, "period", period)
        object.__setattr__(self, "canonical_json", canonical)
        object.__setattr__(self, "sha256", hashlib.sha256(canonical.encode()).hexdigest())

    @property
    def descriptor(self):
        version, start, end, cut, view_start, view_end, identity = self._descriptor
        def thaw(value):
            if isinstance(value, tuple):
                if value and all(isinstance(item, tuple) and len(item) == 2 and isinstance(item[0], str) for item in value):
                    return {key: thaw(item) for key, item in value}
                return [thaw(item) for item in value]
            return value
        return {"mapping_version": version, "source_start": start, "source_end": end,
                "cut_source_station": cut, "view_start": view_start, "view_end": view_end,
                "source_identity": thaw(identity)}

    def to_source(self, view_station: object, *, endpoint_side: str = "launch"):
        if endpoint_side not in ("launch", "terminal"):
            raise ValueError("endpoint_side must be launch or terminal")
        if isinstance(view_station, (bool, np.bool_)):
            raise ValueError("view station must be a finite real scalar or vector, not bool")
        raw = np.asarray(view_station)
        if raw.dtype.kind not in "iuf" or raw.ndim > 1:
            raise ValueError("view station must be a scalar or one-dimensional real numeric vector")
        raw = np.asarray(raw, dtype=float)
        scalar = raw.ndim == 0
        if not np.all(np.isfinite(raw)) or np.any(raw < 0.) or np.any(raw > self.period):
            raise ValueError(f"view station must lie in [0, {self.period}]")
        out = self.source_start + np.mod(self.cut_source_station + raw - self.source_start, self.period)
        if endpoint_side == "terminal": out = np.where(raw == self.period, self.source_end, out)
        if endpoint_side == "launch": out = np.where(raw == 0., self.cut_source_station, out)
        return float(out) if scalar else out

    def split_interval(self, view_start: float, view_end: float) -> tuple[SourcePanel, ...]:
        for value in (view_start, view_end):
            if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.integer, np.floating)):
                raise ValueError("interval endpoints must be finite scalar real numbers")
        a, b = float(view_start), float(view_end)
        if not (math.isfinite(a) and math.isfinite(b) and 0 <= a <= b <= self.period):
            raise ValueError("closed view interval must be bounded by [0, period]")
        if a == b: return ()
        # The single old source seam occurs where c+u reaches source_end.
        seam = self.source_end - self.cut_source_station
        bounds = [a] + ([seam] if a < seam < b else []) + [b]
        panels = []
        for lo, hi in zip(bounds, bounds[1:]):
            if hi <= lo: continue
            unwrapped_lo = self.cut_source_station + lo
            unwrapped_hi = self.cut_source_station + hi
            if unwrapped_hi <= self.source_end:
                slo, shi, side = unwrapped_lo, unwrapped_hi, "before_seam"
            else:
                slo, shi, side = unwrapped_lo - self.period, unwrapped_hi - self.period, "after_seam"
            panels.append(SourcePanel(lo, hi, slo, shi, side))
        return tuple(panels)

    def map_widths(self, source_station: object, half_width_left: object, half_width_right: object, *, provenance: str) -> MappedWidths:
        s, left, right = (np.array(x, dtype=float, copy=True) for x in (source_station, half_width_left, half_width_right))
        if s.ndim != 1 or len(s) < 2 or left.shape != s.shape or right.shape != s.shape or not np.all(np.isfinite(np.r_[s,left,right])) or np.any(np.diff(s) <= 0) or s[0] != self.source_start or s[-1] != self.source_end or np.any(left <= 0) or np.any(right <= 0):
            raise ValueError("width arrays must be finite positive paired samples on a full-period grid")
        if not np.isclose(left[0], left[-1], rtol=0., atol=1e-12) or not np.isclose(right[0], right[-1], rtol=0., atol=1e-12):
            raise UnsupportedWidthSeam("unsupported_width_seam: periodic endpoint widths differ")
        knots = np.r_[self.cut_source_station, s[(s > self.cut_source_station) & (s < self.source_end)], self.source_end, s[(s > self.source_start) & (s < self.cut_source_station)]+self.period]
        view = np.unique(np.r_[0., np.mod(knots-self.cut_source_station, self.period), self.period])
        src = np.asarray(self.to_source(view))
        src = np.where(view == self.period, self.source_end, src)
        l = np.interp(src, s, left); r = np.interp(src, s, right)
        for arr in (view, src, l, r): arr.setflags(write=False)
        return MappedWidths(view, src, l, r, provenance)


class MappedReferenceView:
    """Ordinary query adapter; deliberately not an exact ReferenceCurve."""
    __slots__ = ("source_reference", "mapping")
    def __init__(self, source_reference: object, mapping: PeriodicCutMapping):
        if not getattr(source_reference, "interpolation_periodic", False):
            raise ValueError("mapped view requires a periodic original reference fit")
        if (float(source_reference.start), float(source_reference.end)) != (mapping.source_start, mapping.source_end):
            raise ValueError("mapping source domain does not match reference")
        self.source_reference, self.mapping = source_reference, mapping

    @property
    def is_exact_reference_curve(self): return False

    def source_station(self, view_station: object): return self.mapping.to_source(view_station)

    def evaluate(self, view_station: object):
        q = np.asarray(view_station, dtype=float)
        source = np.asarray(self.mapping.to_source(q, endpoint_side="launch"))
        terminal = q == self.mapping.period
        source = np.where(terminal, self.mapping.source_end, source)
        return self.source_reference.evaluate(float(source) if q.ndim == 0 else source)
