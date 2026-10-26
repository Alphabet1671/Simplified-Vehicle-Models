"""Outward-rounded bounds for a signed chart's bilinear table rectangle.

This bounds the stored chart interpolant only; it is not a physical or
whole-trajectory certificate.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numbers
import sys

import numpy as np

from simplified_vehicle_models.formulations.signed_capability import SignedTransientCapability


@dataclass(frozen=True, slots=True)
class ChartRectangleBounds:
    acceleration_min_lower: float
    acceleration_min_upper: float
    acceleration_max_lower: float
    acceleration_max_upper: float
    domain_speed_lower: float
    domain_speed_upper: float
    u_interval: tuple[float, float]
    curvature_interval: tuple[float, float]
    cell_count: int


def _out(lo: float, hi: float) -> tuple[float, float]:
    if not math.isfinite(lo) or not math.isfinite(hi):
        raise RuntimeError("outward interval arithmetic overflowed binary64")
    widened = math.nextafter(lo, -math.inf), math.nextafter(hi, math.inf)
    if not math.isfinite(widened[0]) or not math.isfinite(widened[1]):
        raise RuntimeError("outward interval endpoint overflowed binary64")
    return widened


def _add(a, b): return _out(a[0] + b[0], a[1] + b[1])
def _sub(a, b): return _out(a[0] - b[1], a[1] - b[0])
def _mul(a, b):
    p = (a[0]*b[0], a[0]*b[1], a[1]*b[0], a[1]*b[1])
    return _out(min(p), max(p))
def _div(a, b):
    if b[0] <= 0.0 <= b[1]:
        raise RuntimeError("interval division by zero-containing denominator")
    q = (a[0]/b[0], a[0]/b[1], a[1]/b[0], a[1]/b[1])
    return _out(min(q), max(q))


def _real_interval(value, name):
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError(f"{name} must be a two-element tuple or list")
    out = []
    for x in value:
        if not isinstance(x, numbers.Real) or isinstance(x, (bool, np.bool_)):
            raise ValueError(f"{name} endpoints must be real non-bool scalars")
        x = float(x)
        if not math.isfinite(x):
            raise ValueError(f"{name} endpoints must be finite")
        out.append(x)
    if out[0] > out[1]:
        raise ValueError(f"{name} must be ordered")
    return tuple(out)


def _cells(axis, interval):
    lo, hi = interval
    if lo == hi:
        if lo == axis[-1]: return [len(axis)-2]
        return [max(0, min(len(axis)-2, int(np.searchsorted(axis, lo, side="right")-1)))]
    return [i for i in range(len(axis)-1)
            if max(lo, float(axis[i])) < min(hi, float(axis[i+1]))]


def _lerp_cell(axis, i, interval):
    left, right = max(interval[0], float(axis[i])), min(interval[1], float(axis[i+1]))
    width = _sub((float(axis[i+1]),)*2, (float(axis[i]),)*2)
    def t(x): return _div(_sub((x,)*2, (float(axis[i]),)*2), width)
    return (t(left), t(right))


def _bilinear_cell(table, uaxis, kaxis, i, j, us, ks):
    f00, f10 = float(table[i,j]), float(table[i+1,j])
    f01, f11 = float(table[i,j+1]), float(table[i+1,j+1])
    one = (1.0, 1.0)
    values = []
    for t in us:
        for r in ks:
            omt, omr = _sub(one, t), _sub(one, r)
            terms = (_mul(_mul(omt, omr), (f00, f00)),
                     _mul(_mul(t, omr), (f10, f10)),
                     _mul(_mul(omt, r), (f01, f01)),
                     _mul(_mul(t, r), (f11, f11)))
            value = _add(_add(_add(terms[0], terms[1]), terms[2]), terms[3])
            values.append(value)
    return min(v[0] for v in values), max(v[1] for v in values)


def _linear_range(axis, values, interval):
    cells = _cells(axis, interval)
    lows, highs = [], []
    for i in cells:
        ts = _lerp_cell(axis, i, interval)
        for t in ts:
            value = _add(_mul(_sub((1.,1.), t), (float(values[i]),)*2),
                          _mul(t, (float(values[i+1]),)*2))
            lows.append(value[0]); highs.append(value[1])
    return min(lows), max(highs)


def bound_signed_chart_rectangle(surface, u_interval, curvature_interval, *, maximum_cells=4096):
    """Enclose signed bilinear table extrema on an in-support ``(u, k)`` rectangle.

    Input binary64 values are treated as exact reals. Every arithmetic node is
    widened with nextafter, assuming IEEE-754 binary64 round-to-nearest.
    """
    if type(surface) is not SignedTransientCapability:
        raise TypeError("surface must be exactly SignedTransientCapability")
    if sys.float_info.radix != 2 or sys.float_info.mant_dig != 53:
        raise RuntimeError("binary64 arithmetic is required")
    if not isinstance(maximum_cells, int) or isinstance(maximum_cells, bool) or maximum_cells <= 0:
        raise ValueError("maximum_cells must be a positive integer")
    ui = _real_interval(u_interval, "u_interval")
    ki = _real_interval(curvature_interval, "curvature_interval")
    if ui[0] < surface.u[0] or ui[1] > surface.u[-1]:
        raise ValueError("u interval outside chart support")
    if ki[0] < surface.curvature[0] or ki[1] > surface.curvature[-1]:
        raise ValueError("curvature interval outside chart support")
    uc, kc = _cells(surface.u, ui), _cells(surface.curvature, ki)
    count = len(uc)*len(kc)
    if count > maximum_cells:
        raise ValueError("rectangle cell budget exceeded")
    if count == 0:
        raise RuntimeError("empty chart cell intersection")
    ranges = []
    for table in (surface.acceleration_min, surface.acceleration_max):
        cell_ranges = []
        for i in uc:
            us = _lerp_cell(surface.u, i, ui)
            for j in kc:
                ks = _lerp_cell(surface.curvature, j, ki)
                cell_ranges.append(_bilinear_cell(table, surface.u, surface.curvature, i, j, us, ks))
        ranges.append((min(x[0] for x in cell_ranges), max(x[1] for x in cell_ranges)))
    domain = _linear_range(surface.curvature, surface.domain_speed_values, ki)
    return ChartRectangleBounds(*ranges[0], *ranges[1], *domain, ui, ki, count)
