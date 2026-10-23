"""Exact-rational interval bounds for stored reference B-spline derivatives.

This primitive encloses only the analytic coordinate splines and their first
three derivatives; it is not a trajectory/vehicle feasibility certificate.
"""
from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
import math
from numbers import Real
from typing import TypeAlias

import numpy as np
from scipy.interpolate import BSpline

from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve

Interval: TypeAlias = tuple[Fraction, Fraction]


@dataclass(frozen=True)
class ReferenceDerivativeBounds:
    north_derivatives: tuple[Interval, Interval, Interval, Interval]
    east_derivatives: tuple[Interval, Interval, Interval, Interval]
    station_interval: tuple[float, float]
    span_count: int
    exact_arithmetic: bool = True
    whole_trajectory_certified: bool = False


def _fraction(value: object) -> Fraction:
    return Fraction.from_float(float(value))


def _validate_spline(spline: object, start: float, end: float) -> tuple[list[Fraction], list[Fraction], int]:
    if type(spline) is not BSpline:
        raise ValueError("unsupported spline implementation")
    t, c, p = np.asarray(spline.t), np.asarray(spline.c), spline.k
    if t.ndim != 1 or c.ndim != 1 or not isinstance(p, int) or p not in (3, 5):
        raise ValueError("unsupported spline layout or degree")
    if len(t) < 2*p+2 or len(c) != len(t)-p-1:
        raise ValueError("unsupported spline coefficient shape")
    if t.dtype.kind != "f" or c.dtype.kind != "f" or t.dtype.itemsize != 8 or c.dtype.itemsize != 8:
        raise ValueError("spline knots and coefficients must be binary floats")
    if not np.all(np.isfinite(t)) or not np.all(np.isfinite(c)) or np.any(np.diff(t) < 0):
        raise ValueError("nonfinite or decreasing spline data")
    if _fraction(t[p]) != _fraction(start) or _fraction(t[len(c)]) != _fraction(end):
        raise ValueError("spline support differs from reference domain")
    # Interior multiplicity p+1 or greater can make derivatives undefined.
    for v in set(t[p+1:len(c)]):
        if int(np.count_nonzero(t == v)) > p-2:
            raise ValueError("unsupported repeated interior knot multiplicity")
    return [_fraction(v) for v in t], [_fraction(v) for v in c], p


def _derivatives(t: list[Fraction], c: list[Fraction], p: int):
    result = []
    for _ in range(4):
        result.append((t, c, p))
        if p == 0:
            break
        next_c = []
        for i in range(len(c)-1):
            denominator = t[i+p+1] - t[i+1]
            if denominator <= 0:
                raise ValueError("unsupported zero derivative denominator")
            next_c.append(Fraction(p)*(c[i+1]-c[i])/denominator)
        t, c, p = t[1:-1], next_c, p-1
    if len(result) != 4:
        raise ValueError("unsupported derivative order")
    return result


def _add(a: Interval, b: Interval) -> Interval:
    return a[0]+b[0], a[1]+b[1]


def _sub(a: Interval, b: Interval) -> Interval:
    return a[0]-b[1], a[1]-b[0]


def _mul(a: Interval, b: Interval) -> Interval:
    values = (a[0]*b[0], a[0]*b[1], a[1]*b[0], a[1]*b[1])
    return min(values), max(values)


def _scale(a: Interval, value: Fraction) -> Interval:
    return _mul(a, (value, value))


def _deboor(data, lo: Fraction, hi: Fraction, span: int) -> Interval:
    t, c, p = data
    d = [(v, v) for v in c[span-p:span+1]]
    station = (lo, hi)
    for r in range(1, p+1):
        for j in range(p, r-1, -1):
            index = span-p+j
            denominator = t[span+j-r+1]-t[index]
            if denominator <= 0:
                raise ValueError("unsupported zero de Boor denominator")
            alpha = ((lo-t[index])/denominator, (hi-t[index])/denominator)
            d[j] = _add(_mul(_sub((Fraction(1), Fraction(1)), alpha), d[j-1]), _mul(alpha, d[j]))
    return d[p]


def _coordinate(spline: BSpline, start: float, end: float, lo: Fraction, hi: Fraction,
                cuts: list[Fraction], span_count: int) -> tuple[Interval, ...]:
    t, c, p = _validate_spline(spline, start, end)
    derivatives = _derivatives(t, c, p)
    accum = [[None, None] for _ in range(4)]
    for a, b in zip(cuts, cuts[1:]):
        if a == b or b < lo or a > hi:
            continue
        left, right = max(a, lo), min(b, hi)
        if left > right:
            continue
        # Locate the unique positive original knot span containing this panel.
        midpoint = (a+b)/2
        original_span = next(i for i in range(p, len(c)) if t[i] <= midpoint < t[i+1])
        for order, data in enumerate(derivatives):
            dt, dc, dp = data
            # Derivative knot vector is trimmed; same geometric span index shifts by order.
            idx = original_span-order
            value = _deboor(data, left, right, idx)
            if accum[order][0] is None:
                accum[order] = [value[0], value[1]]
            else:
                accum[order][0] = min(accum[order][0], value[0])
                accum[order][1] = max(accum[order][1], value[1])
    if any(pair[0] is None for pair in accum):
        raise ValueError("interval did not intersect supported positive spans")
    return tuple((pair[0], pair[1]) for pair in accum)


def bound_reference_derivatives(reference: ReferenceCurve, station_interval: object, *,
                                maximum_spans: int = 4096) -> ReferenceDerivativeBounds:
    """Enclose north/east spline values and derivatives on an in-domain interval."""
    if type(reference) is not ReferenceCurve:
        raise TypeError("reference must be an exact ReferenceCurve")
    if isinstance(maximum_spans, bool) or not isinstance(maximum_spans, int) or maximum_spans <= 0:
        raise ValueError("maximum_spans must be a positive integer")
    if not isinstance(station_interval, (tuple, list)) or len(station_interval) != 2:
        raise TypeError("station_interval must be a pair")
    if any(isinstance(v, bool) or not isinstance(v, Real) for v in station_interval):
        raise TypeError("station endpoints must be real non-bool values")
    try:
        lo_f, hi_f = float(station_interval[0]), float(station_interval[1])
    except (OverflowError, ValueError) as exc:
        raise ValueError("invalid station interval") from exc
    if not math.isfinite(lo_f) or not math.isfinite(hi_f) or lo_f > hi_f:
        raise ValueError("station interval must be finite and ordered")
    if lo_f < reference.start or hi_f > reference.end:
        raise ValueError("station interval is outside the reference domain")
    tx, cx, px = _validate_spline(reference._x, reference.start, reference.end)
    ty, cy, py = _validate_spline(reference._y, reference.start, reference.end)
    if px != py:
        raise ValueError("coordinate spline degrees differ")
    lo, hi = _fraction(lo_f), _fraction(hi_f)
    cuts = sorted({lo, hi, *[v for v in tx+ty if lo < v < hi]})
    positive = [(a, b) for a, b in zip(cuts, cuts[1:]) if a < b]
    if lo == hi:
        # Point query uses a containing right span, except at the domain end.
        source_cuts = sorted({v for v in tx+ty if reference.start <= float(v) <= reference.end})
        i = next((j for j in range(len(source_cuts)-1) if source_cuts[j] <= lo < source_cuts[j+1]), len(source_cuts)-2)
        cuts = [source_cuts[i], source_cuts[i+1]]
        positive = [(cuts[0], cuts[1])]
    count = len(positive)
    if count > maximum_spans:
        raise ValueError("span budget exceeded")
    union = sorted({v for v in tx+ty if lo < v < hi} | {lo, hi})
    if lo == hi:
        union = cuts
    return ReferenceDerivativeBounds(
        _coordinate(reference._x, reference.start, reference.end, lo, hi, union, count),
        _coordinate(reference._y, reference.start, reference.end, lo, hi, union, count),
        (lo_f, hi_f), count)
