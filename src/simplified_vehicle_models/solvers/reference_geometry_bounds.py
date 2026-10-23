"""Exact-rational interval bounds for reference metric and curvature.

This is a reference-geometry enclosure only. It does not certify an offset,
vehicle state, physical feasibility, or a whole trajectory. At C2 cubic knots,
third derivatives (and therefore curvature_s) may jump; closed panels include
the appropriate one-sided limits on both sides of each knot.
"""
from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
import math
from numbers import Real

import numpy as np

from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve
from simplified_vehicle_models.solvers.reference_spline_bounds import (
    Interval, bound_reference_derivatives,
)


@dataclass(frozen=True, slots=True)
class ReferenceGeometryBounds:
    metric_interval: Interval
    curvature_interval: Interval
    curvature_s_interval: Interval
    station_interval: tuple[float, float]
    panel_count: int
    sqrt_bits: int
    exact_rational_enclosure: bool = True
    whole_trajectory_certified: bool = False


def _add(a: Interval, b: Interval) -> Interval:
    return a[0] + b[0], a[1] + b[1]


def _sub(a: Interval, b: Interval) -> Interval:
    return a[0] - b[1], a[1] - b[0]


def _mul(a: Interval, b: Interval) -> Interval:
    v = (a[0]*b[0], a[0]*b[1], a[1]*b[0], a[1]*b[1])
    return min(v), max(v)


def _square(a: Interval) -> Interval:
    if a[0] <= 0 <= a[1]:
        return Fraction(0), max(a[0]*a[0], a[1]*a[1])
    v = (a[0]*a[0], a[1]*a[1])
    return min(v), max(v)


def _div_positive(a: Interval, b: Interval) -> Interval:
    if b[0] <= 0:
        raise ValueError("interval denominator is not strictly positive")
    return _mul(a, (1/b[1], 1/b[0]))


def _sqrt_endpoint_floor(q: Fraction, bits: int) -> Fraction:
    if q < 0:
        raise ValueError("square-root input must be nonnegative")
    scale = 1 << bits
    root = math.isqrt((q.numerator * scale * scale) // q.denominator)
    return Fraction(root, scale)


def _sqrt_interval(q: Interval, bits: int) -> Interval:
    """Enclose sqrt(q) using integer square roots, without libm or float math.

    For q=n/d, R=isqrt(floor(n*2**(2b)/d)) implies
    R**2 <= q*2**(2b) < (R+1)**2. The lower endpoint is R/2**b;
    the upper endpoint is the same when exact, otherwise (R+1)/2**b.
    """
    if q[0] < 0:
        raise ValueError("square-root interval must be nonnegative")
    scale = 1 << bits
    lo = _sqrt_endpoint_floor(q[0], bits)
    n, d = q[1].numerator, q[1].denominator
    r = math.isqrt((n * scale * scale) // d)
    hi = Fraction(r, scale)
    if r*r*d != n*scale*scale:
        hi = Fraction(r+1, scale)
    return lo, hi


def _geometry(reference: ReferenceCurve, a: float, b: float, bits: int):
    d = bound_reference_derivatives(reference, (a, b), maximum_spans=4096)
    x, y = d.north_derivatives, d.east_derivatives
    dx, ddx, dddx = x[1], x[2], x[3]
    dy, ddy, dddy = y[1], y[2], y[3]
    h2 = _add(_square(dx), _square(dy))
    h = _sqrt_interval(h2, bits)
    if h[0] <= 0:
        return None
    dot = _add(_mul(dx, ddx), _mul(dy, ddy))
    hs = _div_positive(dot, h)
    h3 = _mul(h2, h)
    numerator = _sub(_mul(dx, ddy), _mul(dy, ddx))
    k = _div_positive(numerator, h3)
    n_s = _sub(_mul(dx, dddy), _mul(dy, dddx))
    ks = _sub(_div_positive(n_s, h3), _div_positive(_scale(_mul(k, hs), Fraction(3)), h))
    return _sqrt_interval(h2, bits), k, ks


def _scale(a: Interval, n: Fraction) -> Interval:
    return _mul(a, (n, n))


def _validate_real_pair(value: object) -> tuple[float, float]:
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise TypeError("station_interval must be a pair")
    if any(isinstance(v, bool) or not isinstance(v, Real) for v in value):
        raise TypeError("station endpoints must be real non-bool values")
    try:
        lo, hi = float(value[0]), float(value[1])
    except (OverflowError, ValueError) as exc:
        raise ValueError("invalid station interval") from exc
    if not math.isfinite(lo) or not math.isfinite(hi):
        raise ValueError("station endpoints must be finite")
    if Fraction.from_float(lo) != Fraction(value[0]) or Fraction.from_float(hi) != Fraction(value[1]):
        raise ValueError("station endpoints must be exactly representable as binary64")
    if lo > hi:
        raise ValueError("station interval must be ordered")
    return lo, hi


def bound_reference_geometry(reference: ReferenceCurve, station_interval: object, *,
                             maximum_panels: int = 4096, maximum_depth: int = 24,
                             sqrt_bits: int = 80,
                             maximum_curvature_width: Real | None = None) -> ReferenceGeometryBounds:
    """Enclose h, k, and k_s on the entire binary64 source interval.

    Budgets count every panel evaluation. Panels are bisected at a binary64
    interior midpoint; inability to form one fails closed. No sampling fallback
    is used when tangent regularity or requested curvature width is unresolved.
    """
    if type(reference) is not ReferenceCurve:
        raise TypeError("reference must be an exact ReferenceCurve")
    for name, value, minimum in (("maximum_panels", maximum_panels, 1), ("maximum_depth", maximum_depth, 0)):
        if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
            raise ValueError(f"{name} must be an integer >= {minimum}")
    if isinstance(sqrt_bits, bool) or not isinstance(sqrt_bits, int) or not 16 <= sqrt_bits <= 256:
        raise ValueError("sqrt_bits must be an integer from 16 through 256")
    width = None
    if maximum_curvature_width is not None:
        if isinstance(maximum_curvature_width, bool) or not isinstance(maximum_curvature_width, Real):
            raise TypeError("maximum_curvature_width must be a positive real")
        try:
            wf = float(maximum_curvature_width)
        except (OverflowError, ValueError) as exc:
            raise ValueError("invalid maximum_curvature_width") from exc
        if not math.isfinite(wf) or wf <= 0:
            raise ValueError("maximum_curvature_width must be finite and positive")
        width = Fraction.from_float(wf)

    lo, hi = _validate_real_pair(station_interval)
    if lo < reference.start or hi > reference.end:
        raise ValueError("station interval is outside the reference domain")
    # Partition at every source knot from either coordinate; positive intervals
    # cover the requested closed domain. A point query uses one degenerate panel.
    if lo == hi:
        initial = [(lo, hi, 0)]
    else:
        knots = {lo, hi}
        for spline in (reference._x, reference._y):
            knots.update(float(v) for v in spline.t if lo < float(v) < hi)
        cuts = sorted(knots)
        initial = [(a, b, 0) for a, b in zip(cuts, cuts[1:]) if a < b]
    if len(initial) > maximum_panels:
        raise ValueError("initial source panels exceed panel budget")

    pending = list(initial)
    accepted: list[tuple[Interval, Interval, Interval]] = []
    evaluations = 0
    while pending:
        a, b, depth = pending.pop()
        if evaluations >= maximum_panels:
            raise RuntimeError("unresolved bound: panel budget exhausted")
        evaluations += 1
        result = _geometry(reference, a, b, sqrt_bits)
        if result is not None and (width is None or result[1][1]-result[1][0] <= width):
            accepted.append(result)
            continue
        if depth >= maximum_depth:
            raise RuntimeError("unresolved tangent or curvature width at maximum depth")
        midpoint = float((Fraction.from_float(a) + Fraction.from_float(b)) / 2)
        if not a < midpoint < b:
            raise ValueError("unresolved tangent at floating-point resolution")
        # Both children are guaranteed budget before either is evaluated.
        if evaluations + len(pending) + 2 > maximum_panels:
            raise RuntimeError("unresolved bound: panel budget exhausted")
        pending.extend(((midpoint, b, depth+1), (a, midpoint, depth+1)))

    def union(index: int) -> Interval:
        return min(p[index][0] for p in accepted), max(p[index][1] for p in accepted)

    return ReferenceGeometryBounds(union(0), union(1), union(2), (lo, hi), evaluations, sqrt_bits)
