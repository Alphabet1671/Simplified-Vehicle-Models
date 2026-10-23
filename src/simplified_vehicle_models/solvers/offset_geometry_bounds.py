"""Exact-rational enclosures for the stored analytic offset path geometry.

This bounds the actual OffsetTrajectory path, not just its fitted reference.
It is a local geometry primitive and certifies no vehicle or physical policy.
"""
from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
import math
from numbers import Real
from typing import TypeAlias

import numpy as np
from scipy.interpolate import CubicSpline

from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory, ReferenceCurve
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.solvers.reference_spline_bounds import bound_reference_derivatives
from simplified_vehicle_models.solvers.reference_geometry_bounds import (
    _add, _sub, _mul, _square, _div_positive, _sqrt_interval,
)

Interval: TypeAlias = tuple[Fraction, Fraction]


@dataclass(frozen=True, slots=True)
class OffsetGeometryBounds:
    offset_interval: Interval
    fold_interval: Interval
    metric_interval: Interval
    curvature_interval: Interval
    station_interval: tuple[float, float]
    panel_count: int
    sqrt_bits: int
    maximum_panel_curvature_width: Fraction | None
    max_accepted_panel_curvature_width: Fraction
    exact_rational_enclosure: bool = True
    physical_feasibility_certified: bool = False


def _f(v: object) -> Fraction:
    return Fraction.from_float(float(v))


def _horner(coeff: list[Fraction], x: Interval) -> Interval:
    value = (Fraction(0), Fraction(0))
    for c in coeff:
        value = _add(_mul(value, x), (c, c))
    return value


def _poly(curve: CubicSpline, index: int, order: int, a: Fraction, b: Fraction) -> Interval:
    c = np.asarray(curve.c)
    result: list[Fraction] = []
    degree = c.shape[0] - 1
    for row in range(degree + 1):
        power = degree - row
        if power < order:
            continue
        factor = math.factorial(power) // math.factorial(power-order)
        result.append(Fraction(factor) * _f(c[row, index]))
    x0 = _f(curve.x[index])
    return _horner(result, (a-x0, b-x0))


def _reference(reference: ReferenceCurve, a: float, b: float, bits: int):
    d = bound_reference_derivatives(reference, (a, b), maximum_spans=1)
    x, y = d.north_derivatives, d.east_derivatives
    dx, ddx, dddx = x[1], x[2], x[3]
    dy, ddy, dddy = y[1], y[2], y[3]
    h2 = _add(_square(dx), _square(dy))
    h = _sqrt_interval(h2, bits)
    if h[0] <= 0:
        return None
    hs = _div_positive(_add(_mul(dx, ddx), _mul(dy, ddy)), h)
    h3 = _mul(h2, h)
    k = _div_positive(_sub(_mul(dx, ddy), _mul(dy, ddx)), h3)
    ks_num = _sub(_mul(dx, dddy), _mul(dy, dddx))
    ks = _sub(_div_positive(ks_num, h3), _div_positive(_mul((Fraction(3), Fraction(3)), _mul(k, hs)), h))
    return h, hs, k, ks


def _validate(trajectory: object, interval: object, maximum_panels: int,
              maximum_depth: int, sqrt_bits: int, width: Real | None):
    if type(trajectory) is not OffsetTrajectory:
        raise TypeError("trajectory must be an exact OffsetTrajectory")
    if type(trajectory.reference) is not ReferenceCurve or type(trajectory.spline) is not OffsetSpline:
        raise TypeError("unsupported trajectory source types")
    if type(trajectory._offset_curve) is not CubicSpline:
        raise TypeError("offset curve must be an exact stored CubicSpline")
    c, x = np.asarray(trajectory._offset_curve.c), np.asarray(trajectory._offset_curve.x)
    if (x.ndim != 1 or x.dtype != np.dtype(np.float64) or c.ndim != 2
            or c.dtype != np.dtype(np.float64) or c.shape != (4, x.size-1)):
        raise ValueError("unsupported offset polynomial layout")
    if not np.all(np.isfinite(x)) or not np.all(np.isfinite(c)) or np.any(np.diff(x) <= 0):
        raise ValueError("offset polynomial data must be finite and increasing")
    if x[0] > trajectory.reference.start or x[-1] < trajectory.reference.end:
        raise ValueError("offset polynomial does not cover source domain")
    for name, val, minimum in (("maximum_panels", maximum_panels, 1), ("maximum_depth", maximum_depth, 0)):
        if isinstance(val, bool) or not isinstance(val, int) or val < minimum:
            raise ValueError(f"{name} must be an integer >= {minimum}")
    if isinstance(sqrt_bits, bool) or not isinstance(sqrt_bits, int) or not 16 <= sqrt_bits <= 256:
        raise ValueError("sqrt_bits must be an integer from 16 through 256")
    w = None
    if width is not None:
        if isinstance(width, bool) or not isinstance(width, Real):
            raise TypeError("maximum_panel_curvature_width must be a positive real")
        wf = float(width)
        if not math.isfinite(wf) or wf <= 0:
            raise ValueError("maximum_panel_curvature_width must be finite and positive")
        w = Fraction(width)
    if not isinstance(interval, (tuple, list)) or len(interval) != 2:
        raise TypeError("station_interval must be a pair")
    if any(isinstance(v, bool) or not isinstance(v, Real) for v in interval):
        raise TypeError("station endpoints must be real non-bool values")
    lo, hi = float(interval[0]), float(interval[1])
    if (not math.isfinite(lo) or not math.isfinite(hi)
            or Fraction.from_float(lo) != Fraction(interval[0])
            or Fraction.from_float(hi) != Fraction(interval[1])):
        raise ValueError("station endpoints must be exactly representable as binary64")
    if lo > hi or lo < trajectory.reference.start or hi > trajectory.reference.end:
        raise ValueError("station interval is unordered or outside source domain")
    return lo, hi, w


def bound_offset_geometry(trajectory: OffsetTrajectory, station_interval: object, *,
                          maximum_panels: int = 4096, maximum_depth: int = 24,
                          sqrt_bits: int = 80,
                          maximum_panel_curvature_width: Real | None = None) -> OffsetGeometryBounds:
    """Enclose offset, fold, metric and actual path curvature over source station.

    Budgets count every evaluated panel.  The optional curvature-width goal is
    per accepted panel; the returned global union can be wider due to variation.
    No samples, refits, or fallback approximations are used.
    """
    lo, hi, width = _validate(trajectory, station_interval, maximum_panels,
                              maximum_depth, sqrt_bits, maximum_panel_curvature_width)
    loq, hiq = _f(lo), _f(hi)
    ref, pp = trajectory.reference, trajectory._offset_curve
    cuts = {loq, hiq}
    if loq < hiq:
        for spline in (ref._x, ref._y):
            cuts.update(_f(v) for v in spline.t if loq < _f(v) < hiq)
        cuts.update(_f(v) for v in pp.x if loq < _f(v) < hiq)
        initial = [(a,b,0) for a,b in zip(sorted(cuts), sorted(cuts)[1:]) if a < b]
    else:
        initial = [(loq, hiq, 0)]
    if len(initial) > maximum_panels:
        raise ValueError("initial union panels exceed panel budget")
    pending = list(initial)
    accepted: list[tuple[Interval, Interval, Interval, Interval]] = []
    evals = 0
    while pending:
        a,b,depth = pending.pop()
        if evals >= maximum_panels:
            raise RuntimeError("unresolved bound: panel budget exhausted")
        evals += 1
        # Select right polynomial at an interior breakpoint; the domain end uses left.
        mid = (a+b)/2 if a < b else a
        # Exact rational midpoint avoids rounding onto a neighboring breakpoint.
        idx = next((i for i in range(pp.x.size-1)
                    if _f(pp.x[i]) <= mid < _f(pp.x[i+1])), pp.x.size-2)
        if a == b:
            idx = next((i for i in range(pp.x.size-1)
                        if _f(pp.x[i]) <= a < _f(pp.x[i+1])), pp.x.size-2)
        n = _poly(pp, idx, 0, a, b)
        ns = _poly(pp, idx, 1, a, b)
        nss = _poly(pp, idx, 2, a, b)
        geom = _reference(ref, float(a), float(b), sqrt_bits)
        result = None
        if geom is not None:
            h, hs, k, ks = geom
            fold = _sub((Fraction(1), Fraction(1)), _mul(k,n))
            A, B = _mul(h,fold), ns
            As = _sub(_mul(hs,fold), _mul(h, _add(_mul(ks,n), _mul(k,ns))))
            g2 = _add(_square(A), _square(B))
            if g2[0] > 0:
                g = _sqrt_interval(g2, sqrt_bits)
                if g[0] > 0:
                    psi_s = _div_positive(_sub(_mul(A,nss), _mul(B,As)), g2)
                    path_k = _div_positive(_add(_mul(h,k),psi_s),g)
                    result = n,fold,g,path_k
        if result is not None and (width is None or result[3][1]-result[3][0] <= width):
            accepted.append(result)
            continue
        if depth >= maximum_depth:
            raise RuntimeError("unresolved tangent or curvature width at maximum depth")
        m = float((a+b)/2)
        if not float(a) < m < float(b):
            raise ValueError("unresolved geometry at binary64 station resolution")
        if evals + len(pending) + 2 > maximum_panels:
            raise RuntimeError("unresolved bound: panel budget exhausted")
        mq = _f(m)
        pending.extend(((mq,b,depth+1),(a,mq,depth+1)))
    def union(i: int) -> Interval:
        return min(p[i][0] for p in accepted), max(p[i][1] for p in accepted)
    max_width = max((p[3][1]-p[3][0] for p in accepted), default=Fraction(0))
    return OffsetGeometryBounds(union(0), union(1), union(2), union(3),
                                (lo,hi), evals, sqrt_bits, width, max_width)
