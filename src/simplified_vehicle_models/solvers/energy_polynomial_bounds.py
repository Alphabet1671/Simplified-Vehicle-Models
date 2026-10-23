"""Exact bounds for explicit rational energy and control polynomials.

This is a mathematical primitive, not a state evaluator or physical-feasibility
certificate. Its inputs are explicit endpoint energy/rate samples and its
outputs describe only the polynomials constructed from those values.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
import math
from numbers import Real


_SEMANTICS = "explicit_rational_Hermite_and_Lagrange_energy_v1"


def _rational(value, name: str) -> Fraction:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a finite real number (not bool)")
    if isinstance(value, Fraction):
        return value
    if isinstance(value, int):
        return Fraction(value)
    try:
        f = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise TypeError(f"{name} must be an exactly convertible finite real") from exc
    if not math.isfinite(f):
        raise ValueError(f"{name} must be finite")
    # Require lossless conversion to binary64 before taking its exact ratio.
    try:
        if value != f:
            raise ValueError(f"{name} cannot be represented losslessly as binary64")
    except TypeError as exc:
        raise TypeError(f"{name} must be exactly convertible") from exc
    return Fraction.from_float(f)


def _binary64_coordinate(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a finite binary64 real coordinate")
    try:
        f = float(value)
    except (OverflowError, TypeError, ValueError) as exc:
        raise ValueError(f"{name} is not a finite binary64 coordinate") from exc
    if not math.isfinite(f) or value != f:
        raise ValueError(f"{name} must be losslessly representable as binary64")
    return f


def _pair(values, name):
    try:
        a, b = values
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain exactly two values") from exc
    return a, b


def _split(controls: tuple[Fraction, ...], t: Fraction):
    rows = [list(controls)]
    while len(rows[-1]) > 1:
        prev = rows[-1]
        rows.append([(1 - t) * prev[i] + t * prev[i + 1] for i in range(len(prev) - 1)])
    left = tuple(row[0] for row in rows)
    right = tuple(row[-1] for row in reversed(rows))
    return left, right


def _restrict(controls: tuple[Fraction, ...], lo: Fraction, hi: Fraction):
    if lo == hi:
        value = sum(c * math.comb(len(controls) - 1, i) * lo**i * (1 - lo)**(len(controls) - 1 - i)
                    for i, c in enumerate(controls))
        return (value,) * len(controls)
    left, _ = _split(controls, hi)
    if lo == 0:
        return left
    _, segment = _split(left, lo / hi)
    return segment


def _hull(controls):
    return min(controls), max(controls)


@dataclass(frozen=True)
class EnergyPolynomialBounds:
    energy_interval: tuple[Fraction, Fraction]
    hermite_energy_rate_interval: tuple[Fraction, Fraction]
    control_energy_rate_interval: tuple[Fraction, Fraction]
    energy_rate_defect_interval: tuple[Fraction, Fraction]
    source_interval: tuple[float, float]
    subinterval: tuple[float, float]
    mathematical_semantics: str = _SEMANTICS
    physical_feasibility_certified: bool = False


def bound_energy_polynomials(source_interval, energy_endpoints, energy_rate_sites, *, subinterval=None):
    """Enclose explicit cubic-Hermite energy, its derivative, and quadratic control.

    ``energy_rate_sites`` are control-rate values at normalized source positions
    0, 1/2, and 1. Bounds use exact rational Bernstein coefficients and exact
    de Casteljau restriction. The defect range is tight for this quadratic.
    """
    slo, shi = _pair(source_interval, "source_interval")
    slo, shi = _binary64_coordinate(slo, "source start"), _binary64_coordinate(shi, "source end")
    if not slo < shi:
        raise ValueError("source_interval must have strictly positive width")
    w0, w1 = (_rational(v, "energy endpoint") for v in _pair(energy_endpoints, "energy_endpoints"))
    if w0 <= 0 or w1 <= 0:
        raise ValueError("energy endpoints must be positive")
    f0, fm, f1 = (_rational(v, "energy rate") for v in energy_rate_sites)
    if subinterval is None:
        a, b = slo, shi
    else:
        a, b = _pair(subinterval, "subinterval")
        a, b = _binary64_coordinate(a, "subinterval start"), _binary64_coordinate(b, "subinterval end")
    if a > b or a < slo or b > shi:
        raise ValueError("subinterval must be ordered and contained in source_interval")

    h = Fraction.from_float(shi) - Fraction.from_float(slo)
    xlo = (Fraction.from_float(a) - Fraction.from_float(slo)) / h
    xhi = (Fraction.from_float(b) - Fraction.from_float(slo)) / h
    # Cubic Hermite Bernstein ordinates; derivative and control are quadratic.
    energy = (w0, w0 + h * f0 / 3, w1 - h * f1 / 3, w1)
    deriv = (f0, 3 * (w1 - w0) / h - f0 - f1, f1)
    control = (f0, 2 * fm - (f0 + f1) / 2, f1)
    e1 = deriv[1] - control[1]
    # Their difference is 2*e1*x*(1-x); extrema occur at endpoints or x=1/2.
    points = [xlo, xhi]
    if xlo <= Fraction(1, 2) <= xhi:
        points.append(Fraction(1, 2))
    defects = [2 * e1 * x * (1 - x) for x in points]
    return EnergyPolynomialBounds(
        energy_interval=_hull(_restrict(energy, xlo, xhi)),
        hermite_energy_rate_interval=_hull(_restrict(deriv, xlo, xhi)),
        control_energy_rate_interval=_hull(_restrict(control, xlo, xhi)),
        energy_rate_defect_interval=(min(defects), max(defects)),
        source_interval=(slo, shi), subinterval=(a, b),
    )
