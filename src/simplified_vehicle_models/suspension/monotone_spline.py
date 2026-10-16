"""Fritsch-Carlson monotone cubic Hermite spline (NumPy only)."""
from __future__ import annotations

import numpy as np

from .errors import DomainError

__all__ = ["MonotoneSpline", "DomainError"]


def _fritsch_carlson_slopes(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    h = np.diff(x)
    delta = np.diff(y) / h
    n = x.size
    if n == 2:
        return np.array([delta[0], delta[0]])
    m = np.zeros(n)
    m[1:-1] = 0.5 * (delta[:-1] + delta[1:])
    # Interior extrema (or flat segments) get a zero slope so no overshoot.
    m[1:-1][delta[:-1] * delta[1:] <= 0.0] = 0.0
    # Three-point one-sided end slopes, sign-corrected.
    for end, (d0, d1, h0, h1) in (
        (0, (delta[0], delta[1], h[0], h[1])),
        (n - 1, (delta[-1], delta[-2], h[-1], h[-2])),
    ):
        slope = ((2.0 * h0 + h1) * d0 - h0 * d1) / (h0 + h1)
        if slope * d0 <= 0.0:
            slope = 0.0
        elif d0 * d1 <= 0.0 and abs(slope) > 3.0 * abs(d0):
            slope = 3.0 * d0
        m[end] = slope
    # Fritsch-Carlson circle constraint alpha^2 + beta^2 <= 9 on each interval.
    # Scaling only shrinks slopes, so earlier intervals stay admissible.
    for k in range(n - 1):
        if delta[k] == 0.0:
            m[k] = 0.0
            m[k + 1] = 0.0
            continue
        alpha = m[k] / delta[k]
        beta = m[k + 1] / delta[k]
        radius2 = alpha * alpha + beta * beta
        if radius2 > 9.0:
            tau = 3.0 / np.sqrt(radius2)
            m[k] = tau * alpha * delta[k]
            m[k + 1] = tau * beta * delta[k]
    return m


class MonotoneSpline:
    """C1 piecewise cubic Hermite interpolant with Fritsch-Carlson slopes.

    Monotone data give a monotone interpolant. Queries outside
    ``[x[0], x[-1]]`` raise :class:`DomainError`.
    """

    def __init__(self, x, y) -> None:
        x = np.array(x, dtype=float)
        y = np.array(y, dtype=float)
        if x.ndim != 1 or y.shape != x.shape:
            raise ValueError("x and y must be 1-D arrays of equal length")
        if x.size < 2:
            raise ValueError("at least two knots are required")
        if not (np.all(np.isfinite(x)) and np.all(np.isfinite(y))):
            raise ValueError("knots must be finite")
        if np.any(np.diff(x) <= 0.0):
            raise ValueError("x must be strictly increasing")
        self._x = x
        self._y = y
        self._m = _fritsch_carlson_slopes(x, y)
        for array in (self._x, self._y, self._m):
            array.setflags(write=False)

    @property
    def x(self) -> np.ndarray:
        return self._x

    @property
    def y(self) -> np.ndarray:
        return self._y

    @property
    def slopes(self) -> np.ndarray:
        return self._m

    def _locate(self, q, side: str = "right"):
        q = np.asarray(q, dtype=float)
        if not np.all((q >= self._x[0]) & (q <= self._x[-1])):
            raise DomainError(
                f"query outside spline domain [{self._x[0]}, {self._x[-1]}]"
            )
        if side == "right":
            k = np.searchsorted(self._x, q, side="right") - 1
        elif side == "left":
            k = np.searchsorted(self._x, q, side="left") - 1
        else:
            raise ValueError("side must be 'left' or 'right'")
        k = np.clip(k, 0, self._x.size - 2)
        h = self._x[k + 1] - self._x[k]
        t = (q - self._x[k]) / h
        return k, h, t

    def __call__(self, q) -> np.ndarray:
        k, h, t = self._locate(q)
        t2 = t * t
        t3 = t2 * t
        h00 = 2.0 * t3 - 3.0 * t2 + 1.0
        h10 = t3 - 2.0 * t2 + t
        h01 = -2.0 * t3 + 3.0 * t2
        h11 = t3 - t2
        return (
            h00 * self._y[k]
            + h10 * h * self._m[k]
            + h01 * self._y[k + 1]
            + h11 * h * self._m[k + 1]
        )

    def derivative(self, q, side: str = "right") -> np.ndarray:
        """First derivative. ``side`` picks the interval at an interior knot;
        the Hermite construction makes both choices agree (C1)."""
        k, h, t = self._locate(q, side)
        t2 = t * t
        d00 = 6.0 * t2 - 6.0 * t
        d10 = 3.0 * t2 - 4.0 * t + 1.0
        d01 = -6.0 * t2 + 6.0 * t
        d11 = 3.0 * t2 - 2.0 * t
        return (
            d00 * self._y[k] / h
            + d10 * self._m[k]
            + d01 * self._y[k + 1] / h
            + d11 * self._m[k + 1]
        )
