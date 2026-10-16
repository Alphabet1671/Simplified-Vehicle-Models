"""Conservative component ranges of SciPy BDF dense-output polynomials.

``scipy.integrate.solve_ivp(..., method="BDF", dense_output=True)`` returns an
``OdeSolution`` whose per-step interpolants are polynomials of degree equal to
the BDF order (1..5).  Sampling nodes and midpoints cannot certify the range of
such a polynomial between steps, so this module extracts the polynomial of each
interval exactly (in a well-conditioned local variable) and bounds every state
component over the interval with the Bernstein convex-hull property
(a polynomial lies between the min and max of its Bernstein coefficients).
Each interval is optionally subdivided to tighten the hull.

Scope and honesty notes:

* The bound is a bound on the *interpolating polynomial*, not on the true ODE
  trajectory; it says nothing about the integrator's truncation error.
* Floating-point round-off is covered by a deliberately generous, scale-based
  outward padding (``_PAD_EPS`` machine epsilons of the term magnitudes), not
  by rigorous directed-rounding interval arithmetic.
* Only SciPy's ``BdfDenseOutput`` is accepted.  Every other interpolant type
  (RK, LSODA, Radau, subclasses, malformed BDF data...) raises
  :class:`UnsupportedDenseOutputError` rather than being guessed at.
* Non-finite coefficients yield ``(-inf, +inf)`` bounds (fail closed).

Local variable: ``s = (t - t_end) / h`` where ``t_end`` is the interpolant's
right end ``t`` and ``h`` the signed BDF step stored by SciPy.  BDF's basis is
``p_k(s) = prod_{j<=k} (s + j) / (j + 1)`` which does not depend on the step,
so the polynomial is ``y(s) = D[0] + sum_k D[k + 1] p_k(s)``.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import comb

import numpy as np

__all__ = [
    "DensePolynomials",
    "UnsupportedDenseOutputError",
    "dense_component_bounds",
    "dense_polynomial_coefficients",
    "dense_subspan_bounds",
    "evaluate_dense_polynomials",
]

_MAX_DEGREE = 5
_EPS = float(np.finfo(float).eps)
#: Outward padding, in units of machine epsilon times the term magnitude scale.
_PAD_EPS = 256.0
_DEFAULT_SUBDIVISIONS = 8


class UnsupportedDenseOutputError(ValueError):
    """The dense solution is not a verifiable SciPy BDF interpolant."""


def _basis_matrix() -> np.ndarray:
    """Power-basis (ascending in s) coefficients of p_0..p_{MAX-1}; shape (5, 6)."""
    basis = np.zeros((_MAX_DEGREE, _MAX_DEGREE + 1))
    poly = np.array([1.0])
    for k in range(_MAX_DEGREE):
        poly = np.convolve(poly, np.array([float(k), 1.0])) / (k + 1.0)  # (s + k) / (k + 1)
        basis[k, : poly.size] = poly
    return basis


_BASIS = _basis_matrix()


@dataclass(frozen=True)
class DensePolynomials:
    """Per-interval polynomials ``y(s) = sum_j coefficients[i, j] * s**j``.

    ``s = (t - t_end[i]) / step[i]``; the interval ``i`` spans the times
    ``times[i]..times[i + 1]`` (event-truncated spans are honoured), which maps
    to ``s_lower[i]..s_upper[i]``.  ``coefficients`` has shape
    ``(n_intervals, 6, n_states)`` with unused high powers zero.
    """

    times: np.ndarray
    t_end: np.ndarray
    step: np.ndarray
    s_lower: np.ndarray
    s_upper: np.ndarray
    order: np.ndarray
    coefficients: np.ndarray

    @property
    def n_intervals(self) -> int:
        return int(self.t_end.size)

    @property
    def n_states(self) -> int:
        return int(self.coefficients.shape[2])


def _bdf_class():
    try:
        from scipy.integrate._ivp.bdf import BdfDenseOutput
    except Exception as error:  # pragma: no cover - SciPy layout changed
        raise UnsupportedDenseOutputError("cannot locate SciPy BdfDenseOutput") from error
    return BdfDenseOutput


def dense_polynomial_coefficients(solution) -> DensePolynomials:
    """Extract exact local polynomials from a SciPy BDF ``OdeSolution``."""
    bdf = _bdf_class()
    try:
        times = np.asarray(solution.ts, dtype=float)
        pieces = list(solution.interpolants)
    except Exception as error:
        raise UnsupportedDenseOutputError("object is not a SciPy OdeSolution") from error
    if times.ndim != 1 or len(pieces) < 1 or times.size != len(pieces) + 1:
        raise UnsupportedDenseOutputError("interval/interpolant count mismatch")
    if not np.all(np.isfinite(times)):
        raise UnsupportedDenseOutputError("non-finite interval times")

    count = len(pieces)
    first = pieces[0]
    if type(first) is not bdf:
        raise UnsupportedDenseOutputError(f"unsupported dense output {type(first).__name__}; BDF only")
    n_states = int(np.asarray(first.D).shape[1])
    d_padded = np.zeros((count, _MAX_DEGREE + 1, n_states))
    t_end = np.empty(count)
    step = np.empty(count)
    order = np.empty(count, dtype=int)
    for i, piece in enumerate(pieces):
        if type(piece) is not bdf:
            raise UnsupportedDenseOutputError(f"unsupported dense output {type(piece).__name__}; BDF only")
        try:
            k = int(piece.order)
            d = np.asarray(piece.D, dtype=float)
            shift = np.asarray(piece.t_shift, dtype=float)
            denom = np.asarray(piece.denom, dtype=float)
            end = float(piece.t)
            lo, hi = float(piece.t_min), float(piece.t_max)
        except Exception as error:
            raise UnsupportedDenseOutputError("malformed BDF interpolant") from error
        if not 1 <= k <= _MAX_DEGREE or d.shape != (k + 1, n_states):
            raise UnsupportedDenseOutputError("unexpected BDF order or coefficient shape")
        h = float(denom[0]) if denom.size else float("nan")
        if shift.shape != (k,) or denom.shape != (k,) or not np.isfinite(h) or h == 0.0:
            raise UnsupportedDenseOutputError("malformed BDF step data")
        j = np.arange(k)
        scale = max(abs(end), abs(h), 1.0)
        if (
            np.any(np.abs(shift - (end - h * j)) > 1e-9 * scale)
            or np.any(np.abs(denom - h * (1.0 + j)) > 1e-9 * abs(h))
            or not np.isfinite(end)
        ):
            raise UnsupportedDenseOutputError("BDF basis is not the expected Newton form")
        # The requested interval must lie inside the interpolant's own validity range.
        a, b = sorted((times[i], times[i + 1]))
        tol = 1e-9 * max(abs(h), 1e-300) + 64 * _EPS * scale
        if a < min(lo, hi) - tol or b > max(lo, hi) + tol:
            raise UnsupportedDenseOutputError("interval lies outside the interpolant's range")
        d_padded[i, : k + 1] = d
        t_end[i], step[i], order[i] = end, h, k

    # Power-basis coefficients in s.  P maps D[1:] rows to powers of s.
    coeff = np.einsum("mkn,kj->mjn", d_padded[:, 1:], _BASIS)
    coeff[:, 0, :] += d_padded[:, 0, :]

    s_a = (times[:-1] - t_end) / step
    s_b = (times[1:] - t_end) / step
    lower, upper = np.minimum(s_a, s_b), np.maximum(s_a, s_b)
    return DensePolynomials(times, t_end, step, lower, upper, order, coeff)


def evaluate_dense_polynomials(poly: DensePolynomials, interval: int, t) -> np.ndarray:
    """Evaluate one extracted interval at times ``t``; returns ``(n_states, len(t))``."""
    t = np.atleast_1d(np.asarray(t, dtype=float))
    s = (t - poly.t_end[interval]) / poly.step[interval]
    powers = s[None, :] ** np.arange(_MAX_DEGREE + 1)[:, None]
    return np.einsum("jn,jt->nt", poly.coefficients[interval], powers)


_BINOM = np.array([[comb(k, m) for m in range(_MAX_DEGREE + 1)] for k in range(_MAX_DEGREE + 1)], dtype=float)
# Power -> Bernstein: b_i = sum_{m<=i} C(i, m) / C(5, m) * a_m
_TO_BERNSTEIN = np.array(
    [[(_BINOM[i, m] / _BINOM[_MAX_DEGREE, m]) if m <= i else 0.0 for m in range(_MAX_DEGREE + 1)]
     for i in range(_MAX_DEGREE + 1)]
)


def _select_components(index, n_states: int) -> np.ndarray:
    if index is None:
        return np.arange(n_states)
    chosen = np.atleast_1d(np.asarray(index))
    if chosen.dtype.kind not in "iu" or chosen.ndim != 1 or chosen.size == 0:
        raise ValueError("index must be None, an integer, or a non-empty integer sequence")
    if np.any(chosen < -n_states) or np.any(chosen >= n_states):
        raise IndexError("state index out of range")
    return chosen % n_states


def dense_component_bounds(solution, index=None, subdivisions: int = _DEFAULT_SUBDIVISIONS) -> np.ndarray:
    """Conservative per-interval minima/maxima of BDF dense-output components.

    ``solution`` is ``solve_ivp(..., method="BDF", dense_output=True).sol``.
    ``index`` selects state components (``None`` = all, int, or sequence).
    Returns an array of shape ``(n_intervals, n_selected, 2)`` holding
    ``[minimum, maximum]`` that contain the interpolating polynomial over
    each ``[ts[i], ts[i + 1]]`` (the final span may be event-truncated).
    Raises :class:`UnsupportedDenseOutputError` for anything but SciPy BDF.
    """
    if not isinstance(subdivisions, (int, np.integer)) or subdivisions < 1:
        raise ValueError("subdivisions must be a positive integer")
    poly = dense_polynomial_coefficients(solution)
    components = _select_components(index, poly.n_states)
    coeff = poly.coefficients[:, :, components]  # (m, 6, n)
    count, _, n = coeff.shape
    pieces = int(subdivisions)

    frac = np.arange(pieces + 1) / pieces
    span = poly.s_upper - poly.s_lower
    grid = poly.s_lower[:, None] + span[:, None] * frac[None, :]
    grid[:, 0] = poly.s_lower
    grid[:, -1] = poly.s_upper
    # Widen outward a few ulps so that rounding of the time -> s map is covered.
    grid_pad = 8 * _EPS * np.maximum(1.0, np.abs(grid))
    start = (grid[:, :-1] - grid_pad[:, :-1])[:, :, None, None]  # (m, N, 1, 1)
    width = (grid[:, 1:] + grid_pad[:, 1:])[:, :, None, None] - start

    c = coeff[:, None, :, :]  # (m, 1, 6, n)
    ac = np.abs(c)
    # Taylor-shift + scale to w in [0, 1]:  a_m = width^m * sum_k c_k C(k, m) start^(k-m)
    a = np.empty((count, pieces, _MAX_DEGREE + 1, n))
    mag = np.empty_like(a)
    s_abs = np.abs(start)
    w_abs = np.abs(width)
    for m in range(_MAX_DEGREE + 1):
        total = np.zeros((count, pieces, 1, n))
        total_abs = np.zeros_like(total)
        for k in range(m, _MAX_DEGREE + 1):
            total += c[:, :, k : k + 1, :] * (_BINOM[k, m] * start**(k - m))
            total_abs += ac[:, :, k : k + 1, :] * (_BINOM[k, m] * s_abs**(k - m))
        a[:, :, m, :] = (total * width**m)[:, :, 0, :]
        mag[:, :, m, :] = (total_abs * w_abs**m)[:, :, 0, :]

    bern = np.einsum("im,pgmn->pgin", _TO_BERNSTEIN, a)
    bern_mag = np.einsum("im,pgmn->pgin", _TO_BERNSTEIN, mag)
    # Round-off scale: Bernstein term magnitudes plus the raw polynomial magnitude.
    s_max = np.maximum(1.0, np.maximum(np.abs(grid[:, 0]), np.abs(grid[:, -1])))
    raw = (ac[:, 0] * s_max[:, None, None] ** np.arange(_MAX_DEGREE + 1)[None, :, None]).sum(axis=1)
    scale = bern_mag.max(axis=2).max(axis=1) + raw  # (m, n)
    lo = bern.min(axis=2).min(axis=1)
    hi = bern.max(axis=2).max(axis=1)
    pad = _PAD_EPS * _EPS * scale
    lo, hi = lo - pad, hi + pad

    bad = ~(np.isfinite(lo) & np.isfinite(hi) & np.isfinite(scale))
    lo = np.where(bad, -np.inf, lo)
    hi = np.where(bad, np.inf, hi)
    return np.stack([lo, hi], axis=-1)


def dense_subspan_bounds(poly: DensePolynomials, interval, start, stop, components=None) -> np.ndarray:
    """Bernstein-hull bounds on sub-spans of the extracted interval polynomials.

    ``interval`` holds interval indices and ``start``/``stop`` the sub-span as
    fractions in ``[0, 1]`` of that interval (``0`` = ``times[i]``, ``1`` =
    ``times[i + 1]``).  Returns ``(m, n_components, 2)`` bounds on the
    interpolating polynomial over each sub-span, with the same outward padding as
    :func:`dense_component_bounds`.  Invalid fractions fail closed (infinite
    bounds).  The same scope notes apply: the bound is on the interpolant only.
    """
    interval = np.atleast_1d(np.asarray(interval))
    start = np.atleast_1d(np.asarray(start, dtype=float))
    stop = np.atleast_1d(np.asarray(stop, dtype=float))
    if interval.dtype.kind not in "iu" or not (interval.shape == start.shape == stop.shape) \
            or interval.ndim != 1:
        raise ValueError("interval, start and stop must be matching 1-d arrays")
    if np.any(interval < 0) or np.any(interval >= poly.n_intervals):
        raise IndexError("interval index out of range")
    valid = np.isfinite(start) & np.isfinite(stop) & (start >= 0.0) & (stop <= 1.0) & (start <= stop)
    components = _select_components(components, poly.n_states)
    coeff = poly.coefficients[interval][:, :, components]  # (m, 6, n)
    count, _, n = coeff.shape

    span = (poly.s_upper - poly.s_lower)[interval]
    lower = poly.s_lower[interval]
    g0 = lower + span * np.where(valid, start, 0.0)
    g1 = lower + span * np.where(valid, stop, 1.0)
    g0 = np.where(start == 0.0, lower, g0)
    g1 = np.where(stop == 1.0, poly.s_upper[interval], g1)
    pad_a = 8 * _EPS * np.maximum(1.0, np.abs(g0))
    pad_b = 8 * _EPS * np.maximum(1.0, np.abs(g1))
    s0 = (g0 - pad_a)[:, None, None]
    width = (g1 + pad_b)[:, None, None] - s0

    ac = np.abs(coeff)
    s_abs, w_abs = np.abs(s0), np.abs(width)
    a = np.empty((count, _MAX_DEGREE + 1, n))
    mag = np.empty_like(a)
    for m in range(_MAX_DEGREE + 1):
        total = np.zeros((count, 1, n))
        total_abs = np.zeros_like(total)
        for k in range(m, _MAX_DEGREE + 1):
            total += coeff[:, k:k + 1, :] * (_BINOM[k, m] * s0**(k - m))
            total_abs += ac[:, k:k + 1, :] * (_BINOM[k, m] * s_abs**(k - m))
        a[:, m, :] = (total * width**m)[:, 0, :]
        mag[:, m, :] = (total_abs * w_abs**m)[:, 0, :]
    bern = np.einsum("im,pmn->pin", _TO_BERNSTEIN, a)
    bern_mag = np.einsum("im,pmn->pin", _TO_BERNSTEIN, mag)
    s_max = np.maximum(1.0, np.maximum(np.abs(g0), np.abs(g1)))
    raw = (ac * s_max[:, None, None] ** np.arange(_MAX_DEGREE + 1)[None, :, None]).sum(axis=1)
    scale = bern_mag.max(axis=1) + raw
    pad = _PAD_EPS * _EPS * scale
    lo, hi = bern.min(axis=1) - pad, bern.max(axis=1) + pad
    bad = ~(np.isfinite(lo) & np.isfinite(hi) & np.isfinite(scale)) | ~valid[:, None]
    return np.stack([np.where(bad, -np.inf, lo), np.where(bad, np.inf, hi)], axis=-1)
