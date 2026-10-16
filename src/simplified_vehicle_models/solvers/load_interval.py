"""Interval enclosure of the planar load map and its Jacobian.

The planar plant closes a two-unknown loop: the body acceleration guess
``a = (a_x, a_y)`` sets the tire loads, the loads set the tire forces, and the
forces imply an acceleration ``g(a, x)``.  This module evaluates ``g`` and
``dg/da`` over a *box* of states ``x`` and a *box* of guesses ``a`` with
interval arithmetic and forward-mode differentiation, mirroring
``planar_7dof._load_step`` operation by operation.  The interval Krawczyk test
in :func:`certify_load_closure` then proves, for every state in the box, that
exactly one root of ``g(a, x) - a`` lies in the guess box and that the residual
Jacobian is nonsingular there.

Scope and honesty notes:

* A certificate covers the solution branch inside the guess box, not every root
  the plane could contain.  The caller supplies boxes centred on the numerical
  solutions at the accepted nodes, so the proven branch is the one near the
  solver's answers.  It does not by itself prove that the solver's iterate at
  an intermediate state lies on that branch.
* Every primitive is widened outward by one ulp (two for transcendentals) with
  ``nextafter``; that is a conservative stand-in for directed rounding, not
  true directed rounding.
* A division by an interval that contains zero, or any non-finite value, fails
  closed: the box is reported as not certified.
* Derivatives of non-smooth operations (``abs``, ``max``, ``min``, the
  small-slip linear branch) use the hull of the one-sided derivatives, so the
  enclosure is a generalised (Clarke) Jacobian.

Layout: every quantity is an array of shape ``(m, ...)`` for ``m`` boxes.  The
value is ``[lo, hi]``; the tangent with respect to the two guesses has the same
shape plus a trailing axis of length two.
"""

from __future__ import annotations

import numpy as np

from simplified_vehicle_models.core.conventions import GRAVITY

__all__ = ["certify_load_closure", "certify_load_closure_adaptive", "load_map_enclosure"]


def _dn(x):
    return np.nextafter(x, -np.inf)


def _up(x):
    return np.nextafter(x, np.inf)


def _widen(lo, hi, count=1):
    for _ in range(count):
        lo, hi = _dn(lo), _up(hi)
    return lo, hi


def _imul(al, ah, bl, bh):
    with np.errstate(all="ignore"):
        products = np.stack(np.broadcast_arrays(al * bl, al * bh, ah * bl, ah * bh))
        products = np.where(np.isnan(products), 0.0, products)  # 0 * inf = 0 (bounded factor)
    return _widen(products.min(axis=0), products.max(axis=0))


#: Number of tangent directions: two guesses, plus eight state directions when
#: the centered (mean-value) form asks for them.
_NT = 2


class V:
    """Interval value with interval tangents (trailing axis of length two)."""

    __slots__ = ("lo", "hi", "tl", "th")
    #: Make NumPy scalars and arrays defer to the reflected operators.
    __array_ufunc__ = None

    def __init__(self, lo, hi, tl=None, th=None):
        lo, hi = np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)
        self.lo, self.hi = lo, hi
        if tl is None:
            tl = np.zeros(lo.shape + (_NT,))
            th = np.zeros(lo.shape + (_NT,))
        self.tl, self.th = np.asarray(tl, float), np.asarray(th, float)

    @property
    def shape(self):
        return self.lo.shape

    def __add__(self, other):
        other = _lift(other)
        lo, hi = _widen(self.lo + other.lo, self.hi + other.hi)
        tl, th = _widen(self.tl + other.tl, self.th + other.th)
        return V(lo, hi, tl, th)

    __radd__ = __add__

    def __neg__(self):
        return V(-self.hi, -self.lo, -self.th, -self.tl)

    def __sub__(self, other):
        return self + (-_lift(other))

    def __rsub__(self, other):
        return _lift(other) + (-self)

    def __mul__(self, other):
        other = _lift(other)
        lo, hi = _imul(self.lo, self.hi, other.lo, other.hi)
        first = _imul(self.lo[..., None], self.hi[..., None], other.tl, other.th)
        second = _imul(other.lo[..., None], other.hi[..., None], self.tl, self.th)
        tl, th = _widen(first[0] + second[0], first[1] + second[1])
        return V(lo, hi, tl, th)

    __rmul__ = __mul__

    def reciprocal(self):
        straddle = (self.lo <= 0.0) & (self.hi >= 0.0)
        with np.errstate(all="ignore"):
            lo = np.where(straddle, -np.inf, 1.0 / self.hi)
            hi = np.where(straddle, np.inf, 1.0 / self.lo)
        lo, hi = _widen(lo, hi)
        square_lo, square_hi = _isquare(lo, hi)
        return _chain(self, lo, hi, -square_hi, -square_lo)

    def __truediv__(self, other):
        return self * _lift(other).reciprocal()

    def __rtruediv__(self, other):
        return _lift(other) * self.reciprocal()


def _lift(x):
    if isinstance(x, V):
        return x
    x = np.asarray(x, dtype=float)
    return V(x, x)


def _isquare(lo, hi):
    with np.errstate(all="ignore"):
        low = np.where((lo <= 0.0) & (hi >= 0.0), 0.0, np.minimum(lo * lo, hi * hi))
        high = np.maximum(lo * lo, hi * hi)
    return low, high


def _chain(x, lo, hi, slope_lo, slope_hi):
    """Value ``[lo, hi]`` of ``f(x)`` with ``f'`` enclosed by the slope interval."""
    tl, th = _imul(np.asarray(slope_lo)[..., None], np.asarray(slope_hi)[..., None], x.tl, x.th)
    return V(lo, hi, tl, th)


def square(x):
    lo, hi = _isquare(x.lo, x.hi)
    lo, hi = _widen(lo, hi)
    return _chain(x, np.maximum(lo, 0.0), hi, 2.0 * x.lo, 2.0 * x.hi)


def vexp(x):
    with np.errstate(all="ignore"):
        lo, hi = _widen(np.exp(x.lo), np.exp(x.hi), 2)
    lo = np.maximum(lo, 0.0)
    return _chain(x, lo, hi, lo, hi)


def vlog(x):
    """Natural log of a strictly positive interval."""
    with np.errstate(all="ignore"):
        lo, hi = _widen(np.log(x.lo), np.log(x.hi), 2)
        slope = V(x.lo, x.hi).reciprocal()
    return _chain(x, lo, hi, slope.lo, slope.hi)


def vsqrt(x):
    with np.errstate(all="ignore"):
        lo, hi = _widen(np.sqrt(np.maximum(x.lo, 0.0)), np.sqrt(np.maximum(x.hi, 0.0)))
        lo = np.maximum(lo, 0.0)
        slope = 0.5 / V(lo, hi)
    return _chain(x, lo, hi, slope.lo, slope.hi)


def vabs(x):
    lo = np.where(x.lo >= 0.0, x.lo, np.where(x.hi <= 0.0, -x.hi, 0.0))
    hi = np.maximum(np.abs(x.lo), np.abs(x.hi))
    return _chain(x, lo, hi, np.where(x.lo >= 0.0, 1.0, -1.0), np.where(x.hi <= 0.0, -1.0, 1.0))


def _select(condition, first, second):
    condition = np.asarray(condition)
    return V(np.where(condition, first.lo, second.lo), np.where(condition, first.hi, second.hi),
             np.where(condition[..., None], first.tl, second.tl),
             np.where(condition[..., None], first.th, second.th))


def vmax(a, b):
    a, b = _lift(a), _lift(b)
    mixed = V(np.maximum(a.lo, b.lo), np.maximum(a.hi, b.hi),
              np.minimum(a.tl, b.tl), np.maximum(a.th, b.th))
    return _select(a.lo >= b.hi, a, _select(b.lo >= a.hi, b, mixed))


def vmin(a, b):
    return -vmax(-_lift(a), -_lift(b))


def vclamp(x, lower, upper):
    return vmin(vmax(x, lower), upper)


def vhypot(x, y):
    h = vsqrt(square(x) + square(y))
    positive = h.lo > 0.0
    with np.errstate(all="ignore"):
        rec_lo = np.where(positive, _dn(1.0 / h.hi), 0.0)
        rec_hi = np.where(positive, _up(1.0 / h.lo), 0.0)

    def direction(component):
        low, high = _imul(component.lo, component.hi, rec_lo, rec_hi)
        return (np.where(positive, np.clip(low, -1.0, 1.0), -1.0),
                np.where(positive, np.clip(high, -1.0, 1.0), 1.0))

    (cxl, cxh), (cyl, cyh) = direction(x), direction(y)
    first = _imul(cxl[..., None], cxh[..., None], x.tl, x.th)
    second = _imul(cyl[..., None], cyh[..., None], y.tl, y.th)
    tl, th = _widen(first[0] + second[0], first[1] + second[1])
    return V(h.lo, h.hi, tl, th)


def _value_only(x, lower, upper):
    """Tighten the value (not the tangent) by a known bound."""
    return V(np.maximum(x.lo, lower), np.minimum(x.hi, upper), x.tl, x.th)


def _concat(values):
    shape = np.broadcast_shapes(*[v.lo.shape for v in values])
    lo = np.concatenate([np.broadcast_to(v.lo, shape) for v in values], axis=-1)
    hi = np.concatenate([np.broadcast_to(v.hi, shape) for v in values], axis=-1)
    tl = np.concatenate([np.broadcast_to(v.tl, shape + (_NT,)) for v in values], axis=-2)
    th = np.concatenate([np.broadcast_to(v.th, shape + (_NT,)) for v in values], axis=-2)
    return V(lo, hi, tl, th)


def _sum_wheels(v):
    total = V(v.lo[..., 0:1], v.hi[..., 0:1], v.tl[..., 0:1, :], v.th[..., 0:1, :])
    for wheel in range(1, v.lo.shape[-1]):
        total = total + V(v.lo[..., wheel:wheel + 1], v.hi[..., wheel:wheel + 1],
                          v.tl[..., wheel:wheel + 1, :], v.th[..., wheel:wheel + 1, :])
    return total


def _trig(lo, hi):
    from simplified_vehicle_models.solvers.planar_dense_certificate import _cos_range, _sin_range
    cl, ch = _cos_range(lo, hi)
    sl, sh = _sin_range(lo, hi)
    # ``_sin_range`` evaluates cos(x - pi/2) and ``pi/2`` is rounded, so the
    # result can be off by about 6e-17 plus eps*|x| in absolute terms.  A
    # relative ulp widening cannot cover that near zero (sin(0) would be
    # excluded), so add an absolute margin that dominates both effects.
    margin = 1.0e-15 + 8.0 * np.finfo(float).eps * np.maximum(np.abs(lo), np.abs(hi))
    cl, ch = np.maximum(cl - margin, -1.0), np.minimum(ch + margin, 1.0)
    sl, sh = np.maximum(sl - margin, -1.0), np.minimum(sh + margin, 1.0)
    return _widen(cl, ch, 2), _widen(sl, sh, 2)


def _polyval(coefficients, x):
    result = _lift(np.zeros(x.shape))
    for coefficient in coefficients:
        result = result * x + coefficient
    return result


def _saturation(phi, e_1):
    """Enclose ``1 - exp(-(phi + e_1 phi^2 + (e_1^2 + 1/12) phi^3))`` for ``phi >= 0``.

    The exponent ``p`` is strictly increasing in ``phi`` for every ``e_1``: its
    derivative ``1 + 2 e_1 phi + 3 (e_1^2 + 1/12) phi^2`` has discriminant
    ``4 e_1^2 - 12 (e_1^2 + 1/12) = -8 e_1^2 - 1 < 0``.  Evaluating at the two
    endpoints is therefore exact, and it avoids the dependency blow-up of three
    independent occurrences of ``phi``.
    """

    c = e_1 ** 2 + 1.0 / 12.0

    def exponent(x):
        return x + e_1 * x * x + c * x * x * x

    with np.errstate(all="ignore"):
        phi_lo = np.maximum(phi.lo, 0.0)
        p_lo = _dn(exponent(phi_lo) - 1.0e-12 * np.abs(exponent(phi_lo)))
        p_hi = _up(exponent(phi.hi) + 1.0e-12 * np.abs(exponent(phi.hi)))
        p_lo = np.maximum(p_lo, 0.0)
        lo, hi = _widen(1.0 - np.exp(-p_lo), 1.0 - np.exp(-p_hi), 2)
        lo, hi = np.clip(lo, 0.0, 1.0), np.clip(hi, 0.0, 1.0)
        lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
        # d f_bar / d phi = exp(-p) p'(phi), with p' > 0 (see above).
        slope_poly = V(phi_lo, phi.hi)
        slope = 1.0 + (2.0 * e_1) * slope_poly + (3.0 * c) * square(slope_poly)
        slope_lo = np.maximum(slope.lo, 0.0)
        s_lo, s_hi = _imul(np.exp(-p_hi), np.exp(-p_lo), slope_lo, np.maximum(slope.hi, 0.0))
    return _chain(phi, lo, hi, np.maximum(s_lo, 0.0), s_hi)


def _series(p, coefficient, count=12):
    """Partial sums ``S_count`` and ``S_count+1`` of ``sum_k (-p)^k coefficient(k)``."""
    total = np.zeros_like(p)
    partial = []
    term = np.ones_like(p)
    for k in range(count + 2):
        total = total + term * coefficient(k)
        if k >= count:
            partial.append(total.copy())
        term = term * (-p)
    return partial


def _factorial(k):
    result = 1.0
    for i in range(2, k + 1):
        result *= i
    return result


def _h_and_slope(p):
    """Point enclosures of ``h(p) = (1 - e^-p)/p`` and ``h'(p)`` for ``p >= 0``.

    Both are monotone (``h`` decreasing from 1, ``h'`` increasing from -1/2), so
    the caller evaluates the interval endpoints.  Below ``p = 0.5`` the
    alternating Taylor series with decreasing terms brackets the value between
    consecutive partial sums; above, the closed form has no cancellation.
    """
    p = np.maximum(np.asarray(p, float), 0.0)
    small = p < 0.5
    ps = np.where(small, p, 0.0)
    h_series = _series(ps, lambda k: 1.0 / _factorial(k + 1))
    d_series = _series(ps, lambda k: -1.0 / (_factorial(k) * (k + 2.0)))
    big = np.where(small, 1.0, p)
    with np.errstate(all="ignore"):
        h_big = -np.expm1(-big) / big
        d_big = -(-np.expm1(-big) - big * np.exp(-big)) / (big * big)
    h_lo = np.where(small, np.minimum(*h_series), h_big)
    h_hi = np.where(small, np.maximum(*h_series), h_big)
    d_lo = np.where(small, np.minimum(*d_series), d_big)
    d_hi = np.where(small, np.maximum(*d_series), d_big)
    slack = 1.0e-12
    return ((h_lo - slack * np.abs(h_lo) - 1.0e-15, h_hi + slack * np.abs(h_hi) + 1.0e-15),
            (d_lo - slack * np.abs(d_lo) - 1.0e-15, d_hi + slack * np.abs(d_hi) + 1.0e-15))


def _smooth_unit_force_tangent(phi_x, phi_y, phi, lam, e_1, value_x, value_y):
    """Replace the tangents of ``f_bar * unit`` by bounded analytic ones.

    With ``f_bar = G(phi) phi`` and ``G = h(p) q(phi)`` smooth, the nonlinear
    tire force direction times magnitude is ``G(phi) Z`` where
    ``Z = (lam phi_x, phi_y) phi / den``.  ``Z`` is homogeneous of degree one in
    ``(phi_x, phi_y)``, so its partials depend only on the direction cosines and
    on ``T = phi/den`` in ``[1/max(1, lam), 1/min(1, lam)]``.  This avoids the
    ``1/den`` blow-up that interval arithmetic produces near zero slip, where
    the product is in fact perfectly smooth.  Values keep the original
    (sound) enclosure.  Fails closed (infinite tangents) when ``lam`` is not
    strictly positive.
    """
    c = e_1 ** 2 + 1.0 / 12.0
    with np.errstate(all="ignore"):
        positive = phi.lo > 0.0
        rec_lo = np.where(positive, _dn(1.0 / phi.hi), 0.0)
        rec_hi = np.where(positive, _up(1.0 / phi.lo), 0.0)

        def cosine(component):
            low, high = _imul(component.lo, component.hi, rec_lo, rec_hi)
            return (np.where(positive, np.clip(low, -1.0, 1.0), -1.0),
                    np.where(positive, np.clip(high, -1.0, 1.0), 1.0))

        ux = V(*cosine(phi_x))
        uy = V(*cosine(phi_y))
        lam_ok = lam.lo > 0.0
        lam_lo = np.where(lam_ok, lam.lo, 1.0)
        lam_hi = np.where(lam_ok, lam.hi, 1.0)
        t_lo = _dn(1.0 / np.maximum(1.0, lam_hi))
        t_hi = _up(1.0 / np.minimum(1.0, lam_lo))
        l_lo = _dn(lam_lo / np.maximum(lam_hi, 1.0))
        l_hi = _up(lam_hi / np.minimum(lam_lo, 1.0))
    T = V(t_lo, t_hi)
    LT = V(l_lo, l_hi)
    lamv = V(lam_lo, lam_hi)
    px, py = V(phi_x.lo, phi_x.hi), V(phi_y.lo, phi_y.hi)

    def unit_square(u):
        lo, hi = _isquare(u.lo, u.hi)
        return V(np.clip(lo, 0.0, 1.0), np.clip(hi, 0.0, 1.0))

    ux2, uy2 = unit_square(ux), unit_square(uy)
    pair_lo, pair_hi = _imul(ux.lo, ux.hi, uy.lo, uy.hi)
    uxuy = V(np.clip(pair_lo, -0.5, 0.5), np.clip(pair_hi, -0.5, 0.5))
    one = 1.0
    t_sq = T * T
    lt_sq = LT * LT
    d_zx = (LT * (one + ux2 * (one - lt_sq)),
            LT * uxuy * (one - t_sq),
            px * T * (one - lt_sq * ux2))
    d_zy = (T * uxuy * (one - lt_sq),
            T * (one + uy2 * (one - t_sq)),
            -(lamv * py * ux2 * t_sq * T))
    z_x = LT * px
    z_y = T * py

    # G = h(p) q(phi) and its derivative with respect to phi.
    phi_lo = np.maximum(phi.lo, 0.0)
    phi_hi = np.maximum(phi.hi, phi_lo)
    phiv = V(phi_lo, phi_hi)
    p_lo = phi_lo + e_1 * phi_lo ** 2 + c * phi_lo ** 3
    p_hi = phi_hi + e_1 * phi_hi ** 2 + c * phi_hi ** 3
    p_lo, p_hi = np.maximum(_dn(p_lo), 0.0), _up(np.maximum(p_hi, p_lo))
    h_hi_value = _h_and_slope(p_lo)[0][1]
    h_lo_value = _h_and_slope(p_hi)[0][0]
    d_lo_value = _h_and_slope(p_lo)[1][0]
    d_hi_value = _h_and_slope(p_hi)[1][1]
    h_val = V(np.minimum(h_lo_value, 1.0), np.minimum(h_hi_value, 1.0))
    h_slope = V(np.minimum(d_lo_value, 0.0), np.minimum(d_hi_value, 0.0))
    q = one + e_1 * phiv + c * square(phiv)
    q_prime = e_1 + (2.0 * c) * phiv
    p_prime = one + (2.0 * e_1) * phiv + (3.0 * c) * square(phiv)
    p_prime = V(np.maximum(p_prime.lo, 0.0), np.maximum(p_prime.hi, 0.0))
    g_val = h_val * q
    g_prime = h_slope * p_prime * q + h_val * q_prime

    def dot(partials, inputs):
        low = np.zeros(inputs[0].tl.shape)
        high = np.zeros(inputs[0].tl.shape)
        for partial, source in zip(partials, inputs):
            term = _imul(partial.lo[..., None], partial.hi[..., None], source.tl, source.th)
            low, high = low + term[0], high + term[1]
        return _widen(low, high)

    def assemble(partials, z_value, old):
        z_tl, z_th = dot(partials, (phi_x, phi_y, lam))
        g_tl, g_th = _imul(g_prime.lo[..., None], g_prime.hi[..., None], phi.tl, phi.th)
        first = _imul(z_value.lo[..., None], z_value.hi[..., None], g_tl, g_th)
        second = _imul(g_val.lo[..., None], g_val.hi[..., None], z_tl, z_th)
        tl, th = _widen(first[0] + second[0], first[1] + second[1])
        bad = ~lam_ok
        tl = np.where(bad[..., None], -np.inf, tl)
        th = np.where(bad[..., None], np.inf, th)
        # The interval-arithmetic tangent of ``old`` and this analytic one both
        # enclose the same derivative, so intersect them.  If rounding leaves a
        # crossed pair, keep the original (valid) one.
        new_lo, new_hi = np.maximum(old.tl, tl), np.minimum(old.th, th)
        crossed = new_lo > new_hi
        return V(old.lo, old.hi, np.where(crossed, old.tl, new_lo),
                 np.where(crossed, old.th, new_hi))

    return assemble(d_zx, z_x, value_x), assemble(d_zy, z_y, value_y)


def _friction_law(speed, mu_0, mu_s, shape, transition, epsilon):
    argument = speed / max(transition, epsilon)
    soft = vlog(argument + vexp(-argument))
    return mu_s + (mu_0 - mu_s) * vexp(-(shape ** 2) * square(soft))


def _loaded_radius(tire, load, epsilon):
    ratio = load / max(tire.f_z_rated, epsilon)
    from_load = tire.r_1 + tire.r_2 * ratio + tire.r_3 * square(ratio)
    shift = tire.f_y_shift_0 + tire.f_y_shift_fz * ratio
    from_lateral = (tire.k_rl_0 + tire.k_rl_1 * ratio + tire.k_rl_2 * square(ratio)) * square(
        0.0 - shift)
    return vmax(from_load + from_lateral, 1.0e-4)


def load_map_enclosure(vehicle, states_lo, states_hi, guess_lo, guess_hi, speed_floor=1.0,
                       tangents=True, state_tangents=False):
    """Enclose the load map; see :func:`_load_map_enclosure`."""

    global _NT
    previous = _NT
    _NT = 10 if state_tangents else 2
    try:
        return _load_map_enclosure(vehicle, states_lo, states_hi, guess_lo, guess_hi,
                                   speed_floor, tangents, state_tangents)
    finally:
        _NT = previous


def _load_map_enclosure(vehicle, states_lo, states_hi, guess_lo, guess_hi, speed_floor,
                        tangents, state_tangents):
    """Return ``(g_x, g_y)`` as :class:`V` of shape ``(m, 1)`` with guess tangents.

    ``states_*`` have shape ``(m, 12)`` or ``(m, 13)``; only ``v_x, v_y, r``, the
    wheel speeds and the steering angle enter.  ``guess_*`` have shape ``(m, 2)``.
    """
    from simplified_vehicle_models.formulations.planar_7dof import wheel_positions
    from simplified_vehicle_models.tires.unitire_simple import _EPSILON as epsilon

    tire, aero = vehicle.tire, vehicle.aero
    positions = wheel_positions(vehicle)
    lo, hi = np.asarray(states_lo, float), np.asarray(states_hi, float)
    guess_lo, guess_hi = np.asarray(guess_lo, float), np.asarray(guess_hi, float)
    m = lo.shape[0]

    def state(index, direction):
        value = V(lo[:, index:index + 1], hi[:, index:index + 1])
        if state_tangents:
            value.tl[..., direction] = 1.0
            value.th[..., direction] = 1.0
        return value

    v_x, v_y, yaw = state(3, 2), state(4, 3), state(5, 4)
    omega = V(lo[:, 6:10], hi[:, 6:10])
    if state_tangents:
        for wheel in range(4):
            omega.tl[:, wheel, 5 + wheel] = 1.0
            omega.th[:, wheel, 5 + wheel] = 1.0

    def guess(index):
        value = V(guess_lo[:, index:index + 1], guess_hi[:, index:index + 1])
        if tangents:
            value.tl[..., index] = 1.0
            value.th[..., index] = 1.0
        return value

    a_x, a_y = guess(0), guess(1)

    # Wheel steering angles: the two front wheels steer, the rear wheels do not.
    cos_lo, cos_hi = np.ones((m, 4)), np.ones((m, 4))
    sin_lo, sin_hi = np.zeros((m, 4)), np.zeros((m, 4))
    (c_lo, c_hi), (s_lo, s_hi) = _trig(lo[:, 11], hi[:, 11])
    for wheel in (0, 1):
        cos_lo[:, wheel], cos_hi[:, wheel] = c_lo, c_hi
        sin_lo[:, wheel], sin_hi[:, wheel] = s_lo, s_hi
    cos_a, sin_a = V(cos_lo, cos_hi), V(sin_lo, sin_hi)
    if state_tangents:
        # d cos / d steering = -sin and d sin / d steering = cos on the front wheels.
        for wheel in (0, 1):
            cos_a.tl[:, wheel, 9], cos_a.th[:, wheel, 9] = -s_hi, -s_lo
            sin_a.tl[:, wheel, 9], sin_a.th[:, wheel, 9] = c_lo, c_hi

    # Contact velocities in tire axes.
    body_long = v_x - yaw * positions[:, 1]
    body_lat = v_y + yaw * positions[:, 0]
    v_long = body_long * cos_a + body_lat * sin_a
    v_lat = body_lat * cos_a - body_long * sin_a
    speed = vsqrt(square(v_x) + square(v_y))

    # Aerodynamics and axle loads.
    dynamic_pressure = 0.5 * aero.air_density * square(speed)
    downforce = dynamic_pressure * aero.downforce_area
    drag = dynamic_pressure * aero.drag_area
    mass = vehicle.total_mass
    front_axle = (mass * GRAVITY * vehicle.front_mass_fraction + downforce * aero.aero_balance
                  - mass * a_x * vehicle.cg_height / vehicle.wheelbase)
    rear_axle = mass * GRAVITY + downforce - front_axle

    # Lateral transfer with the lifted-wheel saturation of ``quasi_static``.
    total_transfer = mass * a_y * vehicle.cg_height / vehicle.track_width
    front_transfer = total_transfer * vehicle.lateral_load_transfer_distribution
    rear_transfer = total_transfer - front_transfer
    front_cap = vmax(0.5 * front_axle, 0.0)
    rear_cap = vmax(0.5 * rear_axle, 0.0)
    plain_front, plain_rear = front_transfer, rear_transfer
    for _ in range(2):
        clamped = vclamp(front_transfer, -front_cap, front_cap)
        rear_transfer = rear_transfer + (front_transfer - clamped)
        front_transfer = clamped
        clamped = vclamp(rear_transfer, -rear_cap, rear_cap)
        front_transfer = front_transfer + (rear_transfer - clamped)
        rear_transfer = clamped
    # When the whole box keeps both transfers strictly inside their caps, no
    # clamp is active, ``x - clamp(x)`` is exactly zero, and the sequence above
    # returns the unclamped transfers.  Interval arithmetic cannot see that
    # cancellation, so use the unclamped expression for those boxes.  The
    # condition is on the enclosure, hence it holds for every state and guess in
    # the box; the other boxes keep the general (wider) expression.
    inactive = ((plain_front.hi <= front_cap.lo) & (plain_front.lo >= -front_cap.lo)
                & (plain_rear.hi <= rear_cap.lo) & (plain_rear.lo >= -rear_cap.lo))
    front_transfer = _select(inactive, plain_front, front_transfer)
    rear_transfer = _select(inactive, plain_rear, rear_transfer)
    half_front, half_rear = 0.5 * front_axle, 0.5 * rear_axle
    loads = _concat([half_front + front_transfer, half_front - front_transfer,
                     half_rear + rear_transfer, half_rear - rear_transfer])

    # Slip quantities use the radius at the raw (unclamped) load.
    raw_radius = _loaded_radius(tire, loads, epsilon)
    denominator = vmax(vabs(v_long), speed_floor)
    s_x = (raw_radius * omega - v_long) / denominator
    s_y = (0.0 - v_lat) / denominator  # tan(-atan2(v_lat, d)) = -v_lat / d for d > 0

    # UniTire force with the load clamped to compressive values.
    f_z = vmax(loads, 0.0)
    f_zn = f_z / max(tire.f_z_rated, epsilon)
    radius = _loaded_radius(tire, f_z, epsilon)
    k_x = _polyval(tire.k_x, f_zn)
    k_y = _polyval(tire.k_y, f_zn)
    v_roll = vmax(vabs(omega) * radius, tire.vroll_min)
    mu_x = _friction_law(vabs(s_x) * v_roll, tire.mu_0_x, tire.mu_s_x, tire.h_x, tire.v_m_x, epsilon)
    mu_y = _friction_law(vabs(s_y) * v_roll, tire.mu_0_y, tire.mu_s_y, tire.h_y, tire.v_m_y, epsilon)
    phi_x = k_x * s_x / vmax(mu_x * f_z, tire.force_eps)
    phi_y = k_y * s_y / vmax(mu_y * f_z, tire.force_eps)
    phi = vhypot(phi_x, phi_y)
    f_bar = _saturation(phi, tire.e_1)
    lam = 1.0 + (k_y / vmax(k_x, tire.stiff_eps) - 1.0) * f_bar
    scaled_x = lam * phi_x
    den = vhypot(scaled_x, phi_y)

    linear_everywhere = den.hi < tire.slip_eps
    nonlinear_everywhere = den.lo >= tire.slip_eps
    safe_den = vmax(den, tire.slip_eps)
    unit_x = _value_only(scaled_x / safe_den, -1.0, 1.0)
    unit_y = _value_only(phi_y / safe_den, -1.0, 1.0)
    unit_force_x, unit_force_y = _smooth_unit_force_tangent(
        phi_x, phi_y, phi, lam, tire.e_1, f_bar * unit_x, f_bar * unit_y)
    force_x_nl = unit_force_x * mu_x * f_z
    force_y_nl = unit_force_y * mu_y * f_z
    force_x_lin = k_x * s_x
    force_y_lin = k_y * s_y

    def blend(nonlinear, linear):
        hull = V(np.minimum(nonlinear.lo, linear.lo), np.maximum(nonlinear.hi, linear.hi),
                 np.minimum(nonlinear.tl, linear.tl), np.maximum(nonlinear.th, linear.th))
        return _select(linear_everywhere, linear, _select(nonlinear_everywhere, nonlinear, hull))

    force_x = blend(force_x_nl, force_x_lin)
    force_y = blend(force_y_nl, force_y_lin)
    body_x = force_x * cos_a - force_y * sin_a
    body_y = force_x * sin_a + force_y * cos_a
    return (_sum_wheels(body_x) - drag) / mass, _sum_wheels(body_y) / mass


def certify_load_closure(vehicle, bounds, node_guesses, speed_floor=1.0,
                         radius_scales=(1.0, 4.0, 16.0, 64.0)):
    """Krawczyk certificate of the load closure on every dense interval.

    ``bounds`` is the ``(n, states, 2)`` dense enclosure and ``node_guesses`` the
    ``(n + 1, 2)`` converged guesses ``(a_x, a_y)`` at the accepted nodes.  For
    each interval the guess box is centred between the two node guesses and
    grown until the Krawczyk operator maps it strictly into itself.  Returns a
    dictionary; ``certified`` is true only when every interval passed.
    """
    bounds = np.asarray(bounds, float)
    nodes = np.asarray(node_guesses, float)
    if bounds.ndim != 3 or bounds.shape[2] != 2 or bounds.shape[0] == 0 \
            or nodes.shape != (bounds.shape[0] + 1, 2) \
            or not np.all(np.isfinite(bounds)) or not np.all(np.isfinite(nodes)):
        return {"certified": False, "reason": "inputs do not match or are non-finite"}
    return _certify_boxes(vehicle, bounds, nodes[:-1], nodes[1:], speed_floor, radius_scales)


def _certify_boxes(vehicle, bounds, guess_a, guess_b, speed_floor, radius_scales):
    """Krawczyk test on boxes with per-box guess endpoints ``guess_a``/``guess_b``."""
    bounds = np.asarray(bounds, float)
    n = bounds.shape[0]
    lo, hi = bounds[:, :, 0], bounds[:, :, 1]
    center = 0.5 * (guess_a + guess_b)
    half = 0.5 * (hi - lo)
    half_widths = np.concatenate([half[:, 3:10], half[:, 11:12]], axis=1)  # vx vy r w*4 steer
    middle_state = 0.5 * (lo + hi)

    def centered_residual(index, guess):
        """Enclose F(guess, x) over the state box with the mean-value form."""
        with np.errstate(all="ignore"):
            point = load_map_enclosure(vehicle, middle_state[index], middle_state[index],
                                       guess, guess, speed_floor, tangents=False)
            slopes = load_map_enclosure(vehicle, lo[index], hi[index], guess, guess,
                                        speed_floor, tangents=False, state_tangents=True)
            low, high = [], []
            for k in range(2):
                magnitude = np.maximum(np.abs(slopes[k].tl[:, 0, 2:]), np.abs(slopes[k].th[:, 0, 2:]))
                extra = np.sum(magnitude * half_widths[index], axis=1) * (1.0 + 1.0e-12)
                low.append(_dn(point[k].lo[:, 0] - extra - guess[:, k]))
                high.append(_up(point[k].hi[:, 0] + extra - guess[:, k]))
        return np.stack(low, axis=-1), np.stack(high, axis=-1)

    everything = np.arange(n)
    # Krawczyk is sound for any centre, so move the centre to a Newton-corrected
    # point first: the interpolated node guess can sit well off the root of the
    # box's own mid state, and then only an over-wide guess box would contain
    # the image.  Two steps on the mid-state point map; failures leave the
    # original centre.
    for _ in range(2):
        with np.errstate(all="ignore"):
            rad = np.maximum(1.0e-7 * (1.0 + np.abs(center)), 1.0e-9)
            at = load_map_enclosure(vehicle, middle_state, middle_state, center - rad,
                                    center + rad, speed_floor)
            mid_g = np.stack([0.5 * (at[k].lo[:, 0] + at[k].hi[:, 0]) for k in range(2)], axis=-1)
            residual_mid = mid_g - center
            jac_mid = np.empty((n, 2, 2))
            for k in range(2):
                for j in range(2):
                    jac_mid[:, k, j] = 0.5 * (at[k].tl[:, 0, j] + at[k].th[:, 0, j]) \
                        - (1.0 if k == j else 0.0)
            determinant_mid = jac_mid[:, 0, 0] * jac_mid[:, 1, 1] - jac_mid[:, 0, 1] * jac_mid[:, 1, 0]
            usable = np.isfinite(determinant_mid) & (np.abs(determinant_mid) > 1.0e-9) \
                & np.isfinite(residual_mid).all(axis=1)
            safe_det = np.where(usable, determinant_mid, 1.0)
            step = np.stack([
                (jac_mid[:, 1, 1] * residual_mid[:, 0] - jac_mid[:, 0, 1] * residual_mid[:, 1]) / safe_det,
                (-jac_mid[:, 1, 0] * residual_mid[:, 0] + jac_mid[:, 0, 0] * residual_mid[:, 1]) / safe_det,
            ], axis=-1)
        center = np.where(usable[:, None], center - step, center)
    centered_lo, centered_hi = centered_residual(everything, center)
    centered_spread = np.where(np.isfinite(centered_hi - centered_lo),
                               centered_hi - centered_lo, 0.0)
    base = (0.5 * np.abs(guess_b - guess_a) + 0.5 * centered_spread
            + 1.0e-7 * (1.0 + np.abs(center)))
    passed = np.zeros(n, dtype=bool)
    used = np.full(n, np.nan)
    contraction = np.full(n, np.inf)
    determinant = np.full(n, np.nan)
    last_row = np.full(n, np.nan)
    last_residual = np.full(n, np.nan)
    last_scale = np.full(n, np.nan)
    eye = np.eye(2)
    for scale in radius_scales:
        todo = np.flatnonzero(~passed)
        if todo.size == 0:
            break
        count = todo.size
        c = center[todo]
        radius = np.maximum(base[todo] * scale, 1.0e-7 * scale)
        with np.errstate(all="ignore"):
            boxed = load_map_enclosure(vehicle, lo[todo], hi[todo], c - radius, c + radius,
                                       speed_floor)
            f_lo, f_hi = centered_lo[todo], centered_hi[todo]
            # Interval Jacobian of F = g - a, entry [k, j] = dg_k/da_j - delta_kj.
            jac_lo = np.empty((count, 2, 2))
            jac_hi = np.empty((count, 2, 2))
            for k in range(2):
                for j in range(2):
                    jac_lo[:, k, j] = boxed[k].tl[:, 0, j] - eye[k, j]
                    jac_hi[:, k, j] = boxed[k].th[:, 0, j] - eye[k, j]
            middle = 0.5 * (jac_lo + jac_hi)
            det_mid = middle[:, 0, 0] * middle[:, 1, 1] - middle[:, 0, 1] * middle[:, 1, 0]
            ok = (np.isfinite(middle).all(axis=(1, 2)) & np.isfinite(det_mid)
                  & (np.abs(det_mid) > 1.0e-9))
            safe = np.where(ok, det_mid, 1.0)
            inverse = np.empty_like(middle)
            inverse[:, 0, 0] = middle[:, 1, 1] / safe
            inverse[:, 1, 1] = middle[:, 0, 0] / safe
            inverse[:, 0, 1] = -middle[:, 0, 1] / safe
            inverse[:, 1, 0] = -middle[:, 1, 0] / safe
            inverse = np.where(ok[:, None, None], inverse, eye)
            # K = c - C F(c) + (I - C J)(A - c), evaluated row by row.
            k_lo = np.empty((count, 2))
            k_hi = np.empty((count, 2))
            worst = np.zeros(count)
            for k in range(2):
                low, high = c[:, k].copy(), c[:, k].copy()
                for j in range(2):
                    term = _imul(inverse[:, k, j], inverse[:, k, j], f_lo[:, j], f_hi[:, j])
                    low, high = low - term[1], high - term[0]
                row_sum = np.zeros(count)
                for column in range(2):
                    e_lo = np.full(count, eye[k, column])
                    e_hi = e_lo.copy()
                    for j in range(2):
                        term = _imul(inverse[:, k, j], inverse[:, k, j],
                                     jac_lo[:, j, column], jac_hi[:, j, column])
                        e_lo, e_hi = e_lo - term[1], e_hi - term[0]
                    row_sum += np.maximum(np.abs(e_lo), np.abs(e_hi))
                    spread = _imul(e_lo, e_hi, -radius[:, column], radius[:, column])
                    low, high = low + spread[0], high + spread[1]
                k_lo[:, k], k_hi[:, k] = _dn(low), _up(high)
                worst = np.maximum(worst, row_sum)
            inside = np.all((k_lo > c - radius) & (k_hi < c + radius), axis=1)
        success = ok & inside & np.isfinite(k_lo).all(axis=1) & np.isfinite(k_hi).all(axis=1)
        last_row[todo] = worst
        last_residual[todo] = np.max(f_hi - f_lo, axis=1)
        last_scale[todo] = scale
        passed[todo[success]] = True
        used[todo[success]] = scale
        contraction[todo[success]] = worst[success]
        determinant[todo[success]] = np.abs(det_mid[success])
    return {
        "certified": bool(passed.all()),
        "passed_mask": passed,
        "intervals": int(n),
        "intervals_failed": int((~passed).sum()),
        "failed_indices": [int(i) for i in np.flatnonzero(~passed)[:20]],
        "largest_radius_scale": float(np.nanmax(used)) if passed.any() else float("nan"),
        "worst_krawczyk_contraction": float(np.max(contraction[passed])) if passed.any()
        else float("inf"),
        "smallest_residual_jacobian_determinant": float(np.nanmin(determinant))
        if passed.any() else float("nan"),
        "details": {"row_sum": last_row, "residual_width": last_residual, "scale": last_scale,
                    "passed": passed, "centered_width": centered_spread},
        "scope": "unique root of the load closure inside the guess box around the numerical "
                 "solution, for every state in the dense box; outward ulp widening, not "
                 "directed rounding; says nothing about roots outside the box",
    }


def certify_load_closure_adaptive(vehicle, polynomials, node_guesses, speed_floor=1.0,
                                  radius_scales=(1.0, 4.0, 16.0, 64.0), maximum_depth=14,
                                  minimum_span=1.0e-9, components=slice(0, 12),
                                  progress=None):
    """Krawczyk certificate with adaptive time bisection of the dense intervals.

    Every dense interval ``[times[i], times[i + 1]]`` must be covered by
    sub-spans that all pass the Krawczyk test.  A sub-span that fails is bisected
    in time and its two halves are retried (a Bernstein hull over a shorter span
    is tighter, so the state box shrinks), down to ``maximum_depth`` halvings.
    The cover is exact: halves share their midpoint fraction, so no time is left
    out.  A span still failing at the maximum depth makes ``certified`` false.

    ``polynomials`` is a :class:`DensePolynomials`; ``node_guesses`` holds the
    converged guesses ``(a_x, a_y)`` at the ``n + 1`` accepted nodes and only
    centres the guess boxes (the certificate is sound for any centre).
    """
    from simplified_vehicle_models.solvers.dense_domain import dense_subspan_bounds

    nodes = np.asarray(node_guesses, float)
    n = polynomials.n_intervals
    if isinstance(components, slice):
        components = np.arange(polynomials.n_states)[components]
    if nodes.shape != (n + 1, 2) or not np.all(np.isfinite(nodes)):
        return {"certified": False, "reason": "inputs do not match or are non-finite"}
    index = np.arange(n)
    start = np.zeros(n)
    stop = np.ones(n)
    depth = 0
    certified_spans = 0
    certified_time = 0.0
    history = []
    deepest = 0
    max_depth_failures = 0
    failed_intervals = np.zeros(0, dtype=int)
    failed_starts = np.zeros(0)
    scale_used = []
    while index.size:
        bounds = dense_subspan_bounds(polynomials, index, start, stop, components)
        guess_a = nodes[index] + (nodes[index + 1] - nodes[index]) * start[:, None]
        guess_b = nodes[index] + (nodes[index + 1] - nodes[index]) * stop[:, None]
        finite = np.isfinite(bounds).all(axis=(1, 2))
        passed = np.zeros(index.size, dtype=bool)
        if finite.any():
            outcome = _certify_boxes(vehicle, bounds[finite], guess_a[finite], guess_b[finite],
                                     speed_floor, radius_scales)
            passed[np.flatnonzero(finite)] = outcome["passed_mask"]
            if outcome["passed_mask"].any():
                scale_used.append(outcome["largest_radius_scale"])
        certified_spans += int(passed.sum())
        history.append({"depth": depth, "spans": int(index.size), "passed": int(passed.sum())})
        keep = ~passed
        index, start, stop = index[keep], start[keep], stop[keep]
        if index.size == 0:
            break
        if depth >= maximum_depth or np.min(stop - start) <= minimum_span:
            max_depth_failures = int(index.size)
            failed_intervals = np.unique(index)
            failed_starts = start.copy()
            break
        middle = 0.5 * (start + stop)
        index = np.concatenate([index, index])
        new_start = np.concatenate([start, middle])
        new_stop = np.concatenate([middle, stop])
        start, stop = new_start, new_stop
        depth += 1
        deepest = depth
        if progress is not None:
            progress(history[-1])
    return {
        "certified": bool(max_depth_failures == 0 and index.size == 0),
        "dense_intervals": int(n),
        "spans_certified": int(certified_spans),
        "spans_failed_at_maximum_depth": int(max_depth_failures),
        "failed_intervals": [int(i) for i in failed_intervals[:50]],
        "failed_interval_count": int(failed_intervals.size),
        "deepest_bisection": int(deepest),
        "maximum_depth": int(maximum_depth),
        "largest_radius_scale": float(max(scale_used)) if scale_used else float("nan"),
        "levels": history,
        "scope": "for every state in the Bernstein hull of the dense polynomial over each "
                 "sub-span, a unique root of the load closure lies in a guess box centred on the "
                 "numerical solution; outward ulp widening, not directed rounding; roots outside "
                 "the box are not excluded; the polynomial is the interpolant, not the true ODE "
                 "solution",
    }
