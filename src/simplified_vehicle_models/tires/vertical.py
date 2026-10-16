"""Tire vertical load from wheel deflection.

The vertical model needs the tire normal load as a function of the tire
deflection, not the reverse. The steady UniTire gives the loaded radius as a
quadratic in the normalized load ``x = Fz / f_z_rated``::

    r(x) = r_1 + r_2 * x + r_3 * x**2

with the lateral-shift and ``k_rl`` radius corrections equal to zero. The
deflection is ``delta = r_free - r_loaded`` with ``r_free = loaded_radius(0) =
r_1``, so ``delta = -(r_2 * x + r_3 * x**2)``. The elastic load is the exact
closed-form root::

    x = 2 * delta / (-r_2 + sqrt(r_2**2 - 4 * r_3 * delta))

(the numerically stable form of the smaller quadratic root). Taking the root
this way reproduces ``loaded_radius`` to rounding error.

Low-load ramp. The exact law has about 39.7 N at ``delta1 = 0.5 mm`` and is
used above that deflection. Below ``delta1`` the elastic load is a nonnegative
C1 quadratic ramp::

    F1 = F_exact(delta1),  k1 = dF_exact/d(delta) at delta1,
    delta0 = delta1 - 2 * F1 / k1,
    Fs = F1 * ((delta - delta0) / (delta1 - delta0))**2   for delta0 < delta < delta1,
    Fs = 0                                                for delta <= delta0.

Value and slope are continuous at both knots; the second derivative jumps, so
the law is C1 but not C2. The ramp carries about 9.9 N at zero deflection (a
ghost load, no physical meaning) and is exactly zero below ``delta0``
(about -0.5 mm).

Damping. ``F = Fs * (1 + psi(c * delta_rate / (Fs + 1 N)))`` with ``psi(u) = u``
for ``u >= 0`` and ``tanh(u)`` otherwise. A positive ``delta_rate`` is
compression. Because ``tanh > -1``, the load is nonnegative, and it has no
damping where ``Fs = 0``. The law is C1 in the rate at zero. ``contact`` means
the elastic load is positive (``Fs > 0``); at an extreme extension rate the
total load can underflow to zero while ``contact`` stays true.

The returned ``stiffness`` is the elastic (static) ``dFs/d(delta)``. It excludes
the damping term and the damping's own dependence on ``Fs``. An implicit solver
should treat the damping stiffness ``c`` and the ramp slope separately.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.parameters.tire import UniTireSimpleParameters
from simplified_vehicle_models.tires.unitire_simple import loaded_radius

#: Upper knot of the low-load ramp [m].
DEFAULT_BLEND = 5.0e-4
#: Added to the elastic load in the damping denominator [N].
DAMPING_LOAD_FLOOR = 1.0
#: Load up to which the radius law must stay monotone [N].
SUPPORTED_MAXIMUM_LOAD = 10_000.0

_ZERO_COEFFICIENTS = ("f_y_shift_0", "f_y_shift_fz", "k_rl_0", "k_rl_1", "k_rl_2")


@dataclass(frozen=True, slots=True)
class TireVertical:
    """Store the vertical response of the tires, in the shape of the input.

    ``load`` is the nonnegative total normal load [N] including damping,
    ``stiffness`` is the elastic ``dFs/d(delta)`` [N/m], and ``contact`` is
    true where the elastic load is positive.
    """

    load: FloatArray
    stiffness: FloatArray
    contact: np.ndarray


def require_exact_inverse_supported(
    parameters: UniTireSimpleParameters,
    maximum_load: float = SUPPORTED_MAXIMUM_LOAD,
) -> None:
    """Raise ValueError unless the closed-form loaded-radius inverse is valid.

    The inverse needs zero lateral-shift and ``k_rl`` loaded-radius corrections
    and a radius that decreases with load up to ``maximum_load``.
    """

    for name in _ZERO_COEFFICIENTS:
        if getattr(parameters, name) != 0.0:
            raise ValueError(
                f"tire coefficient {name} = {getattr(parameters, name)} must be zero: "
                "the exact vertical-load inverse does not support lateral "
                "loaded-radius corrections"
            )
    if not parameters.f_z_rated > 0.0:
        raise ValueError("f_z_rated must be positive")
    r_2, r_3 = parameters.r_2, parameters.r_3
    x_max = float(maximum_load) / parameters.f_z_rated
    if not (parameters.r_1 > 0.0 and r_2 < 0.0 and r_2 + 2.0 * r_3 * x_max < 0.0):
        raise ValueError(
            "the loaded radius must be positive and decrease with load up to "
            f"{maximum_load} N; r_1={parameters.r_1}, r_2={r_2}, r_3={r_3}"
        )


def _exact_load_and_stiffness(
    parameters: UniTireSimpleParameters,
    delta: FloatArray,
) -> tuple[FloatArray, FloatArray]:
    """Return the exact elastic load and stiffness for ``delta >= 0``."""

    r_2, r_3 = parameters.r_2, parameters.r_3
    discriminant = np.maximum(r_2 * r_2 - 4.0 * r_3 * delta, 0.0)
    x = 2.0 * delta / (-r_2 + np.sqrt(discriminant))
    slope = -(r_2 + 2.0 * r_3 * x)  # -dr/dx > 0
    return parameters.f_z_rated * x, parameters.f_z_rated / slope


def transition_knots(
    parameters: UniTireSimpleParameters,
    blend: float = DEFAULT_BLEND,
) -> tuple[float, float, float, float]:
    """Return ``(delta0, delta1, F1, k1)`` of the low-load ramp.

    ``blend`` is the upper knot ``delta1`` in metres.
    """

    delta1 = float(blend)
    if not (np.isfinite(delta1) and delta1 > 0.0):
        raise ValueError(f"blend must be a positive finite length, received {blend}")
    f1, k1 = _exact_load_and_stiffness(parameters, np.float64(delta1))
    f1, k1 = float(f1), float(k1)
    return delta1 - 2.0 * f1 / k1, delta1, f1, k1


def tire_deflection_load(
    parameters: UniTireSimpleParameters,
    deflection: FloatArray,
    deflection_rate: FloatArray,
    damping: float,
    blend: float = DEFAULT_BLEND,
) -> TireVertical:
    """Return the vertical tire load at a deflection and a deflection rate.

    ``deflection`` is ``r_free - r_loaded`` in metres and
    ``deflection_rate`` is its time derivative in m/s, positive in compression.
    ``damping`` is the tire vertical damping in N*s/m. The inputs broadcast.
    """

    damping = float(damping)
    if not (np.isfinite(damping) and damping >= 0.0):
        raise ValueError(f"tire damping must be finite and nonnegative, received {damping}")
    delta0, delta1, f1, k1 = transition_knots(parameters, blend)
    delta, rate = np.broadcast_arrays(
        np.asarray(deflection, dtype=np.float64),
        np.asarray(deflection_rate, dtype=np.float64),
    )

    exact_load, exact_stiffness = _exact_load_and_stiffness(
        parameters, np.maximum(delta, delta1)
    )
    span = delta1 - delta0
    ramp = np.clip(delta - delta0, 0.0, span)
    ramp_load = f1 * (ramp / span) ** 2
    ramp_stiffness = 2.0 * f1 * ramp / span**2

    above = delta >= delta1
    elastic = np.where(above, exact_load, ramp_load)
    stiffness = np.where(above, exact_stiffness, ramp_stiffness)

    u = damping * rate / (elastic + DAMPING_LOAD_FLOOR)
    psi = np.where(u >= 0.0, u, np.tanh(np.minimum(u, 0.0)))
    load = elastic * (1.0 + psi)
    return TireVertical(load=load, stiffness=stiffness, contact=elastic > 0.0)
