"""Analytic helper for constant-acceleration motion from rest."""
from __future__ import annotations

import math
from numbers import Real


def analytic_constant_acceleration_rest_to_distance(
    distance_m: Real, acceleration_mps2: Real
) -> tuple[float, float]:
    """Return travel time and terminal speed for motion from rest.

    Both inputs must be positive, finite real numbers. Booleans are rejected
    even though Python treats them as integers.
    """
    if (
        isinstance(distance_m, bool)
        or not isinstance(distance_m, Real)
        or not math.isfinite(distance_m)
        or distance_m <= 0
    ):
        raise ValueError("distance_m must be a finite positive real number")
    if (
        isinstance(acceleration_mps2, bool)
        or not isinstance(acceleration_mps2, Real)
        or not math.isfinite(acceleration_mps2)
        or acceleration_mps2 <= 0
    ):
        raise ValueError("acceleration_mps2 must be a finite positive real number")

    return (
        math.sqrt(2.0 * distance_m / acceleration_mps2),
        math.sqrt(2.0 * acceleration_mps2 * distance_m),
    )
