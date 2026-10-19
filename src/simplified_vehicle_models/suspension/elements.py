"""Compiled values of the force-element components a suspension layout nests.

These are plain value holders. A layout (``suspension/layouts``) reads them and
owns the force law.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.suspension.monotone_spline import MonotoneSpline


@dataclass(frozen=True)
class CornerSprings:
    """Wheel-referred spring curves (N against jounce, m) of the front and rear axle.

    Each curve holds the sprung static corner load at zero jounce, so zero
    jounce is the static equilibrium at the design ride height.
    """

    front: MonotoneSpline
    rear: MonotoneSpline

    def axle(self, axle: int) -> MonotoneSpline:
        """Return the curve of axle 0 (front) or 1 (rear)."""

        if axle not in (0, 1):
            raise ValueError("axle must be 0 (front) or 1 (rear)")
        return self.front if axle == 0 else self.rear


@dataclass(frozen=True)
class AxleArbRates:
    """Anti-roll-bar wheel rates (N/m of wheel travel difference) of both axles."""

    front: float
    rear: float

    def __post_init__(self) -> None:
        for name in ("front", "rear"):
            value = getattr(self, name)
            if not (np.isfinite(value) and value >= 0.0):
                raise ValueError(f"arb wheel rate {name} must be finite and nonnegative")

    def axle(self, axle: int) -> float:
        """Return the rate of axle 0 (front) or 1 (rear)."""

        if axle not in (0, 1):
            raise ValueError("axle must be 0 (front) or 1 (rear)")
        return self.front if axle == 0 else self.rear
