"""Cubic offset basis with periodic and open boundary operators.

The spline interpolates a lateral offset at equally spaced control distances
along a track.  A closed track uses a periodic cubic spline, so the offset and
its first two derivatives match across the start/finish seam.  An open course
uses a natural cubic spline, whose endpoint values are free and whose second
derivative vanishes at both ends.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.interpolate import CubicSpline


@dataclass(frozen=True, slots=True)
class OffsetSpline:
    """Map lateral offsets at control points to a smooth track offset field.

    ``control_distance`` holds the spacing of the control points.  A closed
    track excludes the repeated terminal point, so the control points sample
    one period ``[0, lap_length)``.  An open course includes both endpoints.
    """

    track: object
    control_spacing: float
    control_distance: np.ndarray = field(init=False, repr=False, compare=False)
    control_count: int = field(init=False, repr=False, compare=False)
    is_closed: bool = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        if not np.isfinite(self.control_spacing) or self.control_spacing <= 0.0:
            raise ValueError("control_spacing must be finite and positive")
        count = max(4, int(round(self.track.lap_length / self.control_spacing)))
        if self.track.is_closed:
            positions = np.linspace(
                0.0, self.track.lap_length, count, endpoint=False
            )
        else:
            positions = np.linspace(
                0.0, self.track.lap_length, count, endpoint=True
            )
        positions.setflags(write=False)
        object.__setattr__(self, "control_distance", positions)
        object.__setattr__(self, "control_count", int(positions.size))
        object.__setattr__(self, "is_closed", bool(self.track.is_closed))

    def _spline(self, coefficients: object) -> CubicSpline:
        coefficients = np.asarray(coefficients, dtype=float)
        if coefficients.shape != (self.control_count,):
            raise ValueError("coefficients must match control_count")
        if self.is_closed:
            # Duplicate the first control value at the period end so the
            # periodic condition operates on a genuinely periodic table.
            positions = np.r_[self.control_distance, self.track.lap_length]
            values = np.r_[coefficients, coefficients[0]]
            return CubicSpline(positions, values, bc_type="periodic")
        return CubicSpline(self.control_distance, coefficients, bc_type="natural")

    def basis(
        self, distance: object
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return the offset, first, and second derivative basis matrices.

        Every matrix has shape ``(len(distance), control_count)``.  Column
        ``j`` is the spline induced by a unit offset at control point ``j``.
        """

        positions = np.asarray(distance, dtype=float)
        values = np.eye(self.control_count)
        columns = [
            self._spline(values[:, j])(positions) for j in range(self.control_count)
        ]
        first = [
            self._spline(values[:, j])(positions, 1)
            for j in range(self.control_count)
        ]
        second = [
            self._spline(values[:, j])(positions, 2)
            for j in range(self.control_count)
        ]
        return np.column_stack(columns), np.column_stack(first), np.column_stack(second)

    def evaluate(
        self, coefficients: object, distance: object
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return the offset, first, and second derivative at ``distance``."""

        spline = self._spline(coefficients)
        positions = np.asarray(distance, dtype=float)
        return spline(positions), spline(positions, 1), spline(positions, 2)
