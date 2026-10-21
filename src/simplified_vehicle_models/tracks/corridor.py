"""Express track edges as a fold-guarded lateral corridor.

The lateral offset ``n`` is positive to the right of travel. With the
repository's right-handed north/east/down frame, positive curvature is a right
turn and the offset-path fold condition is ``1 - curvature * n > 0``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.core.types import FloatArray


@dataclass(frozen=True, slots=True)
class Corridor:
    """Read-only admissible lateral offsets on the track distance grid."""

    distance: FloatArray
    minimum: FloatArray
    maximum: FloatArray
    provenance: str
    is_closed: bool = True

    def __post_init__(self) -> None:
        for name in ("distance", "minimum", "maximum"):
            values = np.array(getattr(self, name), dtype=float, copy=True)
            values.setflags(write=False)
            object.__setattr__(self, name, values)
        if not (self.distance.shape == self.minimum.shape == self.maximum.shape):
            raise ValueError("corridor arrays must have matching shapes")
        if (not np.all(np.isfinite(self.distance)) or
                not np.all(np.isfinite(self.minimum)) or
                not np.all(np.isfinite(self.maximum))):
            raise ValueError("corridor arrays must be finite")
        if np.any(self.minimum > 0.0) or np.any(self.maximum < 0.0):
            raise ValueError("corridor bounds must contain the centreline")
        if np.any(self.minimum > self.maximum):
            raise ValueError("corridor minimum must not exceed maximum")

    @property
    def half_width(self) -> float:
        """Return the narrowest available one-sided width."""

        return float(min(-np.min(self.minimum), np.min(self.maximum)))

    @property
    def is_periodic(self) -> bool:
        """Whether the endpoint bounds agree, indicating a closed grid."""

        return bool(
            self.is_closed
            and self.distance.size > 1
            and self.distance[-1] > self.distance[0]
            and np.isclose(self.minimum[0], self.minimum[-1], rtol=0.0, atol=1e-12)
            and np.isclose(self.maximum[0], self.maximum[-1], rtol=0.0, atol=1e-12)
        )

    @property
    def is_conditional(self) -> bool:
        """Whether this corridor is not independently surveyed."""

        return self.provenance != "surveyed"


def corridor_from_track(
    track, *, margin: float = 0.0, fold_guard: float = 0.2
) -> Corridor:
    """Build right-positive bounds from track widths and signed curvature."""

    if not getattr(track, "has_edges", False):
        raise ValueError("the track carries no lateral corridor")
    if not np.isfinite(margin) or margin < 0.0:
        raise ValueError("margin must be finite and nonnegative")
    if not np.isfinite(fold_guard) or not 0.0 < fold_guard < 1.0:
        raise ValueError("fold_guard must lie strictly between zero and one")

    distance = np.asarray(track.distance, dtype=float)
    left = np.asarray(track.half_width_left, dtype=float)
    right = np.asarray(track.half_width_right, dtype=float)
    curvature = np.asarray(track.curvature, dtype=float)
    if not all(np.all(np.isfinite(a)) for a in (distance, left, right, curvature)):
        raise ValueError("track corridor inputs must be finite")
    if left.shape != distance.shape or right.shape != distance.shape or curvature.shape != distance.shape:
        raise ValueError("track corridor inputs must match the distance grid")

    minimum = -left + margin
    maximum = right - margin
    # Conservatively bound both sides by the magnitude-based fold limit. This
    # keeps the requested guard even for tracks with legacy/unsigned curvature.
    magnitude = np.abs(curvature)
    limit = np.full(magnitude.shape, np.inf)
    curved = magnitude > 0.0
    limit[curved] = (1.0 - fold_guard) / magnitude[curved]
    maximum = np.minimum(maximum, limit)
    minimum = np.maximum(minimum, -limit)

    if np.any(minimum > 0.0) or np.any(maximum < 0.0):
        raise ValueError("the corridor does not contain the centreline")
    if np.any(minimum > maximum):
        raise ValueError("the corridor is empty")
    return Corridor(
        distance=distance,
        minimum=minimum,
        maximum=maximum,
        provenance=str(track.width_provenance),
        is_closed=bool(track.is_closed),
    )
