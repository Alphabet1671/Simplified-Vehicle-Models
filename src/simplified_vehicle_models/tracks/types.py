from __future__ import annotations

from dataclasses import dataclass

from simplified_vehicle_models.core.types import FloatArray


@dataclass(frozen=True, slots=True)
class TrackProfile:
    """Store a distance and radius track profile."""

    distance: FloatArray
    segment_length: FloatArray
    radius: FloatArray
    curvature: FloatArray
    is_closed: bool
    title: str

    @property
    def lap_length(self) -> float:
        """Return the represented lap length."""

        return float(self.distance[-1] - self.distance[0])
