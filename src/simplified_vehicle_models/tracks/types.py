from __future__ import annotations

from dataclasses import dataclass

from simplified_vehicle_models.core.types import FloatArray


@dataclass(frozen=True, slots=True)
class PathCoordinates:
    """Store the signed geometry of one closed prescribed path.

    The world frame is ``X`` north, ``Y`` east, and ``Z`` down.
    That frame is right handed, so a right turn has positive curvature.
    The heading is unwrapped and falls by one full turn across a lap.
    The last node repeats the first position, so the path is periodic.
    """

    distance: FloatArray
    x_north: FloatArray
    y_east: FloatArray
    heading: FloatArray
    curvature: FloatArray

    @property
    def lap_length(self) -> float:
        """Return the represented lap length."""

        return float(self.distance[-1] - self.distance[0])


@dataclass(frozen=True, slots=True)
class TrackProfile:
    """Store a distance and radius track profile.

    ``radius`` is a nonnegative magnitude for the point-mass limit solver.
    ``curvature`` is signed where the source carries a turn direction.
    A legacy ``distance-radius-profile`` track stores a radius magnitude only.
    Its turn direction is unknown, so its per-wheel left and right split is not
    a turn direction. ``path_coordinates`` holds the signed geometry of a
    ``closed-xy-profile`` track and is ``None`` for a legacy track.
    """

    distance: FloatArray
    segment_length: FloatArray
    radius: FloatArray
    curvature: FloatArray
    is_closed: bool
    title: str
    path_coordinates: PathCoordinates | None = None

    @property
    def lap_length(self) -> float:
        """Return the represented lap length."""

        return float(self.distance[-1] - self.distance[0])

    @property
    def has_signed_geometry(self) -> bool:
        """Return true when the track carries a turn direction."""

        return self.path_coordinates is not None
