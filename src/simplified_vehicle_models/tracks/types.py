from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.core.types import FloatArray


@dataclass(frozen=True, slots=True)
class PathCoordinates:
    """Store the signed geometry of one prescribed path.

    The world frame is ``X`` north, ``Y`` east, and ``Z`` down.
    That frame is right handed, so a right turn has positive curvature.
    The heading is unwrapped. It is measured from north toward east, so its
    derivative along the path is the signed curvature.

    A closed path (``is_closed`` true) repeats the first position at the last
    node, so the path is periodic. Its heading changes by one full turn across
    a lap.

    An open path (``is_closed`` false) has a start and an end. Its last node is
    the end of the course and is not a repeat of the first node. Outside the
    course, ``evaluate_path`` continues the path as a straight line along the
    end tangents.
    """

    distance: FloatArray
    x_north: FloatArray
    y_east: FloatArray
    heading: FloatArray
    curvature: FloatArray
    is_closed: bool = True

    @property
    def lap_length(self) -> float:
        """Return the represented lap length, or the course length if open."""

        return float(self.distance[-1] - self.distance[0])


#: How a lateral corridor was obtained.  ``surveyed`` means the boundary was
#: measured independently of the driver.  ``swept`` means it is the union of
#: driven vehicle envelopes and is therefore an inner bound on the drivable
#: area.  ``assumed`` means it is a declared constant used for sensitivity
#: only.  A result produced from a non-surveyed corridor is conditional.
WIDTH_PROVENANCE = ("surveyed", "swept", "assumed")


@dataclass(frozen=True, slots=True)
class TrackProfile:
    """Store a distance and radius track profile.

    ``radius`` is a nonnegative magnitude for the point-mass limit solver.
    ``curvature`` is signed where the source carries a turn direction.
    A legacy ``distance-radius-profile`` track stores a radius magnitude only.
    Its turn direction is unknown, so its per-wheel left and right split is not
    a turn direction. ``path_coordinates`` holds the signed geometry of a
    ``closed-xy-profile`` or ``open-xy-profile`` track and is ``None`` for a
    legacy track.

    ``is_closed`` is false for an open course. An open course has no wrap at
    the end, so a consumer that assumes a periodic lap must check this flag.
    """

    distance: FloatArray
    segment_length: FloatArray
    radius: FloatArray
    curvature: FloatArray
    is_closed: bool
    title: str
    path_coordinates: PathCoordinates | None = None
    half_width_right: FloatArray | None = None
    half_width_left: FloatArray | None = None
    width_provenance: str | None = None

    def __post_init__(self) -> None:
        left, right = self.half_width_left, self.half_width_right
        if (left is None) != (right is None):
            raise ValueError("both half widths are required together")
        if left is None:
            if self.width_provenance is not None:
                raise ValueError("width provenance requires width data")
            return
        if self.width_provenance not in WIDTH_PROVENANCE:
            raise ValueError(
                f"width provenance must be one of {WIDTH_PROVENANCE}, "
                f"not {self.width_provenance!r}"
            )
        for name in ("half_width_left", "half_width_right"):
            values = np.asarray(getattr(self, name), dtype=float)
            if values.shape != self.distance.shape:
                raise ValueError(f"{name} must match the distance grid")
            if not np.all(np.isfinite(values)) or np.any(values <= 0.0):
                raise ValueError(f"{name} must be positive and finite")
            values.setflags(write=False)
            object.__setattr__(self, name, values)

    @property
    def has_edges(self) -> bool:
        """Return true when the track carries a lateral corridor."""

        return self.half_width_left is not None

    @property
    def lap_length(self) -> float:
        """Return the represented lap length."""

        return float(self.distance[-1] - self.distance[0])

    @property
    def has_signed_geometry(self) -> bool:
        """Return true when the track carries a turn direction."""

        return self.path_coordinates is not None
