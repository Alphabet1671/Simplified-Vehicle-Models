"""Explicit full-course topology views for boundary-condition experiments."""
from __future__ import annotations

from dataclasses import replace

import numpy as np

from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve
from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile


def _snapshot(values):
    result = np.array(values, copy=True)
    result.setflags(write=False)
    return result


def cut_open_track(track: TrackProfile) -> TrackProfile:
    """Return an open-domain full-lap snapshot with the original periodic fit."""
    if not isinstance(track, TrackProfile):
        raise TypeError("track must be a TrackProfile")
    coordinates = track.path_coordinates
    if not track.is_closed or coordinates is None or not coordinates.is_closed:
        raise ValueError("cut_open_track requires a closed signed-geometry track")
    if track.source_reference_interpolation_periodic not in (None, True):
        raise ValueError("source reference metadata conflicts with a closed source")
    if coordinates.distance.shape != track.distance.shape or not np.allclose(
        coordinates.distance, track.distance, rtol=0.0, atol=1e-9
    ):
        raise ValueError("path station grid must match the complete track source grid")
    source_curve = ReferenceCurve(track)
    if not np.isclose(track.distance[0], 0.0, rtol=0.0, atol=1e-10):
        raise ValueError("full-course cut requires source station to start at zero")
    if not np.isclose(track.distance[-1], track.lap_length, rtol=0.0, atol=1e-10):
        raise ValueError("full-course cut must cover the complete track")
    if not np.isclose(coordinates.distance[0], 0.0, rtol=0.0, atol=1e-10):
        raise ValueError("full-course cut requires path station to start at zero")
    arrays = {name: _snapshot(getattr(track, name)) for name in
              ("distance", "segment_length", "radius", "curvature")}
    path = PathCoordinates(
        distance=_snapshot(coordinates.distance),
        x_north=_snapshot(coordinates.x_north), y_east=_snapshot(coordinates.y_east),
        heading=_snapshot(coordinates.heading), curvature=_snapshot(coordinates.curvature),
        is_closed=False,
        reference_distance=(None if coordinates.reference_distance is None
                           else _snapshot(coordinates.reference_distance)),
    )
    left = None if track.half_width_left is None else _snapshot(track.half_width_left)
    right = None if track.half_width_right is None else _snapshot(track.half_width_right)
    # Validation above intentionally builds the original periodic source fit;
    # the result opts into that same fit while retaining an open query domain.
    assert source_curve.interpolation_periodic
    return replace(track, **arrays, path_coordinates=path,
                   half_width_left=left, half_width_right=right,
                   is_closed=False, source_reference_interpolation_periodic=True)
