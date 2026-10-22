"""Replay adapters for controller-in-loop correction; no TV physics here.

A replay adapter drives a rebuilt line with a controller and returns the
achieved samples as a :class:`ReplaySample`.  The baseline
:class:`PointMassReplay` deliberately reuses the quasi-steady point-mass lap
solver: it validates the seam and gives a parity reference, but it is not a
TV (transient-vehicle) controller and must not be reported as one.
"""
from __future__ import annotations

from typing import Protocol

import numpy as np

from simplified_vehicle_models.results.traces import LapMode
from simplified_vehicle_models.solvers.closed_loop_line import ReplaySample
from simplified_vehicle_models.solvers.forward_backward import solve_lap
from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile


def track_from_path(path: PathCoordinates) -> TrackProfile:
    """Build a replay-only track profile from a signed path.

    The profile reuses the path's distance, signed curvature, closure flag, and
    the path object itself, so a replay and its correction share one geometry.
    Grip width is intentionally unset: a replay follows the line, not a
    corridor, and a fabricated width would make downstream consumers believe
    the profile carries edge data.
    """

    distance = np.asarray(path.distance, dtype=float)
    curvature = np.asarray(path.curvature, dtype=float)
    radius = 1.0 / np.maximum(np.abs(curvature), 1.0e-9)
    return TrackProfile(
        distance=distance, segment_length=np.diff(distance), radius=radius,
        curvature=curvature, is_closed=bool(path.is_closed),
        title='line replay', path_coordinates=path,
    )


class ReplayAdapter(Protocol):
    """Callable replay seam: ``replay(path, controller) -> ReplaySample``."""

    def __call__(self, path, controller) -> ReplaySample: ...


class PointMassReplay:
    """Baseline replay without TV; used to validate the seam and parity.

    ``controller`` is accepted for interface parity and ignored, because the
    point-mass lap solver already produces the driven speed profile.
    """

    def __init__(self, vehicle, lap_mode: LapMode, tolerance=1e-6, maximum_iterations=200):
        self.vehicle = vehicle
        self.lap_mode = lap_mode
        self.tolerance = tolerance
        self.maximum_iterations = maximum_iterations

    def __call__(self, path, controller=None):
        track = track_from_path(path)
        lap = solve_lap(self.vehicle, track, self.lap_mode,
                        tolerance=self.tolerance, maximum_iterations=self.maximum_iterations)
        return ReplaySample(track.distance, lap.speed, bool(lap.converged),
                            float(lap.lap_time), tuple(lap.active_longitudinal_constraint))


REPLAY_ADAPTERS = {}


def register_replay(name, factory):
    """Register one replay adapter factory under a non-empty string name."""

    if not isinstance(name, str) or not name:
        raise ValueError('replay adapter name must be a non-empty string')
    REPLAY_ADAPTERS[name] = factory


def build_replay(name, **kwargs):
    """Build a registered replay adapter, rejecting unknown names."""

    if name not in REPLAY_ADAPTERS:
        raise KeyError(f'unknown replay adapter {name!r}')
    return REPLAY_ADAPTERS[name](**kwargs)


register_replay('point-mass', PointMassReplay)
