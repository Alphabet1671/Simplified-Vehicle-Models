"""Tests for the controller replay adapter seam."""

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.controls.replay import (
    REPLAY_ADAPTERS,
    PointMassReplay,
    build_replay,
    track_from_path,
)
from simplified_vehicle_models.results.traces import LapMode
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.types import PathCoordinates
from tests.test_track_width import _write_circle_track


def _open_path() -> PathCoordinates:
    """Return a short straight open path for replay mode checks."""

    distance = np.array([0.0, 5.0, 10.0])
    return PathCoordinates(
        distance=distance, x_north=np.zeros(3), y_east=distance.copy(),
        heading=np.full(3, 0.5 * np.pi), curvature=np.zeros(3), is_closed=False,
    )


class ReplayAdapterTest(unittest.TestCase):
    def test_track_from_path_preserves_distance_curvature_and_topology(self):
        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40.0, nodes=72,
                                                   half_width=3.0))
            rebuilt = track_from_path(track.path_coordinates)
            np.testing.assert_array_equal(rebuilt.distance, track.distance)
            np.testing.assert_array_equal(rebuilt.curvature, track.curvature)
            self.assertTrue(rebuilt.is_closed)
            self.assertIs(rebuilt.path_coordinates, track.path_coordinates)
            self.assertIsNone(rebuilt.half_width_left)
            self.assertIsNone(rebuilt.half_width_right)

    def test_replay_registry_rejects_unknown_names(self):
        with self.assertRaises(KeyError):
            build_replay('not-registered')

    def test_point_mass_is_registered_by_default(self):
        self.assertIn('point-mass', REPLAY_ADAPTERS)
        replay = build_replay('point-mass', vehicle=object(), lap_mode=LapMode.FLYING)
        self.assertIsInstance(replay, PointMassReplay)

    def test_point_mass_replay_wires_the_lap_result(self):
        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40.0, nodes=72,
                                                   half_width=3.0))
        lap = SimpleNamespace(speed=np.full(track.distance.size, 12.0), lap_time=9.5,
                              converged=True, active_longitudinal_constraint=('rear-limit',))
        with patch('simplified_vehicle_models.controls.replay.solve_lap', return_value=lap):
            sample = PointMassReplay(object(), LapMode.FLYING)(track.path_coordinates)
        self.assertEqual(sample.lap_time, 9.5)
        self.assertTrue(sample.feasible)
        np.testing.assert_array_equal(sample.speed, lap.speed)

    def test_open_path_rejects_flying_but_allows_standing_start(self):
        path = _open_path()
        with self.assertRaisesRegex(ValueError, r'open path.*FLYING'):
            PointMassReplay(object(), LapMode.FLYING)(path)
        lap = SimpleNamespace(speed=np.full(path.distance.size, 8.0), lap_time=1.25,
                              converged=True, active_longitudinal_constraint=())
        with patch('simplified_vehicle_models.controls.replay.solve_lap', return_value=lap):
            sample = PointMassReplay(object(), LapMode.STANDING_START)(path)
        self.assertEqual(sample.lap_time, 1.25)
        self.assertTrue(sample.feasible)


if __name__ == '__main__':
    unittest.main()
