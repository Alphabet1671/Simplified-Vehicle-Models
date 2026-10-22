"""Tests for the controller replay adapter seam."""

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.controls.replay import (
    PointMassReplay,
    build_replay,
    track_from_path,
)
from simplified_vehicle_models.results.traces import LapMode
from simplified_vehicle_models.tracks.profile import load_track
from tests.test_track_width import _write_circle_track


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

    def test_replay_registry_rejects_unknown_names(self):
        with self.assertRaises(KeyError):
            build_replay('not-registered')

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


if __name__ == '__main__':
    unittest.main()
