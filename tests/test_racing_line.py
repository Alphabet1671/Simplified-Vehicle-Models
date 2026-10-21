import math
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np

from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.racing_line import offset_path, path_from_solution
from tests.test_track_width import _write_circle_track

NODES = 720


class OffsetPathTest(unittest.TestCase):
    def _circle(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        return load_track(_write_circle_track(Path(directory.name), radius=40.0, nodes=NODES, half_width=3.0))

    def test_positive_offset_on_right_hand_circle_is_inside(self):
        track = self._circle()
        path = offset_path(track.path_coordinates, np.full(track.distance.shape, 3.0))
        np.testing.assert_allclose(path.curvature, 1.0 / 37.0, rtol=1.0e-4)
        self.assertLess(path.lap_length, track.lap_length)

    def test_negative_offset_widens_radius(self):
        track = self._circle()
        path = offset_path(track.path_coordinates, np.full(track.distance.shape, -2.0))
        np.testing.assert_allclose(path.curvature, 1.0 / 42.0, rtol=1.0e-4)
        self.assertGreater(path.lap_length, track.lap_length)

    def test_zero_offset_reproduces_centerline_and_seam_curvature(self):
        track = self._circle()
        path = offset_path(track.path_coordinates, np.zeros(track.distance.shape))
        np.testing.assert_allclose(path.x_north, track.path_coordinates.x_north)
        np.testing.assert_allclose(path.y_east, track.path_coordinates.y_east)
        np.testing.assert_allclose(path.curvature, 1.0 / 40.0, rtol=1.0e-4)
        self.assertAlmostEqual(path.curvature[0], path.curvature[-1], places=14)

    def test_offset_length_must_match(self):
        track = self._circle()
        with self.assertRaises(ValueError):
            offset_path(track.path_coordinates, np.zeros(3))

    def test_path_from_solution_samples_periodically_to_track_grid(self):
        track = self._circle()
        solution = SimpleNamespace(
            offset=np.full(101, 2.0),
            distance=np.linspace(0.0, track.lap_length, 101),
        )
        path = path_from_solution(track, solution)
        self.assertEqual(path.distance.shape, track.distance.shape)
        np.testing.assert_allclose(path.curvature, 1.0 / 38.0, rtol=1.0e-4)
        smooth = path_from_solution(track, solution, smoothing=0.5)
        np.testing.assert_allclose(smooth.x_north, path.x_north, atol=1e-10)


if __name__ == "__main__":
    unittest.main()
