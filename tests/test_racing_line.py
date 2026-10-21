import math
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np

from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.racing_line import _periodic_smooth, offset_path, path_from_solution
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

    def test_periodic_smoothing_support_may_exceed_unique_nodes(self):
        values = np.array([0.0, 1.0, 0.0, -1.0])
        smoothed = _periodic_smooth(values, sigma_samples=20.0)
        self.assertEqual(smoothed.shape, values.shape)
        self.assertTrue(np.all(np.isfinite(smoothed)))
        np.testing.assert_allclose(_periodic_smooth(np.full(4, 2.0), 20.0), 2.0)

    def test_offset_length_must_match(self):
        track = self._circle()
        with self.assertRaises(ValueError):
            offset_path(track.path_coordinates, np.zeros(3))

    def test_path_from_solution_samples_open_track_without_wrapping(self):
        from tools.tracks.open_course import arc_by_angle, build_course, straight, write_course

        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        course = build_course([straight(10.0), arc_by_angle(30.0, 0.4), straight(5.0)], allow_curvature_jumps=True)
        xml_path, _ = write_course(course, Path(directory.name), "open", "open-test", "Open test")
        track = load_track(xml_path)
        solution = SimpleNamespace(
            offset=np.full(track.distance.size, 1.0),
            distance=np.asarray(track.distance),
        )
        path = path_from_solution(track, solution)
        self.assertFalse(path.is_closed)
        self.assertEqual(path.distance.shape, track.distance.shape)
        np.testing.assert_allclose(path.distance[0], 0.0)
        smooth = path_from_solution(track, solution, smoothing=0.5)
        np.testing.assert_allclose(smooth.x_north, path.x_north, atol=1e-9)

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
        broad_smooth = path_from_solution(track, solution, smoothing=100.0)
        self.assertEqual(broad_smooth.distance.shape, track.distance.shape)
        self.assertAlmostEqual(broad_smooth.curvature[0], broad_smooth.curvature[-1], places=14)


if __name__ == "__main__":
    unittest.main()
