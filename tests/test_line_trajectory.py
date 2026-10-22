"""Independent geometry checks for the shared line trajectory."""
from __future__ import annotations

import unittest

import numpy as np

from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory, ReferenceCurve
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile


def _track(distance, north, east, closed=False):
    distance = np.asarray(distance, dtype=float)
    north = np.asarray(north, dtype=float)
    east = np.asarray(east, dtype=float)
    dx = np.gradient(north, distance)
    dy = np.gradient(east, distance)
    heading = np.unwrap(np.arctan2(dy, dx))
    curvature = np.gradient(heading, distance)
    coordinates = PathCoordinates(distance, north, east, heading, curvature, closed)
    return TrackProfile(distance, np.diff(distance), np.ones_like(distance), curvature,
                        closed, "synthetic", coordinates)


def _circle(radius=30.0, count=121):
    s = np.linspace(0.0, 2.0 * np.pi * radius, count)
    theta = s / radius
    # Clockwise in north/east: right turn and positive signed curvature.
    return _track(s, radius * np.cos(theta), radius * np.sin(theta), closed=True)


def _open_curve():
    s = np.linspace(0.0, 18.0, 31)
    return _track(s, s + 0.14 * np.sin(0.7 * s), 0.3 * s ** 2 / 18.0 + 0.4 * np.sin(0.41 * s))


class LineTrajectoryTest(unittest.TestCase):
    def test_reference_circle_metric_and_sign(self):
        track = _circle()
        reference = ReferenceCurve(track)
        sample = reference.evaluate(np.array([13.0, 80.0]))
        self.assertEqual(sample.north.shape, (2,))
        self.assertTrue(np.allclose(sample.metric, 1.0, atol=2e-5))
        self.assertTrue(np.allclose(sample.curvature, 1.0 / 30.0, atol=3e-5))
        scalar = reference.evaluate(13.0)
        self.assertIsInstance(scalar.north, float)

    def test_offset_circle_inward_outward_length_and_sign(self):
        track = _circle()
        reference = ReferenceCurve(track)
        spline = OffsetSpline(track, 20.0)
        length = track.lap_length
        for offset, expected_radius in ((3.0, 27.0), (-3.0, 33.0)):
            trajectory = OffsetTrajectory(reference, spline, np.full(spline.control_count, offset))
            sample = trajectory.evaluate(np.linspace(0.0, length, 100))
            self.assertTrue(np.allclose(sample.metric, expected_radius / 30.0, atol=4e-5))
            self.assertTrue(np.allclose(sample.curvature, 1.0 / expected_radius, atol=5e-5))
            self.assertAlmostEqual(trajectory.station_map.path_station[-1], 2.0 * np.pi * expected_radius, places=3)

    def test_offset_geometry_agrees_with_independent_finite_differences(self):
        track = _open_curve()
        reference = ReferenceCurve(track)
        spline = OffsetSpline(track, 4.0)
        coefficients = np.linspace(-0.3, 0.45, spline.control_count)
        trajectory = OffsetTrajectory(reference, spline, coefficients, station_accuracy=2e-9)
        s = np.linspace(1.0, 17.0, 97)
        sample = trajectory.evaluate(s)
        eps = 1e-4
        before = trajectory.evaluate(s - eps)
        after = trajectory.evaluate(s + eps)
        dx = (after.north - before.north) / (2.0 * eps)
        dy = (after.east - before.east) / (2.0 * eps)
        self.assertTrue(np.allclose(np.hypot(dx, dy), sample.metric, rtol=2e-5, atol=2e-6))
        self.assertTrue(np.allclose(np.arctan2(dy, dx), sample.heading, atol=2e-5))
        heading_rate = np.unwrap(after.heading - before.heading) / (2.0 * eps)
        self.assertTrue(np.allclose(heading_rate / sample.metric, sample.curvature, rtol=3e-4, atol=3e-5))

    def test_closed_seam_round_trip_and_immutable_arrays(self):
        track = _circle(count=81)
        source_before = track.path_coordinates.x_north.copy()
        reference = ReferenceCurve(track)
        spline = OffsetSpline(track, 18.0)
        trajectory = OffsetTrajectory(reference, spline, np.linspace(-.4, .2, spline.control_count))
        left = trajectory.evaluate(0.0)
        right = trajectory.evaluate(track.lap_length)
        self.assertAlmostEqual(left.north, right.north, places=10)
        self.assertAlmostEqual(left.east, right.east, places=10)
        self.assertAlmostEqual(left.metric, right.metric, places=10)
        source = np.linspace(0, track.lap_length, 41)
        self.assertTrue(np.allclose(trajectory.station_map.to_reference(trajectory.station_map.to_path(source)), source))
        self.assertFalse(reference.source_station.flags.writeable)
        self.assertFalse(trajectory.station_map.path_station.flags.writeable)
        self.assertTrue(np.array_equal(track.path_coordinates.x_north, source_before))

    def test_nonconstant_closed_curve_seam_derivatives_and_station_accuracy(self):
        # This ellipse has nonconstant curvature, unlike a circle, and checks
        # periodic derivatives on both the fitted reference and offset path.
        a, b = 42.0, 23.0
        theta = np.linspace(0.0, 2.0 * np.pi, 161)
        speed = np.hypot(-a * np.sin(theta), b * np.cos(theta))
        station = np.r_[0.0, np.cumsum((speed[:-1] + speed[1:]) * np.diff(theta) / 2.0)]
        track = _track(station, a * np.cos(theta), b * np.sin(theta), closed=True)
        reference = ReferenceCurve(track)
        spline = OffsetSpline(track, 15.0)
        trajectory = OffsetTrajectory(reference, spline, np.sin(np.linspace(0, 2*np.pi, spline.control_count, endpoint=False)))
        for field in ("north_s", "east_s", "north_ss", "east_ss", "curvature"):
            self.assertAlmostEqual(getattr(reference.evaluate(0.0), field), getattr(reference.evaluate(track.lap_length), field), places=8)
        self.assertAlmostEqual(trajectory.evaluate(0.0).heading_error, trajectory.evaluate(track.lap_length).heading_error, places=8)
        # Independently integrate a station interior to verify map interpolation,
        # rather than only testing inverse round trip through the same PCHIP.
        from scipy.integrate import quad
        q = 0.371 * track.lap_length
        independent, _ = quad(lambda v: float(trajectory.evaluate(v).metric), 0.0, q, epsabs=1e-10)
        self.assertAlmostEqual(float(trajectory.station_map.to_path(q)), independent, delta=3e-5)

    def test_open_rejects_outside_and_to_path_uses_exact_geometry(self):
        track = _open_curve()
        reference = ReferenceCurve(track)
        spline = OffsetSpline(track, 4.0)
        trajectory = OffsetTrajectory(reference, spline, np.zeros(spline.control_count))
        with self.assertRaises(ValueError):
            reference.evaluate(-1e-6)
        with self.assertRaises(ValueError):
            trajectory.station_map.to_path(track.lap_length + 1e-6)
        path = trajectory.to_path(np.linspace(0.0, track.lap_length, 21))
        sample = trajectory.evaluate(np.linspace(0.0, track.lap_length, 21))
        self.assertTrue(np.allclose(path.x_north, sample.north))
        self.assertTrue(np.allclose(path.y_east, sample.east))
        self.assertTrue(np.all(np.abs(np.diff(path.heading)) < np.pi))

    def test_exported_closed_heading_is_unwrapped(self):
        track = _circle()
        reference = ReferenceCurve(track)
        spline = OffsetSpline(track, 20.0)
        trajectory = OffsetTrajectory(reference, spline, np.zeros(spline.control_count))
        path = trajectory.to_path(np.linspace(0.0, track.lap_length, 101))
        self.assertTrue(np.all(np.abs(np.diff(path.heading)) < np.pi))
        self.assertAlmostEqual(abs(path.heading[-1] - path.heading[0]), 2*np.pi, delta=2e-4)


if __name__ == "__main__":
    unittest.main()
