"""Regression tests separating source-fit periodicity from query closure."""
from __future__ import annotations

import unittest
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory, ReferenceCurve
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile


def _track(distance, north, east, closed=False):
    distance = np.asarray(distance, dtype=float)
    north, east = np.asarray(north, dtype=float), np.asarray(east, dtype=float)
    heading = np.unwrap(np.arctan2(np.gradient(east, distance), np.gradient(north, distance)))
    curvature = np.gradient(heading, distance)
    coordinates = PathCoordinates(distance, north, east, heading, curvature, closed)
    return TrackProfile(distance, np.diff(distance), np.ones_like(distance), curvature,
                        closed, "interpolation-boundary-test", coordinates)


def _loop(closed):
    s = np.linspace(0., 2*np.pi, 33)
    return _track(s, np.cos(s), np.sin(s), closed=closed)


class ReferenceInterpolationBoundaryTests(unittest.TestCase):
    def test_default_and_omitted_are_bitwise_legacy_fit(self):
        for track in (_loop(False), _loop(True)):
            omitted, explicit_none = ReferenceCurve(track), ReferenceCurve(track, interpolation_periodic=None)
            self.assertEqual(omitted.interpolation_periodic, track.is_closed)
            stations = np.linspace(omitted.start, omitted.end, 101)
            for field in ("north", "east", "north_s", "east_s", "north_ss", "east_ss",
                          "north_sss", "east_sss", "metric", "metric_s", "curvature", "curvature_s"):
                self.assertTrue(np.array_equal(getattr(omitted.evaluate(stations), field),
                                                getattr(explicit_none.evaluate(stations), field)), field)

    def test_open_periodic_fit_matches_closed_geometry_but_keeps_open_queries(self):
        opened, closed = _loop(False), _loop(True)
        ref = ReferenceCurve(opened, interpolation_periodic=True)
        closed_ref = ReferenceCurve(closed)
        self.assertFalse(ref.is_closed)
        self.assertTrue(ref.interpolation_periodic)
        stations = np.linspace(ref.start, ref.end, 401)
        for field in ("north", "east", "north_s", "east_s", "north_ss", "east_ss",
                      "north_sss", "east_sss", "metric", "metric_s", "curvature", "curvature_s"):
            np.testing.assert_allclose(getattr(ref.evaluate(stations), field),
                                       getattr(closed_ref.evaluate(stations), field), rtol=1e-12, atol=1e-12,
                                       err_msg=field)
        terminal = ref.evaluate(ref.end)
        self.assertAlmostEqual(terminal.north_s, closed_ref.evaluate(0.).north_s, places=12)
        for station in (-1e-8, ref.end + 1e-8):
            with self.assertRaises(ValueError):
                ref.evaluate(station)
        self.assertAlmostEqual(closed_ref.evaluate(ref.end + .1).north, closed_ref.evaluate(.1).north)

    def test_periodic_fit_metadata_and_type_validation(self):
        self.assertEqual(ReferenceCurve(_loop(False), interpolation_periodic=True).fit_discrepancy[
            "source_interpolation_periodic"], True)
        reference = ReferenceCurve(_loop(False), interpolation_periodic=True)
        with self.assertRaises(AttributeError):
            reference.interpolation_periodic = False
        self.assertTrue(reference.interpolation_periodic)
        for invalid in ("true", 1, 0, object()):
            with self.assertRaises(TypeError):
                ReferenceCurve(_loop(False), interpolation_periodic=invalid)
        with self.assertRaises(ValueError):
            ReferenceCurve(_loop(True), interpolation_periodic=False)
        track = _track(np.arange(8.), np.arange(8.), np.sin(np.arange(8.)))
        with self.assertRaisesRegex(ValueError, "repeat"):
            ReferenceCurve(track, interpolation_periodic=True)

    def test_actual_fsc_source_periodic_fit_preserves_geometry_and_open_domain(self):
        from dataclasses import replace
        source = load_track("tracks_out/fsc_may_2026.xml")
        closed_reference = ReferenceCurve(source)
        coordinates = replace(source.path_coordinates, is_closed=False)
        opened_source = replace(source, is_closed=False, path_coordinates=coordinates)
        periodic_open = ReferenceCurve(opened_source, interpolation_periodic=True)
        default_open = ReferenceCurve(opened_source)
        self.assertEqual(source.path_coordinates.distance.size, 342)
        stations = np.linspace(closed_reference.start, closed_reference.end, 1001)
        for field in ("north", "east", "north_s", "east_s", "north_ss", "east_ss",
                      "north_sss", "east_sss", "metric", "metric_s", "curvature", "curvature_s"):
            np.testing.assert_allclose(getattr(periodic_open.evaluate(stations), field),
                                       getattr(closed_reference.evaluate(stations), field), rtol=1e-12, atol=1e-12,
                                       err_msg=field)
        self.assertAlmostEqual(periodic_open.evaluate(periodic_open.end).curvature,
                               closed_reference.evaluate(0.).curvature, places=12)
        self.assertGreater(abs(default_open.evaluate(0.).curvature - periodic_open.evaluate(0.).curvature), .1)
        self.assertLess(np.max(np.abs(periodic_open.evaluate(stations).curvature)), .24)
        for station in (-1e-6, periodic_open.end + 1e-6):
            with self.assertRaises(ValueError):
                periodic_open.evaluate(station)
        spline = OffsetSpline(opened_source, 20.0)
        zero_open = OffsetTrajectory(periodic_open, spline, np.zeros(spline.control_count))
        zero_closed = OffsetTrajectory(closed_reference, OffsetSpline(source, 20.0),
                                       np.zeros(OffsetSpline(source, 20.0).control_count))
        np.testing.assert_allclose(zero_open.evaluate(stations).north, zero_closed.evaluate(stations).north,
                                   rtol=1e-12, atol=1e-12)
        np.testing.assert_allclose(zero_open.evaluate(stations).east, zero_closed.evaluate(stations).east,
                                   rtol=1e-12, atol=1e-12)

    def test_fingerprint_includes_fit_boundary_without_changing_query_domain(self):
        from simplified_vehicle_models.solvers.convergence_policy import _source_fingerprint
        open_track = _loop(False)
        periodic = ReferenceCurve(open_track, interpolation_periodic=True)
        same_fit = ReferenceCurve(_loop(False), interpolation_periodic=True)
        nonperiodic = ReferenceCurve(_loop(False))
        self.assertEqual(_source_fingerprint(periodic), _source_fingerprint(same_fit))
        # Force identical sampled geometry so the boundary-condition tag itself
        # (not floating-point fit residuals) must distinguish the fingerprints.
        with patch.object(ReferenceCurve, "evaluate", autospec=True) as evaluate:
            def exact_samples(reference, station):
                s = np.asarray(station, dtype=float)
                return type("Geometry", (), {"north": np.interp(s, reference.source_station,
                                                                  reference._x(reference.source_station)),
                                              "east": np.interp(s, reference.source_station,
                                                                 reference._y(reference.source_station))})()
            evaluate.side_effect = exact_samples
            self.assertNotEqual(_source_fingerprint(periodic), _source_fingerprint(nonperiodic))
        self.assertFalse(periodic.is_closed)
        self.assertFalse(nonperiodic.is_closed)


if __name__ == "__main__":
    unittest.main()
