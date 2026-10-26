"""Explicit full-course cut views retain periodic source-fit geometry."""
from dataclasses import replace
from pathlib import Path
import unittest

import numpy as np

from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve

ROOT = Path(__file__).resolve().parents[1]


class CutOpenReferenceViewTest(unittest.TestCase):
    def test_fsc_cut_preserves_source_reference_fit_and_open_domain(self):
        from simplified_vehicle_models.tracks.boundary_view import cut_open_track

        source = load_track(ROOT / "tracks_out/fsc_may_2026.xml")
        before = {name: np.array(getattr(source, name), copy=True)
                  for name in ("distance", "segment_length", "radius", "curvature")}
        old_curve = ReferenceCurve(source)
        view = cut_open_track(source)
        curve = ReferenceCurve(view)
        self.assertFalse(view.is_closed)
        self.assertFalse(view.path_coordinates.is_closed)
        self.assertIs(view.source_reference_interpolation_periodic, True)
        self.assertTrue(curve.interpolation_periodic)
        np.testing.assert_array_equal(curve.source_station, old_curve.source_station)
        stations = np.linspace(0., source.lap_length, 1001)
        for name in ("north", "east", "north_s", "east_s", "north_ss", "east_ss",
                     "north_sss", "east_sss", "metric", "metric_s", "curvature", "curvature_s"):
            np.testing.assert_allclose(getattr(curve.evaluate(stations), name),
                                       getattr(old_curve.evaluate(stations), name), rtol=0., atol=1e-12)
        for name, values in before.items():
            np.testing.assert_array_equal(getattr(source, name), values)
        self.assertFalse(view.distance.flags.writeable)
        self.assertFalse(view.path_coordinates.x_north.flags.writeable)
        with self.assertRaises(ValueError):
            curve.evaluate(-1e-3)
        with self.assertRaises(ValueError):
            curve.evaluate(source.lap_length + 1e-3)
        with self.assertRaises(ValueError):
            ReferenceCurve(view, interpolation_periodic=False)

    def test_default_legacy_cut_keeps_old_nonperiodic_fit(self):
        from tools.diagnostics.compare_qss_track_boundaries import track_view
        source = load_track(ROOT / "tracks_out/fsc_may_2026.xml")
        old_view = track_view(source, is_closed=False)
        self.assertIsNone(old_view.source_reference_interpolation_periodic)
        self.assertFalse(ReferenceCurve(old_view).interpolation_periodic)

    def test_profile_metadata_is_strictly_typed(self):
        source = load_track(ROOT / "tracks_out/fsc_may_2026.xml")
        self.assertIsNone(source.source_reference_interpolation_periodic)
        for bad in (1, "true", np.int64(1)):
            with self.subTest(bad=bad), self.assertRaises((TypeError, ValueError)):
                replace(source, source_reference_interpolation_periodic=bad)

    def test_normalization_preserves_fit_metadata_for_nested_topology(self):
        from types import SimpleNamespace
        from simplified_vehicle_models.solvers.qss_spline_ocp import _reference_curve
        source = load_track(ROOT / "tracks_out/fsc_may_2026.xml")
        view = replace(source, is_closed=False, source_reference_interpolation_periodic=True)
        normalized = _reference_curve(view)
        self.assertTrue(normalized.interpolation_periodic)
        self.assertFalse(normalized.is_closed)
        legacy = replace(source, is_closed=False)
        self.assertFalse(_reference_curve(legacy).interpolation_periodic)
        foreign = SimpleNamespace(path_coordinates=replace(source.path_coordinates, is_closed=True),
                                  is_closed=False, source_reference_interpolation_periodic=True)
        self.assertTrue(_reference_curve(foreign).interpolation_periodic)

    def test_path_grid_must_match_complete_profile_grid(self):
        from simplified_vehicle_models.tracks.boundary_view import cut_open_track
        source = load_track(ROOT / "tracks_out/fsc_may_2026.xml")
        coords = source.path_coordinates
        shortened = replace(coords, distance=coords.distance[:-1],
                            x_north=coords.x_north[:-1], y_east=coords.y_east[:-1],
                            heading=coords.heading[:-1], curvature=coords.curvature[:-1])
        with self.assertRaisesRegex(ValueError, "source grid"):
            cut_open_track(replace(source, path_coordinates=shortened))

    def test_only_full_closed_track_can_be_cut(self):
        from simplified_vehicle_models.tracks.boundary_view import cut_open_track
        source = load_track(ROOT / "tracks_out/fsc_may_2026.xml")
        with self.assertRaises(ValueError):
            cut_open_track(replace(source, is_closed=False))


if __name__ == "__main__":
    unittest.main()
