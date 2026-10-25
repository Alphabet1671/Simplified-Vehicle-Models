import unittest
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.tracks.periodic_cut import (
    PeriodicCutMapping, MappedReferenceView, UnsupportedWidthSeam,
)


class PeriodicCutMappingTests(unittest.TestCase):
    def test_strict_descriptor_mapping_roles_and_panels(self):
        mapping = PeriodicCutMapping({
            "mapping_version": "periodic_cut_v1", "source_start": 10.,
            "source_end": 20., "cut_source_station": 18.,
            "view_start": 0., "view_end": 10., "source_identity": {"fit": "periodic-cubic", "grid": [1, 2]},
        })
        self.assertEqual(mapping.to_source(0.), 18.)
        self.assertEqual(mapping.to_source(10.), 18.)
        self.assertEqual(mapping.to_source(10., endpoint_side="terminal"), 20.)
        with self.assertRaises(AttributeError): mapping.cut_source_station = 19.
        with self.assertRaises(AttributeError): mapping.sha256 = "stale"
        self.assertEqual(mapping.descriptor["source_identity"]["grid"], [1, 2])
        pair_list = PeriodicCutMapping({**mapping.descriptor, "source_identity": [["a", 1], ["b", 2]]})
        self.assertEqual(pair_list.descriptor["source_identity"], [["a", 1], ["b", 2]])
        for key in ("source_start", "source_end", "cut_source_station", "view_start", "view_end"):
            with self.assertRaises(ValueError): PeriodicCutMapping({**mapping.descriptor, key: "10.0"})
        self.assertEqual(mapping.to_source(np.array([0., 2.]), endpoint_side="launch").tolist(), [18., 10.])
        identity = {"recipe": ["quintic", 5], "grid": {"count": 4}}
        descriptor = {"mapping_version": "periodic_cut_v1", "source_start": 0., "source_end": 10.,
                      "cut_source_station": 3., "view_start": 0., "view_end": 10., "source_identity": identity}
        first = PeriodicCutMapping(descriptor)
        original_hash = first.sha256
        identity["recipe"].append("mutated")
        first.descriptor["source_identity"]["grid"]["count"] = 99
        self.assertEqual(first.sha256, original_hash)
        changed = {**descriptor, "source_identity": {"recipe": ["different"], "grid": {"count": 4}}}
        self.assertNotEqual(first.sha256, PeriodicCutMapping(changed).sha256)
        for bad in (True, np.array([[1.]])):
            with self.assertRaises(ValueError): mapping.to_source(bad)
        self.assertEqual([(p.view_start, p.view_end, p.source_start, p.source_end, p.side)
                          for p in mapping.split_interval(0., 10.)],
                         [(0., 2., 18., 20., "before_seam"), (2., 10., 10., 18., "after_seam")])
        with self.assertRaises(ValueError): mapping.to_source(np.nextafter(10., np.inf))
        with self.assertRaises(ValueError): PeriodicCutMapping({**mapping.descriptor, "extra": 1})
        with self.assertRaises(ValueError): PeriodicCutMapping({**mapping.descriptor, "source_start": True})

    def test_width_interpolation_preserves_sides_and_rejects_seam_jump(self):
        mapping = PeriodicCutMapping({"mapping_version": "periodic_cut_v1", "source_start": 0.,
            "source_end": 8., "cut_source_station": 2.5, "view_start": 0., "view_end": 8., "source_identity": {"fit": "periodic-quintic", "grid": [0, 1, 2]}})
        widths = mapping.map_widths([0., 2., 5., 8.], [3., 4., 6., 3.], [7., 5., 4., 7.], provenance="surveyed")
        self.assertEqual(widths.provenance, "surveyed")
        self.assertEqual(widths.view_station[0], 0.)
        self.assertEqual(widths.view_station[-1], 8.)
        self.assertTrue(np.all(np.diff(widths.view_station) > 0.))
        with self.assertRaises(UnsupportedWidthSeam):
            mapping.map_widths([0., 4., 8.], [3., 4., 5.], [7., 5., 6.], provenance="surveyed")

    def test_view_delegates_without_refitting_and_preserves_geometry(self):
        from types import SimpleNamespace
        from scipy.interpolate import make_interp_spline
        from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve
        from simplified_vehicle_models.tracks.periodic_cut import reference_fit_identity
        s = np.array([0., .7, 1.8, 3.1, 4.4, 6., 8.])
        track = SimpleNamespace(path_coordinates=SimpleNamespace(distance=s,
            x_north=10*np.cos(2*np.pi*s/8), y_east=7*np.sin(2*np.pi*s/8),
            curvature=np.zeros_like(s), is_closed=True), is_closed=True)
        ref = ReferenceCurve(track)
        cuts = (0., 2.35, 7.91)
        mappings = [PeriodicCutMapping({"mapping_version": "periodic_cut_v1", "source_start": 0.,
            "source_end": 8., "cut_source_station": cut, "view_start": 0., "view_end": 8.,
            "source_identity": reference_fit_identity(ref)}) for cut in cuts]
        mapping = mappings[1]
        with patch("scipy.interpolate.make_interp_spline", side_effect=AssertionError("refit")):
            view = MappedReferenceView(ref, mapping)
            u = np.array([0., .2, 2.35, 6.9, 8.])
            actual = view.evaluate(u)
        expected_source = np.mod(2.35 + u, 8.)
        expected_source[-1] = 8.
        expected = ref.evaluate(expected_source)
        self.assertEqual(view.mapping.to_source(u, endpoint_side="terminal")[-1], 8.)
        for field in ("north", "east", "north_s", "east_s", "north_ss", "east_ss", "north_sss", "east_sss", "metric", "metric_s", "heading", "curvature", "curvature_s"):
            np.testing.assert_allclose(getattr(actual, field), getattr(expected, field))
        self.assertEqual(view.source_station(u).shape, u.shape)
        self.assertFalse(view.is_exact_reference_curve)
        altered_track = SimpleNamespace(path_coordinates=SimpleNamespace(distance=s,
            x_north=11*np.cos(2*np.pi*s/8), y_east=7*np.sin(2*np.pi*s/8),
            curvature=np.zeros_like(s), is_closed=True), is_closed=True)
        altered_ref = ReferenceCurve(altered_track)
        with self.assertRaisesRegex(ValueError, "source identity"):
            MappedReferenceView(altered_ref, mapping)
        class LookalikeReference:
            __name__ = "ReferenceCurve"
            interpolation_periodic = True
            start = 0.
            end = 8.
        with self.assertRaisesRegex(ValueError, "exact ReferenceCurve"):
            MappedReferenceView(LookalikeReference(), mapping)
        class DerivedReference(ReferenceCurve):
            pass
        subclass_ref = DerivedReference(track)
        with self.assertRaisesRegex(ValueError, "exact ReferenceCurve"):
            MappedReferenceView(subclass_ref, mapping)
        for cut_mapping in mappings:
            cut_view = MappedReferenceView(ref, cut_mapping)
            with self.assertRaises(AttributeError): cut_view.mapping = mapping
            with self.assertRaises(AttributeError): cut_view.source_reference = ref
            sample = np.array([0., .31, 4.7, 8.])
            got = cut_view.evaluate(sample)
            source = cut_mapping.to_source(sample, endpoint_side="terminal")
            want = ref.evaluate(source)
            for field in ("north", "east", "north_s", "east_s", "north_ss", "east_ss", "north_sss", "east_sss", "metric", "metric_s", "heading", "curvature", "curvature_s"):
                np.testing.assert_allclose(getattr(got, field), getattr(want, field), rtol=0., atol=1e-12)


if __name__ == "__main__": unittest.main()
