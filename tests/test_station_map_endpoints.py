"""Exact StationMap endpoint identities without widening open coverage."""
import unittest
from pathlib import Path

import numpy as np

from simplified_vehicle_models.tracks.line_trajectory import StationMap

ROOT = Path(__file__).resolve().parents[1]
FSC_XML = ROOT / "tracks_out/fsc_may_2026.xml"
MIS_XML = ROOT / "models/tracks/mis_ev_endurance_2024_width.xml"


class StationMapEndpointTests(unittest.TestCase):
    def setUp(self):
        source = np.linspace(10.123, 778.418854321, 9)
        path = np.linspace(20.456, 788.828683906432, 9)
        path[1:-1] += np.sin(np.arange(1, 8)) * 7.68295854321
        self.mapping = StationMap(source, path)

    def test_exact_endpoint_identities_scalar_and_vector(self):
        mapping = self.mapping
        for source, physical in ((mapping.start, mapping.path_start),
                                 (mapping.end, mapping.path_end)):
            self.assertEqual(mapping.to_path(source), physical)
            self.assertEqual(mapping.to_reference(physical), source)
        np.testing.assert_array_equal(
            mapping.to_path([mapping.start, mapping.end]),
            [mapping.path_start, mapping.path_end],
        )
        np.testing.assert_array_equal(
            mapping.to_reference([mapping.path_start, mapping.path_end]),
            [mapping.start, mapping.end],
        )

    def test_interior_pchip_values_are_unchanged(self):
        mapping = self.mapping
        source = np.linspace(mapping.start, mapping.end, 101)[1:-1]
        path = np.linspace(mapping.path_start, mapping.path_end, 101)[1:-1]
        np.testing.assert_array_equal(mapping.to_path(source), mapping._forward(source))
        np.testing.assert_array_equal(mapping.to_reference(path), mapping._inverse(path))

    def test_open_domain_rejects_adjacent_outside_values(self):
        mapping = self.mapping
        for value in (np.nextafter(mapping.start, -np.inf),
                      np.nextafter(mapping.end, np.inf)):
            with self.assertRaises(ValueError):
                mapping.to_path(value)
        for value in (np.nextafter(mapping.path_start, -np.inf),
                      np.nextafter(mapping.path_end, np.inf)):
            with self.assertRaises(ValueError):
                mapping.to_reference(value)

    def test_closed_terminal_and_lap_mapping_is_preserved(self):
        source = StationMap([10., 20., 30.], [50., 70., 90.], is_closed=True)
        self.assertEqual(source.to_reference(90.), 30.)
        self.assertEqual(source.to_reference(110.), 20.)
        self.assertEqual(source.to_path(30.), 90.)
        self.assertEqual(source.to_path(40.), 70.)


class RealPreservedCutEndpointTests(unittest.TestCase):
    def _check_fixture(self, fixture):
        from simplified_vehicle_models.solvers.capability_profile import _path_samples
        from simplified_vehicle_models.tracks.boundary_view import cut_open_track
        from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory, ReferenceCurve
        from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
        from simplified_vehicle_models.tracks.profile import load_track

        track = cut_open_track(load_track(fixture))
        spline = OffsetSpline(track, 35.)
        trajectory = OffsetTrajectory(ReferenceCurve(track), spline, np.zeros(spline.control_count))
        station = trajectory.station_map
        self.assertEqual(station.to_reference(station.path_end), station.end)
        self.assertEqual(station.to_path(station.end), station.path_end)
        sampled = trajectory.to_path(np.linspace(station.start, station.end, 5))
        distance, curvature, mapped = _path_samples(sampled, 20., trajectory)
        self.assertEqual(mapped[-1], station.end)
        self.assertEqual(distance[-1], station.path_end)
        self.assertTrue(np.all(np.isfinite(curvature)))

    def test_fsc_preserved_cut_endpoints_and_capability_samples(self):
        self._check_fixture(FSC_XML)

    def test_mis_preserved_cut_endpoints_and_capability_samples(self):
        self._check_fixture(MIS_XML)


if __name__ == "__main__":
    unittest.main()
