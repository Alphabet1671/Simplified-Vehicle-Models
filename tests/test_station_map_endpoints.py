"""Exact StationMap endpoint identities without widening open coverage."""
import importlib.util
import unittest
from pathlib import Path

import numpy as np

from simplified_vehicle_models.tracks.line_trajectory import StationMap

ROOT = Path(__file__).resolve().parents[1]
FSC = ROOT / "IEM24 Files/FSC-2026-May-Endurance.mat"
MIS = Path("/Users/alphabet1671/Desktop/0615CompEndurance.mat")
HAS_REAL_FIXTURES = FSC.is_file() and MIS.is_file() and importlib.util.find_spec("scipy") is not None


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


@unittest.skipUnless(HAS_REAL_FIXTURES, "real FSC and MIS MAT fixtures and SciPy required")
class RealPreservedCutEndpointTests(unittest.TestCase):
    def _check_fixture(self, label, fixture, gps):
        from simplified_vehicle_models.solvers.capability_profile import _path_samples
        from simplified_vehicle_models.tracks.profile import TrackProfile
        from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory, ReferenceCurve
        from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
        from tools.track_gui.closed import ClosedSettings, build_closed
        from tools.track_gui.mat_channels import ChannelSelection

        if gps:
            cfg = dict(start_time=720.9, end_time=1103.6, spacing=.1, smoothing=.5,
                       maximum_radius=100., return_radius=10., minimum_travel=150.,
                       minimum_duration=18., maximum_duration=35.)
            selection = ChannelSelection(x_channel="Latitude_MTI", y_channel="Longitude_MTI", mode="gps")
        else:
            cfg = dict(start_time=550., end_time=1050., spacing=.5, smoothing=1.,
                       maximum_radius=300., return_radius=25., minimum_travel=150.,
                       minimum_duration=15., maximum_duration=120.)
            selection = ChannelSelection(x_channel="VNorth", y_channel="VEast", mode="velocity")
        geometry = build_closed(ClosedSettings(telemetry=fixture, channel_selection=selection,
                                               multi_lap=True, **cfg))
        # Adapt the real source track and preserve the physical cut as an open track.
        from tools.track_gui.track_ops import from_closed_geometry, split_closed
        track = from_closed_geometry(geometry, title=f"{label} endpoint fixture",
                                     maximum_radius=cfg["maximum_radius"])
        cut = track.length * .371
        cut_track = split_closed(track, [cut, track.length * .69])[0]
        from simplified_vehicle_models.tracks.types import PathCoordinates
        coordinates = PathCoordinates(
            cut_track.distance, cut_track.x_north, cut_track.y_east,
            cut_track.heading, cut_track.curvature, False,
        )
        actual = TrackProfile(cut_track.distance, np.diff(cut_track.distance),
                              np.ones_like(cut_track.distance), cut_track.curvature,
                              False, f"{label} preserved cut", coordinates)
        reference = ReferenceCurve(actual)
        spline = OffsetSpline(actual, max(4., actual.lap_length / 20.))
        trajectory = OffsetTrajectory(reference, spline, np.zeros(spline.control_count))
        station = trajectory.station_map
        self.assertEqual(station.to_reference(station.path_end), station.end)
        self.assertEqual(station.to_path(station.end), station.path_end)
        direct = station.to_reference(station.path_end)
        self.assertLessEqual(direct, trajectory.reference.end)
        self.assertTrue(np.isfinite(trajectory.evaluate(direct).curvature))
        sampled = trajectory.to_path(np.linspace(station.start, station.end, 5))
        distance, curvature, mapped = _path_samples(sampled, 20., trajectory)
        self.assertEqual(mapped[-1], station.end)
        self.assertEqual(distance[-1], station.path_end)
        self.assertTrue(np.all(np.isfinite(curvature)))

    def test_fsc_preserved_cut_endpoints_and_capability_samples(self):
        self._check_fixture("FSC", FSC, True)

    def test_mis_preserved_cut_endpoints_and_capability_samples(self):
        self._check_fixture("MIS", MIS, False)


if __name__ == "__main__":
    unittest.main()
