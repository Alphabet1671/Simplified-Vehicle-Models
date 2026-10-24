import unittest

import numpy as np

from simplified_vehicle_models.tracks.geometry_audit import audit_track_geometry
from simplified_vehicle_models.tracks.corridor import corridor_from_track
from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve
from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile


class TrackGeometryAuditTests(unittest.TestCase):
    def circle(self, count=13, transform=None):
        radius = 20.0
        theta = np.linspace(0.0, 2.0 * np.pi, count)
        distance = radius * theta
        north = radius * np.sin(theta)
        east = radius * (1.0 - np.cos(theta))
        heading = theta
        if transform is not None:
            angle, shift = transform
            north, east = (north * np.cos(angle) - east * np.sin(angle) + shift[0],
                           north * np.sin(angle) + east * np.cos(angle) + shift[1])
            heading = heading + angle
        curvature = np.full(count, 1.0 / radius)
        coordinates = PathCoordinates(distance, north, east, heading, curvature)
        track = TrackProfile(distance, np.diff(distance), np.full(count, radius), curvature,
                             True, "analytic circle", path_coordinates=coordinates,
                             half_width_left=np.linspace(4.0, 6.0, count), half_width_right=np.linspace(7.0, 3.0, count),
                             width_provenance="surveyed")
        reference = ReferenceCurve(track)
        corridor = corridor_from_track(track)
        return track, reference, corridor

    def test_circle_reports_finite_diagnostic_and_metric(self):
        result = audit_track_geometry(*self.circle())
        self.assertEqual(result["claim"], "diagnostic_only")
        self.assertAlmostEqual(result["source"]["curvature"]["range"]["min_per_m"], 0.05)
        self.assertAlmostEqual(result["fit"]["metric"]["min_m_per_m"], 1.0, places=3)
        self.assertEqual(result["corridor"]["provenance"], "surveyed")

    def test_translation_rotation_preserves_geometry_diagnostics(self):
        base = audit_track_geometry(*self.circle())
        moved = audit_track_geometry(*self.circle(transform=(0.71, (120.0, -80.0))))
        self.assertAlmostEqual(base["fit"]["metric"]["min_m_per_m"], moved["fit"]["metric"]["min_m_per_m"], places=8)
        self.assertAlmostEqual(base["source_vs_fit"]["xy"]["max_m"], moved["source_vs_fit"]["xy"]["max_m"], places=7)

    def test_pair_budget_is_reported(self):
        result = audit_track_geometry(*self.circle(count=81), crossing_pair_budget=5)
        self.assertTrue(result["crossings"]["truncated"])
        self.assertEqual(result["crossings"]["pairs_examined"], 5)

    def test_reversal_negates_signed_curvature_and_swaps_width_sides(self):
        track, _, _ = self.circle(count=17)
        distance = track.distance[-1] - track.distance[::-1]
        coordinates = PathCoordinates(distance, track.path_coordinates.x_north[::-1],
                                      track.path_coordinates.y_east[::-1],
                                      track.path_coordinates.heading[::-1] + np.pi,
                                      -track.curvature[::-1])
        reversed_track = TrackProfile(distance, np.diff(distance), track.radius[::-1],
                                      -track.curvature[::-1], True, "reversed",
                                      path_coordinates=coordinates,
                                      half_width_left=track.half_width_right[::-1],
                                      half_width_right=track.half_width_left[::-1],
                                      width_provenance="surveyed")
        result = audit_track_geometry(reversed_track, ReferenceCurve(reversed_track),
                                      corridor_from_track(reversed_track))
        self.assertAlmostEqual(result["source"]["curvature"]["range"]["min_per_m"], -0.05)
        self.assertTrue(np.allclose(reversed_track.half_width_left, track.half_width_right[::-1]))
        self.assertLess(result["source"]["curvature"]["range"]["max_per_m"], 0.0)
        self.assertAlmostEqual(reversed_track.half_width_left[0], track.half_width_right[-1])
        self.assertEqual(result["corridor"]["provenance"], "surveyed")

    def test_tangent_kink_seam_is_measured_one_sided(self):
        track, _, corridor = self.circle(count=13)
        # The fit itself retains its periodic condition; source heading mismatch
        # reports the deliberate seam kink independently of query topology.
        heading = track.path_coordinates.heading.copy()
        heading[-1] += 0.2
        coords = PathCoordinates(track.distance, track.path_coordinates.x_north,
                                 track.path_coordinates.y_east, heading, track.curvature)
        kinked = TrackProfile(track.distance, track.segment_length, track.radius, track.curvature,
                              True, "kink", path_coordinates=coords,
                              half_width_left=track.half_width_left,
                              half_width_right=track.half_width_right, width_provenance="surveyed")
        result = audit_track_geometry(kinked, ReferenceCurve(kinked), corridor)
        self.assertIn("one_sided_probe_offset_m", result["seam"])
        self.assertTrue(result["seam"]["source_interpolation_periodic"])
        self.assertGreater(result["source_vs_fit"]["heading_wrapped"]["max_abs_rad"], 0.05)

    def test_nonuniform_curvature_ramp_compares_source_and_fit(self):
        # Analytic parabola r(s)=(s,s^2): k=2/(1+4s^2)^(3/2),
        # sampled at deliberately nonuniform source stations.
        parameter = np.array([-2.0, -1.1, -0.4, 0.0, 0.15, 0.55, 1.3, 2.0])
        distance = np.array([0.0, 0.7, 1.0, 1.2, 1.35, 1.8, 2.8, 4.0])
        north, east = parameter, parameter**2
        heading = np.arctan2(2.0*parameter, np.ones_like(parameter))
        curvature = 2.0 / (1.0 + 4.0*parameter**2)**1.5
        coords = PathCoordinates(distance, north, east, heading, curvature, is_closed=False)
        track = TrackProfile(distance, np.diff(distance), 1.0/curvature, curvature,
                             False, "nonuniform curvature ramp", path_coordinates=coords)
        result = audit_track_geometry(track, ReferenceCurve(track), None)
        gaps = np.diff(distance)
        self.assertAlmostEqual(result["source"]["spacing"]["ratio"], float(gaps.max()/gaps.min()))
        self.assertGreater(result["source"]["curvature"]["range"]["max_per_m"],
                           result["source"]["curvature"]["range"]["min_per_m"])
        self.assertIsNotNone(result["source_vs_fit"]["signed_curvature"]["max_abs_per_m"])

    def test_tight_hairpin_fit_reports_metric_and_source_discrepancy(self):
        theta = np.linspace(-np.pi / 2, np.pi / 2, 9)
        north, east = 8.0 * np.cos(theta), 8.0 * np.sin(theta)
        distance = np.linspace(0.0, 8.0 * np.pi, len(theta))
        heading, curvature = theta + np.pi / 2, np.full(len(theta), 1.0 / 8.0)
        coords = PathCoordinates(distance, north, east, heading, curvature, is_closed=False)
        track = TrackProfile(distance, np.diff(distance), np.full(len(distance), 8.0), curvature,
                             False, "hairpin", path_coordinates=coords,
                             half_width_left=np.full(len(distance), 1.0),
                             half_width_right=np.full(len(distance), 1.0), width_provenance="assumed")
        result = audit_track_geometry(track, ReferenceCurve(track), corridor_from_track(track))
        self.assertTrue(np.isfinite(result["fit"]["metric"]["min_m_per_m"]))
        self.assertGreater(result["source_vs_fit"]["xy"]["max_m"], 0.0)

    def test_asymmetric_width_pinch_station_and_provenance(self):
        track, _, _ = self.circle(count=13)
        left = np.full(track.distance.shape, 6.0)
        left[5] = 1.25
        right = np.full(track.distance.shape, 3.0)
        pinch = TrackProfile(track.distance, track.segment_length, track.radius, track.curvature,
                             True, "pinch", path_coordinates=track.path_coordinates,
                             half_width_left=left, half_width_right=right,
                             width_provenance="assumed")
        result = audit_track_geometry(pinch, ReferenceCurve(pinch), corridor_from_track(pinch))
        self.assertAlmostEqual(result["corridor"]["narrowest_left_width_m"], 1.25)
        self.assertAlmostEqual(result["corridor"]["narrowest_right_width_m"], 3.0)
        self.assertEqual(result["corridor"]["pinch_station_left_m"], track.distance[5])
        self.assertIn("not vehicle-footprint", result["corridor"]["interpretation"])

    def test_ordered_figure_eight_crossing_is_retained_as_diagnostic(self):
        t = np.linspace(0.0, 2.0*np.pi, 33)
        north = 12.0*np.sin(t)
        east = 12.0*np.sin(t)*np.cos(t)
        distance = np.linspace(0.0, 100.0, len(t))
        # Heading/curvature are source diagnostic fields, not used to fit XY.
        dn, de = np.gradient(north, distance), np.gradient(east, distance)
        heading = np.unwrap(np.arctan2(de, dn))
        d2n, d2e = np.gradient(dn, distance), np.gradient(de, distance)
        curvature = (dn*d2e - de*d2n) / np.maximum(dn*dn + de*de, 1e-12)**1.5
        north[-1], east[-1] = north[0], east[0]
        curvature[-1] = curvature[0]
        coords = PathCoordinates(distance, north, east, heading, curvature)
        track = TrackProfile(distance, np.diff(distance), np.ones(len(t)), curvature,
                             True, "figure eight", path_coordinates=coords)
        result = audit_track_geometry(track, ReferenceCurve(track), None,
                                     sample_budget=250, crossing_pair_budget=50_000)
        self.assertGreater(result["crossings"]["count"], 0)
        self.assertGreater(len(result["crossings"]["records"]), 0)
        self.assertNotEqual(result["crossings"]["records"][0]["station_a_m"],
                            result["crossings"]["records"][0]["station_b_m"])

    def test_resampling_is_observed_not_assumed_invariant(self):
        coarse = audit_track_geometry(*self.circle(count=7))
        fine = audit_track_geometry(*self.circle(count=17))
        self.assertGreater(abs(coarse["fit"]["metric"]["min_m_per_m"] - 1.0), 1e-10)
        self.assertGreater(abs(fine["fit"]["metric"]["min_m_per_m"] - 1.0), 1e-10)
        self.assertNotAlmostEqual(coarse["fit"]["metric"]["min_m_per_m"],
                                  fine["fit"]["metric"]["min_m_per_m"], places=8)


if __name__ == "__main__":
    unittest.main()
