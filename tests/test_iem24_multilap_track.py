"""Small deterministic tests for the experimental median-driven-line builder."""
import hashlib
import unittest
import xml.etree.ElementTree as ElementTree
from pathlib import Path

import numpy as np

from simplified_vehicle_models.tracks.profile import load_track
from tools.tracks.build_iem24_multilap_track import periodic_lap_offset, robust_lap_selection

CANONICAL_CSV_SHA256 = "2db5a981f12d0c055afa3b167a9e6efa07cd09b1f6e7b2c51afea44f00e73d77"
ROOT = Path(__file__).resolve().parents[1]
ORIGINAL_XML = ROOT / "models/tracks/2024_michigan_endurance_closed.xml"
CANONICAL_XML = ROOT / "models/tracks/2024_michigan_endurance_multilap_closed.xml"
CANONICAL_CSV = CANONICAL_XML.with_suffix(".csv")
CANDIDATE_CSV = ROOT / "docs/validation/iem24-murphy/multilap_track_candidate.csv"


class MultiLapTrackTests(unittest.TestCase):
    def test_robust_selection_discards_one_bad_line(self):
        phase = np.linspace(0, 2 * np.pi, 256, endpoint=False)
        clean = np.stack([np.sin(phase) + shift for shift in (0, .03, -.03, .02, -.02)])
        outlier = (np.sin(phase) + 3.)[None]
        selected, errors = robust_lap_selection(np.vstack([clean, outlier]))
        np.testing.assert_array_equal(selected, [True] * 5 + [False])
        self.assertGreater(errors[-1], 2.)

    def test_periodic_smoothing_keeps_seam_continuous(self):
        grid = np.arange(0., 100., .5)
        phase = (grid + 1.5) % 100.
        series = np.cos(2 * np.pi * phase / 100.)
        smooth = periodic_lap_offset(phase, series, grid, 100., 2.)
        self.assertLess(abs(smooth[0] - smooth[-1]), .01)
        self.assertLess(np.max(abs(smooth - np.cos(2 * np.pi * grid / 100.))), .02)

    def test_sparse_lap_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "missing section"):
            periodic_lap_offset(np.r_[np.arange(0., 20.), np.arange(80., 100.)],
                                np.zeros(40), np.arange(100.), 100., 2.)


class CanonicalMultiLapTrackTests(unittest.TestCase):
    def test_canonical_track_is_signed_closed_and_has_expected_length(self):
        track = load_track(CANONICAL_XML)
        coordinates = track.path_coordinates
        self.assertTrue(track.is_closed)
        self.assertTrue(track.has_signed_geometry)
        self.assertAlmostEqual(track.lap_length, 1009.19501535, places=8)
        self.assertEqual(coordinates.distance.size, 2034)
        self.assertEqual(coordinates.x_north[-1], coordinates.x_north[0])
        self.assertEqual(coordinates.y_east[-1], coordinates.y_east[0])
        turn = np.trapezoid(coordinates.curvature, coordinates.distance)
        self.assertAlmostEqual(turn, -2 * np.pi, delta=.004)
        self.assertLess(coordinates.heading[-1] - coordinates.heading[0], -6.27)
        self.assertGreater(np.max(coordinates.curvature), 0.)
        self.assertLess(np.min(coordinates.curvature), 0.)

    def test_canonical_csv_hash_is_pinned_independently_of_candidate(self):
        self.assertEqual(hashlib.sha256(CANONICAL_CSV.read_bytes()).hexdigest(),
                         CANONICAL_CSV_SHA256)

    def test_canonical_csv_matches_candidate_when_candidate_is_present(self):
        if not CANDIDATE_CSV.exists():
            self.skipTest("historical candidate artifact is absent")
        self.assertEqual(CANONICAL_CSV.read_bytes(), CANDIDATE_CSV.read_bytes())

    def test_canonical_metadata_refers_to_own_csv_and_records_provenance(self):
        root = ElementTree.parse(CANONICAL_XML).getroot()
        self.assertEqual(root.get("id"), "2024-michigan-endurance-multilap-closed")
        self.assertEqual(root.find("./data").get("ref"), CANONICAL_CSV.name)
        text = CANONICAL_XML.read_text(encoding="utf-8")
        self.assertIn(CANONICAL_CSV_SHA256, text)
        self.assertIn("795d37a904302f712bab1fa1809dbd838c4cb9c0db3e60c9215561487d6c3d2b", text)
        # The XML comment is part of the hash-pinned track record, so it keeps
        # the pre-reorganization generator path (now tools/tracks/...).
        self.assertIn("tools/build_iem24_multilap_track.py", text)
        self.assertIn("counterclockwise", text)
        self.assertNotIn("clockwise from above", text)
        self.assertIn("7.5056 m", text)
        self.assertIn("1009.195015", text)
        self.assertIn("right turn has positive curvature", text)

    def test_original_single_lap_track_remains_unchanged(self):
        track = load_track(ORIGINAL_XML)
        self.assertAlmostEqual(track.lap_length, 1016.70064959, places=8)
        self.assertEqual(
            hashlib.sha256(ORIGINAL_XML.with_suffix(".csv").read_bytes()).hexdigest(),
            "ff8dcdc715eaff07eeb1d7a9e0a0739072480150577622ec8854000a0a44dbd4")
        self.assertEqual(
            hashlib.sha256(ORIGINAL_XML.read_bytes()).hexdigest(),
            "2880418d148b763e1360d25eb4b7a8a4144bb16e6e56017facf7f36bddc47180")


if __name__ == "__main__":
    unittest.main()
