from pathlib import Path
import unittest

import numpy as np

from simplified_vehicle_models.tracks.profile import load_track

ROOT = Path(__file__).resolve().parents[1]
TRACKS = ROOT / "models" / "tracks"


class FSCSyntheticTrackTest(unittest.TestCase):
    def test_centerline_and_synthetic_corridor_load_as_closed_finite_tracks(self):
        centerline = load_track(TRACKS / "fsc_may_endurance_2026_linear.xml")
        edged = load_track(TRACKS / "fsc_may_endurance_2026_width.xml")
        for track in (centerline, edged):
            self.assertTrue(track.is_closed)
            self.assertTrue(np.all(np.isfinite(track.distance)))
            self.assertIsNotNone(track.path_coordinates)
            self.assertTrue(np.all(np.isfinite(track.path_coordinates.x_north)))
            self.assertTrue(np.all(np.isfinite(track.path_coordinates.y_east)))
        self.assertFalse(centerline.has_edges)
        self.assertEqual(edged.width_provenance, "assumed")
        np.testing.assert_array_equal(edged.half_width_left, np.full_like(edged.distance, 1.5))
        np.testing.assert_array_equal(edged.half_width_right, np.full_like(edged.distance, 1.5))


if __name__ == "__main__":
    unittest.main()
