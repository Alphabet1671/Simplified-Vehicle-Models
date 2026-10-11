from pathlib import Path
import tempfile
import unittest

import numpy as np

from simplified_vehicle_models.tracks import load_track


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
TRACK_PATH = REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_v11.xml"


class TrackTests(unittest.TestCase):
    def test_loads_legacy_mis_track(self) -> None:
        track = load_track(TRACK_PATH)

        self.assertEqual(track.title, "2024 Michigan Endurance V11")
        self.assertTrue(track.is_closed)
        self.assertEqual(len(track.distance), 1407)
        self.assertEqual(len(track.segment_length), 1406)
        self.assertAlmostEqual(track.lap_length, 1014.32575, places=8)
        self.assertAlmostEqual(
            float(np.sum(track.segment_length)),
            1014.32575,
            places=8,
        )
        self.assertAlmostEqual(float(np.min(track.radius)), 5.210433541156343)
        np.testing.assert_allclose(track.curvature, 1.0 / track.radius)
        self.assertTrue(np.all(track.segment_length > 0.0))

    def test_excludes_duplicated_terminal_segment(self) -> None:
        track = load_track(TRACK_PATH)

        self.assertAlmostEqual(track.segment_length[-1], 0.41325)
        self.assertEqual(len(track.segment_length), len(track.distance) - 1)

    def test_arrays_are_read_only(self) -> None:
        track = load_track(TRACK_PATH)

        with self.assertRaises(ValueError):
            track.radius[0] = 1.0

    def test_rejects_nonpositive_radius(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "track.csv").write_text(
                "distance,segment_length,radius\n0,1,10\n1,1,0\n",
                encoding="utf-8",
            )
            (root / "track.xml").write_text(
                """<model schema-version="1.0" id="track"
    type="track-model" subtype="distance-radius-profile">
    <metadata><title>Invalid track</title></metadata>
    <data ref="track.csv"/>
    <parameters>
        <parameter name="is_closed" value="true"/>
        <parameter name="terminal_segment_is_duplicate" value="true"/>
    </parameters>
</model>
""",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "radii must be positive"):
                load_track(root / "track.xml")


if __name__ == "__main__":
    unittest.main()
