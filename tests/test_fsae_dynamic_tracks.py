from pathlib import Path
import math
import unittest
import xml.etree.ElementTree as ElementTree

import numpy as np

from simplified_vehicle_models.results.traces import LapMode
from simplified_vehicle_models.solvers.forward_backward import solve_lap
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.xml.loader import load_point_mass_vehicle
from tools.tracks.build_fsae_dynamic_tracks import all_files


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
TRACKS = REPOSITORY_ROOT / "models/tracks"
ACCELERATION = TRACKS / "fsae_2026_acceleration.xml"
SKIDPAD = TRACKS / "fsae_2026_skidpad.xml"
CIRCLE_RADIUS = 9.125
CIRCLE_LENGTH = 2.0 * math.pi * CIRCLE_RADIUS


def _parameters(xml: Path) -> dict[str, float]:
    root = ElementTree.parse(xml).getroot()
    values = {}
    for parameter in root.iter("parameter"):
        try:
            values[parameter.get("name")] = float(parameter.get("value"))
        except ValueError:
            pass
    return values


class FsaeDynamicTrackTests(unittest.TestCase):
    def test_committed_files_match_generator(self) -> None:
        for name, text in all_files().items():
            self.assertEqual((TRACKS / name).read_text(encoding="utf-8"), text, name)

    def test_acceleration_geometry(self) -> None:
        track = load_track(ACCELERATION)
        self.assertFalse(track.is_closed)
        # 0.30 m staging offset plus the 75 m course (D.9.1.1, D.9.2.3).
        self.assertAlmostEqual(track.lap_length, 75.30, places=6)
        self.assertTrue(np.all(track.radius >= 1.0e4))
        self.assertTrue(np.all(np.diff(track.distance) > 0.0))

    def test_skidpad_geometry(self) -> None:
        track = load_track(SKIDPAD)
        self.assertFalse(track.is_closed)
        on_circle = np.isclose(track.radius, CIRCLE_RADIUS)
        # Four full laps of the ring centerline radius (D.10.1.1, D.10.2.3).
        circle_segments = on_circle[:-1] & on_circle[1:]
        # The right-to-left change is a 0.05 m ramp between two circle-radius
        # nodes, so it is counted with the circle segments.
        self.assertAlmostEqual(
            float(np.sum(track.segment_length[circle_segments])),
            4.0 * CIRCLE_LENGTH + 0.05,
            places=6,
        )
        # Right laps first, then left laps (right turn is positive curvature).
        self.assertEqual(float(np.sign(track.curvature[np.argmax(on_circle) + 2])), 1.0)
        self.assertEqual(float(np.sign(track.curvature[len(on_circle) - 1 - np.argmax(on_circle[::-1]) - 2])), -1.0)
        self.assertTrue(track.has_signed_geometry)
        self.assertFalse(track.path_coordinates.is_closed)
        self.assertAlmostEqual(CIRCLE_RADIUS, (15.25 + 21.25) / 4.0)
        self.assertAlmostEqual(2.0 * CIRCLE_RADIUS, 18.25)

    def test_standing_start_solutions(self) -> None:
        vehicle = load_point_mass_vehicle(REPOSITORY_ROOT / "models/formulations/murphy_point_mass.xml")
        accel_track = load_track(ACCELERATION)
        accel = solve_lap(vehicle, accel_track, LapMode.STANDING_START, tolerance=1.0e-5)
        self.assertTrue(accel.converged)
        timed = np.interp([0.30, accel_track.lap_length], accel_track.distance, accel.elapsed_time)
        self.assertGreater(timed[1] - timed[0], 3.0)
        self.assertLess(timed[1] - timed[0], 6.0)

        skid_track = load_track(SKIDPAD)
        skid = solve_lap(vehicle, skid_track, LapMode.STANDING_START, tolerance=1.0e-5)
        self.assertTrue(skid.converged)
        window = _parameters(SKIDPAD)
        at = lambda name: np.interp(window[name], skid_track.distance, skid.elapsed_time)
        right = at("right_timed_lap_end_distance") - at("right_timed_lap_start_distance")
        left = at("left_timed_lap_end_distance") - at("left_timed_lap_start_distance")
        for name in ("right", "left"):
            lap = window[f"{name}_timed_lap_end_distance"] - window[f"{name}_timed_lap_start_distance"]
            self.assertAlmostEqual(lap, CIRCLE_LENGTH, places=6)
        self.assertAlmostEqual(right, left, places=6)
        self.assertGreater(right, 4.0)
        self.assertLess(right, 8.0)


if __name__ == "__main__":
    unittest.main()
