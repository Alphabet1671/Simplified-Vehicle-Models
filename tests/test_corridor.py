from pathlib import Path
import tempfile
import unittest

import numpy as np

from simplified_vehicle_models.tracks.corridor import corridor_from_track
from simplified_vehicle_models.tracks.profile import load_track
from tests.test_track_width import _write_circle_track


class CorridorTest(unittest.TestCase):
    def setUp(self) -> None:
        self._directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._directory.name)

    def tearDown(self) -> None:
        self._directory.cleanup()

    def test_symmetric_corridor_uses_the_right_positive_convention(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        corridor = corridor_from_track(track)
        np.testing.assert_allclose(corridor.maximum, 2.25)
        np.testing.assert_allclose(corridor.minimum, -2.25)
        self.assertEqual(corridor.provenance, "assumed")
        self.assertTrue(corridor.is_conditional)

    def test_narrow_right_side_maps_to_the_positive_bound(self) -> None:
        track = load_track(
            _write_circle_track(self.directory, width_rows=240, side="left")
        )
        corridor = corridor_from_track(track)
        np.testing.assert_allclose(corridor.maximum, 1.0)
        np.testing.assert_allclose(corridor.minimum, -3.0)
        self.assertFalse(corridor.is_conditional)

    def test_margin_shrinks_both_sides(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        corridor = corridor_from_track(track, margin=0.25)
        np.testing.assert_allclose(corridor.maximum, 2.0)
        np.testing.assert_allclose(corridor.minimum, -2.0)

    def test_zero_corridor_is_representable(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        corridor = corridor_from_track(track, margin=2.25)
        np.testing.assert_allclose(corridor.maximum, 0.0)
        np.testing.assert_allclose(corridor.minimum, 0.0)

    def test_margin_wider_than_the_corridor_is_rejected(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        with self.assertRaises(ValueError):
            corridor_from_track(track, margin=3.0)

    def test_fold_guard_clips_a_tight_corner(self) -> None:
        track = load_track(
            _write_circle_track(self.directory, radius=3.0, half_width=4.0)
        )
        corridor = corridor_from_track(track, fold_guard=0.2, margin=0.0)
        fold = 1.0 - track.curvature * corridor.maximum
        self.assertGreaterEqual(float(np.min(fold)), 0.2 - 1.0e-12)
        self.assertLess(float(np.max(corridor.maximum)), 4.0)

    def test_a_track_without_edges_requires_a_fallback_width(self) -> None:
        track = load_track(_write_circle_track(self.directory))
        with self.assertRaisesRegex(ValueError, "provide half_width"):
            corridor_from_track(track)

    def test_a_track_without_edges_uses_a_constant_assumed_width(self) -> None:
        track = load_track(_write_circle_track(self.directory))
        corridor = corridor_from_track(track, half_width=2.0, margin=0.25)
        self.assertEqual(corridor.provenance, "assumed")
        np.testing.assert_allclose(corridor.minimum, -1.75)
        np.testing.assert_allclose(corridor.maximum, 1.75)

    def test_a_fallback_width_cannot_override_track_edges(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        with self.assertRaisesRegex(ValueError, "only valid for a track without edges"):
            corridor_from_track(track, half_width=2.0)

    def test_a_track_without_edges_rejects_a_nonpositive_fallback(self) -> None:
        track = load_track(_write_circle_track(self.directory))
        with self.assertRaisesRegex(ValueError, "must be finite and positive"):
            corridor_from_track(track, half_width=0.0)

    def test_arrays_are_read_only(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        corridor = corridor_from_track(track)
        with self.assertRaises(ValueError):
            corridor.maximum[0] = 0.0

    def test_half_width_does_not_imply_symmetric_zero_bounds(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        corridor = corridor_from_track(track, margin=2.25)
        self.assertEqual(corridor.half_width, 0.0)

    def test_open_track_is_not_periodic_even_when_endpoint_bounds_match(self) -> None:
        from dataclasses import replace

        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        open_track = replace(track, is_closed=False)
        corridor = corridor_from_track(open_track)
        self.assertFalse(corridor.is_periodic)
