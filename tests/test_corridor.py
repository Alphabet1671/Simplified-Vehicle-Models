from pathlib import Path
import tempfile
import unittest
from dataclasses import replace

import numpy as np

from simplified_vehicle_models.tracks.corridor import corridor_from_track
from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve
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
        track = load_track(_write_circle_track(self.directory, width_rows=240, side="left"))
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
        with self.assertRaises(ValueError): corridor_from_track(track, margin=3.0)

    def test_fold_guard_clips_a_tight_corner(self) -> None:
        track = load_track(_write_circle_track(self.directory, radius=3.0, half_width=4.0))
        corridor = corridor_from_track(track, fold_guard=0.2)
        fold = 1.0 - track.curvature * corridor.maximum
        self.assertGreaterEqual(float(np.min(fold)), 0.2 - 1e-12)
        self.assertLess(float(np.max(corridor.maximum)), 4.0)

    def test_a_track_without_edges_requires_a_fallback_width(self) -> None:
        track = load_track(_write_circle_track(self.directory))
        with self.assertRaisesRegex(ValueError, "provide half_width"): corridor_from_track(track)

    def test_a_track_without_edges_uses_a_constant_assumed_width(self) -> None:
        track = load_track(_write_circle_track(self.directory))
        corridor = corridor_from_track(track, half_width=2.0, margin=0.25)
        self.assertEqual(corridor.provenance, "assumed")
        np.testing.assert_allclose(corridor.minimum, -1.75)
        np.testing.assert_allclose(corridor.maximum, 1.75)

    def test_a_fallback_width_cannot_override_track_edges(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        with self.assertRaisesRegex(ValueError, "only valid for a track without edges"): corridor_from_track(track, half_width=2.0)

    def test_a_track_without_edges_rejects_a_nonpositive_fallback(self) -> None:
        track = load_track(_write_circle_track(self.directory))
        with self.assertRaisesRegex(ValueError, "must be finite and positive"): corridor_from_track(track, half_width=0.0)

    def test_arrays_are_read_only(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        corridor = corridor_from_track(track)
        with self.assertRaises(ValueError): corridor.maximum[0] = 0.0

    def test_half_width_does_not_imply_symmetric_zero_bounds(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        self.assertEqual(corridor_from_track(track, margin=2.25).half_width, 0.0)

    def test_open_track_is_not_periodic_even_when_endpoint_bounds_match(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        self.assertFalse(corridor_from_track(replace(track, is_closed=False)).is_periodic)

    def test_signed_fitted_policy_retains_outside_turn_width_and_uses_reference(self) -> None:
        track = load_track(_write_circle_track(self.directory, radius=3.0, half_width=4.0))
        ref = ReferenceCurve(track)
        k = np.asarray(ref.evaluate(track.distance).curvature)
        self.assertTrue(np.all(k > 0))
        # Make source curvature deliberately disagree; fitted k remains authoritative.
        altered = replace(track, curvature=-np.abs(track.curvature))
        c = corridor_from_track(altered, policy="signed-fitted-reference-v1", reference=ref)
        bound = 0.8 / k
        np.testing.assert_allclose(c.maximum, np.minimum(4.0, bound), rtol=0, atol=1e-12)
        np.testing.assert_allclose(c.minimum, -4.0)
        self.assertEqual(c.policy, "signed-fitted-reference-v1")
        self.assertEqual(c.provenance, "assumed")

    def test_signed_negative_curvature_caps_lower_side(self) -> None:
        track = load_track(_write_circle_track(self.directory, radius=3.0, half_width=4.0))
        coordinates = track.path_coordinates
        reversed_coordinates = replace(coordinates, y_east=-coordinates.y_east)
        reverse_track = replace(track, path_coordinates=reversed_coordinates)
        ref = ReferenceCurve(reverse_track)
        c = corridor_from_track(track, policy="signed-fitted-reference-v1", reference=ref)
        self.assertTrue(np.all(c.minimum > -4.0))
        np.testing.assert_allclose(c.maximum, 4.0)

    def test_signed_policy_requires_valid_reference_and_policy_parameters(self) -> None:
        track = load_track(_write_circle_track(self.directory, radius=3.0, half_width=4.0))
        with self.assertRaises(ValueError): corridor_from_track(track, policy="signed-fitted-reference-v1")
        with self.assertRaises(ValueError): corridor_from_track(track, policy="unknown")
        with self.assertRaises((TypeError, ValueError)): corridor_from_track(track, fold_guard=True)
        ref = ReferenceCurve(track)
        with self.assertRaises(ValueError): corridor_from_track(track, policy="signed-fitted-reference-v1", reference=object())
        with self.assertRaises(ValueError): corridor_from_track(replace(track, distance=track.distance + 1), policy="signed-fitted-reference-v1", reference=ref)

    def test_policy_descriptor_is_immutable_and_hash_tracks_inputs(self) -> None:
        track = load_track(_write_circle_track(self.directory, radius=3.0, half_width=4.0))
        ref = ReferenceCurve(track)
        base = corridor_from_track(track, policy="signed-fitted-reference-v1", reference=ref)
        self.assertEqual(len(base.policy_hash), 64)
        self.assertNotEqual(base.policy_hash, corridor_from_track(track, policy="signed-fitted-reference-v1", reference=ref, fold_guard=0.3).policy_hash)
        self.assertNotEqual(base.policy_hash, corridor_from_track(track, policy="signed-fitted-reference-v1", reference=ref, margin=0.1).policy_hash)
        with self.assertRaises(TypeError): base.policy_descriptor["policy"] = "x"

    def test_policy_hash_must_match_descriptor_and_nested_payload_is_canonical(self) -> None:
        from simplified_vehicle_models.tracks.corridor import Corridor
        track = load_track(_write_circle_track(self.directory, radius=3.0, half_width=4.0))
        c = corridor_from_track(track, policy="signed-fitted-reference-v1", reference=ReferenceCurve(track))
        with self.assertRaises(ValueError):
            Corridor(c.distance, c.minimum, c.maximum, c.provenance, c.is_closed, c.policy, c.policy_descriptor, "0"*64)
        alias = ["mutable"]
        descriptor = {"nested": alias}
        bad = Corridor(c.distance, c.minimum, c.maximum, c.provenance, c.is_closed, c.policy, descriptor)
        alias.append("changed")
        self.assertEqual(tuple(bad.policy_descriptor["nested"]), ("mutable",))
        with self.assertRaises((TypeError, ValueError)):
            Corridor(c.distance, c.minimum, c.maximum, c.provenance, c.is_closed, c.policy, {"bad": np.array([1.0])})
        with self.assertRaises((TypeError, ValueError)):
            Corridor(c.distance, c.minimum, c.maximum, c.provenance, c.is_closed, c.policy, {"bad": object()})
        with self.assertRaises(ValueError):
            Corridor(c.distance, c.minimum, c.maximum, c.provenance, c.is_closed, c.policy, {}, "0"*64)

    def test_reference_source_geometry_changes_policy_hash(self) -> None:
        track = load_track(_write_circle_track(self.directory, radius=3.0, half_width=4.0))
        ref = ReferenceCurve(track)
        coords = track.path_coordinates
        changed_xy = replace(coords, y_east=coords.y_east + 0.02*np.sin(np.linspace(0, 2*np.pi, len(coords.y_east))))
        changed_ref = ReferenceCurve(replace(track, path_coordinates=changed_xy))
        first = corridor_from_track(track, policy="signed-fitted-reference-v1", reference=ref)
        second = corridor_from_track(track, policy="signed-fitted-reference-v1", reference=changed_ref)
        self.assertNotEqual(first.policy_hash, second.policy_hash)

if __name__ == "__main__": unittest.main()
