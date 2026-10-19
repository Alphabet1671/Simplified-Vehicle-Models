"""Check the screen-space glyph helpers and the vehicle outline extraction."""
import unittest

import numpy as np

from tools.anim.glyphs import ARROW_POINTS, arc_arrow, arrow_polyline, fit_limits, rotate_clockwise
from tools.anim.scene import build_vehicle_edges, feature_edges, placeholder_scene
from tools.anim.stl import box_triangles, wheel_cylinder_triangles

GEOMETRY = {"wheel_positions_body_m": [[0.8, -0.6], [0.8, 0.6], [-0.7, -0.6], [-0.7, 0.6]],
            "cg_height_m": 0.3}


class GlyphTest(unittest.TestCase):
    def test_clockwise_rotation_turns_up_toward_right(self):
        np.testing.assert_allclose(rotate_clockwise([0.0, 1.0], 90.0), [1.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(rotate_clockwise([1.0, 0.0], 90.0), [0.0, -1.0], atol=1e-12)

    def test_arrow_has_fixed_point_count_and_exact_length(self):
        poly = arrow_polyline([1.0, 2.0], [3.0, 4.0])
        self.assertEqual(poly.shape, (ARROW_POINTS, 2))
        np.testing.assert_allclose(poly[0], [1.0, 2.0])
        np.testing.assert_allclose(poly[1], [4.0, 6.0])
        np.testing.assert_allclose(poly[1], poly[3])
        # both barbs sit behind the tip at the same distance
        back = [np.linalg.norm(poly[k] - poly[1]) for k in (2, 4)]
        self.assertAlmostEqual(back[0], back[1])
        self.assertAlmostEqual(back[0], 0.22 * 5.0)

    def test_zero_arrow_collapses_and_missing_arrow_is_nan_not_zero(self):
        zero = arrow_polyline([1.0, 1.0], [0.0, 0.0])
        self.assertTrue(np.all(zero == [1.0, 1.0]))
        self.assertTrue(np.all(np.isnan(arrow_polyline([0.0, 0.0], [np.nan, 1.0]))))

    def test_arc_arrow_sweeps_clockwise_for_positive_and_counter_for_negative(self):
        up = arc_arrow([0.0, 0.0], 1.0, 90.0, 90.0, points=10)
        np.testing.assert_allclose(up[0], [0.0, 1.0], atol=1e-12)
        np.testing.assert_allclose(up[9], [1.0, 0.0], atol=1e-12)      # clockwise: up -> right
        down = arc_arrow([0.0, 0.0], 1.0, 90.0, -90.0, points=10)
        np.testing.assert_allclose(down[9], [-1.0, 0.0], atol=1e-12)
        self.assertEqual(up.shape, (13, 2))
        self.assertTrue(np.all(np.isnan(arc_arrow([0, 0], 1.0, 0.0, np.nan))))

    def test_arc_arrow_head_points_along_the_travel_direction(self):
        poly = arc_arrow([0.0, 0.0], 1.0, 90.0, 90.0, points=40)
        tip, barb = poly[-2], poly[-3]
        # clockwise travel at screen-right is downward, so the barb lies above the tip
        self.assertGreater(barb[1], tip[1])

    def test_fit_limits_keeps_equal_scale_and_contains_the_box(self):
        (x0, x1), (y0, y1) = fit_limits(0.0, 10.0, 0.0, 2.0, aspect=2.0)
        self.assertAlmostEqual((x1 - x0) / (y1 - y0), 2.0)
        self.assertLessEqual(x0, 0.0)
        self.assertGreaterEqual(x1, 10.0)
        self.assertLessEqual(y0, 0.0)
        self.assertGreaterEqual(y1, 2.0)
        (x0, x1), (y0, y1) = fit_limits(0.0, 2.0, 0.0, 10.0, aspect=2.0)
        self.assertAlmostEqual((x1 - x0) / (y1 - y0), 2.0)
        self.assertGreaterEqual(y1 - y0, 10.0)
        with self.assertRaises(ValueError):
            fit_limits(0, 1, 0, 1, aspect=0.0)


class EdgeTest(unittest.TestCase):
    def test_box_draws_twelve_edges_not_the_triangulation_diagonals(self):
        self.assertEqual(feature_edges(box_triangles((2.0, 1.0, 0.5))).shape, (12, 2, 3))

    def test_cylinder_draws_rims_and_no_cap_fans(self):
        edges = feature_edges(wheel_cylinder_triangles(0.2, 0.18))
        self.assertEqual(edges.shape[0], 48)                  # two 24-gon rims
        radius = np.hypot(edges[..., 0], edges[..., 2])
        np.testing.assert_allclose(radius, 0.2, atol=1e-9)

    def test_an_empty_input_has_no_edges(self):
        self.assertEqual(feature_edges(np.zeros((0, 3, 3))).shape, (0, 2, 3))

    def test_posed_wheels_have_a_constant_segment_count_and_rotate_with_steering(self):
        edges = build_vehicle_edges(placeholder_scene(GEOMETRY))
        flat = edges.posed()
        steered = edges.posed(steering=[0.3, 0.3, 0.0, 0.0], spin=[1.0, 1.0, 1.0, 1.0])
        self.assertEqual(flat.shape, steered.shape)
        self.assertEqual(flat.shape[0], 12 + 4 * (48 + 1))
        # a steered wheel's rim stays at the wheel centre
        rim = steered[12:12 + 48].reshape(-1, 3)
        np.testing.assert_allclose(rim.mean(axis=0), edges.wheel_centres[0], atol=1e-9)
        # steering by 0.3 rad moves the rim's extreme x point
        self.assertNotAlmostEqual(float(np.ptp(flat[12:60, :, 1])),
                                  float(np.ptp(steered[12:60, :, 1])), places=3)

    def test_wheel_centres_sit_one_radius_above_the_ground(self):
        edges = build_vehicle_edges(placeholder_scene(GEOMETRY, wheel_radius=0.25))
        np.testing.assert_allclose(edges.wheel_centres[:, 2], 0.3 - 0.25)
        np.testing.assert_allclose(edges.wheel_radii, 0.25)

    def test_placeholder_rejects_a_bad_wheel_table(self):
        with self.assertRaises(ValueError):
            placeholder_scene({"wheel_positions_body_m": [[0, 0]], "cg_height_m": 0.3})


if __name__ == "__main__":
    unittest.main()
