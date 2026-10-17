"""Check the STL reader, placeholder meshes and the orthographic projections."""
import struct
import tempfile
import unittest
from pathlib import Path

import numpy as np

from tools.anim.scene import VIEWS, body_rotation, project, transform
from tools.anim.stl import (
    StlError, bounding_box, box_triangles, read_stl, wheel_cylinder_triangles,
)

ASCII_ONE = """solid one
  facet normal 0 0 1
    outer loop
      vertex 0 0 0
      vertex 1.5 0 0
      vertex 0 2 -0.25
    endloop
  endfacet
endsolid one
"""


def binary_stl(triangles, header=b"binary file", attribute=0):
    body = b"".join(
        struct.pack("<3f", 0, 0, 1) + b"".join(struct.pack("<3f", *v) for v in tri)
        + struct.pack("<H", attribute) for tri in triangles)
    return header.ljust(80, b" ")[:80] + struct.pack("<I", len(triangles)) + body


class StlReaderTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def write(self, data, name="m.stl"):
        path = Path(self.directory.name) / name
        path.write_bytes(data if isinstance(data, bytes) else data.encode())
        return path

    def test_ascii_one_triangle_is_exact(self):
        triangles = read_stl(self.write(ASCII_ONE))
        self.assertEqual(triangles.shape, (1, 3, 3))
        self.assertEqual(triangles.dtype, np.float64)
        np.testing.assert_array_equal(triangles[0], [[0, 0, 0], [1.5, 0, 0], [0, 2, -0.25]])

    def test_binary_two_triangles_are_exact_even_when_the_header_says_solid(self):
        expected = np.array([[[0, 0, 0], [1, 0, 0], [0, 1, 0]],
                             [[0, 0, 1], [2, 0, 1], [0, 4, 1.5]]], dtype=float)
        for header in (b"binary file", b"solid exported by a CAD tool"):
            triangles = read_stl(self.write(binary_stl(expected, header)))
            np.testing.assert_array_equal(triangles, expected)

    def test_binary_attribute_bytes_are_ignored(self):
        expected = np.array([[[0, 0, 0], [1, 0, 0], [0, 1, 0]]], dtype=float)
        np.testing.assert_array_equal(read_stl(self.write(binary_stl(expected, attribute=7))),
                                      expected)

    def test_truncated_and_oversized_binary_raise(self):
        data = binary_stl(np.zeros((3, 3, 3)) + np.arange(3))
        for broken in (data[:-10], data + b"\0", data[:60], b""):
            with self.assertRaises(StlError):
                read_stl(self.write(broken))

    def test_non_finite_vertex_raises(self):
        triangles = np.array([[[0, 0, 0], [1, 0, 0], [0, np.nan, 0]]], dtype=float)
        with self.assertRaises(StlError):
            read_stl(self.write(binary_stl(triangles)))

    def test_truncated_ascii_raises(self):
        with self.assertRaises(StlError):
            read_stl(self.write(ASCII_ONE.split("endloop")[0]))

    def test_empty_binary_with_zero_triangles_raises(self):
        with self.assertRaises(StlError):
            read_stl(self.write(binary_stl(np.zeros((0, 3, 3)))))


class MeshTest(unittest.TestCase):
    def test_box_is_centred_closed_and_the_right_size(self):
        box = box_triangles((2.0, 1.0, 0.5))
        self.assertEqual(box.shape, (12, 3, 3))
        low, high = bounding_box(box)
        np.testing.assert_allclose(low, [-1.0, -0.5, -0.25])
        np.testing.assert_allclose(high, [1.0, 0.5, 0.25])
        # Closed surface: every undirected edge is shared by exactly two triangles.
        edges = {}
        for tri in box:
            for a, b in ((0, 1), (1, 2), (2, 0)):
                key = frozenset((tuple(tri[a]), tuple(tri[b])))
                edges[key] = edges.get(key, 0) + 1
        self.assertTrue(all(count == 2 for count in edges.values()))
        # Outward orientation: the signed volume is positive and equals the box volume.
        volume = np.sum(np.einsum("ij,ij->i", box[:, 0], np.cross(box[:, 1], box[:, 2]))) / 6.0
        self.assertAlmostEqual(volume, 2.0 * 1.0 * 0.5)

    def test_box_rejects_bad_sizes(self):
        for sizes in ((0, 1, 1), (1, -1, 1), (1, 1), (1, np.nan, 1)):
            with self.assertRaises(ValueError):
                box_triangles(sizes)

    def test_wheel_cylinder_axis_radius_and_width(self):
        wheel = wheel_cylinder_triangles(0.2, 0.18, segments=24)
        points = wheel.reshape(-1, 3)
        radial = np.hypot(points[:, 0], points[:, 2])
        self.assertAlmostEqual(float(radial.max()), 0.2, places=12)
        self.assertAlmostEqual(float(np.abs(points[:, 1]).max()), 0.09, places=12)
        low, high = bounding_box(wheel)
        np.testing.assert_allclose(low[1], -0.09)
        np.testing.assert_allclose(high[1], 0.09)
        volume = np.sum(np.einsum("ij,ij->i", wheel[:, 0], np.cross(wheel[:, 1], wheel[:, 2]))) / 6.0
        self.assertGreater(volume, 0.0)
        self.assertAlmostEqual(volume, 0.5 * 24 * 0.2**2 * np.sin(2 * np.pi / 24) * 0.18, places=12)


class ProjectionTest(unittest.TestCase):
    def test_views_are_orthonormal_and_never_mirrored(self):
        # Camera forward f satisfies right x up = -f, so the picture is a true view.
        expected_forward = {"rear": [1, 0, 0], "right": [0, -1, 0], "top": [0, 0, 1],
                            "track": [0, 0, 1]}
        for name, (right, up) in VIEWS.items():
            self.assertAlmostEqual(float(np.linalg.norm(right)), 1.0)
            self.assertAlmostEqual(float(right @ up), 0.0)
            np.testing.assert_allclose(-np.cross(right, up), expected_forward[name], err_msg=name)

    def test_box_projections_pick_the_named_coordinates(self):
        box = box_triangles((2.0, 1.0, 0.5)) + np.array([0.3, 0.2, -0.1])
        rear = project(box, *VIEWS["rear"])
        np.testing.assert_allclose(rear[..., 0], box[..., 1])
        np.testing.assert_allclose(rear[..., 1], -box[..., 2])
        side = project(box, *VIEWS["right"])
        np.testing.assert_allclose(side[..., 0], box[..., 0])
        np.testing.assert_allclose(side[..., 1], -box[..., 2])
        self.assertEqual(rear.shape, (12, 3, 2))

    def test_project_rejects_non_orthonormal_axes(self):
        with self.assertRaises(ValueError):
            project(np.zeros((1, 3)), [1, 0, 0], [1, 0, 0])
        with self.assertRaises(ValueError):
            project(np.zeros((1, 3)), [2, 0, 0], [0, 1, 0])

    def test_positive_roll_takes_body_y_down_and_clockwise_in_the_rear_view(self):
        roll = 0.2
        y_axis = body_rotation(0, 0, roll) @ [0, 1, 0]
        np.testing.assert_allclose(y_axis, [0, np.cos(roll), np.sin(roll)], atol=1e-15)
        self.assertGreater(y_axis[2], 0.0)  # +z is down: right side down
        screen = project(y_axis, *VIEWS["rear"])
        np.testing.assert_allclose(screen, [np.cos(roll), -np.sin(roll)], atol=1e-15)
        # Clockwise on screen: the right-pointing line has dropped below the horizon.
        self.assertLess(screen[1], 0.0)

    def test_positive_pitch_is_nose_up_in_the_right_view(self):
        pitch = 0.1
        x_axis = body_rotation(0, pitch, 0) @ [1, 0, 0]
        np.testing.assert_allclose(x_axis, [np.cos(pitch), 0, -np.sin(pitch)], atol=1e-15)
        screen = project(x_axis, *VIEWS["right"])
        np.testing.assert_allclose(screen, [np.cos(pitch), np.sin(pitch)], atol=1e-15)
        self.assertGreater(screen[1], 0.0)  # the front rises

    def test_positive_yaw_turns_right_which_is_east_on_the_map(self):
        yaw = 0.3
        np.testing.assert_allclose(body_rotation(yaw, 0, 0) @ [1, 0, 0],
                                   [np.cos(yaw), np.sin(yaw), 0], atol=1e-15)
        # Heading north (+X) turning clockwise from above moves towards east (+Y):
        # on the map (east right, north up) the heading arrow tilts to the right.
        screen = project(body_rotation(yaw, 0, 0) @ [1, 0, 0], *VIEWS["track"])
        self.assertGreater(screen[0], 0.0)
        self.assertGreater(screen[1], 0.0)

    def test_rotation_properties(self):
        np.testing.assert_array_equal(body_rotation(0, 0, 0), np.eye(3))
        rotation = body_rotation(0.7, -0.2, 0.4)
        np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-14)
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0)
        # yaw, then pitch, then roll: Rz @ Ry @ Rx.
        np.testing.assert_allclose(
            rotation, body_rotation(0.7, 0, 0) @ body_rotation(0, -0.2, 0) @ body_rotation(0, 0, 0.4),
            atol=1e-14)

    def test_transform_applies_rotation_scale_then_translation(self):
        triangles = np.array([[[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]])
        rotation = body_rotation(np.pi / 2, 0, 0)
        moved = transform(triangles, rotation, (1.0, 2.0, 3.0), scale=2.0)
        np.testing.assert_allclose(moved[0, 0], [1.0, 2.0 + 2.0, 3.0], atol=1e-15)
        np.testing.assert_allclose(moved[0, 1], [1.0 - 2.0, 2.0, 3.0], atol=1e-15)
        np.testing.assert_array_equal(triangles[0, 0], [1.0, 0, 0])  # input is not modified


if __name__ == "__main__":
    unittest.main()
