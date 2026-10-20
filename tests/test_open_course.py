from pathlib import Path
import math
import tempfile
import unittest

import numpy as np

from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
from simplified_vehicle_models.tracks.profile import load_track
from tools.tracks.open_course import (
    arc,
    arc_by_angle,
    build_course,
    straight,
    transition,
    write_course,
)


def _load(course, directory, **keywords):
    xml, _ = write_course(course, directory, "course", "test-course", "Test Course", **keywords)
    return load_track(xml)


class BuildCourseTests(unittest.TestCase):
    def test_straight_runs_along_north(self) -> None:
        course = build_course([straight(10.0)], maximum_spacing=1.0)
        np.testing.assert_allclose(course.x_north, np.arange(11.0), atol=1e-12)
        np.testing.assert_allclose(course.y_east, 0.0, atol=1e-12)

    def test_right_arc_turns_east(self) -> None:
        radius = 10.0
        course = build_course([arc_by_angle(radius, math.pi / 2.0)], maximum_spacing=0.05)
        # A quarter right turn from north ends facing east, a radius ahead and a radius east.
        self.assertAlmostEqual(course.heading[-1], math.pi / 2.0, places=9)
        self.assertAlmostEqual(course.x_north[-1], radius, places=6)
        self.assertAlmostEqual(course.y_east[-1], radius, places=6)
        self.assertTrue(np.all(course.curvature > 0.0))

    def test_left_arc_turns_west(self) -> None:
        course = build_course([arc_by_angle(10.0, -math.pi / 2.0)], maximum_spacing=0.05)
        self.assertAlmostEqual(course.y_east[-1], -10.0, places=6)
        self.assertTrue(np.all(course.curvature < 0.0))

    def test_clothoid_heading_is_the_curvature_integral(self) -> None:
        section = transition(0.0, 0.1, 0.01)
        course = build_course([section], maximum_spacing=0.1)
        expected = 0.5 * 0.1 * section.length  # mean curvature times length
        self.assertAlmostEqual(course.heading[-1], expected, places=9)

    def test_curvature_jump_is_refused(self) -> None:
        with self.assertRaisesRegex(ValueError, "curvature jumps"):
            build_course([straight(5.0), arc(5.0, 0.1)])

    def test_curvature_jump_can_be_allowed_and_is_recorded(self) -> None:
        course = build_course([straight(5.0), arc(5.0, 0.1)], allow_curvature_jumps=True)
        self.assertEqual(len(course.curvature_jumps), 1)
        self.assertAlmostEqual(course.curvature_jumps[0][1], 0.1)
        # The jump becomes a 0.05 m ramp, so the course is that much longer.
        self.assertAlmostEqual(course.length, 10.05, places=9)
        self.assertAlmostEqual(course.heading[-1], 0.1 * 5.0 + 0.5 * 0.1 * 0.05, places=9)
        self.assertLessEqual(float(np.max(np.diff(course.curvature))), 0.1 + 1e-12)

    def test_transition_makes_a_continuous_course(self) -> None:
        sections = [
            straight(5.0),
            transition(0.0, 0.1, 0.01),
            arc(5.0, 0.1),
            transition(0.1, 0.0, 0.01),
            straight(5.0),
        ]
        course = build_course(sections)
        self.assertEqual(len(course.curvature_jumps), 0)
        self.assertLess(float(np.max(np.abs(np.diff(course.curvature)))), 0.1 * 0.25 / 10.0 + 1e-9)

    def test_boundaries_are_nodes(self) -> None:
        course = build_course([straight(1.3), arc(2.7, 0.0)], maximum_spacing=0.5)
        for boundary in course.boundaries:
            self.assertTrue(np.any(np.isclose(course.distance, boundary, atol=1e-12)))

    def test_rejects_empty_and_bad_sections(self) -> None:
        with self.assertRaises(ValueError):
            build_course([])
        with self.assertRaises(ValueError):
            straight(-1.0)


class OpenLoaderTests(unittest.TestCase):
    def test_round_trip(self) -> None:
        course = build_course(
            [straight(5.0), transition(0.0, 0.1, 0.02), arc(8.0, 0.1), transition(0.1, 0.0, 0.02), straight(5.0)]
        )
        with tempfile.TemporaryDirectory() as name:
            track = _load(course, name)
        self.assertFalse(track.is_closed)
        self.assertTrue(track.has_signed_geometry)
        self.assertFalse(track.path_coordinates.is_closed)
        self.assertAlmostEqual(track.lap_length, course.length, places=9)
        np.testing.assert_allclose(track.path_coordinates.x_north, course.x_north)
        np.testing.assert_allclose(track.curvature, course.curvature)
        np.testing.assert_allclose(track.radius[track.curvature == 0.0], 1.0e5)
        # The last node is the end of the course and does not repeat the first.
        self.assertGreater(
            math.hypot(
                track.path_coordinates.x_north[-1] - track.path_coordinates.x_north[0],
                track.path_coordinates.y_east[-1] - track.path_coordinates.y_east[0],
            ),
            1.0,
        )
        self.assertEqual(len(track.segment_length), len(track.distance) - 1)

    def test_arrays_are_read_only(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            track = _load(build_course([straight(5.0)]), name)
        with self.assertRaises(ValueError):
            track.curvature[0] = 1.0

    def _write(self, directory, rows, parameters=None):
        directory = Path(directory)
        (directory / "t.csv").write_text(
            "distance,x_north,y_east,heading_rad,curvature\n" + rows, encoding="utf-8"
        )
        extra = parameters if parameters is not None else '<parameter name="maximum_radius" value="1e5"/>'
        (directory / "t.xml").write_text(
            '<model schema-version="1.0" id="t" type="track-model" subtype="open-xy-profile">'
            f"<metadata><title>t</title></metadata><data ref=\"t.csv\"/><parameters>{extra}</parameters></model>",
            encoding="utf-8",
        )
        return directory / "t.xml"

    def test_rejects_heading_that_disagrees_with_curvature(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            xml = self._write(name, "0,0,0,0,0\n1,1,0,0.5,0\n2,2,0,0.5,0\n")
            with self.assertRaisesRegex(ValueError, "row 3 heading change"):
                load_track(xml)

    def test_rejects_position_that_disagrees_with_distance(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            xml = self._write(name, "0,0,0,0,0\n1,1,0,0,0\n2,5,0,0,0\n")
            with self.assertRaisesRegex(ValueError, "row 4 position step"):
                load_track(xml)

    def test_rejects_position_that_disagrees_with_heading(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            xml = self._write(name, "0,0,0,0,0\n1,1,0,0,0\n2,1,1,0,0\n")
            with self.assertRaisesRegex(ValueError, "row 4 position step does not follow"):
                load_track(xml)

    def test_rejects_closed_flag(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            xml = self._write(
                name,
                "0,0,0,0,0\n1,1,0,0,0\n2,2,0,0,0\n",
                '<parameter name="maximum_radius" value="1e5"/><parameter name="is_closed" value="true"/>',
            )
            with self.assertRaisesRegex(ValueError, "must not be closed"):
                load_track(xml)

    def test_requires_maximum_radius(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            xml = self._write(name, "0,0,0,0,0\n1,1,0,0,0\n2,2,0,0,0\n", "")
            with self.assertRaisesRegex(ValueError, "maximum_radius"):
                load_track(xml)


class OpenEvaluatePathTests(unittest.TestCase):
    def setUp(self) -> None:
        course = build_course([straight(5.0), arc_by_angle(10.0, math.pi / 2.0)], maximum_spacing=0.1,
                              allow_curvature_jumps=True)
        with tempfile.TemporaryDirectory() as name:
            self.track = _load(course, name)
        self.path = self.track.path_coordinates

    def test_inside_matches_the_table(self) -> None:
        x, y, heading, curvature = evaluate_path(self.path, self.path.distance)
        np.testing.assert_allclose(x, self.path.x_north, atol=1e-12)
        np.testing.assert_allclose(heading, self.path.heading, atol=1e-12)

    def test_does_not_wrap_past_the_end(self) -> None:
        end = self.path.lap_length
        x, y, heading, curvature = evaluate_path(self.path, np.array([end + 3.0]))
        # Straight continuation along the final heading (east), with no curvature.
        end_heading = float(self.path.heading[-1])
        self.assertAlmostEqual(float(heading[0]), end_heading, places=12)
        self.assertAlmostEqual(
            float(y[0]), self.path.y_east[-1] + 3.0 * math.sin(end_heading), places=9
        )
        self.assertAlmostEqual(
            float(x[0]), self.path.x_north[-1] + 3.0 * math.cos(end_heading), places=9
        )
        self.assertEqual(float(curvature[0]), 0.0)

    def test_extends_backwards_before_the_start(self) -> None:
        x, y, heading, curvature = evaluate_path(self.path, np.array([-2.0]))
        self.assertAlmostEqual(float(x[0]), -2.0, places=9)
        self.assertAlmostEqual(float(heading[0]), 0.0, places=9)
        self.assertEqual(float(curvature[0]), 0.0)

    def test_closed_path_still_wraps(self) -> None:
        closed = load_track(
            Path(__file__).resolve().parents[1] / "models/tracks/2024_michigan_endurance_multilap_closed.xml"
        ).path_coordinates
        self.assertTrue(closed.is_closed)
        length = closed.lap_length
        a = evaluate_path(closed, np.array([10.0]))
        b = evaluate_path(closed, np.array([10.0 + length]))
        np.testing.assert_allclose(a[0], b[0], atol=1e-9)
        np.testing.assert_allclose(a[3], b[3], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
