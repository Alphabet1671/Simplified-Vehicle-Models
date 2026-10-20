import json
import unittest

import numpy as np

from tools.track_gui.design import (MAXIMUM_LENGTH, PRESETS, CourseDesign, SectionSpec,
                                    acceleration_design, skidpad_design)
from tools.tracks import build_fsae_dynamic_tracks as fsae


def same_course(a, b):
    return (len(a.distance) == len(b.distance) and a.names == b.names
            and np.allclose(a.x_north, b.x_north, atol=1e-9)
            and np.allclose(a.y_east, b.y_east, atol=1e-9)
            and np.allclose(a.curvature, b.curvature, atol=1e-12))


class DesignTests(unittest.TestCase):
    def test_acceleration_preset_equals_the_committed_builder(self):
        self.assertTrue(same_course(acceleration_design().build(), fsae.acceleration_course()[0]))

    def test_skidpad_presets_equal_the_committed_builder(self):
        for length in (0.0, 6.0):
            with self.subTest(transition=length):
                self.assertTrue(same_course(skidpad_design(length).build(), fsae.skidpad_course(length)[0]))

    def test_every_preset_builds(self):
        for name, factory in PRESETS.items():
            self.assertGreater(factory().build().length, 10.0, name)

    def test_curvature_jump_is_reported_with_the_section_names(self):
        design = CourseDesign((SectionSpec("straight", "a", length=5.0),
                               SectionSpec("arc", "b", length=5.0, curvature=0.1)))
        with self.assertRaisesRegex(ValueError, "curvature jumps.*'a'.*'b'"):
            design.build()
        self.assertEqual(len(CourseDesign(design.sections, allow_curvature_jumps=True).build().curvature_jumps), 1)

    def test_bad_section_names_its_row(self):
        design = CourseDesign((SectionSpec("straight", "ok", length=5.0),
                               SectionSpec("arc-angle", "bad", radius=-1.0, angle_deg=90.0)))
        with self.assertRaisesRegex(ValueError, "section 2 .bad."):
            design.build()

    def test_empty_unknown_and_nonfinite_are_refused(self):
        with self.assertRaisesRegex(ValueError, "no sections"):
            CourseDesign().build()
        with self.assertRaisesRegex(ValueError, "unknown section kind"):
            CourseDesign((SectionSpec("spiral"),)).build()
        with self.assertRaises(ValueError):
            CourseDesign((SectionSpec("straight", length=float("nan")),)).build()
        with self.assertRaisesRegex(ValueError, "spacing"):
            CourseDesign((SectionSpec(),), maximum_spacing=0.0).build()

    def test_too_long_is_refused(self):
        with self.assertRaisesRegex(ValueError, "longer than"):
            CourseDesign((SectionSpec("straight", length=60000.0),)).build()

    def test_build_rejects_invalid_settings(self):
        section = (SectionSpec(),)
        for field in ("start_heading", "start_north", "start_east", "jump_ramp_length"):
            for value in (float("nan"), float("inf"), "bad"):
                with self.subTest(field=field, value=value):
                    with self.assertRaisesRegex(ValueError, field):
                        CourseDesign(section, **{field: value}).build()
        with self.assertRaisesRegex(ValueError, "allow_curvature_jumps"):
            CourseDesign(section, allow_curvature_jumps=1).build()

    def test_jump_ramps_count_toward_maximum_length(self):
        design = CourseDesign(
            (SectionSpec("straight", length=MAXIMUM_LENGTH - 0.01),
             SectionSpec("arc", length=0.01, curvature=0.1)),
            allow_curvature_jumps=True,
        )
        with self.assertRaisesRegex(ValueError, "longer than"):
            design.build()

    def test_json_rejects_malformed_structure_and_types(self):
        payload = json.loads(acceleration_design().to_json())
        cases = [
            ("unknown", {**payload, "extra": 1}),
            ("missing sections", {key: value for key, value in payload.items() if key != "sections"}),
            ("non-object row", {**payload, "sections": [1]}),
            ("wrong setting type", {**payload, "maximum_spacing": "bad"}),
            ("wrong section type", {**payload, "sections": [{**payload["sections"][0], "length": "bad"}]}),
        ]
        for label, bad in cases:
            with self.subTest(case=label):
                with self.assertRaises(ValueError):
                    CourseDesign.from_json(json.dumps(bad))

    def test_start_pose_moves_the_course(self):
        course = CourseDesign((SectionSpec("straight", length=10.0),), start_north=3.0, start_east=-2.0,
                              start_heading=np.pi / 2).build()
        self.assertAlmostEqual(course.x_north[0], 3.0)
        self.assertAlmostEqual(course.y_east[-1], -2.0 + 10.0, places=6)

    def test_right_turn_is_positive_and_turns_east(self):
        course = CourseDesign((SectionSpec("arc-angle", radius=10.0, angle_deg=90.0),)).build()
        self.assertGreater(course.curvature[0], 0.0)
        self.assertAlmostEqual(course.heading[-1], np.pi / 2, places=6)

    def test_json_round_trip(self):
        design = skidpad_design(6.0)
        self.assertEqual(CourseDesign.from_json(design.to_json()), design)

    def test_json_rejects_foreign_and_unknown(self):
        with self.assertRaisesRegex(ValueError, "not a track design"):
            CourseDesign.from_json("{}")
        with self.assertRaisesRegex(ValueError, "not a track design"):
            CourseDesign.from_json("not json")
        bad = json.loads(acceleration_design().to_json())
        bad["sections"][0]["colour"] = 1
        with self.assertRaisesRegex(ValueError, "unknown section field"):
            CourseDesign.from_json(json.dumps(bad))


if __name__ == "__main__":
    unittest.main()
