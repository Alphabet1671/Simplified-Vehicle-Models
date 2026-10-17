"""Check the exporter against one synthetic record per model class (plan section 6.1).

The records have the state layout and the channel set each model class provides.
Nothing here special-cases a model: the window set must follow the channels alone.
"""
import tempfile
import unittest

import numpy as np

from tests.test_animation_panels import (BASE, DYNAMIC, GEOMETRY, QUASI, WHEEL, make_source,
                                         track_geometry)
from tools.anim.encode import NullFrameWriter
from tools.anim.layout import WINDOWS, window_grid, window_visibility
from tools.anim.render import build_figure, render_frames
from tools.anim.resample import MissingChannelError
from tools.anim.scene import placeholder_scene
from tools.anim.style import AnimationStyle

SMALL = AnimationStyle(width=320, height=240, dpi=40)
SECOND = 119.0 / 120.0            # 120 frames: 0, 1/120, ... 119/120

MODELS = {
    "point-mass": dict(
        states=("s", "v_x"), channels=BASE),
    "planar-7dof": dict(
        states=("s", "n", "heading_error", "v_x", "v_y", "yaw_rate", "omega_fl", "omega_fr",
                "omega_rl", "omega_rr", "soc", "steering"),
        channels=BASE + WHEEL + QUASI),
    "body-10dof": dict(
        states=("s", "n", "heading_error", "v_x", "v_y", "yaw_rate", "roll", "pitch",
                "roll_rate", "pitch_rate", "wheel_speed_fl", "wheel_speed_fr", "wheel_speed_rl",
                "wheel_speed_rr"),
        channels=BASE + WHEEL + ("roll", "pitch", "roll-rate", "pitch-rate")),
    "14dof-simple": dict(
        states=("s", "n", "heading_error", "v_x", "v_y", "yaw_rate", "roll", "pitch", "heave",
                "roll_rate", "pitch_rate", "heave_rate", "wheel_speed_fl", "wheel_speed_fr",
                "wheel_speed_rl", "wheel_speed_rr"),
        channels=BASE + WHEEL + DYNAMIC + ("roll-rate", "pitch-rate")),
}

#: (visible windows, attitude_is_dynamic) per model without any option.
EXPECTED = {
    "point-mass": (("track",), False),
    "planar-7dof": (("track", "top"), False),
    "body-10dof": (WINDOWS, True),
    "14dof-simple": (WINDOWS, True),
}


class ModelMatrixTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.sources = {}
        for name, spec in MODELS.items():
            channels = tuple(dict.fromkeys(spec["channels"]))
            cls.sources[name] = make_source(
                cls.directory.name, channels, model=name, state_names=spec["states"],
                steps=121, duration=1.0, t_start=0.0, t_end=SECOND,
                quasi=False)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def figure(self, name, **options):
        source = self.sources[name]
        return (source,) + build_figure(source.record, source, placeholder_scene(GEOMETRY),
                                        track_geometry(), SMALL, **options)

    def test_every_excerpt_is_exactly_120_frames(self):
        for name, source in self.sources.items():
            self.assertEqual(source.count, 120, name)

    def test_the_window_matrix_follows_section_6_1(self):
        for name, (visible, dynamic) in EXPECTED.items():
            v = window_visibility(self.sources[name])
            self.assertEqual(v.visible, visible, name)
            self.assertEqual(v.attitude_is_dynamic, dynamic, name)

    def test_point_mass_has_no_wheel_force_channel_and_hides_rear_right_top(self):
        source = self.sources["point-mass"]
        with self.assertRaises(MissingChannelError) as caught:
            source.require("tire-longitudinal-force")
        self.assertIn("tire-longitudinal-force", str(caught.exception))
        self.assertIn("point-mass", str(caught.exception))
        self.assertEqual(window_visibility(source).hidden, ("rear", "right", "top"))

    def test_planar_hides_attitude_unless_the_flag_and_the_pair_are_both_present(self):
        source = self.sources["planar-7dof"]
        self.assertEqual(window_visibility(source).hidden, ("rear", "right"))
        v = window_visibility(source, allow_quasi_static=True)
        self.assertEqual(v.visible, WINDOWS)
        self.assertFalse(v.attitude_is_dynamic)
        self.assertTrue(v.attitude_is_approximate)

    def test_the_flag_has_no_effect_where_the_pair_is_absent(self):
        for name in ("point-mass",):
            self.assertEqual(window_visibility(self.sources[name]).visible,
                             window_visibility(self.sources[name],
                                               allow_quasi_static=True).visible)

    def test_dynamic_models_ignore_the_flag(self):
        for name in ("body-10dof", "14dof-simple"):
            v = window_visibility(self.sources[name], allow_quasi_static=True)
            self.assertTrue(v.attitude_is_dynamic and not v.attitude_is_approximate, name)

    def test_the_figure_has_exactly_the_visible_windows_as_axes(self):
        for name, (visible, _) in EXPECTED.items():
            _, figure, panels, visibility = self.figure(name)
            self.assertEqual(tuple(panels), visible, name)
            self.assertEqual(len(figure.axes), len(visible), name)
            self.assertEqual(visibility.visible, visible, name)

    def test_freed_space_follows_the_layout_rules(self):
        full = window_grid(window_visibility(self.sources["14dof-simple"]))
        planar = window_grid(window_visibility(self.sources["planar-7dof"]))
        point = window_grid(window_visibility(self.sources["point-mass"]))
        # the track always spans the full height of the window area
        for layout in (full, planar, point):
            left, bottom, width, height = layout["track"]
            self.assertAlmostEqual(bottom + height, full["track"][1] + full["track"][3])
        # freed attitude space goes to the top view first
        self.assertGreater(planar["top"][3], full["top"][3])
        self.assertAlmostEqual(planar["top"][3], planar["track"][3])
        # a lone track fills the figure
        self.assertGreater(point["track"][2], 0.95)

    def test_each_excerpt_renders_120_frames_of_the_right_shape(self):
        for name in MODELS:
            source, figure, panels, _ = self.figure(name)
            writer = render_frames(figure, panels, source, NullFrameWriter(320, 240))
            self.assertEqual(writer.frames, 120, name)
            self.assertEqual(writer.shapes, {(240, 320, 3)}, name)
            self.assertEqual(writer.dtypes, {"uint8"}, name)

    def test_warning_text_names_the_hidden_window_the_missing_channel_and_the_model(self):
        _, figure, _, _ = self.figure("planar-7dof")
        text = "\n".join(figure.footer_lines)
        self.assertIn("rear/right", text)
        self.assertIn("roll", text)
        self.assertIn("pitch", text)
        self.assertIn("planar-7dof", text)
        self.assertIn("--quasi-static-attitude", text)
        _, figure, _, _ = self.figure("point-mass")
        text = "\n".join(figure.footer_lines)
        self.assertIn("point-mass", text)
        self.assertIn("rear/right hidden", text)
        self.assertIn("top hidden", text)
        self.assertIn("--force-top-view", text)

    def test_a_full_model_prints_no_hidden_window_note(self):
        for name in ("body-10dof", "14dof-simple"):
            _, figure, _, visibility = self.figure(name)
            self.assertEqual(visibility.notes, (), name)
            self.assertEqual(figure.footer_lines, (), name)

    def test_planar_with_the_flag_labels_the_attitude_approximate(self):
        spec = MODELS["planar-7dof"]
        source = make_source(self.directory.name, tuple(dict.fromkeys(spec["channels"])),
                             model="planar-7dof-quasi", state_names=spec["states"],
                             steps=121, duration=1.0, quasi=True)
        figure, panels, visibility = build_figure(
            source.record, source, placeholder_scene(GEOMETRY), track_geometry(), SMALL,
            allow_quasi_static=True)
        self.assertEqual(tuple(panels), WINDOWS)
        self.assertIn("APPROXIMATE", " ".join(panels["rear"].legend_lines()))

    def test_scales_do_not_depend_on_the_model_name(self):
        scales = {name: source.scales() for name, source in self.sources.items()}
        for name in ("planar-7dof", "body-10dof", "14dof-simple"):
            self.assertAlmostEqual(scales[name]["tire_vertical_force"], 1300.0, msg=name)
            self.assertAlmostEqual(scales[name]["speed"], 20.0, msg=name)
        self.assertNotIn("tire_force", scales["point-mass"])


if __name__ == "__main__":
    unittest.main()
