import math
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from tests.test_animation_cli import record_file
from tests.test_animation_panels import BASE, DYNAMIC, WHEEL
from tools.anim_gui import settings as S
from tools.gui_common import REPOSITORY_ROOT


def make_settings(record, **changes):
    return replace(S.ExportSettings(record=Path(record), output=Path(record).with_suffix(".mp4")), **changes)


class SettingsTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.record, _ = record_file(self._tmp.name, BASE + WHEEL + DYNAMIC)

    def test_defaults_match_the_exporter_defaults(self):
        from tools.anim import export_animation as cli
        defaults = cli.build_parser().parse_args(["--record", "x"])
        settings = make_settings(self.record)
        self.assertEqual((settings.frame_rate, settings.width, settings.height, settings.crf, settings.preset),
                         (defaults.frame_rate, defaults.width, defaults.height, defaults.crf, defaults.preset))
        self.assertEqual((settings.wheel_radius, settings.wheel_width, settings.transform_scale),
                         (defaults.wheel_radius, defaults.wheel_width, defaults.transform_scale))

    def test_arguments_are_accepted_by_the_real_parser(self):
        from tools.anim import export_animation as cli
        full = make_settings(self.record, time_range=(0.1, 0.3), stl_body=Path("b.stl"), stl_wheel=Path("w.stl"),
                             body_length=2.5, body_width=1.2, body_height=0.9, force_top_view=True,
                             quasi_static_attitude=True, frame_rate=60.0, width=640, height=360, crf=23, preset="fast")
        args = cli.build_parser().parse_args(full.arguments(progress=True))
        self.assertEqual((args.time_range, args.frame_rate, args.width, args.crf, args.preset),
                         ([0.1, 0.3], 60.0, 640, 23, "fast"))
        self.assertTrue(args.force_top_view and args.quasi_static_attitude and args.progress)
        self.assertEqual(args.output, full.output)
        png = replace(full, format="png", output=Path(self._tmp.name) / "frames")
        parsed = cli.build_parser().parse_args(png.arguments())
        self.assertEqual(parsed.png_sequence, png.output)
        self.assertIsNone(parsed.output)

    def test_dry_run_arguments_have_no_output(self):
        arguments = make_settings(self.record).arguments(dry_run=True)
        self.assertIn("--dry-run", arguments)
        self.assertNotIn("--output", arguments)

    def test_check_names_the_problem(self):
        cases = [({"width": 641}, "even"), ({"height": 0}, "even"), ({"frame_rate": 0.0}, "frame rate"),
                 ({"frame_rate": math.nan}, "frame rate"), ({"crf": 52}, "CRF"), ({"preset": "warp"}, "preset"),
                 ({"time_range": (0.3, 0.1)}, "less than"), ({"body_length": -1.0}, "body length"),
                 ({"format": "gif"}, "format"), ({"output": Path("x.avi")}, ".mp4")]
        for changes, word in cases:
            with self.subTest(changes=changes), self.assertRaisesRegex(ValueError, word):
                make_settings(self.record, **changes).check()
        with self.assertRaisesRegex(ValueError, "does not exist"):
            make_settings(Path(self._tmp.name) / "nope.npz").check()
        make_settings(self.record).check()

    def test_default_output_is_under_videos_and_has_a_safe_name(self):
        path = S.default_output(Path("/a/b/my run (2)/run_record.npz"), "mp4")
        self.assertEqual(path.parent, REPOSITORY_ROOT / "videos")
        self.assertEqual(path.name, "my_run_2_run_record.mp4")
        self.assertEqual(S.default_output(Path("/a/run_record.npz"), "png").name, "a_run_record_frames")

    def test_preview_settings_are_a_small_png_window(self):
        preview = S.preview_settings(make_settings(self.record, width=1920, height=1080), 0.2, Path(self._tmp.name) / "p")
        self.assertEqual((preview.format, preview.width, preview.height, preview.time_range), ("png", 640, 360, (0.2, 0.25)))
        preview.check()


if __name__ == "__main__":
    unittest.main()
