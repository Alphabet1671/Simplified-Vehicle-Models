import math
import tempfile
import unittest
from unittest import mock

from tests.test_animation_cli import record_file
from tests.test_animation_panels import BASE, DYNAMIC, WHEEL
from tools.anim_gui import progress


class ProgressTests(unittest.TestCase):
    def test_parses_the_exporters_line(self):
        value = progress.parse_progress("frame 30/72    41.2 frames/s  eta      1 s")
        self.assertEqual((value.done, value.total, value.frames_per_second, value.eta_seconds), (30, 72, 41.2, 1.0))
        self.assertAlmostEqual(value.fraction, 30 / 72)

    def test_nan_eta(self):
        value = progress.parse_progress("frame 1/72     0.0 frames/s  eta    nan s")
        self.assertTrue(math.isnan(value.eta_seconds))
        self.assertEqual(progress.format_eta(value.eta_seconds), "estimating…")

    def test_other_lines_are_not_progress(self):
        for line in ("record: x", "frames: 73 at 120 fps = 0.600 s of video", "wrote out.mp4", ""):
            self.assertIsNone(progress.parse_progress(line))

    def test_the_regex_matches_what_the_exporter_prints(self):
        import contextlib, io
        from tools.anim import export_animation as cli
        text = io.StringIO()
        with mock.patch("tools.anim.render.render_frames") as render, \
                mock.patch("tools.anim.panels.load_track_geometry"), mock.patch("tools.anim.render.build_figure",
                                                                                 return_value=(None, None, None)), \
                mock.patch.object(cli, "make_writer"), contextlib.redirect_stderr(text):
            render.side_effect = lambda figure, panels, source, writer, progress: progress(5, 10)
            tmp = tempfile.mkdtemp()
            record, _ = record_file(tmp, BASE + WHEEL + DYNAMIC)
            plan = cli.Plan(cli.build_parser().parse_args(["--record", str(record), "--png-sequence", tmp + "/f", "--progress"]))
            cli.run(plan)
        value = progress.parse_progress(text.getvalue())
        self.assertEqual((value.done, value.total), (5, 10))

    def test_error_line_and_result_text(self):
        self.assertEqual(progress.parse_error("error: --time-range: bad"), "--time-range: bad")
        self.assertIsNone(progress.parse_error("frame 1/2"))
        self.assertEqual(progress.describe_result(0, "done", []), "Done.")
        self.assertIn("Install ffmpeg", progress.describe_result(3, "done", ["ffmpeg was not found"]))
        self.assertIn("bad range", progress.describe_result(2, "done", ["bad range"]))
        self.assertIn("removed", progress.describe_result(-1, "cancelled", []))
        self.assertIn("exit code 9", progress.describe_result(9, "done", []))


if __name__ == "__main__":
    unittest.main()
