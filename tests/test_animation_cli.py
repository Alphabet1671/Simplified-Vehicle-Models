"""Check the export CLI: dry runs, argument rejection, and a small real render."""
import contextlib
import io
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from tests.test_animation_panels import BASE, DYNAMIC, GEOMETRY, QUASI, WHEEL, make_source
from tools.anim import export_animation as cli

ROOT = Path(__file__).resolve().parents[1]
HAVE_FFMPEG = shutil.which("ffmpeg") is not None and shutil.which("ffprobe") is not None


def record_file(directory, channels, name="rec"):
    """Write a synthetic record and return its path (0.6 s, 61 rows)."""
    source = make_source(directory, channels, model=name)
    return Path(directory) / f"{name}.npz", source


def run_cli(*argv):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        try:
            code = cli.main([str(a) for a in argv])
        except SystemExit as exit_:       # argparse errors
            code = exit_.code
    return code, out.getvalue(), err.getvalue()


class DryRunTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    def dry(self, channels, *flags, name="rec"):
        path, _ = record_file(self.root, channels, name)
        return run_cli("--record", path, "--dry-run", *flags)

    def test_frame_count_follows_the_grid_rule_for_the_full_range(self):
        code, out, _ = self.dry(BASE + WHEEL + DYNAMIC)
        self.assertEqual(code, 0)
        self.assertIn(f"frames: {int(np.floor(0.6 * 120 + 1e-9)) + 1} at 120 fps", out)

    def test_time_range_gives_the_grid_count(self):
        path, _ = record_file(self.root, BASE)
        code, out, _ = run_cli("--record", path, "--time-range", "0.1", "0.3", "--dry-run")
        self.assertEqual(code, 0)
        self.assertIn("frames: 25 at 120 fps", out)       # floor(0.2*120)+1

    def test_ten_to_twelve_seconds_is_241_frames(self):
        # a 14-DoF-length record is not needed for the rule; check the arithmetic directly
        from simplified_vehicle_models.results.run_record import frame_times
        self.assertEqual(frame_times(10.0, 12.0, 120.0).size, 241)

    def test_time_range_outside_the_record_is_an_error(self):
        path, _ = record_file(self.root, BASE)
        code, _, err = run_cli("--record", path, "--time-range", "0.1", "5", "--dry-run")
        self.assertEqual(code, 2)
        self.assertIn("--time-range", err)
        code, _, err = run_cli("--record", path, "--time-range", "0.3", "0.1", "--dry-run")
        self.assertEqual(code, 2)

    def test_frame_rate_zero_is_rejected_naming_the_argument(self):
        path, _ = record_file(self.root, BASE)
        for bad in ("0", "-5", "nan", "inf"):
            code, _, err = run_cli("--record", path, "--frame-rate", bad, "--dry-run")
            self.assertNotEqual(code, 0, bad)
            self.assertIn("--frame-rate", err, bad)

    def test_a_missing_record_exits_nonzero_with_a_clear_message(self):
        code, out, err = run_cli("--record", self.root / "absent.npz", "--dry-run")
        self.assertEqual(code, 2)
        self.assertIn("--record: no such file", err)
        self.assertEqual(out, "")

    def test_a_corrupt_record_exits_nonzero(self):
        bad = self.root / "bad.npz"
        bad.write_bytes(b"not a zip")
        code, _, err = run_cli("--record", bad, "--dry-run")
        self.assertEqual(code, 2)
        self.assertIn("not a valid run record", err)

    def test_planar_attitude_warning_is_printed_exactly_once(self):
        code, out, _ = self.dry(BASE + WHEEL + QUASI)
        self.assertEqual(code, 0)
        self.assertEqual(out.count("roll/pitch: not available"), 1)
        self.assertIn("hidden windows: rear, right", out)
        self.assertIn("visible windows: track, top", out)

    def test_quasi_static_flag_shows_both_windows_labelled_approximate(self):
        code, out, _ = self.dry(BASE + WHEEL + QUASI, "--quasi-static-attitude")
        self.assertEqual(code, 0)
        self.assertIn("visible windows: track, rear, right, top", out)
        self.assertIn("hidden windows: none", out)
        self.assertIn("APPROXIMATE", out)
        self.assertEqual(out.count("roll/pitch: not available"), 1)

    def test_point_mass_hides_three_windows_and_force_top_view_restores_one(self):
        code, out, _ = self.dry(BASE)
        self.assertIn("visible windows: track\n", out)
        self.assertIn("hidden windows: rear, right, top", out)
        self.assertIn("top hidden", out)
        code, out, _ = self.dry(BASE, "--force-top-view")
        self.assertIn("visible windows: track, top\n", out)
        self.assertIn("hidden windows: rear, right", out)

    def test_quasi_static_flag_changes_nothing_on_a_point_mass_record(self):
        _, plain, _ = self.dry(BASE)
        _, flagged, _ = self.dry(BASE, "--quasi-static-attitude")
        self.assertEqual(plain, flagged)

    def test_dry_run_writes_nothing(self):
        path, _ = record_file(self.root, BASE)
        target = self.root / "out.mp4"
        pngs = self.root / "frames"
        code, out, _ = run_cli("--record", path, "--output", target, "--dry-run")
        code2, _, _ = run_cli("--record", path, "--png-sequence", pngs, "--dry-run")
        self.assertEqual((code, code2), (0, 0))
        self.assertFalse(target.exists())
        self.assertFalse(pngs.exists())
        self.assertIn("nothing was rendered", out)

    def test_summary_lists_channels_estimate_and_layout(self):
        _, out, _ = self.dry(BASE + WHEEL + DYNAMIC)
        self.assertIn("channels (", out)
        self.assertIn("estimated output: ~", out)
        self.assertIn("not measured", out)
        self.assertIn("layout: 2x3", out)


class ArgumentTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.path, _ = record_file(self.root, BASE)

    def test_an_output_is_required_unless_dry_run(self):
        code, _, err = run_cli("--record", self.path)
        self.assertNotEqual(code, 0)
        self.assertIn("--output", err)

    def test_output_and_png_sequence_are_exclusive(self):
        code, _, err = run_cli("--record", self.path, "--output", self.root / "a.mp4",
                               "--png-sequence", self.root / "d")
        self.assertNotEqual(code, 0)
        self.assertIn("exclusive", err)

    def test_odd_dimensions_are_rejected(self):
        code, _, err = run_cli("--record", self.path, "--width", "321", "--dry-run")
        self.assertNotEqual(code, 0)
        self.assertIn("--width", err)

    def test_nonpositive_body_size_is_rejected(self):
        code, _, err = run_cli("--record", self.path, "--body-length", "0", "--dry-run")
        self.assertNotEqual(code, 0)
        self.assertIn("--body-length", err)

    def test_a_missing_stl_is_an_error_not_a_traceback(self):
        code, _, err = run_cli("--record", self.path, "--stl-body", self.root / "no.stl",
                               "--dry-run")
        self.assertEqual(code, 2)
        self.assertIn("cannot read the STL file", err)

    def test_stl_transform_is_printed_and_changes_the_scene_only(self):
        from tools.anim.stl import box_triangles
        stl = self.root / "body.stl"
        triangles = box_triangles((2000.0, 1000.0, 300.0))          # millimetres
        with open(stl, "w") as handle:
            handle.write("solid t\n")
            for tri in triangles:
                handle.write("facet normal 0 0 0\nouter loop\n")
                for p in tri:
                    handle.write(f"vertex {p[0]} {p[1]} {p[2]}\n")
                handle.write("endloop\nendfacet\n")
            handle.write("endsolid t\n")
        args = cli.build_parser().parse_args(
            ["--record", str(self.path), "--stl-body", str(stl), "--transform-scale", "0.001",
             "--transform-x", "0.1", "--dry-run"])
        plan = cli.Plan(args)
        points = plan.scene.triangles.reshape(-1, 3)
        self.assertAlmostEqual(float(np.ptp(points[:, 0])), 2.0)
        self.assertAlmostEqual(float(points[:, 0].mean()), 0.1, places=9)
        self.assertTrue(any("scale 0.001" in note for note in plan.scene_notes))
        self.assertEqual(len(plan.scene.wheels), 4)


class RenderTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    def test_a_record_without_a_track_file_fails_before_anything_is_written(self):
        path, _ = record_file(self.root, BASE + WHEEL + DYNAMIC)
        frames = self.root / "frames"
        code, out, err = run_cli("--record", path, "--png-sequence", frames, "--width", "320",
                                 "--height", "240", "--time-range", "0.0", "0.05")
        self.assertEqual(code, 2, err)
        self.assertIn("track", err)
        self.assertFalse(frames.exists())

    def test_png_sequence_of_a_real_excerpt(self):
        from tests.test_run_record_runner import real_child
        root, status, _ = real_child("14dof-simple", "--record")
        self.assertEqual(status, 0)
        frames = self.root / "frames"
        code, out, err = run_cli("--record", root / "run_record.npz", "--png-sequence", frames,
                                 "--width", "320", "--height", "240", "--time-range", "0.1",
                                 "0.15")
        self.assertEqual(code, 0, err)
        self.assertEqual(len(list(frames.glob("frame_*.png"))), 7)       # floor(0.05*120)+1

    @unittest.skipUnless(HAVE_FFMPEG, "ffmpeg and ffprobe are required")
    def test_real_record_renders_to_120_fps_h264(self):
        from tests.test_run_record_runner import real_child
        root, status, _ = real_child("14dof-simple", "--record")
        self.assertEqual(status, 0)
        record = root / "run_record.npz"
        video = self.root / "clip.mp4"
        code, out, err = run_cli("--record", record, "--output", video, "--width", "480",
                                 "--height", "270", "--time-range", "0.1", "0.3")
        self.assertEqual(code, 0, err)
        self.assertIn("render/real = ", out)
        probe = subprocess.run(
            ["ffprobe", "-v", "error", "-count_frames", "-select_streams", "v:0",
             "-show_entries", "stream=r_frame_rate,nb_read_frames,pix_fmt,codec_name",
             "-of", "json", str(video)], capture_output=True, text=True, check=True).stdout
        stream = json.loads(probe)["streams"][0]
        self.assertEqual(stream["r_frame_rate"], "120/1")
        self.assertEqual(stream["nb_read_frames"], "25")
        self.assertEqual((stream["codec_name"], stream["pix_fmt"]), ("h264", "yuv420p"))

    def test_missing_ffmpeg_exits_with_a_message_that_names_it(self):
        from unittest import mock
        from tests.test_run_record_runner import real_child
        root, status, _ = real_child("14dof-simple", "--record")
        self.assertEqual(status, 0)
        with mock.patch("tools.anim.encode.shutil.which", return_value=None):
            code, _, err = run_cli("--record", root / "run_record.npz", "--output",
                                   self.root / "x.mp4", "--time-range", "0.1", "0.2")
        self.assertEqual(code, 3)
        self.assertIn("ffmpeg", err)
        self.assertFalse((self.root / "x.mp4").exists())

    def test_script_runs_from_the_command_line(self):
        path, _ = record_file(self.root, BASE)
        done = subprocess.run([sys.executable, str(ROOT / "tools/anim/export_animation.py"),
                               "--record", str(path), "--dry-run"], capture_output=True,
                              text=True, cwd=self.root)
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertIn("frames: ", done.stdout)


if __name__ == "__main__":
    unittest.main()
