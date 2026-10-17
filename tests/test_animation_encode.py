"""Check the renderer, the figure layout per record, and the frame writers."""
import json
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from tests.test_animation_panels import (BASE, DYNAMIC, GEOMETRY, QUASI, WHEEL, make_source,
                                         track_geometry)
from tools.anim.encode import (EncoderUnavailable, FfmpegFrameWriter, NullFrameWriter,
                               PngSequenceWriter, ffmpeg_command)
from tools.anim.render import build_figure, footer_lines, frame_rgb, render_frames
from tools.anim.scene import placeholder_scene
from tools.anim.style import AnimationStyle

HAVE_FFMPEG = shutil.which("ffmpeg") is not None and shutil.which("ffprobe") is not None
SMALL = AnimationStyle(width=320, height=240, dpi=40)


def figure_for(directory, channels, **options):
    quasi = options.get("allow_quasi_static", False)
    source = make_source(directory, channels, quasi=quasi)
    figure, panels, visibility = build_figure(
        source.record, source, placeholder_scene(GEOMETRY), track_geometry(), SMALL, **options)
    return source, figure, panels, visibility


class BuildFigureTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def boxes(self, figure):
        return [axes.get_position().bounds for axes in figure.axes]

    def test_full_layout_has_five_axes_and_the_track_spans_the_height(self):
        source, figure, panels, visibility = figure_for(self.directory.name,
                                                        BASE + WHEEL + DYNAMIC)
        self.assertEqual(len(figure.axes), 5)
        self.assertEqual(tuple(panels), ("gg", "track", "rear", "right", "top"))
        self.assertLess(panels["gg"].axes.get_position().x1,
                        panels["track"].axes.get_position().x0)
        track = panels["track"].axes.get_position()
        top = panels["top"].axes.get_position()
        self.assertAlmostEqual(track.y1 - track.y0, (top.y1 - panels["rear"].axes.get_position().y0),
                               places=9)

    def test_without_attitude_the_top_axes_spans_the_track_height(self):
        _, figure, panels, _ = figure_for(self.directory.name, BASE + WHEEL + QUASI)
        self.assertEqual(len(figure.axes), 3)
        track, top = panels["track"].axes.get_position(), panels["top"].axes.get_position()
        self.assertAlmostEqual(top.y0, track.y0, places=9)
        self.assertAlmostEqual(top.y1, track.y1, places=9)

    def test_without_wheel_data_the_attitude_axes_span_the_full_height_side_by_side(self):
        _, figure, panels, _ = figure_for(self.directory.name, BASE + DYNAMIC)
        self.assertEqual(tuple(panels), ("gg", "track", "rear", "right"))
        track = panels["track"].axes.get_position()
        for name in ("rear", "right"):
            box = panels[name].axes.get_position()
            self.assertAlmostEqual(box.y0, track.y0, places=9)
            self.assertAlmostEqual(box.y1, track.y1, places=9)
        self.assertLess(panels["rear"].axes.get_position().x1,
                        panels["right"].axes.get_position().x0)

    def test_a_point_mass_record_creates_the_gg_and_track_axes_only(self):
        _, figure, panels, visibility = figure_for(self.directory.name, BASE)
        self.assertEqual(len(figure.axes), 2)
        self.assertEqual(tuple(panels), ("gg", "track"))
        self.assertEqual(visibility.hidden, ("rear", "right", "top"))

    def test_force_top_view_adds_only_the_top_axes(self):
        _, figure, panels, visibility = figure_for(self.directory.name, BASE,
                                                   force_top_view=True)
        self.assertEqual(tuple(panels), ("gg", "track", "top"))
        self.assertEqual(len(figure.axes), 3)

    def test_quasi_static_opt_in_adds_both_attitude_axes(self):
        _, figure, panels, visibility = figure_for(self.directory.name, BASE + WHEEL + QUASI,
                                                   allow_quasi_static=True)
        self.assertEqual(len(figure.axes), 5)
        self.assertTrue(visibility.attitude_is_approximate)

    def test_the_footer_names_every_hidden_window_with_its_reason(self):
        source, figure, _, visibility = figure_for(self.directory.name, BASE + WHEEL + QUASI)
        text = "\n".join(figure.footer_lines)
        self.assertIn("rear/right hidden", text)
        self.assertIn("--quasi-static-attitude", text)
        self.assertEqual(figure.footer_lines, footer_lines(visibility, source.warnings))
        printed = [t.get_text() for t in figure.texts]
        self.assertTrue(any("rear/right hidden" in line for line in printed))

    def test_a_warning_names_the_missing_channels_the_hidden_window_and_the_model(self):
        _, figure, _, _ = figure_for(self.directory.name, BASE + WHEEL + QUASI)
        text = "\n".join(figure.footer_lines)
        self.assertIn("rear/right", text)       # the hidden window
        self.assertIn("roll", text)             # the missing channel
        self.assertIn("synthetic", text)        # the model


class RenderFramesTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def test_ten_frames_reach_the_writer_as_uint8_height_width_3(self):
        source, figure, panels, _ = figure_for(self.directory.name, BASE + WHEEL + DYNAMIC)
        writer = render_frames(figure, panels, source, NullFrameWriter(320, 240),
                               indices=range(10))
        self.assertEqual(writer.frames, 10)
        self.assertEqual(writer.shapes, {(240, 320, 3)})
        self.assertEqual(writer.dtypes, {"uint8"})
        self.assertTrue(writer.closed)

    def test_the_frame_count_is_the_grid_count_by_default(self):
        source, figure, panels, _ = figure_for(self.directory.name, BASE)
        writer = render_frames(figure, panels, source, NullFrameWriter())
        self.assertEqual(writer.frames, source.count)

    def test_rendering_twice_gives_identical_bytes(self):
        digests = []
        for _ in range(2):
            source, figure, panels, _ = figure_for(self.directory.name,
                                                   BASE + WHEEL + DYNAMIC)
            digests.append(render_frames(figure, panels, source, NullFrameWriter(),
                                         indices=range(0, source.count, 9)).digest)
        self.assertEqual(digests[0], digests[1])

    def test_different_frames_differ(self):
        source, figure, panels, _ = figure_for(self.directory.name, BASE + WHEEL + DYNAMIC)
        for panel in panels.values():
            panel.update(0)
        first = frame_rgb(figure).copy()
        for panel in panels.values():
            panel.update(source.count - 1)
        self.assertFalse(np.array_equal(first, frame_rgb(figure)))

    def test_progress_is_called_at_the_end_and_throttled(self):
        source, figure, panels, _ = figure_for(self.directory.name, BASE)
        calls = []
        render_frames(figure, panels, source, NullFrameWriter(),
                      progress=lambda done, total: calls.append((done, total)),
                      progress_interval=3600.0)
        self.assertEqual(calls, [(source.count, source.count)])

    def test_a_failing_panel_closes_the_writer_and_propagates(self):
        source, figure, panels, _ = figure_for(self.directory.name, BASE)
        writer = NullFrameWriter()
        with mock.patch.object(panels["track"], "update", side_effect=RuntimeError("boom")):
            with self.assertRaises(RuntimeError):
                render_frames(figure, panels, source, writer)
        self.assertTrue(writer.closed)
        self.assertEqual(writer.frames, 0)


class WriterTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    @staticmethod
    def frame(value, width=320, height=240):
        return np.full((height, width, 3), value, dtype=np.uint8)

    def test_null_writer_counts_and_hashes_and_rejects_wrong_shapes(self):
        writer = NullFrameWriter(320, 240)
        writer.write(self.frame(1))
        writer.write(self.frame(2))
        self.assertEqual(writer.frames, 2)
        other = NullFrameWriter(320, 240)
        other.write(self.frame(1))
        other.write(self.frame(2))
        self.assertEqual(writer.digest, other.digest)
        with self.assertRaises(ValueError):
            writer.write(np.zeros((240, 320, 3), dtype=np.float32))
        with self.assertRaises(ValueError):
            writer.write(np.zeros((10, 10, 3), dtype=np.uint8))

    def test_png_sequence_writes_numbered_files(self):
        writer = PngSequenceWriter(self.root / "png", 320, 240)
        for value in (10, 20, 30):
            writer.write(self.frame(value))
        writer.close()
        self.assertEqual(sorted(p.name for p in (self.root / "png").iterdir()),
                         ["frame_000000.png", "frame_000001.png", "frame_000002.png"])

    def test_ffmpeg_command_is_the_documented_pipe(self):
        command = ffmpeg_command("ffmpeg", "out.mp4", 320, 240, 120.0, 18, "medium")
        text = " ".join(command)
        for fragment in ("-f rawvideo", "-pix_fmt rgb24", "-s 320x240", "-r 120", "-i -",
                         "-c:v libx264", "-preset medium", "-crf 18", "-pix_fmt yuv420p",
                         "-movflags +faststart"):
            self.assertIn(fragment, text)

    def test_missing_ffmpeg_raises_a_message_that_names_it(self):
        with mock.patch("tools.anim.encode.shutil.which", return_value=None):
            with self.assertRaises(EncoderUnavailable) as caught:
                FfmpegFrameWriter(self.root / "x.mp4", 320, 240)
        self.assertIn("ffmpeg", str(caught.exception))
        self.assertFalse((self.root / "x.mp4").exists())

    def test_odd_dimensions_are_refused_before_ffmpeg_starts(self):
        with self.assertRaises(ValueError):
            FfmpegFrameWriter(self.root / "x.mp4", 321, 240)

    @unittest.skipUnless(HAVE_FFMPEG, "ffmpeg and ffprobe are required")
    def test_real_encode_of_24_frames_reads_back_as_120_fps_h264(self):
        path = self.root / "clip.mp4"
        writer = FfmpegFrameWriter(path, 320, 240, 120.0)
        for k in range(24):
            writer.write(self.frame(10 * k))
        writer.close()
        self.assertTrue(path.exists())
        self.assertFalse(list(self.root.glob("*.partial*")))
        out = subprocess.run(
            ["ffprobe", "-v", "error", "-count_frames", "-select_streams", "v:0",
             "-show_entries", "stream=r_frame_rate,nb_read_frames,codec_name,pix_fmt",
             "-of", "json", str(path)], capture_output=True, text=True, check=True).stdout
        stream = json.loads(out)["streams"][0]
        self.assertEqual(stream["r_frame_rate"], "120/1")
        self.assertEqual(stream["nb_read_frames"], "24")
        self.assertEqual(stream["codec_name"], "h264")
        self.assertEqual(stream["pix_fmt"], "yuv420p")

    @unittest.skipUnless(HAVE_FFMPEG, "ffmpeg is required")
    def test_a_failed_encode_leaves_no_file_and_reports_the_stderr_tail(self):
        path = self.root / "bad.mp4"
        writer = FfmpegFrameWriter(path, 320, 240, 120.0, preset="not-a-preset")
        with self.assertRaises(EncoderUnavailable) as caught:
            for _ in range(5):
                writer.write(self.frame(0))
            writer.close()
        self.assertIn("ffmpeg", str(caught.exception))
        self.assertFalse(path.exists())
        self.assertFalse(list(self.root.glob("*.partial*")))

    @unittest.skipUnless(HAVE_FFMPEG, "ffmpeg and ffprobe are required")
    def test_rendered_frames_encode_end_to_end(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        source, figure, panels, _ = figure_for(directory.name, BASE + WHEEL + DYNAMIC)
        path = self.root / "render.mp4"
        render_frames(figure, panels, source, FfmpegFrameWriter(path, 320, 240, 120.0),
                      indices=range(24))
        out = subprocess.run(["ffprobe", "-v", "error", "-show_entries",
                              "stream=nb_frames,r_frame_rate", "-of", "json", str(path)],
                             capture_output=True, text=True, check=True).stdout
        stream = json.loads(out)["streams"][0]
        self.assertEqual((stream["r_frame_rate"], stream["nb_frames"]), ("120/1", "24"))


if __name__ == "__main__":
    unittest.main()
