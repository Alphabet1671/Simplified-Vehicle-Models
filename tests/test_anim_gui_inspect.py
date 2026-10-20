import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest import mock

from tests.test_animation_cli import record_file
from tests.test_animation_panels import BASE, DYNAMIC, WHEEL
from tools.anim.encode import EncoderUnavailable
from tools.anim_gui import inspect_plan, settings as S


def make_settings(record, **changes):
    return replace(S.ExportSettings(record=Path(record), output=Path(record).with_suffix(".mp4")), **changes)


class InspectTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.record, _ = record_file(self._tmp.name, BASE + WHEEL + DYNAMIC)

    def test_inspect_matches_the_dry_run(self):
        summary = inspect_plan.inspect(make_settings(self.record))
        self.assertEqual(summary.frames, 73)
        self.assertEqual((summary.first_time, round(summary.last_time, 3)), (0.0, 0.6))
        self.assertIn("frames: 73 at 120 fps", summary.text)
        self.assertIn("visible windows:", summary.text)
        self.assertTrue(summary.visible)

    def test_range_changes_the_frame_count(self):
        summary = inspect_plan.inspect(make_settings(self.record, time_range=(0.1, 0.3)))
        self.assertEqual(summary.frames, 25)

    def test_range_outside_the_record_is_a_message(self):
        with self.assertRaisesRegex(ValueError, "outside the record"):
            inspect_plan.inspect(make_settings(self.record, time_range=(0.0, 9.0)))

    def test_a_file_that_is_not_a_record(self):
        bad = Path(self._tmp.name) / "bad.npz"
        bad.write_bytes(b"not a record")
        with self.assertRaisesRegex(ValueError, "not a valid run record"):
            inspect_plan.inspect(make_settings(bad))
        with self.assertRaisesRegex(ValueError, "not a valid run record"):
            inspect_plan.record_span(bad)

    def test_record_span(self):
        first, last = inspect_plan.record_span(self.record)
        self.assertEqual((first, round(last, 3)), (0.0, 0.6))

    def test_encoder_problem(self):
        with mock.patch.object(inspect_plan, "find_ffmpeg", side_effect=EncoderUnavailable("ffmpeg was not found on PATH")):
            self.assertIn("ffmpeg was not found", inspect_plan.encoder_problem())
        with mock.patch.object(inspect_plan, "find_ffmpeg", return_value="/usr/bin/ffmpeg"):
            self.assertIsNone(inspect_plan.encoder_problem())


if __name__ == "__main__":
    unittest.main()
