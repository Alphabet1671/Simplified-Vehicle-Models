import tempfile
import unittest
from types import SimpleNamespace

import numpy as np
from dataclasses import replace
from pathlib import Path
from unittest import mock

from tests.test_animation_cli import record_file
from tests.test_animation_panels import BASE, DYNAMIC, WHEEL
from tools.anim import export_animation as cli
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
        settings = make_settings(self.record)
        summary = inspect_plan.inspect(settings)
        args = inspect_plan.namespace(settings)
        plan = cli.Plan(args)
        self.assertEqual(summary.text, cli.describe(plan))
        self.assertEqual(summary.frames, 73)
        self.assertEqual((summary.first_time, round(summary.last_time, 3)), (0.0, 0.6))
        self.assertEqual(summary.visible, tuple(plan.visibility.visible))
        self.assertEqual(summary.hidden, tuple(plan.visibility.hidden))
        self.assertEqual(summary.warnings, tuple(plan.warning_lines()))
        self.assertEqual(summary.model, plan.source.model_name)
        self.assertTrue(summary.has_wheel_data)
        self.assertIn("frames: 73 at 120 fps", summary.text)
        self.assertIn("visible windows:", summary.text)

    def test_force_top_view_does_not_create_wheel_data(self):
        record, _ = record_file(self._tmp.name, BASE, name="no_wheel")
        settings = make_settings(record, force_top_view=True)
        with mock.patch.object(cli, "run", side_effect=AssertionError("render must not run")):
            summary = inspect_plan.inspect(settings)
        self.assertTrue(summary.visible)
        self.assertIn("top", summary.visible)
        self.assertFalse(summary.has_wheel_data)

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

    def test_record_span_rejects_empty_record(self):
        with mock.patch("simplified_vehicle_models.results.run_record.load_run_record",
                         return_value=SimpleNamespace(time=np.array([]))):
            with self.assertRaisesRegex(ValueError, "no time samples"):
                inspect_plan.record_span(Path("empty.npz"))

    def test_encoder_problem(self):
        with mock.patch.object(inspect_plan, "find_ffmpeg", side_effect=EncoderUnavailable("ffmpeg was not found on PATH")):
            self.assertIn("ffmpeg was not found", inspect_plan.encoder_problem())
        with mock.patch.object(inspect_plan, "find_ffmpeg", return_value="/usr/bin/ffmpeg"):
            self.assertIsNone(inspect_plan.encoder_problem())


if __name__ == "__main__":
    unittest.main()
