"""Focused contracts for the diagnostic yaw-aware runner."""
import contextlib
import io
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from tools.run import run_yaw_aware_forward_driver as runner


class YawAwareForwardRunnerTests(unittest.TestCase):
    def test_parser_accepts_no_tv(self):
        with tempfile.TemporaryDirectory() as directory:
            args = runner.parse_args(["--track", "track.xml", "--output-dir", directory, "--map", "no-tv"])
        self.assertEqual(args.map, "no-tv")
        self.assertEqual(args.model, "planar-7dof")
        self.assertGreater(args.control_spacing, 0.)
        self.assertGreater(args.planning_grid_spacing, 0.)

    def test_prepare_output_refuses_existing_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(FileExistsError):
                runner.prepare_output(directory)

    def test_parser_rejects_vertical_with_clear_planar_only_error(self):
        stderr = io.StringIO()
        with tempfile.TemporaryDirectory() as directory, self.assertRaises(SystemExit), contextlib.redirect_stderr(stderr):
            runner.parse_args(["--track", "track.xml", "--output-dir", directory,
                               "--model", "vertical-14dof"])
        self.assertIn("planar only", stderr.getvalue())

    def test_record_sink_uses_optimized_geometry_argument(self):
        line = object()
        plant = object()
        fake = object()
        with patch("simplified_vehicle_models.results.plant_channels.RunRecordSink",
                   return_value=fake) as sink_type:
            sink = runner.make_record_sink(plant, line, Path("record.npz"), "float64")
        self.assertIs(sink, fake)
        self.assertIs(sink_type.call_args.args[1], line)
        self.assertEqual(sink_type.call_args.kwargs["precision"], "float64")


if __name__ == "__main__":
    unittest.main()
