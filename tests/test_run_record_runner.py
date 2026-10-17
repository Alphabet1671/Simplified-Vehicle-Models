"""Check the runner's --record option: opt-in, additive, and a loadable record."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from simplified_vehicle_models.results.run_record import load_run_record
from tests.test_forward_driver_runner import FakePlanarPlant, run_fake_child
from tools.run.run_iem24_forward_driver import RUN_RECORD_FILE, child, parse_args

#: A 0.6 s segment is ~60 accepted steps: long enough to exercise the sink, short
#: enough for the suite. The default planner and the canonical track are used.
SHORT = ("--duration", "0.6", "--maximum-step", "0.01")


def real_child(model, *extra):
    root = Path(tempfile.mkdtemp())
    args = parse_args(["--output-root", str(root), "--model", model, *SHORT, *extra])
    args.child_attempt = root
    with contextlib.redirect_stdout(io.StringIO()):
        code = child(args)
    return root, code, json.loads((root / "summary.json").read_text())


class RecordOptionTest(unittest.TestCase):
    def test_flags_parse_and_default_to_off(self):
        args = parse_args([])
        self.assertFalse(args.record)
        self.assertEqual(args.record_precision, "float32")
        self.assertFalse(args.quasi_static_attitude)
        on = parse_args(["--record", "--record-precision", "float64", "--quasi-static-attitude"])
        self.assertTrue(on.record and on.quasi_static_attitude)
        self.assertEqual(on.record_precision, "float64")

    def test_quasi_static_attitude_requires_record(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(["--quasi-static-attitude"])

    def test_run_without_record_does_not_pass_a_sink_or_write_a_file(self):
        root, calls, code = run_fake_child("planar-7dof", FakePlanarPlant)
        self.assertEqual(code, 0)
        self.assertNotIn("step_sink", calls[0])
        self.assertFalse((root / RUN_RECORD_FILE).exists())
        self.assertNotIn("run_record_file", json.loads((root / "summary.json").read_text()))


class RealRecordRunTest(unittest.TestCase):
    def check_record(self, root, summary, model):
        self.assertIn(summary["status"], ("Completed", "Duration_Limit"))
        path = root / RUN_RECORD_FILE
        self.assertEqual(summary["run_record_file"], RUN_RECORD_FILE)
        # Manifest sources hash the physical inputs; they were taken from the working tree.
        record = load_run_record(path)
        trace = np.loadtxt(root / "trace.csv", delimiter=",", skiprows=1)
        self.assertEqual(summary["run_record_rows"], record.rows)
        self.assertEqual(record.rows, trace.shape[0])
        np.testing.assert_array_equal(record.time, trace[:, 0])
        np.testing.assert_allclose(record.state, trace[:, 1:], rtol=1e-6, atol=1e-6)
        header = (root / "trace.csv").read_text().splitlines()[0].split(",")
        self.assertEqual(("time",) + record.manifest.state_names, tuple(header))
        self.assertEqual(record.manifest.model["model"], model)
        self.assertEqual(record.manifest.precision, "float32")
        # --maximum-step 0.01 s is longer than the 1/120 s frame period, so the record is
        # flagged as too coarse to be called a 120 fps record.
        self.assertTrue(record.manifest.coarse_for_120_fps)
        self.assertEqual(record.manifest.integrator["maximum_step"], 0.01)
        self.assertGreater(len(record.manifest.sources), 3)
        # The pedal control column is the solver's own pedal trace.
        pedal = np.loadtxt(root / "pedal.csv", delimiter=",", skiprows=1)
        np.testing.assert_allclose(record.control_column("pedal"), pedal[:, 1], atol=1e-6)
        return record

    def test_planar_record_matches_trace_and_pedal(self):
        root, code, summary = real_child("planar-7dof", "--record")
        self.assertEqual(code, 0)
        record = self.check_record(root, summary, "planar-7dof")
        self.assertFalse(record.has_channel("roll"))
        self.assertTrue(record.has_channel("tire-vertical-force"))

    def test_vertical_record_matches_trace_and_has_attitude(self):
        root, code, summary = real_child("14dof-simple", "--record", "--timeout-seconds", "900")
        self.assertEqual(code, 0)
        record = self.check_record(root, summary, "14dof-simple")
        for name in ("roll", "pitch", "heave"):
            self.assertTrue(record.has_channel(name), name)

    def test_recording_does_not_change_the_trace(self):
        plain_root, _, _ = real_child("planar-7dof")
        record_root, _, _ = real_child("planar-7dof", "--record")
        for name in ("trace.csv", "pedal.csv"):
            self.assertEqual((plain_root / name).read_bytes(), (record_root / name).read_bytes(), name)
        self.assertFalse((plain_root / RUN_RECORD_FILE).exists())


if __name__ == "__main__":
    unittest.main()
