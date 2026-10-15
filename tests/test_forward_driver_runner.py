"""Check the hard deadline and attempt retention for forward experiments."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from tools.run_iem24_forward_driver import acceptance_gate, main, parse_args


class RunnerTests(unittest.TestCase):
    def test_saved_periodic_deltas_reject_attempt(self):
        saved = Path("docs/validation/iem24-murphy/verification_2026-10-15/planar_forward/attempt_1792047163837755000/summary.json")
        summary = json.loads(saved.read_text())
        deltas = summary["periodic_state_delta"]
        tolerances = summary["periodic_state_tolerances"]
        self.assertEqual(len(deltas), len(tolerances))
        self.assertTrue(any(abs(delta) > tolerance
                            for delta, tolerance in zip(deltas, tolerances)))
        self.assertFalse(acceptance_gate(summary))
        self.assertFalse(acceptance_gate(summary))

    def test_acceptance_gate_rejects_invalid_periodic_vectors(self):
        valid = {
            "lap_completed": True,
            "periodic_state_delta": [0.0, -0.5],
            "periodic_state_tolerances": [0.01, 0.5],
            "periodicity_valid": True,
            "integration_completed": True,
            "path_tracking_valid": True,
            "resolution_valid": True,
            "checked_rhs_physics_valid": True,
            "between_step_validation_complete": True,
            "refinement_validated": True,
            "contact_valid": True,
        }
        self.assertTrue(acceptance_gate(valid))
        for delta, tolerances in (
            ([], []), ([0.0], []), ([float("inf")], [float("inf")]),
            ([float("nan")], [1.0]), ([0.0], [float("inf")]),
            ([0.0], [-0.1]), (["0"], [1.0]), ([0.0], ["1"]),
        ):
            with self.subTest(delta=delta, tolerances=tolerances):
                candidate = dict(valid, periodic_state_delta=delta,
                                 periodic_state_tolerances=tolerances)
                self.assertFalse(acceptance_gate(candidate))

    def test_limits_reject_nonfinite_and_over_budget_values(self):
        for arguments in (["--timeout-seconds", "301"], ["--timeout-seconds", "nan"],
                          ["--duration", "0"], ["--spacing", ".51"],
                          ["--rtol", "-1"]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(arguments)

    def test_timeout_retains_parent_record_and_log(self):
        with tempfile.TemporaryDirectory() as temporary:
            with patch("tools.run_iem24_forward_driver.subprocess.run",
                       side_effect=subprocess.TimeoutExpired("child", 1.)) as run:
                with contextlib.redirect_stdout(io.StringIO()):
                    result = main(["--output-root", temporary, "--timeout-seconds", "1"])
            self.assertEqual(result, 124)
            self.assertEqual(run.call_args.kwargs["timeout"], 1.)
            attempts = list(Path(temporary).iterdir())
            self.assertEqual(len(attempts), 1)
            parent = json.loads((attempts[0] / "parent.json").read_text())
            self.assertEqual(parent["status"], "Timed_Out")
            self.assertTrue((attempts[0] / "run.log").exists())

    def test_attempts_do_not_overwrite_each_other(self):
        with tempfile.TemporaryDirectory() as temporary:
            with patch("tools.run_iem24_forward_driver.subprocess.run",
                       return_value=subprocess.CompletedProcess([], 0)):
                with contextlib.redirect_stdout(io.StringIO()):
                    main(["--output-root", temporary])
                    main(["--output-root", temporary])
            self.assertEqual(len(list(Path(temporary).iterdir())), 2)


if __name__ == "__main__":
    unittest.main()
