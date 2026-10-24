import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest import mock

from simplified_vehicle_models.solvers import minimum_time_run as supervisor


def _stall_stage(request_path, result_path, artifact_path):
    root = Path(request_path).parent
    request = json.loads(Path(request_path).read_text())
    (root / "stage.json").write_text(json.dumps({"stage": request["fixture_stage"]}))
    (root / "partial.bin").write_bytes(b"partial")
    time.sleep(10)


def _success_late(request_path, result_path, artifact_path):
    Path(result_path).write_text(json.dumps({"status": "success", "accepted": True, "solver_succeeded": True}))
    time.sleep(10)


def _raise_before_solution(request_path, result_path, artifact_path):
    raise RuntimeError("fixture failure")


def _no_result(request_path, result_path, artifact_path):
    return


def _malformed_result(request_path, result_path, artifact_path):
    Path(result_path).write_text("[")


def _success_then_exit(request_path, result_path, artifact_path):
    import os
    Path(result_path).write_text(json.dumps({"status": "success", "accepted": True,
                                              "solver_succeeded": True}))
    os._exit(7)


class MinimumTimeRunTests(unittest.TestCase):
    def test_rejects_nonfinite_budget_and_json_values(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(ValueError):
                supervisor.run_request({"wall_cap_s": float("nan")}, f"{d}/x")
            with self.assertRaises(ValueError):
                supervisor.run_request({"wall_cap_s": 1, "x": float("inf")}, f"{d}/y")

    def test_timeout_authoritative_partial_inventory_by_stage(self):
        for stage in ("setup", "solve", "validate", "export"):
            with self.subTest(stage=stage), tempfile.TemporaryDirectory() as d:
                with mock.patch.object(supervisor, "_worker", _stall_stage):
                    result = supervisor.run_request({"wall_cap_s": 0.6, "fixture_stage": stage}, f"{d}/attempt")
                self.assertEqual(result["status"], "wall_timeout")
                self.assertEqual(result["stage"], stage)
                self.assertFalse(result["accepted"])
                self.assertEqual(result["solve_completed"], stage in ("validate", "export"))
                manifest = json.loads((Path(d) / "attempt" / "artifacts.json").read_text())
                self.assertTrue(any(e["path"] == "partial.bin" and e["sha256"] for e in manifest["artifacts"]))
                self.assertTrue((Path(d) / "attempt" / "timeout_result.json").exists())

    def test_cancel_marker_and_exception_remain_unaccepted(self):
        with tempfile.TemporaryDirectory() as d:
            cancel = Path(d) / "cancel"
            cancel.touch()
            with mock.patch.object(supervisor, "_worker", _stall_stage):
                result = supervisor.run_request({"wall_cap_s": 4, "cancel_marker_path": str(cancel),
                                                  "fixture_stage": "solve"}, f"{d}/cancelled")
            self.assertEqual(result["status"], "cancelled")
            self.assertFalse(result["accepted"])
        with tempfile.TemporaryDirectory() as d:
            with mock.patch.object(supervisor, "_worker", _raise_before_solution):
                result = supervisor.run_request({"wall_cap_s": 3}, f"{d}/exception")
            self.assertEqual(result["status"], "worker_no_result")
            self.assertFalse(result["accepted"])
            self.assertFalse(result["solve_completed"])

    def test_nonzero_exit_does_not_inherit_child_success(self):
        with tempfile.TemporaryDirectory() as d:
            with mock.patch.object(supervisor, "_worker", _success_then_exit):
                result = supervisor.run_request({"wall_cap_s": 3}, f"{d}/exit")
            self.assertEqual(result["status"], "worker_exit_failure")
            self.assertFalse(result["accepted"])

    def test_late_child_success_cannot_change_timeout_verdict(self):
        with tempfile.TemporaryDirectory() as d:
            with mock.patch.object(supervisor, "_worker", _success_late):
                result = supervisor.run_request({"wall_cap_s": 0.8}, f"{d}/late")
            self.assertEqual(result["status"], "wall_timeout")
            self.assertFalse(result["accepted"])
            child_result = json.loads((Path(d) / "late" / "result.json").read_text())
            self.assertTrue(child_result["accepted"])
            self.assertTrue((Path(d) / "late" / "timeout_result.json").exists())
            self.assertFalse(json.loads((Path(d) / "late" / "supervisor_result.json").read_text())["accepted"])

    def test_child_no_result_and_malformed_result_are_fail_closed(self):
        with tempfile.TemporaryDirectory() as d:
            with mock.patch.object(supervisor, "_worker", _no_result):
                result = supervisor.run_request({"wall_cap_s": 3}, f"{d}/none")
            self.assertEqual(result["status"], "worker_no_result")
            self.assertFalse(result["accepted"])
        with tempfile.TemporaryDirectory() as d:
            with mock.patch.object(supervisor, "_worker", _malformed_result):
                result = supervisor.run_request({"wall_cap_s": 3}, f"{d}/bad")
            self.assertEqual(result["status"], "worker_malformed_result")
            self.assertFalse(result["accepted"])
            self.assertTrue(any(a["path"] == "result.json" for a in result["artifacts"]))

    def test_legacy_harness_delegates_to_shared_supervisor(self):
        from tools.diagnostics import verify_no_tv_minimum_lap as harness
        with tempfile.TemporaryDirectory() as d:
            with mock.patch.object(supervisor, "run_request", return_value={"accepted": False}) as shared:
                result = harness.run_bounded_worker({}, Path(d) / "unused")
            shared.assert_called_once()
            self.assertFalse(result["accepted"])


if __name__ == "__main__":
    unittest.main()
