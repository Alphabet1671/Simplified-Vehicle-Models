import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest import mock

from simplified_vehicle_models.solvers import minimum_time_run as supervisor


class _FakeProcess:
    def __init__(self, *, start_error=None, join_error=None, alive=True):
        self.start_error = start_error
        self.join_error = join_error
        self.alive = alive
        self.started = False
        self.terminated = False
        self.killed = False
        self.exitcode = None
        self.exit_on_start = True
    def start(self):
        self.started = True
        if self.start_error:
            raise self.start_error
        if self.exit_on_start:
            self.alive = False
            self.exitcode = 0
    def is_alive(self): return self.alive
    def join(self, timeout=None):
        if self.join_error:
            raise self.join_error
        if timeout == 0.25 and self.terminated:
            return
        if self.timeout_exit:
            self.alive = False
            self.exitcode = 0
    def terminate(self): self.terminated = True
    def kill(self): self.killed = True; self.alive = False
    timeout_exit = True


def _stall_stage(request_path, result_path, artifact_path):
    root = Path(request_path).parent
    request = json.loads(Path(request_path).read_text())
    (root / "stage.json").write_text(json.dumps({"stage": request["fixture_stage"],
                                                   "solve_completed": request.get("fixture_solved", False),
                                                   "events": []}))
    (root / "partial.bin").write_bytes(b"partial")
    time.sleep(10)


def _success_late(request_path, result_path, artifact_path):
    Path(result_path).write_text(json.dumps({"status": "success", "accepted": True, "solver_succeeded": True}))
    time.sleep(10)


def _raise_before_solution(request_path, result_path, artifact_path):
    Path(result_path).write_text(json.dumps({"status": "worker_exception", "accepted": False,
                                              "error_type": "RuntimeError", "error": "fixture failure"}))


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
    def test_nonobject_request_rejected_before_output_creation(self):
        with tempfile.TemporaryDirectory() as d:
            for value in (None, [], "request"):
                with self.subTest(value=value), self.assertRaises(TypeError):
                    supervisor.run_request(value, f"{d}/attempt")
            self.assertEqual(list(Path(d).iterdir()), [])

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
                    result = supervisor.run_request({"wall_cap_s": 0.6, "fixture_stage": stage,
                                                      "fixture_solved": stage in ("validate", "export")}, f"{d}/attempt")
                self.assertEqual(result["status"], "wall_timeout")
                self.assertEqual(result["stage"], stage)
                self.assertFalse(result["accepted"])
                self.assertEqual(result["solve_completed"], stage in ("validate", "export"))
                manifest = json.loads((Path(d) / "attempt" / "artifacts.json").read_text())
                part = next(e for e in manifest["artifacts"] if e["path"] == "partial.bin")
                self.assertEqual(part["status"], "unknown_or_partial")
                self.assertTrue(part["sha256"])
                self.assertTrue((Path(d) / "attempt" / "timeout_result.json").exists())

    def test_timeout_after_solve_failure_stage_retains_completion(self):
        with tempfile.TemporaryDirectory() as d:
            with mock.patch.object(supervisor, "_worker", _stall_stage):
                result = supervisor.run_request({"wall_cap_s": 0.6, "fixture_stage": "failed",
                                                  "fixture_solved": True}, f"{d}/failed-after-solve")
            self.assertEqual(result["stage"], "failed")
            self.assertTrue(result["solve_completed"])
            self.assertFalse(result["accepted"])

    def test_deadline_before_spawn_persists_classified_attempt(self):
        with tempfile.TemporaryDirectory() as d:
            clock = iter((10.0, 12.0))
            fake_process = _FakeProcess(alive=False)
            with mock.patch.object(supervisor.time, "monotonic", side_effect=lambda: next(clock)), \
                 mock.patch.object(supervisor.mp, "get_context") as get_context:
                get_context.return_value.Process.return_value = fake_process
                result = supervisor.run_request({"wall_cap_s": 1}, f"{d}/early")
            self.assertFalse(fake_process.started)
            self.assertEqual(result["status"], "wall_timeout")
            self.assertFalse(result["accepted"])
            attempt = Path(d) / "early"
            self.assertTrue((attempt / "supervisor_result.json").exists())
            self.assertTrue((attempt / "artifacts.json").exists())

    def test_cleanup_escalates_to_kill_and_reaps(self):
        process = _FakeProcess(alive=True)
        process.timeout_exit = False
        status, errors = supervisor._cleanup_process(process)
        self.assertEqual(status, "killed")
        self.assertTrue(process.terminated)
        self.assertTrue(process.killed)
        self.assertEqual(errors, [])

    def test_start_failure_is_classified_and_attempted_child_cleaned(self):
        with tempfile.TemporaryDirectory() as d:
            process = _FakeProcess(start_error=RuntimeError("start failed"), alive=True)
            with mock.patch.object(supervisor.mp, "get_context") as context:
                context.return_value.Process.return_value = process
                result = supervisor.run_request({"wall_cap_s": 3}, f"{d}/start")
            self.assertEqual(result["status"], "worker_start_failure")
            self.assertTrue(process.terminated)
            self.assertTrue(process.killed)
            self.assertTrue(result["cleanup_errors"])
            self.assertFalse(result["accepted"])

    def test_join_failure_is_classified_and_reaped(self):
        with tempfile.TemporaryDirectory() as d:
            process = _FakeProcess(join_error=RuntimeError("join failed"), alive=True)
            process.exit_on_start = False
            with mock.patch.object(supervisor.mp, "get_context") as context:
                context.return_value.Process.return_value = process
                result = supervisor.run_request({"wall_cap_s": 3}, f"{d}/join")
            self.assertEqual(result["status"], "supervisor_error")
            self.assertTrue(process.killed)
            self.assertTrue(result["cleanup_errors"])
            self.assertFalse(result["accepted"])

    def test_worker_exit_after_deadline_is_timeout_not_success(self):
        with tempfile.TemporaryDirectory() as d:
            clock = iter((10.0, 10.0, 10.9, 12.0))
            process = _FakeProcess(alive=True)
            process.exit_on_start = False
            with mock.patch.object(supervisor.time, "monotonic", side_effect=lambda: next(clock)), \
                 mock.patch.object(supervisor.mp, "get_context") as context:
                context.return_value.Process.return_value = process
                result = supervisor.run_request({"wall_cap_s": 1}, f"{d}/post-deadline")
            self.assertEqual(result["status"], "wall_timeout")
            self.assertFalse(result["accepted"])
            self.assertFalse(process.alive)

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
            self.assertEqual(result["status"], "worker_exception")
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
            self.assertEqual(result["raw_result"], "[")

    def test_legacy_harness_delegates_to_shared_supervisor(self):
        from tools.diagnostics import verify_no_tv_minimum_lap as harness
        with tempfile.TemporaryDirectory() as d:
            with mock.patch.object(supervisor, "run_request", return_value={"accepted": False}) as shared:
                result = harness.run_bounded_worker({}, Path(d) / "unused")
            shared.assert_called_once()
            self.assertFalse(result["accepted"])


if __name__ == "__main__":
    unittest.main()
