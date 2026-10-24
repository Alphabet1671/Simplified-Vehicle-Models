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
    (root / "artifacts.json").write_text(json.dumps({"artifacts": [
        {"path": "partial.bin", "status": "unknown_or_partial", "sha256": None}],
        "evidence_complete": False}))
    time.sleep(10)


def _stall_ladder_after_partial(request_path, result_path, artifact_path):
    root = Path(request_path).parent
    (root / "stage.json").write_text(json.dumps({
        "stage": "solve", "solve_completed": False,
        "ladder_solve_completed": False, "any_rung_solve_completed": True,
        "current_attempt": 1,
    }))
    partial = root / "attempts" / "rung-0"
    partial.mkdir(parents=True)
    (partial / "result.json").write_text(json.dumps({"solve_completed": True, "accepted": False}))
    time.sleep(10)


def _success_late(request_path, result_path, artifact_path):
    Path(result_path).write_text(json.dumps({"status": "success", "accepted": True, "solver_succeeded": True}))
    time.sleep(10)


def _actual_compat_stall_worker(request_path, result_path, artifact_path):
    from tools.run import solve_minimum_time_line as runner
    request = json.loads(Path(request_path).read_text())
    def stall_solve(*args, **kwargs):
        output = Path(request["config"]["output"])
        (output / "line.csv").write_text("partial,real,output")
        stage = Path(request_path).parent / "stage.json"
        from tools.run.solve_minimum_time_line import _publish_worker_stage
        _publish_worker_stage(stage.parent, "solve", solve_completed=False)
        from simplified_vehicle_models.solvers.minimum_time_run import _write_compact_artifact_index
        names = ["result.json", "compat_summary.snapshot.json", "line.csv", "racing_line.csv",
                 "racing_line_track.csv", "racing_line_track.xml", "rebuilt_width.csv", "trajectory.npz"]
        _write_compact_artifact_index(stage.parent, names, stage="compat_solve",
                                      external_root=output)
        time.sleep(10)
    with mock.patch.object(runner, "solve_line", side_effect=stall_solve):
        runner._compat_worker_entry(request, request_path, result_path, artifact_path)


def _raise_before_solution(request_path, result_path, artifact_path):
    Path(result_path).write_text(json.dumps({"status": "worker_exception", "accepted": False,
                                              "error_type": "RuntimeError", "error": "fixture failure"}))


def _no_result(request_path, result_path, artifact_path):
    return


def _malformed_result(request_path, result_path, artifact_path):
    Path(result_path).write_text("[")


def _oversize_result(request_path, result_path, artifact_path):
    with Path(result_path).open("wb") as stream:
        stream.write(b" " * (supervisor.MAX_RESULT_BYTES + 1))


def _success_then_exit(request_path, result_path, artifact_path):
    import os
    Path(result_path).write_text(json.dumps({"status": "success", "accepted": True,
                                              "solver_succeeded": True}))
    os._exit(7)


class MinimumTimeRunTests(unittest.TestCase):
    def test_oversize_request_fails_before_output_creation(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(ValueError, "request.*limit"):
                supervisor.run_request({"wall_cap_s": 3, "note": "x" * (1024 * 1024 + 1)}, f"{d}/oversize")
            self.assertEqual(list(Path(d).iterdir()), [])

    def test_compat_live_solve_timeout_reports_phase_and_real_partial_reference(self):
        from tools.diagnostics.generate_minimum_time_cli_fixture import build_fixture
        with tempfile.TemporaryDirectory() as d:
            fixture = Path(d) / "fixture"
            generated = build_fixture(fixture)
            output = Path(d) / "output"
            output.mkdir()
            root = output / ".supervision"
            config = {"track": str(fixture / "tiny_track.xml"), "formulation": generated["formulation"],
                      "surface": str(fixture / "surface.npz"), "output": str(output),
                      "margin": 0.1, "half_width": 2.0, "spacings": [5.0, 2.0, 1.0],
                      "wall_cap_s": 10.0, "solver": "legacy", "control_mode": "ideal"}
            raw = {"worker_kind": "compat_line_v1", "wall_cap_s": 10.0, "output": str(output), "config": config}
            with mock.patch.object(supervisor, "_worker", _actual_compat_stall_worker):
                result = supervisor.run_request(raw, str(root))
            self.assertEqual(result["status"], "wall_timeout")
            self.assertEqual(result["stage"], "solve")
            self.assertFalse(result["accepted"])
            self.assertFalse(result["solve_completed"])
            manifest = json.loads((root / "artifacts.json").read_text())
            line_ref = next(item for item in manifest["artifacts"] if item["path"] == "line.csv")
            self.assertEqual(line_ref["base"], "caller_output")
            self.assertTrue((output / "line.csv").exists())
            self.assertEqual(line_ref["status"], "unknown_or_partial")
            self.assertFalse((root / "line.csv").exists())

    def test_public_worker_index_finalization_keeps_stage_export_until_done(self):
        from tools.run import solve_minimum_time_line as runner
        with tempfile.TemporaryDirectory() as d:
            root = Path(d) / ".supervision"
            output = Path(d) / "output"
            root.mkdir(); output.mkdir()
            request_path = root / "request.json"
            request_path.write_text("{}")
            request = {"track": str(Path(d) / "track.xml"), "formulation": str(Path(d) / "form.xml"),
                       "surface": str(Path(d) / "surface.npz"), "output": str(output), "margin": .1,
                       "half_width": 2., "spacings": [5., 2.], "offset_spacing": 10.,
                       "path_smoothing": 0., "wall_cap_s": 10.}
            observed = []
            def fake_ladder(request_file, result_file, artifact_file):
                Path(result_file).write_text(json.dumps({"status": "success", "solve_completed": True,
                    "ladder_solve_completed": True, "any_rung_solve_completed": True,
                    "attempts": [], "accepted": False}))
            def slow_index(*args, **kwargs):
                observed.append(json.loads((root / "stage.json").read_text())["stage"])
                return {"index_stage": kwargs.get("stage")}
            with mock.patch.object(runner, "_validate_output_paths"), \
                 mock.patch("simplified_vehicle_models.solvers.input_preflight.preflight_inputs", return_value=None), \
                 mock.patch.object(runner, "_public_spline_worker_request", return_value={"source_hashes": {},
                     "runtime_provenance": {}}), \
                 mock.patch("tools.diagnostics.verify_no_tv_minimum_lap._attempt_ladder_worker", side_effect=fake_ladder), \
                 mock.patch.object(supervisor, "_write_compact_artifact_index", side_effect=slow_index):
                runner._prepare_public_spline_request_in_worker(request, str(request_path),
                    str(root / "result.json"), str(root / "solution.npz"))
            self.assertEqual(observed, ["export"])
            self.assertEqual(json.loads((root / "stage.json").read_text())["stage"], "complete")

    def test_worker_index_finalization_keeps_terminal_stage_export_until_done(self):
        import argparse
        from tools.run import solve_minimum_time_line as runner
        with tempfile.TemporaryDirectory() as d:
            root = Path(d) / ".supervision"
            output = Path(d) / "output"
            output.mkdir()
            root.mkdir()
            result_path = root / "result.json"
            request_path = root / "request.json"
            request_path.write_text(json.dumps({"worker_kind": "compat_line_v1", "config": {"output": str(output)}}))
            runner._publish_worker_stage(root, "validate", solve_completed=True)
            runner._publish_worker_stage(root, "export", solve_completed=True)
            observed = []
            def slow_index(*args, **kwargs):
                stage = json.loads((root / "stage.json").read_text())
                observed.append(stage["stage"])
                time.sleep(0.05)
                return {"index_stage": kwargs.get("stage")}
            def fake_pipeline(args, stage_callback=None):
                for name in ("setup", "seed", "solve", "validate", "export", "validate"):
                    stage_callback(name, solve_completed=True)
                (output / "summary.json").write_text("summary")
                return {"accepted": False, "solve_completed": True}
            with mock.patch.object(runner, "_run_compat_pipeline", side_effect=fake_pipeline), \
                 mock.patch.object(supervisor, "_write_compact_artifact_index", side_effect=slow_index):
                runner._compat_worker_entry({"config": {"track": output / "t.xml", "output": output,
                    "formulation": output / "f.xml", "surface": output / "s.npz"}},
                    str(request_path), str(result_path), str(root / "solution.npz"))
            self.assertTrue(observed)
            self.assertEqual(observed[-1], "export")
            final_stage = json.loads((root / "stage.json").read_text())
            self.assertEqual(final_stage["stage"], "complete")

    def test_final_digest_index_hashes_are_current_and_stage_has_history(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            stage_path = root / "stage.json"
            initial = {"stage": "solve", "events": [{"stage": "setup", "started_monotonic": 1.0,
                                                        "ended_monotonic": 1.1}],
                       "solve_completed": True, "ladder_solve_completed": True,
                       "any_rung_solve_completed": True}
            stage_path.write_text(json.dumps(initial))
            summary = root / "summary.json"
            summary.write_text("done")
            from tools.run.solve_minimum_time_line import _publish_worker_stage
            _publish_worker_stage(root, "complete", solve_completed=True,
                                  ladder_solve_completed=True, any_rung_solve_completed=True)
            (root / "result.json").write_text('{"result":"final"}')
            stage_after_complete = Path(root / "stage.json").read_bytes()
            supervisor._write_compact_artifact_index(root, ["summary.json", "result.json"], stage="complete")
            self.assertEqual(Path(root / "stage.json").read_bytes(), stage_after_complete)
            manifest = json.loads((root / "artifacts.json").read_text())
            entries = {item["path"]: item for item in manifest["artifacts"]}
            import hashlib
            self.assertEqual(entries["summary.json"]["sha256"], hashlib.sha256(b"done").hexdigest())
            self.assertEqual(entries["result.json"]["sha256"], hashlib.sha256(b'{"result":"final"}').hexdigest())
            self.assertNotIn("stage.json", entries)
            self.assertEqual(manifest["index_stage"], "complete")
            final_stage = json.loads(stage_path.read_text())
            self.assertEqual(final_stage["stage"], "complete")
            self.assertGreaterEqual(len(final_stage["events"]), 2)
            self.assertTrue(final_stage["ladder_solve_completed"])

    def test_external_artifact_index_uses_explicit_safe_base_without_placeholders(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d) / "supervision"
            output = Path(d) / "output"
            root.mkdir(); output.mkdir()
            partial = output / "line.csv"
            partial.write_text("real partial line")
            manifest = supervisor._write_compact_artifact_index(
                root, ["line.csv", "trajectory.npz"], stage="compat_setup", external_root=output)
            entries = {item["path"]: item for item in manifest["artifacts"]}
            self.assertEqual(entries["line.csv"], {"path": "line.csv", "base": "caller_output",
                "status": "unknown_or_partial", "sha256": None})
            self.assertEqual(entries["trajectory.npz"]["base"], "caller_output")
            self.assertEqual(entries["trajectory.npz"]["status"], "not_created")
            self.assertFalse((root / "line.csv").exists())
            self.assertFalse((root / "trajectory.npz").exists())
            supervisor._inventory(root)
            bad = {"artifacts": [{"path": "../../secret", "base": "caller_output"}]}
            (root / "artifacts.json").write_text(json.dumps(bad))
            fallback = supervisor._inventory(root)
            self.assertEqual(fallback["digest_index_status"], "unavailable_or_oversize")
            self.assertFalse(any(item.get("path") == "../../secret" for item in fallback["artifacts"]))
            self.assertEqual(len(fallback["artifacts"]), 0)

    def test_parent_finalization_does_not_read_artifact_payload_bytes(self):
        with tempfile.TemporaryDirectory() as d:
            with mock.patch.object(supervisor, "_worker", _raise_before_solution):
                with mock.patch.object(Path, "read_bytes", side_effect=AssertionError("parent payload read")):
                    result = supervisor.run_request({"wall_cap_s": 3}, f"{d}/bounded")
            self.assertEqual(result["status"], "worker_exception")

    def test_oversize_result_is_classified_without_reading_all(self):
        with tempfile.TemporaryDirectory() as d:
            with mock.patch.object(supervisor, "_worker", _oversize_result):
                result = supervisor.run_request({"wall_cap_s": 3}, f"{d}/bad-result")
            self.assertEqual(result["status"], "worker_malformed_result")

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
                self.assertIn({"path": "partial.bin", "status": "unknown_or_partial", "sha256": None}, manifest["artifacts"])
                self.assertNotIn("digest_index_status", manifest)
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

    def test_explicit_ideal_spline_selects_shared_public_route(self):
        from types import SimpleNamespace
        from tools.run import solve_minimum_time_line as runner
        args = SimpleNamespace(control_mode="ideal", solver="spline", legacy_solver=False,
                               path_smoothing=0.0, offset_spacing=None)
        expected = {"accepted": False, "numerically_validated": False}
        with mock.patch.object(runner, "_run_public_shared_spline", return_value=expected) as route, \
             mock.patch.object(runner, "_run_compat") as compat:
            self.assertIs(runner.run(args), expected)
        route.assert_called_once_with(args)
        compat.assert_not_called()

    def test_cli_fixture_generator_is_fresh_and_loadable(self):
        from tools.diagnostics.generate_minimum_time_cli_fixture import build_fixture
        from simplified_vehicle_models.tracks.profile import load_track
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "fixture"
            generated = build_fixture(output)
            self.assertEqual(generated["geometry_parameters"], {"radius_m": 10.0, "segments": 32})
            self.assertFalse(generated["chart_is_physical_validation"])
            self.assertTrue(load_track(output / "tiny_track.xml").is_closed)
            with self.assertRaises(FileExistsError):
                build_fixture(output)

    def test_public_ladder_envelope_composes_three_single_rung_requests(self):
        from tools.run.solve_minimum_time_line import build_shared_spline_ladder_request
        from tools.diagnostics.verify_no_tv_minimum_lap import MatrixCase, _validate_request_options
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, formulation, surface = root / "track.xml", root / "formulation.xml", root / "surface.npz"
            source.write_text('<model schema-version="1.0" type="track-model"><data ref="geometry.csv" /></model>')
            (root / "geometry.csv").write_text("distance,north,east\\n0,0,0\\n10,10,0\\n")
            formulation.write_text("<formulation/>")
            surface.write_bytes(b"fixture")
            case = MatrixCase("tiny", source, "closed", "closed_flying", 0.0, None)
            request = build_shared_spline_ladder_request(
                case, source=source, formulation=formulation, surface=surface,
                spacings=(5.0, 2.0, 1.0), margin=0.1, half_width=None,
                basis_spacing=10.0, wall_cap_s=30.0,
            )
            self.assertEqual(request["worker_kind"], "no_tv_spline_ladder_v1")
            self.assertEqual(request["request_envelope_version"], 2)
            self.assertEqual(request["mesh_spacings_m"], [5.0, 2.0, 1.0])
            self.assertEqual([r["mesh_spacing_m"] for r in request["rung_requests"]], [5.0, 2.0, 1.0])
            self.assertTrue(all(r["request_envelope_version"] == 1 for r in request["rung_requests"]))
            _validate_request_options(request)
            for rung in request["rung_requests"]:
                _validate_request_options(rung)
            invalid_nested = json.loads(json.dumps(request))
            invalid_nested["rung_requests"][1]["worker_kind"] = "no_tv_spline_ladder_v1"
            with self.assertRaises(ValueError):
                _validate_request_options(invalid_nested)
            invalid_policy = json.loads(json.dumps(request))
            invalid_policy["rung_requests"][1]["settings"]["tolerance"] *= 2
            with self.assertRaises(ValueError):
                _validate_request_options(invalid_policy)
            with self.assertRaises(ValueError):
                _validate_request_options(dict(request, worker_kind="no_tv_spline_v1"))

    def test_ladder_fixed_dispatch_retains_each_rung_in_one_worker(self):
        from tools.diagnostics import verify_no_tv_minimum_lap as harness
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            requests = [
                {"worker_kind": "no_tv_spline_v1", "request_envelope_version": 1,
                 "mesh_spacing_m": spacing, "mesh_spacings_m": [spacing],
                 "case": {}, "source_geometry_contract": "native_topology_reference_v1"}
                for spacing in (5.0, 2.0, 1.0)
            ]
            request = {"worker_kind": "no_tv_spline_ladder_v1", "request_envelope_version": 2,
                       "wall_cap_s": 20.0, "mesh_spacings_m": [5.0, 2.0, 1.0],
                       "case": {}, "source_hashes": {}, "provenance": {},
                       "rung_requests": requests}
            request_path, result_path, artifact_path = root / "request.json", root / "result.json", root / "solution.npz"
            request_path.write_text(json.dumps(request))
            calls = []
            def run_rung(rung_path, rung_result, rung_artifact, stage_callback=None):
                rung = json.loads(Path(rung_path).read_text())
                calls.append(rung["mesh_spacing_m"])
                stage_callback("solve", False)
                output = Path(rung_result)
                output.write_text(json.dumps({"status": "success", "accepted": False,
                                              "solver_succeeded": True, "numerically_validated": True,
                                              "solve_completed": True, "wall_seconds": 0.01,
                                              "checks": {"numeric": True}, "artifact": Path(rung_artifact).name}))
                Path(rung_artifact).write_bytes(f"rung-{rung['mesh_spacing_m']}".encode())
            with mock.patch.object(harness, "_validate_request_options"), \
                 mock.patch.object(harness, "MatrixCase", return_value=type("Case", (), {"topology": "closed"})()), \
                 mock.patch.object(harness, "_attempt_worker", side_effect=run_rung):
                harness._attempt_ladder_worker(str(request_path), str(result_path), str(artifact_path))
            result = json.loads(result_path.read_text())
            self.assertEqual(calls, [5.0, 2.0, 1.0])
            self.assertEqual([a["spacing_m"] for a in result["attempts"]], [5.0, 2.0, 1.0])
            self.assertFalse(result["accepted"])
            self.assertFalse(result["numerically_validated"])
            self.assertTrue(all((root / "attempts" / f"rung-{i}" / "result.json").is_file() for i in range(3)))

    def test_ladder_partial_solve_never_means_ladder_completed(self):
        from tools.diagnostics import verify_no_tv_minimum_lap as harness
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            requests = [{"worker_kind": "no_tv_spline_v1", "request_envelope_version": 1,
                         "mesh_spacing_m": spacing, "mesh_spacings_m": [spacing],
                         "case": {}, "source_geometry_contract": "native_topology_reference_v1"}
                        for spacing in (5.0, 2.0)]
            outer = {"worker_kind": "no_tv_spline_ladder_v1", "request_envelope_version": 2,
                     "wall_cap_s": 20.0, "mesh_spacings_m": [5.0, 2.0],
                     "case": {}, "source_hashes": {}, "provenance": {}, "rung_requests": requests}
            request_path, result_path = root / "request.json", root / "result.json"
            request_path.write_text(json.dumps(outer))
            def run_rung(rung_path, rung_result, rung_artifact, stage_callback=None):
                rung = json.loads(Path(rung_path).read_text())
                solved = rung["mesh_spacing_m"] == 5.0
                stage_callback("solve", solved)
                result = {"status": "Solve_Succeeded" if solved else "solver_failed",
                          "solver_succeeded": solved,
                          "solve_completed": solved,
                          "wall_seconds": 0.25}
                Path(rung_result).write_text(json.dumps(result))
            with mock.patch.object(harness, "_validate_request_options"), \
                 mock.patch.object(harness, "MatrixCase", return_value=type("Case", (), {"topology": "closed"})()), \
                 mock.patch.object(harness, "_attempt_worker", side_effect=run_rung):
                harness._attempt_ladder_worker(str(request_path), str(result_path), str(root / "solution.npz"))
            result = json.loads(result_path.read_text())
            stage = json.loads((root / "stage.json").read_text())
            self.assertEqual(len(result["attempts"]), 2)
            self.assertTrue(result["any_rung_solve_completed"])
            self.assertFalse(result["ladder_solve_completed"])
            self.assertFalse(result["solve_completed"])
            self.assertTrue(stage["any_rung_solve_completed"])
            self.assertFalse(stage["solve_completed"])
            self.assertFalse(result["accepted"])
            self.assertEqual(result["solve_wall_seconds_sum"], 0.5)
            self.assertTrue(all("started_monotonic" in event and "ended_monotonic" in event
                                for event in stage["events"][:-1]))

    def test_ladder_timeout_after_rung_solve_retains_partial_but_not_full_completion(self):
        from tools.diagnostics import verify_no_tv_minimum_lap as harness
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            marker = root / "stage.json"
            marker.write_text(json.dumps({"stage": "solve", "solve_completed": False,
                                          "any_rung_solve_completed": True,
                                          "ladder_solve_completed": False}))
            proc = _FakeProcess(alive=False)
            proc.exitcode = None
            with mock.patch.object(supervisor, "_worker", _stall_ladder_after_partial):
                result = supervisor.run_request({"wall_cap_s": 0.6, "worker_kind": "no_tv_spline_ladder_v1"},
                                                str(root / "attempt"))
            self.assertEqual(result["status"], "wall_timeout")
            self.assertFalse(result["solve_completed"])
            self.assertFalse(result["ladder_solve_completed"])
            self.assertFalse(result["accepted"])

    def test_default_solver_and_controller_routes_remain_compat(self):
        from types import SimpleNamespace
        from tools.run import solve_minimum_time_line as runner
        with mock.patch.object(runner, "build_policy", return_value=object()), \
             mock.patch.object(runner, "build_replay", return_value=object()):
            self._check_compat_routes(runner, SimpleNamespace)

    def _check_compat_routes(self, runner, SimpleNamespace):
        for args in (
            SimpleNamespace(control_mode="ideal", solver=None, legacy_solver=False),
            SimpleNamespace(control_mode="allocation", solver="spline", legacy_solver=False,
                            control_policy="registered"),
            SimpleNamespace(control_mode="closed_loop", solver="spline", legacy_solver=False,
                            replay_adapter="registered"),
        ):
            with self.subTest(args=args), \
                 mock.patch.object(runner, "_run_public_shared_spline") as shared, \
                 mock.patch.object(runner, "_run_compat", return_value={"route": "compat"}) as compat:
                self.assertEqual(runner.run(args), {"route": "compat"})
                shared.assert_not_called()
                compat.assert_called_once_with(args)

    def test_standard_request_adapter_roundtrips_effective_configuration(self):
        from tools.run.solve_minimum_time_line import build_shared_spline_request
        from tools.diagnostics.verify_no_tv_minimum_lap import MatrixCase
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "track.xml"
            formulation = root / "formulation.xml"
            surface = root / "signed.npz"
            source.write_text('<model schema-version="1.0" type="track-model"><data ref="geometry.csv" /></model>')
            (root / "geometry.csv").write_text("distance,north,east\\n0,0,0\\n1,1,0\\n")
            formulation.write_text('<formulation/>')
            surface.write_bytes(b"surface")
            case = MatrixCase("custom", source, "closed", "closed_flying", 0.0, None)
            request = build_shared_spline_request(
                case, source=source, formulation=formulation, surface=surface,
                spacings=(2.5,), margin=0.23, half_width=None,
                basis_spacing=10.0, wall_cap_s=12.0,
            )
            decoded = json.loads(json.dumps(request, allow_nan=False))
            self.assertEqual(decoded["worker_kind"], "no_tv_spline_v1")
            self.assertEqual(decoded["case"]["track"], str(source))
            self.assertEqual(decoded["corridor"], {"margin_m": 0.23, "half_width_m": None})
            self.assertEqual(decoded["execution_identity"]["corridor"], decoded["corridor"])
            self.assertEqual(decoded["mesh_spacings_m"], [2.5])
            self.assertEqual(decoded["basis_spacing_m"], 10.0)
            self.assertEqual(decoded["start_treatment"], "closed_flying")
            self.assertIn("git_head", decoded["provenance"])
            self.assertTrue(decoded["source_hashes"])
            from tools.diagnostics.verify_no_tv_minimum_lap import _validate_request_options
            _validate_request_options(decoded)
            from tools.diagnostics.verify_no_tv_minimum_lap import _same_refinement_conditions
            left = {"request": dict(decoded), "actual_initializer": {"fingerprint": "same"},
                    "surface_representation": "signed_transient_normalized_v1",
                    "source_hashes_after": decoded["source_hashes"], "provenance_after": decoded["provenance"]}
            right = {"request": dict(decoded), "actual_initializer": {"fingerprint": "same"},
                     "surface_representation": "signed_transient_normalized_v1",
                     "source_hashes_after": decoded["source_hashes"], "provenance_after": decoded["provenance"]}
            self.assertTrue(_same_refinement_conditions(left, right))
            right["request"]["corridor"] = {"margin_m": .24, "half_width_m": None}
            self.assertFalse(_same_refinement_conditions(left, right))
            right["request"] = dict(decoded, start_speed_mps=0.25)
            self.assertFalse(_same_refinement_conditions(left, right))
            mutations = (
                dict(decoded, request_envelope_version=0),
                dict(decoded, route_scope="single_fine_rung_diagnostc"),
                dict(decoded, mesh_spacing_m=5.0),
                dict(decoded, settings=dict(decoded["settings"], start_speed=2.0)),
                dict(decoded, case=dict(decoded["case"], topology="open")),
                dict(decoded, worker_kind="arbitrary"),
                dict(decoded, mystery=1),
                dict(decoded, mesh_spacings_m=[5.0, 2.0]),
            )
            for invalid in mutations:
                with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                    _validate_request_options(invalid)
            with self.assertRaisesRegex(ValueError, "serialized case track"):
                build_shared_spline_request(
                    case, source=formulation, formulation=formulation, surface=surface,
                    spacings=(2.5,), margin=0.23, half_width=None,
                    basis_spacing=10.0, wall_cap_s=12.0,
                )
            from tools.diagnostics import verify_no_tv_minimum_lap as harness
            request_path, result_path, artifact_path = (root / "request.json", root / "result.json", root / "solution.npz")
            request_path.write_text(json.dumps(decoded))
            with mock.patch.object(harness, "make_track_view", return_value=object()), \
                 mock.patch.object(harness, "corridor_from_track", return_value=object()) as corridor_spy, \
                 mock.patch.object(harness, "compile_formulation", side_effect=RuntimeError("stop after corridor")):
                harness._attempt_worker(str(request_path), str(result_path), str(artifact_path))
            self.assertEqual(corridor_spy.call_args.kwargs["margin"], .23)
            self.assertIsNone(corridor_spy.call_args.kwargs["half_width"])
            open_result_path = root / "open_result.json"
            open_case = MatrixCase("rolling", source, "open", "open_rolling_10", 7.25, None)
            open_request = build_shared_spline_request(
                open_case, source=source, formulation=formulation, surface=surface,
                spacings=(2.5,), margin=0.19, half_width=None,
                basis_spacing=10.0, wall_cap_s=12.0,
            )
            request_path.write_text(json.dumps(open_request))
            with mock.patch.object(harness, "make_track_view", return_value=object()), \
                 mock.patch.object(harness, "corridor_from_track", return_value=object()), \
                 mock.patch.object(harness, "compile_formulation", return_value=type("Compiled", (), {"model": object()})()), \
                 mock.patch.object(harness, "point_mass_view_of_planar", return_value=object()), \
                 mock.patch.object(harness, "load_surface_dispatch", return_value=(type("Surface", (), {"provenance_hash": "expected"})(), "signed")), \
                 mock.patch.object(harness, "capability_hash", return_value="expected"), \
                 mock.patch.object(harness, "QssSettings", side_effect=RuntimeError("stop after settings")) as settings_spy:
                harness._attempt_worker(str(request_path), str(open_result_path), str(artifact_path))
            self.assertEqual(settings_spy.call_args.kwargs["start_speed"], 7.25)

    def test_transport_contract_single_ladder_identity_immutable(self):
        from tools.run.solve_minimum_time_line import build_shared_spline_ladder_request
        from tools.diagnostics.verify_no_tv_minimum_lap import MatrixCase, _validate_request_options
        from simplified_vehicle_models.solvers.track_problem_contract import validate_problem_contract
        import numpy as np
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, formulation, surface = root / "track.xml", root / "form.xml", root / "chart.npz"
            source.write_text("<track/>"); formulation.write_text("<form/>"); surface.write_bytes(b"chart")
            raw = {"schema": "track_problem_v1", "topology": "open", "source_stations": [0., 1., 2.],
                "lower_offset": [-1., -1., -1.], "upper_offset": [1., 1., 1.],
                "boundary": {"start_mode": "rolling", "start_speed_mps": 3., "finish_mode": "free"}}
            original = json.loads(json.dumps(raw))
            case = MatrixCase("rolling", source, "open", "open_rolling_10", 0.5, 1.0, "diagnostic")
            with mock.patch("tools.diagnostics.verify_no_tv_minimum_lap.source_hashes", return_value={"input": "hash"}), \
                 mock.patch("tools.diagnostics.no_tv_provenance.runtime_provenance", return_value={"runtime": {"python": "test"}}):
                request = build_shared_spline_ladder_request(case, source=source, formulation=formulation, surface=surface,
                    spacings=(5., 2.5), margin=.1, half_width=1., basis_spacing=35., wall_cap_s=3.,
                    track_problem_contract=raw)
            self.assertEqual(raw, original)
            expected = validate_problem_contract(raw)
            self.assertEqual(request["case"]["start_speed_mps"], 3.)
            self.assertEqual(request["settings"]["start_speed"], 3.)
            self.assertEqual(request["track_problem_contract"], expected["normalized_contract"])
            for rung in request["rung_requests"]:
                self.assertEqual(rung["track_problem_contract"], expected["normalized_contract"])
            _validate_request_options(request)
            drifted = json.loads(json.dumps(request))
            drifted["rung_requests"][1]["track_problem_contract"]["boundary"]["start_speed_mps"] = 4.
            with self.assertRaisesRegex(ValueError, "outer and rung"):
                _validate_request_options(drifted)
            malformed = dict(raw, boundary=dict(raw["boundary"], start_speed_mps=-1.))
            with mock.patch("tools.diagnostics.verify_no_tv_minimum_lap.source_hashes", side_effect=AssertionError("hashes must not run")):
                with self.assertRaisesRegex(ValueError, "invalid track_problem_contract"):
                    build_shared_spline_ladder_request(case, source=source, formulation=formulation, surface=surface,
                        spacings=(5.,), margin=.1, half_width=1., basis_spacing=35., wall_cap_s=3.,
                        track_problem_contract=malformed)

    def test_public_closed_periodic_contract_preserves_legacy_case_on_builder(self):
        from tools.run.solve_minimum_time_line import _prepare_public_spline_request_in_worker
        contract = {"schema": "track_problem_v1", "topology": "closed", "source_stations": [0., 1., 2.],
            "lower_offset": [-1., -1., -1.], "upper_offset": [1., 1., 1.],
            "boundary": {"start_mode": "flying", "start_speed_mps": 3., "finish_mode": "periodic"}}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); request_path = root / "request.json"; result_path = root / "result.json"
            track_path = root / "closed.xml"
            track_path.write_text('<model schema-version="1.0" type="track-model"><data ref="geometry.csv"/></model>')
            (root / "geometry.csv").write_text("distance,north,east\\n0,0,0\\n1,1,0\\n")
            request = {"worker_kind": "public_spline_setup_v1", "wall_cap_s": 3., "track": str(track_path),
                "formulation": str(root / "form.xml"), "surface": str(root / "chart.npz"), "output": str(root / "out"),
                "margin": .1, "half_width": 2., "spacings": [5., 2.5], "offset_spacing": 35., "path_smoothing": 0.,
                "track_problem_contract": contract}
            request_path.write_text(json.dumps(request))
            from tools.run.solve_minimum_time_line import _public_spline_worker_request
            args = type("Args", (), {k: request[k] for k in request if k != "worker_kind"})()
            with mock.patch("tools.run.solve_minimum_time_line.load_track",
                            return_value=type("Track", (), {"is_closed": True})()), \
                 mock.patch("tools.run.solve_minimum_time_line._validate_output_paths"), \
                 mock.patch("tools.run.solve_minimum_time_line._validate_track_for_line_pipeline"), \
                 mock.patch("tools.run.solve_minimum_time_line.build_shared_spline_ladder_request",
                            return_value={"track_problem_contract": contract, "case": {"start_speed_mps": 0.},
                                "settings": {"start_speed": 0.}, "rung_requests": []}) as builder:
                built = _public_spline_worker_request(args)
            self.assertEqual(built["case"]["start_speed_mps"], 0.)
            self.assertEqual(built["settings"]["start_speed"], 0.)
            builder.assert_called_once()
            self.assertEqual(builder.call_args.kwargs["track_problem_contract"], contract)

    def test_public_worker_contract_none_preserves_namespace_defaults(self):
        from tools.run.solve_minimum_time_line import _public_spline_worker_request
        args = type("Args", (), {"track": "ignored", "output": "ignored", "formulation": "ignored",
            "surface": "ignored", "margin": .1, "half_width": None, "spacings": [5., 2.5],
            "offset_spacing": 35., "path_smoothing": 0.})()
        with mock.patch("tools.run.solve_minimum_time_line.load_track", return_value=type("Track", (), {"is_closed": True})()), \
             mock.patch("tools.run.solve_minimum_time_line._validate_output_paths"), \
             mock.patch("tools.run.solve_minimum_time_line._validate_track_for_line_pipeline"), \
             mock.patch("tools.run.solve_minimum_time_line.build_shared_spline_ladder_request", return_value={"legacy": True}) as builder:
            self.assertEqual(_public_spline_worker_request(args), {"legacy": True})
        self.assertIsNone(builder.call_args.kwargs["track_problem_contract"])

    def test_public_worker_pure_contract_gate_precedes_input_preflight(self):
        from tools.run.solve_minimum_time_line import _prepare_public_spline_request_in_worker
        contract = {"schema": "track_problem_v1", "topology": "open", "source_stations": [0., 1., 2.],
            "lower_offset": [-1., -1., -1.], "upper_offset": [1., 1., 1.],
            "boundary": {"start_mode": "rolling", "start_speed_mps": -1., "finish_mode": "free"}}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); request_path = root / "request.json"; result_path = root / "result.json"
            request = {"worker_kind": "public_spline_setup_v1", "wall_cap_s": 3., "track": str(root / "missing.xml"),
                "formulation": str(root / "missing-form.xml"), "surface": str(root / "missing-chart.npz"),
                "output": str(root / "out"), "margin": .1, "half_width": 1., "spacings": [5.],
                "offset_spacing": 35., "path_smoothing": 0., "track_problem_contract": contract}
            request_path.write_text(json.dumps(request))
            with mock.patch("simplified_vehicle_models.solvers.input_preflight.preflight_inputs",
                            side_effect=AssertionError("filesystem admission must not run")), \
                 mock.patch("tools.run.solve_minimum_time_line._validate_output_paths",
                            side_effect=AssertionError("output admission must not run")), \
                 mock.patch("tools.diagnostics.verify_no_tv_minimum_lap._attempt_ladder_worker",
                            side_effect=AssertionError("solver must not run")):
                _prepare_public_spline_request_in_worker(request, str(request_path), str(result_path), str(root / "solution.npz"))
            result = json.loads(result_path.read_text())
            self.assertEqual(result["status"], "track_problem_invalid")
            self.assertEqual(result["validation_status"], "not_run")
            self.assertFalse(result["solver_attempted"])
