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
