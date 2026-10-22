import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from tools.diagnostics import verify_no_tv_minimum_lap as harness


class NoTvMinimumLapHarnessTest(unittest.TestCase):
    def test_preregistered_cases_keep_full_geometry_and_explicit_starts(self):
        cases = harness.default_cases()
        self.assertEqual(cases["mis_closed"].track, cases["mis_cut_open"].track)
        self.assertEqual(cases["fsc_closed"].track, cases["fsc_cut_open"].track)
        self.assertEqual(cases["mis_closed"].start_treatment, "closed_flying")
        self.assertEqual(cases["mis_cut_open"].start_treatment, "open_rolling_10")
        self.assertEqual(cases["acceleration"].start_treatment, "open_rolling_10")
        self.assertIn("NOTstandingstart", cases["acceleration"].candidate_only_reason)

    def test_full_cut_open_view_retains_mis_geometry_and_width_samples(self):
        cases = harness.default_cases()
        closed = harness.make_track_view(cases["mis_closed"])
        opened = harness.make_track_view(cases["mis_cut_open"])
        self.assertTrue(closed.is_closed)
        self.assertFalse(opened.is_closed)
        np.testing.assert_array_equal(closed.distance, opened.distance)
        np.testing.assert_array_equal(closed.curvature, opened.curvature)
        np.testing.assert_array_equal(closed.half_width_left, opened.half_width_left)
        np.testing.assert_array_equal(closed.half_width_right, opened.half_width_right)

    def test_attempt_records_only_preregistered_rungs_and_no_tv(self):
        request = harness.build_attempt(harness.default_cases()["acceleration"],
            mesh_spacing_m=5.0, basis_spacing_m=35.0, maximum_iterations=7,
            wall_cap_s=3.0, seed_name="centerline", formulation=Path("form.xml"),
            surface=Path("surface.npz"))
        self.assertEqual(request["settings"]["maximum_iterations"], 7)
        self.assertEqual(request["settings"]["start_speed"], 10.0)
        self.assertTrue(request["no_torque_vectoring"])
        self.assertFalse(request["mode_switches"])
        self.assertFalse(request["controller_globals"])
        with self.assertRaisesRegex(ValueError, "preregistered"):
            harness.build_attempt(harness.default_cases()["acceleration"],
                mesh_spacing_m=3.0, basis_spacing_m=35.0, maximum_iterations=7,
                wall_cap_s=3.0, seed_name="centerline", formulation=Path("form.xml"),
                surface=Path("surface.npz"))

    def test_default_dense_settings_preserve_legacy_sites_and_json_restores_tuple(self):
        request = harness.build_attempt(harness.default_cases()["acceleration"],
            mesh_spacing_m=5.0, basis_spacing_m=35.0, maximum_iterations=7,
            wall_cap_s=3.0, seed_name="centerline", formulation=Path("form.xml"),
            surface=Path("surface.npz"))
        self.assertEqual(request["settings"]["off_collocation_fractions"], harness.DEFAULT_OFF_COLLOCATION_FRACTIONS)
        serialized = harness._json_safe(request)
        self.assertIsInstance(serialized["settings"]["off_collocation_fractions"], list)
        self.assertEqual(tuple(serialized["settings"]["off_collocation_fractions"]),
                         harness.DEFAULT_OFF_COLLOCATION_FRACTIONS)
        self.assertEqual(request["settings"]["tolerance"], 1e-6)
        self.assertEqual(request["settings"]["hessian_approximation"], "exact")
        self.assertEqual(request["dense_maximum_refinements"], 4)
        self.assertFalse(request["matched_reference"])

    def test_attempt_persists_explicit_hessian_approximation(self):
        request = harness.build_attempt(harness.default_cases()["acceleration"],
            mesh_spacing_m=5.0, basis_spacing_m=35.0, maximum_iterations=7,
            wall_cap_s=3.0, seed_name="centerline", formulation=Path("form.xml"),
            surface=Path("surface.npz"), hessian_approximation="limited-memory")
        self.assertEqual(request["settings"]["hessian_approximation"], "limited-memory")

    def test_invalid_hessian_approximation_is_rejected(self):
        case = harness.default_cases()["acceleration"]
        with self.assertRaisesRegex(ValueError, "hessian_approximation"):
            harness.build_attempt(case, mesh_spacing_m=5.0, basis_spacing_m=35.0,
                maximum_iterations=7, wall_cap_s=3.0, seed_name="centerline",
                formulation=Path("form.xml"), surface=Path("surface.npz"),
                hessian_approximation="bad-hessian")

    def test_explicit_dense_sites_and_invalid_controls_are_rejected(self):
        case = harness.default_cases()["acceleration"]
        request = harness.build_attempt(case, mesh_spacing_m=5.0, basis_spacing_m=35.0,
            maximum_iterations=7, wall_cap_s=3.0, seed_name="centerline",
            formulation=Path("form.xml"), surface=Path("surface.npz"),
            off_collocation_divisions=32, nlp_tolerance=1e-8, dense_maximum_refinements=6)
        self.assertEqual(request["settings"]["off_collocation_fractions"],
                         tuple(i / 32 for i in range(1, 32) if i != 16))
        for tolerance in (0.0, float("nan")):
            with self.assertRaisesRegex(ValueError, "nlp_tolerance"):
                harness.build_attempt(case, mesh_spacing_m=5.0, basis_spacing_m=35.0,
                    maximum_iterations=7, wall_cap_s=3.0, seed_name="centerline",
                    formulation=Path("form.xml"), surface=Path("surface.npz"), nlp_tolerance=tolerance)
        for refinements in (1, float("nan")):
            with self.assertRaises((ValueError, OverflowError)):
                harness.build_attempt(case, mesh_spacing_m=5.0, basis_spacing_m=35.0,
                    maximum_iterations=7, wall_cap_s=3.0, seed_name="centerline",
                    formulation=Path("form.xml"), surface=Path("surface.npz"), dense_maximum_refinements=refinements)
        for iterations in (True, float("nan"), "7"):
            with self.assertRaisesRegex(ValueError, "maximum_iterations"):
                harness.build_attempt(case, mesh_spacing_m=5.0, basis_spacing_m=35.0,
                    maximum_iterations=iterations, wall_cap_s=3.0, seed_name="centerline",
                    formulation=Path("form.xml"), surface=Path("surface.npz"))

    def test_diagnostic_rungs_are_explicitly_labelled(self):
        case = harness.default_cases()["acceleration"]
        with self.assertRaisesRegex(ValueError, "diagnostic"):
            harness.build_attempt(case, mesh_spacing_m=0.625, basis_spacing_m=12.0,
                maximum_iterations=7, wall_cap_s=3.0, seed_name="centerline",
                formulation=Path("form.xml"), surface=Path("surface.npz"))
        request = harness.build_attempt(case, mesh_spacing_m=0.625, basis_spacing_m=12.0,
            maximum_iterations=7, wall_cap_s=3.0, seed_name="centerline",
            formulation=Path("form.xml"), surface=Path("surface.npz"), diagnostic_rungs=True)
        self.assertEqual(request["mesh_rung_status"], "diagnostic_not_preregistered")
        self.assertEqual(request["basis_rung_status"], "diagnostic_not_preregistered")

    def test_matched_reference_request_is_opt_in(self):
        request = harness.build_attempt(harness.default_cases()["acceleration"],
            mesh_spacing_m=5.0, basis_spacing_m=35.0, maximum_iterations=7,
            wall_cap_s=3.0, seed_name="centerline", formulation=Path("form.xml"),
            surface=Path("surface.npz"), matched_reference=True)
        self.assertTrue(request["matched_reference"])

    def test_energy_profile_seed_is_opt_in_and_serialized(self):
        request = harness.build_attempt(harness.default_cases()["acceleration"],
            mesh_spacing_m=5.0, basis_spacing_m=35.0, maximum_iterations=7,
            wall_cap_s=3.0, seed_name="centerline", formulation=Path("form.xml"),
            surface=Path("surface.npz"), seed_profile="energy-nlp", strict_profile_seed=True)
        self.assertEqual(request["seed_profile"], "energy-nlp")
        self.assertTrue(request["strict_profile_seed"])
        with self.assertRaisesRegex(ValueError, "seed_profile"):
            harness.build_attempt(harness.default_cases()["acceleration"], mesh_spacing_m=5.0,
                basis_spacing_m=35.0, maximum_iterations=7, wall_cap_s=3.0,
                seed_name="centerline", formulation=Path("form.xml"), surface=Path("surface.npz"),
                seed_profile="bad")

    def test_worker_records_profile_seed_fallback(self):
        class FakeSpline:
            control_count = 2
            control_distance = np.array([0.0, 1.0])
        class FakeSolution:
            distance = np.array([0.0, 1.0]); offset = np.zeros(2); heading_error = np.zeros(2)
            curvature = np.zeros(2); speed = np.full(2, 10.); acceleration = np.zeros(2)
            midpoint_acceleration = np.zeros(1); coefficients = np.zeros(2)
            state_representation = "energy_hs_v1"; time_quadrature = "fixture"; time_quadrature_metadata = {}
            status = "Solve_Succeeded"; lap_time = 1.; objective_value = 1.; iterations = 1; maximum_violation = 0.
        request = harness.build_attempt(harness.default_cases()["acceleration"], mesh_spacing_m=5., basis_spacing_m=35.,
            maximum_iterations=7, wall_cap_s=3., seed_name="centerline", formulation=Path("form.xml"),
            surface=Path("surface.npz"), seed_profile="energy-nlp")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); request_path = root / "request.json"; result_path = root / "result.json"
            request_path.write_text(json.dumps(harness._json_safe(request)), encoding="utf-8")
            with patch.object(harness, "make_track_view", return_value=object()), \
                 patch.object(harness, "corridor_from_track", return_value=object()), \
                 patch.object(harness, "compile_formulation", return_value=SimpleNamespace(model=object())), \
                 patch.object(harness, "point_mass_view_of_planar", return_value=object()), \
                 patch.object(harness, "load_surface_dispatch", return_value=(SimpleNamespace(provenance_hash="ok"), "fixture")), \
                 patch.object(harness, "capability_hash", return_value="ok"), \
                 patch.object(harness, "build_mesh", return_value=np.array([0., 1.])), \
                 patch.object(harness, "OffsetSpline", return_value=FakeSpline()), \
                 patch.object(harness, "initial_guess", return_value=np.zeros(7)), \
                 patch.object(harness, "_seed_domain_report", return_value={}), \
                 patch.object(harness, "build_profile_line_seed", return_value=(None, {"available": False, "reason": "fixture"})), \
                 patch.object(harness, "solve_spline_line", return_value=FakeSolution()), \
                 patch.object(harness, "validate_line", return_value={"checks": {"geometry_consistency": True, "dense_feasibility": True}, "model_feasible": True}), \
                 patch("tools.run.solve_minimum_time_line._write_trajectory_metadata", side_effect=lambda path, solution: np.savez(path, metadata=np.array("fixture"))):
                harness._attempt_worker(str(request_path), str(result_path), str(root / "solution.npz"))
            result = json.loads(result_path.read_text())
        self.assertFalse(result["profile_seed"]["used"])
        self.assertEqual(result["profile_seed"]["fallback"], "explicit-old-seed")
        self.assertEqual(result["profile_seed"]["diagnostics"]["reason"], "fixture")

    def test_worker_persists_dense_and_trajectory_metadata(self):
        class FakeSpline:
            control_count = 2
            control_distance = np.array([0.0, 1.0])
        class FakeSolution:
            distance = np.array([0.0, 1.0]); offset = np.array([0.0, 0.0])
            heading_error = np.array([0.0, 0.0]); curvature = np.array([0.0, 0.0])
            speed = np.array([10.0, 10.0]); acceleration = np.array([0.0, 0.0])
            midpoint_acceleration = np.array([0.0]); coefficients = np.array([0.0, 0.0])
            state_representation = "energy_hs_v1"; time_quadrature = "fixture_quadrature"
            time_quadrature_metadata = {"rule": "fixture"}; status = "Solve_Succeeded"
            lap_time = 1.0; objective_value = 1.0; iterations = 2; maximum_violation = 0.0
        request = harness.build_attempt(harness.default_cases()["acceleration"], mesh_spacing_m=5.0,
            basis_spacing_m=35.0, maximum_iterations=7, wall_cap_s=3.0, seed_name="centerline",
            formulation=Path("form.xml"), surface=Path("surface.npz"), dense_maximum_refinements=6)
        captured = {}
        def fake_validate(solution, corridor, surface, settings, **kwargs):
            captured["fractions"] = settings.off_collocation_fractions
            captured["refinements"] = kwargs["maximum_refinements"]
            return {"checks": {"geometry_consistency": True, "dense_feasibility": True},
                    "model_feasible": True}
        def fake_trajectory_metadata(path, solution):
            np.savez(path, metadata=np.array("fixture"))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); request_path = root / "request.json"; result_path = root / "result.json"
            request_path.write_text(json.dumps(harness._json_safe(request)), encoding="utf-8")
            with patch.object(harness, "make_track_view", return_value=object()), \
                 patch.object(harness, "corridor_from_track", return_value=object()), \
                 patch.object(harness, "compile_formulation", return_value=SimpleNamespace(model=object())), \
                 patch.object(harness, "point_mass_view_of_planar", return_value=object()), \
                 patch.object(harness, "load_surface_dispatch", return_value=(SimpleNamespace(provenance_hash="ok"), "fixture")), \
                 patch.object(harness, "capability_hash", return_value="ok"), \
                 patch.object(harness, "build_mesh", return_value=np.array([0.0, 1.0])), \
                 patch.object(harness, "OffsetSpline", return_value=FakeSpline()), \
                 patch.object(harness, "initial_guess", return_value=np.zeros(12)), \
                 patch.object(harness, "_seed_domain_report", return_value={}), \
                 patch.object(harness, "solve_spline_line", return_value=FakeSolution()), \
                 patch.object(harness, "validate_line", side_effect=fake_validate), \
                 patch("tools.run.solve_minimum_time_line._write_trajectory_metadata", side_effect=fake_trajectory_metadata):
                harness._attempt_worker(str(request_path), str(result_path), str(root / "solution.npz"))
            result = json.loads(result_path.read_text())
            self.assertEqual(result["status"], "Solve_Succeeded")
            self.assertEqual(result["validation_status"], "incomplete")
            self.assertEqual(captured["fractions"], harness.DEFAULT_OFF_COLLOCATION_FRACTIONS)
            self.assertEqual(captured["refinements"], 6)
            with np.load(root / "solution.npz", allow_pickle=False) as artifact:
                self.assertEqual(str(artifact["state_representation"]), "energy_hs_v1")
                self.assertEqual(str(artifact["time_quadrature"]), "fixture_quadrature")
                np.testing.assert_array_equal(artifact["control_station_m"], [0.0, 1.0])
            self.assertTrue((root / result["trajectory_metadata_artifact"]).is_file())
            self.assertIsNone(result["checks"]["matched_baseline"])
            self.assertIsNone(result["checks"]["replay_agreement"])
            self.assertFalse(result["reference"]["available"])

    def test_reference_failure_records_false_checks_without_losing_artifact(self):
        class FakeSpline:
            control_count = 2
            control_distance = np.array([0.0, 1.0])
        class FakeSolution:
            distance = np.array([0.0, 1.0]); offset = np.array([0.0, 0.0])
            heading_error = np.array([0.0, 0.0]); curvature = np.array([0.0, 0.0])
            speed = np.array([10.0, 10.0]); acceleration = np.array([0.0, 0.0])
            midpoint_acceleration = np.array([0.0]); coefficients = np.array([0.0, 0.0])
            state_representation = "energy_hs_v1"; time_quadrature = "fixture_quadrature"
            time_quadrature_metadata = {"rule": "fixture"}; status = "Solve_Succeeded"
            lap_time = 1.0; objective_value = 1.0; iterations = 2; maximum_violation = 0.0
        request = harness.build_attempt(harness.default_cases()["acceleration"], mesh_spacing_m=5.0,
            basis_spacing_m=35.0, maximum_iterations=7, wall_cap_s=3.0, seed_name="centerline",
            formulation=Path("form.xml"), surface=Path("surface.npz"), matched_reference=True)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); request_path = root / "request.json"; result_path = root / "result.json"
            request_path.write_text(json.dumps(harness._json_safe(request)), encoding="utf-8")
            with patch.object(harness, "make_track_view", return_value=object()), \
                 patch.object(harness, "corridor_from_track", return_value=object()), \
                 patch.object(harness, "compile_formulation", return_value=SimpleNamespace(model=object())), \
                 patch.object(harness, "point_mass_view_of_planar", return_value=object()), \
                 patch.object(harness, "load_surface_dispatch", return_value=(SimpleNamespace(provenance_hash="ok"), "fixture")), \
                 patch.object(harness, "capability_hash", return_value="ok"), \
                 patch.object(harness, "build_mesh", return_value=np.array([0.0, 1.0])), \
                 patch.object(harness, "OffsetSpline", return_value=FakeSpline()), \
                 patch.object(harness, "initial_guess", return_value=np.zeros(12)), \
                 patch.object(harness, "_seed_domain_report", return_value={}), \
                 patch.object(harness, "solve_spline_line", return_value=FakeSolution()), \
                 patch.object(harness, "validate_line", return_value={"checks": {"geometry_consistency": True, "dense_feasibility": True}, "model_feasible": True}), \
                 patch.object(harness, "_fixed_path_profile_evidence", return_value={"available": False, "reason": "fixture failure", "matched_baseline": False, "replay_agreement": False, "replay": {"available": False}, "empirical": False}), \
                 patch("tools.run.solve_minimum_time_line._write_trajectory_metadata", side_effect=lambda path, solution: np.savez(path, metadata=np.array("fixture"))):
                harness._attempt_worker(str(request_path), str(result_path), str(root / "solution.npz"))
            result = json.loads(result_path.read_text())
            self.assertTrue((root / "solution.npz").is_file())
            self.assertFalse(result["checks"]["matched_baseline"])
            self.assertFalse(result["checks"]["replay_agreement"])
            self.assertEqual(result["validation_status"], "failed")

    def test_profile_step_exception_preserves_other_profile_records(self):
        class Profile:
            converged = True; status = "converged"; iterations = 1; lap_time = 1.0; max_change = 0.0
        class Path:
            distance = np.array([0., 1.]); x_north = np.array([0., 1.]); y_east = np.array([0., 0.])
        class Trajectory:
            reference = SimpleNamespace(source_station=np.array([0., 1.]))
            spline = object(); coefficients = np.array([0.])
            def to_path(self, stations): return Path()
        solution = SimpleNamespace(trajectory=Trajectory(), state_evaluator=object())
        def profile(*args, **kwargs):
            if kwargs["step"] == .5: raise ValueError("fixture .5 failure")
            return Profile()
        with patch.object(harness, "OffsetTrajectory", return_value=Trajectory()), \
             patch.object(harness, "capability_profile", side_effect=profile):
            evidence = harness._fixed_path_profile_evidence(solution, harness.default_cases()["acceleration"], object())
        self.assertFalse(evidence["available"])
        self.assertEqual(len(evidence["candidate"]["profiles"]), 3)
        self.assertEqual(evidence["candidate"]["profiles"][0]["status"], "converged")
        self.assertEqual(evidence["candidate"]["profiles"][1]["status"], "exception")

    def test_json_serialization_converts_nonfinite_diagnostics_to_null(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / "result.json"
            harness._write_json_new_or_replace(destination, {"a": float("nan"), "b": np.inf}, initial=True)
            self.assertEqual(json.loads(destination.read_text()), {"a": None, "b": None})

    def test_exclusive_worker_output_refuses_existing_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "attempt"; output.mkdir()
            request = {"wall_cap_s": 0.1}
            with self.assertRaises(FileExistsError):
                harness.run_bounded_worker(request, output)

    def test_timeout_terminates_and_persists_failure(self):
        class Process:
            exitcode = None
            def __init__(self, *args, **kwargs):
                self.terminated = self.joined = False
                self.result_path = Path(kwargs["args"][1])
            def start(self):
                # Simulate a child that wrote a result immediately before it hung.
                self.result_path.write_text('{"status": "partial"}', encoding="utf-8")
            def join(self, *args): self.joined = True
            def is_alive(self): return not self.terminated
            def terminate(self): self.terminated = True
        class Context:
            def Process(self, *args, **kwargs): return Process(*args, **kwargs)
        created = []
        original = Context.Process
        def remember(self, *args, **kwargs):
            process = original(self, *args, **kwargs); created.append(process); return process
        Context.Process = remember
        with tempfile.TemporaryDirectory() as directory:
            request = {"wall_cap_s": 0.001, "case": {"name": "fixture"}}
            with patch.object(harness.mp, "get_context", return_value=Context()):
                result = harness.run_bounded_worker(request, Path(directory) / "attempt")
            self.assertTrue(created[0].terminated)
            self.assertTrue(created[0].joined)
            self.assertEqual(result["status"], "wall_timeout")
            self.assertFalse(result["accepted"])
            attempt = Path(directory) / "attempt"
            worker_evidence = json.loads((attempt / "result.json").read_text())
            timeout = json.loads((attempt / "timeout_result.json").read_text())
            self.assertEqual(worker_evidence["status"], "partial")
            self.assertEqual(timeout["error_type"], "TimeoutError")

    def test_worker_exit_without_result_is_persisted_as_failure(self):
        class Process:
            exitcode = 7
            def start(self): pass
            def join(self, *args): pass
            def is_alive(self): return False
            def terminate(self): raise AssertionError("must not terminate completed worker")
        class Context:
            def Process(self, *args, **kwargs): return Process()
        with tempfile.TemporaryDirectory() as directory:
            request = {"wall_cap_s": 1.0, "case": {"name": "fixture"}}
            with patch.object(harness.mp, "get_context", return_value=Context()):
                result = harness.run_bounded_worker(request, Path(directory) / "attempt")
            self.assertEqual(result["status"], "worker_no_result")
            self.assertFalse(result["solver_succeeded"])
            self.assertFalse(result["accepted"])

    def test_source_hashes_recursively_include_data_and_source_dependencies(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            xml, child = root / "track.xml", root / "child.xml"
            csv, binary = root / "track.csv", root / "surface.npz"
            csv.write_text("distance,x_north,y_east\n", encoding="utf-8")
            binary.write_bytes(b"fixture")
            child.write_text('<model><source ref="surface.npz" /></model>', encoding="utf-8")
            xml.write_text('<model><data ref="track.csv" /><source ref="child.xml" /></model>', encoding="utf-8")
            hashes = harness.source_hashes(xml)
            self.assertEqual(set(hashes), {str(xml.resolve()), str(child.resolve()),
                                           str(csv.resolve()), str(binary.resolve())})
            self.assertTrue(all(len(value) == 64 for value in hashes.values()))

    def test_source_hashes_reject_missing_recursive_dependency(self):
        with tempfile.TemporaryDirectory() as directory:
            xml = Path(directory) / "track.xml"
            xml.write_text('<model><source ref="missing.xml" /></model>', encoding="utf-8")
            with self.assertRaisesRegex(FileNotFoundError, "dependency"):
                harness.source_hashes(xml)

    def test_aggregate_separates_unqualified_candidates_from_refinement(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); case = root / "acceleration"; case.mkdir()
            for attempt, offset in (("mesh-5_basis-35_centerline", 0.0),
                                    ("mesh-2.5_basis-35_centerline", 0.1)):
                target = case / attempt; target.mkdir()
                np.savez(target / "solution.npz", distance=np.array([0., 1.]),
                         offset=np.array([offset, offset]), speed=np.array([10., 11.]),
                         curvature=np.array([0., 0.]))
            def row(mesh, artifact, *, seed="centerline", settings=None, hashes=None, dense=True):
                return {"request": {"case": {"name": "acceleration"}, "seed": seed,
                        "settings": settings if settings is not None else {"tolerance": 1e-6},
                        "source_hashes": hashes if hashes is not None else {"input": "a"},
                        "mesh_spacing_m": mesh, "basis_spacing_m": 35.0},
                        "source_hashes_after": hashes if hashes is not None else {"input": "a"},
                        "surface_representation": "signed_transient_normalized_v1",
                        "artifact": artifact, "objective_s": 5.0, "status": "Solve_Succeeded", "solver_succeeded": True,
                        "dense_original_model": {"checks": {"geometry_consistency": True,
                        "dense_feasibility": dense}, "model_feasible": dense}}
            first = row(5.0, "mesh-5_basis-35_centerline/solution.npz", dense=False)
            second = row(2.5, "mesh-2.5_basis-35_centerline/solution.npz", seed="low_amplitude_offset")
            report = harness.aggregate([first, second], root)
        self.assertEqual(len(report["candidate_diagnostics"]), 1)
        self.assertGreater(report["candidate_diagnostics"][0]["rms_path"], 0.0)
        self.assertIn("rms_speed", report["candidate_diagnostics"][0])
        self.assertIn("rms_curvature", report["candidate_diagnostics"][0])
        self.assertEqual(report["mesh_refinement"], [])
        self.assertEqual(report["basis_refinement"], [])
        self.assertFalse(report["strict_aggregate"]["accepted"])
        self.assertEqual(report["strict_aggregate"]["status"], "incomplete")

    def test_aggregate_qualifies_only_one_axis_with_matching_conditions(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); case = root / "acceleration"; case.mkdir()
            rows = []
            for attempt, mesh, basis in (("m5b35", 5.0, 35.0), ("m25b35", 2.5, 35.0),
                                         ("m5b175", 5.0, 17.5)):
                target = case / attempt; target.mkdir()
                np.savez(target / "solution.npz", distance=np.array([0., 1.]),
                         offset=np.array([mesh, basis]), speed=np.array([10., 11.]),
                         curvature=np.array([0., 0.]))
                rows.append({"request": {"case": {"name": "acceleration"}, "seed": "centerline",
                             "settings": {"tolerance": 1e-6}, "source_hashes": {"input": "same"},
                             "mesh_spacing_m": mesh, "basis_spacing_m": basis},
                              "actual_basis": {"control_count": 4 if basis == 35.0 else 6,
                                               "control_station_m": [0., 1.] if basis == 35.0 else [0., .5, 1.]},
                             "source_hashes_after": {"input": "same"},
                             "surface_representation": "signed_transient_normalized_v1",
                             "artifact": f"{attempt}/solution.npz", "objective_s": 5.0,
                             "status": "Solve_Succeeded", "solver_succeeded": True, "dense_original_model": {"checks": {
                             "geometry_consistency": True, "dense_feasibility": True}, "model_feasible": True}})
            report = harness.aggregate(rows, root)
            final_source_changed = harness.aggregate(rows, root, source_data_unchanged=False)
        self.assertEqual(len(report["mesh_refinement"]), 1)
        self.assertEqual(report["mesh_refinement"][0]["kind"], "mesh_refinement")
        self.assertEqual(len(report["basis_refinement"]), 1)
        self.assertEqual(report["basis_refinement"][0]["kind"], "basis_refinement")
        self.assertEqual(len(final_source_changed["candidate_diagnostics"]), 3)
        self.assertEqual(final_source_changed["mesh_refinement"], [])
        self.assertEqual(final_source_changed["basis_refinement"], [])

    def test_aggregate_requires_actual_reference_evidence_for_every_axis(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); case = root / "acceleration"; case.mkdir()
            rows = []
            for attempt, mesh, basis, grid in (("m5b35", 5.0, 35.0, [0., 1.]),
                                               ("m25b35", 2.5, 35.0, [0., 1.]),
                                               ("m5b175", 5.0, 17.5, [0., .5, 1.])):
                target = case / attempt; target.mkdir()
                np.savez(target / "solution.npz", distance=np.array([0., 1.]),
                         offset=np.zeros(2), speed=np.array([10., 10.]), curvature=np.zeros(2))
                rows.append({"attempt_id": f"acceleration/{attempt}", "request": {"case": {"name": "acceleration"},
                             "seed": "centerline", "settings": {"tolerance": 1e-6}, "source_hashes": {"input": "same"},
                             "mesh_spacing_m": mesh, "basis_spacing_m": basis}, "source_hashes_after": {"input": "same"},
                             "surface_representation": "signed_transient_normalized_v1", "artifact": f"{attempt}/solution.npz",
                             "objective_s": 5.0, "status": "Solve_Succeeded", "solver_succeeded": True, "actual_basis": {"control_station_m": grid},
                             "dense_original_model": {"checks": {"geometry_consistency": True, "dense_feasibility": True}, "model_feasible": True},
                             "checks": {"matched_baseline": True, "replay_agreement": True},
                             "reference": {"available": True, "empirical": False}})
            report = harness.aggregate(rows, root)
            rows[-1]["checks"]["replay_agreement"] = False
            incomplete = harness.aggregate(rows, root)
        self.assertFalse(report["strict_aggregate"]["accepted"])
        self.assertEqual(report["case_status"]["acceleration"]["status"], "incomplete")
        self.assertFalse(incomplete["strict_aggregate"]["accepted"])
        self.assertEqual(incomplete["case_status"]["acceleration"]["status"], "incomplete")
        # A passing mesh pair cannot borrow unrelated passing rows for a basis pair.
        rows[-1]["checks"]["replay_agreement"] = True
        rows[0]["checks"]["matched_baseline"] = False
        no_mesh = harness.aggregate(rows, root)
        self.assertFalse(no_mesh["strict_aggregate"]["accepted"])

    def test_aggregate_does_not_borrow_disjoint_edge_endpoints(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); case = root / "acceleration"; case.mkdir()
            rows = []
            # Mesh edge (a,b) and basis edge (a,c) have reference flags, but
            # their source/settings differ so no single axis edge is qualified.
            for attempt, mesh, basis, grid, settings in (("a", 5., 35., [0., 1.], {"tolerance": 1e-6}),
                                                         ("b", 2.5, 35., [0., 1.], {"tolerance": 1e-6}),
                                                         ("c", 5., 17.5, [0., .5, 1.], {"tolerance": 1e-8})):
                target = case / attempt; target.mkdir()
                np.savez(target / "solution.npz", distance=np.array([0., 1.]), offset=np.zeros(2),
                         speed=np.ones(2), curvature=np.zeros(2))
                rows.append({"attempt_id": attempt, "request": {"case": {"name": "acceleration"}, "seed": "centerline",
                             "settings": settings, "source_hashes": {"input": "same"}, "mesh_spacing_m": mesh, "basis_spacing_m": basis},
                             "source_hashes_after": {"input": "same"}, "surface_representation": "signed_transient_normalized_v1",
                             "artifact": f"{attempt}/solution.npz", "objective_s": 5., "status": "Solve_Succeeded", "solver_succeeded": True,
                             "actual_basis": {"control_station_m": grid}, "dense_original_model": {"checks": {"geometry_consistency": True, "dense_feasibility": True}, "model_feasible": True},
                             "checks": {"matched_baseline": True, "replay_agreement": True}, "reference": {"available": True, "empirical": False}})
            report = harness.aggregate(rows, root, expected_cases=("acceleration",),
                                       required_mesh_rungs=(5., 2.5), required_basis_rungs=(35., 17.5))
        self.assertFalse(report["strict_aggregate"]["accepted"])
        self.assertIsNone(report["case_status"]["acceleration"]["basis_evidence"])

    def test_dense_qualification_rejects_failed_solver_status_despite_true_flags(self):
        row = {"status": "Maximum_Iterations_Exceeded", "solver_succeeded": True,
               "artifact": "solution.npz", "dense_original_model": {"checks": {
               "geometry_consistency": True, "dense_feasibility": True}, "model_feasible": True}}
        self.assertFalse(harness._dense_live_qualified(row))
        row["status"] = "Solve_Succeeded"
        self.assertTrue(harness._dense_live_qualified(row))

    def test_aggregate_rejects_nominal_basis_change_with_identical_actual_grid(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); case = root / "acceleration"; case.mkdir()
            rows = []
            for attempt, basis in (("b35", 35.0), ("b175", 17.5)):
                target = case / attempt; target.mkdir()
                np.savez(target / "solution.npz", distance=np.array([0., 1.]),
                         offset=np.array([0., 0.]), speed=np.array([10., 11.]), curvature=np.array([0., 0.]))
                rows.append({"request": {"case": {"name": "acceleration"}, "seed": "centerline",
                             "settings": {"tolerance": 1e-6}, "source_hashes": {"input": "same"},
                             "mesh_spacing_m": 1.25, "basis_spacing_m": basis},
                             "actual_basis": {"control_count": 4, "control_station_m": [0., 25.1, 50.2, 75.3]},
                             "source_hashes_after": {"input": "same"},
                             "surface_representation": "signed_transient_normalized_v1",
                             "artifact": f"{attempt}/solution.npz", "objective_s": 5.0,
                             "status": "Solve_Succeeded", "solver_succeeded": True, "dense_original_model": {"checks": {
                             "geometry_consistency": True, "dense_feasibility": True}, "model_feasible": True}})
            report = harness.aggregate(rows, root)
        self.assertEqual(len(report["candidate_diagnostics"]), 1)
        self.assertEqual(report["basis_refinement"], [])

    def test_aggregate_rejects_in_attempt_source_mutation_but_keeps_candidates(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); case = root / "acceleration"; case.mkdir()
            rows = []
            for attempt, mesh in (("m5", 5.0), ("m25", 2.5)):
                target = case / attempt; target.mkdir()
                np.savez(target / "solution.npz", distance=np.array([0., 1.]),
                         offset=np.array([mesh, mesh]), speed=np.array([10., 11.]),
                         curvature=np.array([0., 0.]))
                rows.append({"request": {"case": {"name": "acceleration"}, "seed": "centerline",
                             "settings": {"tolerance": 1e-6}, "source_hashes": {"input": "same"},
                             "mesh_spacing_m": mesh, "basis_spacing_m": 35.0},
                             "source_hashes_after": {"input": "changed"} if mesh == 2.5 else {"input": "same"},
                             "surface_representation": "signed_transient_normalized_v1",
                             "artifact": f"{attempt}/solution.npz", "objective_s": 5.0,
                             "status": "Solve_Succeeded", "solver_succeeded": True, "dense_original_model": {"checks": {
                             "geometry_consistency": True, "dense_feasibility": True}, "model_feasible": True}})
            report = harness.aggregate(rows, root, source_data_unchanged=True)
        self.assertEqual(len(report["candidate_diagnostics"]), 1)
        self.assertEqual(report["mesh_refinement"], [])

    def test_run_matrix_rejects_source_mutation_between_attempts(self):
        case = harness.default_cases()["acceleration"]
        args = SimpleNamespace(output=None, formulation=Path("form.xml"), surface=Path("surface.npz"),
                               cases="acceleration", mesh_spacings=[5.0, 2.5], basis_spacings=[35.0],
                               seeds=["centerline"], maximum_iterations=1, wall_cap_s=1.0,
                               base_commit="fixture")
        snapshots = iter([{"input": "initial"}, {"input": "initial"}, {"input": "initial"},
                          {"input": "changed"}, {"input": "changed"}, {"input": "changed"}])
        captured_requests = []
        def fake_worker(request, output):
            captured_requests.append(request)
            return {"request": request, "artifact": None, "solver_succeeded": False}
        with tempfile.TemporaryDirectory() as directory:
            args.output = Path(directory) / "matrix"
            with patch.object(harness, "default_cases", return_value={"acceleration": case}), \
                 patch.object(harness, "source_hashes", side_effect=lambda *paths: next(snapshots)), \
                 patch.object(harness, "load_surface_dispatch", return_value=(None, "fixture")), \
                 patch.object(harness, "run_bounded_worker", side_effect=fake_worker):
                manifest = harness.run_matrix(args)
        self.assertEqual([request["source_hashes"] for request in captured_requests],
                         [{"input": "initial"}, {"input": "changed"}])
        self.assertTrue(manifest["results"][0]["source_data_unchanged"])
        self.assertTrue(manifest["results"][1]["source_data_unchanged"])
        self.assertFalse(manifest["source_data_unchanged"])
        self.assertEqual(manifest["aggregate"]["mesh_refinement"], [])

    def test_mocked_solver_status_failure_cannot_be_accepted(self):
        # This exercises the only status-to-acceptance contract used by the worker.
        strict = harness.strict_validation(False, {name: True for name in harness.REQUIRED_NUMERICAL_CHECKS})
        self.assertEqual(strict["validation_status"], "solver_failed")
        self.assertFalse(strict["accepted"])


if __name__ == "__main__":
    unittest.main()
