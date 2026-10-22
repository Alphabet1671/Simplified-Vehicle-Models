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
                        "surface_representation": "signed_transient_normalized_v1",
                        "artifact": artifact, "objective_s": 5.0, "solver_succeeded": True,
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
                             "surface_representation": "signed_transient_normalized_v1",
                             "artifact": f"{attempt}/solution.npz", "objective_s": 5.0,
                             "solver_succeeded": True, "dense_original_model": {"checks": {
                             "geometry_consistency": True, "dense_feasibility": True}, "model_feasible": True}})
            report = harness.aggregate(rows, root)
        self.assertEqual(len(report["mesh_refinement"]), 1)
        self.assertEqual(report["mesh_refinement"][0]["kind"], "mesh_refinement")
        self.assertEqual(len(report["basis_refinement"]), 1)
        self.assertEqual(report["basis_refinement"][0]["kind"], "basis_refinement")

    def test_mocked_solver_status_failure_cannot_be_accepted(self):
        # This exercises the only status-to-acceptance contract used by the worker.
        strict = harness.strict_validation(False, {name: True for name in harness.REQUIRED_NUMERICAL_CHECKS})
        self.assertEqual(strict["validation_status"], "solver_failed")
        self.assertFalse(strict["accepted"])


if __name__ == "__main__":
    unittest.main()
