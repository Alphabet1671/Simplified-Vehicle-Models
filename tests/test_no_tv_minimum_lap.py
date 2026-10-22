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
            saved = json.loads((Path(directory) / "attempt" / "result.json").read_text())
            self.assertEqual(saved["error_type"], "TimeoutError")

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

    def test_source_hashes_include_xml_and_referenced_assets(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            xml = root / "track.xml"; csv = root / "track.csv"
            csv.write_text("distance,x_north,y_east\n", encoding="utf-8")
            xml.write_text('<model><data ref="track.csv" /></model>', encoding="utf-8")
            hashes = harness.source_hashes(xml)
            self.assertEqual(set(hashes), {str(xml.resolve()), str(csv.resolve())})
            self.assertTrue(all(len(value) == 64 for value in hashes.values()))

    def test_aggregate_compares_completed_artifacts_and_stays_incomplete(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); case = root / "acceleration"; case.mkdir()
            for attempt, offset in (("mesh-5_basis-35_centerline", 0.0),
                                    ("mesh-2.5_basis-35_centerline", 0.1)):
                target = case / attempt; target.mkdir()
                np.savez(target / "solution.npz", distance=np.array([0., 1.]),
                         offset=np.array([offset, offset]), speed=np.array([10., 11.]),
                         curvature=np.array([0., 0.]))
            def row(mesh, artifact):
                return {"request": {"case": {"name": "acceleration"},
                        "mesh_spacing_m": mesh, "basis_spacing_m": 35.0},
                        "artifact": artifact, "objective_s": 5.0}
            report = harness.aggregate([row(5.0, "mesh-5_basis-35_centerline/solution.npz"),
                                        row(2.5, "mesh-2.5_basis-35_centerline/solution.npz")], root)
        self.assertEqual(len(report["comparisons"]), 1)
        self.assertGreater(report["comparisons"][0]["rms_path"], 0.0)
        self.assertIn("rms_speed", report["comparisons"][0])
        self.assertIn("rms_curvature", report["comparisons"][0])
        self.assertFalse(report["strict_aggregate"]["accepted"])
        self.assertEqual(report["strict_aggregate"]["status"], "incomplete")

    def test_mocked_solver_status_failure_cannot_be_accepted(self):
        # This exercises the only status-to-acceptance contract used by the worker.
        strict = harness.strict_validation(False, {name: True for name in harness.REQUIRED_NUMERICAL_CHECKS})
        self.assertEqual(strict["validation_status"], "solver_failed")
        self.assertFalse(strict["accepted"])


if __name__ == "__main__":
    unittest.main()
