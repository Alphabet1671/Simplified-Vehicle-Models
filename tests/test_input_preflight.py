"""Worker input admission must fail before track/model/solver work."""
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest import mock


class InputPreflightTests(unittest.TestCase):
    def test_missing_public_track_is_typed_setup_failure_before_loader(self):
        from tools.run import solve_minimum_time_line as runner

        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            request_path = root / "request.json"
            request_path.write_text("{}")
            request = {"track": str(root / "missing-track.xml"),
                       "formulation": str(root / "formulation.xml"),
                       "surface": str(root / "chart.npz"), "output": str(root / "output"),
                       "margin": .1, "half_width": 2., "spacings": [5.],
                       "offset_spacing": 10., "path_smoothing": 0., "wall_cap_s": 2.}
            (root / "formulation.xml").write_text("<formulation/>")
            (root / "chart.npz").write_bytes(b"placeholder")
            with mock.patch.object(runner, "_validate_output_paths"), \
                 mock.patch.object(runner, "load_track", side_effect=AssertionError("track loader reached")), \
                 mock.patch.object(runner, "_public_spline_worker_request", side_effect=AssertionError("setup reached")):
                runner._prepare_public_spline_request_in_worker(
                    request, str(request_path), str(root / "result.json"), str(root / "solution.npz"))
            import json
            result = json.loads((root / "result.json").read_text())
            self.assertEqual(result["status"], "missing_input")
            self.assertEqual(result["failure_stage"], "setup")
            self.assertFalse(result["solver_attempted"])
            self.assertEqual(result["role"], "track")
            self.assertEqual(Path(result["path"]), Path(__import__("os").path.abspath(root / "missing-track.xml")))
            self.assertEqual(result["cause"], "not_found")
            self.assertFalse(result["accepted"])

    def test_strict_worker_missing_chart_is_typed_and_skips_track_model_and_solver(self):
        from tools.diagnostics import verify_no_tv_minimum_lap as worker
        import json
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            track = root / "track.xml"
            formulation = root / "formulation.xml"
            track.write_text("<track/>")
            formulation.write_text("<formulation/>")
            request = {"case": {"track": str(track), "name": "fixture", "topology": "closed",
                                "start_treatment": "closed_flying", "start_speed_mps": 0.0,
                                "fallback_half_width_m": None},
                       "formulation": str(formulation), "surface": str(root / "missing-chart.npz"),
                       "stage_budgets_s": {}, "settings": {},
                       "source_geometry_contract": "native_topology_reference_v1"}
            request_path, result_path = root / "request.json", root / "result.json"
            request_path.write_text(json.dumps(request))
            with mock.patch.object(worker, "_validate_request_options", wraps=worker._validate_request_options) as validate, \
                 mock.patch.object(worker, "make_track_view", side_effect=AssertionError("track load reached")):
                worker._attempt_worker(str(request_path), str(result_path), str(root / "solution.npz"))
            result = json.loads(result_path.read_text())
            validate.assert_called_once()
            self.assertEqual(result["status"], "missing_input")
            self.assertEqual(result["validation_status"], "setup_unavailable")
            self.assertFalse(result["solver_attempted"])
            self.assertEqual(result["role"], "chart")
            self.assertEqual(result["cause"], "not_found")
            self.assertFalse(result["accepted"])

    def test_preflight_diagnoses_wrong_kind_without_opening_asset(self):
        from simplified_vehicle_models.solvers.input_preflight import preflight_inputs
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            directory = root / "chart.npz"
            directory.mkdir()
            result = preflight_inputs([("chart", directory)])
            self.assertEqual(result["status"], "setup_unavailable")
            self.assertEqual(result["cause"], "wrong_kind")
            self.assertEqual(result["role"], "chart")
            self.assertEqual(result["path"], __import__("os").path.abspath(directory))
