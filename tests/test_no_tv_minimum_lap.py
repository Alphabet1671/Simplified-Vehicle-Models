import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from tools.diagnostics import verify_no_tv_minimum_lap as harness
from simplified_vehicle_models.solvers import input_preflight


class NoTvMinimumLapHarnessTest(unittest.TestCase):
    def test_structured_numerical_options_roundtrip_and_validation(self):
        case = harness.default_cases()["acceleration"]
        base = dict(mesh_spacing_m=5.0, basis_spacing_m=35.0, maximum_iterations=1,
                    wall_cap_s=3.0, seed_name="centerline", formulation=Path("form.xml"),
                    surface=Path("surface.npz"))
        legacy = harness.build_attempt(case, **base)
        self.assertNotIn("track_problem_contract", legacy)
        contract = {"schema": "track_problem_v1", "topology": "open", "source_stations": [0., 1.],
                    "lower_offset": [-1., -1.], "upper_offset": [1., 1.],
                    "boundary": {"start_mode": "rolling", "start_speed_mps": 10., "finish_mode": "free"}}
        explicit = harness.build_attempt(case, **base, track_problem_contract=contract)
        self.assertEqual(explicit["track_problem_contract"]["boundary"]["start_speed_mps"], 10.)
        self.assertEqual(explicit["case"]["start_speed_mps"], 10.)
        explicit_speed = dict(contract, boundary={"start_mode": "rolling", "start_speed_mps": 3., "finish_mode": "free"})
        derived = harness.build_attempt(case, **base, track_problem_contract=explicit_speed)
        self.assertEqual(derived["case"]["start_speed_mps"], 3.)
        self.assertEqual(derived["settings"]["start_speed"], 3.)
        from tools.run.solve_minimum_time_line import build_shared_spline_ladder_request
        with patch.object(harness, "source_hashes", return_value={"input": "hash"}), \
             patch("tools.diagnostics.no_tv_provenance.runtime_provenance", return_value={"runtime": {"python": "fixture"}}):
            ladder = build_shared_spline_ladder_request(case, source=case.track, formulation=Path("form.xml"),
                surface=Path("surface.npz"), spacings=(5., 2.5), margin=.1, half_width=case.fallback_half_width_m,
                basis_spacing=35., wall_cap_s=3., track_problem_contract=explicit_speed)
        self.assertEqual(ladder["track_problem_contract"], ladder["rung_requests"][0]["track_problem_contract"])
        self.assertEqual(ladder["case"]["start_speed_mps"], 3.)
        harness._validate_request_options(ladder)
        drifted = json.loads(json.dumps(ladder))
        drifted["track_problem_contract"]["boundary"]["start_speed_mps"] = 4.
        with self.assertRaisesRegex(ValueError, "outer and rung"):
            harness._validate_request_options(drifted)
        closed_case = harness.default_cases()["mis_closed"]
        closed_contract = {"schema": "track_problem_v1", "topology": "closed", "source_stations": [0., 1.],
            "lower_offset": [-1., -1.], "upper_offset": [1., 1.],
            "boundary": {"start_mode": "flying", "start_speed_mps": 3., "finish_mode": "periodic"}}
        closed_request = harness.build_attempt(closed_case, **base, track_problem_contract=closed_contract)
        self.assertEqual(closed_request["case"]["start_speed_mps"], 0.)
        self.assertEqual(closed_request["settings"]["start_speed"], 0.)
        wire = json.loads(json.dumps(harness._json_safe(explicit), allow_nan=False))
        self.assertEqual(wire["track_problem_contract"], explicit["track_problem_contract"])
        self.assertEqual(legacy["energy_defect_scaling"], "legacy_reference_speed_v1")
        self.assertEqual(legacy["coefficient_coordinate_scale"], 1.0)
        options = dict(energy_defect_scaling="cell_length_v1", coefficient_coordinate_scale=0.1,
                       mesh_stations=[0.0, 0.5, 2.0], constraint_stations=[0.25, 1.25])
        request = harness.build_attempt(case, **base, **options, diagnostic_rungs=True)
        wire = json.loads(json.dumps(harness._json_safe(request), allow_nan=False))
        self.assertEqual(wire["energy_defect_scaling"], options["energy_defect_scaling"])
        self.assertEqual(wire["coefficient_coordinate_scale"], options["coefficient_coordinate_scale"])
        self.assertEqual(wire["mesh_stations"], options["mesh_stations"])
        self.assertEqual(wire["constraint_stations"], options["constraint_stations"])
        canonical = harness.build_attempt(case, **base, mesh_stations=[0., .5, 2.],
                                           constraint_stations=[1.25, .25, .25], diagnostic_rungs=True)
        self.assertEqual(canonical["constraint_stations"], [.25, 1.25])
        self.assertEqual(harness.build_attempt(case, **base, **dict(options, mesh_stations=[0., 2., 5.]), diagnostic_rungs=True)["mesh_rung_status"], "diagnostic_not_preregistered")
        with tempfile.TemporaryDirectory() as directory:
            request_file = Path(directory) / "request.json"
            request_file.write_text(json.dumps({"case": "acceleration", "mesh_spacing_m": 5.,
                "basis_spacing_m": 35., "maximum_iterations": 1, "wall_cap_s": 3.,
                "seed_name": "centerline", **options}), encoding="utf-8")
            with patch.object(harness, "runtime_provenance", return_value={"git_head": "fixture"}), \
                 patch.object(harness, "source_hashes", return_value={"fixture": "hash"}):
                loaded = harness._load_structured_request(request_file, formulation=Path("form.xml"),
                    surface=Path("surface.npz"))
            self.assertEqual(loaded["energy_defect_scaling"], "cell_length_v1")
            request_file.write_text(json.dumps({"case": "acceleration", "mesh_spacing_m": 5.,
                "basis_spacing_m": 35., "maximum_iterations": 1, "wall_cap_s": 3.,
                "seed_name": "centerline", "track_problem_contract": contract}), encoding="utf-8")
            with patch.object(harness, "runtime_provenance", return_value={"git_head": "fixture"}), \
                 patch.object(harness, "source_hashes", return_value={"fixture": "hash"}):
                loaded_contract = harness._load_structured_request(request_file, formulation=Path("form.xml"),
                    surface=Path("surface.npz"))
            self.assertEqual(loaded_contract["track_problem_contract"]["schema"], "track_problem_v1")
            legacy_request = dict(loaded)
            for key in ("request_schema_version", "energy_defect_scaling", "coefficient_coordinate_scale",
                        "mesh_stations", "constraint_stations"):
                legacy_request.pop(key, None)
            self.assertEqual(harness._request_policy_identity(legacy_request)["energy_defect_scaling"],
                             "legacy_reference_speed_v1")
            altered = dict(legacy_request, energy_defect_scaling="cell_length_v1")
            self.assertNotEqual(harness._request_policy_identity(legacy_request),
                                harness._request_policy_identity(altered))
            request_file.write_text('{"case":"acceleration","unknown":1}', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unknown keys"):
                with patch.object(harness, "runtime_provenance", return_value={"git_head": "fixture"}), \
                     patch.object(harness, "source_hashes", return_value={"fixture": "hash"}):
                    harness._load_structured_request(request_file, formulation=Path("form.xml"),
                        surface=Path("surface.npz"))
        for bad in (float("nan"), True, 0.0):
            with self.assertRaisesRegex(ValueError, "coefficient_coordinate_scale"):
                harness.build_attempt(case, **base, coefficient_coordinate_scale=bad)
        for bad_mesh in ([0., 1., 1., 2.], [0., 2., 1.], [0., float("nan"), 2.]):
            with self.assertRaisesRegex(ValueError, "mesh_stations"):
                harness.build_attempt(case, **base, mesh_stations=bad_mesh, diagnostic_rungs=True)
        offset_mesh = harness.build_attempt(case, **base, mesh_stations=[10., 11., 20.],
            constraint_stations=[15.], diagnostic_rungs=True)
        self.assertEqual(offset_mesh["mesh_stations"], [10., 11., 20.])
        with self.assertRaisesRegex(ValueError, "constraint_stations"):
            harness.build_attempt(case, **base, mesh_stations=[0., 1., 2.],
                                  constraint_stations=[2., 3.], diagnostic_rungs=True)
        with self.assertRaisesRegex(ValueError, "coefficient_coordinate_scale"):
            harness.build_attempt(case, **base, coefficient_coordinate_scale=float("nan"))

    def test_worker_raw_request_options_rejected_before_setup(self):
        base = harness.build_attempt(harness.default_cases()["acceleration"], mesh_spacing_m=5.,
            basis_spacing_m=35., maximum_iterations=1, wall_cap_s=3., seed_name="centerline",
            formulation=Path("form.xml"), surface=Path("surface.npz"))
        harness._validate_request_options(base)
        v1 = dict(base)
        for key in ("request_schema_version", "schema_version", "energy_defect_scaling",
                    "coefficient_coordinate_scale", "mesh_stations", "constraint_stations"):
            v1.pop(key, None)
        harness._validate_request_options(v1)
        v1["schema_version"] = 1
        harness._validate_request_options(v1)
        v2 = dict(base, schema_version=2)
        harness._validate_request_options(v2)
        mutations = [
            ({"request_schema_version": 1, "schema_version": True}, "schema_version"),
            ({"request_schema_version": 1, "schema_version": 1.0}, "schema_version"),
            ({"request_schema_version": 1, "schema_version": 9}, "schema_version"),
            ({"request_schema_version": 1, "schema_version": 2}, "conflicts"),
            ({"request_schema_version": True}, "schema"),
            ({"request_schema_version": 1, "schema_version": 1, "mesh_stations": [0., 1.]}, "schema v1"),
            ({"mesh_stations": [0., 2., 1.]}, "strictly increasing"),
            ({"mesh_stations": [0., float("nan"), 2.]}, "finite"),
            ({"constraint_stations": [float("nan")]}, "finite"),
            ({"coefficient_coordinate_scale": True}, "coefficient_coordinate_scale"),
            ({"coefficient_coordinate_scale": float("nan")}, "coefficient_coordinate_scale"),
            ({"energy_defect_scaling": "bad"}, "energy_defect_scaling"),
            ({"no_torque_vectoring": False}, "no_torque_vectoring"),
        ]
        for mutation, message in mutations:
            request = dict(base); request.update(mutation)
            with self.subTest(mutation=mutation), self.assertRaisesRegex(ValueError, message):
                harness._validate_request_options(request)
        harness._validate_request_options(dict(base, mesh_stations=[10., 11., 20.],
                                                constraint_stations=[15.]))
        contract = {"schema": "track_problem_v1", "topology": "open", "source_stations": [0., 1.],
                    "lower_offset": [-1., -1.], "upper_offset": [1., 1.],
                    "boundary": {"start_mode": "rolling", "start_speed_mps": 10., "finish_mode": "free"}}
        explicit = dict(base, track_problem_contract=contract)
        harness._validate_request_options(explicit)
        with self.assertRaisesRegex(ValueError, "invalid track_problem_contract"):
            harness._validate_request_options(dict(base, track_problem_contract=dict(contract, extra=True)))
        with tempfile.TemporaryDirectory() as directory:
            request_path = Path(directory) / "request.json"
            result_path = Path(directory) / "result.json"
            raw = dict(base, track_problem_contract=dict(contract, boundary={"start_mode":"standing", "start_speed_mps":0., "finish_mode":"free"}))
            request_path.write_text(json.dumps(harness._json_safe(raw)), encoding="utf-8")
            with patch("simplified_vehicle_models.solvers.input_preflight.preflight_inputs", side_effect=AssertionError("pure request gate must precede filesystem")):
                harness._attempt_worker(str(request_path), str(result_path), str(Path(directory) / "solution.npz"))
            result = json.loads(result_path.read_text())
            self.assertEqual(result["status"], "track_problem_unsupported")
            self.assertFalse(result["solver_attempted"])
            self.assertEqual(result.get("validation_status"), "not_run")
        with tempfile.TemporaryDirectory() as directory:
            request_path = Path(directory) / "request.json"
            result_path = Path(directory) / "result.json"
            raw = dict(base, track_problem_contract=dict(contract, extra=True))
            request_path.write_text(json.dumps(harness._json_safe(raw)), encoding="utf-8")
            with (patch("simplified_vehicle_models.solvers.input_preflight.preflight_inputs", side_effect=AssertionError("must not check filesystem")) as preflight,
                  patch.object(harness, "make_track_view", side_effect=AssertionError("must not load geometry")) as make_track,
                  patch.object(harness, "solve_spline_line", side_effect=AssertionError("must not solve")) as solve):
                harness._attempt_worker(str(request_path), str(result_path), str(Path(directory) / "solution.npz"))
            result = json.loads(result_path.read_text())
            self.assertEqual(result["status"], "track_problem_invalid")
            self.assertEqual(result["validation_status"], "not_run")
            self.assertFalse(result["solver_attempted"])
            self.assertFalse(result["solver_succeeded"])
            self.assertEqual(result["track_problem"]["status"], "invalid")
            self.assertTrue(result["track_problem"]["reasons"])
            self.assertIn("invalid track_problem_contract", result["error"])
            self.assertEqual((preflight.call_count, make_track.call_count, solve.call_count), (0, 0, 0))
        with tempfile.TemporaryDirectory() as directory:
            request_path = Path(directory) / "request.json"
            result_path = Path(directory) / "result.json"
            raw = dict(base, request_schema_version=True)
            request_path.write_text(json.dumps(harness._json_safe(raw)), encoding="utf-8")
            with patch.object(harness, "make_track_view", side_effect=AssertionError("setup must not run")):
                harness._attempt_worker(str(request_path), str(result_path), str(Path(directory) / "solution.npz"))
            result = json.loads(result_path.read_text())
            self.assertIn("unsupported request_schema_version", result["error"])
            self.assertNotIn("requested_numerical_options", result)

    def test_loaded_contract_identity_requires_matching_nonnull_hash(self):
        left = {"request": {"track_problem_contract": {"schema": "track_problem_v1"}},
                "track_problem": {"contract_hash": None}}
        right = {"request": {"track_problem_contract": {"schema": "track_problem_v1"}},
                 "track_problem": {"contract_hash": None}}
        self.assertFalse(harness._same_refinement_conditions(left, right))

    def test_preregistered_cases_keep_full_geometry_and_explicit_starts(self):
        cases = harness.default_cases()
        self.assertEqual(cases["mis_closed"].track, cases["mis_cut_open"].track)
        self.assertEqual(cases["fsc_closed"].track, cases["fsc_cut_open"].track)
        self.assertEqual(cases["mis_closed"].start_treatment, "closed_flying")
        self.assertEqual(cases["mis_cut_open"].start_treatment, "open_rolling_10")
        self.assertEqual(cases["acceleration"].start_treatment, "open_rolling_10")
        self.assertIn("NOTstandingstart", cases["acceleration"].candidate_only_reason)
        self.assertEqual(harness.build_attempt(cases["mis_cut_open"], mesh_spacing_m=5.0,
            basis_spacing_m=35.0, maximum_iterations=1, wall_cap_s=3.0, seed_name="centerline",
            formulation=Path("form.xml"), surface=Path("surface.npz"))["source_geometry_contract"],
            harness.SOURCE_GEOMETRY_CONTRACT)
        self.assertEqual(harness.build_attempt(cases["mis_closed"], mesh_spacing_m=5.0,
            basis_spacing_m=35.0, maximum_iterations=1, wall_cap_s=3.0, seed_name="centerline",
            formulation=Path("form.xml"), surface=Path("surface.npz"))["source_geometry_contract"],
            harness.NATIVE_GEOMETRY_CONTRACT)

    def test_full_cut_open_view_preserves_mis_and_fsc_source_geometry(self):
        cases = harness.default_cases()
        from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve
        for closed_name, open_name in (("mis_closed", "mis_cut_open"),
                                       ("fsc_closed", "fsc_cut_open")):
            with self.subTest(track=open_name):
                closed = harness.make_track_view(cases[closed_name])
                opened = harness.make_track_view(cases[open_name])
                self.assertTrue(closed.is_closed)
                self.assertFalse(opened.is_closed)
                np.testing.assert_array_equal(closed.distance, opened.distance)
                np.testing.assert_array_equal(closed.curvature, opened.curvature)
                np.testing.assert_array_equal(closed.half_width_left, opened.half_width_left)
                np.testing.assert_array_equal(closed.half_width_right, opened.half_width_right)
                self.assertTrue(opened.source_reference_interpolation_periodic)
                closed_curve, open_curve = ReferenceCurve(closed), ReferenceCurve(opened)
                dense = np.linspace(0.0, closed.lap_length, 1001)
                for name in ("north", "east", "north_s", "east_s", "north_ss", "east_ss",
                             "north_sss", "east_sss", "metric", "metric_s", "curvature", "curvature_s"):
                    np.testing.assert_allclose(getattr(open_curve.evaluate(dense), name),
                                               getattr(closed_curve.evaluate(dense), name), rtol=0.0, atol=1e-12)
                with self.assertRaisesRegex(ValueError, "outside the open interval"):
                    open_curve.evaluate(-1e-6)
                with self.assertRaisesRegex(ValueError, "outside the open interval"):
                    open_curve.evaluate(opened.lap_length + 1e-6)

    def test_worker_rejects_missing_or_mismatched_cut_geometry_contract_before_setup(self):
        case = harness.default_cases()["fsc_cut_open"]
        base = harness.build_attempt(case, mesh_spacing_m=5.0, basis_spacing_m=35.0,
            maximum_iterations=1, wall_cap_s=3.0, seed_name="centerline",
            formulation=Path("form.xml"), surface=Path("surface.npz"))
        for contract in (None, harness.NATIVE_GEOMETRY_CONTRACT, "unknown_contract"):
            request = dict(base)
            if contract is None:
                request.pop("source_geometry_contract")
            else:
                request["source_geometry_contract"] = contract
            with tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                request_path, result_path = root / "request.json", root / "result.json"
                request_path.write_text(json.dumps(harness._json_safe(request)), encoding="utf-8")
                with patch.object(harness, "make_track_view", side_effect=AssertionError("geometry setup must not run")), \
                     patch.object(harness, "compile_formulation", side_effect=AssertionError("compile must not run")):
                    harness._attempt_worker(str(request_path), str(result_path), str(root / "solution.npz"))
                result = json.loads(result_path.read_text())
            self.assertEqual(result["status"], "geometry_contract_mismatch")
            self.assertFalse(result["accepted"])
            self.assertEqual(result["validation_status"], "solver_failed")
            self.assertEqual(result["error_type"], "GeometryContractMismatch")
            self.assertNotIn("artifact", result)

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

    def test_profile_control_projection_is_opt_in_and_part_of_identity(self):
        case = harness.default_cases()["acceleration"]
        base = dict(mesh_spacing_m=5.0, basis_spacing_m=35.0, maximum_iterations=7,
                    wall_cap_s=3.0, seed_name="centerline", formulation=Path("form.xml"),
                    surface=Path("surface.npz"), seed_profile="energy-nlp")
        direct = harness.build_attempt(case, **base)
        bounded = harness.build_attempt(case, **base, profile_control_projection="bounded_energy_l1")
        self.assertEqual(direct["profile_control_projection"], "direct_gradient")
        self.assertEqual(direct["initializer"]["profile_control_projection"], "direct_gradient")
        self.assertEqual(bounded["initializer"]["profile_control_projection"], "bounded_energy_l1")
        left = {"request": direct, "actual_initializer": {"stable_identity":
                harness._initializer_stable_identity(direct, {"profile_seed": {"used": False}})}}
        right = {"request": bounded, "actual_initializer": {"stable_identity":
                 harness._initializer_stable_identity(bounded, {"profile_seed": {"used": False}})}}
        self.assertFalse(harness._same_refinement_conditions(left, right))

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

    def test_explicit_contract_worker_fresh_manifest_roundtrip(self):
        import csv
        from dataclasses import replace
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
        from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface, save_capability_surface
        from simplified_vehicle_models.solvers.run_manifest import load_run_manifest_v1
        from simplified_vehicle_models.solvers.track_problem_contract import validate_problem_contract
        from simplified_vehicle_models.tracks.profile import load_track
        from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve, OffsetTrajectory
        from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
        from simplified_vehicle_models.solvers.qss_spline_ocp import (
            SplineSolution, _SplineStateEvaluator, TIME_QUADRATURE_NAME, _time_quadrature_metadata,
        )
        from simplified_vehicle_models.formulations.point_mass_capability import capability_hash
        from tools.diagnostics.no_tv_provenance import runtime_provenance
        from tools.diagnostics.verify_no_tv_minimum_lap import source_hashes

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            geometry = root / "geometry.csv"
            with geometry.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(("distance", "x_north", "y_east", "heading_rad", "curvature"))
                for station in range(11):
                    writer.writerow((station, station, 0., 0., 0.))
            track_xml = root / "straight.xml"
            track_xml.write_text('''<model schema-version="1.0" id="fresh-straight" type="track-model" subtype="open-xy-profile">
  <metadata><title>Fresh 10m open straight</title></metadata><data ref="geometry.csv"/>
  <parameters><parameter name="is_closed" value="false"/><parameter name="terminal_segment_is_duplicate" value="false"/><parameter name="maximum_radius" value="100000"/></parameters>
</model>''', encoding="utf-8")
            formulation = Path("models/formulations/iem24_murphy_real_planar_7dof.xml").resolve()
            vehicle = point_mass_view_of_planar(compile_formulation(formulation).model)
            surface = root / "fresh-capability.npz"
            chart_speed = np.array([0., 5., 10., 20., 30.])
            chart_curvature = np.array([-.2, -.1, 0., .1, .2])
            save_capability_surface(CapabilitySurface(
                chart_speed, chart_curvature, np.full((5, 5), 4.), np.full((5, 5), -4.),
                np.full(5, 30.), capability_hash(vehicle)), surface)
            case = replace(harness.default_cases()["acceleration"], track=track_xml,
                           topology="open", start_speed_mps=10., start_treatment="open_rolling_10",
                           fallback_half_width_m=2.)
            track = harness.make_track_view(case)
            reference = ReferenceCurve(track)
            corridor = harness.corridor_from_track(track, margin=.1, half_width=2.)
            contract = {"schema": "track_problem_v1", "topology": "open",
                "source_stations": reference.source_station.tolist(),
                "lower_offset": np.asarray(corridor.minimum, dtype=float).tolist(),
                "upper_offset": np.asarray(corridor.maximum, dtype=float).tolist(),
                "boundary": {"start_mode": "rolling", "start_speed_mps": 10., "finish_mode": "free"}}
            request = harness.build_attempt(case, mesh_spacing_m=5., basis_spacing_m=35., maximum_iterations=1,
                wall_cap_s=3., seed_name="centerline", formulation=formulation, surface=surface,
                diagnostic_rungs=True, track_problem_contract=contract)
            request["source_hashes"] = source_hashes(case.track, formulation, surface)
            request["provenance"] = runtime_provenance(Path.cwd())

            spline = OffsetSpline(track, 35.)
            coefficients = np.zeros(spline.control_count)
            trajectory = OffsetTrajectory(reference, spline, coefficients)
            distance = np.array([0., 5., 10.])
            speed = np.full(3, 10.)
            acceleration = np.zeros(3)
            midpoint = np.zeros(2)
            evaluator = _SplineStateEvaluator(trajectory, distance, speed, acceleration, midpoint)
            solution = SplineSolution(distance=distance, offset=np.zeros(3), heading_error=np.zeros(3),
                curvature=np.zeros(3), speed=speed, acceleration=acceleration, lap_time=1.,
                status="Solve_Succeeded", iterations=0, maximum_violation=0., coefficients=coefficients,
                trajectory=trajectory, midpoint_acceleration=midpoint, objective_value=1.,
                state_evaluator=evaluator, control_representation="quadratic_acceleration_v1",
                time_quadrature=TIME_QUADRATURE_NAME, time_quadrature_metadata=_time_quadrature_metadata())
            request_path, result_path = root / "request.json", root / "result.json"
            artifact_path = root / "solution.npz"
            request_path.write_text(json.dumps(harness._json_safe(request), allow_nan=False), encoding="utf-8")
            from simplified_vehicle_models.tracks.geometry_audit import audit_track_geometry as original_audit
            audit_seen = []
            def audit_spy(track_arg, reference_arg, corridor_arg, **kwargs):
                audit_seen.append(reference_arg)
                return original_audit(track_arg, reference_arg, corridor_arg, **kwargs)
            def solve_spy(*args, **kwargs):
                self.assertTrue((root / "geometry_audit_v1.json").is_file(),
                                "audit JSON must exist before NLP invocation")
                return solution
            with patch.object(harness, "audit_track_geometry", side_effect=audit_spy) as audit_mock, \
                 patch.object(harness, "solve_spline_line", side_effect=solve_spy) as solve_mock:
                harness._attempt_worker(str(request_path), str(result_path), str(artifact_path))
            result = json.loads(result_path.read_text(encoding="utf-8"))
            self.assertEqual(solve_mock.call_count, 1, result)
            self.assertEqual(audit_mock.call_count, 1)
            self.assertTrue((root / "geometry_audit_v1.json").is_file())
            audit_document = json.loads((root / "geometry_audit_v1.json").read_text(encoding="utf-8"))
            self.assertEqual(audit_document["version"], "geometry_audit_v1")
            self.assertEqual(audit_document["claim"], "diagnostic_only")
            self.assertEqual(result["geometry_audit"]["artifact"], "geometry_audit_v1.json")
            self.assertEqual(result["geometry_audit"]["claim"], "diagnostic_only")
            self.assertEqual(result["status"], "Solve_Succeeded", result)
            self.assertFalse(result["accepted"])
            self.assertEqual(result["validation_status"], "incomplete")
            self.assertFalse(result.get("evidence_complete", False))
            self.assertTrue((root / "trajectory_metadata.npz").is_file())
            manifest = load_run_manifest_v1(root)
            effective = json.loads((root / manifest["effective_request"]["path"]).read_text(encoding="utf-8"))
            boundary = json.loads((root / manifest["boundary_event_contract"]["path"]).read_text(encoding="utf-8"))
            expected = validate_problem_contract(contract)
            for identity in (effective["track_problem"], boundary["track_problem"]):
                self.assertEqual(identity["status"], "supported")
                self.assertEqual(identity["enforcement"], "requested_boundary_enforced")
                self.assertEqual(identity["contract_hash"], expected["normalized_hash"])
                self.assertEqual(identity["normalized_contract"], expected["normalized_contract"])
            self.assertEqual(manifest["effective_request"]["sha256"], __import__("hashlib").sha256(
                (root / manifest["effective_request"]["path"]).read_bytes()).hexdigest())
            self.assertEqual(manifest["boundary_event_contract"]["sha256"], __import__("hashlib").sha256(
                (root / manifest["boundary_event_contract"]["path"]).read_bytes()).hexdigest())
            self.assertTrue(manifest["transitive_dependencies"])
            from simplified_vehicle_models.solvers.qss_spline_ocp import _reference_curve
            expected_reference = _reference_curve(track)
            self.assertEqual((audit_seen[0].start, audit_seen[0].end,
                              audit_seen[0].interpolation_periodic),
                             (expected_reference.start, expected_reference.end,
                              expected_reference.interpolation_periodic))
            audit_member = next(member for member in manifest["transitive_dependencies"]
                                if member["path"] == "geometry_audit_v1.json")
            audit_bytes = (root / audit_member["path"]).read_bytes()
            self.assertEqual(audit_member["sha256"], __import__("hashlib").sha256(audit_bytes).hexdigest())
            self.assertEqual(audit_bytes, (json.dumps(audit_document, sort_keys=True, separators=(",", ":"),
                allow_nan=False) + "\n").encode("utf-8"))

    def test_worker_forwards_serialized_nondefault_options_and_records_realized_sites(self):
        class FakeSpline:
            control_count = 2
            control_distance = np.array([0.0, 2.0])
        class FakeSolution:
            distance = np.array([0.0, 2.0]); offset = np.zeros(2); heading_error = np.zeros(2)
            curvature = np.zeros(2); speed = np.full(2, 10.); acceleration = np.zeros(2)
            midpoint_acceleration = np.zeros(1); coefficients = np.zeros(2)
            state_representation = "energy_hs_v1"; time_quadrature = "fixture"; time_quadrature_metadata = {}
            status = "Solve_Succeeded"; lap_time = 1.; objective_value = 1.; iterations = 1; maximum_violation = 0.
        case = harness.default_cases()["acceleration"]
        request = harness.build_attempt(case, mesh_spacing_m=5., basis_spacing_m=35., maximum_iterations=1,
            wall_cap_s=3., seed_name="centerline", formulation=Path("form.xml"), surface=Path("surface.npz"),
            energy_defect_scaling="cell_length_v1", coefficient_coordinate_scale=.1,
            mesh_stations=[0., .5, 2.], constraint_stations=[1.25, .25])
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); request_path = root / "request.json"; result_path = root / "result.json"
            request_path.write_text(json.dumps(harness._json_safe(request), allow_nan=False), encoding="utf-8")
            captured = {}
            def capture(*args, **kwargs):
                captured.update(kwargs)
                return FakeSolution()
            fake_track = SimpleNamespace(distance=np.array([0., 2.]), lap_length=2., is_closed=False)
            with patch.object(input_preflight, "preflight_inputs", return_value=None), \
                 patch.object(harness, "make_track_view", return_value=fake_track), \
                 patch.object(harness, "corridor_from_track", return_value=object()), \
                 patch.object(harness, "compile_formulation", return_value=SimpleNamespace(model=object())), \
                 patch.object(harness, "point_mass_view_of_planar", return_value=object()), \
                 patch.object(harness, "load_surface_dispatch", return_value=(SimpleNamespace(provenance_hash="ok"), "fixture")), \
                 patch.object(harness, "capability_hash", return_value="ok"), \
                 patch.object(harness, "OffsetSpline", return_value=FakeSpline()), \
                 patch.object(harness, "initial_guess", return_value=np.zeros(7)), \
                 patch.object(harness, "_seed_domain_report", return_value={}), \
                 patch.object(harness, "solve_spline_line", side_effect=capture), \
                 patch.object(harness, "validate_line", return_value={"checks": {}, "model_feasible": False}), \
                 patch("tools.run.solve_minimum_time_line._write_trajectory_metadata", side_effect=lambda path, solution: np.savez(path, metadata=np.array("fixture"))):
                harness._attempt_worker(str(request_path), str(result_path), str(root / "solution.npz"))
            result = json.loads(result_path.read_text())
        self.assertEqual(captured["energy_defect_scaling"], "cell_length_v1")
        self.assertEqual(captured["coefficient_coordinate_scale"], .1)
        self.assertEqual(captured["constraint_stations"], [.25, 1.25])
        self.assertEqual(result["request"]["energy_defect_scaling"], "cell_length_v1")
        self.assertEqual(result["actual_mesh"]["stations_m"], [0., .5, 2.])
        self.assertGreater(len(result["actual_constraint_stations_m"]), 2)
        self.assertEqual(result["actual_request_schema_version"], 2)

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
            surface=Path("surface.npz"), seed_profile="energy-nlp",
            profile_control_projection="bounded_energy_l1")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); request_path = root / "request.json"; result_path = root / "result.json"
            request_path.write_text(json.dumps(harness._json_safe(request)), encoding="utf-8")
            with patch.object(input_preflight, "preflight_inputs", return_value=None), \
                 patch.object(harness, "make_track_view", return_value=object()), \
                 patch.object(harness, "corridor_from_track", return_value=object()), \
                 patch.object(harness, "compile_formulation", return_value=SimpleNamespace(model=object())), \
                 patch.object(harness, "point_mass_view_of_planar", return_value=object()), \
                 patch.object(harness, "load_surface_dispatch", return_value=(SimpleNamespace(provenance_hash="ok"), "fixture")), \
                 patch.object(harness, "capability_hash", return_value="ok"), \
                 patch.object(harness, "build_mesh", return_value=np.array([0., 1.])), \
                 patch.object(harness, "OffsetSpline", return_value=FakeSpline()), \
                 patch.object(harness, "initial_guess", return_value=np.zeros(7)), \
                 patch.object(harness, "_seed_domain_report", return_value={}), \
                 patch.object(harness, "build_profile_line_seed", return_value=(None, {"available": False, "reason": "fixture", "projection_attempted": True})), \
                 patch.object(harness, "solve_spline_line", return_value=FakeSolution()), \
                 patch.object(harness, "validate_line", return_value={"checks": {"geometry_consistency": True, "dense_feasibility": True}, "model_feasible": True}), \
                 patch("tools.run.solve_minimum_time_line._write_trajectory_metadata", side_effect=lambda path, solution: np.savez(path, metadata=np.array("fixture"))):
                harness._attempt_worker(str(request_path), str(result_path), str(root / "solution.npz"))
            result = json.loads(result_path.read_text())
        self.assertFalse(result["profile_seed"]["used"])
        self.assertEqual(result["profile_seed"]["fallback"], "explicit-old-seed")
        self.assertEqual(result["profile_seed"]["diagnostics"]["reason"], "fixture")
        self.assertTrue(result["profile_seed"]["initializer_attempted"])
        self.assertTrue(result["profile_seed"]["control_projection_attempted"])
        self.assertIsNone(result["profile_seed"]["control_projection_used_by_seed"])
        self.assertEqual(result["actual_initializer"]["stable_identity"]["actual_control_projection"], None)

    def test_profile_seed_precedes_old_initializer_and_strict_failure_skips_it(self):
        class FakeSpline:
            control_count = 2
            control_distance = np.array([0.0, 1.0])
        class FakeSolution:
            distance = np.array([0., 1.]); offset = heading_error = curvature = np.zeros(2)
            speed = np.full(2, 10.); acceleration = np.zeros(2); midpoint_acceleration = np.zeros(1)
            coefficients = np.zeros(2); state_representation = "energy_hs_v1"; time_quadrature = "fixture"
            time_quadrature_metadata = {}; status = "Solve_Succeeded"; lap_time = objective_value = 1.
            iterations = 1; maximum_violation = 0.
        request = harness.build_attempt(harness.default_cases()["acceleration"], mesh_spacing_m=5., basis_spacing_m=35.,
            maximum_iterations=7, wall_cap_s=3., seed_name="centerline", formulation=Path("form.xml"),
            surface=Path("surface.npz"), seed_profile="energy-nlp",
            profile_control_projection="bounded_energy_l1")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); request_path = root / "request.json"; result_path = root / "result.json"
            request_path.write_text(json.dumps(harness._json_safe(request)), encoding="utf-8")
            profile_seed = np.zeros(7); captured = {}
            def capture_profile(*args, **kwargs):
                captured["control_projection"] = kwargs["control_projection"]
                return profile_seed, {"available": True, "projection_attempted": True}
            def capture_solution(*args, **kwargs):
                captured["seed"] = kwargs["seed"]
                return FakeSolution()
            with patch.object(input_preflight, "preflight_inputs", return_value=None), \
                 patch.object(harness, "make_track_view", return_value=object()), \
                 patch.object(harness, "corridor_from_track", return_value=object()), \
                 patch.object(harness, "compile_formulation", return_value=SimpleNamespace(model=object())), \
                 patch.object(harness, "point_mass_view_of_planar", return_value=object()), \
                 patch.object(harness, "load_surface_dispatch", return_value=(SimpleNamespace(provenance_hash="ok"), "fixture")), \
                 patch.object(harness, "capability_hash", return_value="ok"), \
                 patch.object(harness, "build_mesh", return_value=np.array([0., 1.])), \
                 patch.object(harness, "OffsetSpline", return_value=FakeSpline()), \
                 patch.object(harness, "initial_guess", side_effect=AssertionError("old seed must not run")), \
                 patch.object(harness, "build_profile_line_seed", side_effect=capture_profile), \
                 patch.object(harness, "solve_spline_line", side_effect=capture_solution) as solve_spy, \
                 patch.object(harness, "validate_line", return_value={"checks": {"geometry_consistency": True, "dense_feasibility": True}, "model_feasible": True}), \
                 patch("tools.run.solve_minimum_time_line._write_trajectory_metadata", side_effect=lambda path, solution: np.savez(path, metadata=np.array("fixture"))):
                harness._attempt_worker(str(request_path), str(result_path), str(root / "solution.npz"))
            result = json.loads(result_path.read_text())
        self.assertEqual(result["status"], "manifest_unavailable")
        self.assertIn("required source input is absent", result["manifest_error"])
        self.assertTrue(result["solver_succeeded"])
        self.assertFalse(result["accepted"])
        np.testing.assert_array_equal(captured["seed"], profile_seed)
        self.assertTrue(result["profile_seed"]["used"])
        self.assertEqual(captured["control_projection"], "bounded_energy_l1")
        forwarded = solve_spy.call_args.kwargs
        self.assertEqual(forwarded["energy_defect_scaling"], "legacy_reference_speed_v1")
        self.assertEqual(forwarded["coefficient_coordinate_scale"], 1.0)
        self.assertIsNone(forwarded["constraint_stations"])
        self.assertTrue(result["profile_seed"]["initializer_attempted"])
        self.assertTrue(result["profile_seed"]["control_projection_attempted"])
        self.assertEqual(result["profile_seed"]["control_projection_used_by_seed"], "bounded_energy_l1")

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
            formulation=Path("form.xml"), surface=Path("surface.npz"), dense_maximum_refinements=6,
            control_representation="quadratic_energy_rate_over_metric_v1",
            speed_domain_margin_mps=.125)
        captured = {}
        def fake_validate(solution, corridor, surface, settings, **kwargs):
            captured["fractions"] = settings.off_collocation_fractions
            captured["control_representation"] = settings.control_representation
            captured["speed_domain_margin_mps"] = settings.speed_domain_margin_mps
            captured["refinements"] = kwargs["maximum_refinements"]
            return {"checks": {"geometry_consistency": True, "dense_feasibility": True},
                    "model_feasible": True}
        def fake_trajectory_metadata(path, solution):
            np.savez(path, metadata=np.array("fixture"))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); request_path = root / "request.json"; result_path = root / "result.json"
            request_path.write_text(json.dumps(harness._json_safe(request)), encoding="utf-8")
            with patch.object(input_preflight, "preflight_inputs", return_value=None), \
                 patch.object(harness, "make_track_view", return_value=object()), \
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
            self.assertEqual(result["status"], "manifest_unavailable")
            self.assertIn("required source input is absent", result["manifest_error"])
            self.assertTrue(result["solver_succeeded"])
            self.assertFalse(result["accepted"])
            self.assertEqual(result["validation_status"], "incomplete")
            self.assertEqual(captured["fractions"], harness.DEFAULT_OFF_COLLOCATION_FRACTIONS)
            self.assertEqual(captured["control_representation"],
                             "quadratic_energy_rate_over_metric_v1")
            self.assertEqual(captured["speed_domain_margin_mps"], .125)
            self.assertEqual(captured["refinements"], 6)
            self.assertEqual(result["control_representation"],
                             "quadratic_energy_rate_over_metric_v1")
            self.assertEqual(result["speed_domain_margin_mps"], .125)
            with np.load(root / "solution.npz", allow_pickle=False) as artifact:
                self.assertEqual(str(artifact["state_representation"]), "energy_hs_v1")
                self.assertEqual(str(artifact["control_representation"]),
                                 "quadratic_energy_rate_over_metric_v1")
                self.assertEqual(float(artifact["speed_domain_margin_mps"]), .125)
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
            with patch.object(input_preflight, "preflight_inputs", return_value=None), \
                 patch.object(harness, "make_track_view", return_value=object()), \
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

    def test_worker_persists_spline_failure_diagnostics_without_solution_artifact(self):
        from simplified_vehicle_models.solvers.qss_spline_ocp import SplineSolveError
        class FakeSpline:
            control_count = 2
            control_distance = np.array([0., 1.])
        request = harness.build_attempt(harness.default_cases()["acceleration"], mesh_spacing_m=5.,
            basis_spacing_m=35., maximum_iterations=1, wall_cap_s=3., seed_name="centerline",
            formulation=Path("form.xml"), surface=Path("surface.npz"))
        diagnostics = {"return_status": "Maximum_Iterations_Exceeded", "iter_count": 1,
                       "total_iterations": 1, "objective_value": 9.,
                       "physical_max_constraint_violation": .25,
                       "physical_max_bound_violation": .5, "layout": {"decision_size": 3},
                       "dual_x_coordinates": "scaled_decision",
                       "dual_g_coordinates": "unscaled_constraint_residuals"}
        error = SplineSolveError("quasi-steady line solver failed: Maximum_Iterations_Exceeded",
                                 diagnostics, failed_values=np.array([1., 2., 3.]),
                                 constraint_values=np.array([.25]), dual_g=np.array([.5]))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); request_path = root / "request.json"; result_path = root / "result.json"
            request_path.write_text(json.dumps(harness._json_safe(request)), encoding="utf-8")
            with patch.object(input_preflight, "preflight_inputs", return_value=None), \
                 patch.object(harness, "make_track_view", return_value=object()), \
                 patch.object(harness, "corridor_from_track", return_value=object()), \
                 patch.object(harness, "compile_formulation", return_value=SimpleNamespace(model=object())), \
                 patch.object(harness, "point_mass_view_of_planar", return_value=object()), \
                 patch.object(harness, "load_surface_dispatch", return_value=(SimpleNamespace(provenance_hash="ok"), "fixture")), \
                 patch.object(harness, "capability_hash", return_value="ok"), \
                 patch.object(harness, "build_mesh", return_value=np.array([0., 1.])), \
                 patch.object(harness, "OffsetSpline", return_value=FakeSpline()), \
                 patch.object(harness, "initial_guess", return_value=np.zeros(7)), \
                 patch.object(harness, "_seed_domain_report", return_value={}), \
                 patch.object(harness, "solve_spline_line", side_effect=error), \
                 patch.object(harness, "validate_line", side_effect=AssertionError("failed iterate must not validate")):
                harness._attempt_worker(str(request_path), str(result_path), str(root / "solution.npz"))
            result = json.loads(result_path.read_text())
            self.assertFalse(result["accepted"]); self.assertFalse(result["solver_succeeded"])
            audit_path = root / "geometry_audit_v1.json"
            audit_document = json.loads(audit_path.read_text(encoding="utf-8"))
            self.assertEqual(audit_document["claim"], "diagnostic_only")
            self.assertEqual(result["geometry_audit"]["artifact"], audit_path.name)
            self.assertNotIn("artifact", result)
            self.assertEqual(result["failure_diagnostics"]["total_iterations"], 1)
            self.assertEqual(result["failure_diagnostics"]["physical_max_constraint_violation"], .25)
            self.assertTrue((root / result["failed_artifact"]).is_file())
            with np.load(root / result["failed_artifact"], allow_pickle=False) as saved:
                np.testing.assert_array_equal(saved["failed_values"], [1., 2., 3.])
                self.assertIn("constraint_values", saved); self.assertIn("dual_g", saved)
                metadata = json.loads(str(saved["metadata"]))
                self.assertEqual(metadata["dual_x_coordinates"], "scaled_decision")
                self.assertEqual(metadata["dual_g_coordinates"], "unscaled_constraint_residuals")
                self.assertNotIn("dual_coordinates", metadata)

    def test_generic_runtime_error_keeps_legacy_failure_shape(self):
        class FakeSpline:
            control_count = 2
            control_distance = np.array([0., 1.])
        request = harness.build_attempt(harness.default_cases()["acceleration"], mesh_spacing_m=5.,
            basis_spacing_m=35., maximum_iterations=1, wall_cap_s=3., seed_name="centerline",
            formulation=Path("form.xml"), surface=Path("surface.npz"))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); request_path = root / "request.json"; result_path = root / "result.json"
            request_path.write_text(json.dumps(harness._json_safe(request)), encoding="utf-8")
            with patch.object(input_preflight, "preflight_inputs", return_value=None), \
                 patch.object(harness, "make_track_view", return_value=object()), \
                 patch.object(harness, "corridor_from_track", return_value=object()), \
                 patch.object(harness, "compile_formulation", return_value=SimpleNamespace(model=object())), \
                 patch.object(harness, "point_mass_view_of_planar", return_value=object()), \
                 patch.object(harness, "load_surface_dispatch", return_value=(SimpleNamespace(provenance_hash="ok"), "fixture")), \
                 patch.object(harness, "capability_hash", return_value="ok"), \
                 patch.object(harness, "build_mesh", return_value=np.array([0., 1.])), \
                 patch.object(harness, "OffsetSpline", return_value=FakeSpline()), \
                 patch.object(harness, "initial_guess", return_value=np.zeros(7)), \
                 patch.object(harness, "_seed_domain_report", return_value={}), \
                 patch.object(harness, "solve_spline_line", side_effect=RuntimeError("legacy failure")):
                harness._attempt_worker(str(request_path), str(result_path), str(root / "solution.npz"))
            result = json.loads(result_path.read_text())
        self.assertEqual(result["error_type"], "RuntimeError")
        self.assertFalse(result["accepted"]); self.assertFalse(result["solver_succeeded"])
        self.assertNotIn("failure_diagnostics", result)
        self.assertNotIn("failed_artifact", result)

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
            with patch("simplified_vehicle_models.solvers.minimum_time_run.mp.get_context", return_value=Context()):
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
            with patch("simplified_vehicle_models.solvers.minimum_time_run.mp.get_context", return_value=Context()):
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

    def test_source_hashes_include_model_ref_dependencies(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            root.joinpath("parent.xml").write_text('<model><model ref="nested.xml" /></model>')
            root.joinpath("nested.xml").write_text('<model><source ref="input.csv" /></model>')
            root.joinpath("input.csv").write_text("value\\n")
            hashes = harness.source_hashes(root / "parent.xml")
            self.assertEqual(set(hashes), {str((root / name).resolve())
                                           for name in ("parent.xml", "nested.xml", "input.csv")})

    def test_source_hashes_reject_missing_recursive_dependency(self):
        with tempfile.TemporaryDirectory() as directory:
            xml = Path(directory) / "track.xml"
            xml.write_text('<model><source ref="missing.xml" /></model>', encoding="utf-8")
            with self.assertRaisesRegex(FileNotFoundError, "dependency"):
                harness.source_hashes(xml)

    def test_failed_artifact_never_enters_refinement_candidates(self):
        row = {"request": {"case": {"name": "acceleration"}},
               "failed_artifact": "failed_iterate.npz", "status": "Maximum_Iterations_Exceeded",
               "solver_succeeded": False, "accepted": False}
        with tempfile.TemporaryDirectory() as directory:
            report = harness.aggregate([row], Path(directory), expected_cases=("acceleration",))
        self.assertEqual(report["candidate_diagnostics"], [])
        self.assertEqual(report["mesh_refinement"], [])
        self.assertFalse(report["case_status"]["acceleration"]["accepted"])

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
        self.assertEqual(report["candidate_diagnostics"][0]["comparison_status"], "comparable")
        self.assertTrue(report["candidate_diagnostics"][0]["geometry_contract_match"])
        self.assertEqual(report["mesh_refinement"], [])
        self.assertEqual(report["basis_refinement"], [])
        self.assertFalse(report["strict_aggregate"]["accepted"])
        self.assertEqual(report["strict_aggregate"]["status"], "incomplete")

    def test_aggregate_mixed_cut_geometry_contracts_have_no_physical_deltas(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); case_dir = root / "mis_cut_open"; case_dir.mkdir()
            rows = []
            for attempt, mesh, contract, offset in (
                    ("new-fit", 5.0, harness.SOURCE_GEOMETRY_CONTRACT, 0.0),
                    ("legacy-refit", 2.5, None, 100.0)):
                target = case_dir / attempt; target.mkdir()
                artifact = target / "solution.npz"
                np.savez(artifact, distance=np.array([0., 1.]),
                         offset=np.array([offset, offset]), speed=np.array([10., 12.]),
                         curvature=np.array([0., 0.]))
                request = {"case": {"name": "mis_cut_open", "topology": "full_cut_open"},
                           "seed": "centerline", "settings": {"tolerance": 1e-6},
                           "source_hashes": {"input": "same"}, "mesh_spacing_m": mesh,
                           "basis_spacing_m": 35.0}
                if contract is not None:
                    request["source_geometry_contract"] = contract
                rows.append({"request": request, "source_hashes_after": {"input": "same"},
                             "surface_representation": "signed", "artifact": f"{attempt}/solution.npz",
                             "objective_s": 100.0 + offset, "status": "Solve_Succeeded",
                             "solver_succeeded": True, "dense_original_model": {"checks": {
                                 "geometry_consistency": True, "dense_feasibility": True},
                                 "model_feasible": True}})
            report = harness.aggregate(rows, root, expected_cases=("mis_cut_open",))
        self.assertEqual(len(report["candidate_diagnostics"]), 1)
        comparison = report["candidate_diagnostics"][0]
        self.assertEqual(comparison["comparison_status"], "non_comparable_geometry")
        self.assertFalse(comparison["geometry_contract_match"])
        self.assertEqual(comparison["left"]["source_geometry_contract"], harness.SOURCE_GEOMETRY_CONTRACT)
        self.assertIsNone(comparison["right"]["source_geometry_contract"])
        for field in ("time_difference_s", "time_relative_difference", "rms_offset_m", "rms_speed_mps"):
            self.assertIsNone(comparison[field], field)
        self.assertFalse(comparison["metric_diagnostic_within_user_budget"])
        self.assertEqual(report["mesh_refinement"], [])
        self.assertEqual(report["basis_refinement"], [])
        self.assertFalse(report["strict_aggregate"]["accepted"])

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
                               base_commit=None)
        args.baseline_label = "historical-label"
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
        self.assertEqual(manifest["base_commit"], manifest["git_head"])
        self.assertIsNone(manifest["requested_base_commit"])
        self.assertEqual(manifest["baseline_label"], "historical-label")
        self.assertEqual(manifest["provenance_initial"]["git_head"], manifest["git_head"])
        self.assertIn("provenance_afterfinal", manifest)
        self.assertEqual(manifest["aggregate"]["mesh_refinement"], [])

    def test_mocked_solver_status_failure_cannot_be_accepted(self):
        # This exercises the only status-to-acceptance contract used by the worker.
        strict = harness.strict_validation(False, {name: True for name in harness.REQUIRED_NUMERICAL_CHECKS})
        self.assertEqual(strict["validation_status"], "solver_failed")
        self.assertFalse(strict["accepted"])


    def test_initializer_options_are_canonical_and_invalid_values_rejected(self):
        case = harness.default_cases()["acceleration"]
        request = harness.build_attempt(case, mesh_spacing_m=5., basis_spacing_m=35., maximum_iterations=1,
            wall_cap_s=1., seed_name="centerline", formulation=Path("f"), surface=Path("s"),
            seed_profile="energy-nlp", strict_profile_seed=True, profile_step=3.,
            maximum_profile_iterations=11, maximum_profile_seconds=7.)
        self.assertEqual(request["initializer"]["profile_options"]["profile_step_m"], 3.)
        self.assertTrue(request["initializer"]["strict_profile_seed"])
        for bad in (0., float("nan")):
            with self.assertRaisesRegex(ValueError, "profile_step"):
                harness.build_attempt(case, mesh_spacing_m=5., basis_spacing_m=35., maximum_iterations=1,
                    wall_cap_s=1., seed_name="centerline", formulation=Path("f"), surface=Path("s"), profile_step=bad)

    def test_explicit_contract_hash_is_required_for_comparison_identity(self):
        contract = {"schema": "track_problem_v1", "topology": "open",
                    "source_stations": [0., 1.], "lower_offset": [-1., -1.],
                    "upper_offset": [1., 1.],
                    "boundary": {"start_mode": "rolling", "start_speed_mps": 3., "finish_mode": "free"}}
        from simplified_vehicle_models.solvers.track_problem_contract import validate_problem_contract
        digest = validate_problem_contract(contract)["normalized_hash"]
        request = {"settings": {}, "initializer": "seed", "convergence_criteria": None,
                   "source_hashes": {"input": "a"}, "provenance": {"git_head": "a"},
                   "track_problem_contract": contract}
        row = {"request": request, "source_hashes_after": {"input": "a"},
               "provenance_after": {"git_head": "a"}, "surface_representation": "signed",
               "track_problem": {"contract_hash": None}}
        right = {**row, "request": dict(request), "track_problem": {"contract_hash": None}}
        self.assertFalse(harness._same_refinement_conditions(row, right))
        row["track_problem"] = {"contract_hash": digest}
        right["track_problem"] = {"contract_hash": digest}
        self.assertTrue(harness._same_refinement_conditions(row, right))
        right["request"]["track_problem_contract"] = dict(contract, topology="closed",
            boundary={"start_mode":"flying", "start_speed_mps":3., "finish_mode":"periodic"})
        self.assertFalse(harness._same_refinement_conditions(row, right))
        right["request"]["track_problem_contract"] = contract
        right["track_problem"]["contract_hash"] = "f" * 64
        self.assertFalse(harness._same_refinement_conditions(row, right))

    def test_explicit_criteria_are_required_for_qualified_edge(self):
        metrics = {"rms_xy_m": .1, "rms_metric": .1, "rms_offset_m": .1, "length_relative": .1,
                   "rms_speed_mps": .1, "rms_curvature_per_m": .1}
        self.assertFalse(harness._criteria_qualified(metrics, None))
        self.assertTrue(harness._criteria_qualified(metrics, {name: .1 for name in metrics}))
        self.assertFalse(harness._criteria_qualified(metrics, {name: .09 for name in metrics}))
        with self.assertRaisesRegex(ValueError, "nonnegative"):
            harness._convergence_criteria({name: -1. for name in metrics})

    def test_same_conditions_reject_initializer_or_runtime_provenance_changes(self):
        request = {"settings": {}, "initializer": {"seed": "centerline"}, "convergence_criteria": None,
                   "source_hashes": {"input": "a"}, "provenance": {"git_head": "a"}}
        left = {"request": request, "source_hashes_after": {"input": "a"}, "provenance_after": {"git_head": "a"},
                "actual_initializer": {"fingerprint": "one"}, "surface_representation": "signed"}
        right = {**left, "request": dict(request), "actual_initializer": {"fingerprint": "two"}}
        self.assertFalse(harness._same_refinement_conditions(left, right))
        right["actual_initializer"] = {"fingerprint": "one"}
        right["request"]["provenance"] = {"git_head": "other"}
        self.assertFalse(harness._same_refinement_conditions(left, right))

    def test_base_commit_resolution_requires_actual_head(self):
        from tools.diagnostics.no_tv_provenance import resolve_base_commit
        def git(repo, *args):
            return "head" if args == ("rev-parse", "HEAD") else "other"
        self.assertEqual(resolve_base_commit(Path("."), None, git=git), "head")
        with self.assertRaisesRegex(ValueError, "actual current HEAD"):
            resolve_base_commit(Path("."), "historical", git=git)

    def test_stable_initializer_identity_ignores_seed_bytes_and_elapsed_diagnostics(self):
        request = {"settings": {}, "initializer": {"seed": "centerline", "seed_profile": "energy-nlp"},
                   "convergence_criteria": None, "source_hashes": {"input": "a"},
                   "provenance": {"git_head": "a", "runtime": {"casadi": "3.8.1"}}}
        stable = {"requested_mode": request["initializer"], "actual_profile_used": True,
                  "fallback_category": None, "used": "energy-nlp-profile",
                  "implementation_runtime": request["provenance"]}
        left = {"request": request, "source_hashes_after": {"input": "a"},
                "provenance_after": request["provenance"], "surface_representation": "signed",
                "actual_initializer": {"stable_identity": stable, "seed_sha256": "a",
                                        "profile_seed": {"diagnostics": {"elapsed_s": 1.}}}}
        right = {**left, "request": dict(request), "actual_initializer": {
            "stable_identity": dict(stable), "seed_sha256": "different-vector-length-and-bytes",
            "profile_seed": {"diagnostics": {"elapsed_s": 99.}}}}
        self.assertTrue(harness._same_refinement_conditions(left, right))
        right["actual_initializer"]["stable_identity"]["fallback_category"] = "explicit-old-seed"
        self.assertFalse(harness._same_refinement_conditions(left, right))
        right["actual_initializer"]["stable_identity"] = dict(stable)
        right["request"]["initializer"] = {"seed": "low_amplitude_offset"}
        self.assertFalse(harness._same_refinement_conditions(left, right))
        right["request"]["initializer"] = dict(request["initializer"])
        right["request"]["initializer"]["profile_options"] = {"profile_step_m": 3.0}
        self.assertFalse(harness._same_refinement_conditions(left, right))
        right["request"]["initializer"] = dict(request["initializer"])
        left["request"]["source_geometry_contract"] = harness.SOURCE_GEOMETRY_CONTRACT
        right["request"]["source_geometry_contract"] = harness.NATIVE_GEOMETRY_CONTRACT
        self.assertFalse(harness._same_refinement_conditions(left, right))

    def test_user_metric_budgets_are_diagnostic_and_never_strict_evidence(self):
        metrics = {"rms_xy_m": 1., "rms_metric": 1., "rms_offset_m": 1., "length_relative": 1.,
                   "rms_speed_mps": 1., "rms_curvature_per_m": 1.}
        comparison = {"left": {"mesh_m": 5., "basis_m": 35., "attempt_id": "a"},
                      "right": {"mesh_m": 2.5, "basis_m": 35., "attempt_id": "b"},
                      "reference_qualified": True, "time_relative_difference": 0.,
                      "geometry_state_convergence": None, "convergence_criteria": {name: 1e99 for name in metrics},
                      **metrics}
        self.assertTrue(harness._criteria_qualified(metrics, comparison["convergence_criteria"]))
        self.assertIsNone(comparison["geometry_state_convergence"])
        self.assertIsNone(harness._qualified_edge([comparison], kind="mesh_refinement"))


if __name__ == "__main__":
    unittest.main()
