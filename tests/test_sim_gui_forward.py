import hashlib
import json
import os
import tempfile
import inspect
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np

from tools.sim_gui.settings import RunSettings, solution_model_for, validate


class ForwardSettingsTests(unittest.TestCase):
    def test_hybrid_telemetry_has_no_nondeterministic_decision_timing_fields(self):
        from tools.sim_gui.telemetry_schema import HYBRID_FIELDS
        self.assertIn("decision_query_count",HYBRID_FIELDS)
        for name in ("decision_candidate_seconds","decision_probe_seconds","decision_transition_seconds"):
            self.assertNotIn(name,HYBRID_FIELDS)

    def test_policy_row_uses_normalized_dataclass_powertrain_diagnostics(self):
        from tools.sim_gui.forward_execute import _policy_row
        from simplified_vehicle_models.solvers.forward_driver import StepSample
        diagnostics = SimpleNamespace(to_dict=lambda: {"wheel_motor_torque_nm": [1., 2., 3., 4.]})
        sample = StepSample(0, 0., np.zeros(2), np.zeros(2), SimpleNamespace(throttle=0., brake_force=0., steering_rate=0.),
            None, None, None, None, diagnostics)
        row = _policy_row(sample)
        self.assertEqual(row["applied_powertrain_diagnostics"]["wheel_motor_torque_nm"], [1., 2., 3., 4.])

    def test_forward_backend_rejects_symlinked_attempt_ancestor_escape(self):
        from tools.sim_gui.forward_execute import _run_impl
        from tools.sim_gui.settings import RunSettings
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            case_dir = root / "case"
            outside = root / "outside"
            case_dir.mkdir()
            outside.mkdir()
            (outside / "run.log").write_text("parent log")
            (case_dir / "redirect").symlink_to(outside, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, "symlink|resolve inside"):
                _run_impl({}, case_dir / "redirect", RunSettings(solver_kind="forward-driver"), case_dir)

    def test_canonical_execution_source_identity_is_cwd_independent_and_fail_closed(self):
        import subprocess, sys
        from tools.sim_gui.source_identity import execution_source_identity, canonical_repo_path
        expected = execution_source_identity()
        self.assertEqual(canonical_repo_path(Path.cwd() / "tools/sim_gui/source_identity.py"),
                         "tools/sim_gui/source_identity.py")
        code = ("from tools.sim_gui.source_identity import execution_source_identity; "
                "print(execution_source_identity()['tools/sim_gui/source_identity.py'])")
        result = subprocess.run([sys.executable, "-c", code], cwd="/tmp", check=True,
                                capture_output=True, text=True,
                                env={**os.environ, "PYTHONPATH": str(Path(__file__).resolve().parents[1])})
        self.assertEqual(result.stdout.strip(), expected["tools/sim_gui/source_identity.py"])
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            required=tuple(expected)
            for rel in required:
                path = root / rel
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("original")
            from tools.sim_gui import source_identity
            with mock.patch.object(source_identity, "_REQUIRED", ("tools/sim_gui/source_identity.py",)):
                initial = source_identity.execution_source_identity(root=root)
                (root / "tools/sim_gui/source_identity.py").write_text("changed")
                changed = source_identity.execution_source_identity(root=root)
                self.assertNotEqual(initial, changed)
                (root / "tools/sim_gui/source_identity.py").unlink()
                with self.assertRaises(FileNotFoundError):
                    source_identity.execution_source_identity(root=root)

    def test_hybrid_mode_settings_default_enum_migration_and_fail_closed_validation(self):
        defaults = RunSettings(solver_kind="forward-driver")
        self.assertEqual(defaults.forward_longitudinal_mode, "historical-preview-v1")
        old = defaults.to_json()
        old.pop("forward_longitudinal_mode")
        old["forward_physical_brake_planning"] = True
        self.assertEqual(RunSettings.from_json(old).forward_longitudinal_mode, "physical-reachable-v1")
        hybrid = RunSettings(solver_kind="forward-driver", forward_longitudinal_mode="hybrid-drive-brake-v1")
        from tools.sim_gui.settings import forward_driver_config
        config = forward_driver_config(hybrid)
        self.assertEqual((config["hybrid_horizon_s"], config["hybrid_slabs"]), (3.0, 30))
        with self.assertRaisesRegex(ValueError, "requires the full vertical"):
            validate(hybrid, "planar-7dof", None)
        with self.assertRaisesRegex(ValueError, "contradicts"):
            RunSettings.from_json({**hybrid.to_json(), "forward_physical_brake_planning": True})
        with self.assertRaises(ValueError):
            RunSettings.from_json({**hybrid.to_json(), "forward_longitudinal_mode": "hybrid-guess"})
        self.assertEqual(forward_driver_config(hybrid)["initial_speed_mps"], 6.0)

    def test_hybrid_plan_remains_on_vertical_plant_with_tuple_steering_adapter(self):
        import numpy as np
        from types import SimpleNamespace
        from tools.sim_gui.forward_execute import _hybrid_steering_plan
        from simplified_vehicle_models.solvers.hybrid_speed_plan import HybridSpeedPlanSample
        hybrid_plan = SimpleNamespace(distance_m=np.array([0., 1.]), speed_mps=np.array([2., 3.]),
            curvature_inv_m=np.array([.1, .2]), is_closed=False,
            sample=lambda station: HybridSpeedPlanSample(station_m=.5, segment_index=0, speed_mps=2.5,
                acceleration_mps2=.4, corner_speed_cap_mps=3., braking_speed_bound_mps=1.8,
                binding_index=1, binding_event_station_m=1., distance_to_binding_event_m=.5,
                braking_onset_station_m=None, distance_to_braking_onset_m=None,
                braking_region=False, active_constraint="drive", status="nominal",
                curvature_inv_m=.15, future_event_index=None, future_event_station_m=None,
                distance_to_future_event_m=None, future_event_speed_cap_mps=None))
        adapter = _hybrid_steering_plan(hybrid_plan)
        self.assertEqual(adapter.sample(.5), (2.5, .15, .4))
        self.assertIs(adapter.speed, hybrid_plan.speed_mps)
        self.assertIsNot(adapter, hybrid_plan)
        self.assertIs(hybrid_plan.sample(.5).__class__, HybridSpeedPlanSample)

    def test_hybrid_telemetry_projection_preserves_distinct_acceleration_and_command_stages(self):
        from tools.sim_gui.telemetry_schema import HYBRID_FIELDS, HYBRID_SCHEMA, project, schema_document
        raw = {"schema": HYBRID_SCHEMA, "target_progress_acceleration_mps2": 1.2,
            "raw_acceleration_request_mps2": .9, "body_vx_dot_target_mps2": .8,
            "phase": "drive", "status": "nominal", "event_crossed": None,
            "event_crossing_time_s": None, "event_speed_cap_mps": None,
            "authority_domain_exceeded": False, "provider_identity": "provider",
            "plan_identity": "plan"}
        accepted = {"time": .25, "station_m": 4., "longitudinal_mode": "hybrid-drive-brake-v1",
            "decision": raw, "plan_sample": {"speed_mps": 5., "corner_speed_cap_mps": 8.,
                "braking_speed_bound_mps": 6.}, "planner_horizon_s": 3., "rollout_slabs": 30,
            "requested_command": {"throttle": .7, "brake_force": 0.},
            "mapped_command": {"throttle": .6, "brake_force": 10.},
            "applied_command": {"throttle": .5, "brake_force": 12.}, "pedal": .45,
            "allocator_identity": "independent-wheel-torque-v1"}
        row = project(accepted)
        self.assertEqual(row["allocator_identity"], "independent-wheel-torque-v1")
        self.assertEqual(row["provider_identity"], "provider")
        self.assertEqual(row["target_progress_acceleration_mps2"], 1.2)
        self.assertEqual(row["raw_acceleration_request_mps2"], .9)
        self.assertEqual(row["target_body_vx_dot_mps2"], .8)
        self.assertEqual((row["requested_brake_force_n"], row["mapped_brake_force_n"],
                          row["applied_brake_force_n"]), (0., 10., 12.))
        self.assertEqual(row["plan_speed_target_mps"], 5.)
        self.assertEqual(row["corner_speed_cap_mps"], 8.)
        self.assertEqual(row["backward_braking_ceiling_mps"], 6.)
        self.assertEqual(row["unavailable_reason"]["actual_event_time_s"],
                         "actual event transition is not instrumented")
        fields = schema_document("hybrid-drive-brake-v1")["fields"]
        self.assertEqual(set(fields), set(HYBRID_FIELDS))
        self.assertEqual({name: fields[name]["unit"] for name in (
            "time_s", "station_m", "plan_speed_target_mps", "corner_speed_cap_mps",
            "backward_braking_ceiling_mps", "target_progress_acceleration_mps2",
            "raw_acceleration_request_mps2", "target_body_vx_dot_mps2",
            "requested_throttle", "requested_brake_force_n", "mapped_brake_force_n",
            "applied_brake_force_n", "planner_horizon_s")}, {
            "time_s": "s", "station_m": "m", "plan_speed_target_mps": "m/s",
            "corner_speed_cap_mps": "m/s", "backward_braking_ceiling_mps": "m/s",
            "target_progress_acceleration_mps2": "m/s^2", "raw_acceleration_request_mps2": "m/s^2",
            "target_body_vx_dot_mps2": "m/s^2", "requested_throttle": "1",
            "requested_brake_force_n": "N", "mapped_brake_force_n": "N",
            "applied_brake_force_n": "N", "planner_horizon_s": "s"})
        for name in ("requested_throttle", "mapped_throttle", "applied_throttle"):
            self.assertGreaterEqual(row[name], 0.)
            self.assertLessEqual(row[name], 1.)
        for name in ("requested_brake_force_n", "mapped_brake_force_n", "applied_brake_force_n"):
            self.assertGreaterEqual(row[name], 0.)
        self.assertGreater(row["time_s"], 0.)
        self.assertGreaterEqual(row["station_m"], 0.)
        self.assertGreater(row["plan_speed_target_mps"], 0.)
        self.assertGreaterEqual(row["corner_speed_cap_mps"], row["plan_speed_target_mps"])
        self.assertGreater(row["planner_horizon_s"], 0.)
        self.assertEqual(row["rollout_slabs"], 30)

    def test_gui_physical_and_transient_controls_are_strict_and_explicit(self):
        from tools.sim_gui.settings import forward_driver_config
        defaults = RunSettings(solver_kind="forward-driver")
        self.assertFalse(defaults.forward_physical_brake_planning)
        self.assertFalse(defaults.forward_transient_corner_planning)
        enabled = RunSettings(solver_kind="forward-driver", forward_physical_brake_planning=True,
                              forward_transient_corner_planning=True)
        config = forward_driver_config(enabled)
        self.assertEqual(config["longitudinal_mode"], "physical-reachable-v1")
        self.assertTrue(config["transient_corner_planning"])
        for bad in (0, 1, "true", None):
            with self.assertRaises((TypeError, ValueError)):
                RunSettings.from_json({**enabled.to_json(), "forward_physical_brake_planning": bad})

    def test_gui_constructor_matches_current_preview_driver_api(self):
        from simplified_vehicle_models.solvers.forward_driver import PreviewDriver
        from tools.sim_gui import forward_execute
        signature = inspect.signature(PreviewDriver)
        self.assertNotIn("live_limit", signature.parameters)
        source = Path(forward_execute.__file__).read_text()
        driver_call = source.split("driver = PreviewDriver(", 1)[1].split("\n    initial =", 1)[0]
        self.assertNotIn("live_limit", driver_call)
        self.assertIn("live_limit=True", source.split("plan = build_speed_plan(", 1)[1].split("\n    if time.perf_counter", 1)[0])

    def test_gui_backend_runs_actual_policy_driver_and_solver(self):
        from simplified_vehicle_models.parameters.vehicle import PlanarVehicleParameters
        from tools.sim_gui.settings import forward_driver_config
        from tools.sim_gui import forward_execute
        settings = RunSettings(solver_kind="forward-driver", forward_longitudinal_mode="legacy", forward_transient_corner_planning=True)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            formulation, track_path = root / "formulation.xml", root / "track.xml"
            formulation.write_text("formulation")
            track_path.write_text("track")
            component = root / "component.xml"
            component.write_text("component")
            frozen = {str(path.resolve()): "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()
                      for path in (formulation, track_path, component)}
            from tools.sim_gui.source_identity import execution_source_identity, identity_digest, combined_source_closure
            execution=execution_source_identity(); complete=combined_source_closure(frozen,execution)
            identity = {"solver_kind":"forward-driver", "source_model":"planar-7dof",
                "solution_model":"planar-7dof", "formulation":str(formulation.resolve()),
                "formulation_sha256":hashlib.sha256(formulation.read_bytes()).hexdigest(),
                "track":str(track_path.resolve()), "track_sha256":hashlib.sha256(track_path.read_bytes()).hexdigest(),
                "settings":settings.to_json(), "settings_sha256":hashlib.sha256(json.dumps(settings.to_json(),sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest(),
                "driver_config":forward_driver_config(settings), "component_source_hashes":frozen,
                "authority_source_closure_sha256":identity_digest(frozen),"execution_source_hashes":execution,
                "execution_source_identity_sha256":identity_digest(execution),"complete_source_closure":complete,
                "complete_source_closure_sha256":identity_digest(complete),
                "source_identity_manifest_sha256":identity_digest({k:"required" for k in sorted(execution)})}
            case={"solver_kind":"forward-driver","model":"planar-7dof","formulation":str(formulation),"track":str(track_path),"argv":[],"input_identity":identity}
            attempt=root/"attempt"; attempt.mkdir()
            control=object.__new__(PlanarVehicleParameters)
            for key,value in {"wheelbase":1.5,"steering_maximum_angle":.5,"steering_maximum_rate":2.,"maximum_pedal_force":1000.,
                "drivetrain":SimpleNamespace(powertrain=SimpleNamespace(energy_storage=SimpleNamespace(initial_state_of_charge=.5)))}.items():
                object.__setattr__(control,key,value)
            manifest=SimpleNamespace(assembly=SimpleNamespace(path=component,model_id="assembly",model_type="vehicle",subtype="test"),
                formulation=SimpleNamespace(model_id="form",model_type="formulation",subtype="test",path=formulation),
                loaded_components=[SimpleNamespace(path=component,role="vehicle",model_id="vehicle",subtype="test")],
                capabilities=[], unused_roles=lambda: [])
            compiled=SimpleNamespace(model=control,manifest=manifest)
            track=SimpleNamespace(lap_length=1.,is_closed=False,path_coordinates=object())
            plan=SimpleNamespace(distance=np.array([0.,1.]),speed=np.array([0.,1.]),curvature=np.zeros(2),is_closed=False)
            envelope=SimpleNamespace(distance=np.array([0.,1.]),curvature=np.zeros(2),corner_speed=np.ones(2),entry_speed=np.ones(2),deceleration=np.ones(1),binding_index=np.zeros(2),is_closed=False,sample=lambda station:None)
            class Plant:
                state_names=("n",)
                def __init__(self,*args): pass
                def initial_state(self, speed): return np.array([0.])
            record_path=root/"record.npz"
            recorded_metadata={}
            def close_sink(metadata):
                recorded_metadata.update(metadata)
                record_path.write_bytes(b"synthetic run record")
                return record_path
            sink=SimpleNamespace(rows=0, close=close_sink)
            result=SimpleNamespace(time=np.array([0.,1.]),state=np.array([[0.,0.]]),dense_sample_state=np.zeros((1,1)),dense_samples=1,
                lap_completed=True,wall_seconds=.01,accepted_steps=1,midpoint_sample_validation_complete=True)
            with mock.patch.object(forward_execute,"compile_formulation",return_value=compiled), \
                 mock.patch.object(forward_execute,"load_track",return_value=track), \
                 mock.patch.object(forward_execute,"evaluate_path",return_value=(None,None,None,np.zeros(2))), \
                 mock.patch.object(forward_execute,"build_speed_plan",return_value=plan) as build_plan, \
                 mock.patch.object(forward_execute,"build_braking_envelope",return_value=envelope), \
                 mock.patch.object(forward_execute,"yaw_transition_speed_caps",return_value=SimpleNamespace(caps=np.array([.5,.5]),curvature_gradient=np.zeros(2),provenance={})), \
                 mock.patch.object(forward_execute,"restrict_braking_envelope",side_effect=lambda env,caps,provenance=None: SimpleNamespace(**{**vars(env),"corner_speed":np.array([.5,.5]),"entry_speed":np.array([.5,.5])})) as restrict_envelope, \
                 mock.patch.object(forward_execute,"PlanarForwardPlant",Plant), \
                 mock.patch.object(forward_execute,"RunRecordSink",return_value=sink), \
                 mock.patch.object(forward_execute,"solve_forward",return_value=result) as solve, \
                 mock.patch("numpy.savez_compressed", side_effect=lambda path, **kwargs: Path(path).write_bytes(b"synthetic envelope")), mock.patch.object(forward_execute,"source_hashes",return_value=frozen), \
                 mock.patch.object(forward_execute,"vehicle_geometry",return_value={}):
                self.assertEqual(forward_execute._run_impl(case,attempt,settings,root),0)
            build_plan.assert_called_once()
            self.assertTrue(build_plan.call_args.kwargs["live_limit"])
            actual_driver=solve.call_args.args[1]
            self.assertIsInstance(actual_driver, __import__("simplified_vehicle_models.solvers.forward_driver",fromlist=["PreviewDriver"]).PreviewDriver)
            self.assertIsNotNone(actual_driver.envelope_policy)
            self.assertEqual(actual_driver.steering_feedforward,"actuator-inverse")
            restrict_envelope.assert_called_once()
            summary=json.loads((attempt/"summary.json").read_text())
            evidence=summary["transient_corner_evidence"]
            self.assertGreater(evidence["plan_speed_changed_nodes"],0)
            self.assertGreater(evidence["envelope_corner_speed_changed_nodes"],0)
            self.assertEqual(evidence["steering_selection"],actual_driver.steering_feedforward)
            self.assertEqual(recorded_metadata["transient_corner_evidence"],evidence)
            solve.assert_called_once()

    def test_selected_physical_mode_builds_physical_policy_and_context(self):
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
        from tools.sim_gui import forward_execute
        from tools.sim_gui.settings import forward_driver_config
        from tools.sim_gui.source_identity import execution_source_identity, identity_digest, combined_source_closure
        settings = RunSettings(forward_longitudinal_mode="physical-reachable-v1")
        self.assertFalse(settings.forward_physical_brake_planning)
        with self.assertRaisesRegex(ValueError, "full vertical"):
            validate(settings, "planar-7dof", None)
        self.assertEqual(forward_driver_config(RunSettings(forward_physical_brake_planning=True))["longitudinal_mode"], "physical-reachable-v1")
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            formulation = Path("models/formulations/iem24_murphy_real_14dof_simple.xml").resolve()
            track_path = Path("models/tracks/fsae_2026_acceleration.xml").resolve()
            compiled = compile_formulation(str(formulation))
            sources = {formulation, track_path, Path(compiled.manifest.assembly.path).resolve()}
            sources.update(Path(item.path).resolve() for item in compiled.manifest.loaded_components)
            frozen = {str(p): "sha256:" + hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
            execution = execution_source_identity()
            complete = combined_source_closure(frozen, execution)
            identity = {"solver_kind": "forward-driver", "source_model": "14dof-simple", "solution_model": "14dof-simple",
                "formulation": str(formulation), "formulation_sha256": hashlib.sha256(formulation.read_bytes()).hexdigest(),
                "track": str(track_path), "track_sha256": hashlib.sha256(track_path.read_bytes()).hexdigest(),
                "settings": settings.to_json(), "settings_sha256": hashlib.sha256(json.dumps(settings.to_json(), sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest(),
                "driver_config": forward_driver_config(settings), "component_source_hashes": frozen,
                "authority_source_closure_sha256": identity_digest(frozen), "execution_source_hashes": execution,
                "execution_source_identity_sha256": identity_digest(execution), "complete_source_closure": complete,
                "complete_source_closure_sha256": identity_digest(complete),
                "source_identity_manifest_sha256": identity_digest({k: "required" for k in sorted(execution)})}
            case = {"solver_kind": "forward-driver", "model": "14dof-simple", "formulation": str(formulation), "track": str(track_path), "argv": [], "input_identity": identity}
            attempt = root / "attempt"; attempt.mkdir()
            plan = SpeedPlan(np.array([0., 75.]), np.array([0., 20.]), np.zeros(2), is_closed=False)
            envelope = SimpleNamespace(distance=plan.distance, is_closed=False, sample=lambda station: None)
            predictor = SimpleNamespace(request_current=lambda *args: None)
            class ReachedSolver(Exception): pass
            with mock.patch.object(forward_execute, "build_speed_plan", return_value=plan), \
                 mock.patch.object(forward_execute, "build_braking_envelope", return_value=envelope), \
                 mock.patch.object(forward_execute, "BrakeAuthorityPredictor", return_value=predictor) as build_predictor, \
                 mock.patch.object(forward_execute, "solve_forward", side_effect=ReachedSolver) as solve:
                with self.assertRaises(ReachedSolver):
                    forward_execute._run_impl(case, attempt, settings, root)
            driver = solve.call_args.args[1]
            self.assertEqual(driver.envelope_policy.settings.mode, "physical-reachable-v1")
            self.assertIs(driver.brake_current_predictor, predictor)
            self.assertTrue(callable(driver.brake_current_context_factory))
            self.assertTrue(callable(driver.brake_planning_context_factory))
            build_predictor.assert_called_once()

    def test_short_vertical_gui_run_records_full_plant_and_incomplete_status(self):
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.results.run_record import load_run_record
        from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
        from simplified_vehicle_models.tracks.profile import load_track
        from tools.sim_gui import forward_execute
        from tools.sim_gui.execute import PreparedCase, finish_case
        from tools.sim_gui.settings import forward_driver_config

        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            case_dir = root / "study" / "cases" / "vertical"
            case_dir.mkdir(parents=True)
            formulation = Path("models/formulations/iem24_murphy_real_14dof_simple.xml").resolve()
            track_path = Path("models/tracks/fsae_2026_acceleration.xml").resolve()
            compiled = compile_formulation(str(formulation))
            source_paths = {formulation, track_path, Path(compiled.manifest.assembly.path).resolve()}
            source_paths.update(Path(item.path).resolve() for item in compiled.manifest.loaded_components)
            frozen = {str(path): "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()
                      for path in source_paths}
            settings = RunSettings(solver_kind="forward-driver", forward_duration=.01,
                                   forward_wall_seconds=120., forward_spacing=.5)
            config = forward_driver_config(settings)
            track = load_track(track_path)
            distance = np.array([0., track.lap_length])
            plan = SpeedPlan(distance, np.array([0., 20.]), np.zeros(2), is_closed=False)
            envelope = SimpleNamespace(distance=distance, curvature=np.zeros(2),
                corner_speed=np.array([20., 20.]), entry_speed=np.array([20., 20.]),
                deceleration=np.array([1.]), binding_index=np.array([0, 0]), is_closed=False,
                maximum_speed=20., sample=lambda station: SimpleNamespace(corner_speed=20.,
                    entry_speed=20., deceleration=1., binding_index=0, binding_station=0.,
                    binding_distance=0., speed_domain_max=20., squared_speed=400.))
            attempt = case_dir / "attempt_1"
            attempt.mkdir()
            identity = {"solver_kind":"forward-driver", "source_model":"14dof-simple",
                "solution_model":"14dof-simple", "formulation":str(formulation),
                "formulation_sha256":hashlib.sha256(formulation.read_bytes()).hexdigest(),
                "track":str(track_path), "track_sha256":hashlib.sha256(track_path.read_bytes()).hexdigest(),
                "settings":settings.to_json(), "settings_sha256":hashlib.sha256(json.dumps(settings.to_json(),sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest(),
                "driver_config":config, "component_source_hashes":frozen}
            case = {"solver_kind":"forward-driver", "model":"14dof-simple", "formulation":str(formulation),
                    "track":str(track_path), "argv":[], "input_identity":identity}
            with mock.patch.object(forward_execute, "build_speed_plan", return_value=plan), \
                 mock.patch.object(forward_execute, "build_braking_envelope", return_value=envelope):
                self.assertEqual(forward_execute._run_impl(case, attempt, settings, root), 0)

            record = load_run_record(attempt / "run_record.npz")
            self.assertEqual(len(record.manifest.state_names), 38 + getattr(compiled.model.suspension.network, "state_count", 0))
            for name in ("heave", "roll", "pitch"):
                self.assertIn(name, record.manifest.state_names)
            self.assertGreater(record.time.size, 1)
            self.assertLessEqual(record.time[-1], settings.forward_duration)
            manifest = record.manifest
            self.assertEqual(manifest.model["model"], "14dof-simple")
            self.assertEqual(manifest.model["formulation_xml"], str(formulation))
            self.assertEqual(manifest.model["plant_class"], "VerticalForwardPlant")
            record_metadata = json.loads((attempt / "inputs.json").read_text())
            self.assertEqual(record_metadata["solution_model"], "14dof-simple")
            self.assertEqual(record_metadata["reduction"], "identity")
            self.assertEqual(record_metadata["source_model"], "14dof-simple")
            self.assertEqual(manifest.model["mechanical_dof"], 14)

            summary = json.loads((attempt / "summary.json").read_text())
            telemetry=summary["task5_telemetry"]
            policy_rows=[json.loads(line) for line in (attempt/"accepted_policy.jsonl").read_text().splitlines()]
            from tools.sim_gui.telemetry_schema import FIELDS, project, schema_document
            self.assertEqual(telemetry["schema"],schema_document("historical-preview-v1"))
            fields = telemetry["schema"]["fields"]
            self.assertEqual(len(telemetry["rows"]),len(policy_rows))
            for row,policy in zip(telemetry["rows"],policy_rows):
                self.assertEqual(row,project(policy))
                self.assertEqual(set(row), set(fields))
            manifest=load_run_record(attempt/"run_record.npz").manifest
            self.assertEqual(manifest.task5_telemetry,telemetry)
            self.assertEqual(summary["status"], "incomplete")
            case_payload = {"settings":settings.to_json(), "driver_config":config, "input_identity":identity}
            (case_dir / "case.json").write_text(json.dumps(case_payload))
            prepared = PreparedCase(case_dir, formulation, case_dir / "vehicle.xml", [],
                model="14dof-simple", timeout_seconds=10., solver_kind="forward-driver")
            finished = finish_case(prepared, attempt, return_code=0, timed_out=False,
                                   cancelled=False, wall_seconds=.1, command=[])
            self.assertEqual(finished["status"], "failed")
            self.assertIn(json.loads((attempt / "parent.json").read_text())["status"],
                          {"Child_Incomplete", "Child_Invalid_Artifacts"})

    def test_forward_vertical_source_resolves_to_vertical(self):
        from tools.sim_gui.settings import solution_model_for
        from simplified_vehicle_models.compiler.compile import compile_formulation
        settings = RunSettings(solver_kind="forward-driver")
        self.assertEqual(solution_model_for(settings, "14dof-simple"), "14dof-simple")
        self.assertEqual(solution_model_for(settings, "planar-7dof"), "planar-7dof")
        compiled = compile_formulation("models/formulations/iem24_murphy_real_14dof_simple.xml").model
        from simplified_vehicle_models.formulations.vertical_14dof import VerticalVehicleParameters
        from simplified_vehicle_models.solvers.vertical_forward import vertical_plant_class
        self.assertIsInstance(compiled, VerticalVehicleParameters)
        from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
        plan = SpeedPlan(np.array([0., 10.]), np.array([5., 5.]), np.zeros(2), is_closed=False)
        plant = vertical_plant_class(compiled)(compiled, plan)
        self.assertEqual(plant.state_size, 38 + plant.network_states)
        self.assertIn("roll", plant.state_names)
        initial = plant.initial_state(0.)
        self.assertEqual(initial.shape, (plant.state_size,))
        self.assertTrue(np.all(np.isfinite(initial)))

    def test_defaults_roundtrip_and_legacy_default_preserved(self):
        self.assertEqual(RunSettings().solver_kind, "forward-driver")
        settings = RunSettings(solver_kind="forward-driver")
        self.assertEqual(RunSettings.from_json(settings.to_json()), settings)
        self.assertEqual(settings.forward_duration, 300.)
        self.assertEqual(settings.forward_wall_seconds, 600.)
        self.assertEqual(solution_model_for(settings, "14dof-simple"), "14dof-simple")
        with self.assertRaisesRegex(ValueError, "point-mass is unsupported"):
            solution_model_for(settings, "point-mass")

    def test_strict_fields_and_ranges(self):
        for key, value in (("forward_deceleration_margin", 1.01), ("forward_spacing", .6),
                           ("forward_wall_seconds", 601.), ("forward_speed_points", True),
                           ("forward_speed_points", 1), ("forward_duration", float("inf")),
                           ("forward_duration", True), ("forward_feedforward_distance", 0)):
            with self.subTest(key=key, value=value):
                settings = RunSettings(solver_kind="forward-driver", **{key: value})
                with self.assertRaises(ValueError):
                    validate(settings, "planar-7dof", Path("unused"))
        with self.assertRaisesRegex(ValueError, "retired yaw"):
            RunSettings.from_json({"yaw_map": "legacy"})

    def test_invalid_before_compile(self):
        from tools.sim_gui import execute
        from tools.sim_gui.study import CaseSpec, StudyDefinition
        settings = RunSettings(solver_kind="forward-driver", forward_deceleration_margin=1.1)
        definition = StudyDefinition("bad", "models/formulations/iem24_murphy_real_planar_7dof.xml",
            "models/tracks/2024_michigan_endurance_closed.xml", "planar-7dof", {}, settings,
            "one-at-a-time", ())
        case = CaseSpec("bad", "bad", "planar-7dof", definition.track, {}, {})
        with mock.patch("tools.sim_gui.execute.compile_formulation") as compile_model, \
             mock.patch("tools.sim_gui.variant.write_variant") as write_variant:
            with self.assertRaises(ValueError):
                execute.prepare_case(Path("/tmp/unused"), definition, case)
            compile_model.assert_not_called()
            write_variant.assert_not_called()

    def test_run_case_dispatches_to_forward_adapter(self):
        from tools.sim_gui.run_case import run_case
        settings = RunSettings(solver_kind="forward-driver")
        with tempfile.TemporaryDirectory() as folder:
            case_dir = Path(folder) / "case"
            case_dir.mkdir()
            # Attempts are contained by the case directory; exercise dispatch at a legitimate child path.
            attempt = case_dir / "attempt"
            payload = {"solver_kind": "forward-driver", "settings": settings.to_json(),
                "model": "planar-7dof", "track": "unused", "formulation": "unused.xml",
                "input_identity": {}}
            (case_dir / "case.json").write_text(json.dumps(payload))
            with mock.patch("tools.sim_gui.forward_execute._run_impl", side_effect=RuntimeError("synthetic child failure")):
                self.assertEqual(run_case(case_dir, attempt), 4)
            failure = json.loads((attempt / "summary.json").read_text())
            self.assertEqual(failure["message"], "synthetic child failure")
            self.assertIn("accepted_policy_prefix_rows", failure)

    def test_prepared_identity_rejects_mutated_helper_and_calibration_before_compile(self):
        from tools.sim_gui.forward_execute import run
        from tools.sim_gui import source_identity
        from tools.sim_gui.settings import forward_driver_config
        settings=RunSettings(solver_kind="forward-driver",forward_physical_brake_planning=True,forward_duration=.001)
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); (root/"tools/sim_gui").mkdir(parents=True)
            helper=root/"tools/sim_gui/source_identity.py"; helper.write_text("identity-helper-original")
            form=root/"form.xml"; track=root/"track.xml"; calibration=root/"tire-calibration.xml"
            form.write_text("formulation-v1"); track.write_text("track-v1"); calibration.write_text("calibration-v1")
            required=("tools/sim_gui/source_identity.py",)
            with mock.patch.object(source_identity,"REPOSITORY_ROOT",root), mock.patch.object(source_identity,"_REQUIRED",required):
                sources=source_identity.execution_source_identity(root=root)
                component={str(p.resolve()):"sha256:"+hashlib.sha256(p.read_bytes()).hexdigest() for p in (form,track,calibration)}
                complete=source_identity.combined_source_closure(component,sources,root=root)
                prepared={"solver_kind":"forward-driver","source_model":"14dof-simple","solution_model":"14dof-simple",
                    "formulation":str(form.resolve()),"formulation_sha256":hashlib.sha256(form.read_bytes()).hexdigest(),
                    "track":str(track.resolve()),"track_sha256":hashlib.sha256(track.read_bytes()).hexdigest(),
                    "settings":settings.to_json(),"settings_sha256":hashlib.sha256(json.dumps(settings.to_json(),sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest(),
                    "driver_config":forward_driver_config(settings),"component_source_hashes":component,
                    "authority_source_closure_sha256":source_identity.identity_digest(component),"execution_source_hashes":sources,
                    "execution_source_identity_sha256":source_identity.identity_digest(sources),"complete_source_closure":complete,
                    "complete_source_closure_sha256":source_identity.identity_digest(complete),
                    "source_identity_manifest_sha256":source_identity.identity_digest({k:"required" for k in sorted(sources)})}
                case={"model":"14dof-simple","formulation":str(form),"track":str(track),"argv":[],"input_identity":prepared}
                for label,target in (("helper",helper),("compiled-calibration",calibration)):
                    target.write_text(target.read_text()+"-mutated-after-prepare")
                    attempt=root/("attempt-"+label); attempt.mkdir()
                    with mock.patch("tools.sim_gui.forward_execute.execution_source_identity",
                                    side_effect=lambda: source_identity.execution_source_identity(root=root)), \
                         mock.patch("tools.sim_gui.forward_execute.compile_formulation") as compile_model:
                        self.assertEqual(run(case,attempt,settings,root),4)
                        compile_model.assert_not_called()
                    self.assertIn("identity",json.loads((attempt/"summary.json").read_text())["message"])

    def test_execution_source_identity_changes_and_missing_required_fail_before_compile(self):
        from tools.sim_gui.forward_execute import run
        from tools.sim_gui.source_identity import execution_source_identity, identity_digest, combined_source_closure
        settings = RunSettings(solver_kind="forward-driver", forward_physical_brake_planning=True)
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); form=root/"f.xml"; track=root/"t.xml"; component=root/"c.xml"
            for p in (form,track,component): p.write_text(p.name)
            comps={str(component.resolve()):"sha256:"+hashlib.sha256(component.read_bytes()).hexdigest()}
            execution=execution_source_identity()
            full=combined_source_closure(comps,execution)
            case={"solver_kind":"forward-driver","model":"planar-7dof","formulation":str(form),"track":str(track),"argv":[],
                "input_identity":{"solver_kind":"forward-driver","source_model":"planar-7dof","solution_model":"planar-7dof",
                "formulation":str(form.resolve()),"formulation_sha256":hashlib.sha256(form.read_bytes()).hexdigest(),
                "track":str(track.resolve()),"track_sha256":hashlib.sha256(track.read_bytes()).hexdigest(),
                "settings":settings.to_json(),"settings_sha256":hashlib.sha256(json.dumps(settings.to_json(),sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest(),
                "driver_config":__import__("tools.sim_gui.settings",fromlist=["forward_driver_config"]).forward_driver_config(settings),
                "component_source_hashes":comps,"authority_source_closure_sha256":identity_digest(comps),
                "execution_source_hashes":execution,"execution_source_identity_sha256":identity_digest(execution),
                "complete_source_closure":full,"complete_source_closure_sha256":identity_digest(full),
                "source_identity_manifest_sha256":identity_digest({k:"required" for k in sorted(execution)})}}
            # Content changes after preparation are detected by comparing fresh hashes,
            # without mutating the prepared identity object.
            changed=dict(execution); changed["tools/sim_gui/settings.py"]="sha256:"+"0"*64
            self.assertNotEqual(changed,execution)
            attempt=root/"attempt"; attempt.mkdir()
            changed_case=json.loads(json.dumps(case)); changed_case["input_identity"]["execution_source_hashes"]=changed
            with mock.patch("tools.sim_gui.forward_execute.compile_formulation") as compile_model:
                self.assertEqual(run(changed_case,attempt,settings,root),4); compile_model.assert_not_called()
            self.assertIn("execution source identity mismatch",json.loads((attempt/"summary.json").read_text())["message"])
            attempt2=root/"attempt2"; attempt2.mkdir()
            with mock.patch("tools.sim_gui.source_identity._REQUIRED", execution_source_identity.__globals__["_REQUIRED"] + ("missing/required.py",)):
                with self.assertRaises(FileNotFoundError):
                    execution_source_identity()

    def test_planar_physical_context_rejected_before_plant_integration(self):
        from tools.sim_gui.forward_execute import _run_impl
        from tools.sim_gui.source_identity import execution_source_identity, identity_digest, combined_source_closure
        from tools.sim_gui.settings import forward_driver_config
        settings=RunSettings(solver_kind="forward-driver",forward_physical_brake_planning=True)
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); form=root/"f.xml"; track=root/"t.xml"; component=root/"c.xml"
            for p in (form,track,component): p.write_text(p.name)
            form.write_text("form")
            track.write_text("track")
            comps={str(component.resolve()):"sha256:"+hashlib.sha256(component.read_bytes()).hexdigest(),
                str(form.resolve()):"sha256:"+hashlib.sha256(form.read_bytes()).hexdigest(),
                str(track.resolve()):"sha256:"+hashlib.sha256(track.read_bytes()).hexdigest()}
            sources=execution_source_identity(); complete=combined_source_closure(comps,sources)
            identity={"solver_kind":"forward-driver","source_model":"planar-7dof","solution_model":"planar-7dof",
                "formulation":str(form.resolve()),"formulation_sha256":hashlib.sha256(form.read_bytes()).hexdigest(),
                "track":str(track.resolve()),"track_sha256":hashlib.sha256(track.read_bytes()).hexdigest(),"settings":settings.to_json(),
                "settings_sha256":hashlib.sha256(json.dumps(settings.to_json(),sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest(),
                "driver_config":forward_driver_config(settings),"component_source_hashes":comps,
                "authority_source_closure_sha256":identity_digest(comps),"execution_source_hashes":sources,
                "execution_source_identity_sha256":identity_digest(sources),"complete_source_closure":complete,
                "complete_source_closure_sha256":identity_digest(complete),
                "source_identity_manifest_sha256":identity_digest({k:"required" for k in sorted(sources)})}
            case={"model":"planar-7dof","formulation":str(form),"track":str(track),"argv":[],"input_identity":identity}
            attempt=root/"attempt"; attempt.mkdir()
            with mock.patch("tools.sim_gui.forward_execute.compile_formulation",return_value=SimpleNamespace(
                model=object(),manifest=SimpleNamespace(loaded_components=[],assembly=SimpleNamespace(path=component)))):
                # Full graph compilation is not part of this rejection test; load path is patched to typed planar.
                from simplified_vehicle_models.parameters.vehicle import PlanarVehicleParameters
                model=object.__new__(PlanarVehicleParameters)
                object.__setattr__(model,"wheelbase",1.5)
                object.__setattr__(model,"steering_maximum_angle",.5)
                object.__setattr__(model,"steering_maximum_rate",2.)
                object.__setattr__(model,"maximum_pedal_force",1000.)
                object.__setattr__(model,"drivetrain",SimpleNamespace(drivetrain=SimpleNamespace(transmission=SimpleNamespace(driven_wheels=("rl","rr"),gear_efficiency=1.)),powertrain=SimpleNamespace(energy_storage=SimpleNamespace(initial_state_of_charge=.5))))
                object.__setattr__(model,"motor",SimpleNamespace(unit_count=2,gear_ratio=np.ones(2)))
                manifest=SimpleNamespace(loaded_components=[],assembly=SimpleNamespace(path=component))
                with mock.patch("tools.sim_gui.forward_execute.compile_formulation",return_value=SimpleNamespace(model=model,manifest=manifest)), \
                     mock.patch("tools.sim_gui.forward_execute.load_track",return_value=SimpleNamespace(lap_length=1.,is_closed=True,path_coordinates=object())), \
                     mock.patch("tools.sim_gui.forward_execute.evaluate_path",return_value=(None,None,None,np.zeros(2))), \
                     mock.patch("tools.sim_gui.forward_execute.build_speed_plan",return_value=SimpleNamespace(distance=np.array([0.,1.]),speed=np.array([5.,5.]),curvature=np.zeros(2),is_closed=True)), \
                     mock.patch("tools.sim_gui.forward_execute.build_braking_envelope",return_value=SimpleNamespace(distance=np.array([0.,1.]),curvature=np.zeros(2),corner_speed=np.ones(2)*5,entry_speed=np.ones(2)*5,deceleration=np.ones(1),binding_index=np.zeros(2),is_closed=True)):
                    with self.assertRaisesRegex(ValueError,"requires the full vertical 14DoF"):
                        _run_impl(case,attempt,settings,root)

    def test_component_identity_rejected_before_compile(self):
        from tools.sim_gui.forward_execute import run
        settings = RunSettings(solver_kind="forward-driver")
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            formulation, track, component = (root / name for name in ("formulation.xml", "track.xml", "component.xml"))
            formulation.write_text("form")
            track.write_text("track")
            component.write_text("original")
            source_identity = {str(component): "sha256:" + hashlib.sha256(component.read_bytes()).hexdigest()}
            case = {"solver_kind": "forward-driver", "model": "planar-7dof",
                "formulation": str(formulation), "track": str(track), "argv": [],
                "input_identity": {"component_source_hashes": source_identity}}
            attempt = root / "attempt"
            attempt.mkdir()
            component.write_text("mutated-after-prepare")
            with mock.patch("tools.sim_gui.forward_execute.compile_formulation") as compile_model:
                self.assertEqual(run(case, attempt, settings, Path(folder)), 4)
                compile_model.assert_not_called()
            failure = json.loads((attempt / "summary.json").read_text())
            self.assertIn("immutable component source mismatch", failure["message"])

    def test_identity_rejected_before_compile(self):
        from tools.sim_gui.forward_execute import run
        settings = RunSettings(solver_kind="forward-driver")
        case = {"solver_kind": "forward-driver", "model": "planar-7dof", "formulation": "missing.xml",
                "track": "missing-track.xml", "argv": [], "input_identity": {}}
        with tempfile.TemporaryDirectory() as folder, mock.patch(
                "tools.sim_gui.forward_execute.compile_formulation") as compile_model:
            attempt = Path(folder) / "attempt"
            attempt.mkdir()
            self.assertEqual(run(case, attempt, settings, Path(folder)), 4)
            compile_model.assert_not_called()


class ForwardFinishTests(unittest.TestCase):
    def test_contract_requires_counts_and_real_record(self):
        import numpy as np
        from tools.sim_gui.execute import PreparedCase, finish_case
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            case_dir = root / "study" / "cases" / "case_000"
            attempt = case_dir / "attempt_1"
            attempt.mkdir(parents=True)
            track = Path("models/tracks/2024_michigan_endurance_closed.xml").resolve()
            formulation = (case_dir / "model" / "formulation.xml")
            formulation.parent.mkdir(parents=True)
            formulation.write_text("selected-formulation")
            settings = RunSettings(solver_kind="forward-driver")
            from tools.sim_gui.settings import forward_driver_config
            settings_hash = hashlib.sha256(json.dumps(settings.to_json(), sort_keys=True,
                separators=(",", ":"), allow_nan=False).encode()).hexdigest()
            identity = {"solver_kind": "forward-driver", "source_model": "planar-7dof",
                "solution_model": "planar-7dof", "track": str(track),
                "track_sha256": hashlib.sha256(track.read_bytes()).hexdigest(),
                "formulation": str(formulation.resolve()),
                "formulation_sha256": hashlib.sha256(formulation.read_bytes()).hexdigest(),
                "settings": settings.to_json(), "settings_sha256": settings_hash,
                "driver_config": forward_driver_config(settings), "yaw_map": None,
                "component_source_hashes": {}}
            (case_dir / "case.json").write_text(json.dumps({"input_identity": identity, "settings": settings.to_json(),
                "driver_config": identity["driver_config"]}))
            from simplified_vehicle_models.results.run_record import RunRecorder, sha256_of
            source_base = root / "study"
            source_base.mkdir(parents=True, exist_ok=True)
            source_a, source_b, source_c = (source_base / name for name in ("source-a.xml", "source-b.xml", "source-c.xml"))
            for index, source in enumerate((source_a, source_b, source_c)):
                source.write_text(f"source-{index}")
            source_hashes = {str(source.resolve()): sha256_of(source) for source in (source_a, source_b, source_c)}
            identity["component_source_hashes"] = source_hashes
            (case_dir / "case.json").write_text(json.dumps({"input_identity": identity,
                "settings": settings.to_json(), "driver_config": identity["driver_config"]}))
            record = attempt / "run_record.npz"
            recorder = RunRecorder(record, state_names=("s",), state_units=("m",),
                control_names=(), channel_names=())
            recorder.append(0., 0, [0.])
            recorder.append(1., 1, [1.])
            recorder.append(2., 2, [2.])
            recorder.close({"sources": source_hashes})
            digest = hashlib.sha256(record.read_bytes()).hexdigest()
            envelope = attempt / "envelope_arrays.npz"
            np.savez(envelope, distance=np.arange(3.), curvature=np.zeros(3), corner_speed=np.ones(3),
                     entry_speed=np.ones(3), deceleration=np.ones(2), binding_index=np.zeros(3))
            envelope_digest = hashlib.sha256(envelope.read_bytes()).hexdigest()
            policy = attempt / "accepted_policy.jsonl"
            policy.write_text("{}\n{}\n{}\n")
            policy_digest = hashlib.sha256(policy.read_bytes()).hexdigest()
            inputs = {**identity, "formulation": str(formulation.resolve()), "track": str(track),
                "formulation_sha256": identity["formulation_sha256"], "track_sha256": identity["track_sha256"],
                "run_record_sha256": digest, "completed_forward_result": True, "lap_completed": True,
                "settings": settings.to_json(), "driver_config": identity["driver_config"],
                "settings_sha256": settings_hash,
                "midpoint_sample_validation_complete": True, "accepted_states_validated": 3,
                "dense_samples_validated": 2, "accepted_policy_file": "accepted_policy.jsonl",
                "accepted_policy_rows": 3, "accepted_policy_sha256": policy_digest,
                "source_hashes": source_hashes}
            (attempt / "inputs.json").write_text(json.dumps(inputs))
            summary = {"status": "completed", "lap_completed": True, "midpoint_sample_validation_complete": True,
                "accepted_steps": 2, "accepted_states_validated": 3, "midpoint_samples": 2,
                "accepted_policy_rows": 3, "forward_driver_config": identity["driver_config"],
                "dense_samples_validated": 2, "validation_counts": {"accepted_states_validated": 3, "dense_probes_validated": 2},
                "settings_sha256": settings_hash, "run_record_sha256": digest,
                "sampled_tracking_pass": False, "envelope_arrays_sha256": envelope_digest,
                "accepted_policy_sha256": policy_digest, "accepted_policy_rows": 3,
                "source_hashes": inputs["source_hashes"]}
            (attempt / "summary.json").write_text(json.dumps(summary))
            prepared = PreparedCase(case_dir, formulation, case_dir / "model/vehicle.xml", [],
                                    model="planar-7dof", timeout_seconds=660, solver_kind="forward-driver")
            args = dict(return_code=0, timed_out=False, cancelled=False, wall_seconds=1, command=[])
            self.assertEqual(finish_case(prepared, attempt, **args)["status"], "completed")
            self.assertFalse(summary["sampled_tracking_pass"], "tracking evidence is not the completion gate")
            source_b.write_text("mutated")
            self.assertEqual(finish_case(prepared, attempt, **args)["status"], "failed")
            source_b.write_text("source-1")
            self.assertEqual(finish_case(prepared, attempt, **args)["status"], "completed")
            inputs["accepted_states_validated"] = True
            (attempt / "inputs.json").write_text(json.dumps(inputs))
            self.assertEqual(finish_case(prepared, attempt, **args)["status"], "failed")
            inputs["accepted_states_validated"] = 3
            (attempt / "inputs.json").write_text(json.dumps(inputs))
            summary["midpoint_samples"] = None
            (attempt / "summary.json").write_text(json.dumps(summary))
            self.assertEqual(finish_case(prepared, attempt, **args)["status"], "failed")

    def test_synthetic_forward_solver_keeps_requested_applied_stages(self):
        from simplified_vehicle_models.solvers.forward_driver import (DriverCommand, DriverObservation,
            RateSystem, SpeedPlan, solve_forward)
        from simplified_vehicle_models.solvers.braking_envelope import BrakingEnvelope
        from simplified_vehicle_models.solvers.envelope_driver import EnvelopeDriverSettings, EnvelopePolicy
        from tools.sim_gui import forward_execute
        plan = SpeedPlan(np.array([0., 100.]), np.array([5., 5.]), np.zeros(2), is_closed=False)
        envelope = BrakingEnvelope(np.array([0., 100.]), np.zeros(2), np.array([60., 60.]),
            np.array([60., 60.]), np.array([4.]), np.array([0, 0]), False, 60.,
            {"method": "synthetic", "segment_speed_domain_max": (60.,)})
        class Plant:
            state_size = 1
            def observe(self, state):
                return DriverObservation(float(state[0]), 0., 0., 5., 0., 0.)
            def evaluate(self, time, state, command):
                return RateSystem(np.array([max(command.throttle, 0.) - command.brake_force * .001]))
        from simplified_vehicle_models.solvers.forward_driver import PreviewDriver
        driver = PreviewDriver(plan, 1., .5, 2., 1000., envelope_policy=EnvelopePolicy(envelope,
            EnvelopeDriverSettings(feedforward_distance_m=2.)), pedal_travel_time=.02)
        samples = []
        result = solve_forward(Plant(), driver, np.array([0.]), duration=.04, wall_seconds=2.,
            maximum_wall_seconds=2., maximum_step=.01, initial_pedal=0., step_sink=samples.append)
        self.assertTrue(result.midpoint_sample_validation_complete)
        self.assertTrue(samples)
        sample = samples[-1]
        self.assertIsNotNone(sample.requested_command)
        self.assertIsNotNone(sample.mapped_command)
        self.assertIsNotNone(sample.policy_diagnostics)
        self.assertIsNotNone(sample.pedal)


class ForwardWidgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PyQt6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def test_visibility_and_roundtrip_has_only_supported_workflows(self):
        from tools.sim_gui.qt.settings_tab import SettingsTab
        widget = SettingsTab()
        widget.show()
        self.app.processEvents()
        self.assertEqual([widget.solver_kind.itemData(i) for i in range(widget.solver_kind.count())],
                         ["forward-driver", "point-mass"])
        self.assertEqual(widget.settings().solver_kind, "forward-driver")
        self.assertTrue(widget.forward_duration.isVisible())
        widget.forward_feedforward_distance.setValue(2.25)
        settings = widget.settings()
        self.assertEqual(settings.solver_kind, "forward-driver")
        self.assertEqual(RunSettings.from_json(settings.to_json()), settings)
        widget.solver_kind.setCurrentIndex(widget.solver_kind.findData("point-mass"))
        self.app.processEvents()
        self.assertTrue(widget.point_mass_duration.isVisible())
        self.assertTrue(widget.forward_duration.isHidden())
        widget.deleteLater()


if __name__ == "__main__":
    unittest.main()
