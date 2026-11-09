"""Portable bounded transition-runner selection and evidence contract tests."""
import unittest
import json
import tempfile
from pathlib import Path
import numpy as np

from tools.run.validate_hybrid_transitions import CASE_IDS, _make_case, prospective_cap_event_station, transition_gate, validate_runner_config, trace_metrics, run_case, VALIDATION_RESOURCE_CONTRACT_VERSION, MAX_PERMITTED_WALL_SECONDS
from tools.run.validate_hybrid_transitions import _build_terminal_stop_fixture, seam_crossing_evidence, seam_continuity_evidence, terminal_stop_gate


class HybridTransitionRunnerTests(unittest.TestCase):
    def test_atomic_json_is_strict_and_preserves_existing_destination(self):
        from tools.run.validate_hybrid_transitions import _atomic_json
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"strict.json"
            with self.assertRaisesRegex(ValueError,"Out of range float"):
                _atomic_json(path,{"invalid":float("nan")})
            self.assertFalse(path.exists())
            _atomic_json(path,{"valid":1.25,"status":"NOT_APPLICABLE"})
            parsed=json.loads(path.read_text(),parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
            self.assertEqual(parsed,{"valid":1.25,"status":"NOT_APPLICABLE"})
            original=path.read_bytes()
            with self.assertRaises(FileExistsError): _atomic_json(path,{"valid":float("inf")})
            self.assertEqual(path.read_bytes(),original)

    def test_stop_postprocess_strict_json_marks_irrelevant_event_overshoot_na(self):
        # Exercises the production postprocessor/atomic serializer without a solver run.
        from dataclasses import replace
        from types import SimpleNamespace
        from tools.run.validate_hybrid_transitions import _postprocess_forward_result
        from tools.run.validate_hybrid_driver import FORMULATION
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.solvers.forward_driver import DriverCommand, ForwardResult, StepSample
        from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
        from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant
        model=compile_formulation(str(FORMULATION)).model
        steering=SpeedPlan(np.array([0.,600.]),np.array([3.,3.]),np.zeros(2),False)
        plant=VerticalForwardPlant(model,steering); plant.minimum_contact_load=100.
        states=np.stack([plant.initial_state(3.),plant.initial_state(.009)])
        states[:,0]=[0.,2.]; ts=np.array([0.,1.]); result=ForwardResult(ts,states.T,False,.1,0,"accepted stop")
        diagnostic=SimpleNamespace(candidate_count=0,rollout_samples=0,phase="coast",status="nominal",event_crossed=None,event_crossing_speed_mps=None,event_crossing_time_s=None,event_speed_cap_mps=None)
        samples=[StepSample(i,float(t),x.copy(),np.zeros(38),DriverCommand(0.,0.,0.),0.,DriverCommand(0.,0.,0.),policy_diagnostics=diagnostic) for i,(t,x) in enumerate(zip(ts,states))]
        plan=SimpleNamespace(distance_m=np.array([0.,2.]),corner_speed_cap_mps=np.array([3.,.01]))
        with tempfile.TemporaryDirectory() as tmp:
            row=_postprocess_forward_result(result,samples,plant,plan,case_id="stop-neutral",v0=3.,target=.01,pedal=0.,event=5.,duration=1.,forecast_slabs=30,source_before={"x":"a"},source_after={"x":"a"},identity={"provider_identity":"p","provider_model_hash":"m","provider_source_closure_hash":"c","plan_identity":"plan"},output_dir=Path(tmp))
            payload=(Path(tmp)/"stop-neutral.json").read_text()
            parsed=json.loads(payload,parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
            self.assertIn(row["status"],("PASS","FAIL"))
            self.assertIsNone(row["max_post_event_overshoot_mps"])
            self.assertEqual(parsed["max_post_event_overshoot_mps"],None)
            self.assertEqual(parsed["max_post_event_overshoot_status"],"NOT_APPLICABLE")
            self.assertGreaterEqual(parsed["max_stop_cap_overshoot_mps"],0.)
            self.assertEqual(parsed["trace_metrics"]["arrival_index"],1)
            self.assertIsNone(parsed["trace_metrics"]["distance_error_percent"])
            self.assertIn("NOT_APPLICABLE",payload)

    def test_real_record_postprocessor_success_and_failure_checkpoint_smoke(self):
        from dataclasses import replace
        from unittest.mock import patch
        from tools.run.validate_hybrid_transitions import _postprocess_forward_result, _atomic_npz, run_case
        from tools.run import validate_hybrid_transitions as runner
        from tools.run.validate_hybrid_transitions import FORMULATION, ROOT
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.solvers.forward_driver import AppliedPowertrainDiagnostics, DriverCommand, DriverObservation, ForwardResult, StepSample
        from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityPredictor
        from simplified_vehicle_models.solvers.envelope_driver import PedalPolicyContext
        from simplified_vehicle_models.solvers.hybrid_driver import HybridLongitudinalPolicy, HybridPolicyConfig
        from simplified_vehicle_models.solvers.hybrid_speed_plan import build_hybrid_speed_plan
        from simplified_vehicle_models.solvers.longitudinal_capability import CurrentVehicleCapability, CapabilityQuery
        from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
        from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant, SOC_INDEX
        source=ROOT/".superpowers/sdd/hybrid-implementation-plan/task5-fixture-fix-20261106-signed/actual-3-p1.npz"
        with np.load(source,allow_pickle=False) as src: states=src["state"][[0,len(src["state"])//2,-1]].copy()
        model=compile_formulation(str(FORMULATION)).model; stations=np.linspace(0,600,601); caps=np.full_like(stations,40.); steering=SpeedPlan(stations,caps,np.zeros_like(stations),False); plant=VerticalForwardPlant(model,steering)
        obs=[plant.observe(row) for row in states]; kin=[plant.hybrid_kinematic_context(row) for row in states]
        self.assertEqual(states.shape,(3,38)); self.assertTrue(all(np.isfinite(o.speed) for o in obs)); self.assertTrue(all(hasattr(k,"vy_dot_estimate_mps2") for k in kin))
        provider=CurrentVehicleCapability(model.planar,wheel_radii_m=(.29,)*4,speed_grid_mps=(.005,.01,5.,10.,20.,22.,25.,40.),curvature_grid_inv_m=(-.001,0.,.001),soc_grid=(.8,.9),source_label="task5-transition-actual-38state-v4",calibration_label="compiled-IEM-current")
        stations=np.linspace(0.,600.,601); caps=np.full_like(stations,22.); caps[stations>=99.]=10.
        event_meta=runner.prospective_cap_event_station(provider,initial_speed_mps=20.,peak_speed_mps=22.,target_speed_mps=10.,curvature_inv_m=0.,soc=float(states[0,SOC_INDEX])); event=float(np.ceil(event_meta["event_station_m"])); caps[stations>=event]=10.
        plan=build_hybrid_speed_plan(stations,np.zeros_like(stations),caps,provider,initial_speed_mps=20.,storage_soc=float(states[0,SOC_INDEX]),is_closed=False)
        pc=PedalPolicyContext(1/.175,1/.175,.02,model.planar.maximum_pedal_force,.85,1/.175,1/.175); predictor=BrakeAuthorityPredictor(model.planar,maximum_abs_curvature_inv_m=.001,minimum_speed_mps=.005,maximum_speed_mps=40.)
        policy=HybridLongitudinalPolicy(plan,provider,predictor,HybridPolicyConfig(maximum_brake_force_n=model.planar.maximum_pedal_force,pedal_context=pc,slabs=30))
        cmd=policy.decide(observation=obs[1],pedal_position=float(.25),kinematics=kin[1],soc=float(states[1,SOC_INDEX]),brake_context=plant.brake_authority_context(states[1],predictor=predictor))
        self.assertGreaterEqual(cmd.decision.candidate_count,0)
        result=ForwardResult(np.asarray([0.,1.,2.]),states.T.copy(),False,.1,0,"accepted-hold")
        plant.minimum_contact_load=1.
        samples=[]
        for i,row in enumerate(states):
            d=cmd.decision
            samples.append(StepSample(i,float(i),row.copy(),np.zeros(38),DriverCommand(cmd.throttle,cmd.brake_force_n,0.),.25,DriverCommand(cmd.throttle,cmd.brake_force_n,0.),policy_diagnostics=d,applied_powertrain_diagnostics=AppliedPowertrainDiagnostics((11.25,-22.5,33.75,-44.0)) if i==1 else None))
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp); saved=_postprocess_forward_result(result,samples,plant,plan,case_id="20-to-10-p-plus1",v0=20.,target=10.,pedal=.25,event=0.,duration=2.,forecast_slabs=30,source_before={"core":"same"},source_after={"core":"same"},identity={"provider_identity":provider.identity,"provider_model_hash":provider.model_hash,"provider_source_closure_hash":provider.source_closure_hash,"plan_identity":plan.identity},output_dir=out)
            self.assertIn("status",saved)
            with np.load(out/"20-to-10-p-plus1-raw.npz",allow_pickle=False) as archive:
                self.assertEqual(archive["state"].shape,(3,38)); self.assertEqual(archive["phase"].dtype.kind,"U"); self.assertFalse(bool(archive["contact_load_trace_available"]))
                self.assertEqual(str(archive["stop_oracle_json"]),"")
            target_path=out/"atomic.npz"; _atomic_npz(target_path,values=np.arange(3)); original=target_path.read_bytes()
            with self.assertRaises(FileExistsError): _atomic_npz(target_path,values=np.arange(4))
            self.assertEqual(target_path.read_bytes(),original)
            mock_z={"v0":20.,"target":10.,"pedal":.25,"event":0.,"plan":plan,"provider":provider,"preflight":None,"event_meta":None,"plant":plant,"driver":None,"initial":states[0]}
            fake_result=ForwardResult(np.asarray([0.,1.,2.]),states.T.copy(),False,.1,7,"mock accepted result")
            with tempfile.TemporaryDirectory() as run_tmp:
                case_dir=Path(run_tmp)/"case"
                def fake_solve(*args,**kwargs):
                    for accepted in samples: kwargs["step_sink"](accepted)
                    return fake_result
                injected=RuntimeError("injected real-wrapper postprocess")
                with patch("tools.run.validate_hybrid_transitions._assemble",return_value=mock_z), patch("tools.run.validate_hybrid_transitions.solve_forward",side_effect=fake_solve), patch("tools.run.validate_hybrid_transitions._postprocess_forward_result",side_effect=injected):
                    with self.assertRaisesRegex(RuntimeError,"injected real-wrapper postprocess") as caught:
                        run_case("20-to-10-p-plus1",case_dir,duration=2.,wall_seconds=90.,forecast_slabs=30)
                checkpoint=case_dir/"20-to-10-p-plus1-solver-return.npz"
                failure=case_dir/"20-to-10-p-plus1-postprocessing-failure.json"
                self.assertIs(caught.exception.__cause__,injected)
                self.assertTrue(checkpoint.exists()); self.assertTrue(failure.exists())
                failure_row=json.loads(failure.read_text()); self.assertEqual(failure_row["stage"],"postprocessing"); self.assertEqual(failure_row["status"],"FAILED_POSTPROCESSING"); self.assertIn(checkpoint.name,failure_row["solver_checkpoint"])
                self.assertFalse((case_dir/"20-to-10-p-plus1.json").exists())
                with np.load(checkpoint,allow_pickle=False) as raw:
                    self.assertEqual(raw["accepted_state"].shape,(3,38)); self.assertEqual(raw["sample_index"].tolist(),[0,1,2]); self.assertEqual(raw["accepted_time"].tolist(),[0.,1.,2.]); self.assertEqual(raw["pedal"].tolist(),[.25]*3); self.assertTrue(raw["requested_command_available"].all()); self.assertTrue(raw["applied_command_available"].all()); self.assertEqual(raw["candidate_count"].shape,(3,)); self.assertEqual(raw["rollout_samples"].shape,(3,)); self.assertEqual(raw["event_crossed"].shape,(3,)); self.assertEqual(raw["forward_state"].shape,(38,3)); self.assertEqual(raw["forward_evaluations"],7); checkpoint_identity=json.loads(str(raw["identity"])); self.assertEqual(checkpoint_identity["actual_requested_wall_seconds"],90.); self.assertEqual(checkpoint_identity["validation_resource_contract_version"],VALIDATION_RESOURCE_CONTRACT_VERSION); self.assertEqual(float(raw["actual_requested_wall_seconds"]),90.); self.assertEqual(str(raw["validation_resource_contract_version"]),VALIDATION_RESOURCE_CONTRACT_VERSION); self.assertEqual(raw["policy_diagnostics_available"].tolist(),[True]*3); self.assertEqual(raw["applied_powertrain_diagnostics_available"].tolist(),[False,True,False]); torque_payload=json.loads(str(raw["applied_powertrain_diagnostics_json"][1])); self.assertEqual(torque_payload,{"wheel_motor_torque_nm":[11.25,-22.5,33.75,-44.0]}); self.assertEqual(raw["applied_powertrain_diagnostics_json"][0],""); self.assertTrue(bool(raw["source_hashes_match"]))
                fail_path=case_dir/"20-to-10-p-plus1-first-failure.npz"; fail_path.write_bytes(b"keep-existing")
                from tools.run.validate_hybrid_transitions import save_failure_trace
                with self.assertRaises(FileExistsError): save_failure_trace(fail_path,samples=samples,plant=plant,exception=RuntimeError("mock failure"),diagnostics={},source_before={},source_after={},identity={})
                self.assertEqual(fail_path.read_bytes(),b"keep-existing")
                original_checkpoint=checkpoint.read_bytes()
                with self.assertRaises(FileExistsError): run_case("20-to-10-p-plus1",case_dir,forecast_slabs=30)
                self.assertEqual(checkpoint.read_bytes(),original_checkpoint)
            original_sha=runner._sha(runner.CORE_SOURCE_NAMES); changed=dict(original_sha); changed[runner.CORE_SOURCE_NAMES[0]]="changed-during-assembly"
            with tempfile.TemporaryDirectory() as hash_tmp:
                hash_case=Path(hash_tmp)/"hash-case"
                with patch("tools.run.validate_hybrid_transitions._sha",side_effect=[original_sha,changed,changed,changed]), patch("tools.run.validate_hybrid_transitions._assemble",return_value=mock_z), patch("tools.run.validate_hybrid_transitions.solve_forward",side_effect=fake_solve), patch("tools.run.validate_hybrid_transitions._postprocess_forward_result",wraps=_postprocess_forward_result):
                    hash_result=run_case("20-to-10-p-plus1",hash_case,duration=2.,wall_seconds=90.,forecast_slabs=30)
                self.assertEqual(hash_result["status"],"FAIL"); self.assertFalse(hash_result["source_hashes_match"])
            bad_source_samples=[replace(x,requested_command=None) for x in samples]
            with tempfile.TemporaryDirectory() as bad_tmp:
                bad_out=Path(bad_tmp)
                row=_postprocess_forward_result(result,bad_source_samples,plant,plan,case_id="20-to-10-p-plus1",v0=20.,target=10.,pedal=.25,event=0.,duration=2.,forecast_slabs=30,source_before={"core":"before"},source_after={"core":"changed"},identity={"provider_identity":provider.identity,"provider_model_hash":provider.model_hash,"provider_source_closure_hash":provider.source_closure_hash,"plan_identity":plan.identity},output_dir=bad_out,source_hashes_match_required=True)
                self.assertEqual(row["status"],"FAIL")
                self.assertFalse(row["source_hashes_match"])
            missing=[replace(x,requested_command=None) for x in samples]
            with tempfile.TemporaryDirectory() as missing_tmp:
                missing_out=Path(missing_tmp)
                row=_postprocess_forward_result(result,missing,plant,plan,case_id="20-to-10-p-plus1",v0=20.,target=10.,pedal=.25,event=0.,duration=2.,forecast_slabs=30,source_before={"core":"same"},source_after={"core":"same"},identity={"provider_identity":provider.identity,"provider_model_hash":provider.model_hash,"provider_source_closure_hash":provider.source_closure_hash,"plan_identity":plan.identity},output_dir=missing_out)
                self.assertEqual(row["status"],"FAIL")
                with np.load(missing_out/"20-to-10-p-plus1-raw.npz",allow_pickle=False) as missing_archive:
                    self.assertFalse(missing_archive["requested_command_available"].any())

    def test_stop_run_uses_assembled_model_for_oracle_before_postprocess(self):
        from unittest.mock import patch
        from tools.run import validate_hybrid_transitions as runner
        from simplified_vehicle_models.solvers.forward_driver import ForwardResult

        assembled = runner._assemble("stop-neutral", 6., forecast_slabs=30)
        self.assertIs(assembled["plant"].vehicle, assembled["model"])
        self.assertFalse(hasattr(assembled["plant"], "model"))
        result = ForwardResult(np.asarray([0., 1.]), np.column_stack((assembled["initial"], assembled["initial"])), False, .1, 0, "mocked solver result")
        sentinel = RuntimeError("stop-wiring-postprocess-sentinel")
        observed = {}
        original_oracle = runner._prospective_stop_distance

        def observe_oracle(*args, **kwargs):
            observed["oracle_model"] = args[2]
            observed["oracle_provider"] = args[1]
            observed["oracle_call"] = True
            return original_oracle(*args, **kwargs)

        def mock_solve(plant, driver, initial, **kwargs):
            observed["plant"] = plant
            observed["initial"] = initial
            observed["kwargs"] = kwargs
            return result

        with tempfile.TemporaryDirectory() as tmp:
            case_dir = Path(tmp) / "stop-case"
            with patch.object(runner, "solve_forward", side_effect=mock_solve), patch.object(
                    runner, "_prospective_stop_distance", side_effect=observe_oracle), patch.object(
                    runner, "_postprocess_forward_result", side_effect=sentinel):
                with self.assertRaisesRegex(RuntimeError, "stop-wiring-postprocess-sentinel") as caught:
                    run_case("stop-neutral", case_dir, duration=6., wall_seconds=90., forecast_slabs=30)

        # The postprocess sentinel proves the actual prospective oracle completed;
        # no solver implementation or arbitrary plant.model shim is exercised.
        self.assertIs(caught.exception.__cause__, sentinel)
        self.assertTrue(observed["oracle_call"])
        self.assertEqual(observed["oracle_model"].planar.total_mass, assembled["model"].planar.total_mass)
        self.assertEqual(observed["oracle_provider"].identity, assembled["provider"].identity)
        self.assertEqual(observed["plant"].vehicle.planar.total_mass, assembled["model"].planar.total_mass)
        np.testing.assert_array_equal(observed["initial"], assembled["initial"])
        self.assertEqual(observed["kwargs"]["stop_speed_mps"], .01)
        self.assertEqual(observed["kwargs"]["initial_pedal"], 0.)
        self.assertEqual(assembled["policy"].config.slabs, 30)

    def test_matrix_ids_are_unique_and_bounded(self):
        self.assertEqual(len(CASE_IDS), 13)
        self.assertEqual(len(set(CASE_IDS)), len(CASE_IDS))

    def test_experimental_slab_override_is_instance_local_and_validated(self):
        from simplified_vehicle_models.solvers.hybrid_driver import HybridPolicyConfig
        import inspect
        from tools.run import validate_hybrid_transitions as runner
        self.assertEqual(inspect.signature(run_case).parameters["forecast_slabs"].default,60)
        self.assertEqual(HybridPolicyConfig().slabs,60)
        self.assertIn('add_argument("--forecast-slabs",type=int,default=60',inspect.getsource(runner.main))
        for invalid in (0,61,True,1.5):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                run_case("prospective-drive-brake",None,forecast_slabs=invalid)

    def test_tv_allocator_rejection_is_real_provider_gate(self):
        from simplified_vehicle_models.solvers.longitudinal_capability import validate_execution_allocator
        class Provider: identity="actual-capability"
        with self.assertRaisesRegex(ValueError,"no matching execution path"):
            validate_execution_allocator("actual-capability",None,100.,Provider())

    def test_terminal_stop_fixture_is_immutable_positive_endpoint_and_provider_bound(self):
        from tools.run.validate_hybrid_driver import FORMULATION
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.solvers.longitudinal_capability import CurrentVehicleCapability
        model=compile_formulation(str(FORMULATION)).model
        provider=CurrentVehicleCapability(model.planar,wheel_radii_m=(.29,)*4,speed_grid_mps=(.005,.01,5.,10.,20.,22.,25.,40.),curvature_grid_inv_m=(-.001,0.,.001),soc_grid=(.8,.9),source_label="task5-transition-actual-38state-v4",calibration_label="compiled-IEM-current")
        plan,evidence=_build_terminal_stop_fixture(provider,soc=.85)
        self.assertEqual(evidence["status"],"TERMINAL_PRESCRIPTION_NOT_BUILDER_CERTIFIED")
        self.assertEqual((plan.speed_mps[0],plan.speed_mps[-1]),(3.,.01)); self.assertFalse(plan.speed_mps.flags.writeable)
        self.assertEqual(plan.provider_identity,provider.identity); self.assertEqual(plan.source_closure_hash,provider.source_closure_hash)
        mid=plan.sample(float(plan.distance_m[-1]/2))
        self.assertAlmostEqual(mid.speed_mps,2.12132034356,places=4)
        self.assertAlmostEqual(mid.corner_speed_cap_mps,mid.speed_mps,places=5)
        self.assertAlmostEqual(plan.sample(2.).corner_speed_cap_mps,.01,places=12)
        with self.assertRaises(ValueError): plan.speed_mps[0]=2.

    def test_real_prospective_oracle_stops_at_target_records_queries_and_converges(self):
        from tools.run.validate_hybrid_driver import FORMULATION
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.solvers.longitudinal_capability import CurrentVehicleCapability
        from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityPredictor
        from tools.run.validate_hybrid_transitions import _prospective_stop_distance
        model=compile_formulation(str(FORMULATION)).model
        provider=CurrentVehicleCapability(model.planar,wheel_radii_m=(.29,)*4,speed_grid_mps=(.005,.01,5.,10.,20.,22.,25.,40.),curvature_grid_inv_m=(-.001,0.,.001),soc_grid=(.8,.9),source_label="task5-transition-actual-38state-v4",calibration_label="compiled-IEM-current")
        plan,evidence=_build_terminal_stop_fixture(provider,soc=.85)
        predictor=BrakeAuthorityPredictor(model.planar,maximum_abs_curvature_inv_m=.001,minimum_speed_mps=.005,maximum_speed_mps=40.)
        coarse=_prospective_stop_distance(plan,provider,model,predictor,initial_speed_mps=3.,soc=.85,dt=.002)
        dense=_prospective_stop_distance(plan,provider,model,predictor,initial_speed_mps=3.,soc=.85,dt=.001,verify_dense=False)
        self.assertAlmostEqual(coarse["terminal_speed_mps"],.01,places=12)
        self.assertGreater(coarse["terminal_speed_mps"],0.)
        self.assertLessEqual(coarse["terminal_event_bracket"]["event_fraction_of_full_step"],1.)
        self.assertAlmostEqual(coarse["distance_m"],dense["distance_m"],delta=.01)
        self.assertEqual(coarse["explicit_physical_oracle_query_count"],len(coarse["explicit_physical_oracle_queries"]))
        self.assertIn("excludes internal HybridLongitudinalPolicy",coarse["explicit_physical_oracle_query_scope"])
        self.assertNotIn("provider_queries",coarse)
        self.assertEqual(len(coarse["integrated_samples"]),coarse["samples"])
        self.assertTrue(all("policy_decision" in row and "explicit_physical_oracle_query" in row for row in coarse["integrated_samples"]))
        actual_speeds=[q["speed_mps"] for q in coarse["explicit_physical_oracle_queries"]]
        self.assertEqual(coarse["explicit_physical_oracle_query_speed_mps_range"],[min(actual_speeds),max(actual_speeds)])
        self.assertEqual(coarse["explicit_physical_oracle_queries"][-1]["query_role"],"terminal_endpoint_authority_check")
        self.assertTrue(all(q["in_domain"] for q in coarse["explicit_physical_oracle_queries"]))
        self.assertEqual(coarse["dense_convergence"]["reference_dt_s"],.001)
        self.assertAlmostEqual(coarse["distance_m"],2.0046201716663954,delta=.01)
        self.assertEqual(coarse["terminal_event_bracket"]["event_speed_mps"],.01)
        self.assertAlmostEqual(coarse["terminal_event_bracket"]["event_time_s"],coarse["time_s"],places=12)
        self.assertAlmostEqual(coarse["terminal_event_bracket"]["event_station_m"],coarse["distance_m"],places=12)
        self.assertGreater(coarse["explicit_physical_oracle_query_speed_mps_range"][0],.005)

    def test_terminal_fixture_midpoint_speed_and_cap_follow_squared_speed_reference(self):
        from tools.run.validate_hybrid_driver import FORMULATION
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.solvers.longitudinal_capability import CurrentVehicleCapability
        model=compile_formulation(str(FORMULATION)).model
        provider=CurrentVehicleCapability(model.planar,wheel_radii_m=(.29,)*4,speed_grid_mps=(.005,.01,5.,10.,20.,22.,25.,40.),curvature_grid_inv_m=(-.001,0.,.001),soc_grid=(.8,.9),source_label="task5-transition-actual-38state-v4",calibration_label="compiled-IEM-current")
        plan,_=_build_terminal_stop_fixture(provider,soc=.85)
        sample=plan.sample(1.)
        self.assertAlmostEqual(sample.speed_mps,2.1213,places=3)
        self.assertAlmostEqual(sample.corner_speed_cap_mps,sample.speed_mps,places=5)
        self.assertAlmostEqual(plan.speed_mps[-1],.01,places=12)
        self.assertAlmostEqual(plan.corner_speed_cap_mps[-1],.01,places=12)

    def test_terminal_stop_gate_requires_native_event_all_guards_and_distance_oracle(self):
        args=dict(reached_end=True,finite_state=True,contact_min=1.,arrival_index=1,speed_mps=[3.,.01],cap_mps=[3.,.01],requested_available=[True,True],source_hashes_match=True,candidate_counts=[0,1],rollout_samples=[0,90],distance_error_percent=4.)
        self.assertTrue(terminal_stop_gate(**args))
        for changes in ({"arrival_index":None},{"speed_mps":[3.,.52]},{"requested_available":[True,False]},{"source_hashes_match":False},{"candidate_counts":[4,0]},{"rollout_samples":[181,0]},{"distance_error_percent":5.1}):
            with self.subTest(changes=changes): self.assertFalse(terminal_stop_gate(**{**args,**changes}))

    def test_seam_crossing_requires_actual_accepted_pair(self):
        self.assertEqual(seam_crossing_evidence([495.,501.])['path_fraction'],5/6)
        with self.assertRaisesRegex(ValueError,"no seam crossing"):
            seam_crossing_evidence([495.,499.])

    def test_real_accepted_sample_seam_bracket_enforces_no_reset_and_records_commands(self):
        from types import SimpleNamespace
        from simplified_vehicle_models.solvers.forward_driver import PreviewDriver, SpeedPlan
        from simplified_vehicle_models.solvers.hybrid_speed_plan import HybridSpeedPlan
        from tools.run.validate_hybrid_transitions import _build_terminal_stop_fixture
        from tools.run.validate_hybrid_driver import FORMULATION
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.solvers.longitudinal_capability import CurrentVehicleCapability
        model=compile_formulation(str(FORMULATION)).model
        provider=CurrentVehicleCapability(model.planar,wheel_radii_m=(.29,)*4,speed_grid_mps=(.005,.01,5.,10.,20.,22.,25.,40.),curvature_grid_inv_m=(-.001,0.,.001),soc_grid=(.8,.9),source_label="task5-transition-actual-38state-v4",calibration_label="compiled-IEM-current")
        plan=HybridSpeedPlan(np.array([0.,500.,1000.]),np.array([20.,20.,20.]),np.zeros(3),np.full(3,20.),np.full(3,20.),np.array([0,1,2]),np.zeros(3),True,.85,.6,provider.identity,provider.source_closure_hash,"seam-test","TEST",0,(),"test",1,1e-8)
        driver=PreviewDriver(SpeedPlan(np.array([0.,1.]),np.array([20.,20.]),np.zeros(2)),2.7,.5,.5,1000.,throttle_apply_rate=1/.175,throttle_release_rate=1/.175,brake_apply_rate=1/.175,brake_release_rate=1/.175)
        from simplified_vehicle_models.solvers.vertical_forward import STEERING_INDEX
        states=np.zeros((2,38)); states[:,3]=20.; station=np.array([495.,505.]); time=np.array([1.,1.5]); pedal=np.array([.4,.4]); applied=np.column_stack((pedal,np.zeros(2),np.array([.02,.02]))); requested=np.column_stack((pedal,np.zeros(2),np.array([.4,.4]))); states[:,13]=.4; states[:,STEERING_INDEX]=[.05,.10]
        args=dict(station_m=station,time_s=time,state=states,requested_command=requested,applied_command=applied,pedal=pedal,requested_available=[True,True],pedal_available=[True,True],plan=plan,driver=driver)
        evidence=seam_continuity_evidence(**args)
        self.assertEqual(evidence["station_before_m"],495.); self.assertEqual(evidence["station_after_m"],505.)
        self.assertEqual(evidence["requested_command_before"],requested[0].tolist())
        self.assertEqual(evidence["signed_pedal_before"],.4)
        self.assertAlmostEqual(evidence["station_progress_m"],10.)
        self.assertAlmostEqual(evidence["maximum_progress_bound_m"],10.05)
        for changes in ({"requested_available":[False,True]},{"pedal_available":[True,False]},{"pedal":[.4,-.4]},{"station_m":[495.,550.]},{"state":np.vstack([states[0],states[1]+100.])},{"state":np.vstack([states[0],states[1]+np.eye(1,38,STEERING_INDEX)[0]*.5])}):
            with self.subTest(changes=list(changes)),self.assertRaises(ValueError): seam_continuity_evidence(**{**args,**changes})
        for invalid_rate in (None, float("nan"), float("inf"), -0.1):
            broken=SimpleNamespace(**{**driver.__dict__,"maximum_steering_rate":invalid_rate})
            with self.subTest(invalid_rate=invalid_rate),self.assertRaisesRegex(ValueError,"maximum_steering_rate"):
                seam_continuity_evidence(**{**args,"driver":broken})
        missing=SimpleNamespace(**{key:value for key,value in driver.__dict__.items() if key!="maximum_steering_rate"})
        with self.assertRaisesRegex(ValueError,"maximum_steering_rate unavailable"):
            seam_continuity_evidence(**{**args,"driver":missing})
        zero_rate=SimpleNamespace(**{**driver.__dict__,"maximum_steering_rate":0.})
        same_angle=states.copy(); same_angle[:,STEERING_INDEX]=.05
        self.assertTrue(seam_continuity_evidence(**{**args,"driver":zero_rate,"state":same_angle}))
        with self.assertRaisesRegex(ValueError,"no seam crossing"):
            seam_continuity_evidence(**{**args,"station_m":np.array([490.,495.])})

    def test_stop_plus_point3_shares_compiled_model_assembly_without_solver(self):
        from tools.run import validate_hybrid_transitions as runner
        assembled = runner._assemble("stop-plus-point3", 6., forecast_slabs=30)
        self.assertIs(assembled["plant"].vehicle, assembled["model"])
        self.assertFalse(hasattr(assembled["plant"], "model"))
        self.assertEqual(assembled["pedal"], .3)
        self.assertEqual(assembled["policy"].config.slabs, 30)

    def test_transition_matrix_carries_requested_initial_pedals(self):
        self.assertEqual([_make_case(name, 8.)[2] for name in CASE_IDS[:3]], [1., 0., -.5])
        self.assertEqual([_make_case(name, 8.)[2] for name in CASE_IDS[3:6]], [1., 0., -.5])

    def test_signed_curvature_cases_are_real_not_zeroed(self):
        self.assertEqual(_make_case("left-curvature", 8.)[3], 1/30)
        self.assertEqual(_make_case("right-curvature", 8.)[3], -1/30)

    def test_real_corner_assembly_policy_query_validity_at_initial_state_and_perturbation(self):
        from tools.run import validate_hybrid_transitions as runner
        from simplified_vehicle_models.solvers.longitudinal_capability import CapabilityQuery
        from simplified_vehicle_models.solvers.vertical_forward import SOC_INDEX
        for case_id,sign in (("left-curvature",1),("right-curvature",-1)):
            with self.subTest(case_id=case_id):
                z=runner._assemble(case_id,8.,forecast_slabs=30); plant=z["plant"]; policy=z["policy"]; x=z["initial"]
                self.assertEqual(np.sign(z["curvature"]),sign)
                self.assertTrue(np.all(z["steering"].speed[:45]==10.))
                def query_decide(state):
                    context=plant.hybrid_kinematic_context(state)
                    progress=(context.vx_mps*np.cos(context.heading_error_rad)-context.vy_mps*np.sin(context.heading_error_rad))/(1-context.curvature_inv_m*context.lateral_error_m)
                    soc=float(state[SOC_INDEX]); capability=z["provider"].evaluate(CapabilityQuery(float(progress),float(context.curvature_inv_m),soc,0.))
                    self.assertTrue(context.vy_dot_valid)
                    self.assertTrue(capability.in_domain,capability.limiting_constraints)
                    decision=policy.decide(observation=plant.observe(state),pedal_position=0.,kinematics=context,soc=soc,brake_context=plant.brake_authority_context(state,predictor=policy.predictor))
                    self.assertNotEqual(decision.decision.status,"capability_domain_exit")
                    return float(progress),decision.decision.status
                p0,s0=query_decide(x)
                perturbed=x.copy(); perturbed[3]+=1e-10
                p1,s1=query_decide(perturbed)
                self.assertGreater(p1,p0)
                self.assertIn(s0,("event_unknown:event_beyond_horizon","nominal"))
                self.assertIn(s1,("event_unknown:event_beyond_horizon","nominal"))


    def test_stop_cases_retain_operational_target(self):
        neutral = _make_case("stop-neutral", 8.)
        throttle = _make_case("stop-plus-point3", 8.)
        self.assertEqual(neutral[1:3], (.01, 0.))
        self.assertEqual(throttle[1:3], (.01, .3))

    def test_transition_gate_rejects_one_bad_sample_and_missing_arrival(self):
        shared = dict(reached_end=True, finite_state=True, contact_min=10.,
                      event_arrived=True, candidate_counts=[0, 1], rollout_samples=[0, 60])
        self.assertTrue(transition_gate(post_event_overshoots=[0., .2], **shared))
        self.assertFalse(transition_gate(post_event_overshoots=[0., .51], **shared))
        self.assertFalse(transition_gate(post_event_overshoots=[0.], **{**shared, "event_arrived": False}))
        self.assertFalse(transition_gate(post_event_overshoots=[0.], **{**shared, "finite_state": False}))

    def test_actual_rollout_and_source_hash_are_required_when_requested(self):
        args=dict(reached_end=True,finite_state=True,contact_min=10.,event_arrived=True,post_event_overshoots=[0.],candidate_counts=[1],rollout_samples=[60],require_rollout=True)
        self.assertTrue(transition_gate(**args))
        self.assertFalse(transition_gate(**{**args,"candidate_counts":[0]}))
        self.assertFalse(transition_gate(**{**args,"rollout_samples":[0]}))
        self.assertFalse(transition_gate(**{**args,"source_hashes_match":False}))

    def test_event_station_uses_physical_provider_once_margin_and_lead(self):
        class Provider:
            def evaluate(self, query):
                class Result:
                    in_domain = True
                    limiting_constraints = ()
                    drive_net_progress_acceleration_mps2 = 2.
                    brake_net_progress_acceleration_mps2 = -4.
                return Result()
        result = prospective_cap_event_station(Provider(), initial_speed_mps=20.,
            peak_speed_mps=22., target_speed_mps=10., curvature_inv_m=0., soc=.85,
            brake_margin=.60, throttle_release_s=.35, actuator_lag_s=.10,
            fixed_buffer_m=20.)
        drive = (22.**2-20.**2)/(2*2.)
        brake = (22.**2-10.**2)/(2*4.*.60)
        self.assertAlmostEqual(result["event_station_m"], drive+brake+22*.45+20.)
        self.assertEqual(result["brake_margin"], .60)

    def test_config_rejects_nonfinite_and_out_of_budget_values(self):
        self.assertEqual(validate_runner_config(duration=12, wall_seconds=120), (12., 120.))
        self.assertEqual(validate_runner_config(duration=12, wall_seconds=240), (12., 240.))
        self.assertEqual(MAX_PERMITTED_WALL_SECONDS, 240.)
        self.assertEqual(VALIDATION_RESOURCE_CONTRACT_VERSION, "task5-validator-resource-v2-240s")
        self.assertEqual(validate_runner_config(duration=8, wall_seconds=90), (8., 90.))
        for duration, wall in ((float("nan"), 1), (1, float("inf")), (0, 1), (13, 1), (1, 240.0001), (1, -1), (1, float("nan"))):
            with self.subTest(duration=duration, wall=wall), self.assertRaises(ValueError):
                validate_runner_config(duration=duration, wall_seconds=wall)

    def test_resource_contract_identity_in_strict_success_config_and_failure_checkpoint(self):
        import numpy as np
        from types import SimpleNamespace
        from tools.run.validate_hybrid_transitions import save_failure_trace, _atomic_json
        from tools.run import validate_hybrid_transitions as runner
        config=runner._strict_json_value if hasattr(runner,"_strict_json_value") else {"actual_requested_wall_seconds":240.,"validation_resource_contract_version":VALIDATION_RESOURCE_CONTRACT_VERSION}
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"config.json"
            _atomic_json(path,config)
            parsed=json.loads(path.read_text(),parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
            self.assertEqual(parsed["actual_requested_wall_seconds"],240.)
            self.assertEqual(parsed["validation_resource_contract_version"],VALIDATION_RESOURCE_CONTRACT_VERSION)
            class Plant:
                state_size=2
                def observe(self,row): return SimpleNamespace(speed=float(row[1]))
            d=SimpleNamespace(phase="drive",status="nominal",candidate_count=0,rollout_samples=0,event_crossed=None,event_crossing_speed_mps=None,event_speed_cap_mps=None)
            sample=SimpleNamespace(time=.1,state=[1.,20.],rates=[2.,.5],pedal=1.,requested_command=SimpleNamespace(throttle=1.,brake_force=0.),command=SimpleNamespace(throttle=.5,brake_force=0.),policy_diagnostics=d)
            failure=Path(tmp)/"failure.npz"
            save_failure_trace(failure,samples=[sample],plant=Plant(),exception=RuntimeError("mock"),diagnostics={},source_before={},source_after={},identity={"actual_requested_wall_seconds":240.})
            with np.load(failure,allow_pickle=False) as archive:
                payload=json.loads(str(archive["identity"]))
            self.assertEqual(payload["actual_requested_wall_seconds"],240.)
            self.assertEqual(payload["validation_resource_contract_version"],VALIDATION_RESOURCE_CONTRACT_VERSION)

    def test_trace_metrics_requires_real_event_arrival_finite_data_and_solver_end(self):
        kwargs=dict(time_s=[0,1],state=[[0,1],[1,2]],station_m=[0,2],speed_mps=[3,1],cap_mps=[3,1],contact_min_n=1,event_station_m=1,target_speed_mps=1,candidate_count=[1,2],rollout_samples=[60,60],solver_status="finished")
        result=trace_metrics(**kwargs)
        self.assertEqual(result["arrival_index"],1)
        self.assertEqual(result["candidate_count_max"],2)
        self.assertTrue(result["passed"])
        self.assertFalse(trace_metrics(**{**kwargs,"candidate_count":[0,0],"rollout_samples":[0,0],"require_rollout":True})["passed"])
        self.assertFalse(trace_metrics(**{**kwargs,"source_hashes_match":False})["passed"])
        self.assertFalse(trace_metrics(**{**kwargs,"candidate_count":[0,0],"rollout_samples":[0,0],"require_rollout":True,"solver_status":"finished"})["passed"])
        with self.assertRaises(ValueError): trace_metrics(**{**kwargs,"state":[[0,float("nan")],[1,2]]})
        with self.assertRaises(ValueError): trace_metrics(**{**kwargs,"state":[[0,1]],"station_m":[0]})
        for changes in ({"speed_mps":[3,float("nan")]},{"station_m":[0,0]},{"solver_status":"failed"},{"contact_min_n":-1}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                trace_metrics(**{**kwargs,**changes})

    def test_partial_failure_trace_preserves_accepted_rows_and_failure_identity(self):
        import tempfile
        from pathlib import Path
        from types import SimpleNamespace
        from tools.run.validate_hybrid_transitions import save_failure_trace
        class Plant:
            state_size=2
            def observe(self,row): return SimpleNamespace(speed=float(row[1]))
        d=SimpleNamespace(phase="drive",status="nominal",candidate_count=2,rollout_samples=60,event_crossed=None,event_crossing_speed_mps=None,event_speed_cap_mps=10.)
        sample=SimpleNamespace(time=.1,state=[1.,20.],rates=[2.,.5],pedal=1.,requested_command=SimpleNamespace(throttle=1.,brake_force=0.),command=SimpleNamespace(throttle=.5,brake_force=0.),policy_diagnostics=d)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"partial.npz"
            save_failure_trace(path,samples=[sample],plant=Plant(),exception=TimeoutError("synthetic timeout"),diagnostics={"failure_kind":"timeout"},source_before={"core":"a"},source_after={"core":"a"},identity={"policy":"test"})
            with __import__("numpy").load(path,allow_pickle=False) as z:
                self.assertEqual(z["accepted_samples"],1)
                self.assertEqual(str(z["solver_status"]),"FAILED_OBSERVER_ROWS_ONLY")
                self.assertEqual(z["candidate_count"].tolist(),[2])
                self.assertTrue(bool(z["source_hashes_match"]))
                self.assertEqual(z["failure_message"],"synthetic timeout")
                self.assertEqual(z["trajectory_availability"],"observer_rows_delivered_before_failure")
                path_empty=Path(tmp)/"empty.npz"
                save_failure_trace(path_empty,samples=[],plant=Plant(),exception=TimeoutError("RHS timeout before observer replay"),diagnostics={"last_time":.2},source_before={"core":"a"},source_after={"core":"a"},identity={"policy":"test"})
                with __import__("numpy").load(path_empty,allow_pickle=False) as z:
                    self.assertEqual(z["accepted_samples"],0)
                    self.assertEqual(z["solver_status"],"FAILED_NO_OBSERVER_TRACE")
                    self.assertEqual(z["trajectory_availability"],"unavailable_before_observer_replay")

    def test_actual_forward_integration_error_keeps_original_diagnostics_and_empty_observer_state(self):
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        from types import SimpleNamespace
        from tools.run.validate_hybrid_transitions import run_case
        from simplified_vehicle_models.solvers.forward_driver import ForwardIntegrationError
        import numpy as np
        diag={"failure_kind":"validation_timeout","last_accepted_time":12.}
        error=ForwardIntegrationError("accepted-state validation exceeded wall budget",diag,np.asarray([10.,20.]),12.)
        z={"v0":20.,"target":10.,"pedal":1.,"event":99.,"plan":SimpleNamespace(identity="plan-id"),"provider":SimpleNamespace(identity="prov",model_hash="model",source_closure_hash="closure"),"preflight":None,"event_meta":None,"plant":SimpleNamespace(state_size=2,observe=lambda state:SimpleNamespace(speed=float(state[1]))),"driver":None,"initial":np.asarray([0.,20.])}
        with tempfile.TemporaryDirectory() as tmp:
            output=Path(tmp)/"case"
            with patch("tools.run.validate_hybrid_transitions._assemble",return_value=z), patch("tools.run.validate_hybrid_transitions.solve_forward",side_effect=error):
                with self.assertRaisesRegex(RuntimeError,"accepted-state validation exceeded wall budget.*last_s=None.*last_v=None") as caught:
                    run_case("20-to-10-p-plus1",output,duration=12.,wall_seconds=120.)
            self.assertIs(caught.exception.__cause__,error)
            with np.load(output/"20-to-10-p-plus1-first-failure.npz",allow_pickle=False) as archive:
                self.assertEqual(int(archive["accepted_samples"]),0)
                self.assertIn("validation_timeout",str(archive["failure_diagnostics"]))

    def test_cli_preserves_original_exception_before_or_after_case_creation(self):
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        from tools.run.validate_hybrid_transitions import main
        for creates_dir in (False,True):
            with self.subTest(creates_dir=creates_dir), tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp)/"out"
                def fail(case,directory,**kwargs):
                    if creates_dir: directory.mkdir(parents=True)
                    raise RuntimeError("original-case-error")
                with patch("tools.run.validate_hybrid_transitions.run_case",side_effect=fail):
                    self.assertEqual(main(["--case","20-to-10-p-plus1","--output",str(root)]),1)
                self.assertTrue((root/"task-5-transitions-report.json").exists())
                artifacts=list((root/"20-to-10-p-plus1").glob("*-cli-failure.json"))
                self.assertEqual(len(artifacts),1)
                self.assertIn("original-case-error",artifacts[0].read_text())

    def test_cli_does_not_clobber_existing_failure_artifact(self):
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        from tools.run.validate_hybrid_transitions import main
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/"out"
            def fail(case,directory,**kwargs):
                directory.mkdir(parents=True)
                (directory/(case+"-cli-failure.json")).write_text("preserve me")
                raise RuntimeError("original-case-error")
            with patch("tools.run.validate_hybrid_transitions.run_case",side_effect=fail):
                self.assertEqual(main(["--case","20-to-10-p-plus1","--output",str(root)]),1)
            self.assertEqual((root/"20-to-10-p-plus1"/"20-to-10-p-plus1-cli-failure.json").read_text(),"preserve me")
            self.assertTrue((root/"task-5-transitions-report.json").exists())

    def test_cli_bounds_and_case_selection_exist(self):
        import inspect
        from tools.run import validate_hybrid_transitions as runner
        source = inspect.getsource(runner.main)
        self.assertIn('wall-seconds",type=float,default=90.', source)
        self.assertEqual(inspect.signature(run_case).parameters["wall_seconds"].default,90.)
        self.assertIn("MAX_PERMITTED_WALL_SECONDS=240", inspect.getsource(runner))
        self.assertIn("wall-seconds", source)
        self.assertIn("duration", source)
        self.assertIn("if row[\"status\"] not in (\"PASS\",\"EXPECTED_UNSUPPORTED\"): break", source)


if __name__ == "__main__":
    unittest.main()
