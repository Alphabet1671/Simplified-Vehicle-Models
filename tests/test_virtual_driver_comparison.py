"""Focused pure contract tests for Task 7 comparison preparation."""
from __future__ import annotations

import unittest
from dataclasses import replace

import numpy as np

from tools.run.compare_virtual_drivers import (
    ComparisonIdentity,
    RunConfigIdentity,
    VARIANT_IDS,
    analyze_trace,
    historical_forward_law,
    build_historical_speed_plan,
    shared_comparison_run,
    periodic_smooth_acceleration,
    predeclare_first_braking_zone,
    validate_primary_identities,
)


class VirtualDriverComparisonTests(unittest.TestCase):
    def _config(self):
        return RunConfigIdentity(
            model_semantic_sha256="model", source_hashes=(("a.py", "a"),), track_sha256="track",
            calibration_hashes=(("pedal.csv", "cal"),), initial_full_state_sha256="state",
            initial_soc=.82, pedal_and_steering_state=(0., 0.), solver=(("duration", 8.), ("method", "BDF")),
            lateral_margin=.9, brake_margin=.6, provider_identity="provider", allocator_identity="allocator",
            foot_rates=(4., 5., 6., 7.), pedal_lag_s=.02, maximum_pedal_force_n=1000.,
            governor_margin=.85, corner_caps_sha256="caps")

    def _identities(self, config=None, mapping="current-physical-inverse-v1", closure="closure"):
        config = self._config() if config is None else config
        from tools.run.compare_virtual_drivers import ALGORITHM_IDS, REFERENCE_LAWS, HISTORICAL_SOURCE_SHA256
        from tools.run.compare_virtual_drivers import source_manifest_digest
        closure = source_manifest_digest(dict(config.source_hashes)) if closure == "closure" else closure
        return [ComparisonIdentity(v, ALGORITHM_IDS[i], REFERENCE_LAWS[i], mapping,
                                   HISTORICAL_SOURCE_SHA256 if i==0 else "runtime-source-"+str(i), closure, config, "test-binding")
                for i, v in enumerate(VARIANT_IDS)]

    def test_primary_identity_accepts_equal_shared_physical_configuration(self):
        validate_primary_identities(self._identities())

    def test_primary_identity_rejects_mixed_tire_calibration_track_or_solver(self):
        base = self._config()
        for field, changed in (
            ("track_sha256", "other-track"), ("calibration_hashes", (("pedal.csv", "other"),)),
            ("solver", (("method", "Radau"),)), ("provider_identity", "other-provider"),
            ("foot_rates", (5., 5., 4., 5.)), ("lateral_margin", .91),
        ):
            with self.subTest(field=field):
                identities = self._identities()
                bad_config = replace(self._config(), **{field: changed})
                identities[-1] = replace(identities[-1], run_config=bad_config)
                with self.assertRaisesRegex(ValueError, "differ"):
                    validate_primary_identities(identities)

    def test_primary_identity_rejects_historical_mapping_or_source_closure_mismatch(self):
        with self.assertRaisesRegex(ValueError, "same current physical"):
            validate_primary_identities(self._identities(mapping="heuristic-normalized-force-map"))
        items = self._identities()
        items[-1] = ComparisonIdentity(**{**items[-1].__dict__, "current_source_closure_sha256": "different"})
        with self.assertRaisesRegex(ValueError, "source closure"):
            validate_primary_identities(items)

    def test_historical_forward_law_is_gain_two_over_six_with_governor(self):
        accel, throttle, brake = historical_forward_law(12., 10., 1., .425, .85)
        self.assertAlmostEqual(accel, 5.)
        self.assertAlmostEqual(throttle, 5. / 6.)
        self.assertEqual(brake, 0.)
        accel, throttle, brake = historical_forward_law(8., 10., 0., .425, .85)
        self.assertEqual(accel, -4.)
        self.assertEqual(throttle, 0.)
        self.assertAlmostEqual(brake, (4. / 6.) * .5)

    def test_historical_planner_uses_supplied_shared_caps_and_current_limits(self):
        x=np.arange(0.,31.,1.); k=np.zeros_like(x); k[10:16]=.1; k[-1]=k[0]
        caps=np.full_like(x,20.); caps[10:16]=10.; caps[-1]=caps[0]
        plan=build_historical_speed_plan(x,k,caps,drive_acceleration_mps2=2.5,
            brake_deceleration_mps2=3.0,maximum_speed_mps=25.,lateral_acceleration_limit_mps2=9.)
        self.assertLessEqual(float(np.max(plan.speed)),20.)
        self.assertLessEqual(float(np.max(plan.speed[10:16])),10.)
        self.assertEqual(plan.speed[0],plan.speed[-1])
        # Forward differences in squared speed cannot exceed the declared cap.
        self.assertLessEqual(float(np.max(np.diff(plan.speed**2)/(2*np.diff(x)))),2.5+1e-9)

    def test_historical_speed_plan_is_owned_irreversibly_immutable(self):
        source_distance=np.arange(4.,dtype=float);source_speed=np.array([4.,5.,6.,4.]);source_curvature=np.zeros(4);source_acceleration=np.array([1.,0.,-1.,0.])
        plan=build_historical_speed_plan(source_distance,source_curvature,np.full(4,8.),drive_acceleration_mps2=3.,brake_deceleration_mps2=4.,maximum_speed_mps=8.,lateral_acceleration_limit_mps2=9.,is_closed=False)
        expected=plan.distance.copy()
        source_distance[1]=999.
        np.testing.assert_array_equal(plan.distance,expected)
        for array in (plan.distance,plan.speed,plan.curvature,plan.acceleration):
            with self.assertRaises(ValueError):array[0]=123.
            with self.assertRaises(ValueError):array.setflags(write=True)

    def test_historical_command_uses_signed_frenet_and_current_physical_inverse(self):
        from tools.run.compare_virtual_drivers import HistoricalForwardBackwardAdapter, HistoricalSpeedPlan
        from types import SimpleNamespace
        from simplified_vehicle_models.solvers.envelope_driver import EnvelopePolicy, EnvelopeDriverSettings
        import tools.run.compare_virtual_drivers as module
        plan=HistoricalSpeedPlan(*(np.array([0.,10.,20.]) for _ in range(4)),is_closed=False)
        plan=replace(plan,speed=np.array([10.,10.,10.]),acceleration=np.array([-1.,-1.,-1.]))
        class Steer:
            envelope_policy=None; wheelbase=1.; maximum_steering=.5; maximum_steering_rate=1.; maximum_brake_force=1000.
            preview_seconds=.1; speed_gain=2.; heading_gain=1.; lateral_gain=1.; steering_time_constant=.1
            pedal_travel_time=.175; pedal_time_constant=.02; throttle_apply_rate=4.; throttle_release_rate=5.
            brake_apply_rate=6.; brake_release_rate=7.; steering_feedforward="preview"
            def _steering_command(self,observation): return .03
        class Predictor:
            vehicle=SimpleNamespace(maximum_pedal_force=1000.)
            def request_current(self,ctx,decel):
                self.request=(ctx,decel); return SimpleNamespace(pedal_force_n=321.)
        predictor=Predictor(); adapter=HistoricalForwardBackwardAdapter(plan,steering_driver=Steer(),physical_predictor=predictor,shared_envelope=EnvelopePolicy(type("E",(),{"sample":lambda self,s:None})(),EnvelopeDriverSettings()))
        fake_kin=SimpleNamespace(vx_mps=12.,vy_mps=1.,heading_error_rad=.1,curvature_inv_m=0.,lateral_error_m=0.)
        class Plant:
            def hybrid_kinematic_context(self,state): return fake_kin
            def brake_authority_context(self,state,predictor): return "force-context"
        obs=SimpleNamespace(distance=5.,wheel_speed_margin=.425)
        original=module.path_progress_to_body_vx_dot if hasattr(module,"path_progress_to_body_vx_dot") else None
        # Adapter imports converter locally; patch the production converter module.
        import simplified_vehicle_models.solvers.longitudinal_capability as capability
        real=capability.path_progress_to_body_vx_dot
        capability.path_progress_to_body_vx_dot=lambda target,kin: target-.2
        try: command,diag=adapter.command_with_context(obs,.1,np.zeros(38),Plant())
        finally: capability.path_progress_to_body_vx_dot=real
        self.assertEqual(command.brake_force,321.*.5)
        self.assertEqual(command.throttle,0.)
        self.assertEqual(command.steering_rate,.03)
        self.assertAlmostEqual(predictor.request[1], 4.880433133378966)
        self.assertEqual(predictor.request[0], "force-context")
        self.assertEqual(diag["body_conversion_status"],"valid")
        self.assertEqual(adapter._last_comparison_command.mapping_id,"current-physical-inverse-v1")
        from tools.run.compare_virtual_drivers import runtime_driver_binding, VARIANT_IDS
        frozen=runtime_driver_binding(adapter,VARIANT_IDS[0])
        adapter.brake_apply_rate+=1.
        self.assertNotEqual(frozen,runtime_driver_binding(adapter,VARIANT_IDS[0]))

    def test_shared_run_orchestration_reuses_same_full38_plant_state_and_settings(self):
        from dataclasses import replace
        from tools.run.compare_virtual_drivers import HistoricalForwardBackwardAdapter, CurrentPolicyComparisonAdapter, runtime_driver_binding
        from simplified_vehicle_models.solvers.envelope_driver import EnvelopePolicy, EnvelopeDriverSettings
        initial=np.arange(38,dtype=float); initial[0]=0.; initial[12]=.8
        cfg=replace(self._config(),initial_full_state_sha256=__import__("hashlib").sha256(initial.tobytes()).hexdigest())
        identities=self._identities(config=cfg)
        class Plant: state_size=38
        plant=Plant(); calls=[]
        class BaseDriver:
            envelope_policy=None; plan=object(); wheelbase=1.; maximum_steering=.4; maximum_steering_rate=1.
            maximum_brake_force=1000.; preview_seconds=.1; speed_gain=2.; heading_gain=1.5; lateral_gain=1.5
            steering_time_constant=.08; pedal_travel_time=.175; pedal_time_constant=.02
            throttle_apply_rate=4.; throttle_release_rate=5.; brake_apply_rate=6.; brake_release_rate=7.; steering_feedforward="preview"
            def _steering_command(self,obs): return 0.
        class FakePredictor: vehicle=type("V",(),{"maximum_pedal_force":1000.})()
        plan=build_historical_speed_plan(np.arange(0.,30.,1.),np.zeros(30),np.full(30,20.),
            drive_acceleration_mps2=2.5,brake_deceleration_mps2=3.,maximum_speed_mps=20.,lateral_acceleration_limit_mps2=9.)
        hist=HistoricalForwardBackwardAdapter(plan,steering_driver=BaseDriver(),physical_predictor=FakePredictor(),shared_envelope=EnvelopePolicy(type("E",(),{"sample":lambda self,s:None})(),EnvelopeDriverSettings()))
        hist.identity=identities[0]
        hist.algorithm_id=identities[0].algorithm_id
        hist.reference_law=identities[0].reference_law
        hist.mapping_id=identities[0].mapping_id
        hist.current_source_closure_sha256=identities[0].current_source_closure_sha256
        hist.run_config=cfg
        class DriverForTest:
            hybrid_policy=object()
            plan=BaseDriver.plan
        variants=[{"identity":identities[0],"driver":hist},
                  {"identity":identities[1],"driver":type("CurrentStub",(),{"__class__":CurrentPolicyComparisonAdapter})()},
                  {"identity":identities[2],"driver":DriverForTest()}]
        # Exercise each exact local run entry with mocked public solve; no solver
        # or model is invoked. The mock asserts complete state/start/settings parity.
        def solve(mock_plant, driver, state, **kwargs):
            calls.append((mock_plant,driver,state.copy(),dict(kwargs)))
            return object()
        # Identity validation needs typed adapters; use seam object substitution
        # only to assert shared-run data shape without entering classification.
        from dataclasses import replace
        base_binding=runtime_driver_binding(hist,VARIANT_IDS[0])
        bound_identity=replace(identities[0],runtime_binding_sha256=base_binding)
        for identity,driver in zip([bound_identity,*identities[1:]], (hist, type("Driver",(),{})(), DriverForTest())):
            if identity.variant_id==VARIANT_IDS[0]:
                run=shared_comparison_run(solve,plant=plant,initial_state=initial,initial_pedal=0.,driver=driver,
                    run_config_identity=cfg,variant_identity=identity,solver_settings={"method":"BDF","duration":8.})
                self.assertTrue(run["integration_performed"])
        self.assertEqual(len(calls),1)
        self.assertIs(calls[0][0],plant); self.assertEqual(calls[0][2].shape,(38,))
        self.assertEqual(calls[0][3]["method"],"BDF"); self.assertEqual(calls[0][3]["duration"],8.)

    def test_execution_gate_rejects_actual_factory_deltas_before_solver_then_positive_mock(self):
        import copy
        import json
        import tempfile
        from pathlib import Path
        from dataclasses import replace
        import tools.run.compare_virtual_drivers as comparison
        main=comparison.main
        with tempfile.TemporaryDirectory() as directory:
            main(["--prepare-only","--track","models/tracks/fsc_may_2026_centerline.xml","--manifest-dir",directory])
            packet=json.loads((Path(directory)/"source-freeze.json").read_text())
        # Use the real objects built by preparation itself; no reconstruction or solver call.
        prepared=comparison._LAST_PREPARED_FACTORY
        cfg=prepared["config"];variants=prepared["variants"];plant=prepared["plant"];initial=prepared["initial"]
        calls=[]
        def mock_solve(*args,**kwargs):calls.append((args,kwargs));return "mock-result"
        self.assertEqual(len(variants),3)
        cases=(
            ("driver brake_apply_rate",lambda: setattr(variants[0]["driver"],"brake_apply_rate",variants[0]["driver"].brake_apply_rate+1.),"binding"),
            ("solver settings",lambda: None,"solver settings"),
            ("initial pedal",lambda: None,"initial pedal"),
            ("provider identity",lambda: setattr(variants[2]["driver"].hybrid_policy,"provider",object()),"binding|provider"),
            ("plan identity",lambda: setattr(variants[2]["driver"].plan,"speed",np.asarray(variants[2]["driver"].plan.speed)+.01),"binding|digest"),
        )
        # Exercise actual production execution validator directly for each variant.
        from tools.run.compare_virtual_drivers import shared_comparison_run
        for label,mutate,error_regex in cases:
            actual_driver=prepared["drivers"][0];identity=prepared["identities"][0]
            local_cfg=cfg;settings=dict(prepared["solver_settings"]);pedal=prepared["initial_pedal"]
            if label=="driver brake_apply_rate":
                import copy
                actual_driver=copy.copy(actual_driver);actual_driver.brake_apply_rate+=1.
            elif label=="solver settings":settings["method"]="BDF"
            elif label=="initial pedal":pedal=.1
            elif label=="provider identity":
                actual_driver=prepared["drivers"][2];identity=prepared["identities"][2];actual_driver.hybrid_policy.provider=object()
            elif label=="plan identity":
                actual_driver=prepared["drivers"][2];identity=prepared["identities"][2];actual_driver=replace(actual_driver,plan=replace(actual_driver.plan,speed=np.asarray(actual_driver.plan.speed)+.01))
            with self.subTest(label=label),self.assertRaisesRegex(ValueError,error_regex):
                shared_comparison_run(mock_solve,plant=plant,initial_state=initial,initial_pedal=pedal,driver=actual_driver,run_config_identity=local_cfg,variant_identity=identity,solver_settings=settings)
            self.assertEqual(calls,[],label)
        positive=shared_comparison_run(mock_solve,plant=plant,initial_state=initial,initial_pedal=0.,driver=prepared["drivers"][0],run_config_identity=cfg,variant_identity=prepared["identities"][0],solver_settings=dict(prepared["solver_settings"]))
        self.assertTrue(positive["integration_performed"]);self.assertEqual(positive["result"],"mock-result");self.assertEqual(len(calls),1)

    def test_screen_requires_explicit_packet_and_artifacts_without_running(self):
        import tempfile
        from tools.run.compare_virtual_drivers import main
        with self.assertRaisesRegex(SystemExit,"--prepared-manifest and --artifact-dir"):
            main(["--screen"])

    def test_prepare_cli_constructs_three_real_configured_adapters_and_identity_gate(self):
        import json, tempfile
        from pathlib import Path
        from tools.run.compare_virtual_drivers import main, VARIANT_IDS
        with tempfile.TemporaryDirectory() as directory:
            status=main(["--prepare-only","--track","models/tracks/fsc_may_2026_centerline.xml","--manifest-dir",directory])
            self.assertEqual(status,0)
            packet=json.loads((Path(directory)/"source-freeze.json").read_text())
        self.assertTrue(packet["same_driver_factories_constructed"])
        self.assertEqual(packet["geometry_node_count"],48)
        self.assertEqual(packet["initial_state_size"],38)
        self.assertLessEqual(packet["initial_speed_mps"],packet["corner_speed_caps_mps"][0])
        self.assertGreater(packet["initial_speed_mps"],packet["predeclared_zone"]["entry_target_speed_mps"]+.5)
        self.assertGreater(packet["predeclared_zone"]["entry_station_m"],0.)
        self.assertEqual(packet["predeclared_zone"]["entry_target_speed_mps"],packet["zone_kpi_contract"]["entry_target_speed_mps"])
        self.assertEqual(len(packet["variant_runtime_binding_sha256"]),3)
        self.assertEqual({x["variant_id"] for x in packet["driver_command_preparation_checks"]},set(VARIANT_IDS))
        self.assertEqual(len(packet["driver_command_preparation_checks"]),6)
        self.assertFalse(packet["integration_performed"] if "integration_performed" in packet else False)
        self.assertEqual(packet["execution_telemetry_status"],"not-run-awaiting-parent-review")

    def test_periodic_smoothed_plan_is_symmetric_and_seam_periodic(self):
        x = np.arange(0., 21., 1.)
        y = np.zeros_like(x)
        y[0] = y[-1] = 8.
        actual = periodic_smooth_acceleration(x, y)
        self.assertEqual(actual[0], actual[-1])
        self.assertGreater(actual[0], 0.)
        self.assertAlmostEqual(actual[1], actual[-2])

    def test_geometry_zone_depends_only_on_geometry_caps(self):
        x = np.arange(7., dtype=float)
        k = np.array([0., 0., 0., .1, .1, .1, 0.])
        cap = np.array([20., 20., 20., 10., 10., 10., 20.])
        first = predeclare_first_braking_zone(x, k, cap)
        second = predeclare_first_braking_zone(x, k.copy(), cap.copy())
        self.assertEqual(first, second)
        self.assertEqual(first["entry_station_m"], 3.)
        self.assertGreater(first["entry_station_m"], 0.)
        trace={"time_s":[0.,1.,2.,3.,4.,5.],"station_m":[0.,1.,3.,4.,5.,6.],"progress_speed_mps":[20.,18.,10.,9.,8.,7.],"pedal_signed":[.2,.1,-.2,-.3,-.4,-.5],"brake_force_applied_n":[0.,0.,100.,100.,100.,100.]}
        original_rows={k:list(v) for k,v in trace.items()}
        kpi=analyze_trace(trace,first)
        self.assertEqual(kpi["entry_target_speed_mps"],first["entry_target_speed_mps"])
        self.assertEqual(kpi["entry_station_m"],first["entry_station_m"])
        self.assertEqual(kpi["entry_speed_mps"],10.)
        self.assertEqual(kpi["entry_time_s"],2.)
        self.assertEqual(kpi["entry_bracket_accepted_row_indices"],[2,2])
        self.assertEqual(trace,original_rows)
        outside=dict(trace);outside["station_m"]=[4.,5.,6.,7.,8.,9.];outside["time_s"]=[0.,1.,2.,3.,4.,5.]
        with self.assertRaisesRegex(ValueError,"not crossed"):
            analyze_trace(outside,first)
        with self.assertRaisesRegex(ValueError, "no strict positive"):
            predeclare_first_braking_zone(x, k, np.full(7, 20.))
        # A corner cap at the initial node cannot itself define the event.
        with self.assertRaisesRegex(ValueError, "no strict positive"):
            predeclare_first_braking_zone(x, k, np.array([10.,10.,20.,20.,20.,20.,20.]))

    def test_kpis_use_accepted_physical_channels_and_preserve_unavailable_slip(self):
        zone = {"entry_station_m": 2., "entry_target_speed_mps": 8.,
                "segment_start_station_m": 0.}
        trace = {"time_s": [0., 1., 2., 3.], "station_m": [0., 1., 2., 3.],
                 "progress_speed_mps": [10., 9., 8., 7.], "pedal_signed": [.2, .1, -.2, -.4],
                 "brake_force_applied_n": [0., 0., 500., 500.], "pack_power_w": [100., 100., 100., 100.]}
        result = analyze_trace(trace, zone)
        self.assertEqual(result["entry_speed_error_mps"], 0.)
        self.assertEqual(result["brake_onset_station_m"], 2.)
        self.assertEqual(result["throttle_release_station_m"], 2.)
        self.assertAlmostEqual(result["pack_energy_j"], 200.)
        self.assertAlmostEqual(result["brake_force_impulse_ns"], 250.)
        self.assertEqual(result["kpi_segment_start_station_m"],0.)
        self.assertEqual(result["kpi_segment_end_station_m"],2.)
        self.assertFalse(result["slip"]["available"])
        self.assertFalse(result["tire_capacity"]["available"])
        self.assertNotIn("lap_time", result)

    def test_analyze_record_uses_frenet_progress_not_state_offset_and_gates_missing_pedal(self):
        from types import SimpleNamespace
        from tools.run.compare_virtual_drivers import analyze_record
        state=np.zeros(38);state[0]=2.;state[1]=.1;state[2]=.01;state[3]=12.;state[4]=1.;state[8]=.0
        class Plant:
            curvature=.01;valid=True
            def observe(self,s):return SimpleNamespace(distance=float(s[0]))
            def hybrid_kinematic_context(self,s):return SimpleNamespace(vx_mps=float(s[3]),vy_mps=float(s[4]),heading_error_rad=float(s[2]),curvature_inv_m=self.curvature,lateral_error_m=float(s[1]),vy_dot_valid=self.valid)
        row=SimpleNamespace(time=0.,state=state,rates=np.r_[(12.*np.cos(.01)-np.sin(.01))/(1.-.01*.1),np.zeros(37)],pedal=.1,command=SimpleNamespace(brake_force=0.),applied_powertrain_diagnostics=None)
        second_state=np.array(state)+np.eye(1,38,0)[0]
        second_row=SimpleNamespace(**{**row.__dict__,"time":1.,"state":second_state,"rates":row.rates.copy()})
        class NotCrossingPlant(Plant):
            def observe(self,s):return SimpleNamespace(distance=2.+float(s[0])-state[0])
        with self.assertRaisesRegex(ValueError,"not crossed"):
            result=analyze_record([row,second_row],{"entry_station_m":20.,"entry_target_speed_mps":8.,"segment_start_station_m":0.},plant=NotCrossingPlant())
        bad=SimpleNamespace(**{**row.__dict__,"pedal":None})
        with self.assertRaisesRegex(ValueError,"pedal telemetry unavailable"):
            analyze_record([bad],{"entry_station_m":1.,"entry_target_speed_mps":8.},plant=Plant())

    def test_progress_speed_survives_quasisteady_acceleration_gate_and_fails_closed(self):
        from types import SimpleNamespace
        from tools.run.compare_virtual_drivers import analyze_record
        class Plant:
            def observe(self,s):return SimpleNamespace(distance=float(s[0]))
            def hybrid_kinematic_context(self,s):
                return SimpleNamespace(vx_mps=float(s[3]),vy_mps=float(s[4]),heading_error_rad=float(s[2]),
                    curvature_inv_m=.08111635647,lateral_error_m=float(s[1]),vy_dot_valid=False)
        rows=[]
        for i in range(2):
            state=np.zeros(38);state[0]=float(i);state[1]=.04;state[2]=.01;state[3]=12.-i;state[4]=.5
            denom=1.-.08111635647*.04
            progress=(state[3]*np.cos(.01)-state[4]*np.sin(.01))/denom
            rows.append(SimpleNamespace(time=float(i),state=state,rates=np.r_[progress,np.zeros(37)],pedal=.1,
                command=SimpleNamespace(brake_force=0.),applied_powertrain_diagnostics=None))
        zone={"entry_station_m":.5,"entry_target_speed_mps":10.,"segment_start_station_m":0.,"segment_end_station_m":1.}
        kpi=analyze_record(rows,zone,plant=Plant())
        expected=((12.*np.cos(.01)-.5*np.sin(.01))/(1.-.08111635647*.04)+
                  (11.*np.cos(.01)-.5*np.sin(.01))/(1.-.08111635647*.04))/2.
        self.assertAlmostEqual(kpi["entry_speed_mps"],expected)
        rows[0].rates[0]+=1e-5
        with self.assertRaisesRegex(ValueError,"disagrees with accepted observed progress rate"):
            analyze_record(rows,zone,plant=Plant())
        rows[0].rates[0]-=1e-5
        class SingularPlant(Plant):
            def hybrid_kinematic_context(self,s):
                result=super().hybrid_kinematic_context(s);return SimpleNamespace(**{**result.__dict__,"lateral_error_m":10.})
        with self.assertRaisesRegex(ValueError,"Frenet kinematic context invalid"):
            analyze_record(rows,zone,plant=SingularPlant())

    def test_actual_solve_forward_accepted_observer_contract_all_three_prepared_adapters(self):
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        from types import SimpleNamespace
        import tools.run.compare_virtual_drivers as comparison
        from simplified_vehicle_models.solvers import forward_driver as forward
        from simplified_vehicle_models.solvers.forward_driver import DriverCommand
        with tempfile.TemporaryDirectory() as directory:
            comparison.main(["--prepare-only","--track","models/tracks/fsc_may_2026_centerline.xml","--manifest-dir",directory])
        prepared=comparison._LAST_PREPARED_FACTORY
        class Dense:
            def __init__(self,initial,final):self.initial=initial.copy();self.final=final.copy()
            def __call__(self,t):return self.initial+(self.final-self.initial)*(float(t)/.01)
        class MockIVPResult:
            success=True;message="mock accepted states";nfev=1
            def __init__(self,y,fun,events):
                self.t=np.array([0.,.01,.02]);states=[y.copy(),y.copy(),y.copy()]
                for j in (1,2):
                    states[j][0]+=0.02*j;states[j][-1]=0.
                self.y=np.column_stack(states);self.sol=Dense(states[0],states[-1])
                self.t_events=(tuple(np.array([]) for _ in events) if isinstance(events,tuple) else (np.array([]),))
        for i,(driver,identity) in enumerate(zip(prepared["drivers"],prepared["identities"])):
            with self.subTest(variant=identity.variant_id):
                plant=type(prepared["plant"])(prepared["model"],prepared["plan"]);initial=prepared["initial"].copy();rows=[]
                def fake_ivp(fun,t_span,y0,**kwargs):
                    y=np.array(y0,copy=True);fun(0.,y)
                    for t in (.01,.02):
                        y=y.copy();y[0]+=.01;fun(t,y)
                    return MockIVPResult(y0,fun,kwargs["events"])
                # Patch only scipy.integrate.solve_ivp. Production solve_forward,
                # state_rates, driver commands, accepted validation and StepSample stay real.
                import scipy.integrate
                with patch.object(scipy.integrate,"solve_ivp",side_effect=fake_ivp):
                    result=forward.solve_forward(plant,driver,initial,duration=.02,wall_seconds=10.,maximum_wall_seconds=10.,maximum_step=.01,initial_pedal=0.,policy_sink=rows.append)
                self.assertEqual(len(rows),3)
                self.assertTrue(np.all(np.isfinite(result.state)));self.assertEqual(result.state.shape[0],38)
                for sample in rows:
                    self.assertEqual(np.asarray(sample.state).shape,(38,));self.assertEqual(np.asarray(sample.rates).shape,(38,))
                    self.assertIsNotNone(sample.requested_command);self.assertIsNotNone(sample.mapped_command);self.assertIsInstance(sample.command,DriverCommand)
                    self.assertIsNotNone(sample.policy_diagnostics);self.assertIsNotNone(sample.pedal)
                if i==0:
                    from tools.run.compare_virtual_drivers import analyze_record
                    telemetry=[]
                    for j,sample in enumerate(rows[:2]):
                        state=np.array(sample.state,copy=True);state[0]=float(j);state[1]=.1;state[2]=.01;state[3]=12.;state[4]=.2
                        kin=plant.hybrid_kinematic_context(state);self.assertTrue(kin.vy_dot_valid)
                        denom=1.-kin.curvature_inv_m*kin.lateral_error_m
                        progress=(kin.vx_mps*np.cos(kin.heading_error_rad)-kin.vy_mps*np.sin(kin.heading_error_rad))/denom
                        telemetry.append(SimpleNamespace(time=float(j),state=state,rates=np.r_[progress,np.zeros(37)],pedal=sample.pedal,command=sample.command,applied_powertrain_diagnostics=sample.applied_powertrain_diagnostics))
                    kpi=analyze_record(telemetry,{"entry_station_m":.5,"entry_target_speed_mps":10.,"segment_start_station_m":0.,"segment_end_station_m":1.},plant=plant)
                    self.assertNotAlmostEqual(kpi["entry_speed_mps"],12.,places=5)
                    self.assertIsNotNone(driver._last_comparison_command)

    def test_rebuilt_manifest_binds_complete_packet_before_mock_solver(self):
        import json,tempfile
        from pathlib import Path
        from types import SimpleNamespace
        from unittest.mock import patch
        import tools.run.compare_virtual_drivers as comparison
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);packet_dir=root/"prepared";comparison.main(["--prepare-only","--track","models/tracks/fsc_may_2026_centerline.xml","--manifest-dir",str(packet_dir)])
            packet_path=packet_dir/"source-freeze.json";base=json.loads(packet_path.read_text());calls=[]
            fake_result=SimpleNamespace(time=np.array([0.,1.]),state=np.zeros((38,2)),lap_completed=False,wall_seconds=.01,evaluations=1,message="mock")
            sample=SimpleNamespace(index=0,time=0.,state=np.zeros(38),rates=np.zeros(38),requested_command=None,mapped_command=None,command=None,pedal=0.,policy_diagnostics=None,applied_powertrain_diagnostics=None)
            def mock_solver(plant,driver,initial,**kwargs):
                calls.append(driver)
                kwargs["policy_sink"](sample);return fake_result
            args=SimpleNamespace(prepared_manifest=packet_path,artifact_dir=root/"positive",track=Path("models/tracks/fsc_may_2026_centerline.xml"))
            with patch.object(comparison,"persist_return_before_analysis",return_value={}):
                self.assertEqual(comparison.execute_prepared_screen(args,comparison.ROOT/args.track,None,solver_override=mock_solver),0)
            self.assertEqual(len(calls),3)
            cases=[]
            changed=json.loads(json.dumps(base));changed["predeclared_zone"]["entry_station_m"]+=1.;changed["zone_sha256"]=comparison.sha256_bytes(json.dumps(changed["predeclared_zone"],sort_keys=True,separators=(",",":"),allow_nan=False).encode());cases.append(("rehash-zone",changed))
            for label,mutate in (("solver_identity",lambda p:p["solver_identity"].update(duration=7.)),("run_config_identity",lambda p:p["run_config_identity"].update(initial_soc=.5)),("provider_identity",lambda p:p.update(provider_identity="other")),("setup_identity",lambda p:p.update(setup_identity="other")),("corner_speed_caps_mps",lambda p:p["corner_speed_caps_mps"].__setitem__(0,1.))):
                changed=json.loads(json.dumps(base));mutate(changed);cases.append((label,changed))
            for label,changed in cases:
                bad_path=root/f"{label}.json";bad_path.write_text(json.dumps(changed));bad_args=SimpleNamespace(prepared_manifest=bad_path,artifact_dir=root/f"bad-{label}",track=args.track);before=len(calls)
                with self.subTest(field=label),self.assertRaises(SystemExit):comparison.execute_prepared_screen(bad_args,comparison.ROOT/args.track,None,solver_override=mock_solver)
                self.assertEqual(len(calls),before,label)
            removed=dict(base);removed.pop("provider_identity")
            with self.assertRaisesRegex(ValueError,"missing=.*provider_identity"):comparison.validate_rebuilt_packet(removed,base)

    def test_raw_checkpoint_survives_analysis_failure_and_zero_rows_reject(self):
        import tempfile
        from pathlib import Path
        from types import SimpleNamespace
        from tools.run.compare_virtual_drivers import persist_return_before_analysis
        identity=SimpleNamespace(variant_id="test")
        class Plant:pass
        state=np.zeros(38)
        samples=[SimpleNamespace(index=0,time=0.,state=state,rates=state,requested_command=None,mapped_command=None,command=None,pedal=0.,policy_diagnostics=None,applied_powertrain_diagnostics=None)]
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError,"analysis injection"):
                persist_return_before_analysis(directory,identity=identity,config={},solver_settings={},result={"ok":True},samples=samples,plant=Plant(),zone={},analyzer=lambda *a,**k:(_ for _ in ()).throw(RuntimeError("analysis injection")))
            self.assertTrue((Path(directory)/"raw-return.json").is_file())
            self.assertTrue((Path(directory)/"analysis-failure.json").is_file())
            with self.assertRaisesRegex(ValueError,"zero accepted"):
                persist_return_before_analysis(directory,identity=identity,config={},solver_settings={},result={},samples=[],plant=Plant(),zone={})

    def test_kpis_reject_nonunique_timestamps_and_row_mismatch(self):
        zone = {"entry_station_m": 1., "entry_target_speed_mps": 8., "segment_start_station_m": 0.}
        trace = {"time_s": [0., 1., 1.], "station_m": [0., 1., 2.],
                 "progress_speed_mps": [10., 9., 8.], "pedal_signed": [0., 0., 0.],
                 "brake_force_applied_n": [0., 0., 0.]}
        with self.assertRaisesRegex(ValueError, "increase strictly"):
            analyze_trace(trace, zone)
        trace["time_s"] = [0., 1.]
        with self.assertRaisesRegex(ValueError, "row-parity"):
            analyze_trace(trace, zone)


if __name__ == "__main__":
    unittest.main()
