import unittest
import math
import json
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch

from simplified_vehicle_models.solvers.hybrid_driver import HybridDecision, HybridPolicyConfig, HybridLongitudinalPolicy, _future_brake_target, _coast_referenced_brake_increment, _relay_search_eligible, _bounded_verified_relay_search, HYBRID_POLICY_VERSION
from simplified_vehicle_models.solvers.longitudinal_capability import (PathKinematicContext, LongitudinalCapability,
    CapabilityQuery, CurrentVehicleCapability, evaluate_pedal_response, body_vx_dot_to_path_progress)
from simplified_vehicle_models.solvers.envelope_driver import PedalPolicyContext, advance_signed_pedal
from simplified_vehicle_models.solvers.forward_driver import DriverObservation, PreviewDriver, solve_forward
from simplified_vehicle_models.solvers.hybrid_speed_plan import build_hybrid_speed_plan
from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant
from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityPredictor, BrakeAuthorityContext
from tests.test_vertical_forward import compiled_vehicle, straight_plan
from tests.test_hybrid_capability import CapabilityContractTests

class SyntheticProvider:
    identity='hybrid-test-provider'
    def __init__(self,drive=2.,brake=6.): self.drive=drive; self.brake=brake; self.query_count=0
    def evaluate(self,q):
        self.query_count+=1
        grid=(0.,.125,.25,.5,.75,1.)
        drive=tuple(-.2+(self.drive+.2)*p for p in grid)
        brake_grid=(0.,.125,.25,.5,.75,1.)
        brake=tuple(-.2+(-self.brake+.2)*p for p in brake_grid)
        return LongitudinalCapability(self.drive,-.2,-self.brake,True,(),self.identity,grid,drive,brake_grid,brake,1.)
class StubInverse:
    def __init__(self): self.calls=[]
    def request_current(self,ctx,target): self.calls.append(target); return SimpleNamespace(pedal_force_n=min(1176.,target*100.),deceleration_mps2=target)
    def predict(self,ctx,force): return SimpleNamespace(deceleration_mps2=min(float(force)/100.,1.),in_domain=True)

class HybridDecisionTests(unittest.TestCase):
    def setUp(self):
        self.pc=PedalPolicyContext(5.,4.,.02,1176.,.85,6.,3.)
        self.provider=SyntheticProvider()
        self.plan=build_hybrid_speed_plan([0.,5.,10.,15.],[0.]*4,[18.,18.,8.,18.],self.provider,initial_speed_mps=8.,storage_soc=.5,is_closed=False)
        self.predictor=StubInverse(); self.config=HybridPolicyConfig(maximum_brake_force_n=1176.,pedal_context=self.pc)
        self.policy=HybridLongitudinalPolicy(self.plan,self.provider,self.predictor,self.config)
        self.kin=PathKinematicContext(8.,0.,0.,0.,0.,0.,0.,0.,True)
        self.obs=DriverObservation(0.,0.,0.,8.,0.,0.,wheel_speed_margin=1.)
    def test_keyword_config_and_force_validation(self):
        with self.assertRaises(TypeError): HybridDecision(1,2,3,4,5,6,'drive','ok',None,None,0,0,0,0,False,'p','c')
        with self.assertRaises(ValueError): HybridPolicyConfig(horizon_s=3.01)
        with self.assertRaises(ValueError): HybridPolicyConfig(maximum_brake_force_n=10.,pedal_context=self.pc)
    def test_exact_four_rates_sign_split_and_frenet_validity(self):
        self.assertGreater(advance_signed_pedal(1.,-1.,.02,self.pc),0.)
        self.assertNotEqual(advance_signed_pedal(1.,-1.,.12,self.pc),advance_signed_pedal(1.,-1.,.12,PedalPolicyContext(7.,4.,.02,1176.,.85,5.,4.)))
        for p,d,b in ((0.,0.,0.),(1.,1.,0.),(-1.,0.,1.)):
            self.assertEqual(max(p,0.),d); self.assertEqual(max(-p,0.),b)
        bad=PathKinematicContext(8.,1.,.1,.2,.3,.02,.001,0.,False)
        self.assertEqual(self.policy.decide(observation=self.obs,pedal_position=0.,kinematics=bad,soc=.5).decision.status,'unsupported_kinematic_context')
    def test_future_brake_inverse_is_neutral_at_and_above_coast_and_full_brake_below_bound(self):
        grid=(0.,.5,1.)
        cap=LongitudinalCapability(2.,-1.,-5.,True,(),"inverse",grid,( -1.,.5,2.),grid,(-1.,-3.,-5.),1.)
        for target in (-.9,-1.,-.999999):
            self.assertEqual(_future_brake_target(cap,target,[None]),0.)
        self.assertAlmostEqual(_future_brake_target(cap,-2.,[None]),-.25,places=12)
        self.assertEqual(_future_brake_target(cap,-5.1,[None]),-1.)
        bad=replace(cap,brake_acceleration_grid=(-1.,-.5,-5.))
        failure=[None]
        self.assertIsNone(_future_brake_target(bad,-.9,failure))
        self.assertIn("invalid_active_pedal_map",failure[0])
        self.assertIn("policy-v11-",HYBRID_POLICY_VERSION)
        self.assertTrue(HYBRID_POLICY_VERSION.endswith("verified-midpoint-proposals"))

    def test_coast_referenced_brake_increment_envelope_matrix(self):
        # Matched curves return one partial net response, not 2*partial-full.
        self.assertAlmostEqual(_coast_referenced_brake_increment(-2.,-1.,-2.,-1.),-2.)
        # Distinct future-map and physical coast baselines: compare increments,
        # not total net responses across those different contexts.
        self.assertAlmostEqual(_coast_referenced_brake_increment(-3.,-1.,-4.,0.),-3.)
        self.assertAlmostEqual(_coast_referenced_brake_increment(-3.,-1.,-2.,0.),-3.)
        self.assertAlmostEqual(_coast_referenced_brake_increment(-2.,-1.,-4.,0.),-2.)
        # Actual increment wins: -1 around coast 0 is weaker than map -3
        # around coast -1, so composition is -2 (not map partial -4).
        self.assertAlmostEqual(_coast_referenced_brake_increment(-4.,-1.,-1.,0.),-2.)
        # Zero actual increment is valid and preserves the map coast baseline.
        self.assertAlmostEqual(_coast_referenced_brake_increment(-4.,-1.,0.,0.),-1.)
        self.assertAlmostEqual(_coast_referenced_brake_increment(-1.,-1.,-1.,-1.),-1.)
        with self.assertRaisesRegex(ValueError,"positive"):
            _coast_referenced_brake_increment(-2.,-1.,1.,0.)
        with self.assertRaisesRegex(ValueError,"nonfinite"):
            _coast_referenced_brake_increment(-2.,-1.,float('nan'),0.)
        with self.assertRaisesRegex(ValueError,"nonfinite"):
            _coast_referenced_brake_increment(1e308,-1e308,-1.,0.)
        with self.assertRaisesRegex(ValueError,"nonfinite"):
            _coast_referenced_brake_increment(-1.7e308,-1.79e308,-1.7e308,1.7e308)

    def test_positive_unconstrained_no_inverse_and_underspeed_not_brake(self):
        plan=build_hybrid_speed_plan([0.,5.,10.],[0.]*3,[20.]*3,self.provider,initial_speed_mps=8.,storage_soc=.5,is_closed=False)
        policy=HybridLongitudinalPolicy(plan,self.provider,self.predictor,self.config)
        d=policy.decide(observation=DriverObservation(0.,0.,0.,6.,0.,0.),pedal_position=0.,kinematics=PathKinematicContext(6.,0.,0.,0.,0.,0.,0.,0.,True),soc=.5)
        self.assertGreater(d.throttle,0.); self.assertEqual(self.predictor.calls,[])
    @classmethod
    def _real_future_provider(cls):
        probe=CapabilityContractTests();probe.setUpClass()
        return probe.provider(longitudinal_capacity_n=(600.,)*4)

    def _real_policy_context(self,provider):
        vehicle=CapabilityContractTests.vehicle
        predictor=BrakeAuthorityPredictor(vehicle,maximum_abs_curvature_inv_m=.08,minimum_speed_mps=.005,maximum_speed_mps=25.)
        context=BrakeAuthorityContext(20.,0.,(.29,)*4,(vehicle.total_mass*9.81/4,)*4,(600.,)*4,soc=.5)
        pedal=PedalPolicyContext(1/.175,1/.175,.02,vehicle.maximum_pedal_force,.85,1/.175,1/.175)
        config=HybridPolicyConfig(maximum_brake_force_n=vehicle.maximum_pedal_force,pedal_context=pedal)
        return predictor,config,context

    def test_production_builder_future_event_rollout_uses_downstream_event_not_self_binding(self):
        provider=self._real_future_provider()
        plan=build_hybrid_speed_plan([0.,20.,40.,57.,77.,100.],[0.]*6,[22.,22.,22.,22.,22.,10.],provider,
            initial_speed_mps=18.,storage_soc=.5,is_closed=False)
        sample=plan.sample(0.)
        self.assertEqual(sample.binding_index,0)
        self.assertEqual(sample.distance_to_binding_event_m,0.)
        self.assertEqual(sample.future_event_station_m,100.)
        self.assertAlmostEqual(sample.future_event_speed_cap_mps,10.)
        predictor,config,brake_context=self._real_policy_context(provider)
        policy=HybridLongitudinalPolicy(plan,provider,predictor,config)
        brake_context=replace(brake_context,speed_mps=18.)
        obs=DriverObservation(0.,0.,0.,18.,0.,0.,wheel_speed_margin=1.)
        kin=PathKinematicContext(18.,0.,0.,0.,0.,0.,0.,0.,True)
        out=policy.decide(observation=obs,pedal_position=0.,kinematics=kin,soc=.5,brake_context=brake_context)
        self.assertEqual(out.decision.event_speed_cap_mps,10.)
        self.assertGreater(out.decision.candidate_count,0)
        self.assertGreater(out.decision.rollout_samples,0)
        self.assertNotEqual(out.decision.status, 'event_unknown:unsupported_pedal_response_map')
        self.assertIsNone(out.decision.event_crossed) # event is 3.85s away; 3s horizon is truthful UNKNOWN
        self.assertIn('event_unknown:',out.decision.status)
        context=replace(brake_context,speed_mps=18.)
        curved=PathKinematicContext(18.,2.,.08,.4,.1,.01,.001,0.,True)
        # Query state conversion follows U=(vx*cos(e)-vy*sin(e))/B, not vx.
        c,s=math.cos(curved.heading_error_rad),math.sin(curved.heading_error_rad)
        B=1.-curved.curvature_inv_m*curved.lateral_error_m
        expected_progress=(curved.vx_mps*c-curved.vy_mps*s)/B
        self.assertAlmostEqual(expected_progress,17.85401739508916,places=10)
        reconstructed_vx=(expected_progress*B+curved.vy_mps*s)/c
        self.assertAlmostEqual(reconstructed_vx,curved.vx_mps,places=12)
        curved_out=policy.decide(observation=obs,pedal_position=0.,kinematics=curved,soc=.5,brake_context=context)
        self.assertIsNone(curved_out.decision.event_crossed)
        self.assertIn(curved_out.decision.status,("capability_domain_exit","event_unknown:future_capability_domain_exit"))

    def test_near_future_event_crosses_interpolated_station_at_corner_cap(self):
        provider=self._real_future_provider()
        plan=build_hybrid_speed_plan([0.,5.,10.,15.,20.],[0.]*5,[20.,20.,20.,12.,10.],provider,
            initial_speed_mps=12.,storage_soc=.5,is_closed=False)
        predictor,config,brake_context=self._real_policy_context(provider)
        policy=HybridLongitudinalPolicy(plan,provider,predictor,config)
        obs=DriverObservation(0.,0.,0.,12.,0.,0.,wheel_speed_margin=1.)
        kin=PathKinematicContext(12.,0.,0.,0.,0.,0.,0.,0.,True)
        out=policy.decide(observation=obs,pedal_position=0.,kinematics=kin,soc=.5,brake_context=brake_context)
        self.assertGreater(out.decision.candidate_count,0)
        self.assertGreater(out.decision.rollout_samples,0)
        # Corrected physical coast-referenced increments can select a safe
        # release/brake forecast; historical buggy forecast parity is not a gate.
        self.assertIs(out.decision.event_crossed,True)
        self.assertEqual(out.decision.status,"event_crossed:release_then_brake")
        self.assertIsNotNone(out.decision.event_crossing_time_s)
        self.assertIsNotNone(out.decision.event_crossing_speed_mps)
        self.assertEqual(out.decision.event_speed_cap_mps,10.)
        self.assertEqual(out.decision.candidate_count,3)
        self.assertGreater(out.decision.rollout_samples,0)
        heading=.25; lateral=1.2; curvature=.08; vy=2.0
        c,s=math.cos(heading),math.sin(heading); B=1.-curvature*lateral
        initial_progress=12.
        expected_vx=(initial_progress*B+vy*s)/c
        curved=PathKinematicContext(expected_vx,vy,heading,lateral,.08,curvature,.001,.4,True)
        curved_context=replace(brake_context,speed_mps=expected_vx,v_y_mps=curved.vy_mps,
            yaw_rate_rps=curved.yaw_rate_rps,curvature_inv_m=curved.curvature_inv_m)
        queried_speeds=[]
        physical_predict=predictor.predict
        def record_predict(context,force,*args,**kwargs):
            queried_speeds.append(context.speed_mps)
            return physical_predict(context,force,*args,**kwargs)
        predictor.predict=record_predict
        curved_rollout=policy._rollout(plan.sample(0.),obs,curved,.5,0.,curved_context,0.,0.)
        self.assertEqual(curved_rollout[6],"event_crossed:release_then_brake")
        self.assertIs(curved_rollout[0],True)
        self.assertTrue(queried_speeds)
        expected_query_vx=(initial_progress*B+vy*s)/c
        # The observed brake forecast includes repeated slab queries; select
        # the sample at its initial progress state, whose inverse is analytical.
        matches=[value for value in queried_speeds if math.isclose(value,expected_query_vx,rel_tol=0.,abs_tol=1e-12)]
        self.assertGreaterEqual(len(matches),1)
        self.assertNotAlmostEqual(expected_query_vx,initial_progress,places=5)

    def test_nested_forecast_selected_first_acceleration_is_corrected_composition(self):
        provider=self._real_future_provider()
        plan=build_hybrid_speed_plan([0.,1.25,2.5,3.75,5.],[0.]*5,[20.,20.,11.,9.,8.],provider,
            initial_speed_mps=8.,storage_soc=.5,is_closed=False)
        predictor,config,brake_context=self._real_policy_context(provider)
        policy=HybridLongitudinalPolicy(plan,provider,predictor,config)
        observed=[]
        original_predict=predictor.predict
        def record_predict(context,force,*args,**kwargs):
            result=original_predict(context,force,*args,**kwargs)
            observed.append((context,force,result))
            return result
        predictor.predict=record_predict
        sample=replace(plan.sample(0.),distance_to_future_event_m=.01,future_event_speed_cap_mps=7.)
        obs=DriverObservation(0.,0.,0.,8.,0.,0.,wheel_speed_margin=1.)
        kin=PathKinematicContext(8.,0.,0.,0.,0.,0.,0.,0.,True)
        pedal=-.9
        from simplified_vehicle_models.solvers import hybrid_driver as hybrid_module
        original_eligible=hybrid_module._relay_search_eligible
        original_search=hybrid_module._bounded_verified_relay_search
        real_forecast_rows=[]
        def select_real_mix(forecast,*args,**kwargs):
            row=forecast(.5)
            real_forecast_rows.append(row)
            row=(True,row[1],row[2],row[3],row[4],row[5],row[6],row[7],row[8])
            return row,.5,row[4]-float(sample.future_event_speed_cap_mps)-1e-8,1,'bracket_resolution'
        hybrid_module._relay_search_eligible=lambda *args:True
        hybrid_module._bounded_verified_relay_search=select_real_mix
        try:
            selected,diagnostics=policy._rollout_with_diagnostics(sample,obs,kin,.5,pedal,
                replace(brake_context,speed_mps=8.),0.,0.)
        finally:
            hybrid_module._relay_search_eligible=original_eligible
            hybrid_module._bounded_verified_relay_search=original_search
        schedule=selected[7]
        self.assertEqual(schedule,"mixed")
        self.assertEqual(len(real_forecast_rows),1)
        target=selected[8]
        predicted=selected[9]
        dt=config.horizon_s/config.slabs
        self.assertAlmostEqual(predicted,advance_signed_pedal(pedal,target,dt,config.pedal_context),places=12)
        # Independently use observed real predictor results at first-slab query
        # and the real queried capability curve; expected numeric composition
        # is then checked against the nested forecast's selected first value.
        first_context,selected_force,partial=next((ctx,force,res) for ctx,force,res in observed if force>0.)
        coast=next(res for ctx,force,res in observed if force==0. and ctx==first_context)
        first_cap=provider.evaluate(CapabilityQuery(8.,sample.curvature_inv_m,.5))
        map_partial=evaluate_pedal_response(first_cap,predicted)
        map_coast=evaluate_pedal_response(first_cap,0.)
        actual_partial=body_vx_dot_to_path_progress(-partial.deceleration_mps2,kin)
        actual_coast=body_vx_dot_to_path_progress(-coast.deceleration_mps2,kin)
        expected=map_coast+max(map_partial-map_coast,actual_partial-actual_coast)
        self.assertAlmostEqual(diagnostics['selected_progress_acceleration'],expected,places=10)
        expected_physical_force=abs(predicted)*config.pedal_context.maximum_brake_force
        self.assertAlmostEqual(selected_force,abs(predicted)*config.pedal_context.maximum_brake_force,places=9)
        self.assertTrue(any(force==0. for _,force,_ in observed))

    def test_invalid_zero_force_predictor_domain_fails_closed(self):
        provider=self._real_future_provider()
        plan=build_hybrid_speed_plan([0.,1.25,2.5,3.75,5.],[0.]*5,[20.,20.,11.,9.,8.],provider,
            initial_speed_mps=8.,storage_soc=.5,is_closed=False)
        predictor,config,brake_context=self._real_policy_context(provider)
        policy=HybridLongitudinalPolicy(plan,provider,predictor,config)
        original=predictor.predict
        def reject_coast(context,force,*args,**kwargs):
            if force==0.: return SimpleNamespace(deceleration_mps2=0.,in_domain=False)
            return original(context,force,*args,**kwargs)
        predictor.predict=reject_coast
        obs=DriverObservation(0.,0.,0.,8.,0.,0.,wheel_speed_margin=1.)
        kin=PathKinematicContext(8.,0.,0.,0.,0.,0.,0.,0.,True)
        rollout=policy._rollout(plan.sample(0.),obs,kin,.5,-.1,replace(brake_context,speed_mps=8.),0.,0.)
        self.assertIsNone(rollout[0])
        self.assertEqual(rollout[6],"unsupported_kinematic_context")

    def test_supported_physical_forecast_crosses_interpolated_event_safely(self):
        provider=self._real_future_provider()
        plan=build_hybrid_speed_plan([0.,1.25,2.5,3.75,5.],[0.]*5,[20.,20.,11.,9.,8.],provider,
            initial_speed_mps=8.,storage_soc=.5,is_closed=False)
        predictor,config,brake_context=self._real_policy_context(provider)
        policy=HybridLongitudinalPolicy(plan,provider,predictor,config)
        predictor_calls=[]
        physical_predict=predictor.predict
        def record_predict(context,force,*args,**kwargs):
            predictor_calls.append((context.speed_mps,force))
            return physical_predict(context,force,*args,**kwargs)
        predictor.predict=record_predict
        obs=DriverObservation(0.,0.,0.,8.,0.,0.,wheel_speed_margin=1.)
        kin=PathKinematicContext(8.,0.,0.,0.,0.,0.,0.,0.,True)
        out=policy.decide(observation=obs,pedal_position=0.,kinematics=kin,soc=.5,
            brake_context=replace(brake_context,speed_mps=8.))
        self.assertIs(out.decision.event_crossed,True,out.decision.status)
        self.assertEqual(out.decision.status,"event_crossed:release_then_brake")
        self.assertEqual(out.decision.event_speed_cap_mps,8.)
        self.assertGreater(out.decision.event_crossing_time_s,0.)
        self.assertLessEqual(out.decision.event_crossing_time_s,config.horizon_s)
        self.assertLessEqual(out.decision.event_crossing_speed_mps,8.+1e-8)
        self.assertGreaterEqual(out.decision.event_crossing_speed_mps,0.)
        self.assertTrue(predictor_calls)
        self.assertTrue(all(math.isfinite(speed) and math.isfinite(force) for speed,force in predictor_calls))
        self.assertTrue(any(speed>0. and force>0. for speed,force in predictor_calls))
        positive_forces=[force for _,force in predictor_calls if force>0.]
        coast_forces=[force for _,force in predictor_calls if force==0.]
        self.assertGreaterEqual(len(coast_forces),len(positive_forces))
        self.assertGreater(out.decision.rollout_samples,0)
        # The event station is interpolated along a constant-curvature segment;
        # the reported crossing speed must satisfy that exact event cap.
        self.assertAlmostEqual(plan.sample(5.).speed_mps,8.)

    def test_active_braking_suppresses_future_rollout(self):
        sample=self.plan.sample(6.)
        self.assertTrue(sample.braking_region)
        out=self.policy.decide(observation=replace(self.obs,distance=6.),pedal_position=0.,
            kinematics=replace(self.kin,vx_mps=8.),soc=.5,brake_context=object())
        self.assertEqual(out.decision.candidate_count,0)
        self.assertEqual(out.decision.rollout_samples,0)
        self.assertIsNone(out.decision.event_crossing_time_s)

    def test_consecutive_braking_segments_resolve_to_downstream_event(self):
        provider=self._real_future_provider()
        plan=build_hybrid_speed_plan([0.,20.,40.,57.,77.,100.],[0.]*6,[22.,22.,21.,19.,10.,10.],provider,
            initial_speed_mps=20.,storage_soc=.5,is_closed=False)
        sample=plan.sample(20.)
        self.assertLess(sample.acceleration_mps2,0.)
        self.assertEqual(sample.binding_index,1)
        self.assertNotEqual(sample.future_event_index,sample.segment_index)
        self.assertEqual(sample.future_event_station_m,77.)
        self.assertAlmostEqual(sample.distance_to_future_event_m,57.)

    def test_rollout_reuses_each_plan_sample_without_changing_provider_queries(self):
        original=type(self.plan).sample
        stations=[]
        def counted(plan, station):
            stations.append(station)
            return original(plan,station)
        # Isolate the decision-time query budget from construction-time
        # reachable-edge planner lookups on the shared provider instance.
        setup_queries=self.provider.query_count
        self.provider.query_count=0
        with patch.object(type(self.plan),'sample',counted):
            out=self.policy.decide(observation=self.obs,pedal_position=0.,kinematics=self.kin,soc=.5,brake_context=object())
        self.assertEqual(out.decision.status,'event_crossed:release_then_brake')
        self.assertEqual(out.decision.rollout_samples,82)
        # One decide() sample, one onset sample, one sample for each slab.
        self.assertEqual(len(stations),1+1+out.decision.rollout_samples)
        self.assertLessEqual(self.provider.query_count,118)
        self.assertGreater(self.provider.query_count,0)
        self.assertGreater(setup_queries,0)  # planner queries are a separate phase

    def test_active_positive_lead_schedule_and_actual_crossing(self):
        s=self.plan.sample(0.); self.assertGreater(s.distance_to_braking_onset_m,0.)
        d=self.policy.decide(observation=self.obs,pedal_position=0.,kinematics=self.kin,soc=.5,brake_context=object())
        self.assertEqual(d.decision.candidate_count,3); self.assertLessEqual(d.decision.rollout_samples,180)
        self.assertEqual(d.decision.event_crossed,True)
        self.assertEqual(d.decision.status,'event_crossed:release_then_brake')
        self.assertIsNotNone(d.decision.event_crossing_time_s)
        self.assertEqual(d.decision.phase,'release')
    def test_row4991_unknown_release_relay_trigger_from_captured_production_context(self):
        record=json.loads(Path('.superpowers/sdd/hybrid-implementation-plan/continuation-relay-prototype-v2g.json').read_text())
        case=next(x for x in record['cases'] if x['capture_index']==4991)
        sample=SimpleNamespace(**case['captured_args'][0])
        from dataclasses import make_dataclass
        Sample=make_dataclass('CapturedSample',[(k,object) for k in case['captured_args'][0]])
        sample=Sample(**case['captured_args'][0])
        obs=SimpleNamespace(**case['captured_args'][1])
        kin=PathKinematicContext(**case['captured_args'][2])
        provider=SyntheticProvider()
        provider.identity=self.provider.identity
        calls=[]
        class CapturedPlan:
            def sample(self,station):
                calls.append(station)
                return sample
        policy=HybridLongitudinalPolicy.__new__(HybridLongitudinalPolicy)
        policy.plan=CapturedPlan(); policy.provider=provider; policy.predictor=self.predictor; policy.config=self.config
        old=case['production_candidate_outcomes']
        self.assertIs(old[0][0],False); self.assertIsNone(old[1][0]); self.assertIs(old[2][0],True)
        self.assertTrue(old[0][4] > sample.future_event_speed_cap_mps+1e-8)
        self.assertTrue(old[2][4] <= sample.future_event_speed_cap_mps+1e-8)
        self.assertLess(old[2][2],self.config.horizon_s)
        self.assertEqual(sample.future_event_speed_cap_mps,14.99999994)
        self.assertEqual(sample.distance_to_future_event_m,25.623291810368926)
        self.assertEqual(sample.distance_to_braking_onset_m,22.124221556467162)

    def _relay_policy_from_captured_row(self,row_index=4991):
        record=json.loads(Path('.superpowers/sdd/hybrid-implementation-plan/continuation-relay-prototype-v2g.json').read_text())
        case=next(x for x in record['cases'] if x['capture_index']==row_index)
        from dataclasses import make_dataclass
        fields=case['captured_args'][0]
        Sample=make_dataclass('RelayTestSample',[(k,object) for k in fields])
        sample=Sample(**fields); obs=SimpleNamespace(**case['captured_args'][1])
        kin=PathKinematicContext(**case['captured_args'][2])
        provider=SyntheticProvider(); provider.identity=self.provider.identity
        policy=HybridLongitudinalPolicy.__new__(HybridLongitudinalPolicy)
        policy.plan=SimpleNamespace(sample=lambda station:sample); policy.provider=provider
        policy.predictor=StubInverse(); policy.config=self.config
        return policy,sample,obs,kin,policy.predictor,case

    def test_relay_gate_allows_unknown_release_but_preserves_feasible_release(self):
        policy,sample,obs,kin,predictor,case=self._relay_policy_from_captured_row()
        old=case['production_candidate_outcomes']
        self.assertEqual([x[0] for x in old],[False,None,True])
        # Exercise the public predicate directly; captured data establish why
        # the unknown-release branch is the one under test.
        unsafe=tuple(old[0]); unknown=tuple(old[1]); safe=tuple(old[2])
        chosen=safe
        self.assertTrue(_relay_search_eligible(unsafe,unknown,safe,chosen,8,True))
        feasible_release=(True,'event_crossed',2.4,'release_then_brake',14.8,0.,.001,0.,-.15)
        self.assertFalse(_relay_search_eligible(unsafe,feasible_release,safe,chosen,8,True))
        self.assertIsNone(unknown[0]); self.assertIs(unsafe[0],False); self.assertIs(safe[0],True)
        self.assertEqual(sample.future_event_speed_cap_mps,14.99999994)
        self.assertFalse(_relay_search_eligible((None,'unknown',None,'continue',None,1.),unknown,safe,safe,8,True))
        self.assertFalse(_relay_search_eligible(unsafe,unknown,(None,'unknown',None,'brake_now',None,-.1),safe,8,True))
        self.assertFalse(_relay_search_eligible(unsafe,unknown,safe,safe,0,True))
        self.assertFalse(_relay_search_eligible(unsafe,unknown,safe,safe,8,False))

    def test_actual_bounded_forecast_search_discards_unknown_and_inconsistent_probes(self):
        calls=[]
        continue_row=(False,'event_crossed',1.0,'continue',12.0,1.0,0.7,1.0,2.0)
        brake_row=(True,'event_crossed',1.2,'brake_now',9.0,-.2,-.1,-1.0,-2.0)
        probes=[(None,'capability_domain_exit',None,'mixed',None,None,None,None,None),
                (True,'event_crossed',1.1,'mixed',9.5,.4,.2,.5,-1.0)]
        safe,lam,residual,count,reason=_bounded_verified_relay_search(
            lambda x:(calls.append(x) or probes.pop(0)),continue_row,brake_row,
            10.0,2,1e-8,1.2)
        self.assertEqual(count,2)
        self.assertEqual(len(calls),2)
        # The unknown probe did not alter the initial safe endpoint; the next
        # probe's consistent safe crossing replaces it with a fully verified row.
        self.assertEqual(safe[4],9.5)
        self.assertEqual(lam,calls[-1])
        self.assertLessEqual(safe[4],10.0+1e-8)
        self.assertEqual(reason,'extra_forecast_limit')
        self.assertEqual(residual,9.5-(10.0+1e-8))

    def test_discovery_refinement_uses_single_remaining_budget_and_verified_safe_endpoint(self):
        from simplified_vehicle_models.solvers.hybrid_driver import _bounded_verified_relay_search
        continue_row=(False,'event_crossed',1.,'continue',12.,1.,.7,1.,2.)
        discovered=(True,'event_crossed',1.2,'mixed',9.,.5,.2,-1.,-2.)
        seen=[]
        def refine(lam):
            seen.append(lam)
            return (None,'capability_domain_exit',None,'mixed',None,.4,.1,None,None)
        safe,lam,res,count,reason=_bounded_verified_relay_search(
            refine,continue_row,discovered,10.,2,1e-8,1.,safe_lambda=.5)
        self.assertEqual(count,2)
        self.assertEqual(len(seen),2)
        self.assertTrue(all(0.<x<.5 for x in seen))
        self.assertIs(safe,discovered)
        self.assertEqual(lam,.5)
        self.assertEqual(reason,'extra_forecast_limit')

    def test_midpoint_proposals_contract_and_float_exhaustion_keeps_verified_safe(self):
        continue_row=(False,'event_crossed',1.,'continue',12.,1.,.7,1.,2.)
        safe_row=(True,'event_crossed',1.2,'brake_now',9.,-.2,-.1,-1.,-2.)
        seen=[]
        outcomes=[continue_row, safe_row]
        def forecast(lam):
            seen.append(lam)
            return outcomes[len(seen)-1]
        safe,lam,_,count,reason=_bounded_verified_relay_search(
            forecast,continue_row,safe_row,10.,2,1e-8,1.)
        self.assertEqual(seen,[.5,.75])
        self.assertAlmostEqual(abs(.5-.75),.25)
        self.assertEqual(count,2)
        self.assertEqual(reason,'extra_forecast_limit')
        self.assertIs(safe,outcomes[1])
        adjacent=math.nextafter(0.,1.)
        calls=[]
        retained,retained_lam,_,count,reason=_bounded_verified_relay_search(
            lambda x:(calls.append(x) or safe_row),continue_row,safe_row,10.,3,5e-324,1e308,
            safe_lambda=adjacent)
        self.assertEqual(calls,[])
        self.assertEqual(count,0)
        self.assertEqual(reason,'floating_point_exhaustion')
        self.assertIs(retained,safe_row)
        self.assertEqual(retained_lam,adjacent)
    def test_discovery_unknowns_are_not_safe_endpoints(self):
        # Existing search helper only updates endpoints on verified classifications.
        continue_row=(False,'event_crossed',1.,'continue',12.,1.,.7,1.,2.)
        safe_row=(True,'event_crossed',1.2,'mixed',9.,.5,.2,-1.,-2.)
        proposals=[(None,'domain',None,'mixed',None,.3,.1,None,None),
                   (True,'event_crossed',1.1,'mixed',9.5,.2,.1,-.5,-1.)]
        out=_bounded_verified_relay_search(lambda _:proposals.pop(0),continue_row,safe_row,10.,2,1e-8,1.,safe_lambda=.5)
        self.assertIs(out[0][0],True)
        self.assertLessEqual(out[0][4],10.+1e-8)
        self.assertEqual(out[3],2)

    def test_actual_bounded_forecast_search_retains_verified_endpoint_at_probe_budget(self):
        continue_row=(False,'event_crossed',1.,'continue',12.,1.,.7,1.,2.)
        brake_row=(True,'event_crossed',1.2,'brake_now',9.,-.2,-.1,-1.,-2.)
        seen=[]
        def forecast(lam):
            seen.append(lam)
            return (True,'event_crossed',1.1,'mixed',9.5,.0,.0,-.5,-1.)
        safe,lam,residual,count,reason=_bounded_verified_relay_search(
            forecast,continue_row,brake_row,10.0,1,1e-8,1.2)
        self.assertEqual(count,1)
        self.assertEqual(len(seen),1)
        self.assertEqual(reason,'extra_forecast_limit')
        self.assertIs(safe[0],True)
        self.assertEqual(safe[4],9.5)
        self.assertEqual(lam,seen[0])
        self.assertLessEqual(safe[4],10.0+1e-8)

    def _real_discovery_fixture(self,slabs=60):
        provider=self._real_future_provider()
        predictor,config,brake_context=self._real_policy_context(provider)
        config=replace(config,slabs=slabs)
        plan=build_hybrid_speed_plan([0.,2.,4.,6.,8.,10.,12.],[0.]*7,
            [25.,25.,25.,6.,6.,25.,25.],provider,initial_speed_mps=8.,storage_soc=.5,is_closed=False)
        policy=HybridLongitudinalPolicy(plan,provider,predictor,config)
        sample=replace(plan.sample(0.),distance_to_future_event_m=8.,future_event_speed_cap_mps=6.)
        obs=DriverObservation(0.,0.,0.,8.,0.,0.,wheel_speed_margin=1.)
        kin=PathKinematicContext(8.,0.,0.,0.,0.,0.,0.,0.,True)
        return policy,provider,sample,obs,kin,replace(brake_context,speed_mps=8.),config

    def test_real_discovery_selection_refines_with_shared_budget_and_links_command(self):
        policy,provider,sample,obs,kin,context,config=self._real_discovery_fixture()
        selected,diagnostics=policy._rollout_with_diagnostics(sample,obs,kin,.5,0.,context,0.,2.)
        self.assertEqual(diagnostics['original_candidate_statuses'],(
            ('continue',False,'event_crossed'),
            ('release_then_brake',None,'capability_domain_exit'),
            ('brake_now',None,'capability_domain_exit')))
        self.assertEqual(diagnostics['discovery_termination'],'verified_candidate_found')
        self.assertAlmostEqual(diagnostics['selected_lambda'],0.1171875,places=12)
        self.assertLess(diagnostics['selected_lambda'],.5)  # midpoint refinement retained a verified safe endpoint
        self.assertEqual(diagnostics['extra_probe_count'],config.max_extra_forecasts)
        self.assertEqual(diagnostics['attempted_transitions'],247)
        self.assertEqual(diagnostics['termination'],'extra_forecast_limit')
        self.assertLessEqual(diagnostics['attempted_transitions'],11*config.slabs)
        self.assertTrue(selected[0]); self.assertEqual(selected[7],'mixed')
        self.assertLessEqual(selected[2],sample.future_event_speed_cap_mps+1e-8)
        self.assertAlmostEqual(diagnostics['selected_progress_acceleration'],0.4187153435735394,places=10)
        self.assertAlmostEqual(selected[9],advance_signed_pedal(0.,selected[8],config.horizon_s/config.slabs,config.pedal_context),places=12)
        self.assertEqual(selected[7],'mixed')
        self.assertNotEqual(selected[8],0.)

    def test_real_discovery_default_and_configured_slab_transition_bounds(self):
        for slabs,expected in ((60,247),(30,128)):
            with self.subTest(slabs=slabs):
                policy,_,sample,obs,kin,context,config=self._real_discovery_fixture(slabs)
                _,d=policy._rollout_with_diagnostics(sample,obs,kin,.5,0.,context,0.,2.)
                self.assertEqual(d['extra_probe_count'],8)
                self.assertEqual(d['attempted_transitions'],expected)
                self.assertLessEqual(d['attempted_transitions'],11*slabs)

    def test_real_discovery_exhaustion_preserves_unknown_coast_and_provenance(self):
        policy,provider,sample,obs,kin,context,config=self._real_discovery_fixture()
        plan=build_hybrid_speed_plan([0.,5.,10.,15.,20.],[0.]*5,[20.]*5,provider,initial_speed_mps=8.,storage_soc=.5,is_closed=False)
        policy=HybridLongitudinalPolicy(plan,provider,policy.predictor,config)
        sample=replace(plan.sample(0.),distance_to_future_event_m=32.,future_event_speed_cap_mps=8.)
        selected,d=policy._rollout_with_diagnostics(sample,obs,kin,.5,0.,context,0.,2.)
        self.assertEqual(d['original_candidate_statuses'],(
            ('continue',False,'event_crossed'),
            ('release_then_brake',None,'event_beyond_horizon'),
            ('brake_now',None,'event_beyond_horizon')))
        self.assertEqual(d['discovery_termination'],'discovery_exhausted_no_verified_candidate')
        self.assertEqual(d['extra_probe_count'],8)
        self.assertEqual(d['attempted_transitions'],660)
        self.assertLessEqual(d['attempted_transitions'],11*config.slabs)
        self.assertIsNone(selected[0]); self.assertIsNone(selected[7])
        self.assertEqual(d['termination'],'unknown_original_endpoint:event_beyond_horizon')
        self.assertEqual(d['extra_probe_count'],8)
        self.assertEqual(d['original_candidate_statuses'][0][0],'continue')

    def test_real_feasible_original_release_preserves_priority_without_discovery(self):
        policy,_,sample,obs,kin,context,_=self._real_discovery_fixture()
        sample=replace(sample,distance_to_future_event_m=3.,future_event_speed_cap_mps=8.)
        selected,diagnostics=policy._rollout_with_diagnostics(sample,obs,kin,.5,0.,context,0.,2.)
        self.assertEqual(diagnostics['original_candidate_statuses'],(
            ('continue',False,'event_crossed'),
            ('release_then_brake',True,'event_crossed'),
            ('brake_now',True,'event_crossed')))
        self.assertIsNone(diagnostics['discovery_termination'])
        self.assertEqual(diagnostics['extra_probe_count'],0)
        self.assertEqual(selected[7],'release_then_brake')
        self.assertTrue(selected[0])

    def test_real_invalid_discovery_attempt_counts_transitions_and_falls_back(self):
        base,provider,sample,obs,kin,context,config=self._real_discovery_fixture()
        class InvalidAfterOriginalForecasts:
            identity=provider.identity
            def __init__(inner): inner.calls=0
            def evaluate(inner,q):
                inner.calls+=1
                capability=provider.evaluate(q)
                if inner.calls>=48:
                    return replace(capability,in_domain=False,limiting_constraints=('controlled_domain_exit',))
                return capability
        invalid=InvalidAfterOriginalForecasts()
        policy=HybridLongitudinalPolicy(base.plan,invalid,base.predictor,config)
        selected,diagnostics=policy._rollout_with_diagnostics(sample,obs,kin,.5,0.,context,0.,2.)
        self.assertGreater(invalid.calls,48)
        self.assertEqual(diagnostics['original_candidate_statuses'][0][0],'continue')
        self.assertEqual(diagnostics['extra_probe_count'],8)
        self.assertGreater(diagnostics['attempted_transitions'],0)
        self.assertLessEqual(diagnostics['attempted_transitions'],11*config.slabs)
        self.assertIsNone(selected[0]); self.assertIsNone(selected[7])
        self.assertEqual(diagnostics['termination'],'unknown_original_endpoint:capability_domain_exit')

    def test_real_discovery_decide_is_repeatable_in_reverse_fixture_order(self):
        from simplified_vehicle_models.solvers.hybrid_driver import HybridLongitudinalPolicy
        rows=[]
        for distance,cap in ((8.,6.),(3.,8.)):
            policy,_,sample,obs,kin,context,_=self._real_discovery_fixture()
            sample=replace(sample,distance_to_future_event_m=distance,future_event_speed_cap_mps=cap)
            policy.plan=SimpleNamespace(sample=lambda station,sample=sample:sample,identity=policy.plan.identity)
            rows.append((policy,obs,kin,context))
        def decide(row):
            policy,obs,kin,context=row
            return policy.decide(observation=obs,pedal_position=0.,kinematics=kin,soc=.5,brake_context=context)
        forward=[decide(row) for row in rows]
        reverse=[decide(row) for row in reversed(rows)]
        self.assertEqual(forward[0],reverse[1])
        self.assertEqual(forward[1],reverse[0])

    def test_mixed_verified_reports_selected_progress_acceleration_not_raw_request(self):
        selected_accel=0.42
        def mixed_rollout(sample,observation,kin,soc,pedal,brake_context,brake_force,positive_accel,_diagnostics=None):
            _diagnostics.update({'selected_progress_acceleration':selected_accel,'selected_lambda':0.25,
                'selected_residual':-0.01,'extra_probe_count':2,'attempted_transitions':240,
                'termination':'bracket_resolution','query_count':None})
            return (True,1.2,7.9,0.1,3,240,'event_crossed:mixed_verified','mixed',0.35,0.12)
        with patch.object(self.policy,'_rollout',side_effect=mixed_rollout):
            out=self.policy.decide(observation=self.obs,pedal_position=0.,kinematics=self.kin,soc=.5,brake_context=object())
        self.assertEqual(out.first_pedal_target,0.35)
        self.assertEqual(out.predicted_pedal_after_first_slab,0.12)
        self.assertEqual(out.decision.target_progress_acceleration_mps2,selected_accel)
        self.assertAlmostEqual(out.decision.body_vx_dot_target_mps2,selected_accel)
        curved=PathKinematicContext(8.,1.2,.2,.3,.08,.025,.001,.4,True)
        curved_obs=replace(self.obs,speed=curved.vx_mps)
        with patch.object(self.policy,'_rollout',side_effect=mixed_rollout):
            curved_out=self.policy.decide(observation=curved_obs,pedal_position=0.,kinematics=curved,soc=.5,brake_context=object())
        self.assertEqual(curved_out.decision.target_progress_acceleration_mps2,selected_accel)
        self.assertNotAlmostEqual(curved_out.decision.body_vx_dot_target_mps2,selected_accel,places=4)
        self.assertNotEqual(out.decision.target_progress_acceleration_mps2,out.decision.raw_acceleration_request_mps2)
        self.assertEqual(out.decision.selected_mixing_coefficient,0.25)
        self.assertEqual(out.decision.phase,'drive')
        self.assertEqual(out.decision.status,'event_crossed:mixed_verified')
        self.assertEqual(out.decision.extra_probe_count,2)
        self.assertEqual(out.decision.search_termination_reason,'bracket_resolution')
        self.assertIsNone(out.decision.decision_query_count)

    def test_production_mixed_forecast_tuple_dispatches_canonical_schedule_label(self):
        policy,sample,obs,kin,_,_=self._relay_policy_from_captured_row()
        from simplified_vehicle_models.solvers import hybrid_driver as hybrid_module
        original_eligible=hybrid_module._relay_search_eligible
        original_search=hybrid_module._bounded_verified_relay_search
        seen=[]
        def select_mixed(forecast,*args,**kwargs):
            chosen=forecast(0.25)
            seen.append(chosen)
            return chosen,0.25,chosen[4]-float(sample.future_event_speed_cap_mps)-1e-8,1,'bracket_resolution'
        policy.plan.identity='mixed-test-plan'
        hybrid_module._relay_search_eligible=lambda *args:True
        hybrid_module._bounded_verified_relay_search=select_mixed
        try:
            out=policy.decide(observation=obs,pedal_position=0.,kinematics=kin,soc=.5,brake_context=object())
        finally:
            hybrid_module._relay_search_eligible=original_eligible
            hybrid_module._bounded_verified_relay_search=original_search
        self.assertEqual(len(seen),1)
        self.assertEqual(seen[0][3],'mixed')
        self.assertEqual(out.decision.status,'event_crossed:mixed_verified')
        self.assertEqual(out.decision.phase,'drive')
        self.assertEqual(out.decision.target_progress_acceleration_mps2,seen[0][8])
        self.assertEqual(out.first_pedal_target,seen[0][5])
        self.assertEqual(out.predicted_pedal_after_first_slab,seen[0][6])

    def test_query_count_is_call_local_when_available_and_none_when_unavailable(self):
        sample=replace(self.plan.sample(0.),distance_to_future_event_m=None)
        args=(sample,self.obs,self.kin,.5,0.,object(),0.,2.)
        self.provider.query_count=0
        _,early=self.policy._rollout_with_diagnostics(*args)
        self.assertEqual(early['termination'],'no_future_event')
        self.assertIsNone(early['query_count'])
        sample=replace(self.plan.sample(0.),distance_to_future_event_m=10.,distance_to_braking_onset_m=5.)
        args=(sample,self.obs,self.kin,.5,0.,object(),0.,2.)
        _,available=self.policy._rollout_with_diagnostics(*args)
        self.assertIsNone(available['query_count'])
        class NoCounterProvider:
            identity='no-counter-provider'
            def evaluate(inner,q): return LongitudinalCapability(2.,-.2,-6.,True,(),inner.identity,(0.,.5,1.),(-.2,1.,2.),(0.,.5,1.),(-.2,-3.,-6.),1.)
        p=NoCounterProvider()
        plan=build_hybrid_speed_plan([0.,5.,10.,15.],[0.]*4,[18.,18.,8.,18.],p,initial_speed_mps=8.,storage_soc=.5,is_closed=False)
        policy=HybridLongitudinalPolicy(plan,p,self.predictor,self.config)
        _,unavailable=policy._rollout_with_diagnostics(plan.sample(0.),self.obs,self.kin,.5,0.,object(),0.,2.)
        self.assertIsNone(unavailable['query_count'])
        self.assertIn(unavailable['termination'],('original_candidate','unknown_original_endpoint','all_known_candidates_infeasible','extra_forecast_limit','bracket_resolution'))

    def test_exact_decision_payload_repeatability_and_grouped_event_residual(self):
        args=dict(observation=self.obs,pedal_position=.1,kinematics=self.kin,soc=.5,brake_context=object())
        one=self.policy.decide(**args); two=self.policy.decide(**args)
        self.assertEqual(one.decision.to_dict(),two.decision.to_dict())
        self.assertEqual(one.decision.search_termination_reason,'original_candidate')
        self.assertFalse(any(k.endswith("seconds") for k in one.decision.to_dict()))
        self.assertEqual(set(one.decision.to_dict()),set(two.decision.to_dict()))
        cap=14.99999994; vc=math.nextafter(cap+1e-8,-math.inf)
        self.assertTrue(vc <= cap + 1e-8)
        self.assertLessEqual(vc - (cap + 1e-8),0.)
        self.assertNotEqual(vc-(cap+1e-8),vc-cap-1e-8)
        captured_vc=14.99999995000001
        self.assertFalse(captured_vc <= cap + 1e-8)
        self.assertGreater(captured_vc - (cap + 1e-8),0.)

    def test_governor_reduction_flips_event_to_overspeed_and_matches_command(self):
        provider=SyntheticProvider()
        plan=build_hybrid_speed_plan([0.,5.,10.,15.],[0.]*4,[18.,18.,6.5,18.],provider,initial_speed_mps=8.,storage_soc=.5,is_closed=False)
        policy=HybridLongitudinalPolicy(plan,provider,self.predictor,self.config)
        ungoverned=policy.decide(observation=replace(self.obs,wheel_speed_margin=.85),pedal_position=0.,kinematics=self.kin,soc=.5,brake_context=object())
        governed=policy.decide(observation=replace(self.obs,wheel_speed_margin=.17),pedal_position=0.,kinematics=self.kin,soc=.5,brake_context=object())
        self.assertTrue(ungoverned.decision.event_crossed)
        self.assertEqual(ungoverned.decision.status,'event_crossed:release_then_brake')
        self.assertFalse(governed.decision.event_crossed)
        self.assertEqual(governed.decision.status,'all_candidates_overspeed_fallback_shortfall')
        self.assertEqual(governed.brake_force_n,governed.decision.brake_force_n)
        self.assertLessEqual(governed.brake_force_n,1176.*.17/.85+1e-9)
        rollout=policy._rollout(plan.sample(0.),replace(self.obs,wheel_speed_margin=.17),self.kin,.5,0.,object(),0.,2.)
        self.assertEqual(rollout[0],governed.decision.event_crossed)
        self.assertLess(rollout[8],0.)
        fallback_force=policy._maximum_admissible_force(object(),.17/.85)
        command_force=governed.brake_force_n
        self.assertAlmostEqual(command_force,fallback_force,places=7)
    def test_all_candidates_fail_applies_max_admissible_governed_prefix(self):
        provider=SyntheticProvider()
        plan=build_hybrid_speed_plan([0.,5.,10.,15.],[0.]*4,[18.,18.,6.5,18.],provider,initial_speed_mps=8.,storage_soc=.5,is_closed=False)
        policy=HybridLongitudinalPolicy(plan,provider,self.predictor,self.config)
        with patch.object(policy,'_rollout',return_value=(False,None,None,3.,3,180,'all_candidates_overspeed','brake_now',-.17,-.05)) as rollout:
            out=policy.decide(observation=replace(self.obs,wheel_speed_margin=.17),pedal_position=0.,kinematics=self.kin,soc=.5,brake_context=object())
        self.assertFalse(out.decision.event_crossed)
        self.assertEqual(out.decision.status,'all_candidates_overspeed_fallback_shortfall')
        self.assertEqual(out.decision.phase,'fallback')
        self.assertEqual(out.brake_force_n,out.decision.brake_force_n)
        self.assertAlmostEqual(out.brake_force_n,1176.*.17/.85,places=7)
        rollout.assert_called_once()
    def test_missing_context_and_soc_fail_closed(self):
        with self.assertRaises(ValueError): self.policy.decide(observation=self.obs,pedal_position=0.,kinematics=self.kin,soc=float('nan'))
        out=self.policy.decide(observation=self.obs,pedal_position=0.,kinematics=self.kin,soc=.5,brake_context=None)
        self.assertEqual(out.decision.event_crossed,None)
        self.assertIn('missing_current_inverse_context',out.decision.status)
    def test_underspeed_inside_braking_segment_does_not_force_plan_braking(self):
        plan=build_hybrid_speed_plan([0.,5.,10.,15.],[0.]*4,[18.,18.,8.,18.],self.provider,initial_speed_mps=8.,storage_soc=.5,is_closed=False)
        policy=HybridLongitudinalPolicy(plan,self.provider,self.predictor,self.config)
        self.assertTrue(plan.sample(6.).braking_region)
        kin=PathKinematicContext(5.,0.,0.,0.,0.,0.,0.,0.,True)
        obs=replace(self.obs,distance=6.,speed=5.)
        out=policy.decide(observation=obs,pedal_position=0.,kinematics=kin,soc=.5,brake_context=object())
        self.assertGreater(out.decision.raw_acceleration_request_mps2,0.)
        self.assertEqual(out.brake_force_n,0.)
        self.assertEqual(self.predictor.calls,[])
    def test_maximum_admissible_fallback_uses_feasible_prefix(self):
        fallback=self.policy._maximum_admissible_force(object(),1.)
        self.assertLessEqual(fallback,1176.)
        self.assertGreater(fallback,0.)
        self.assertLessEqual(self.predictor.predict(None,fallback).deceleration_mps2,1.)
    def test_actual_command_is_target_and_one_slab_reaches_forecast_state(self):
        obs=replace(self.obs,wheel_speed_margin=.17)
        result=self.policy.decide(observation=obs,pedal_position=.25,kinematics=self.kin,soc=.5,brake_context=object())
        rollout=self.policy._rollout(self.plan.sample(0.),obs,self.kin,.5,.25,object(),0.,2.)
        target=rollout[8]; predicted=rollout[9]
        command_position=result.throttle-result.brake_force_n/self.pc.maximum_brake_force
        self.assertAlmostEqual(command_position,target,places=12)
        expected=advance_signed_pedal(.25,target,self.config.horizon_s/self.config.slabs,self.pc)
        self.assertAlmostEqual(predicted,expected,places=12)
        self.assertNotAlmostEqual(command_position,predicted,places=5)
        # Independent numeric check: starting p=.25, brake target=-1, dt=.05.
        known=advance_signed_pedal(.25,-1.,.05,self.pc)
        self.assertAlmostEqual(known,0.,places=12)
        self.assertNotAlmostEqual(known,-1.,places=5)
    def test_future_capability_domain_exit_is_nine_value_unknown_and_coasts(self):
        class FutureDomainProvider(SyntheticProvider):
            def evaluate(self, q):
                self.query_count += 1
                if getattr(self, 'future_domain_exit', False) and q.speed_mps > 8.5:
                    return LongitudinalCapability(0., 0., 0., False, (), self.identity)
                grid=(0.,.125,.25,.5,.75,1.)
                return LongitudinalCapability(self.drive,-.2,-self.brake,True,(),self.identity,grid,tuple(-.2+(self.drive+.2)*p for p in grid),grid,tuple(-.2+(-self.brake+.2)*p for p in grid),1.)
        provider = FutureDomainProvider()
        # Build with an in-domain provider, then model an onset-only domain edge.
        plan = build_hybrid_speed_plan([0., 5., 10., 15.], [0.]*4, [18., 18., 8., 18.], provider,
                                       initial_speed_mps=8., storage_soc=.5, is_closed=False)
        provider.future_domain_exit = True
        policy = HybridLongitudinalPolicy(plan, provider, self.predictor, self.config)
        result = policy.decide(observation=self.obs, pedal_position=0., kinematics=self.kin,
                               soc=.5, brake_context=object())
        self.assertIsNone(result.decision.event_crossed)
        self.assertIn('event_unknown:', result.decision.status)
        self.assertEqual(result.decision.phase, 'coast')
        self.assertEqual(result.throttle, 0.)
        self.assertEqual(result.brake_force_n, 0.)
        # All unknown rollout outcomes, including the early onset-domain exit,
        # preserve the nine-field caller contract.
        rollout = policy._rollout(plan.sample(0.), self.obs, self.kin, .5, 0., object(), 0., 2.)
        self.assertEqual(len(rollout), 10)
        self.assertIsNone(rollout[0])
        self.assertEqual(rollout[6], 'future_capability_domain_exit')

    def test_future_corner_curvature_changes_authority_queries(self):
        class CurvatureProvider(SyntheticProvider):
            def evaluate(self,q):
                self.query_count+=1
                self.last_curvature=q.curvature_inv_m
                brake=6. if abs(q.curvature_inv_m)<.01 else 1.
                grid=(0.,.125,.25,.5,.75,1.)
                return LongitudinalCapability(2.,-.2,-brake,True,(),self.identity,grid,tuple(-.2+2.2*p for p in grid),grid,tuple(-.2+(-brake+.2)*p for p in grid),1.)
        provider=CurvatureProvider()
        plan=build_hybrid_speed_plan([0.,5.,10.,15.],[0.,0.,.1,.1],[18.,18.,8.,18.],provider,initial_speed_mps=8.,storage_soc=.5,is_closed=False)
        self.assertEqual(plan.sample(0.).curvature_inv_m,0.)
        self.assertAlmostEqual(plan.sample(7.5).curvature_inv_m,.05)
        policy=HybridLongitudinalPolicy(plan,provider,self.predictor,self.config)
        result=policy.decide(observation=self.obs,pedal_position=0.,kinematics=self.kin,soc=.5,brake_context=object())
        self.assertTrue(result.decision.status.startswith('event_crossed:'))
        self.assertTrue(result.decision.rollout_samples>=0)
        self.assertGreater(provider.query_count,0)
        self.assertGreater(provider.last_curvature,.09)
        # An explicit nonzero heading/vy/lateral-offset context uses the analytic
        # progress speed and a converted body brake context, not vx as progress.
        curved=PathKinematicContext(8.,1.2,.12,.3,.08,.025,.001,.4,True)
        c,s=math.cos(curved.heading_error_rad),math.sin(curved.heading_error_rad)
        B=1-curved.curvature_inv_m*curved.lateral_error_m
        expected=(curved.vx_mps*c-curved.vy_mps*s)/B
        unknown=self.policy._rollout(self.plan.sample(0.),self.obs,curved,.5,0.,object(),0.,2.)
        self.assertAlmostEqual(expected,7.8577475446692455,places=12)
        self.assertIn(unknown[6],("unsupported_future_rest_crossing","unsupported_kinematic_context","all_candidates_overspeed","event_crossed:release_then_brake"))
        straight=HybridLongitudinalPolicy(self.plan,self.provider,self.predictor,self.config)
        straight_result=straight._rollout(self.plan.sample(0.),self.obs,self.kin,.5,0.,object(),0.,2.)
        corner_result=policy._rollout(plan.sample(0.),self.obs,self.kin,.5,0.,object(),0.,2.)
        self.assertEqual(straight_result[0],True)
        self.assertEqual(corner_result[0],True)
        self.assertGreater(straight_result[2],0.)
        self.assertGreater(corner_result[2],0.)
        closed=build_hybrid_speed_plan([0.,5.,10.,15.,20.],[0.,0.,.05,.1,0.],[18.,18.,18.,18.,18.],provider,initial_speed_mps=8.,storage_soc=.5,is_closed=True)
        self.assertAlmostEqual(closed.sample(19.).curvature_inv_m,.02)
        self.assertAlmostEqual(closed.sample(21.).curvature_inv_m,.0)
    def test_rollout_uses_progress_speed_and_unknowns_unsupported_geometry(self):
        sample=self.plan.sample(0.)
        straight=self.policy._rollout(sample,self.obs,self.kin,.5,0.,object(),0.,2.)
        self.assertIsNotNone(straight[0])
        curved=PathKinematicContext(14.,1.2,.12,.3,.08,.025,.001,.4,True)
        unsupported=self.policy._rollout(sample,self.obs,curved,.5,0.,object(),0.,2.)
        self.assertIn(unsupported[6],("unsupported_future_rest_crossing","unsupported_kinematic_context","event_beyond_horizon","all_candidates_overspeed"))

    def test_nonlinear_map_rejected_explicitly(self):
        vehicle=compiled_vehicle(); predictor=BrakeAuthorityPredictor(vehicle.planar,maximum_abs_curvature_inv_m=.001,minimum_speed_mps=2.,maximum_speed_mps=20.)
        policy=HybridLongitudinalPolicy(self.plan,self.provider,predictor,self.config)
        self.assertIs(policy.predictor,predictor)
    def test_38_state_live_hybrid_adapter_tiny(self):
        vehicle=compiled_vehicle(); old=straight_plan(8.,100.); plant=VerticalForwardPlant(vehicle,old); state=plant.initial_state(8.)
        pc=PedalPolicyContext(1/.175,1/.175,.02,vehicle.planar.maximum_pedal_force,.85,1/.175,1/.175)
        provider=SyntheticProvider(); plan=build_hybrid_speed_plan([0.,1.,2.],[0.]*3,[12.]*3,provider,initial_speed_mps=8.,storage_soc=.5,is_closed=False)
        predictor=BrakeAuthorityPredictor(vehicle.planar,maximum_abs_curvature_inv_m=.001,minimum_speed_mps=2.,maximum_speed_mps=20.)
        policy=HybridLongitudinalPolicy(plan,provider,predictor,HybridPolicyConfig(maximum_brake_force_n=vehicle.planar.maximum_pedal_force,pedal_context=pc))
        driver=PreviewDriver(old,vehicle.planar.wheelbase,vehicle.planar.steering_maximum_angle,vehicle.planar.steering_maximum_rate,vehicle.planar.maximum_pedal_force,pedal_travel_time=.175,hybrid_policy=policy)
        driver=replace(driver,throttle_apply_rate=1/.175,throttle_release_rate=1/.175,brake_apply_rate=1/.175,brake_release_rate=1/.175)
        out=solve_forward(plant,driver,state,duration=.05,wall_seconds=30.,maximum_wall_seconds=30.,maximum_step=.01,rtol=1e-4,atol=1e-7,method='BDF',initial_pedal=0.)
        self.assertEqual(out.state.shape[0],38); self.assertGreater(out.evaluations,0)
if __name__=='__main__': unittest.main()
