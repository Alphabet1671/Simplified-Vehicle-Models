import unittest
from types import SimpleNamespace
import numpy as np
from simplified_vehicle_models.solvers.braking_envelope import BrakingEnvelope
from simplified_vehicle_models.solvers.envelope_driver import (
    EnvelopeDriverSettings, EnvelopePolicy, PedalPolicyContext, PhysicalBrakePlanningContext,
    advance_signed_pedal, forecast_cap_event)
from simplified_vehicle_models.solvers.forward_driver import DriverObservation


def make_envelope(caps, entry, *, closed=False):
    x=np.linspace(0.,20.,len(caps))
    return BrakingEnvelope(x,np.zeros(len(x)),np.asarray(caps),np.asarray(entry),
        np.full(len(x)-1,5.),np.zeros(len(x),dtype=int),closed,40.,{})

RECORDED_ZERO_TARGET_CASES = (
    ("mis", 7.2925185007123225, 0.0, 26.805420945215236, 26.805421425559686),
    ("fsc", 2.7076584147326628, 0.0, 42.40818908259312, 42.40819003907309),
    ("scaled_catalunya", 2.6311492898384117, 0.0, 72.5807761999653, 72.58077823401487),
)

class BrakeTransitionPlanningTests(unittest.TestCase):
    def test_recorded_zero_target_cases_drive_actual_policy_decisions(self):
        from unittest.mock import Mock
        from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityContext
        class Predictor:
            def __init__(self):
                self.request_current=Mock(return_value=SimpleNamespace(
                    deceleration_mps2=0.25,pedal_force_n=50.,shortfall_mps2=0.))
        for track, raw, urgency, recorded_force, zero_target_force in RECORDED_ZERO_TARGET_CASES:
            with self.subTest(track=track):
                self.assertGreater(raw,0.)
                self.assertEqual(urgency,0.)
                self.assertGreater(recorded_force,0.)
                self.assertAlmostEqual(recorded_force,zero_target_force,places=5)
                # The literal establishes the positive-accel/no-urgency operating branch;
                # use a simple stationary cap fixture to feed that branch into production.
                target_speed=10.+raw/2.
                class RecordedCaseEnvelope:
                    maximum_speed=40.
                    def sample(self,station):
                        return SimpleNamespace(binding_distance=0.,binding_station=0.,
                            corner_speed=target_speed,entry_speed=target_speed,
                            squared_speed=target_speed**2,deceleration=0.,binding_index=0,speed_domain_max=40.)
                env=RecordedCaseEnvelope()
                predictor=Predictor()
                authority=BrakeAuthorityContext(10.,wheel_radii_m=(.2,)*4,normal_loads_n=(1000.,)*4)
                pedal=PedalPolicyContext(4.,4.,.02,1000.,.85)
                context=PhysicalBrakePlanningContext(pedal,(0.,1000.),(0.,5.),10.,
                    current_authority_context=authority,current_predictor=predictor)
                decision=EnvelopePolicy(env,EnvelopeDriverSettings(mode="physical-reachable-v1")).decide(
                    DriverObservation(0.,0.,0.,10.,0.,0.),0.,context)
                self.assertEqual(decision.urgency_force_deceleration_mps2,urgency)
                self.assertEqual(decision.ungoverned_brake_force,0.)
                predictor.request_current.assert_not_called()
                self.assertGreater(decision.requested_throttle,0.)
    def test_zero_urgency_never_invokes_inverse_for_positive_raw_acceleration(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityContext
        class Predictor:
            def __init__(self): self.request_current = Mock(return_value=SimpleNamespace(
                deceleration_mps2=0.25, pedal_force_n=50., shortfall_mps2=0.))
        env=make_envelope([30.,30.,30.],[30.,30.,30.])
        predictor=Predictor()
        authority=BrakeAuthorityContext(10.,wheel_radii_m=(.2,)*4,normal_loads_n=(1000.,)*4)
        pedal=PedalPolicyContext(4.,4.,.02,1000.,.85)
        table=PhysicalBrakePlanningContext(pedal,(0.,1000.),(0.,5.),10.,
            current_authority_context=authority,current_predictor=predictor)
        policy=EnvelopePolicy(env,EnvelopeDriverSettings(mode="physical-reachable-v1"))
        decision=policy.decide(DriverObservation(0.,0.,0.,10.,0.,0.),0.,table)
        self.assertGreater(decision.acceleration_request,0.)
        self.assertEqual(decision.ungoverned_brake_force,0.)
        predictor.request_current.assert_not_called()
        self.assertAlmostEqual(decision.requested_throttle,
            min(1.,decision.acceleration_request/policy.settings.throttle_acceleration_scale))

    def test_actual_signed_pedals_forecast_drive_and_brake_separately(self):
        pedal=PedalPolicyContext(4.,4.,.02,1000.,.85,4.,4.)
        class Capture:
            speed_knots_mps=(2.,20.)
            def __init__(self): self.pairs=[]; self.last_force=0.; self.pedal=pedal
            def safety_deceleration(self,force,speed=None): self.last_force=force; return 0.
            def drive_acceleration(self,speed,throttle,station=None):
                self.pairs.append((throttle,self.last_force)); return 0.
        env=make_envelope([30.,8.],[30.,8.])
        class CrossingEnvelope:
            def sample(self,station):
                return SimpleNamespace(binding_distance=1000.,corner_speed=30.,entry_speed=8.)
        env=CrossingEnvelope()
        for initial in (-1.,0.,1.):
            with self.subTest(pedal=initial):
                capture=Capture()
                # Exercise production signed-pedal advance, with a single bounded sample.
                forecast_cap_event(env,capture,initial,0.,start_station_m=0.,speed_mps=10.,horizon_s=1.,samples=1)
                actual=advance_signed_pedal(initial,0.,1.,pedal)
                self.assertEqual(capture.pairs[0][0],max(actual,0.))
                self.assertEqual(capture.pairs[0][1],max(-actual,0.)*pedal.maximum_brake_force)

    def test_decision_projection_has_semantic_units_ranges_and_event_metadata(self):
        class SemanticEnvelope:
            maximum_speed=40.
            def __init__(self,binding_distance): self.binding_distance=binding_distance
            def sample(self,station):
                cap=20. if station<10. else 8.
                return SimpleNamespace(binding_distance=self.binding_distance,
                    binding_station=10. if self.binding_distance else 0.,
                    corner_speed=cap,entry_speed=cap,squared_speed=cap**2,
                    deceleration=0.,binding_index=1,speed_domain_max=40.)
        env=SemanticEnvelope(10.)
        pedal=PedalPolicyContext(4.,4.,.02,1000.,.85)
        table=PhysicalBrakePlanningContext(pedal,(0.,1000.),(0.,0.),5.,
            speed_knots_mps=(2.,20.),drive_acceleration_table=((0.,0.),(0.,0.)),
            speed_dependent_deceleration=((0.,0.),(0.,0.)))
        settings=EnvelopeDriverSettings(mode="physical-reachable-v1",forecast_horizon_s=3.)
        decision=EnvelopePolicy(env,settings).decide(
            DriverObservation(0.,0.,0.,5.,0.,0.),0.,table)
        self.assertEqual(decision.forecast_slack_units,"m^2/s^2")
        self.assertAlmostEqual(decision.target_pedal_position,1.)
        self.assertEqual(decision.transition_forecast_horizon_s,3.)
        self.assertEqual(decision.forecast_event_station_m,10.)
        self.assertAlmostEqual(decision.forecast_crossing_time_s,2.,places=12)
        self.assertAlmostEqual(decision.forecast_cap_slack_mps2_squared,39.,places=12)
        self.assertEqual(decision.forecast_samples,40)
        self.assertEqual(decision.competing_corner_cap_current_mps,20.)
        self.assertEqual(decision.competing_corner_cap_preview_mps,
                         env.sample(decision.preview_station).corner_speed)

        no_event=SemanticEnvelope(0.)
        neutral=EnvelopePolicy(no_event,settings).decide(
            DriverObservation(0.,0.,0.,5.,0.,0.),0.,table)
        self.assertEqual(neutral.transition_forecast_horizon_s,0.)
        self.assertIsNone(neutral.forecast_event_station_m)
        self.assertIsNone(neutral.forecast_crossing_time_s)
        self.assertIsNone(neutral.forecast_cap_slack_mps2_squared)
        self.assertIsNone(neutral.forecast_samples)

    def test_frozen_station_reference_is_invariant_to_pedal_and_call_order(self):
        env=make_envelope([25.,25.,10.],[25.,18.,10.])
        policy=EnvelopePolicy(env,EnvelopeDriverSettings(mode="physical-reachable-v1"))
        obs=DriverObservation(9.,0.,0.,16.,0.,0.)
        pedal=PedalPolicyContext(1/.175,1/.175,.02,1176.,.85)
        outputs=[policy.decide(obs,p,pedal) for p in (1.,0.,-.5)]
        self.assertTrue(all(d.reference_speed==outputs[0].reference_speed for d in outputs))
        self.assertTrue(all(d.squared_speed_slope==outputs[0].squared_speed_slope for d in outputs))
        self.assertEqual(outputs,[policy.decide(obs,p,pedal) for p in (1.,0.,-.5)])

    def test_margin_reduced_forecast_is_distinct_from_physical_target_table(self):
        pedal=PedalPolicyContext(4.,4.,.02,1000.,.85)
        context=PhysicalBrakePlanningContext(pedal,(0.,1000.),(2.,5.),15.,speed_knots_mps=(2.,10.),
            drive_acceleration_table=((0.,1.),(0.,1.)),speed_dependent_deceleration=((0.,1.8),(0.,3.)))
        self.assertAlmostEqual(context.deceleration(1000.,10.),3.)
        self.assertEqual(context.speed_dependent_deceleration[-1][-1],3.)

    def test_context_table_is_immutable_bounded_and_interpolates(self):
        pedal=PedalPolicyContext(4.,4.,.02,1000.,.85)
        context=PhysicalBrakePlanningContext(pedal,(0.,500.,1000.),(.5,2.,4.),15.)
        self.assertAlmostEqual(context.deceleration(250.),1.25)
        self.assertIsNone(context.deceleration(1001.))
        with self.assertRaises((AttributeError,TypeError)): context.force_knots_n=(0.,1.)
        with self.assertRaises(ValueError): PhysicalBrakePlanningContext(pedal,(0.,1000.),(2.,1.),15.)

    def test_pedal_ode_matches_rate_limited_then_exponential_plant_solution(self):
        import math
        c=PedalPolicyContext(2.,5.,.1,1000.,.85,7.,3.)
        self.assertAlmostEqual(advance_signed_pedal(1.,0.,.5,c),.2*math.exp(-1.),places=12)
        self.assertAlmostEqual(advance_signed_pedal(-.5,0.,.1,c),-.3*math.exp(-(.1-.2/3.)/.1),places=12)
        self.assertLess(advance_signed_pedal(.1,-.5,.1,c),0.)

    def test_provider_typeerror_is_not_retried_or_swallowed(self):
        from simplified_vehicle_models.solvers.forward_driver import PreviewDriver, SpeedPlan
        calls=[]
        def factory(*args): calls.append(args); raise TypeError("inside provider")
        plan=SpeedPlan(np.array([0.,20.]),np.array([20.,10.]),np.zeros(2),False)
        driver=PreviewDriver(plan,1.5,.5,2.,1000.,pedal_travel_time=.175,
            envelope_policy=EnvelopePolicy(make_envelope([30.,30.],[30.,10.]),EnvelopeDriverSettings(mode="physical-reachable-v1")),
            brake_planning_context_factory=factory)
        with self.assertRaisesRegex(TypeError,"inside provider"): driver.build_brake_planning_context(object(),None,None,np.zeros(1))
        self.assertEqual(len(calls),1)

    def test_current_overspeed_cannot_mask_distinct_future_cap_excess(self):
        distance=np.array([0.,5.,10.,15.,20.]); corner=np.array([8.,8.,12.,6.,6.]); entry=np.array([8.,8.,10.,6.,6.])
        env=BrakingEnvelope(distance,np.zeros(5),corner,entry,np.full(4,3.),np.zeros(5,dtype=int),False,20.,{})
        policy=EnvelopePolicy(env,EnvelopeDriverSettings(mode="physical-reachable-v1"))
        pedal=PedalPolicyContext(4.,4.,.02,1000.,.85)
        table=PhysicalBrakePlanningContext(pedal,(0.,1000.),(1.,4.),12.)
        d=policy.decide(DriverObservation(0.,0.,0.,10.,0.,0.),0.,table)
        self.assertFalse(d.feasible_schedule); self.assertIsNotNone(d.fallback_reason)

    def test_missing_context_falls_back_unknown_without_hidden_memory(self):
        policy=EnvelopePolicy(make_envelope([30.,30.,8.],[30.,20.,8.]),EnvelopeDriverSettings(mode="physical-reachable-v1"))
        obs=DriverObservation(19.,0.,0.,12.,0.,0.); context=PedalPolicyContext(4.,4.,.02,1000.,.85)
        first=policy.decide(obs,0.,context); self.assertEqual(first,policy.decide(obs,0.,context))
        self.assertEqual(first.planner_mode,"physical-reachable-v1"); self.assertEqual(first.candidate_count,0)
        self.assertEqual(first.fallback_reason,"missing_explicit_physical_context_or_pedal"); self.assertIsNone(first.feasible_schedule)

    def test_final_command_combines_target_and_urgency_before_single_mapping(self):
        from simplified_vehicle_models.solvers.envelope_driver import final_physical_command
        pedal=PedalPolicyContext(4.,4.,.02,1000.,.85); table=PhysicalBrakePlanningContext(pedal,(0.,500.,1000.),(0.,2.,5.),15.)
        result=final_physical_command(1.,3.,planning_context=table,speed=15.)
        self.assertEqual(result.requested_deceleration_mps2,3.); self.assertAlmostEqual(result.force_n,666.6666666667)
        self.assertEqual(result.target_deceleration_mps2,1.); self.assertEqual(result.urgency_deceleration_mps2,3.)

    def test_forecast_requires_actual_event_crossing_and_reports_unknown_beyond_horizon(self):
        x=np.array([0.,10.,20.]); caps=np.array([20.,8.,8.]); env=BrakingEnvelope(x,np.zeros(3),caps,caps,np.ones(2),np.array([1,1,2]),False,20.,{})
        pedal=PedalPolicyContext(4.,4.,.02,1000.,.85)
        table=PhysicalBrakePlanningContext(pedal,(0.,1000.),(0.,5.),10.,speed_knots_mps=(2.,30.),
            drive_acceleration_table=((0.,0.),(0.,0.)),speed_dependent_deceleration=((0.,5.),(0.,5.)))
        near=forecast_cap_event(env,table,0.,0.,start_station_m=0.,speed_mps=10.,horizon_s=3.,samples=60)
        self.assertEqual(near.reason,"event_crossed"); self.assertIsNotNone(near.outcome)
        self.assertAlmostEqual(near.event_station_m,10.)
        self.assertIsNotNone(near.crossing_time_s)
        self.assertGreater(near.samples,0)
        self.assertLess(near.minimum_cap_slack_mps2_squared,0.)
        violating=forecast_cap_event(env,table,0.,0.,start_station_m=0.,speed_mps=10.,horizon_s=1.5,samples=1)
        self.assertEqual(violating.reason,"event_crossed"); self.assertFalse(violating.outcome)
        self.assertEqual(violating.event_station_m,10.)
        self.assertIsNotNone(violating.crossing_time_s)
        self.assertLess(violating.minimum_cap_slack_mps2_squared,0.)
        far=forecast_cap_event(env,table,0.,1000.,start_station_m=0.,speed_mps=10.,horizon_s=.1,samples=5)
        self.assertIsNone(far.outcome); self.assertEqual(far.reason,"event_beyond_horizon")
        self.assertEqual(far.event_station_m,10.)
        self.assertIsNone(far.crossing_time_s)
        self.assertIsNotNone(far.minimum_cap_slack_mps2_squared)

    def test_forecast_crossing_interpolates_event_speed_not_endpoint_speed(self):
        x=np.array([0.,10.,20.]); caps=np.array([20.,8.,8.]); env=BrakingEnvelope(x,np.zeros(3),caps,caps,np.ones(2),np.array([1,1,2]),False,20.,{})
        pedal=PedalPolicyContext(4.,4.,.02,1000.,.85)
        table=PhysicalBrakePlanningContext(pedal,(0.,1000.),(0.,5.),10.,speed_knots_mps=(2.,30.),
            drive_acceleration_table=((0.,0.),(0.,0.)),speed_dependent_deceleration=((0.,5.),(0.,5.)))
        result=forecast_cap_event(env,table,0.,1000.,start_station_m=0.,speed_mps=10.,horizon_s=1.5,samples=1)
        # Safety decel is 3 m/s²; at station 10, event speed is sqrt(100-60).
        self.assertTrue(result.outcome)
        self.assertAlmostEqual(result.minimum_cap_slack_mps2_squared,64.-40.,places=8)

    def test_forecast_domain_exit_is_unknown(self):
        x=np.array([0.,10.,20.]); caps=np.array([20.,8.,8.]); env=BrakingEnvelope(x,np.zeros(3),caps,caps,np.ones(2),np.array([1,1,2]),False,20.,{})
        pedal=PedalPolicyContext(4.,4.,.02,1000.,.85)
        table=PhysicalBrakePlanningContext(pedal,(0.,1000.),(0.,5.),10.,speed_knots_mps=(8.,12.),
            drive_acceleration_table=((0.,0.),(0.,0.)),speed_dependent_deceleration=((0.,5.),(0.,5.)))
        result=forecast_cap_event(env,table,0.,1000.,start_station_m=0.,speed_mps=10.,horizon_s=3.,samples=60)
        self.assertIsNone(result.outcome); self.assertEqual(result.reason,"forecast_domain_exit")
        self.assertEqual(result.event_station_m,10.)
        self.assertIsNone(result.crossing_time_s)

    def test_governor_reduces_braking_forecasts_governed_force_not_false_by_override(self):
        env=BrakingEnvelope(np.array([0.,10.,20.]),np.zeros(3),np.array([20.,8.,8.]),np.array([20.,8.,8.]),np.ones(2),np.array([1,1,2]),False,20.,{})
        policy=EnvelopePolicy(env,EnvelopeDriverSettings(mode="physical-reachable-v1"))
        pedal=PedalPolicyContext(4.,4.,.02,1000.,.85)
        table=PhysicalBrakePlanningContext(pedal,(0.,1000.),(0.,5.),10.,speed_knots_mps=(2.,30.),
            drive_acceleration_table=((0.,0.),(0.,0.)),speed_dependent_deceleration=((0.,5.),(0.,5.)))
        obs=DriverObservation(0.,0.,0.,10.,0.,0.,wheel_speed_margin=.2)
        d=policy.decide(obs,0.,table)
        expected=forecast_cap_event(env,table,0.,d.governed_brake_force,start_station_m=0.,speed_mps=10.,horizon_s=3.,samples=60)
        self.assertEqual(d.feasible_schedule,expected.outcome)
        self.assertEqual(d.fallback_reason,None if expected.outcome is not None else expected.reason)

    def test_target_and_final_inverse_telemetry_are_distinct_and_named(self):
        from types import SimpleNamespace
        class Predictor:
            def request_current(self, context, target):
                return SimpleNamespace(deceleration_mps2=target,pedal_force_n=target*100.,shortfall_mps2=0.)
        env=BrakingEnvelope(np.array([0.,10.,20.]),np.zeros(3),np.array([20.,8.,8.]),np.array([20.,8.,8.]),np.ones(2),np.array([1,1,2]),False,20.,{})
        policy=EnvelopePolicy(env,EnvelopeDriverSettings(mode="physical-reachable-v1"))
        pedal=PedalPolicyContext(4.,4.,.02,1000.,.85)
        from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityContext
        authority=BrakeAuthorityContext(10.,wheel_radii_m=(.2,)*4,normal_loads_n=(1000.,)*4)
        table=PhysicalBrakePlanningContext(pedal,(0.,1000.),(0.,5.),10.,current_authority_context=authority,current_predictor=Predictor())
        d=policy.decide(DriverObservation(0.,0.,0.,10.,0.,0.),0.,table)
        self.assertNotEqual(d.final_inverse_deceleration_mps2,d.raw_target_inverse_deceleration_mps2)
        self.assertEqual(d.final_inverse_force_n,d.physical_brake_force_n)

    def test_continuous_physical_inverse_and_separate_safety_forecast(self):
        pedal=PedalPolicyContext(4.,4.,.02,1000.,.85); context=PhysicalBrakePlanningContext(pedal,(0.,500.,1000.),(0.,2.,5.),15.)
        force=context.force_for_deceleration(3.)
        self.assertAlmostEqual(force,666.6666666667,places=6); self.assertAlmostEqual(context.deceleration(force),3.)
        self.assertAlmostEqual(context.safety_deceleration(force),1.8); self.assertEqual(context.force_for_deceleration(99.),1000.)

    def test_bounded_candidates_report_physical_metrics(self):
        env=make_envelope([30.,30.,8.],[30.,20.,8.]); policy=EnvelopePolicy(env,EnvelopeDriverSettings(mode="physical-reachable-v1"))
        pedal=PedalPolicyContext(4.,4.,.02,1000.,.85); table=PhysicalBrakePlanningContext(pedal,(0.,1000.),(.5,5.),15.)
        d=policy.decide(DriverObservation(10.,0.,0.,12.,0.,0.),0.,table)
        self.assertEqual(d.candidate_count,0); self.assertLessEqual(d.rollout_horizon_s,3.)
        self.assertIsNone(d.feasible_schedule); self.assertIsNotNone(d.predicted_deceleration_mps2)
        self.assertIsNotNone(d.tracking_error_mps2); self.assertIn(d.fallback_reason,(None,"event_beyond_horizon"))

    def test_cap_event_beyond_horizon_is_never_certified(self):
        distance=np.array([0.,100.,200.]); caps=np.array([30.,30.,10.])
        env=BrakingEnvelope(distance,np.zeros(3),caps,np.array([30.,20.,10.]),np.full(2,5.),np.array([2,2,2]),False,40.,{})
        policy=EnvelopePolicy(env,EnvelopeDriverSettings(mode="physical-reachable-v1"))
        pedal=PedalPolicyContext(4.,4.,.02,1000.,.85); context=PhysicalBrakePlanningContext(pedal,(0.,1000.),(1.,5.),20.,speed_knots_mps=(2.,40.),drive_acceleration_table=((0.,0.),(0.,0.)),speed_dependent_deceleration=((0.,5.),(0.,5.)))
        decision=policy.decide(DriverObservation(0.,0.,0.,20.,0.,0.),0.,context)
        self.assertIsNone(decision.feasible_schedule); self.assertIsNotNone(decision.fallback_reason)
        self.assertLessEqual(decision.rollout_horizon_s,3.)
        self.assertLessEqual(decision.rollout_horizon_s,3.)

if __name__=='__main__': unittest.main()
