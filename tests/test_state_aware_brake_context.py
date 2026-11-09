import unittest
from dataclasses import replace

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.brake_authority import (
    BrakeAuthorityContext, BrakeAuthorityPredictor,
)
from simplified_vehicle_models.solvers.envelope_driver import (
    EnvelopeDriverSettings, EnvelopePolicy, PedalPolicyContext, PhysicalBrakePlanningContext)
from simplified_vehicle_models.solvers.braking_envelope import BrakingEnvelope, propagate_braking_caps
from simplified_vehicle_models.solvers.forward_driver import DriverObservation, PreviewDriver, SpeedPlan
from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant


class StateAwareBrakeContextTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(
            "models/formulations/iem24_murphy_real_14dof_simple.xml").model.planar
        cls.predictor = BrakeAuthorityPredictor(cls.vehicle)

    def context(self, speed=20., loads=(3000.,) * 4):
        return BrakeAuthorityContext(speed, 0., (.29,) * 4, loads)

    def test_current_inverse_matches_validated_linear_inverse_at_multiple_speeds_loads(self):
        for speed, loads in ((20., (3000.,) * 4), (15., (3300., 3000., 2800., 2900.)),
                             (10., (3500., 3200., 2700., 2600.))):
            context = self.context(speed, loads)
            for target in (3., 6.):
                with self.subTest(speed=speed, loads=loads, target=target):
                    old = self.predictor.request(context, target)
                    new = self.predictor.request_current(context, target)
                    self.assertAlmostEqual(new.pedal_force_n, old.pedal_force_n, places=8)

    def test_pure_order_independent_and_changes_with_current_state(self):
        base = self.context(20.)
        other_v = replace(base, speed_mps=15.)
        # Coupled current load/radius state; this vehicle's rolling coefficients
        # are zero, so radius-dependent wheel inertia carries the load transfer.
        other_load = replace(base, normal_loads_n=(4000., 3000., 2500., 2500.),
                             wheel_radii_m=(.28, .285, .295, .30))
        requests = [self.predictor.request_current(c, 4.).pedal_force_n
                    for c in (base, other_v, other_load, base)]
        self.assertEqual(requests[0], requests[3])
        self.assertNotEqual(requests[0], requests[1])
        self.assertNotEqual(requests[0], requests[2])

    def test_positive_raw_with_positive_urgency_still_brakes_and_actual_3_6_targets_invert(self):
        # At positive speed-error feedback the unconstrained request stays positive;
        # the nearby binding cap independently requires deceleration.
        distance=np.array([0.,1.,2.,3.]); caps=np.array([20.,8.,8.,8.]); entry,binding,_=propagate_braking_caps(distance,caps,np.full(3,6.),is_closed=False)
        pedal=PedalPolicyContext(5.7,5.7,.02,self.vehicle.maximum_pedal_force,.85,5.7,5.7)
        from unittest.mock import Mock
        predictor=self.predictor
        predictor.request_current=Mock(wraps=predictor.request_current)
        context=self.context(20.)
        table=PhysicalBrakePlanningContext(pedal,(0.,self.vehicle.maximum_pedal_force),(0.,10.),20.,
            current_authority_context=context,current_predictor=predictor)
        from types import SimpleNamespace
        class UrgentEnvelope:
            maximum_speed=30.
            def sample(self,station):
                return SimpleNamespace(binding_distance=4.,binding_station=4.,
                    corner_speed=20.,entry_speed=8.,squared_speed=400.,deceleration=6.,binding_index=0,speed_domain_max=30.)
        env=UrgentEnvelope()
        policy=EnvelopePolicy(env,EnvelopeDriverSettings(mode="physical-reachable-v1"))
        decision=policy.decide(DriverObservation(0.,0.,0.,12.,0.,0.),0.,table)
        self.assertGreater(decision.acceleration_request,0.)
        self.assertGreater(decision.urgency_force_deceleration_mps2,0.)
        self.assertGreater(decision.ungoverned_brake_force,0.)
        for target in (3.,6.):
            with self.subTest(target=target):
                from types import SimpleNamespace
                class TargetEnvelope:
                    maximum_speed=30.
                    def sample(self,station):
                        return SimpleNamespace(binding_distance=0.,binding_station=0.,
                            corner_speed=20.,entry_speed=20.,squared_speed=400.,
                            deceleration=0.,binding_index=0,speed_domain_max=30.)
                # Speed is above the cap, so the policy's raw physical target is chosen;
                # no event urgency can obscure the exact inverse request.
                desired_speed=12.-target/2.
                env_target=TargetEnvelope()
                env_target.sample=lambda station: SimpleNamespace(binding_distance=0.,binding_station=0.,
                    corner_speed=desired_speed,entry_speed=desired_speed,squared_speed=desired_speed**2,
                    deceleration=0.,binding_index=0,speed_domain_max=30.)
                target_decision=EnvelopePolicy(env_target,EnvelopeDriverSettings(mode="physical-reachable-v1")).decide(
                    DriverObservation(0.,0.,0.,12.,0.,0.),0.,table)
                calls=[call.args[1] for call in predictor.request_current.call_args_list]
                self.assertIn(target, [round(value,6) for value in calls])
                self.assertGreater(target_decision.ungoverned_brake_force,0.)
                self.assertAlmostEqual(target_decision.final_inverse_deceleration_mps2,target,places=7)
                self.assertEqual(target_decision.final_inverse_force_n,target_decision.physical_brake_force_n)

    def test_physical_policy_uses_live_state_force_not_frozen_context_table(self):
        distance=np.array([0.,10.,20.,30.]); caps=np.array([12.,12.,12.,12.])
        entry,binding,_=propagate_braking_caps(distance,caps,np.full(3,2.),is_closed=False)
        env=BrakingEnvelope(distance,np.zeros(4),caps,entry,np.full(3,2.),binding,False,20.,{})
        plan=SpeedPlan(distance,np.full(4,20.),np.zeros(4),False)
        policy=EnvelopePolicy(env,EnvelopeDriverSettings(mode="physical-reachable-v1"))
        pedal=PedalPolicyContext(5.7,5.7,.02,self.vehicle.maximum_pedal_force,.85,5.7,5.7)
        predictor=self.predictor
        base=self.context(20.)
        table=PhysicalBrakePlanningContext(pedal,(0.,self.vehicle.maximum_pedal_force),(0.,10.),20.,
            predictor=predictor,current_authority_context=base)
        driver=PreviewDriver(plan,self.vehicle.wheelbase,self.vehicle.steering_maximum_angle,
            self.vehicle.steering_maximum_rate,self.vehicle.maximum_pedal_force,
            envelope_policy=policy,brake_planning_context=table,
            brake_current_context_factory=lambda _plant,_obs,_pedal,state: replace(base,speed_mps=float(state[0])),
            brake_current_predictor=predictor)
        obs=DriverObservation(0.,0.,0.,15.,0.,0.)
        _,decision=driver.command_with_context(obs,0.,plant_state=np.array([15.]),plant=object())
        live=replace(base,speed_mps=15.)
        expected=predictor.request_current(live,max(0.,-decision.acceleration_request)).pedal_force_n
        self.assertAlmostEqual(decision.physical_brake_force_n,expected,places=8)
        frozen=predictor.request_current(base,max(0.,-decision.acceleration_request)).pedal_force_n
        self.assertNotAlmostEqual(decision.physical_brake_force_n,frozen,places=5)

    def test_inverse_matches_context_projection_and_respects_capacity_prefix(self):
        context=replace(self.context(16.),v_y_mps=.8,yaw_rate_rps=.12,steering_rad=.25)
        for target in (2.,4.):
            result=self.predictor.request_current(context,target)
            self.assertAlmostEqual(result.deceleration_mps2,target,places=7)
        limited=replace(context,longitudinal_capacity_n=(80.,80.,90.,90.))
        capped=self.predictor.request_current(limited,4.)
        replay=self.predictor.predict(limited,capped.pedal_force_n,target_deceleration_mps2=4.)
        np.testing.assert_allclose(capped.wheel_torques_nm,replay.wheel_torques_nm,rtol=1e-12,atol=1e-12)
        self.assertEqual(capped.limiting_constraint,"tire_contact_capacity")
        self.assertLess(capped.deceleration_mps2,4.)
        self.assertAlmostEqual(capped.shortfall_mps2,4.-capped.deceleration_mps2,places=9)

    def test_inverse_rejects_current_speed_outside_domain_without_clamping(self):
        predictor=BrakeAuthorityPredictor(self.vehicle,minimum_speed_mps=2.,maximum_speed_mps=40.)
        low=replace(self.context(1.5),speed_mps=1.5)
        with self.assertRaisesRegex(ValueError,"current speed"):
            predictor.request_current(low,3.)

    def test_full_plant_provider_preserves_speed_below_predictor_domain(self):
        plan=SpeedPlan(np.array([0.,100.]),np.array([5.,5.]),np.zeros(2),False)
        model=compile_formulation("models/formulations/iem24_murphy_real_14dof_simple.xml").model
        plant=VerticalForwardPlant(model,plan)
        state=plant.initial_state(1.5)
        predictor=BrakeAuthorityPredictor(model.planar,minimum_speed_mps=2.,maximum_speed_mps=40.)
        context=plant.brake_authority_context(state,predictor=predictor)
        self.assertAlmostEqual(context.speed_mps,float(state[3]),places=12)
        with self.assertRaisesRegex(ValueError,"current speed"):
            predictor.request_current(context,3.)

    def test_fast_inverse_rejects_unsupported_hydraulic_map(self):
        predictor = BrakeAuthorityPredictor(self.vehicle)
        predictor._linear_fast_path = False
        with self.assertRaises(NotImplementedError):
            predictor.request_current(self.context(), 3.)


if __name__ == "__main__":
    unittest.main()
