import unittest
import numpy as np
from simplified_vehicle_models.solvers.braking_envelope import BrakingEnvelope
from simplified_vehicle_models.solvers.envelope_driver import EnvelopeDriverSettings, EnvelopePolicy, PedalPolicyContext
from simplified_vehicle_models.solvers.forward_driver import DriverObservation, PreviewDriver, RateSystem, solve_forward


def envelope(caps, entry, decel=None, maximum=60.):
    x=np.array([0.,10.,20.]); c=np.array(caps,dtype=float); e=np.array(entry,dtype=float)
    return BrakingEnvelope(x,np.zeros(3),c,e,np.array(decel or [5.,5.]),np.array([2,2,2]),False,maximum,{})
def obs(s=0.,v=0.,margin=float('inf')): return DriverObservation(s,0.,0.,v,0.,0.,wheel_speed_margin=margin)

class EnvelopePolicyTests(unittest.TestCase):
    def test_descending_q_uses_half_slope_and_is_deterministic(self):
        env=envelope([30,30,10],[np.sqrt(300),np.sqrt(200),10])
        pol=EnvelopePolicy(env,EnvelopeDriverSettings(response_allowance_s=0.,disabled_transition_time_s=.01,positive_acceleration_reserve_mps2=0.))
        d=pol.decide(obs(0,10.),0.)
        self.assertAlmostEqual(d.acceleration_request,.5*d.squared_speed_slope+2.*(d.reference_speed-10.))
        self.assertLess(d.squared_speed_slope,0.)
        self.assertEqual(d,pol.decide(obs(0,10.),0.))
    def test_full_throttle_below_flat_cap_and_none_at_cap(self):
        env=envelope([20,20,20],[20,20,20],maximum=20.)
        self.assertEqual(EnvelopePolicy(env).decide(obs(v=0),0.).requested_throttle,1.)
        self.assertEqual(EnvelopePolicy(env).decide(obs(v=20),0.).requested_throttle,0.)
    def test_legacy_feedforward_absent_and_repeat_reorder(self):
        env=envelope([30,30,30],[30,30,30]); pol=EnvelopePolicy(env)
        d=pol.decide(obs(v=5),-.5)
        self.assertEqual(d.governed_brake_force,0.)
        self.assertEqual(d,pol.decide(obs(v=5),-.5))
        self.assertEqual(d.to_dict(),pol.decide(obs(v=5),-.5).to_dict())
    def test_actual_pedal_reserve_governor_shortfall(self):
        env=envelope([30,30,10],[np.sqrt(300),np.sqrt(200),10])
        ctx=PedalPolicyContext(4.,6.,.02,1000.,.85)
        d=EnvelopePolicy(env).decide(obs(v=10,margin=.425),1.,ctx)
        self.assertAlmostEqual(d.transition_time_s,1/4+1/6+.06+.06)
        self.assertGreater(d.reserve_distance_m,5.)
        self.assertEqual(d.governor_factor,.5)
        self.assertEqual(d.governor_shortfall, d.ungoverned_brake_force>d.governed_brake_force+1e-12)
    def test_signed_observations_and_closed_station_wrap(self):
        x=np.array([0.,10.,20.]); caps=np.array([30.,30.,30.])
        env=BrakingEnvelope(x,np.zeros(3),caps,caps,np.array([5.,5.]),np.array([0,1,2]),True,60.,{})
        policy=EnvelopePolicy(env)
        base=DriverObservation(2.,0.,0.,8.,0.,0.,0.)
        for sign in (-1.,1.):
            o=DriverObservation(2.,sign*.03,sign*.04,8.,sign*.2,sign*.06,sign*.1)
            self.assertEqual(policy.decide(o,0.),policy.decide(o,0.))
        o=DriverObservation(-18.,-.03,-.04,8.,-.2,-.06,-.1)
        self.assertEqual(policy.decide(o,0.),policy.decide(o,0.))
        self.assertAlmostEqual(policy.decide(o,0.).preview_entry_speed,policy.decide(DriverObservation(2.,-.03,-.04,8.,-.2,-.06,-.1),0.).preview_entry_speed)
        with self.assertRaises((TypeError,ValueError)):
            policy.decide(DriverObservation(0.,float('nan'),0.,8.,0.,0.),0.)

    def test_negative_actual_speed_rejected_and_derived_overflow_is_clear(self):
        env=envelope([30,30,30],[30,30,30]); policy=EnvelopePolicy(env)
        with self.assertRaisesRegex(ValueError,'speed'):
            policy.decide(obs(v=-.1),0.)
        huge=EnvelopeDriverSettings(disabled_transition_time_s=1e308,positive_acceleration_reserve_mps2=1e308)
        with self.assertRaisesRegex(ValueError,'reserve|transition'):
            EnvelopePolicy(env,huge).decide(obs(v=1e308),0.)
    def test_local_global_binding_ties_and_preview_past_corner(self):
        class Sample:
            def __init__(self,q,slope=7.,cap=10.,station=0.):
                self.squared_speed=q; self.squared_speed_slope=slope
                self.corner_speed=cap; self.entry_speed=np.sqrt(max(q,0.))
                self.deceleration=3.; self.binding_index=1; self.binding_station=10.
                self.binding_distance=10.-station; self.speed_domain_max=cap
        class Fake:
            is_closed=False; maximum_speed=20.; distance=np.array([0.,10.,20.])
            def __init__(self,now,future): self.now=now; self.future=future
            def sample(self,s): return self.now if s==0. else self.future
        settings=EnvelopeDriverSettings(disabled_transition_time_s=.2,positive_acceleration_reserve_mps2=0.)
        now=Sample(100.,cap=10.)
        d=EnvelopePolicy(Fake(now,Sample(400.,slope=-8.,cap=20.)),settings).decide(obs(v=60.),0.)
        self.assertEqual(d.squared_speed,100.)
        self.assertEqual(d.squared_speed_slope,0.)
        tie=EnvelopePolicy(Fake(Sample(400.,cap=20.),Sample(400.,slope=-9.,cap=20.,station=12.)),settings).decide(obs(v=60.),0.)
        self.assertEqual(tie.squared_speed_slope,-9.)
        self.assertGreater(tie.preview_station,10.)
        # A global cap stricter than both local and preview q is independently binding.
        class LowGlobal(Fake): maximum_speed=5.
        gd=EnvelopePolicy(LowGlobal(Sample(400.,cap=20.),Sample(400.,slope=-9.,cap=20.)),settings).decide(obs(v=5.),0.)
        self.assertEqual(gd.squared_speed,25.)
        self.assertEqual(gd.squared_speed_slope,0.)
        self.assertEqual(gd.requested_throttle,0.)
    def test_descending_preview_slope_survives_exact_local_and_global_cap_ties(self):
        class Sample:
            def __init__(self,q,slope,cap):
                self.squared_speed=q; self.squared_speed_slope=slope
                self.corner_speed=cap; self.entry_speed=np.sqrt(q)
                self.deceleration=3.; self.binding_index=1; self.binding_station=10.
                self.binding_distance=10.; self.speed_domain_max=cap
        class Fake:
            is_closed=False; maximum_speed=20.; distance=np.array([0.,10.,20.])
            def __init__(self,now,future): self.now=now; self.future=future
            def sample(self,s): return self.now if s==0. else self.future
        settings=EnvelopeDriverSettings(disabled_transition_time_s=.2,positive_acceleration_reserve_mps2=0.)
        descending=Sample(100.,-8.,10.)
        # The local cap equals qpreview exactly; at the boundary retain braking feedforward.
        local=EnvelopePolicy(Fake(Sample(400.,0.,20.),descending),settings).decide(obs(v=10.),0.)
        self.assertEqual(local.squared_speed,100.)
        self.assertEqual(local.squared_speed_slope,-8.)
        self.assertEqual(local.acceleration_request,-4.)
        # Global cap is independently tied to qpreview while local cap is nonbinding.
        global_tie=EnvelopePolicy(type('GlobalFake',(Fake,),{'maximum_speed':10.})(
            Sample(400.,0.,20.),Sample(100.,-6.,20.)),settings).decide(obs(v=10.),0.)
        self.assertEqual(global_tie.squared_speed,100.)
        self.assertEqual(global_tie.squared_speed_slope,-6.)
        self.assertEqual(global_tie.acceleration_request,-3.)

    def test_nonbinding_rising_preview_keeps_full_slope(self):
        class Sample:
            def __init__(self,q,slope,cap):
                self.squared_speed=q; self.squared_speed_slope=slope
                self.corner_speed=cap; self.entry_speed=np.sqrt(q)
                self.deceleration=3.; self.binding_index=1; self.binding_station=10.
                self.binding_distance=10.; self.speed_domain_max=cap
        class Fake:
            is_closed=False; maximum_speed=20.; distance=np.array([0.,10.,20.])
            def __init__(self): self.now=Sample(400.,0.,20.); self.future=Sample(100.,5.,20.)
            def sample(self,s): return self.now if s==0. else self.future
        settings=EnvelopeDriverSettings(disabled_transition_time_s=.2,positive_acceleration_reserve_mps2=0.)
        d=EnvelopePolicy(Fake(),settings).decide(obs(v=10.),0.)
        self.assertEqual(d.squared_speed_slope,5.)
        self.assertEqual(d.acceleration_request,2.5)

    def test_rising_preview_slope_at_cap_tie_stays_zero(self):
        class Sample:
            def __init__(self,q,slope,cap):
                self.squared_speed=q; self.squared_speed_slope=slope
                self.corner_speed=cap; self.entry_speed=np.sqrt(q)
                self.deceleration=3.; self.binding_index=1; self.binding_station=10.
                self.binding_distance=10.; self.speed_domain_max=cap
        class Fake:
            is_closed=False; maximum_speed=10.; distance=np.array([0.,10.,20.])
            def sample(self,s): return Sample(400.,0.,20.) if s==0. else Sample(100.,5.,10.)
        settings=EnvelopeDriverSettings(disabled_transition_time_s=.2,positive_acceleration_reserve_mps2=0.)
        d=EnvelopePolicy(Fake(),settings).decide(obs(v=10.),0.)
        self.assertEqual(d.squared_speed_slope,0.)
        self.assertEqual(d.acceleration_request,0.)

    def test_driver_context_uses_explicit_rates_and_disabled_governor(self):
        from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
        plan=SpeedPlan(np.array([0.,20.]),np.array([10.,10.]),np.zeros(2),False)
        env=envelope([30,30,30],[30,30,30]); pol=EnvelopePolicy(env)
        driver=PreviewDriver(plan,1.5,.5,2.,1000.,envelope_policy=pol,
                             throttle_apply_rate=4.,throttle_release_rate=6.,
                             brake_apply_rate=5.,brake_release_rate=6.)
        _,d=driver.command_with_context(obs(v=10),.5)
        self.assertAlmostEqual(d.transition_time_s,.5/6.+1/5.+.06+.06)
        direct=PreviewDriver(plan,1.5,.5,2.,1000.,envelope_policy=pol,brake_governor_margin=.85)
        _,d=direct.command_with_context(obs(v=10,margin=.425),None)
        self.assertEqual(d.governor_factor,.5)
        self.assertAlmostEqual(d.ungoverned_brake_force,0.)
    def test_flat_corner_cap_coasts_and_context_validates(self):
        env=envelope([10,10,10],[10,10,10]); d=EnvelopePolicy(env).decide(obs(v=10),0.)
        self.assertEqual((d.requested_throttle,d.governed_brake_force),(0.,0.))
        with self.assertRaises((ValueError,TypeError)): PedalPolicyContext(0,1,1,1,1)
    def test_default_command_parity_and_context_fallback(self):
        from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
        plan=SpeedPlan(np.array([0.,20.]),np.array([10.,8.]),np.array([0.,.1]),False)
        base=PreviewDriver(plan,1.5,.5,2.,1000.)
        o=DriverObservation(3.,.02,.01,9.,.1,.03,.2)
        legacy=base.command(o)
        # Preserve baseline steering and longitudinal calculations independently.
        target,curv,ff=plan.sample(o.distance)
        pc=plan.sample(o.distance+max(o.speed,0.)*base.preview_seconds)[1]
        course=o.heading_error+np.arctan2(o.lateral_speed,max(o.speed,.25))
        desired=np.clip(np.arctan(base.wheelbase*pc)-base.heading_gain*course-np.arctan2(base.lateral_gain*o.lateral_error,max(o.speed,1.))-.1*(o.yaw_rate-o.speed*curv),-base.maximum_steering,base.maximum_steering)
        steer=float(np.clip((desired-o.steering)/base.steering_time_constant,-base.maximum_steering_rate,base.maximum_steering_rate))
        accel=ff+base.speed_gain*(target-o.speed)
        governor=np.clip(o.wheel_speed_margin/base.brake_governor_margin,0.,1.)
        expected=(np.clip(accel/6.,0.,1.),np.clip(-accel/6.,0.,1.)*governor*base.maximum_brake_force,steer)
        np.testing.assert_array_equal((legacy.throttle,legacy.brake_force,legacy.steering_rate),expected)
        self.assertEqual(base.command_with_context(o),(legacy,None))
    def test_solver_sink_adds_no_physics_or_policy_samples(self):
        class CountingEnvelope:
            is_closed=False; maximum_speed=60.; distance=np.array([0.,10.,20.])
            def __init__(self):
                self.calls=0
                self.base=envelope([30,30,30],[30,30,30])
            def sample(self,s):
                self.calls+=1
                return self.base.sample(s)
        from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
        plan=SpeedPlan(np.array([0.,20.]),np.array([10.,10.]),np.zeros(2),False)
        class Plant:
            state_size=1
            def __init__(self): self.observes=0; self.evaluates=0
            def observe(self,state): self.observes+=1; return obs(float(state[0]),10.)
            def evaluate(self,time,state,command): self.evaluates+=1; return RateSystem(np.array([1.]))
        def run(with_sink):
            env=CountingEnvelope(); policy=EnvelopePolicy(env)
            driver=PreviewDriver(plan,1.5,.5,2.,1000.,envelope_policy=policy)
            plant=Plant(); samples=[]
            solve_forward(plant,driver,np.array([0.]),duration=.02,maximum_step=.01,
                          step_sink=samples.append if with_sink else None)
            return (plant.observes,plant.evaluates,env.calls,len(samples))
        no_sink=run(False); with_sink=run(True)
        self.assertEqual(no_sink[:3],with_sink[:3])
        self.assertGreater(with_sink[3],0)
        self.assertGreater(no_sink[2],0.)
        self.assertLessEqual(with_sink[2],no_sink[2])
    def test_signed_synthetic_solver_observation_keeps_steering_finite(self):
        from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
        plan=SpeedPlan(np.array([0.,20.]),np.array([10.,10.]),np.array([0.,.1]),False)
        env=envelope([30,30,30],[30,30,30])
        driver=PreviewDriver(plan,1.5,.5,2.,1000.,envelope_policy=EnvelopePolicy(env))
        o=DriverObservation(2.,-.03,-.04,8.,-.2,-.06,-.1)
        command,decision=driver.command_with_context(o,0.)
        self.assertTrue(np.isfinite(command.steering_rate))
        self.assertEqual(decision,driver.envelope_policy.decide(o,0.))

    def test_solver_accepted_step_carries_cached_policy_diagnostics(self):
        env=envelope([30,30,30],[30,30,30])
        from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
        plan=SpeedPlan(np.array([0.,20.]),np.array([10.,10.]),np.zeros(2),False)
        driver=PreviewDriver(plan,1.5,.5,2.,1000.,envelope_policy=EnvelopePolicy(env))
        class Plant:
            state_size=1
            def observe(self,state): return obs(float(state[0]),10.)
            def evaluate(self,time,state,command): return RateSystem(np.array([1.]))
        samples=[]
        solve_forward(Plant(),driver,np.array([0.]),duration=.02,maximum_step=.01,step_sink=samples.append)
        self.assertTrue(samples); self.assertIsNotNone(samples[0].policy_diagnostics)
        self.assertEqual(samples[0].policy_diagnostics.actual_speed,10.)

if __name__=='__main__': unittest.main()
