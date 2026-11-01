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
    def test_flat_corner_cap_coasts_and_context_validates(self):
        env=envelope([10,10,10],[10,10,10])
        d=EnvelopePolicy(env).decide(obs(v=10),0.)
        self.assertEqual((d.requested_throttle,d.governed_brake_force),(0.,0.))
        with self.assertRaises((ValueError,TypeError)): PedalPolicyContext(0,1,1,1,1)
    def test_solver_accepted_step_carries_cached_policy_diagnostics(self):
        env=envelope([30,30,30],[30,30,30])
        plan=type('Plan',(),{})
        from simplified_vehicle_models.solvers.forward_driver import SpeedPlan
        speedplan=SpeedPlan(np.array([0.,20.]),np.array([10.,10.]),np.zeros(2),False)
        driver=PreviewDriver(speedplan,1.5,.5,2.,1000.,envelope_policy=EnvelopePolicy(env))
        class Plant:
            state_size=1
            def observe(self,state): return obs(float(state[0]),10.)
            def evaluate(self,time,state,command): return RateSystem(np.array([1.]))
        samples=[]
        solve_forward(Plant(),driver,np.array([0.]),duration=.02,maximum_step=.01,step_sink=samples.append)
        self.assertTrue(samples)
        self.assertIsNotNone(samples[0].policy_diagnostics)
        self.assertEqual(samples[0].policy_diagnostics.actual_speed,10.)

if __name__=='__main__': unittest.main()
