"""Portable short full 38-state closed-loop braking maneuver regression."""
import unittest
import numpy as np

from simplified_vehicle_models.solvers.braking_envelope import BrakingEnvelope, propagate_braking_caps
from simplified_vehicle_models.solvers.envelope_driver import (
    EnvelopeDriverSettings, EnvelopePolicy, PedalPolicyContext, PhysicalBrakePlanningContext)
from simplified_vehicle_models.solvers.forward_driver import PreviewDriver, SpeedPlan, solve_forward
from tests.test_vertical_forward import compiled_vehicle
from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant

class BrakingManeuverEvidenceTests(unittest.TestCase):
    def test_full_38_state_closed_loop_applies_physical_braking_on_straight(self):
        vehicle=compiled_vehicle(); planar=vehicle.planar
        distance=np.array([0.,10.,20.,40.,80.])
        caps=np.array([25.,25.,10.,10.,10.])
        entry, binding, _=propagate_braking_caps(distance,caps,np.full(4,2.0),is_closed=False)
        env=BrakingEnvelope(distance,np.zeros(5),caps,entry,np.full(4,2.),binding,False,25.,{})
        plan=SpeedPlan(distance,np.array([20.,20.,10.,10.,10.]),np.zeros(5),False)
        policy=EnvelopePolicy(env,EnvelopeDriverSettings(mode="physical-reachable-v1"))
        rates=(1/.175,1/.175,1/.175,1/.175)
        factory_calls=[]
        pedal=PedalPolicyContext(rates[1],rates[2],.02,planar.maximum_pedal_force,.85,rates[0],rates[3])
        # Physical context is built once before RHS integration from explicit
        # full-plant state loads and the accepted hydraulic authority predictor.
        from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityPredictor
        predictor=BrakeAuthorityPredictor(planar,maximum_abs_curvature_inv_m=.01,minimum_speed_mps=2.,maximum_speed_mps=40.)
        plant=VerticalForwardPlant(vehicle,plan)
        initial=plant.initial_state(20.)
        def factory(_plant,_obs,pc,state):
            factory_calls.append(np.array(state,copy=True))
            return _plant.brake_planning_context(state,pc,predictor=predictor,knot_count=4)
        driver=PreviewDriver(plan,planar.wheelbase,planar.steering_maximum_angle,
            planar.steering_maximum_rate,planar.maximum_pedal_force,
            pedal_travel_time=None,throttle_apply_rate=rates[0],throttle_release_rate=rates[1],
            brake_apply_rate=rates[2],brake_release_rate=rates[3],envelope_policy=policy,
            brake_planning_context_factory=factory)
        samples=[]
        result=solve_forward(plant,driver,initial,duration=2.0,wall_seconds=100.,maximum_wall_seconds=120.,
            maximum_step=.01,rtol=2e-6,atol=2e-8,method="BDF",initial_pedal=0.,step_sink=samples.append)
        self.assertTrue(samples)
        self.assertEqual(plant.state_size,38)
        applied_peak=max(sample.command.brake_force for sample in samples)
        self.assertGreater(applied_peak,0.)
        self.assertTrue(all(sample.policy_diagnostics.planner_mode=="physical-reachable-v1" for sample in samples))
        self.assertTrue(np.all(np.isfinite(result.state)))
        self.assertGreaterEqual(plant.minimum_contact_load,0.)
        self.assertLessEqual(applied_peak,planar.maximum_pedal_force)
        self.assertTrue(np.all(np.isfinite(result.state[16:20])))
        self.assertTrue(np.all(np.isfinite(result.state[26:38])))
        self.assertTrue(result.midpoint_sample_validation_complete)
        # A controller that never responded to the scheduled cap would continue
        # to accelerate under the high initial speed; require a real speed drop.
        self.assertLess(float(result.state[3,-1]),20.)
        self.assertGreaterEqual(float(result.state[0,-1]),0.)
        # Realized longitudinal deceleration and cap slack are taken from the
        # plant output, not requested commands or surrogate table values.
        speeds=np.asarray([sample.state[3] for sample in samples],dtype=float)
        stations=np.asarray([sample.state[0] for sample in samples],dtype=float)
        peak=float(np.max(speeds))
        self.assertLessEqual(peak,20.5)
        self.assertLessEqual(float(np.min(stations)),40.)
        self.assertTrue(all(sample.requested_command is not None and sample.command is not None for sample in samples))
        self.assertEqual(len(factory_calls),1)
        self.assertTrue(np.array_equal(factory_calls[0],initial))

if __name__=='__main__': unittest.main()
