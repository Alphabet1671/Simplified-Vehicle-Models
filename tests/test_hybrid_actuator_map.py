import unittest
from dataclasses import replace

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.longitudinal_capability import (
    CapabilityQuery, CurrentVehicleCapability, LongitudinalCapability,
    evaluate_pedal_response, pedal_for_acceleration,
)



class HybridActuatorMapTests(unittest.TestCase):
    def test_causal_prefix_point_two_response_inverse_and_saturation(self):
        # Endpoint is -1.2 while physical force saturates at tau=.2. This is the
        # exact q=.2 counterexample: p=-.1 achieves half of coast-to-endpoint.
        grid=(0.,.2,.5,1.)
        response=(0.,-1.2,-1.2,-1.2)
        cap=LongitudinalCapability(2.,0.,-1.2,True,(),"q=.2",(0.,1.),(0.,2.),grid,response,.2)
        self.assertAlmostEqual(evaluate_pedal_response(cap,-.1),-.6,places=12)
        self.assertAlmostEqual(evaluate_pedal_response(cap,-.5),-1.2,places=12)
        self.assertAlmostEqual(evaluate_pedal_response(cap,-1.),-1.2,places=12)
        target=-.6
        pedal=pedal_for_acceleration(cap,target)
        self.assertAlmostEqual(pedal,-.1,places=12)
        self.assertAlmostEqual(evaluate_pedal_response(cap,pedal),target,places=12)

    def test_drive_interpolates_physical_partial_throttle_and_is_immutable_payload(self):
        grid=(0.,.125,.25,.5,.75,1.)
        vals=(-.3,-.05,.2,.7,1.2,1.7)
        cap=LongitudinalCapability(1.7,-.3,-3.,True,(),"drive-grid",grid,vals,(0.,1.),(-.3,-3.),1.)
        self.assertAlmostEqual(evaluate_pedal_response(cap,.125),-.05)
        self.assertAlmostEqual(evaluate_pedal_response(cap,.375),.45)
        altered=replace(cap,drive_acceleration_grid=vals[:-1]+(1.8,))
        self.assertNotEqual(cap.drive_acceleration_grid,altered.drive_acceleration_grid)

    def test_basic_provider_missing_map_fails_closed_and_domain(self):
        legacy=LongitudinalCapability(2.,-.2,-4.,True,(),"legacy")
        with self.assertRaisesRegex(ValueError,"unsupported_missing_physical_pedal_map"):
            evaluate_pedal_response(legacy,-.1)
        outside=LongitudinalCapability(0.,0.,0.,False,("outside_grid_domain",),"legacy")
        with self.assertRaisesRegex(ValueError,"outside capability domain"):
            evaluate_pedal_response(outside,.1)
        with self.assertRaises(ValueError):
            evaluate_pedal_response(legacy,1.01)

    def test_bad_active_map_rejected(self):
        good=LongitudinalCapability(1.,0.,-2.,True,(),"bad",(0.,.5,1.),(0.,.5,1.),(0.,.5,1.),(0.,-1.,-2.),.5)
        malformed=(
            replace(good,brake_pedal_grid=(0.,.5,.5),brake_acceleration_grid=(0.,-1.,-2.)),
            replace(good,brake_pedal_grid=(0.,float('nan'),1.)),
            replace(good,brake_acceleration_grid=(0.,float('nan'),-2.)),
            replace(good,brake_pedal_grid=(0.,.5)),
            replace(good,brake_acceleration_grid=(0.,-1.,-1.)),
        )
        for cap in malformed:
            with self.subTest(cap=cap):
                with self.assertRaisesRegex(ValueError,"invalid_active_pedal_map"):
                    evaluate_pedal_response(cap,-.4)
                with self.assertRaisesRegex(ValueError,"invalid_active_pedal_map"):
                    pedal_for_acceleration(cap,-.5)


class RealProviderPedalOracleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle=compile_formulation("models/formulations/iem24_murphy_real_planar_7dof.xml").model

    def provider(self, capacity=300.):
        return CurrentVehicleCapability(self.vehicle,wheel_radii_m=(.29,)*4,
            speed_grid_mps=(5.,15.,25.),curvature_grid_inv_m=(0.,.03,.08),
            soc_grid=(.2,.5,.8),source_label="real-formulation",calibration_label="real-cal-v1",
            longitudinal_capacity_n=(capacity,)*4)

    def test_real_provider_prefix_tau_and_same_map_at_node_and_offgrid(self):
        provider=self.provider()
        node=provider.evaluate(CapabilityQuery(15.,0.,.5))
        self.assertTrue(node.in_domain)
        self.assertAlmostEqual(node.brake_pedal_threshold,.2004142758,places=7)
        # Independently derive the full-pedal predictor's returned physical-force
        # threshold from the same three load-transfer/contact updates.
        from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar,normal_loads,tire_force_capacities,_allocate_axle_force
        from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityContext,BrakeAuthorityPredictor
        view=point_mass_view_of_planar(self.vehicle);radii=np.asarray((.29,)*4);speed=15.;acc=0.
        predictor=BrakeAuthorityPredictor(self.vehicle,maximum_abs_curvature_inv_m=.08,minimum_speed_mps=5.,maximum_speed_mps=25.)
        for _ in range(3):
            loads=normal_loads(view,speed,acc,0.);lc,latc=tire_force_capacities(view,loads);lat=np.zeros(4)
            _allocate_axle_force(lat,latc,slice(0,2),0.);_allocate_axle_force(lat,latc,slice(2,4),0.)
            rem=np.minimum(lc*np.sqrt(np.maximum(0.,1.-np.divide(lat,latc,out=np.zeros(4),where=latc>0)**2)),300.)
            ctx=BrakeAuthorityContext(speed,0.,tuple(radii),tuple(loads),tuple(rem),soc=.5)
            full=predictor.predict(ctx,self.vehicle.maximum_pedal_force);acc=-full.deceleration_mps2
        self.assertAlmostEqual(node.brake_pedal_threshold,full.pedal_force_n/self.vehicle.maximum_pedal_force,places=12)
        # Real predictor response at physical pedal .1 is about half of the
        # coast-to-tire-capped endpoint for this real q≈.2 common-prefix node.
        p10=evaluate_pedal_response(node,-.1)
        self.assertAlmostEqual(p10,node.coast_net_progress_acceleration_mps2+
            .1/node.brake_pedal_threshold*(node.brake_net_progress_acceleration_mps2-node.coast_net_progress_acceleration_mps2),places=10)
        self.assertAlmostEqual(evaluate_pedal_response(node,-.5),node.brake_net_progress_acceleration_mps2,places=10)
        for query in (CapabilityQuery(15.,0.,.5),CapabilityQuery(12.5,.015,.35)):
            cap=provider.evaluate(query)
            for pedal in (-.1,-.15,-.19):
                target=evaluate_pedal_response(cap,pedal)
                inverse=pedal_for_acceleration(cap,target)
                self.assertAlmostEqual(inverse,pedal,places=10)
                self.assertAlmostEqual(evaluate_pedal_response(cap,inverse),target,places=10)
        # Off-grid query is the weighted physical response of each corner curve,
        # evaluated at one common physical pedal position (not moving knots).
        query=CapabilityQuery(12.5,.015,.35);off=provider.evaluate(query);pedal=-.35
        supports=[]
        for axis,value in zip((provider.speed_grid,provider.curvature_grid,provider.soc_grid),(query.speed_mps,query.curvature_inv_m,query.soc)):
            lo=max(0,min(int(np.searchsorted(axis,value,side='right')-1),len(axis)-2));supports.append((lo,(value-axis[lo])/(axis[lo+1]-axis[lo])))
        expected=0.
        for mask in range(8):
            ix=tuple(supports[d][0]+((mask>>d)&1) for d in range(3));weight=np.prod([supports[d][1] if (mask>>d)&1 else 1.-supports[d][1] for d in range(3)])
            grid=provider._brake_grids[ix[0],ix[1],ix[2],:];values=provider._brake_maps[ix[0],ix[1],ix[2],:]
            while len(grid)>2 and grid[-1]==grid[-2] and values[-1]==values[-2]: grid=grid[:-1];values=values[:-1]
            expected+=weight*np.interp(abs(pedal),grid,values)
        self.assertAlmostEqual(evaluate_pedal_response(off,pedal),expected,places=10)

    def test_real_partial_throttle_samples_match_independent_powertrain_oracle(self):
        from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar,normal_loads,tire_force_capacities,_allocate_axle_force
        from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityContext,BrakeAuthorityPredictor
        from simplified_vehicle_models.solvers.forward_driver import DriverCommand
        from simplified_vehicle_models.solvers.forward_powertrain import evaluate_forward_powertrain
        from simplified_vehicle_models.solvers.vertical_forward import _drivetrain_context
        radii=np.asarray((.29,)*4);speed=15.;soc=.5;capacity=300.;k=0.
        provider=self.provider(capacity);cap=provider.evaluate(CapabilityQuery(speed,k,soc))
        view=point_mass_view_of_planar(self.vehicle);driven,gearing,eff=_drivetrain_context(self.vehicle)
        predictor=BrakeAuthorityPredictor(self.vehicle,maximum_abs_curvature_inv_m=.08,minimum_speed_mps=5.,maximum_speed_mps=25.)
        acc=0.;oracle=None
        for _ in range(3):
            loads=normal_loads(view,speed,acc,speed*speed*k);lc,latc=tire_force_capacities(view,loads);lat=np.zeros(4)
            _allocate_axle_force(lat,latc,slice(0,2),0.);_allocate_axle_force(lat,latc,slice(2,4),0.)
            rem=np.minimum(lc*np.sqrt(np.maximum(0.,1.-np.divide(lat,latc,out=np.zeros(4),where=latc>0)**2)),capacity)
            ctx=BrakeAuthorityContext(speed,k,tuple(radii),tuple(loads),tuple(rem),soc=soc)
            resistance=predictor.predict(ctx,0.).deceleration_mps2*(self.vehicle.total_mass+self.vehicle.wheel_inertia*np.sum(1./radii**2))
            oracle=[]
            for throttle in (0.,.125,.25,.5,.75,1.):
                pt=evaluate_forward_powertrain(self.vehicle,driven,gearing,eff,speed/radii,soc,DriverCommand(throttle,0.,0.))
                force=np.asarray(pt.motor_wheel_torques)/radii
                prefix=min(1.,float(np.min(np.divide(rem,force,out=np.full(4,np.inf),where=force>0))))
                oracle.append((float(np.sum(force))*prefix-resistance)/(self.vehicle.total_mass+self.vehicle.wheel_inertia*np.sum(1./radii**2)))
            acc=oracle[-1]
        np.testing.assert_allclose(cap.drive_acceleration_grid,oracle,rtol=0.,atol=1e-10)
        self.assertAlmostEqual(evaluate_pedal_response(cap,.375),np.interp(.375,(0.,.125,.25,.5,.75,1.),oracle),places=10)

    def test_real_provider_is_consumed_by_production_active_forecast(self):
        from tests.test_hybrid_driver import HybridDecisionTests
        from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityPredictor
        from simplified_vehicle_models.solvers.hybrid_driver import HybridLongitudinalPolicy
        from simplified_vehicle_models.solvers.hybrid_speed_plan import build_hybrid_speed_plan
        fixture=HybridDecisionTests();fixture.setUp();provider=self.provider()
        plan=build_hybrid_speed_plan([0.,5.,10.,15.],[0.]*4,[18.,18.,8.,18.],provider,initial_speed_mps=8.,storage_soc=.5,is_closed=False)
        predictor=BrakeAuthorityPredictor(self.vehicle,maximum_abs_curvature_inv_m=.08,minimum_speed_mps=5.,maximum_speed_mps=25.)
        policy=HybridLongitudinalPolicy(plan,provider,predictor,fixture.config)
        class CountedProvider:
            identity=provider.identity
            def __init__(self,delegate): self.delegate=delegate;self.queries=[]
            def evaluate(self,query): self.queries.append(query);return self.delegate.evaluate(query)
        counted=CountedProvider(provider)
        policy=HybridLongitudinalPolicy(plan,counted,predictor,fixture.config)
        from unittest.mock import patch
        import simplified_vehicle_models.solvers.hybrid_driver as hd
        forward_pedals=[];orig_eval=hd.evaluate_pedal_response
        def tracked_eval(cap,p): forward_pedals.append(float(p));return orig_eval(cap,p)
        with patch.object(hd,"evaluate_pedal_response",side_effect=tracked_eval) as forward_map, patch.object(hd,"pedal_for_acceleration",wraps=hd.pedal_for_acceleration) as inverse_map, patch("simplified_vehicle_models.solvers.brake_authority.BrakeAuthorityPredictor.predict",side_effect=AssertionError("rollout invoked brake predictor")), patch("simplified_vehicle_models.solvers.forward_powertrain.evaluate_forward_powertrain",side_effect=AssertionError("rollout invoked powertrain")):
            result=policy.decide(observation=fixture.obs,pedal_position=0.,kinematics=fixture.kin,soc=.5,brake_context=object())
        self.assertIsNone(result.decision.event_crossed)
        self.assertIn("event_unknown:",result.decision.status)
        self.assertEqual(result.decision.event_speed_cap_mps,8.)
        self.assertEqual(result.decision.candidate_count,0)
        self.assertEqual(result.decision.rollout_samples,0)
        # Missing supported brake context fails closed before candidate rollout;
        # it must not fabricate a selected pedal or predictor invocation.
        self.assertEqual(forward_map.call_count,0)
        self.assertEqual(inverse_map.call_count,0)
        self.assertEqual(len(counted.queries),1)
        self.assertEqual(forward_pedals,[])

    def test_provider_lookup_matches_scalar_reference_and_precomputes_brake_metadata(self):
        from unittest.mock import patch
        import simplified_vehicle_models.solvers.longitudinal_capability as lc
        original=lc._stored_brake_grid
        with patch.object(lc,"_stored_brake_grid",wraps=original) as stored:
            p=self.provider()
            setup_calls=stored.call_count
            self.assertGreater(setup_calls,0)
            for q in (CapabilityQuery(15.,0.,.5),CapabilityQuery(12.5,.015,.35),
                      CapabilityQuery(8.7,.041,.27),CapabilityQuery(21.3,.067,.73)):
                result=p.evaluate(q)
                axes=(p.speed_grid,p.curvature_grid,p.soc_grid)
                lo=[];frac=[]
                for axis,x in zip(axes,(q.speed_mps,q.curvature_inv_m,q.soc)):
                    h=min(max(int(np.searchsorted(axis,x,side="right")-1),0),len(axis)-2)
                    lo.append(h);frac.append((x-axis[h])/(axis[h+1]-axis[h]))
                supports=[]; expected=np.zeros(3);tau=0.
                for mask in range(8):
                    ix=tuple(lo[d]+((mask>>d)&1) for d in range(3))
                    w=float(np.prod([frac[d] if (mask>>d)&1 else 1.-frac[d] for d in range(3)]))
                    if w>1e-15:
                        supports.append((ix,w));expected+=w*p._table[ix];tau+=w*p._brake_tau[ix]
                bg=tuple(sorted(set(k for ix,w in supports for k in original(p._brake_grids[ix],p._brake_maps[ix])[0])))
                bv=[]
                for pedal in bg:
                    bv.append(sum(w*float(np.interp(pedal,*original(p._brake_grids[ix],p._brake_maps[ix]))) for ix,w in supports))
                self.assertEqual(result.brake_pedal_grid,bg)
                np.testing.assert_allclose(result.brake_acceleration_grid,bv,rtol=0.,atol=1e-12)
                np.testing.assert_allclose((result.drive_net_progress_acceleration_mps2,result.coast_net_progress_acceleration_mps2,result.brake_net_progress_acceleration_mps2),expected,rtol=0.,atol=1e-12)
                self.assertAlmostEqual(result.brake_pedal_threshold,tau,delta=1e-12)
            self.assertEqual(stored.call_count,setup_calls)
        self.assertIsInstance(p._brake_curves,tuple)
        self.assertTrue(all(isinstance(item,tuple) and all(isinstance(x,tuple) for x in item) for item in p._brake_curves))

    def test_real_provider_storage_cannot_be_reenabled_or_mutated(self):
        p=self.provider()
        for name in ("_table","_valid","_drive_maps","_brake_maps","_brake_grids","_brake_tau"):
            arr=getattr(p,name)
            with self.subTest(name=name):
                with self.assertRaises(ValueError): arr.setflags(write=True)
        self.assertIsInstance(p._reasons,tuple)
        with self.assertRaises(TypeError): p._reasons[0][0][0]=("mutated",)


if __name__ == "__main__":
    unittest.main()
