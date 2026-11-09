import unittest
from dataclasses import dataclass, replace, fields
import math

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.longitudinal_capability import (
    CapabilityQuery, LongitudinalCapability, PathKinematicContext,
    CurrentVehicleCapability, body_vx_dot_to_path_progress,
    path_progress_to_body_vx_dot, validate_execution_allocator,
)


@dataclass(frozen=True)
class SyntheticTVProvider:
    identity: str = "synthetic-tv"
    def evaluate(self, query):
        reduction=abs(query.yaw_moment_request_nm)*.01
        return LongitudinalCapability(5.-reduction,-.2,-6.+reduction,True,
                                      ("synthetic_joint_reserve",),self.identity)


class CapabilityContractTests(unittest.TestCase):
    @staticmethod
    def old_evaluate(provider,query):
        """Independent frozen pre-pack evaluator; intentionally mirrors old source."""
        import numpy as np
        import math
        if not isinstance(query,CapabilityQuery): raise TypeError("query must be CapabilityQuery")
        if query.yaw_moment_request_nm!=0: raise ValueError("torque allocation provider has no matching execution path")
        axes=(provider.speed_grid,provider.curvature_grid,provider.soc_grid)
        vals=(query.speed_mps,query.curvature_inv_m,query.soc)
        if not all(a[0]<=x<=a[-1] for a,x in zip(axes,vals)):
            return LongitudinalCapability(0.,0.,0.,False,("outside_grid_domain",),provider.identity)
        lo=[];f=[]
        for a,x in zip(axes,vals):
            h=min(max(int(np.searchsorted(a,x,side="right")-1),0),len(a)-2)
            lo.append(h);f.append((x-a[h])/(a[h+1]-a[h]))
        out=np.zeros(3);tau=0.;supports=[]
        for mask in range(8):
            ix=tuple(lo[d]+((mask>>d)&1) for d in range(3))
            w=math.prod(f[d] if (mask>>d)&1 else 1-f[d] for d in range(3))
            if w>1e-15:
                supports.append((ix,w));out+=w*provider._table[ix];tau+=w*provider._brake_tau[ix]
        ok=all(bool(provider._valid[ix]) for ix,w in supports)
        reasons=tuple(dict.fromkeys(reason for ix,w in supports for reason in provider._reasons[ix[0]][ix[1]][ix[2]]))
        drive_grid=provider._drive_pedal_grid;drive_values_array=np.zeros(len(drive_grid))
        for ix,w in supports: drive_values_array+=w*provider._drive_maps[ix[0],ix[1],ix[2],:]
        drive_values=tuple(float(x) for x in drive_values_array)
        flat=lambda ix:(ix[0]*provider._brake_curve_shape[1]+ix[1])*provider._brake_curve_shape[2]+ix[2]
        brake_curves=[provider._brake_curves[flat(ix)] for ix,w in supports]
        brake_grid=tuple(sorted(set(knot for grid,values in brake_curves for knot in grid)))
        brake_values_array=np.zeros(len(brake_grid))
        for (grid,values),(ix,w) in zip(brake_curves,supports):
            brake_values_array+=w*np.interp(brake_grid,grid,values)
        brake_values=tuple(float(x) for x in brake_values_array)
        if not ok: reasons=tuple(dict.fromkeys(reasons+("unsupported_interpolation_node",)))
        return LongitudinalCapability(*map(float,out),ok,reasons,provider.identity,drive_grid,drive_values,brake_grid,brake_values,float(tau))

    @staticmethod
    def payload(capability):
        return tuple((field.name,getattr(capability,field.name)) for field in fields(capability))

    @classmethod
    def setUpClass(cls):
        cls.vehicle=compile_formulation("models/formulations/iem24_murphy_real_planar_7dof.xml").model

    def provider(self, **kw):
        args=dict(wheel_radii_m=(.29,)*4,speed_grid_mps=(5.,15.,25.),
                  curvature_grid_inv_m=(0.,.03,.08),soc_grid=(.2,.5,.8),
                  source_label="iem24_murphy_real_planar_7dof.xml",
                  calibration_label="compiled-IEM-calibration-v1")
        args.update(kw)
        return CurrentVehicleCapability(self.vehicle,**args)

    def test_query_validation_and_synthetic_tv_contract(self):
        for bad in (float("nan"),float("inf")):
            with self.assertRaises(ValueError): CapabilityQuery(1.,0.,.5,bad)
        for vals in ((float("nan"),0.,.5,0.),(1.,float("inf"),.5,0.),(1.,0.,-0.1,0.),(1.,0.,1.1,0.)):
            with self.assertRaises(ValueError): CapabilityQuery(*vals)
        with self.assertRaises(ValueError): CapabilityQuery(-1.,0.,.5)
        p=SyntheticTVProvider(); q=CapabilityQuery(15.,.02,.8,25.)
        a=p.evaluate(q); self.assertEqual(a,p.evaluate(q)); self.assertEqual(a.allocator_identity,p.identity)
        self.assertLess(a.drive_net_progress_acceleration_mps2,p.evaluate(CapabilityQuery(15.,.02,.8)).drive_net_progress_acceleration_mps2)
        for cap,exe,alloc in (("tv","wrong","tv"),("wrong","tv","tv"),("tv","tv","wrong")):
            class A: identity=alloc
            with self.assertRaisesRegex(ValueError,"no matching execution path"):
                validate_execution_allocator(cap,exe,25.,A())
        with self.assertRaisesRegex(ValueError,"no matching execution path"):
            validate_execution_allocator(p.identity,p.identity,25.)
        validate_execution_allocator(p.identity,p.identity,25.,p)

    def test_frenet_straight_identity_and_independent_curved_fd(self):
        straight=PathKinematicContext(15.,0.,0.,0.,0.,0.,0.,0.,True)
        self.assertEqual(path_progress_to_body_vx_dot(2.,straight),2.)
        # Independent time-difference check of progress velocity U/B. Curvature
        # varies with station and vy-dot is prescribed, rather than round-trip parity.
        ctx=PathKinematicContext(14.,1.2,.12,.3,.08,.025,.001,.4,True)
        a=1.7; vx_dot=path_progress_to_body_vx_dot(a,ctx)
        def rhs(y):
            vx,vy,e,n,s=y
            k=ctx.curvature_inv_m+ctx.curvature_gradient_inv_m2*s
            progress=(vx*np.cos(e)-vy*np.sin(e))/(1.-k*n)
            return np.array([vx_dot,ctx.vy_dot_estimate_mps2,ctx.yaw_rate_rps-k*progress,
                             vx*np.sin(e)+vy*np.cos(e),progress])
        y0=np.array([ctx.vx_mps,ctx.vy_mps,ctx.heading_error_rad,ctx.lateral_error_m,0.])
        def rate(t):
            steps=max(1,int(abs(t)*100000)); h=t/steps; y=y0.copy()
            for _ in range(steps):
                k1=rhs(y);k2=rhs(y+h*k1/2);k3=rhs(y+h*k2/2);k4=rhs(y+h*k3)
                y+=h*(k1+2*k2+2*k3+k4)/6
            vx,vy,e,n,s=y; k=ctx.curvature_inv_m+ctx.curvature_gradient_inv_m2*s
            return (vx*np.cos(e)-vy*np.sin(e))/(1.-k*n)
        h=1e-3; fd=(rate(h)-rate(-h))/(2*h)
        self.assertAlmostEqual(fd,a,delta=1.e-5)
        self.assertAlmostEqual((rate(h/2)-rate(-h/2))/h,a,delta=1.e-5)
        self.assertAlmostEqual(body_vx_dot_to_path_progress(vx_dot,ctx),a,places=12)
        with self.assertRaisesRegex(ValueError,"vy_dot"):
            path_progress_to_body_vx_dot(1.,PathKinematicContext(1.,0.,0.,0.,0.,0.,0.,0.,False))
        singular=PathKinematicContext(1.,0.,1.57079632679,0.,0.,0.,0.,0.,True)
        with self.assertRaisesRegex(ValueError,"Frenet"):
            path_progress_to_body_vx_dot(1.,singular)

    def test_actual_provider_curvature_capacity_order_immutable_and_domain(self):
        p=self.provider()
        straight=p.evaluate(CapabilityQuery(15.,0.,.5)); corner=p.evaluate(CapabilityQuery(15.,.03,.5))
        self.assertTrue(straight.in_domain)
        self.assertTrue(corner.in_domain)
        self.assertGreater(corner.brake_net_progress_acceleration_mps2,straight.brake_net_progress_acceleration_mps2)
        # Curvature demand reduces brake authority; unsupported high-curvature
        # nodes fail closed rather than preserving a straight-line drive cap.
        self.assertFalse(p.evaluate(CapabilityQuery(25.,.03,.5)).in_domain)
        self.assertIn("lateral_demand_exceeds_axle_capacity",p.evaluate(CapabilityQuery(25.,.03,.5)).limiting_constraints)
        qs=[CapabilityQuery(8.,0.,.2),CapabilityQuery(15.,.03,.5),CapabilityQuery(20.,.03,.8)]
        first=[p.evaluate(q) for q in qs];second=[p.evaluate(q) for q in reversed(qs)]
        self.assertEqual(first,list(reversed(second)))
        self.assertFalse(p.evaluate(CapabilityQuery(30.,0.,.5)).in_domain)
        with self.assertRaises(AttributeError): p.identity="changed"
        for args in (dict(wheel_radii_m=(.29,.29,float("nan"),.29)),
                     dict(normal_loads_n=(1.,2.,float("inf"),4.)),
                     dict(longitudinal_capacity_n=(1.,2.,-1.,4.)),
                     dict(longitudinal_capacity_n=(1.,2.,float("nan"),4.)),
                     dict(speed_grid_mps=(1.,float("nan"))),dict(soc_grid=(-.1,.8))):
            with self.assertRaises(ValueError): self.provider(**args)
        zero=self.provider(longitudinal_capacity_n=(0.,0.,0.,0.))
        z=zero.evaluate(CapabilityQuery(15.,0.,.5))
        self.assertTrue(z.in_domain);self.assertLess(z.coast_net_progress_acceleration_mps2,0.)
        self.assertAlmostEqual(z.drive_net_progress_acceleration_mps2,z.coast_net_progress_acceleration_mps2,places=12)

    def test_precomputed_brake_cell_pack_independent_reference_support_matrix(self):
        p=self.provider()
        self.assertEqual(len(p._brake_cell_interpolants), (len(p.speed_grid)-1)*(len(p.curvature_grid)-1)*(len(p.soc_grid)-1))
        checked=[0]
        def reference(query):
            checked[0]+=1
            return self.old_evaluate(p,query)
        def make(f):
            return CapabilityQuery(p.speed_grid[0]+f[0]*(p.speed_grid[1]-p.speed_grid[0]),
                p.curvature_grid[0]+f[1]*(p.curvature_grid[1]-p.curvature_grid[0]),
                p.soc_grid[0]+f[2]*(p.soc_grid[1]-p.soc_grid[0]))
        def support_count(q):
            lo=[]; frac=[]
            for axis,x in zip((p.speed_grid,p.curvature_grid,p.soc_grid),(q.speed_mps,q.curvature_inv_m,q.soc)):
                h=min(max(int(np.searchsorted(axis,x,side="right")-1),0),len(axis)-2)
                lo.append(h);frac.append((x-axis[h])/(axis[h+1]-axis[h]))
            return sum(math.prod(frac[d] if (mask>>d)&1 else 1-frac[d] for d in range(3))>1e-15 for mask in range(8))
        queries=[make((.5,.5,.5)),make((0.,.5,.5)),make((0.,0.,.5)),make((0.,0.,0.)),
                 make((4.800031453169534e-13,.9999993302585813,.9999999999999999)),
                 make((8.928128934023541e-08,.9999999868058336,.9999999846743417)),
                 make((.6539225335338404,.9999999999999998,.99999999999989)),
                 make((.2084959912829929,.9999999999997744,.5258241085163375)),
                 make((.09412345622921847,2.817227340710142e-15,.041880336369846005)),
                 make((.9999999999999993,5.8453185845805e-14,.9999999999999998)),
                 make((3.196827263056226e-16,9.094763695056836e-16,2.0547912236608127e-16)),
                 make((.9999955564915548,7.762749373859486e-10,.8802689180835283))]
        # Deterministically scan randomized fractions once, retaining one case per
        # cardinality (including near-cutoff floating point support patterns).
        rng=np.random.default_rng(314159); witnesses={support_count(q):q for q in queries}
        for _ in range(25000):
            f=rng.uniform(1e-16,1.-1e-16,size=3)
            q=make(tuple(float(x) for x in f)); witnesses.setdefault(support_count(q),q)
            if len(witnesses)==8: break
        queries=list(witnesses.values())
        counts=set(witnesses)
        self.assertEqual(counts,set(range(1,9)),f"coverage support counts={sorted(counts)}")
        for q in queries:
            expected=reference(q)
            actual=p.evaluate(q)
            self.assertEqual(self.payload(actual),self.payload(expected),f"query={q}, supports={support_count(q)}")
        for q in (CapabilityQuery(p.speed_grid[0]-.01,p.curvature_grid[0],p.soc_grid[0]),
                  CapabilityQuery(25.,.08,.8),CapabilityQuery(25.,.03,.5),CapabilityQuery(15.,0.,.5),
                  CapabilityQuery(15.,.03,.5),CapabilityQuery(15.,0.,.8)):
            actual=p.evaluate(q)
            if (q.speed_mps,q.curvature_inv_m,q.soc)==(25.,.03,.5):
                self.assertFalse(actual.in_domain)
                self.assertIn("lateral_demand_exceeds_axle_capacity",actual.limiting_constraints)
            self.assertEqual(self.payload(actual),self.payload(reference(q)))
        malformed=object.__new__(CapabilityQuery)
        for name,value in (("speed_mps",float("nan")),("curvature_inv_m",.02),("soc",.4),("yaw_moment_request_nm",0.)):
            object.__setattr__(malformed,name,value)
        for label,newq,ref in (("wrong type",object(),lambda:self.old_evaluate(p,object())),
                               ("yaw",CapabilityQuery(15.,.02,.4,1.),lambda:self.old_evaluate(p,CapabilityQuery(15.,.02,.4,1.))),
                               ("NaN",malformed,lambda:self.old_evaluate(p,malformed)),
                               ("infinite",object.__new__(CapabilityQuery),None)):
            if label=="infinite":
                for name,value in (("speed_mps",float("inf")),("curvature_inv_m",.02),("soc",.4),("yaw_moment_request_nm",0.)):
                    object.__setattr__(newq,name,value)
                ref=lambda:self.old_evaluate(p,newq)
            def outcome(call):
                try: return ("ok",self.payload(call()))
                except Exception as exc: return ("error",type(exc),str(exc))
            self.assertEqual(outcome(lambda:p.evaluate(newq)),outcome(ref),label)
        for pass_index in range(5):
            ordered=queries if pass_index%2==0 else list(reversed(queries))
            for q in ordered:
                self.assertEqual(self.payload(p.evaluate(q)),self.payload(reference(q)))
        for grid,rows in p._brake_cell_interpolants:
            self.assertEqual(len(rows),8); self.assertEqual(tuple(sorted(set(grid))),grid)
            with self.assertRaises(TypeError): rows[0][0]=123.
        with self.assertRaises(AttributeError): p._brake_cell_interpolants=()
        self.assertEqual(checked[0],6*len(queries)+6)

    def test_brake_cell_pack_performance_cap_falls_back_exactly(self):
        from unittest.mock import patch
        baseline=self.provider()
        queries=(CapabilityQuery(10.,.015,.35),CapabilityQuery(15.,.03,.5),
                 CapabilityQuery(5.,0.,.2),CapabilityQuery(15.,0.,.5))
        expected=[self.payload(self.old_evaluate(baseline,q)) for q in queries]
        with patch("simplified_vehicle_models.solvers.longitudinal_capability._BRAKE_CELL_INTERPOLANT_MAX_CELLS",0):
            fallback=self.provider()
        self.assertIsNone(fallback._brake_cell_interpolants)
        self.assertEqual(fallback.identity,baseline.identity)
        for q,result in zip(queries,expected):
            self.assertEqual(self.payload(fallback.evaluate(q)),result)
        q=CapabilityQuery(15.,.03,.5)
        self.assertEqual(self.payload(fallback.evaluate(q)),self.payload(baseline.evaluate(q)))

    def test_asymmetric_wheel_radii_effective_mass_and_signed_force_oracle(self):
        # Carried Task 2 oracle: FL/FR/RL/RR radii must enter wheel inertia as
        # sum(Iw / r_i**2), not a common-radius approximation. Compare the
        # provider query's signed progress accelerations with its explicitly
        # declared effective-mass convention at the source grid node.
        radii=(.25,.28,.31,.34)
        p=self.provider(wheel_radii_m=radii)
        q=CapabilityQuery(15.,0.,.5)
        c=p.evaluate(q)
        self.assertTrue(c.in_domain)
        effective_mass=self.vehicle.total_mass + self.vehicle.wheel_inertia*sum(1./r**2 for r in radii)
        self.assertAlmostEqual(effective_mass,296.8707309,places=6)
        # The capability implementation exposes its raw grid node only as an
        # immutable table. This exact-node check verifies interpolation does
        # not perturb the common-prefix signed provider values.
        node=p._table[p.speed_grid.index(15.)][p.curvature_grid.index(0.)][p.soc_grid.index(.5)]
        np.testing.assert_allclose(
            [c.drive_net_progress_acceleration_mps2,
             c.coast_net_progress_acceleration_mps2,
             c.brake_net_progress_acceleration_mps2],
            node,rtol=0.,atol=1e-12)
        self.assertAlmostEqual(c.drive_net_progress_acceleration_mps2,1.54203,delta=.02)
        self.assertAlmostEqual(c.coast_net_progress_acceleration_mps2,-.716600,delta=.02)
        self.assertAlmostEqual(c.brake_net_progress_acceleration_mps2,-13.40027,delta=.03)

    def test_model_calibration_source_hashes_bind_and_optional_derate(self):
        p=self.provider(); old=p.model_hash
        changed=replace(self.vehicle,maximum_pedal_force=self.vehicle.maximum_pedal_force*.9)
        q=CurrentVehicleCapability(changed,wheel_radii_m=(.29,)*4,speed_grid_mps=(5.,15.),curvature_grid_inv_m=(0.,.03),soc_grid=(.2,.8),source_label="iem24_murphy_real_planar_7dof.xml",calibration_label="compiled-IEM-calibration-v1")
        self.assertNotEqual(old,q.model_hash);self.assertNotEqual(p.calibration_hash,q.calibration_hash)
        self.assertNotEqual(p.identity,q.identity)
        with self.assertRaisesRegex(ValueError,"source closure hash mismatch"):
            self.provider(expected_source_hash="stale")
        with self.assertRaisesRegex(ValueError,"model source hash mismatch"):
            self.provider(model_source_hashes={"models/formulations/iem24_murphy_real_planar_7dof.xml":"stale"})
        limited=self.provider(longitudinal_capacity_n=(100.,100.,100.,100.))
        self.assertLess(limited.evaluate(CapabilityQuery(15.,0.,.5)).drive_net_progress_acceleration_mps2,
                        p.evaluate(CapabilityQuery(15.,0.,.5)).drive_net_progress_acceleration_mps2)

    def test_pure_out_of_order_provider_contract(self):
        class TableProvider:
            identity="fixed-provider"
            def evaluate(self,q): return LongitudinalCapability(3.,-.5,-4.,True,("fixture",),self.identity)
        p=TableProvider();queries=[CapabilityQuery(x,k,s) for x,k,s in ((4.,0.,.2),(8.,.1,.8),(2.,-.1,.5))]
        before=[p.evaluate(q) for q in queries];after=[p.evaluate(q) for q in reversed(queries)]
        self.assertEqual(before,list(reversed(after)))

if __name__ == "__main__": unittest.main()
