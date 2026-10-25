import unittest
import numpy as np
from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.solvers.yaw_aware_profile import YawPath
from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings
from simplified_vehicle_models.solvers.yaw_profile_transition import transition_intervals, edge_evidence


def make_map(mask=None, into=100., out=100.):
    speed=np.array([0.,10.,20.]); ax=np.array([-2.,0.,2.]); ay=np.array([-20.,0.,20.])
    shape=(3,3,3)
    return YawCapabilityMap(speed,ax,ay,np.full(shape,into),np.full(shape,out),
        np.ones(shape,dtype=bool) if mask is None else mask,1.,"physical_ideal_tv_v1","test")


class TransitionIntervalTests(unittest.TestCase):
    def setUp(self):
        self.path=YawPath(np.array([0.,.05, .1]),np.zeros(3),np.zeros(3),False)
        self.settings=CoupledProfileSettings()

    def test_analytic_rectangle_distances(self):
        for ds in (1.,.05,.01):
            path=YawPath(np.array([0.,ds,2*ds]),np.zeros(3),np.zeros(3),False)
            cap=make_map()
            for direction in ('forward','backward'):
                fixed=100.
                got=transition_intervals(path,cap,0,fixed,direction,(0.,400.),settings=self.settings)
                expected=(fixed-2*ds*2,fixed+2*ds*2)
                self.assertEqual(len(got),1)
                self.assertAlmostEqual(got[0].lower,expected[0],places=7)
                self.assertAlmostEqual(got[0].upper,expected[1],places=7)

    def test_edge_evidence_strict_oracle(self):
        cap=make_map(); e=edge_evidence(self.path,cap,0,25.,25.,settings=self.settings)
        self.assertTrue(e['feasible'])
        self.assertEqual(e['reason'].value,'supported')
        self.assertEqual(e['capability_identity'], cap.source_identity)
        with self.assertRaises(TypeError):
            edge_evidence(self.path,cap,0,25.,25.,settings=None)

    def test_map_endpoint_bounds_are_intersected(self):
        cap=make_map()
        forward=transition_intervals(self.path,cap,0,0.,'forward',(0.,1000.),settings=self.settings)
        self.assertTrue(all(x.upper <= 400. for x in forward))
        backward=transition_intervals(self.path,cap,0,400.,'backward',(0.,1000.),settings=self.settings)
        self.assertTrue(all(x.upper <= 400. for x in backward))
        self.assertEqual(transition_intervals(self.path,cap,0,401.,'forward',(0.,1000.),settings=self.settings),())

    def test_masked_hole_and_exact_knot_face(self):
        mask=np.ones((3,3,3),dtype=bool)
        # Mask all but the speed=10 face. Strict knot query remains supported.
        mask[0,:,:]=False; mask[2,:,:]=False
        cap=make_map(mask=mask)
        e=edge_evidence(self.path,cap,0,100.,100.,settings=self.settings)
        self.assertTrue(e['feasible'])
        iv=transition_intervals(self.path,cap,0,100.,'forward',(0.,400.),settings=self.settings)
        self.assertTrue(any(x.lower <= 100. <= x.upper for x in iv))

    def test_asymmetric_yaw_and_signed_curvature(self):
        cap=make_map(into=3.,out=7.)
        path=YawPath(np.array([0.,.05,.1]),np.array([.02,.02,.02]),np.array([-.01,-.01,-.01]),False)
        for direction in ('forward','backward'):
            iv=transition_intervals(path,cap,0,100.,direction,(0.,400.),settings=self.settings)
            for item in iv:
                for q in (item.lower,item.upper,(item.lower+item.upper)/2):
                    left,right=(100.,q) if direction=='forward' else (q,100.)
                    self.assertTrue(edge_evidence(path,cap,0,left,right,settings=self.settings)['feasible'])

    def test_random_dense_membership_constant_rectangles(self):
        rng=np.random.default_rng(1729); cap=make_map()
        for _ in range(100):
            fixed=float(rng.uniform(10,250)); ds=float(rng.choice([.01,.05,.2,1.]))
            path=YawPath(np.array([0.,ds,2*ds]),np.zeros(3),np.zeros(3),False)
            lo,hi=sorted((max(0.,fixed-8*ds),fixed+8*ds))
            iv=transition_intervals(path,cap,0,fixed,'forward',(lo,hi),settings=self.settings)
            probes=np.linspace(lo,hi,101)
            for q in probes:
                # Compare dense interior samples; algebraic map-face bounds can
                # differ by a few ulps after q->u->ax round trips. Strict
                # representable face behavior is checked separately below.
                if any(abs(float(q)-x) <= 1e-11*max(1.,abs(x)) for x in (fixed-2*ds*2., fixed+2*ds*2.)):
                    continue
                strict=edge_evidence(path,cap,0,fixed,float(q),settings=self.settings)['feasible']
                oracle=any((q>x.lower or (x.lower_included and q==x.lower)) and (q<x.upper or (x.upper_included and q==x.upper)) for x in iv)
                self.assertEqual(oracle,strict)
            for x in iv:
                for q in (x.lower, x.upper):
                    self.assertEqual(x.lower_included if q == x.lower else x.upper_included,
                                     edge_evidence(path,cap,0,fixed,float(q),settings=self.settings)['feasible'])


if __name__=='__main__': unittest.main()
