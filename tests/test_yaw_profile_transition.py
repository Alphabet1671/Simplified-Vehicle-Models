import unittest
import numpy as np
from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.solvers.yaw_aware_profile import YawPath
from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings
from simplified_vehicle_models.solvers.yaw_profile_transition import transition_intervals, edge_evidence, _transition_cuts


def make_map(mask=None, into=100., out=100.):
    speed=np.array([0.,10.,20.]); ax=np.array([-2.,0.,2.]); ay=np.array([-20.,0.,20.])
    shape=(3,3,3)
    return YawCapabilityMap(speed,ax,ay,np.full(shape,into),np.full(shape,out),
        np.ones(shape,dtype=bool) if mask is None else mask,1.,"physical_ideal_tv_v1","test")


class TransitionIntervalTests(unittest.TestCase):
    def setUp(self):
        self.path=YawPath(np.array([0.,.05, .1]),np.zeros(3),np.zeros(3),False)
        self.settings=CoupledProfileSettings()

    def test_restricted_cuts_match_full_axis_scan_randomized(self):
        rng=np.random.default_rng(415)
        def full(axes,lo,hi,fixed,ds,k,direction):
            cuts=[lo,hi]
            for u in axes[0]:
                if lo<u<hi: cuts.append(float(u))
            for a in axes[1]:
                sq=fixed+ds*a if direction=="forward" else fixed-ds*a
                if sq>=0 and lo<np.sqrt(sq)<hi: cuts.append(float(np.sqrt(sq)))
            if k:
                for y in axes[2]:
                    sq=y/k
                    if sq>=0 and lo<np.sqrt(sq)<hi: cuts.append(float(np.sqrt(sq)))
            return sorted(cuts)
        for _ in range(1000):
            axes=(np.sort(rng.uniform(0.,100.,51)),np.sort(rng.uniform(-20.,20.,61)),
                  np.sort(rng.uniform(-80.,80.,71)))
            lo=float(rng.uniform(.01,80.)); hi=lo+float(rng.uniform(1.e-10,5.))
            fixed=float(rng.uniform(0.,1.e5)); ds=float(10.**rng.uniform(-7,1)); k=float(rng.uniform(-.3,.3))
            direction=("forward","backward")[rng.integers(2)]
            np.testing.assert_array_equal(_transition_cuts(axes,lo,hi,fixed,ds,k,direction),
                                          full(axes,lo,hi,fixed,ds,k,direction))

    def test_narrow_high_speed_cell_uses_local_polynomial_without_unresolved(self):
        # Fine map cells at substantial u are ill-conditioned if multilinear
        # weights are expanded in global u then affine-composed.
        speed=np.array([12.86,12.87,12.871,12.88])
        ax=np.array([-.02,-.01,0.,.01])
        ay=np.array([16.54,16.55,16.56,16.57])
        ii,jj,kk=np.indices((4,4,4))
        moment=1.e9+3.e6*ii-7.e6*jj+11.e6*kk+1.e5*ii*jj*kk
        cap=YawCapabilityMap(speed,ax,ay,moment,moment+2.e8,np.ones((4,4,4),bool),
                             1.,"physical_ideal_tv_v1","narrow-local")
        path=YawPath(np.array([0.,.5,1.]),np.full(3,.1),np.zeros(3),False)
        u=12.8705; fixed=u*u-.5*(-.005)
        intervals=transition_intervals(path,cap,0,fixed,"forward",
                                       (fixed-.01,fixed+.01),settings=self.settings)
        self.assertTrue(intervals)
        for item in intervals:
            for q in (item.lower,(item.lower+item.upper)/2,item.upper):
                self.assertTrue(edge_evidence(path,cap,0,fixed,q,settings=self.settings)["feasible"])

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
        mask=np.ones((7,3,3),dtype=bool)
        mask[3,:,:]=False  # isolated masked speed plane creates a true hole
        cap=YawCapabilityMap(np.array([0.,5.,10.,15.,20.,30.,40.]),np.array([-10.,0.,10.]),np.array([-20.,0.,20.]),
            np.full((7,3,3),100.),np.full((7,3,3),100.),mask,1.,"physical_ideal_tv_v1","test-hole")
        long_path=YawPath(np.array([0.,100.,200.]),np.zeros(3),np.zeros(3),False)
        # A supported speed face (u=5) has feasible strict nonzero corners;
        # the unsupported speed=10 plane remains a separate hole in the sweep.
        e=edge_evidence(long_path,cap,0,25.,25.,settings=self.settings)
        self.assertTrue(e['feasible'])
        iv=transition_intervals(long_path,cap,0,100.,'forward',(0.,1600.),settings=self.settings)
        self.assertGreaterEqual(len(iv),2)
        for item in iv:
            for q in (item.lower,item.upper,np.nextafter(item.lower,item.upper),np.nextafter(item.upper,item.lower)):
                self.assertTrue(edge_evidence(long_path,cap,0,100.,float(q),settings=self.settings)['feasible'])

    def test_asymmetric_yaw_and_signed_curvature(self):
        cap=make_map(into=3.,out=7.)
        path=YawPath(np.array([0.,.05,.1]),np.array([.02,.02,.02]),np.array([-.01,-.01,-.01]),False)
        for direction in ('forward','backward'):
            iv=transition_intervals(path,cap,0,100.,direction,(0.,400.),settings=self.settings)
            for item in iv:
                for q in (item.lower,item.upper,(item.lower+item.upper)/2):
                    left,right=(100.,q) if direction=='forward' else (q,100.)
                    self.assertTrue(edge_evidence(path,cap,0,left,right,settings=self.settings)['feasible'])

    def test_empty_domain_and_standing_launch(self):
        cap=make_map()
        self.assertEqual(transition_intervals(self.path,cap,0,100.,'forward',(500.,600.),settings=self.settings),())
        iv=transition_intervals(self.path,cap,0,0.,'forward',(0.,100.),settings=self.settings)
        self.assertTrue(iv)
        self.assertTrue(any(x.lower == 0. for x in iv))
        self.assertTrue(edge_evidence(self.path,cap,0,0.,0.,settings=self.settings)['feasible'])

    def test_masked_hole_yields_disconnected_supported_regions(self):
        mask=np.ones((3,3,3),dtype=bool)
        mask[:,1,:]=False
        mask[:,0,1]=True
        mask[1,1,1]=True
        cap=make_map(mask=mask)
        iv=transition_intervals(self.path,cap,0,100.,'forward',(0.,400.),settings=self.settings)
        self.assertGreaterEqual(len(iv),2)
        for x in iv:
            for q in (x.lower,x.upper,np.nextafter(x.lower,x.upper),np.nextafter(x.upper,x.lower)):
                self.assertTrue(edge_evidence(self.path,cap,0,100.,float(q),settings=self.settings)['feasible'])

    def test_exact_pinned_acceleration_and_lateral_faces(self):
        cap=make_map()
        # Fixed q and selected endpoint make ax exactly a grid knot; likewise
        # zero curvature pins ay at its exact grid face.
        q=100.+2*self.path.distance[1]*2.
        ev=edge_evidence(self.path,cap,0,100.,q,settings=self.settings)
        self.assertTrue(ev['feasible'])
        self.assertAlmostEqual(ev['longitudinal_acceleration'],2.,places=12)
        self.assertEqual(ev['lateral_acceleration'],0.)
        # Test exact interior acceleration face ax=0 with a singleton q-domain;
        # the ax=2 outer map edge above remains a direct strict-query check.
        accel_intervals = transition_intervals(self.path, cap, 0, 100., 'forward',
                                               (100., 100.), settings=self.settings)
        self.assertTrue(any(x.lower <= 100. <= x.upper for x in accel_intervals), accel_intervals)
        lateral_intervals = transition_intervals(self.path, cap, 0, 100., 'forward',
                                                 (100., 100.), settings=self.settings)
        self.assertTrue(any(x.lower <= 100. <= x.upper for x in lateral_intervals), lateral_intervals)
        # The exact (ax=0, ay=0) query has one strict nonzero support
        # corner. Keep that corner feasible while masking only zero-weight
        # neighbors; strict map support semantics remain unchanged.
        mask=np.zeros((3,3,3),dtype=bool)
        mask[:,1,1]=True
        facecap=make_map(mask=mask)
        self.assertTrue(edge_evidence(self.path,facecap,0,100.,100.,settings=self.settings)['feasible'])

    def test_repeated_tangent_margin_root_is_feasible_point(self):
        # With fixed Q=0, zero curvature and kp=.2, constant zero into-moment
        # yields M_into-D = -.2*u^2: a repeated root at u=0, feasible only
        # at the standing point. The oracle must preserve that zero-width set.
        cap=make_map(into=0.,out=100.)
        path=YawPath(np.array([0.,.05,.1]),np.zeros(3),np.array([.2,.2,.2]),False)
        iv=transition_intervals(path,cap,0,0.,'forward',(0.,400.),settings=self.settings)
        self.assertTrue(any(x.lower == 0. and x.upper == 0. for x in iv), iv)
        self.assertTrue(edge_evidence(path,cap,0,0.,0.,settings=self.settings)['feasible'])
        for item in iv:
            self.assertTrue(edge_evidence(path,cap,0,0.,item.lower,settings=self.settings)['feasible'])
            self.assertTrue(edge_evidence(path,cap,0,0.,item.upper,settings=self.settings)['feasible'])

    def test_near_tangent_margin_root(self):
        # The exact tangent occurs at u=10: demand=20 and constant margin=20.
        # A tiny downward offset retains a single near-tangent feasible point.
        cap=make_map(into=20.,out=100.)
        path=YawPath(np.array([0.,.05,.1]),np.array([0.,0.,0.]),np.array([.2,.2,.2]),False)
        iv=transition_intervals(path,cap,0,100.,'forward',(0.,400.),settings=self.settings)
        self.assertTrue(iv)
        for x in iv:
            self.assertTrue(edge_evidence(path,cap,0,100.,x.lower,settings=self.settings)['feasible'])
            self.assertTrue(edge_evidence(path,cap,0,100.,x.upper,settings=self.settings)['feasible'])

    def test_backward_near_zero_roundoff_keeps_exact_boundary(self):
        cap=make_map()
        # Pick a fixed q whose backward acceleration-domain limit is zero;
        # cancellation in the free-q transform may put that limit a few ulps
        # below zero, but q=0 itself is strictly feasible.
        ds=float(self.path.distance[1]-self.path.distance[0])
        fixed=2.0*ds*float(cap.longitudinal_acceleration[0])
        fixed=abs(fixed)
        iv=transition_intervals(self.path,cap,0,fixed,'backward',(0.,400.),settings=self.settings)
        self.assertTrue(any(item.lower == 0. for item in iv),iv)
        self.assertTrue(edge_evidence(self.path,cap,0,0.,fixed,settings=self.settings)['feasible'])

    def test_backward_dense_oracle_membership(self):
        cap=make_map(into=8.,out=5.)
        path=YawPath(np.array([0.,.05,.1]),np.array([-.03,-.03,-.03]),np.array([.02,.02,.02]),False)
        iv=transition_intervals(path,cap,0,100.,'backward',(0.,400.),settings=self.settings)
        for q in np.linspace(0.,400.,601):
            strict=edge_evidence(path,cap,0,float(q),100.,settings=self.settings)['feasible']
            represented=any(x.lower <= q <= x.upper for x in iv)
            self.assertEqual(represented,strict)

    def test_random_dense_membership_nonrectangular_maps(self):
        rng=np.random.default_rng(71341)
        for case in range(100):
            base=make_map()
            into=rng.uniform(.2,30.,size=(3,3,3)); out=rng.uniform(.2,30.,size=(3,3,3))
            mask=rng.random((3,3,3))>.16
            mask[1,1,1]=True
            cap=YawCapabilityMap(base.speed,base.longitudinal_acceleration,base.lateral_acceleration,
                into,out,mask,1.,"physical_ideal_tv_v1",f"random-{case}")
            path=YawPath(np.array([0.,.05,.1]),rng.uniform(-.03,.03,3),rng.uniform(-.04,.04,3),False)
            fixed=float(rng.uniform(1.,380.)); direction="forward" if case%2==0 else "backward"
            iv=transition_intervals(path,cap,0,fixed,direction,(0.,400.),settings=self.settings)
            for q in np.linspace(0.,400.,151):
                left,right=(fixed,float(q)) if direction=="forward" else (float(q),fixed)
                strict=edge_evidence(path,cap,0,left,right,settings=self.settings)['feasible']
                represented=any(x.lower <= q <= x.upper for x in iv)
                self.assertEqual(represented,strict,(case,direction,q,iv))
            for item in iv:
                for q in (item.lower,item.upper,np.nextafter(item.lower,item.upper),np.nextafter(item.upper,item.lower)):
                    left,right=(fixed,float(q)) if direction=="forward" else (float(q),fixed)
                    self.assertTrue(edge_evidence(path,cap,0,left,right,settings=self.settings)['feasible'],(case,item,q))

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


class EdgeFeasibilityMatrixTests(unittest.TestCase):
    def test_matrix_matches_scalar_strict_oracle_exactly(self):
        from simplified_vehicle_models.solvers.yaw_profile_transition import edge_feasibility_matrix
        rng = np.random.default_rng(3)
        speed = np.linspace(0., 10., 11); ax = np.linspace(-3., 3., 7); ay = np.linspace(-6., 6., 9)
        shape = (speed.size, ax.size, ay.size)
        mask = rng.random(shape) > .15
        into = rng.uniform(0., 40., shape); out = rng.uniform(0., 40., shape)
        cap = YawCapabilityMap(speed, ax, ay, into, out, mask, 2., "physical_no_tv_v1", "matrix-test")
        d = np.linspace(0., 1., 21); k = np.linspace(-.25, .25, 21)
        path = YawPath(d, k, np.gradient(k, d), False)
        settings = CoupledProfileSettings()
        mismatches = feasible = 0
        for seg in range(20):
            q0 = np.r_[0., rng.uniform(0., 90., 12), speed[1]**2, speed[4]**2]
            q1 = np.r_[q0[1:] + rng.normal(0., .2, q0.size - 1).clip(-q0[1:]), speed[4]**2, 0.]
            matrix = edge_feasibility_matrix(path, cap, seg, q0, q1)
            for a in range(q0.size):
                for b in range(q1.size):
                    if q0[a] + q1[b] == 0: continue
                    scalar = bool(edge_evidence(path, cap, seg, float(q0[a]), float(q1[b]), settings=settings)["feasible"])
                    feasible += scalar
                    mismatches += scalar != bool(matrix[a, b])
        self.assertGreater(feasible, 5)  # both outcomes exercised
        self.assertEqual(mismatches, 0)

    def test_matrix_matches_scalar_on_exact_knots_and_tolerance_band(self):
        from simplified_vehicle_models.solvers.yaw_profile_transition import edge_feasibility_matrix
        speed = np.linspace(0., 10., 11); ax = np.linspace(-3., 3., 7); ay = np.linspace(-6., 6., 9)
        shape = (speed.size, ax.size, ay.size)
        mask = np.ones(shape, bool); mask[-1, -1, :] = False; mask[0, 0, 0] = False
        cap = YawCapabilityMap(speed, ax, ay, np.full(shape, 50.), np.full(shape, 50.), mask, 1.,
                               "physical_no_tv_v1", "edge-test")
        d = np.linspace(0., .2, 5); path = YawPath(d, np.full(5, .05), np.zeros(5), False)
        ds = float(d[1] - d[0]); settings = CoupledProfileSettings()
        top = speed[-1] ** 2
        pairs = []
        for target_ax in (ax[0], ax[-1], 0., ax[-1] + 1e-13, ax[-1] + 1e-11, ax[0] - 1e-13, ax[0] - 1e-11):
            for qm in (top, top - 1e-9, 1., 0.25):
                q0 = qm - ds * target_ax; q1 = qm + ds * target_ax
                if q0 >= 0 and q1 >= 0 and q0 + q1 > 0:
                    pairs.append((q0, q1))
        pairs += [(top, top), (top + 1e-11, top + 1e-11), (top + 1e-9, top + 1e-9)]
        q0 = np.array([p[0] for p in pairs]); q1 = np.array([p[1] for p in pairs])
        got = edge_feasibility_matrix(path, cap, 0, q0, q1, pairwise=True)
        want = [bool(edge_evidence(path, cap, 0, a, b, settings=settings)["feasible"]) for a, b in pairs]
        self.assertEqual(got.tolist(), want)
        self.assertIn(True, want); self.assertIn(False, want)
