import unittest
import numpy as np

from simplified_vehicle_models.planning.yaw_corridor_validation import (
    check_footprint_samples, check_swept_footprint,
)
from simplified_vehicle_models.tracks.corridor import Corridor


class Reference:
    start=0.; end=100.; is_closed=False
    def evaluate(self,s):
        s=np.asarray(s)
        return type('G',(),{'north':s,'east':np.zeros_like(s),'heading':np.zeros_like(s)})()


class FootprintTests(unittest.TestCase):
    def setUp(self):
        self.ref=Reference()
        self.corridor=Corridor(np.array([0.,100.]),np.array([-3.,-3.]),
                               np.array([3.,3.]),'surveyed',False)

    def test_missing_body_dimensions_unknown_but_wheels_reported(self):
        result=check_footprint_samples([10.],[50.],[0.],[0.],[50.],self.corridor,self.ref,
                                      wheelbase=2.,track_width=1.)
        self.assertEqual(result['status'],'unknown')
        self.assertEqual(result['wheel_contact']['status'],'pass')
        self.assertEqual(result['body_envelope']['status'],'unknown')

    def test_body_and_wheels_pass_straight(self):
        result=check_footprint_samples([1.],[50.],[0.],[0.],[50.],self.corridor,self.ref,
            front_extent=2.,rear_extent=1.,half_width=1.,wheelbase=2.,track_width=1.)
        self.assertEqual(result['status'],'unknown')
        self.assertIn('empirical only', result['geometric_representation'])
        self.assertEqual(result['body_envelope']['status'], 'unknown')

    def test_yaw_corner_outside_while_center_inside(self):
        narrow=Corridor(np.array([0.,100.]),np.array([-1.,-1.]),np.array([1.,1.]),'surveyed',False)
        result=check_footprint_samples([1.],[50.],[0.],[np.pi/6],[50.],narrow,self.ref,
            front_extent=2.,rear_extent=1.,half_width=.8)
        self.assertEqual(result['status'],'fail')

    def test_asymmetric_corridor_and_pinched_station(self):
        pinch=Corridor(np.array([0.,50.,100.]),np.array([-2.,-2.,-2.]),
                       np.array([2.,.4,2.]),'surveyed',False)
        result=check_footprint_samples([1.],[50.],[0.],[0.],[50.],pinch,self.ref,
            front_extent=1.,rear_extent=1.,half_width=.7)
        self.assertEqual(result['status'],'fail')

    def test_swept_never_certifies_sampling_alone(self):
        result=check_swept_footprint([1.,2.],[20.,21.],[0.,0.],[0.,0.],[20.,21.],
            self.corridor,self.ref,front_extent=1.,rear_extent=1.,half_width=.5)
        self.assertEqual(result['status'],'unknown')
        self.assertFalse(result['between_step_containment_certified'])

    def test_positive_supplied_bound_certifies_only_interval_field(self):
        result=check_swept_footprint([1.,2.],[20.,21.],[0.,0.],[0.,0.],[20.,21.],
            self.corridor,self.ref,front_extent=1.,rear_extent=1.,half_width=.5,
            interval_bounds=[.5])
        self.assertFalse(result['between_step_containment_certified'])
        self.assertFalse(result['rigorous_interval_bounds']['certified'])
        self.assertEqual(result['status'], 'unknown')

    def test_dense_cubic_quarter_excursion_cannot_pass(self):
        # p(u)=80u(1-u)(u-.5) has zero at u=0,.5,1 but a large
        # quarter-step excursion; construct the reported dense observations.
        u=np.array([0.,.25,.5,.75,1.])
        x=50.+80.*u*(1.-u)*(u-.5)
        result=check_swept_footprint(u,x,np.zeros_like(u),np.zeros_like(u),x,
            self.corridor,self.ref,front_extent=.1,rear_extent=.1,half_width=.2)
        self.assertIn(result['status'], ('unknown','fail'))
        self.assertFalse(result['between_step_containment_certified'])
        self.assertGreater(abs(x[1]-50.), 3.)

    def test_inward_notch_edge_sample_is_not_a_containment_pass(self):
        # All body vertices lie in this narrow span, but the top edge crosses
        # a notch centered between its coarse samples; sampling is empirical.
        corridor=Corridor(np.array([0.,49.9,50.,50.1,100.]),
            np.array([-5.,-5.,-5.,-5.,-5.]),np.array([5.,5.,.0,5.,5.]),'surveyed',False)
        result=check_footprint_samples([1.],[50.],[0.],[0.],[50.],corridor,self.ref,
            front_extent=.5,rear_extent=.5,half_width=2.,edge_samples=2)
        self.assertNotEqual(result['body_envelope']['status'],'pass')
        self.assertFalse(result['between_step_containment_certified'])

    def test_enclosing_bound_arithmetic_straight_corridor_and_cubic(self):
        from simplified_vehicle_models.planning.yaw_corridor_validation import _motion_clearance_lower_bound
        # Analytic lower bound: 2 m clearance minus 0.25 m translation and
        # 0.1 rad rotation at 2 m radius gives 1.55 m.
        self.assertAlmostEqual(_motion_clearance_lower_bound(2.,.25,.1,2.),1.55)
        u=np.array([0.,.25,.5,.75,1.]); p=80.*u*(1.-u)*(u-.5)
        self.assertAlmostEqual(p[0],0.); self.assertAlmostEqual(p[2],0.); self.assertAlmostEqual(p[4],0.)
        self.assertAlmostEqual(abs(p[1]),3.75)

    def test_nonpositive_interval_bound_fails(self):
        result=check_swept_footprint([1.,2.],[20.,21.],[0.,0.],[0.,0.],[20.,21.],
            self.corridor,self.ref,front_extent=1.,rear_extent=1.,half_width=.5,
            interval_bounds=[-.1])
        self.assertEqual(result['status'],'unknown')
        self.assertFalse(result['between_step_containment_certified'])


if __name__=='__main__': unittest.main()
