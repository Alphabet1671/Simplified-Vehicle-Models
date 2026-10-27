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
        self.assertEqual(result['status'],'pass')
        self.assertEqual(result['geometric_representation'],'oriented rectangular body polygon with sampled edges')

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
        self.assertTrue(result['between_step_containment_certified'])
        self.assertTrue(result['rigorous_interval_bounds']['certified'])

    def test_nonpositive_interval_bound_fails(self):
        result=check_swept_footprint([1.,2.],[20.,21.],[0.,0.],[0.,0.],[20.,21.],
            self.corridor,self.ref,front_extent=1.,rear_extent=1.,half_width=.5,
            interval_bounds=[-.1])
        self.assertEqual(result['status'],'fail')
        self.assertFalse(result['between_step_containment_certified'])


if __name__=='__main__': unittest.main()
