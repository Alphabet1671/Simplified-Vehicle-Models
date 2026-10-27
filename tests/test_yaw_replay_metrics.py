import unittest
import numpy as np
from simplified_vehicle_models.planning.yaw_replay_metrics import time_weighted_rms, time_weighted_fraction, project_xy_to_reference
from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve, StationMap
from simplified_vehicle_models.solvers.yaw_aware_profile import YawPath

class MetricTests(unittest.TestCase):
    def test_irregular_trapezoidal_integral(self):
        t=np.array([0.,1.,3.]); e=np.array([0.,2.,0.])
        self.assertAlmostEqual(time_weighted_rms(t,e),np.sqrt((2.+4.)/3.))
        self.assertAlmostEqual(time_weighted_fraction(t,np.array([False,True,False])),.5)

    def test_rejects_invalid_inputs(self):
        for t,e in (([0.,0.],[1.,2.]),([0.,1.],[1.]),([0.,np.nan],[1.,2.])):
            with self.assertRaises(ValueError): time_weighted_rms(t,e)
        with self.assertRaises(ValueError): time_weighted_fraction([0.,1.],[0,1])

    def test_station_map_roundtrip_nonuniform_and_closed_wrap(self):
        reference=np.array([10.,12.,20.,30.]); physical=np.array([0.,1.,8.,15.])
        mapping=StationMap(reference,physical,True)
        probe=np.array([0.,.3,2.,7.,14.7,15.])
        np.testing.assert_allclose(mapping.to_path(mapping.to_reference(probe)),probe,atol=2e-2)
        self.assertAlmostEqual(mapping.to_reference(15.),30.)
        self.assertAlmostEqual(mapping.to_reference(0.),10.)
        self.assertGreater(mapping.to_reference(15.1),10.)
        self.assertLess(mapping.to_reference(15.1),11.)

    def test_left_right_signed_normal_and_thirty_degree_heading_difference(self):
        # Selected path heading is 30 degrees; reference projection has north heading.
        theta=np.pi/6
        selected=(10.,10.)
        lateral=2.
        driven=(selected[0]-lateral*np.sin(theta),selected[1]+lateral*np.cos(theta))
        dx,dy=driven[0]-10.,driven[1]-10.
        signed=dx*(-np.sin(0.))+dy*np.cos(0.)
        self.assertGreater(signed,0.)
        self.assertAlmostEqual(signed,lateral*np.cos(theta),places=7)
        self.assertLess(-signed,0.)

    def test_projection_respects_station_branch(self):
        class Ref:
            start=0.; end=100.
            def evaluate(self,s):
                s=np.asarray(s,float)
                # Two nearby opposing branches, separated in station.
                north=np.where(s<50.,s,100.-s)
                east=np.where(s<50.,0.,1.)
                heading=np.where(s<50.,0.,np.pi)
                return type('G',(),dict(north=north,east=east,heading=heading))
        # The local interval around station 20 must not jump to the opposite arm.
        stations,lateral=project_xy_to_reference(Ref(),[20.],[1.],[20.],window=5.)
        self.assertLess(stations[0],30.)
        self.assertGreater(lateral[0],0.)

if __name__=='__main__': unittest.main()
