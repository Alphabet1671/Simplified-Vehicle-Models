import unittest
from unittest.mock import patch
import numpy as np
from scipy.optimize import OptimizeResult
from simplified_vehicle_models.planning.yaw_aware_line import YawLineSettings, optimize_yaw_aware_line
from tests.test_yaw_aware_line import capability, circle_track


def profile_for(path, time_value=10.):
    from simplified_vehicle_models.solvers.yaw_aware_profile import YawAwareProfile
    speed=np.full(len(path.distance),5.)
    elapsed=np.r_[0.,np.cumsum(path.segment_lengths/5.)]
    return YawAwareProfile(path.distance,speed,np.zeros_like(speed),elapsed,time_value,True,1,0.,{"resolution_converged":True})


class YawLineStoppingContractTests(unittest.TestCase):
    def run_plan(self, **overrides):
        track=circle_track()
        values=dict(control_spacing=160.,grid_spacing=100.,dense_subdivisions=1,
            optimizer_max_iterations=2,maximum_outer_passes=3,maximum_speed=20.,seed_blends=(),
            maximum_profile_evaluations=64)
        values.update(overrides)
        settings=YawLineSettings(**values)
        return track,settings

    def minimizing(self, value=10.):
        def solve(fun,x0,**kwargs):
            fun(np.asarray(x0))
            return OptimizeResult(x=np.asarray(x0),success=True,nit=1)
        return solve

    def test_single_pass_is_nonconverged_and_records_cap(self):
        track,settings=self.run_plan(maximum_outer_passes=1)
        with patch('simplified_vehicle_models.planning.yaw_aware_line._corner_complexes',return_value=[(0.,track.lap_length)]), patch('simplified_vehicle_models.planning.yaw_aware_line.minimize',side_effect=self.minimizing()):
            result=optimize_yaw_aware_line(track,None,capability(),settings=settings)
        self.assertEqual(result.diagnostics['optimizer_stop_kind'],'single_pass')
        self.assertFalse(result.diagnostics['converged'])
        self.assertEqual(result.diagnostics['effective_maximum_outer_passes'],1)

    def test_two_complete_passes_within_tolerance_and_three_pass_cap(self):
        track,settings=self.run_plan(maximum_outer_passes=7)
        with patch('simplified_vehicle_models.planning.yaw_aware_line._corner_complexes',return_value=[(0.,track.lap_length)]), patch('simplified_vehicle_models.planning.yaw_aware_line.minimize',side_effect=self.minimizing()):
            result=optimize_yaw_aware_line(track,None,capability(),settings=settings)
        self.assertEqual(result.diagnostics['completed_full_passes'],2, result.diagnostics['outer_passes'])
        self.assertEqual(result.status,'converged')
        self.assertEqual(result.diagnostics['requested_maximum_outer_passes'],7)
        self.assertEqual(result.diagnostics['effective_maximum_outer_passes'],3)

    def test_no_active_windows_do_not_count_as_completed(self):
        track,settings=self.run_plan()
        with patch('simplified_vehicle_models.planning.yaw_aware_line._corner_complexes',return_value=[]):
            result=optimize_yaw_aware_line(track,None,capability(),settings=settings)
        self.assertEqual(result.diagnostics['completed_full_passes'],0)
        self.assertFalse(result.diagnostics['converged'])

    def test_optional_poll_and_incomplete_poll_are_explicit(self):
        track,settings=self.run_plan(directional_poll=True,maximum_profile_evaluations=64)
        with patch('simplified_vehicle_models.planning.yaw_aware_line._corner_complexes',return_value=[(0.,track.lap_length)]), patch('simplified_vehicle_models.planning.yaw_aware_line.minimize',side_effect=self.minimizing()):
            result=optimize_yaw_aware_line(track,None,capability(),settings=settings)
        self.assertIn(result.diagnostics['directional_poll_status'],('coordinate_poll_stopping_at_0.01_m','improving_direction_found'))
        self.assertGreaterEqual(result.diagnostics['poll_best_improvement_s'],0.)
        track,settings=self.run_plan(directional_poll=True,maximum_profile_evaluations=5)
        with patch('simplified_vehicle_models.planning.yaw_aware_line._corner_complexes',return_value=[(0.,track.lap_length)]), patch('simplified_vehicle_models.planning.yaw_aware_line.minimize',side_effect=self.minimizing()):
            result=optimize_yaw_aware_line(track,None,capability(),settings=settings)
        self.assertEqual(result.diagnostics['directional_poll_status'],'inconclusive')
        self.assertFalse(result.diagnostics['converged'])


if __name__=='__main__': unittest.main()
