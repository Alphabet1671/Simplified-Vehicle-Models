import unittest
from unittest.mock import patch
import numpy as np

from simplified_vehicle_models.planning.yaw_aware_line import YawLineSettings, optimize_yaw_aware_line
from simplified_vehicle_models.solvers.yaw_aware_profile import YawAwareProfile
from tests.test_yaw_aware_line import capability, circle_track


def profile_for(path, time_value=10.):
    speed=np.full(len(path.distance),5.)
    elapsed=np.r_[0.,np.cumsum(path.segment_lengths/5.)]
    return YawAwareProfile(path.distance,speed,np.zeros_like(speed),elapsed,time_value,True,1,0.,
                           {"resolution_converged":True})


class YawLineBudgetSettingsTests(unittest.TestCase):
    def test_appended_global_budget_defaults_and_seed_blends(self):
        settings=YawLineSettings(seed_blends=(.1,.9))
        self.assertEqual(settings.maximum_profile_evaluations,24)
        self.assertEqual(settings.maximum_wall_seconds,900.)
        self.assertEqual(settings.seed_blends,(.1,.9))

    def test_seed_blends_and_global_budgets_validate(self):
        with self.assertRaises(ValueError): YawLineSettings(seed_blends=(1.1,))
        with self.assertRaises(ValueError): YawLineSettings(maximum_profile_evaluations=0)
        with self.assertRaises(ValueError): YawLineSettings(maximum_wall_seconds=0.)

    def test_global_profile_budget_keeps_validated_incumbent(self):
        track=circle_track(); calls=[]
        def fake_profile(path, *_args, **_kwargs):
            calls.append(path.distance.copy())
            return profile_for(path,100.-len(calls))
        settings=YawLineSettings(control_spacing=160.,grid_spacing=100.,dense_subdivisions=1,
            optimizer_max_iterations=1,maximum_outer_passes=1,maximum_speed=20.,boundary="flying",
            seed_blends=(.2,.8),maximum_profile_evaluations=2)
        with patch("simplified_vehicle_models.planning.yaw_aware_line.yaw_aware_profile",fake_profile):
            result=optimize_yaw_aware_line(track,None,capability(),settings=settings)
        self.assertEqual(len(calls),2)
        self.assertEqual(result.status,"budget_exceeded")
        self.assertEqual(result.diagnostics["evaluated_profile_count"],2)
        self.assertEqual(result.diagnostics["search_stop_reason"],"profile_evaluations")

    def test_exact_seed_coefficients_are_cached(self):
        track=circle_track(); calls=[]
        def fake_profile(path, *_args, **_kwargs):
            calls.append(1)
            return profile_for(path,10.)
        settings=YawLineSettings(control_spacing=160.,grid_spacing=100.,dense_subdivisions=1,
            optimizer_max_iterations=1,maximum_outer_passes=1,maximum_speed=20.,boundary="flying",
            seed_blends=(0.,),maximum_profile_evaluations=8)
        with patch("simplified_vehicle_models.planning.yaw_aware_line.yaw_aware_profile",fake_profile):
            result=optimize_yaw_aware_line(track,None,capability(),settings=settings)
        self.assertLess(len(calls),settings.maximum_profile_evaluations)
        self.assertGreaterEqual(result.diagnostics["cache_hits"],1)

    def test_failed_profile_is_cached(self):
        track=circle_track(); calls=[]
        def failed(path, *_args, **_kwargs):
            calls.append(1); raise ValueError("map feasible transition")
        settings=YawLineSettings(control_spacing=160.,grid_spacing=100.,dense_subdivisions=1,
            optimizer_max_iterations=1,maximum_outer_passes=1,maximum_speed=20.,boundary="flying",
            seed_blends=(0.,),maximum_profile_evaluations=8)
        with patch("simplified_vehicle_models.planning.yaw_aware_line._qp_seed",side_effect=lambda spline,*args: (np.zeros(spline.control_count),True,1,0.)), \
             patch("simplified_vehicle_models.planning.yaw_aware_line.yaw_aware_profile",failed):
            with self.assertRaises(Exception) as caught:
                optimize_yaw_aware_line(track,None,capability(),settings=settings)
        self.assertEqual(len(calls),1)
        self.assertTrue(any(row["reason"]=="capability_domain" for row in caught.exception.diagnostics))

    def test_wall_budget_is_checked_before_qp(self):
        track=circle_track()
        settings=YawLineSettings(maximum_wall_seconds=1.e-6,seed_blends=(.2,))
        with patch("simplified_vehicle_models.planning.yaw_aware_line._qp_seed") as qp:
            with self.assertRaises(Exception):
                optimize_yaw_aware_line(track,None,capability(),settings=settings)
        qp.assert_not_called()


if __name__ == '__main__': unittest.main()
