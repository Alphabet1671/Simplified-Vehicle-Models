import unittest
from unittest import mock
import numpy as np
from simplified_vehicle_models.formulations.yaw_capability import YawCapabilityMap
from simplified_vehicle_models.solvers.yaw_aware_profile import YawPath
from simplified_vehicle_models.solvers.yaw_profile_contracts import CoupledProfileSettings, YawProfileSolveError
from simplified_vehicle_models.solvers.coupled_yaw_profile import solve_coupled_profile
from simplified_vehicle_models.solvers.yaw_profile_validation import validate_profile


def make_map():
    v=np.linspace(0,8,17); ax=np.linspace(-2,2,9); ay=np.linspace(-2,2,17); shape=(len(v),len(ax),len(ay))
    mask=np.ones(shape,bool)
    for i,speed in enumerate(v): mask[i,:,np.abs(ay)>max(.25,1.25-.12*speed)]=False
    return YawCapabilityMap(v,ax,ay,np.full(shape,1.e6),np.full(shape,1.e6),mask,1.,"physical_no_tv_v1","coupled-test")


class CoupledProfileTests(unittest.TestCase):
    def test_finds_and_independently_validates_task1_witness(self):
        distance=np.arange(0.,.5001,.05); k=np.linspace(.01,.08,len(distance)); path=YawPath(distance,k,np.gradient(k,distance),False)
        result=solve_coupled_profile(path,make_map(),boundary="standing",maximum_speed=8.,settings=CoupledProfileSettings(max_refinement_rounds=4))
        self.assertEqual(result["q"][0],0.)
        validation=validate_profile(path,make_map(),np.sqrt(result["q"]),maximum_speed=8.,boundary="standing",settings=CoupledProfileSettings())
        self.assertTrue(validation.valid, (result["q"], validation))
        self.assertEqual(result["diagnostics"]["status"],"feasible_sampled")

    def test_open_profile_budget_failure_is_typed(self):
        distance=np.array([0.,.05,.1]); path=YawPath(distance,np.zeros(3),np.zeros(3),False)
        settings=CoupledProfileSettings(q_seed_count=33,max_candidates_per_station=2)
        with self.assertRaises(YawProfileSolveError) as caught:
            solve_coupled_profile(path,make_map(),boundary="standing",maximum_speed=8.,settings=settings)
        self.assertEqual(caught.exception.status,"budget_exceeded")

    def test_closed_rotation_uses_tightest_domain_and_restores_original_order(self):
        d=np.array([0.,.1,.2,.3,.4]); k=np.array([0.,.1,.5,.2,0.]); path=YawPath(d,k,np.zeros_like(k),True)
        v=np.linspace(0,8,17); ax=np.linspace(-2,2,9); ay=np.linspace(-2,2,17); shape=(len(v),len(ax),len(ay))
        cap=YawCapabilityMap(v,ax,ay,np.full(shape,100.),np.full(shape,100.),np.ones(shape,bool),1.,"physical_no_tv_v1","closed-rotation")
        result=solve_coupled_profile(path,cap,boundary="flying",maximum_speed=8.,settings=CoupledProfileSettings(max_refinement_rounds=2,max_seam_anchors=8))
        diag=result["diagnostics"]
        self.assertEqual(diag["closed_seam_rotation"],2)
        self.assertEqual(len(diag["tested_seam_anchors"]),len(set(diag["tested_seam_anchors"])))
        self.assertLessEqual(len(diag["tested_seam_anchors"]),32)
        self.assertEqual(result["q"].shape,(len(d),))
        self.assertAlmostEqual(result["q"][0],result["q"][-1],places=12)
        self.assertAlmostEqual(diag["validation"].seam_speed_error,0.,places=12)

    def test_strict_validator_rejection_never_returns_success(self):
        distance=np.arange(0.,.2001,.05); k=np.zeros_like(distance)
        path=YawPath(distance,k,k,False)
        rejected=mock.Mock(valid=False, recomputed_time=None)
        with mock.patch("simplified_vehicle_models.solvers.coupled_yaw_profile.validate_profile", return_value=rejected):
            with self.assertRaises(YawProfileSolveError) as caught:
                solve_coupled_profile(path,make_map(),boundary="standing",maximum_speed=8.,
                    settings=CoupledProfileSettings(max_refinement_rounds=2))
        self.assertNotEqual(caught.exception.status,"feasible_sampled")
        self.assertEqual(caught.exception.status,"strict_validation_failed")

    def test_repeat_run_is_deterministic_and_incumbent_time_does_not_increase(self):
        distance=np.arange(0.,.5001,.05); k=np.linspace(.01,.08,len(distance))
        path=YawPath(distance,k,np.gradient(k,distance),False)
        settings=CoupledProfileSettings(max_refinement_rounds=4)
        real_validate=validate_profile
        observed=[]
        def record_validation(*args,**kwargs):
            result=real_validate(*args,**kwargs)
            if result.valid:
                observed.append(float(result.recomputed_time))
            return result
        with mock.patch("simplified_vehicle_models.solvers.coupled_yaw_profile.validate_profile", side_effect=record_validation):
            first=solve_coupled_profile(path,make_map(),boundary="standing",maximum_speed=8.,settings=settings)
            first_times=tuple(observed)
            observed.clear()
            second=solve_coupled_profile(path,make_map(),boundary="standing",maximum_speed=8.,settings=settings)
            second_times=tuple(observed)
        np.testing.assert_array_equal(first["q"],second["q"])
        self.assertEqual(first["time_seconds"],second["time_seconds"])
        self.assertEqual(first["diagnostics"]["tested_seam_anchors"],second["diagnostics"]["tested_seam_anchors"])
        self.assertTrue(first_times)
        self.assertEqual(first_times,second_times)
        self.assertTrue(all(b <= a*(1.+1e-10) for a,b in zip(first_times,first_times[1:])),first_times)

    def test_closed_periodic_profile(self):
        d=np.array([0.,.1,.2,.3]); k=np.zeros(4); path=YawPath(d,k,k,True)
        v=np.linspace(0,8,17); ax=np.linspace(-2,2,9); ay=np.linspace(-2,2,17); shape=(len(v),len(ax),len(ay))
        cap=YawCapabilityMap(v,ax,ay,np.full(shape,100.),np.full(shape,100.),np.ones(shape,bool),1.,"physical_no_tv_v1","closed")
        result=solve_coupled_profile(path,cap,boundary="flying",maximum_speed=8.,settings=CoupledProfileSettings(max_refinement_rounds=2))
        self.assertAlmostEqual(result["q"][0],result["q"][-1],places=12)
        self.assertGreater(result["time_seconds"],0)
