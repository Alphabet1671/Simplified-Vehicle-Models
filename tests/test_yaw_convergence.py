from dataclasses import replace
import unittest
import numpy as np
from simplified_vehicle_models.planning.yaw_convergence import ConvergenceSample,compare_samples,convergence_study


def sample(label='coarse',geometry=1.,control=20.,**kw):
    values=dict(label=label,geometry_spacing_m=geometry,control_spacing_m=control,lap_time_s=10.,
        reference_distance=np.array([0.,5.,10.]),speed=np.array([5.,6.,5.]),offset=np.zeros(3),
        profile_valid=True,profile_resolution_converged=True,optimizer_converged=True)
    values.update(kw);return ConvergenceSample(**values)


class ConvergenceTests(unittest.TestCase):
    def test_same_grid_repeats_cannot_prove_mesh_convergence(self):
        result=convergence_study([sample(),sample('repeat')],dimension='geometry')
        self.assertEqual(result['status'],'inconclusive')
        self.assertTrue(result['optimizer_converged'])

    def test_true_refinement_compares_on_common_reference_station(self):
        fine=sample('fine',geometry=.5,reference_distance=np.array([0.,2.5,5.,7.5,10.]),
                    speed=np.array([5.,5.5,6.,5.5,5.]),offset=np.zeros(5))
        result=compare_samples(sample(),fine,dimension='geometry')
        self.assertEqual(result['status'],'converged')
        self.assertEqual(result['speed_max_difference_mps'],0.)
        self.assertEqual(result['thresholds']['geometric_time_relative'],.005)
        self.assertEqual(result['thresholds']['geometric_speed_max_mps'],.1)
        self.assertEqual(result['thresholds']['geometric_offset_max_m'],.05)
        self.assertEqual(result['thresholds']['profile_candidate_time_relative'],.001)
        self.assertEqual(result['optimizer_status'],['converged','converged'])
        self.assertIn('piecewise-linear',result['interpolation_rule'])

    def test_budget_profile_or_optimizer_failure_remains_inconclusive(self):
        for changes in ({'budget_reason':'wall_deadline'},{'profile_valid':False},
                        {'profile_resolution_converged':False},{'optimizer_converged':False}):
            with self.subTest(changes=changes):
                result=compare_samples(sample(),sample('fine',geometry=.5,**changes),dimension='geometry')
                self.assertEqual(result['status'],'inconclusive')

    def test_mismatched_control_resolution_does_not_prove_mesh_convergence(self):
        self.assertEqual(compare_samples(sample(),sample('fine',geometry=.5,control=10.),dimension='geometry')['status'],'inconclusive')

    def test_control_convergence_distinct_from_mesh(self):
        rows=[sample(),sample('fine',control=10.)]
        control=convergence_study(rows,dimension='control')
        self.assertEqual(control['status'],'converged')
        self.assertEqual(control['finest_pair_status'],'converged')
        self.assertEqual(control['convergence_claim'],'finest tested pair meets criteria')
        self.assertEqual(convergence_study(rows,dimension='geometry')['status'],'inconclusive')

    def test_faster_time_not_enough_when_speed_or_line_changes(self):
        fine=sample('fine',geometry=.5,speed=np.array([5.,7.,5.]))
        self.assertEqual(compare_samples(sample(),fine,dimension='geometry')['status'],'not_converged')

    def test_invalid_nonfinite_arrays_rejected_and_data_readonly(self):
        with self.assertRaises(ValueError):sample(speed=np.array([5.,float('nan'),5.]))
        with self.assertRaises(ValueError):sample(lap_time_s=float('inf'))
        self.assertFalse(sample().speed.flags.writeable)

    def test_single_resolution_inconclusive(self):
        self.assertEqual(convergence_study([sample()],dimension='geometry')['status'],'inconclusive')
