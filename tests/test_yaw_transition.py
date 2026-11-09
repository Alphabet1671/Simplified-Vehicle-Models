import unittest
from types import SimpleNamespace

import numpy as np

from simplified_vehicle_models.solvers.yaw_transition import yaw_transition_speed_caps


class YawTransitionTests(unittest.TestCase):
    def setUp(self):
        self.vehicle = SimpleNamespace(total_mass=1000., wheelbase=2.5,
                                       front_mass_fraction=.6, yaw_inertia=1200.)

    def test_force_budget_analytic_constant_curvature(self):
        calls = []
        def authority(v, ay):
            calls.append((v, ay))
            return 10000., 10000.
        x = np.array([0., 1., 3.])
        k = np.array([.1, .1, .1])
        caps = np.array([50., 40., 30.])
        result = yaw_transition_speed_caps(x, k, self.vehicle, caps, force_authority=authority)
        np.testing.assert_array_equal(result.caps, caps)
        self.assertEqual(calls, [])
        self.assertFalse(result.caps.flags.writeable)

    def test_nonuniform_curvature_slope_limits_and_never_raises(self):
        x = np.array([0., 1., 3., 4.])
        k = np.array([0., .1, .1, -.1])
        caps = np.full(4, 20.)
        def authority(v, ay):
            return 1000., 1000.
        result = yaw_transition_speed_caps(x, k, self.vehicle, caps, force_authority=authority)
        self.assertAlmostEqual(result.curvature_gradient[0], .1)
        self.assertAlmostEqual(result.curvature_gradient[1], .1)
        self.assertAlmostEqual(result.curvature_gradient[2], .2)
        self.assertAlmostEqual(result.curvature_gradient[3], .2)
        self.assertTrue(np.all(result.caps <= caps))
        self.assertLess(result.caps[-1], result.caps[0])

    def test_closed_seam_uses_worst_adjacent_signed_secants(self):
        x = np.array([0., 1., 2., 3., 4.])
        k = np.array([0., .1, -.1, -.1, 0.])
        caps = np.full(5, 20.)
        result = yaw_transition_speed_caps(x, k, self.vehicle, caps, is_closed=True,
                                           force_authority=lambda v, ay: (500., 500.))
        self.assertEqual(result.curvature_gradient[0], .1)
        self.assertEqual(result.curvature_gradient[-1], .1)
        self.assertEqual(result.caps[0], result.caps[-1])

    def test_standing_nodes_remain_zero_and_metadata_warns(self):
        result = yaw_transition_speed_caps([0., 1., 2.], [0., .2, .2], self.vehicle,
                                           [0., 15., 15.], force_authority=lambda v, ay:(1e6,1e6))
        self.assertEqual(result.caps[0], 0.)
        self.assertIn("not a physical guarantee", result.provenance["caveat"])

    def test_invalid_inputs_and_authority_fail_loudly(self):
        with self.assertRaises(ValueError):
            yaw_transition_speed_caps([0., 0.], [0., 1.], self.vehicle, [2., 2.],
                                      force_authority=lambda v, ay:(1.,1.))
        with self.assertRaises(ValueError):
            yaw_transition_speed_caps([0., 1.], [0., 1.], self.vehicle, [-1., 2.],
                                      force_authority=lambda v, ay:(1.,1.))
        def broken(v, ay):
            raise RuntimeError("oracle failure")
        with self.assertRaises(RuntimeError):
            yaw_transition_speed_caps([0., 1.], [0., 1.], self.vehicle, [2., 2.], force_authority=broken)

    def test_nonmonotone_authority_uses_first_sampled_crossing(self):
        def authority(speed, ay):
            return (0., 0.) if 3. <= speed <= 5. else (1e6, 1e6)
        result=yaw_transition_speed_caps([0.,1.],[0.,.1],self.vehicle,[10.,10.],force_authority=authority)
        self.assertLessEqual(float(result.caps.max()),3.)
        self.assertTrue(result.provenance['sampled_nonmonotone_station_indices'])

    def test_rollover_loads_never_reach_tire_authority(self):
        from unittest.mock import patch
        from simplified_vehicle_models.compiler.compile import compile_formulation
        car=compile_formulation('models/formulations/iem24_murphy_real_planar_7dof.xml').model
        with patch('simplified_vehicle_models.formulations.planar_7dof.evaluate_normal_loads',return_value=np.array([-1.,1.,1.,1.])), patch('simplified_vehicle_models.tires.unitire_simple.evaluate_unitire') as tire:
            result=yaw_transition_speed_caps([0.,1.],[0.,.1],car,[5.,5.])
        tire.assert_not_called()
        np.testing.assert_array_equal(result.caps,[0.,0.])

    def test_actual_compiled_car_small_grid(self):
        from simplified_vehicle_models.compiler.compile import compile_formulation
        compiled = compile_formulation("models/formulations/iem24_murphy_real_planar_7dof.xml")
        car = compiled.model
        result = yaw_transition_speed_caps([0., 1., 2.], [0., .01, 0.], car,
                                           [12., 12., 12.])
        self.assertLessEqual(float(np.max(result.caps)), 12.)


if __name__ == "__main__":
    unittest.main()
