import unittest
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.parameters.vehicle import PointMassVehicleParameters
from simplified_vehicle_models.solvers.braking_envelope import (
    BrakingEnvelope, build_braking_envelope, propagate_braking_caps,
)


class PropagationTests(unittest.TestCase):
    def test_open_analytic_envelope_and_piecewise_sample(self):
        entry, binding, iterations = propagate_braking_caps(
            np.array([0., 10., 20.]), np.array([30., 30., 10.]),
            np.array([5., 5.]), is_closed=False,
        )
        np.testing.assert_allclose(entry**2, [300., 200., 100.])
        np.testing.assert_array_equal(binding, [2, 2, 2])
        self.assertGreater(iterations, 0)
        self.assertAlmostEqual(entry[1]**2 - entry[2]**2, 100.)

    def test_nonuniform_grid_respects_distance(self):
        entry, _, _ = propagate_braking_caps(
            np.array([0., 2., 10.]), np.array([20., 20., 0.]),
            np.array([2., 2.]), is_closed=False,
        )
        np.testing.assert_allclose(entry**2, [40., 32., 0.])

    def test_whole_corner_cap_and_zero_authority(self):
        entry, bind, _ = propagate_braking_caps(
            np.array([0., 4., 9.]), np.array([7., 7., 7.]),
            np.zeros(2), is_closed=False,
        )
        np.testing.assert_allclose(entry, [7., 7., 7.])
        np.testing.assert_array_equal(bind, [0, 1, 2])

    def test_envelope_sample_and_readonly_arrays(self):
        env = BrakingEnvelope(
            np.array([0., 10., 20.]), np.zeros(3), np.array([30.,30.,10.]),
            np.array([np.sqrt(300),np.sqrt(200),10.]), np.array([5.,5.]),
            np.array([2,2,2]), False, 60., {"source":"test"},
        )
        result = env.sample(15.)
        self.assertAlmostEqual(result.squared_speed, 150.)
        self.assertAlmostEqual(result.squared_speed_slope, -10.)
        self.assertEqual(result.binding_index, 2)
        self.assertEqual(env.sample(20.).squared_speed_slope, 0.)
        with self.assertRaises(ValueError):
            env.distance[0] = 3.

    def test_builder_bounds_queries_and_invalid_authority_fails(self):
        vehicle = PointMassVehicleParameters.__new__(PointMassVehicleParameters)
        with patch("simplified_vehicle_models.formulations.point_mass.maximum_corner_speed", return_value=10.), \
             patch("simplified_vehicle_models.formulations.point_mass.maximum_deceleration_limit", return_value=4.) as oracle:
            env = build_braking_envelope(
                [0., 10., 20.], [0., .1, 0.], vehicle, is_closed=False,
                speed_query_points=3, deceleration_margin=.5,
            )
        self.assertEqual(oracle.call_count, 3)
        self.assertEqual(env.provenance["oracle_calls"], 3)
        self.assertEqual(env.provenance["unique_oracle_domains"], 1)
        np.testing.assert_allclose(env.deceleration, [2., 2.])
        oracle.assert_called()
        with patch("simplified_vehicle_models.formulations.point_mass.maximum_corner_speed", return_value=10.), \
             patch("simplified_vehicle_models.formulations.point_mass.maximum_deceleration_limit", return_value=float("nan")):
            with self.assertRaises(ValueError):
                build_braking_envelope([0., 10.], [0., 0.], vehicle, is_closed=False, speed_query_points=2)

    def test_invalid_inputs_rejected(self):
        for args in [
            ([0, 0], [1, 1], [1]), ([0, 1], [1], [1]),
            ([0, 1], [1, 1], [-1]), ([0, float('nan')], [1, 1], [1]),
        ]:
            with self.subTest(args=args):
                with self.assertRaises((TypeError, ValueError)):
                    propagate_braking_caps(np.array(args[0]), np.array(args[1]), np.array(args[2]), is_closed=False)


if __name__ == '__main__':
    unittest.main()
