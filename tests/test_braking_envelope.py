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

    def test_closed_nonuniform_seam_and_rotation(self):
        # Unique ring nodes at 0, 2, 7; edge lengths 2, 5, 3 (seam).
        x = np.array([0., 2., 7., 10.])
        caps = np.array([20., 20., 3., 20.])
        b = np.array([4., 4., 4.])
        q, bind, _ = propagate_braking_caps(x, caps, b, is_closed=True)
        np.testing.assert_allclose(q[[0, -1]], [q[0], q[0]])
        self.assertLess(q[0], caps[0])
        self.assertEqual(bind[0], 2)
        # Reindex same ring at node 2; remap its nonuniform edge lengths.
        xr = np.array([0., 3., 5., 10.])
        cr = np.array([3., 20., 20., 3.])
        br = np.array([4., 4., 4.])
        qr, br_idx, _ = propagate_braking_caps(xr, cr, br, is_closed=True)
        np.testing.assert_allclose(qr[:3], [q[2], q[0], q[1]], atol=1e-8)

    def test_closed_zero_authority_global_minimum_and_seam_sample(self):
        x = np.array([0., 2., 7., 10.])
        caps = np.array([20., 9., 8., 20.])
        entry, binding, _ = propagate_braking_caps(x, caps, np.zeros(3), is_closed=True)
        np.testing.assert_allclose(entry, [8., 8., 8., 8.])

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

    def test_builder_lateral_margin_scales_radius_before_oracle(self):
        vehicle = PointMassVehicleParameters.__new__(PointMassVehicleParameters)
        def corner_oracle(vehicle, radius, **kwargs):
            return radius * radius
        with patch("simplified_vehicle_models.formulations.point_mass.maximum_corner_speed", side_effect=corner_oracle) as oracle, \
             patch("simplified_vehicle_models.formulations.point_mass.maximum_deceleration_limit", return_value=2.):
            env = build_braking_envelope([0., 10.], [.2, .2], vehicle, is_closed=False,
                                         lateral_margin=.5, maximum_speed=100., speed_query_points=2)
        self.assertAlmostEqual(oracle.call_args_list[0].args[1], .5/.2)
        self.assertAlmostEqual(env.corner_speed[0], 6.25)

    def test_builder_rejects_margins_above_one_and_invalid_types(self):
        vehicle = PointMassVehicleParameters.__new__(PointMassVehicleParameters)
        with patch("simplified_vehicle_models.formulations.point_mass.maximum_corner_speed", return_value=10.), patch("simplified_vehicle_models.formulations.point_mass.maximum_deceleration_limit", return_value=2.):
            for kwargs in ({"lateral_margin":1.01}, {"deceleration_margin":1.01}, {"lateral_margin":True}, {"deceleration_margin":None}, {"lateral_margin":float("nan")}):
                with self.subTest(kwargs=kwargs), self.assertRaises((TypeError, ValueError)):
                    build_braking_envelope([0.,1.],[0.,0.],vehicle,is_closed=False,**kwargs)

    def test_builder_bounds_queries_and_invalid_authority_fails(self):
        vehicle = PointMassVehicleParameters.__new__(PointMassVehicleParameters)
        with patch("simplified_vehicle_models.formulations.point_mass.maximum_corner_speed", return_value=10.), \
             patch("simplified_vehicle_models.formulations.point_mass.maximum_deceleration_limit", return_value=4.) as oracle:
            env = build_braking_envelope(
                [0., 10., 20.], [0., .1, 0.], vehicle, is_closed=False,
                speed_query_points=3, deceleration_margin=.5,
            )
        self.assertAlmostEqual(oracle.call_args_list[0].args[1], 0.)
        self.assertAlmostEqual(oracle.call_args_list[0].args[2], .1)
        self.assertEqual(oracle.call_count, 3)
        self.assertEqual(env.provenance["oracle_calls"], 3)
        self.assertEqual(env.provenance["unique_oracle_domains"], 1)
        np.testing.assert_allclose(env.deceleration, [2., 2.])
        oracle.assert_called()
        with patch("simplified_vehicle_models.formulations.point_mass.maximum_corner_speed", return_value=10.), \
             patch("simplified_vehicle_models.formulations.point_mass.maximum_deceleration_limit", return_value=float("nan")):
            with self.assertRaises(ValueError):
                build_braking_envelope([0., 10.], [0., 0.], vehicle, is_closed=False, speed_query_points=2)

    def test_constructor_rejects_invalid_topology_and_bounds(self):
        base = [np.array([0., 1., 2.]), np.zeros(3), np.array([5., 5., 5.]),
                np.array([4., 4., 4.]), np.ones(2), np.array([0, 0, 0])]
        bad_cases = [
            (np.array([0., 1., 1.]), *base[1:]),
            (*base[:3], np.array([6., 4., 4.]), *base[4:]),
            (*base[:4], np.array([-1., 1.]), base[5]),
            (*base[:5], np.array([0, 0, 3])),
        ]
        for arrays in bad_cases:
            with self.subTest(arrays=arrays):
                with self.assertRaises((TypeError, ValueError)):
                    BrakingEnvelope(*arrays, False, 10., {})

    def test_binding_distance_to_next_node_is_not_zero(self):
        entry, binding, _ = propagate_braking_caps(
            [0., 10., 20.], [30., 10., 10.], [5., 5.], is_closed=False
        )
        self.assertEqual(binding[0], 1)
        env = BrakingEnvelope(
            np.array([0., 10., 20.]), np.zeros(3), np.array([30., 10., 10.]),
            np.array([np.sqrt(300.), 10., 10.]), np.array([1., 1.]),
            np.array([1, 1, 2]), False, 60., {},
        )
        self.assertEqual(env.sample(0.).binding_distance, 10.)
        self.assertEqual(env.sample(10.).binding_distance, 0.)

    def test_closed_endpoint_consistency_is_enforced(self):
        x=np.array([0., 2., 5., 10.]); c=np.array([5., 3., 4., 5.]); e=c.copy(); k=np.array([.1,.2,.3,.1])
        base=[x,k,c,e,np.ones(3),np.array([0,1,2,0])]
        for index, changed in ((1,np.array([.1,.2,.3,.2])), (2,np.array([5.,3.,4.,4.])), (3,np.array([5.,3.,4.,4.]))):
            args=list(base); args[index]=changed
            with self.subTest(index=index), self.assertRaises(ValueError):
                BrakingEnvelope(*args, True, 10., {})

    def test_closed_local_binding_distance_zero_and_local_cap(self):
        env = BrakingEnvelope(
            np.array([0., 2., 7., 10.]), np.zeros(4), np.array([5., 3., 4., 5.]),
            np.array([5., 3., 4., 5.]), np.ones(3), np.array([0, 1, 2, 0]),
            True, 10., {"segment_speed_domain_max": (3., 3., 4.)},
        )
        sample = env.sample(2.1)
        self.assertEqual(sample.binding_distance, 0.)
        self.assertEqual(sample.corner_speed, 3.)
        self.assertEqual(sample.speed_domain_max, 3.)
        wrapped = env.sample(-.1)
        self.assertGreaterEqual(wrapped.binding_distance, 0.)

    def test_invalid_inputs_rejected(self):
        for args in [
            ([0, 0], [1, 1], [1]), ([0, 1], [1], [1]),
            ([0, 1], [1, 1], [-1]), ([0, float('nan')], [1, 1], [1]),
            ([0, 1], [1e308, 1e308], [1]),
        ]:
            with self.subTest(args=args):
                with self.assertRaises((TypeError, ValueError)):
                    propagate_braking_caps(np.array(args[0]), np.array(args[1]), np.array(args[2]), is_closed=False)


if __name__ == '__main__':
    unittest.main()
