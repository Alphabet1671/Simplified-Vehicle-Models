import unittest
from types import SimpleNamespace

import numpy as np

from simplified_vehicle_models.solvers.vertical_capability import constrain_corner_speeds


class VerticalCapabilityTests(unittest.TestCase):
    def test_bounds_sign_cache_and_nonmutation(self):
        d = np.array([0., 1., 2., 3.]); k = np.array([.1, -.1, 0., .2]); caps = np.array([30., 25., 20., 30.])
        original = caps.copy(); calls = []
        def oracle(vehicle, speed, ay):
            key = (speed, ay)
            calls.append(key)
            if ay < 0 or abs(ay) > 20:
                raise RuntimeError("unresolved")
            return SimpleNamespace(residual_inf=1e-11)
        out = constrain_corner_speeds(object(), d, k, caps, max_queries=20, samples_per_station=4, solver=oracle)
        self.assertTrue(np.all(out.caps <= caps))
        np.testing.assert_array_equal(caps, original)
        self.assertEqual(out.query_count, len(calls))
        self.assertLessEqual(out.query_count, 20)
        self.assertTrue(any(ay < 0 for _, ay in calls))
        self.assertTrue(any(s.status == "unresolved" for s in out.samples))
        self.assertTrue(any(s.status == "solved" for s in out.samples))
        self.assertEqual(len({(s.speed, s.lateral_acceleration) for s in out.samples}), len(calls))
        self.assertTrue(all("maximum" in out.caveat for _ in [0]))
        self.assertEqual(out.reasons[0], "restricted-to-tested-prefix")
        self.assertEqual(out.reasons[1], "unresolved-retained")

    def test_zero_budget_is_explicitly_not_queried(self):
        result = constrain_corner_speeds(None, [0., 1.], [.1, .2], [10., 10.], max_queries=0)
        self.assertEqual(result.reasons, ("not-queried", "not-queried"))
        self.assertEqual(result.query_count, 0)

    def test_query_budget_and_standing_domain(self):
        calls = []
        def oracle(vehicle, speed, ay):
            calls.append(speed)
            return SimpleNamespace(residual_inf=0.)
        r = constrain_corner_speeds(None, [0., 1., 2.], [.1, .1, .1], [3., 4., 4.],
                                    max_queries=3, samples_per_station=3, solver=oracle)
        self.assertEqual(len(calls), 1)  # identical speed/ay is cached across stations
        self.assertTrue(all(s >= 3. for s in calls))
        self.assertEqual(r.caps[0], 3.)

    def test_single_sample_tests_planar_cap_and_programmer_errors_escape(self):
        calls = []
        def oracle(vehicle, speed, ay):
            calls.append((speed, ay))
            return SimpleNamespace(residual_inf=1e-11)
        result = constrain_corner_speeds(None, [0., 1.], [.1, -.1], [15., 12.],
                                         max_queries=2, samples_per_station=1, solver=oracle)
        self.assertEqual({speed for speed, _ in calls}, {15., 12.})
        self.assertEqual(result.caps.tolist(), [15., 12.])
        self.assertTrue(all(reason == "planar-cap-tested" for reason in result.reasons))
        def broken(vehicle, speed, ay):
            raise TypeError("programmer bug")
        with self.assertRaises(TypeError):
            constrain_corner_speeds(None, [0.], [.1], [10.], max_queries=1,
                                    samples_per_station=1, solver=broken)

    def test_closed_seam_and_all_unresolved_retained(self):
        def reject(vehicle, speed, ay):
            raise RuntimeError("unresolved")
        result = constrain_corner_speeds(None, [0., 1., 2.], [.1, .2, .1], [10., 9., 10.],
                                         max_queries=1, samples_per_station=1, is_closed=True,
                                         solver=reject)
        self.assertEqual(result.caps[0], result.caps[-1])
        self.assertIn("unresolved-retained", result.reasons)
        self.assertTrue(all(s.status == "unresolved" for s in result.samples))

    def test_actual_vertical_vehicle_small_case(self):
        # Exercise real compiled 14-DoF model without a lap-scale workload.
        from simplified_vehicle_models.compiler.compile import compile_formulation
        formulation = compile_formulation("models/formulations/iem24_murphy_real_14dof_simple.xml")
        vehicle = formulation.model
        from simplified_vehicle_models.solvers.vertical_equilibrium import _solve_steady_corner_detail
        # Identity is intentionally retained; the solver must not mutate the compiled model.
        before = vehicle.planar.total_mass
        r = constrain_corner_speeds(vehicle, [0., 1.], [.01, .01], [10., 10.], max_queries=2, samples_per_station=1)
        self.assertEqual(r.query_count, 1)  # identical sampled query is cached
        self.assertTrue(np.all(r.caps <= 10.))
        self.assertEqual(vehicle.planar.total_mass, before)


if __name__ == "__main__":
    unittest.main()
