"""Soundness tests for the sharpened load-map enclosure and the adaptive certificate."""

from pathlib import Path
import unittest

import numpy as np
from scipy.integrate import solve_ivp

import simplified_vehicle_models.formulations.planar_7dof as planar
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers import load_interval as li
from simplified_vehicle_models.solvers.dense_domain import (
    dense_polynomial_coefficients,
    dense_subspan_bounds,
    evaluate_dense_polynomials,
)
from tests.test_load_interval import FORMULATION, _point_map


class SubspanBoundTests(unittest.TestCase):
    def test_subspan_bounds_contain_the_polynomial_and_fail_closed(self):
        solution = solve_ivp(lambda t, y: [y[1], -y[0] + 0.3 * np.sin(3.0 * t)], (0.0, 6.0),
                             [1.0, 0.0], method="BDF", rtol=1e-6, atol=1e-9, dense_output=True)
        poly = dense_polynomial_coefficients(solution.sol)
        random = np.random.default_rng(0)
        for _ in range(300):
            i = int(random.integers(poly.n_intervals))
            f0 = float(random.uniform(0.0, 0.9))
            f1 = float(random.uniform(f0, 1.0))
            bounds = dense_subspan_bounds(poly, [i], [f0], [f1])[0]
            times = poly.times[i] + (poly.times[i + 1] - poly.times[i]) * np.linspace(f0, f1, 120)
            values = evaluate_dense_polynomials(poly, i, times)
            self.assertTrue(np.all(values.min(axis=1) >= bounds[:, 0]))
            self.assertTrue(np.all(values.max(axis=1) <= bounds[:, 1]))
        bad = dense_subspan_bounds(poly, [0], [0.5], [0.2])
        self.assertTrue(np.all(np.isinf(bad)))
        with self.assertRaises(IndexError):
            dense_subspan_bounds(poly, [poly.n_intervals], [0.0], [1.0])


class SharpenedEnclosureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(FORMULATION).model

    def _state(self, random, speed=None):
        speed = speed if speed is not None else random.uniform(6.0, 35.0)
        return np.array([0, 0, 0, speed, random.uniform(-1.5, 1.5), random.uniform(-0.8, 0.8),
                         *(speed / 0.2 * random.uniform(0.9, 1.1, 4)), 0.8,
                         random.uniform(-0.3, 0.3)])

    def test_point_tangent_encloses_finite_differences_and_is_tight(self):
        random = np.random.default_rng(11)
        for _ in range(25):
            x = self._state(random)
            state = planar.PlanarState(x[3], x[4], x[5], x[6:10], 0.8)
            evaluation = planar.evaluate_planar_dynamics(self.vehicle, state, np.zeros(4), x[11])
            a = np.array([evaluation.longitudinal_acceleration - state.yaw_rate * state.v_y,
                          evaluation.lateral_acceleration + state.yaw_rate * state.v_x])
            gx, gy = li.load_map_enclosure(self.vehicle, x[None], x[None], (a - 1e-9)[None],
                                           (a + 1e-9)[None])
            for k, enclosure in enumerate((gx, gy)):
                for j in range(2):
                    h = 1.0e-5
                    step = np.zeros(2)
                    step[j] = h
                    derivative = (_point_map(self.vehicle, state, a + step, x[11])[k]
                                  - _point_map(self.vehicle, state, a - step, x[11])[k]) / (2 * h)
                    self.assertLessEqual(enclosure.tl[0, 0, j] - 1e-6, derivative)
                    self.assertGreaterEqual(enclosure.th[0, 0, j] + 1e-6, derivative)
                    self.assertLess(float(enclosure.th[0, 0, j] - enclosure.tl[0, 0, j]), 1.0e-3)

    def test_box_tangent_encloses_sampled_finite_differences(self):
        random = np.random.default_rng(12)
        for _ in range(25):
            x = self._state(random)
            half = np.array([0, 0, 0, 0.02, 0.02, 0.005, 0.1, 0.1, 0.1, 0.1, 0, 0.002])
            guess = random.uniform(-6.0, 6.0, 2)
            radius = np.array([0.02, 0.02])
            gx, gy = li.load_map_enclosure(self.vehicle, (x - half)[None], (x + half)[None],
                                           (guess - radius)[None], (guess + radius)[None])
            for _ in range(6):
                y = x + half * random.uniform(-1, 1, 12)
                a = guess + radius * random.uniform(-1, 1, 2)
                state = planar.PlanarState(y[3], y[4], y[5], y[6:10], 0.8)
                for k, enclosure in enumerate((gx, gy)):
                    for j in range(2):
                        h = 1.0e-5
                        step = np.zeros(2)
                        step[j] = h
                        derivative = (_point_map(self.vehicle, state, a + step, y[11])[k]
                                      - _point_map(self.vehicle, state, a - step, y[11])[k]) / (2 * h)
                        self.assertLessEqual(enclosure.tl[0, 0, j] - 1e-6, derivative)
                        self.assertGreaterEqual(enclosure.th[0, 0, j] + 1e-6, derivative)

    def test_zero_slip_corner_is_enclosed(self):
        # Standing wheels at rolling speed: slip -> 0, where 1/den blows up.
        random = np.random.default_rng(13)
        x = self._state(random, speed=12.0)
        x[4] = 0.0
        x[5] = 0.0
        x[11] = 0.0
        radius = self.vehicle.tire.r_1
        x[6:10] = x[3] / radius
        state = planar.PlanarState(x[3], x[4], x[5], x[6:10], 0.8)
        a = np.array([0.0, 0.0])
        gx, gy = li.load_map_enclosure(self.vehicle, x[None], x[None], (a - 0.5)[None], (a + 0.5)[None])
        for _ in range(20):
            probe = random.uniform(-0.5, 0.5, 2)
            px, py = _point_map(self.vehicle, state, probe, x[11])
            self.assertTrue(gx.lo[0, 0] <= px <= gx.hi[0, 0])
            self.assertTrue(gy.lo[0, 0] <= py <= gy.hi[0, 0])

    def test_h_functions_are_bracketed_by_high_precision_values(self):
        from mpmath import mp, mpf, exp, diff
        mp.dps = 40
        p = np.array([0.0, 1e-9, 1e-3, 0.2, 0.49999, 0.5, 0.7, 3.0, 25.0])
        (h_lo, h_hi), (d_lo, d_hi) = li._h_and_slope(p)
        for i, value in enumerate(p):
            x = mpf(float(value))
            h = (1 - exp(-x)) / x if x != 0 else mpf(1)
            slope = diff(lambda z: (1 - exp(-z)) / z, x) if x != 0 else mpf(-0.5)
            self.assertLessEqual(h_lo[i], float(h))
            self.assertGreaterEqual(h_hi[i], float(h))
            self.assertLessEqual(d_lo[i], float(slope))
            self.assertGreaterEqual(d_hi[i], float(slope))

    def test_inactive_clamp_matches_general_expression_on_wide_boxes(self):
        # Where a clamp can be active the general expression must still be used.
        random = np.random.default_rng(14)
        x = self._state(random)
        a = np.array([0.0, 40.0])  # huge lateral acceleration: transfer saturates
        gx, gy = li.load_map_enclosure(self.vehicle, x[None], x[None], (a - 1.0)[None], (a + 1.0)[None])
        state = planar.PlanarState(x[3], x[4], x[5], x[6:10], 0.8)
        for _ in range(20):
            probe = a + random.uniform(-1.0, 1.0, 2)
            px, py = _point_map(self.vehicle, state, probe, x[11])
            self.assertTrue(gx.lo[0, 0] <= px <= gx.hi[0, 0])
            self.assertTrue(gy.lo[0, 0] <= py <= gy.hi[0, 0])


class AdaptiveCertificateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(FORMULATION).model

    def _polynomial(self, speed_start, speed_end, steps=3):
        """A synthetic smooth trajectory fitted by SciPy BDF, in the 12-state layout."""

        def rhs(t, y):
            out = np.zeros(12)
            out[3] = (speed_end - speed_start) / 1.0
            out[6:10] = (speed_end - speed_start) / 0.2
            out[5] = 0.02
            out[11] = 0.01
            return out

        y0 = np.array([0, 0, 0, speed_start, 0.1, 0.02, *(np.full(4, speed_start / 0.2)), 0.8, 0.0])
        solution = solve_ivp(rhs, (0.0, 0.2 * steps), y0, method="BDF", rtol=1e-8, atol=1e-10,
                             dense_output=True, max_step=0.05)
        return dense_polynomial_coefficients(solution.sol), solution

    def _guesses(self, solution, poly):
        guesses = np.empty((poly.n_intervals + 1, 2))
        for row, time in enumerate(poly.times):
            y = solution.sol(time)
            state = planar.PlanarState(y[3], y[4], y[5], y[6:10], 0.8)
            evaluation = planar.evaluate_planar_dynamics(self.vehicle, state, np.zeros(4), y[11])
            guesses[row] = (evaluation.longitudinal_acceleration - state.yaw_rate * state.v_y,
                            evaluation.lateral_acceleration + state.yaw_rate * state.v_x)
        return guesses

    def test_smooth_trajectory_certifies_and_cover_is_exact(self):
        poly, solution = self._polynomial(14.0, 15.0)
        result = li.certify_load_closure_adaptive(self.vehicle, poly, self._guesses(solution, poly),
                                                  maximum_depth=14)
        self.assertTrue(result["certified"], result)
        self.assertEqual(result["spans_failed_at_maximum_depth"], 0)
        # Every level's spans either pass or are split into exactly two.
        carried = poly.n_intervals
        for level in result["levels"]:
            self.assertEqual(level["spans"], carried)
            carried = 2 * (level["spans"] - level["passed"])
        self.assertEqual(carried, 0)

    def test_shallow_depth_limit_fails_closed(self):
        poly, solution = self._polynomial(14.0, 15.0)
        result = li.certify_load_closure_adaptive(self.vehicle, poly, self._guesses(solution, poly),
                                                  maximum_depth=0)
        self.assertFalse(result["certified"])

    def test_bad_inputs_fail_closed(self):
        poly, solution = self._polynomial(14.0, 15.0)
        guesses = self._guesses(solution, poly)
        self.assertFalse(li.certify_load_closure_adaptive(self.vehicle, poly, guesses[:-1])["certified"])
        broken = guesses.copy()
        broken[0, 0] = np.nan
        self.assertFalse(li.certify_load_closure_adaptive(self.vehicle, poly, broken)["certified"])

    def test_wheel_speed_far_from_rolling_with_loose_node_does_not_certify_wrong_root(self):
        # A guess centre far from the true root must either re-centre to it or
        # fail; it must never certify a box that excludes the actual solution.
        poly, solution = self._polynomial(14.0, 15.0)
        guesses = self._guesses(solution, poly) + np.array([3.0, -3.0])
        result = li.certify_load_closure_adaptive(self.vehicle, poly, guesses, maximum_depth=10)
        if result["certified"]:
            y = solution.sol(poly.times[0])
            state = planar.PlanarState(y[3], y[4], y[5], y[6:10], 0.8)
            evaluation = planar.evaluate_planar_dynamics(self.vehicle, state, np.zeros(4), y[11])
            root = np.array([evaluation.longitudinal_acceleration - state.yaw_rate * state.v_y,
                             evaluation.lateral_acceleration + state.yaw_rate * state.v_x])
            self.assertLess(float(np.max(np.abs(root - guesses[0]))), 6.0)


if __name__ == "__main__":
    unittest.main()
