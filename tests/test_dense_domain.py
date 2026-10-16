"""Tests for conservative BDF dense-interpolant component bounds."""

import time
import unittest

import numpy as np
from scipy.integrate import solve_ivp
from scipy.integrate._ivp.bdf import BdfDenseOutput
from scipy.integrate._ivp.common import OdeSolution

from simplified_vehicle_models.solvers.dense_domain import (
    UnsupportedDenseOutputError,
    dense_component_bounds,
    dense_polynomial_coefficients,
    evaluate_dense_polynomials,
)


def _newton_basis(s, count):
    """BDF basis p_k(s) = prod_{j<=k} (s + j) / (j + 1), computed independently."""
    x = (np.asarray(s, dtype=float)[None, :] + np.arange(count)[:, None]) / (1.0 + np.arange(count)[:, None])
    return np.cumprod(x, axis=0)


def _synthetic_solution(d_rows, t_end=1.0, h=1.0):
    """One BDF interval with Newton-basis coefficients ``d_rows`` (order = len - 1)."""
    d = np.asarray(d_rows, dtype=float)
    order = d.shape[0] - 1
    piece = BdfDenseOutput(t_end - h, t_end, h, order, d)
    return OdeSolution([t_end - h, t_end], [piece], alt_segment=True), piece


def _arch_coefficients(amplitude):
    """Order-3 polynomial amplitude * s (s + 1/2) (s + 1) in the BDF Newton basis.

    It is exactly zero at both nodes (s = -1, 0) and at the midpoint (s = -1/2)
    but has interior extrema of magnitude amplitude * sqrt(3) / 36 ~ 0.048 * amplitude.
    """
    samples = np.array([-0.9, -0.25, -0.1])
    truth = amplitude * samples * (samples + 0.5) * (samples + 1.0)
    # y = D0 + sum_k D[k+1] p_k(s), with D0 = value at s = 0 = 0.
    d_tail = np.linalg.solve(_newton_basis(samples, 3).T, truth)
    return np.vstack([np.zeros((1, 1)), d_tail[:, None]])


class SyntheticIntervalTests(unittest.TestCase):
    def test_interior_peak_missed_by_nodes_and_midpoint_is_bounded(self):
        solution, _ = _synthetic_solution(_arch_coefficients(1.0))
        limit = 0.04
        for t in (0.0, 0.5, 1.0):  # both nodes and the midpoint sit at (numerically) zero
            self.assertLess(abs(float(solution(t)[0])), 1e-12)
        grid = np.linspace(0.0, 1.0, 100001)
        true_max = float(np.max(solution(grid)[0]))
        true_min = float(np.min(solution(grid)[0]))
        self.assertGreater(true_max, limit)  # the hidden excursion violates the limit
        bounds = dense_component_bounds(solution, 0)
        self.assertEqual(bounds.shape, (1, 1, 2))
        low, high = bounds[0, 0]
        self.assertGreaterEqual(low, -1.0)
        self.assertLessEqual(low, true_min)
        self.assertGreaterEqual(high, true_max)
        # Conservative but useful: far tighter than 5x the true excursion, and it exposes the violation.
        self.assertGreater(high, limit)
        self.assertLess(high, true_max * 1.05)
        self.assertGreater(low, true_min * 1.05 - 1e-12)

    def test_subdivision_tightens_but_always_contains_truth(self):
        solution, _ = _synthetic_solution(_arch_coefficients(1.0))
        grid = np.linspace(0.0, 1.0, 20001)
        values = solution(grid)[0]
        widths = []
        for subdivisions in (1, 2, 8, 32):
            low, high = dense_component_bounds(solution, 0, subdivisions=subdivisions)[0, 0]
            self.assertLessEqual(low, values.min())
            self.assertGreaterEqual(high, values.max())
            widths.append(high - low)
        self.assertEqual(widths, sorted(widths, reverse=True))

    def test_ordinary_degree_one_segment(self):
        solution, _ = _synthetic_solution([[2.0, -1.0], [3.0, 0.5]], t_end=4.0, h=0.5)
        # y(s) = D0 + D1 * s with s = (t - 4) / 0.5 over t in [3.5, 4]  ->  s in [-1, 0].
        bounds = dense_component_bounds(solution)
        np.testing.assert_allclose(bounds[0, :, 0], [-1.0, -1.5], atol=1e-12)
        np.testing.assert_allclose(bounds[0, :, 1], [2.0, -1.0], atol=1e-12)
        self.assertTrue(np.all(bounds[..., 0] <= bounds[..., 1]))

    def test_component_selection(self):
        solution, _ = _synthetic_solution([[2.0, -1.0, 7.0], [3.0, 0.5, 0.0]])
        full = dense_component_bounds(solution)
        self.assertEqual(full.shape, (1, 3, 2))
        np.testing.assert_array_equal(dense_component_bounds(solution, 1), full[:, 1:2])
        np.testing.assert_array_equal(dense_component_bounds(solution, [2, 0]), full[:, [2, 0]])
        np.testing.assert_array_equal(dense_component_bounds(solution, -1), full[:, 2:3])
        with self.assertRaises(IndexError):
            dense_component_bounds(solution, 3)
        with self.assertRaises(ValueError):
            dense_component_bounds(solution, 0, subdivisions=0)

    def test_nonfinite_coefficients_fail_closed(self):
        solution, _ = _synthetic_solution([[np.nan, 1.0], [1.0, np.inf]])
        bounds = dense_component_bounds(solution)
        self.assertTrue(np.all(bounds[0, :, 0] == -np.inf))
        self.assertTrue(np.all(bounds[0, :, 1] == np.inf))

    def test_unsupported_and_malformed_interpolants_fail_closed(self):
        class Custom(BdfDenseOutput):
            pass

        class Foreign:
            t_min, t_max = 0.0, 1.0

            def __call__(self, t):
                return np.zeros((1,) + np.shape(t))

        _, piece = _synthetic_solution([[0.0], [1.0]])
        for interpolant in (Foreign(), Custom(0.0, 1.0, 1.0, 1, np.array([[0.0], [1.0]]))):
            solution = OdeSolution([0.0, 1.0], [interpolant])
            with self.assertRaises(UnsupportedDenseOutputError):
                dense_component_bounds(solution)
        with self.assertRaises(UnsupportedDenseOutputError):
            dense_component_bounds(object())
        # Corrupted Newton basis / out-of-range / bad order must also be refused.
        broken = BdfDenseOutput(0.0, 1.0, 1.0, 2, np.zeros((3, 1)))
        broken.t_shift = broken.t_shift + 0.25
        with self.assertRaises(UnsupportedDenseOutputError):
            dense_component_bounds(OdeSolution([0.0, 1.0], [broken], alt_segment=True))
        with self.assertRaises(UnsupportedDenseOutputError):
            dense_component_bounds(OdeSolution([5.0, 6.0], [piece], alt_segment=True))
        with self.assertRaises(UnsupportedDenseOutputError):
            dense_component_bounds(OdeSolution([0.0, 1.0], [BdfDenseOutput(0.0, 1.0, 1.0, 1, np.zeros((3, 1)))]))


class SolveIvpTests(unittest.TestCase):
    @staticmethod
    def _oscillator(**kwargs):
        def rhs(t, y):
            return [y[1], 5.0 * (1.0 - y[0] ** 2) * y[1] - y[0], -0.3 * y[2] + np.sin(3.0 * t)]

        return solve_ivp(rhs, (0.0, 12.0), [2.0, 0.0, 1.0], method="BDF", dense_output=True,
                         rtol=1e-6, atol=1e-9, **kwargs)

    def test_linear_trajectory_bounds_are_tight_and_enclose_nodes(self):
        result = solve_ivp(lambda t, y: [2.0, -1.0], (0.0, 5.0), [1.0, 3.0], method="BDF",
                           dense_output=True, rtol=1e-8, atol=1e-10)
        bounds = dense_component_bounds(result.sol)
        self.assertEqual(bounds.shape, (result.t.size - 1, 2, 2))
        left, right = result.y[:, :-1].T, result.y[:, 1:].T
        lo, hi = np.minimum(left, right), np.maximum(left, right)
        # Enclose the node values from the integrator...
        self.assertTrue(np.all(bounds[..., 0] <= lo + 1e-9))
        self.assertTrue(np.all(bounds[..., 1] >= hi - 1e-9))
        # ...and a straight line is hardly widened beyond round-off.
        np.testing.assert_allclose(bounds[..., 0], lo, atol=1e-8)
        np.testing.assert_allclose(bounds[..., 1], hi, atol=1e-8)

    def test_extracted_polynomial_matches_public_solution_inside_each_interval(self):
        result = self._oscillator()
        poly = dense_polynomial_coefficients(result.sol)
        self.assertEqual(poly.n_intervals, result.t.size - 1)
        self.assertGreaterEqual(int(poly.order.max()), 2)  # higher orders are exercised
        fractions = np.array([0.0, 0.07, 0.31, 0.5, 0.83, 0.97, 1.0])
        worst = 0.0
        for i in range(poly.n_intervals):
            t = poly.times[i] + fractions * (poly.times[i + 1] - poly.times[i])
            expected = np.column_stack([result.sol.interpolants[i](x) for x in t]).T.T
            got = evaluate_dense_polynomials(poly, i, t)
            worst = max(worst, float(np.max(np.abs(got - expected) / (1.0 + np.abs(expected)))))
        self.assertLess(worst, 1e-9)

    def test_bounds_contain_public_dense_samples_everywhere(self):
        result = self._oscillator()
        bounds = dense_component_bounds(result.sol)
        self.assertEqual(bounds.shape, (result.t.size - 1, 3, 2))
        for i in range(bounds.shape[0]):
            t = np.linspace(result.t[i], result.t[i + 1], 41)
            samples = result.sol(t)  # public API; interior points resolve to interval i
            self.assertTrue(np.all(samples.T >= bounds[i, :, 0] - 1e-12), msg=f"interval {i} below")
            self.assertTrue(np.all(samples.T <= bounds[i, :, 1] + 1e-12), msg=f"interval {i} above")
        # The oscillator is a relaxation oscillation: non-trivial, finite, ordered ranges.
        self.assertTrue(np.all(np.isfinite(bounds)))
        self.assertTrue(np.all(bounds[..., 0] <= bounds[..., 1]))

    def test_event_truncated_final_interval(self):
        def hit(t, y):
            return y[0] - 0.9

        hit.terminal = True
        hit.direction = -1.0
        result = solve_ivp(lambda t, y: [-y[0]], (0.0, 20.0), [2.0], method="BDF", dense_output=True,
                           rtol=1e-8, atol=1e-10, events=hit)
        self.assertEqual(result.status, 1)
        last = result.sol.interpolants[-1]
        self.assertGreater(last.t_max, result.t[-1] + 1e-12)  # truncated: interpolant extends past the event
        bounds = dense_component_bounds(result.sol)
        self.assertEqual(bounds.shape[0], result.t.size - 1)
        self.assertGreaterEqual(bounds[-1, 0, 0], 0.9 - 1e-6)  # truncated span ends at the event value
        self.assertGreaterEqual(bounds[-1, 0, 1], float(result.y[0, -2]) - 1e-9)
        self.assertLess(bounds[-1, 0, 0], float(result.y[0, -1]) + 1e-9)
        self.assertLess(bounds[-1, 0, 0], bounds[-1, 0, 1])

    def test_unsupported_methods_fail_closed(self):
        for method in ("RK45", "LSODA", "Radau", "DOP853"):
            result = solve_ivp(lambda t, y: [-y[0]], (0.0, 2.0), [1.0], method=method, dense_output=True)
            with self.assertRaises(UnsupportedDenseOutputError, msg=method):
                dense_component_bounds(result.sol)

    def test_fifteen_thousand_intervals_are_fast(self):
        rng = np.random.default_rng(7)
        count, states = 15000, 12
        pieces, t_end = [], 0.0
        times = [0.0]
        for _ in range(count):
            h = 0.01 * (1.0 + rng.random())
            order = int(rng.integers(1, 6))
            pieces.append(BdfDenseOutput(t_end, t_end + h, h, order, rng.normal(size=(order + 1, states))))
            t_end += h
            times.append(t_end)
        solution = OdeSolution(times, pieces, alt_segment=True)
        started = time.perf_counter()
        bounds = dense_component_bounds(solution)
        elapsed = time.perf_counter() - started
        self.assertEqual(bounds.shape, (count, states, 2))
        self.assertLess(elapsed, 10.0)
        # Spot-check enclosure against dense sampling of the true polynomials.
        for i in rng.integers(0, count, size=25).tolist():
            t = np.linspace(times[i], times[i + 1], 201)
            values = pieces[i](t)
            self.assertTrue(np.all(values.T >= bounds[i, :, 0] - 1e-12))
            self.assertTrue(np.all(values.T <= bounds[i, :, 1] + 1e-12))


if __name__ == "__main__":
    unittest.main()
