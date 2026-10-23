"""Independent exact-oracle tests for signed chart rectangle bounds."""
from fractions import Fraction
import unittest

import numpy as np

from simplified_vehicle_models.formulations.signed_capability import SignedTransientCapability
from simplified_vehicle_models.solvers.signed_chart_bounds import bound_signed_chart_rectangle


def exact_bilinear(axis_u, axis_k, values, u, k):
    """Test-only exact binary-rational interpolation; no production helpers."""
    i = len(axis_u) - 2
    for n in range(1, len(axis_u)):
        if u <= Fraction.from_float(float(axis_u[n])):
            i = n - 1
            break
    j = len(axis_k) - 2
    for n in range(1, len(axis_k)):
        if k <= Fraction.from_float(float(axis_k[n])):
            j = n - 1
            break
    u0, u1 = map(lambda x: Fraction.from_float(float(x)), axis_u[i:i+2])
    k0, k1 = map(lambda x: Fraction.from_float(float(x)), axis_k[j:j+2])
    tu, tk = (u-u0)/(u1-u0), (k-k0)/(k1-k0)
    f00, f10 = Fraction.from_float(float(values[i,j])), Fraction.from_float(float(values[i+1,j]))
    f01, f11 = Fraction.from_float(float(values[i,j+1])), Fraction.from_float(float(values[i+1,j+1]))
    return (1-tu)*(1-tk)*f00 + tu*(1-tk)*f10 + (1-tu)*tk*f01 + tu*tk*f11


class SignedChartBoundsTests(unittest.TestCase):
    def make_chart(self):
        u = np.array([0., .23, .71, 1.])
        k = np.array([-.2, -.07, .3])
        amin = np.array([[-10., -8., -9.], [-7., -12., -5.], [-11., -6., -13.], [-8., -10., -7.]])
        amax = -amin + 2.
        domain = np.array([20., 25., 22.])
        return SignedTransientCapability(u, k, amax, amin, domain, .5, "test")

    def test_exact_corner_oracle_contains_acceleration_extrema(self):
        chart = self.make_chart()
        bounds = bound_signed_chart_rectangle(chart, (.1, .9), (-.15, .2))
        u_corners = [.1, *[float(x) for x in chart.u if .1 < x < .9], .9]
        k_corners = [-.15, *[float(x) for x in chart.curvature if -.15 < x < .2], .2]
        corners = [(Fraction.from_float(u), Fraction.from_float(k))
                   for u in u_corners for k in k_corners]
        for table, lower, upper in (
                (chart.acceleration_min, bounds.acceleration_min_lower, bounds.acceleration_min_upper),
                (chart.acceleration_max, bounds.acceleration_max_lower, bounds.acceleration_max_upper)):
            exact = [exact_bilinear(chart.u, chart.curvature, table, u, k) for u, k in corners]
            self.assertLessEqual(Fraction.from_float(lower), min(exact))
            self.assertGreaterEqual(Fraction.from_float(upper), max(exact))
            # Do not pass by returning whole-table extrema: restricted rectangle
            # bounds should remain within small outward-rounding slack of oracle.
            slack = Fraction.from_float(1e-12)
            self.assertLessEqual(Fraction.from_float(lower), min(exact))
            self.assertGreaterEqual(Fraction.from_float(lower), min(exact) - slack)
            self.assertLessEqual(Fraction.from_float(upper), max(exact) + slack)
        d = [Fraction.from_float(float(x)) for x in chart.domain_speed_values]
        ks = [Fraction.from_float(float(x)) for x in chart.curvature]
        k_lo, k_hi = map(Fraction.from_float, (-.15, .2))
        def domain(k):
            j = next(j for j in range(len(ks)-1) if ks[j] <= k <= ks[j+1])
            t = (k-ks[j])/(ks[j+1]-ks[j])
            return (1-t)*d[j]+t*d[j+1]
        dexact = [domain(k_lo), domain(k_hi)] + [d[j] for j, k in enumerate(ks) if k_lo <= k <= k_hi]
        self.assertLessEqual(Fraction.from_float(bounds.domain_speed_lower), min(dexact))
        self.assertGreaterEqual(Fraction.from_float(bounds.domain_speed_upper), max(dexact))

    def test_budget_degenerate_nodes_and_frozen_output(self):
        chart = self.make_chart()
        whole = bound_signed_chart_rectangle(chart, (0., 1.), (-.2, .3))
        exact_budget = bound_signed_chart_rectangle(chart, (0., 1.), (-.2, .3), maximum_cells=whole.cell_count)
        self.assertEqual(exact_budget.cell_count, whole.cell_count)
        point = bound_signed_chart_rectangle(chart, (.23, .23), (-.07, -.07), maximum_cells=1)
        self.assertEqual(point.cell_count, 1)
        with self.assertRaises((AttributeError, TypeError)):
            point.cell_count = 0
        with self.assertRaises(ValueError):
            bound_signed_chart_rectangle(chart, (0., 1.), (-.2, .3), maximum_cells=whole.cell_count - 1)

    def test_strict_inputs_and_type_identity(self):
        chart = self.make_chart()
        for bad in (True, False, "0.2", complex(.1), np.array(.1), float("nan"), float("inf")):
            with self.subTest(bad=repr(bad)), self.assertRaises((TypeError, ValueError)):
                bound_signed_chart_rectangle(chart, (bad, .5), (-.2, .2))
        for bad in ((-.3, .2), (0., 1.1), (.7, .2)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                bound_signed_chart_rectangle(chart, bad, (-.2, .2))
        with self.assertRaises(ValueError):
            bound_signed_chart_rectangle(chart, (0., 1.), (-.3, .2))
        class Derived(SignedTransientCapability):
            pass
        with self.assertRaises(TypeError):
            bound_signed_chart_rectangle(Derived(**{name: getattr(chart, name) for name in (
                "u", "curvature", "acceleration_max", "acceleration_min", "domain_speed_values",
                "speed_floor", "provenance_hash")}), (0., 1.), (-.2, .2))

    def test_random_nonuniform_saddles_match_exact_cell_corner_oracle(self):
        rng = np.random.default_rng(347)
        for trial in range(60):
            u = np.array([0., .17, .63, 1.])
            k = np.array([-.2, -.11, .08, .3])
            amin = rng.integers(-20, 4, size=(4, 4)).astype(float)
            # Force alternating saddle signs while keeping amin < amax.
            amin[1::2, 1::2] -= 5.
            amax = amin + rng.integers(2, 17, size=(4, 4))
            chart = SignedTransientCapability(u, k, amax, amin, np.array([20., 18., 26., 23.]), .5, f"{trial}")
            ulo, uhi, klo, khi = .07, .91, -.16, .21
            result = bound_signed_chart_rectangle(chart, (ulo, uhi), (klo, khi))
            us = [ulo, *[x for x in u if ulo < x < uhi], uhi]
            ks = [klo, *[x for x in k if klo < x < khi], khi]
            exact_pairs = [(Fraction.from_float(x), Fraction.from_float(y)) for x in us for y in ks]
            for table, lo, hi in ((chart.acceleration_min, result.acceleration_min_lower, result.acceleration_min_upper),
                                  (chart.acceleration_max, result.acceleration_max_lower, result.acceleration_max_upper)):
                values = [exact_bilinear(u, k, table, x, y) for x, y in exact_pairs]
                self.assertLessEqual(Fraction.from_float(lo), min(values))
                self.assertGreaterEqual(Fraction.from_float(hi), max(values))
                slack = Fraction.from_float(1.e-12)
                self.assertGreaterEqual(Fraction.from_float(lo), min(values)-slack)
                self.assertLessEqual(Fraction.from_float(hi), max(values)+slack)

    def test_outward_endpoint_overflow_fails_closed(self):
        from simplified_vehicle_models.solvers.signed_chart_bounds import _out
        with self.assertRaises(RuntimeError):
            _out(-np.finfo(float).max, -np.finfo(float).max)
        with self.assertRaises(RuntimeError):
            _out(np.finfo(float).max, np.finfo(float).max)


if __name__ == "__main__":
    unittest.main()
