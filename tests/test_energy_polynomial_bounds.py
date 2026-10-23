from fractions import Fraction
import random
import unittest

from simplified_vehicle_models.solvers.energy_polynomial_bounds import bound_energy_polynomials


def power_oracle(x, h, w0, w1, f0, f1):
    # Independent power-basis Hermite expression.
    return ((2*x**3 - 3*x**2 + 1)*w0 + (x**3 - 2*x**2 + x)*h*f0
            + (-2*x**3 + 3*x**2)*w1 + (x**3 - x**2)*h*f1)


def derivative_oracle(x, h, w0, w1, f0, f1):
    return ((6*x**2 - 6*x)*w0/h + (3*x**2 - 4*x + 1)*f0
            + (-6*x**2 + 6*x)*w1/h + (3*x**2 - 2*x)*f1)


def control_oracle(x, f0, fm, f1):
    return 2*(x-Fraction(1,2))*(x-1)*f0 - 4*x*(x-1)*fm + 2*x*(x-Fraction(1,2))*f1


class EnergyPolynomialBoundsTests(unittest.TestCase):
    def test_random_exact_power_oracle_and_enclosures(self):
        rng = random.Random(7419)
        for _ in range(60):
            h = Fraction.from_float(float(Fraction(rng.randint(1, 8), rng.randint(1, 5))))
            w0, w1 = Fraction(rng.randint(1, 30), 3), Fraction(rng.randint(1, 30), 3)
            f0, fm, f1 = (Fraction(rng.randint(-15, 15), 4) for _ in range(3))
            result = bound_energy_polynomials((0.0, float(h)), (w0, w1), (f0, fm, f1))
            for x in (Fraction(0), Fraction(1, 7), Fraction(1, 2), Fraction(5, 6), Fraction(1)):
                w = power_oracle(x, h, w0, w1, f0, f1)
                dw = derivative_oracle(x, h, w0, w1, f0, f1)
                fc = control_oracle(x, f0, fm, f1)
                self.assertLessEqual(result.energy_interval[0], w)
                self.assertLessEqual(w, result.energy_interval[1])
                self.assertLessEqual(result.hermite_energy_rate_interval[0], dw)
                self.assertLessEqual(dw, result.hermite_energy_rate_interval[1])
                self.assertLessEqual(result.control_energy_rate_interval[0], fc)
                self.assertLessEqual(fc, result.control_energy_rate_interval[1])

    def test_zero_defect_and_exact_quadratic_tight_defect(self):
        linear = bound_energy_polynomials((0., 2.), (10, 14), (2, 2, 2))
        self.assertEqual(linear.energy_rate_defect_interval, (Fraction(0), Fraction(0)))
        delta = Fraction(6)
        # Select endpoints/rates giving defect 2*delta*x*(1-x).
        h, w0, f0, f1, fm = Fraction(1), Fraction(9), Fraction(0), Fraction(0), Fraction(-3)
        w1 = w0 + h * (f0 + f1 + 2*fm + delta) / 3
        full = bound_energy_polynomials((0., 1.), (w0, w1), (f0, fm, f1))
        self.assertEqual(full.energy_rate_defect_interval, (Fraction(0), delta/2))
        sub = bound_energy_polynomials((0., 1.), (w0, w1), (f0, fm, f1), subinterval=(0., .25))
        self.assertEqual(sub.energy_rate_defect_interval, (Fraction(0), 3*delta/8))

    def test_preserves_negative_interior_and_marks_semantics(self):
        result = bound_energy_polynomials((0., 1.), (1, 1), (100, 0, 100))
        self.assertLess(result.energy_interval[0], 0)
        self.assertFalse(result.physical_feasibility_certified)
        self.assertEqual(result.mathematical_semantics, "explicit_rational_Hermite_and_Lagrange_energy_v1")

    def test_validation_and_immutability(self):
        good = ((0., 1.), (2, 3), (1, 2, 3))
        for args in [((True, 1.), (2, 3), (1, 2, 3)), ((0., 0.), (2, 3), (1, 2, 3)),
                     ((0., 1.), (2, 3), (1, 2)), ((0., 1.), (True, 3), (1, 2, 3))]:
            with self.assertRaises((TypeError, ValueError)):
                bound_energy_polynomials(*args)
        with self.assertRaises(ValueError):
            bound_energy_polynomials(*good, subinterval=(0., 1.0000000000000002))
        exact = bound_energy_polynomials((0., 1.), (Fraction(1, 10**100), 3), (1, 2, 3))
        self.assertEqual(exact.energy_interval[0], Fraction(1, 10**100))
        result = bound_energy_polynomials(*good)
        with self.assertRaises(Exception):
            result.energy_interval = (0, 0)

    def test_subinterval_and_coordinate_origins(self):
        a = bound_energy_polynomials((10., 14.), (7, 9), (1, 2, -1), subinterval=(11., 13.))
        self.assertEqual(a.subinterval, (11., 13.))
        self.assertEqual(a.source_interval, (10., 14.))
        with self.assertRaises(ValueError):
            bound_energy_polynomials((10., 14.), (7, 9), (1, 2, -1), subinterval=(13., 11.))


if __name__ == "__main__":
    unittest.main()
