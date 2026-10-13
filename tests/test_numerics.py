"""Tests for the shared numerics module.

The tests cover linear interpolation, monotone validation, and smoothing.
They also cover the shape rules and the extrapolation guard.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from simplified_vehicle_models.core.numerics import (
    interpolate_1d,
    interpolate_2d,
    smooth,
    validate_monotone,
)


class MonotoneValidationTests(unittest.TestCase):
    """Check the monotone abscissa rule."""

    def test_accepts_a_strictly_increasing_abscissa(self) -> None:
        validate_monotone(np.array([0.0, 1.0, 2.0, 5.0]), "speed")
        validate_monotone(np.array([-3.0, -1.0, 0.5]), "torque")

    def test_duplicate_abscissa_raises(self) -> None:
        with self.assertRaises(ValueError) as context:
            validate_monotone(np.array([0.0, 1.0, 1.0, 2.0]), "speed")
        self.assertIn("speed", str(context.exception))
        self.assertIn("2", str(context.exception))

    def test_decreasing_abscissa_raises(self) -> None:
        with self.assertRaises(ValueError) as context:
            validate_monotone(np.array([0.0, 3.0, 2.0, 4.0]), "speed")
        message = str(context.exception)
        self.assertIn("speed", message)
        self.assertIn("1", message)

    def test_two_dimensional_abscissa_raises(self) -> None:
        with self.assertRaises(ValueError):
            validate_monotone(np.zeros((2, 2)), "speed")

    def test_non_finite_abscissa_raises(self) -> None:
        for value in (np.nan, np.inf, -np.inf):
            with self.subTest(value=value):
                with self.assertRaises(ValueError) as context:
                    validate_monotone(np.array([0.0, value, 2.0]), "speed")
                message = str(context.exception)
                self.assertIn("speed", message)
                self.assertIn("finite", message)


class InterpolateOneDimensionTests(unittest.TestCase):
    """Check the one-dimensional interpolation."""

    def setUp(self) -> None:
        self.abscissa = np.array([0.0, 10.0, 20.0, 40.0])
        self.values = np.array([0.0, 30.0, 40.0, 100.0])

    def test_exact_at_every_node(self) -> None:
        result = interpolate_1d(self.abscissa, self.values, self.abscissa.copy())
        self.assertTrue(np.allclose(result, self.values, rtol=0.0, atol=0.0))

    def test_exact_at_midpoints(self) -> None:
        query = np.array([5.0, 15.0, 30.0])
        expected = np.array([15.0, 35.0, 70.0])
        result = interpolate_1d(self.abscissa, self.values, query)
        self.assertTrue(np.allclose(result, expected, rtol=0.0, atol=1.0e-12))

    def test_query_below_range_raises(self) -> None:
        with self.assertRaises(ValueError) as context:
            interpolate_1d(self.abscissa, self.values, -1.0)
        message = str(context.exception)
        self.assertIn("query", message)
        self.assertIn("-1", message)
        self.assertIn("0", message)
        self.assertIn("40", message)

    def test_query_above_range_raises(self) -> None:
        with self.assertRaises(ValueError) as context:
            interpolate_1d(self.abscissa, self.values, 41.0)
        message = str(context.exception)
        self.assertIn("query", message)
        self.assertIn("41", message)
        self.assertIn("40", message)

    def test_array_query_above_range_raises(self) -> None:
        with self.assertRaises(ValueError):
            interpolate_1d(self.abscissa, self.values, np.array([1.0, 99.0]))

    def test_scalar_query_keeps_scalar_shape(self) -> None:
        result = interpolate_1d(self.abscissa, self.values, 5.0)
        self.assertEqual(np.shape(result), ())
        self.assertAlmostEqual(float(result), 15.0, places=12)

    def test_array_query_keeps_array_shape(self) -> None:
        query = np.array([[5.0, 15.0], [30.0, 20.0]])
        result = interpolate_1d(self.abscissa, self.values, query)
        self.assertEqual(result.shape, query.shape)
        self.assertAlmostEqual(float(result[0, 0]), 15.0, places=12)
        self.assertAlmostEqual(float(result[1, 1]), 40.0, places=12)

    def test_mismatched_values_raise(self) -> None:
        with self.assertRaises(ValueError):
            interpolate_1d(self.abscissa, np.array([1.0, 2.0]), 5.0)

    def test_duplicate_abscissa_raises(self) -> None:
        with self.assertRaises(ValueError):
            interpolate_1d(np.array([0.0, 0.0, 1.0]), np.array([0.0, 1.0, 2.0]), 0.5)

    def test_non_finite_query_raises(self) -> None:
        for value in (np.nan, np.inf, -np.inf):
            with self.subTest(value=value):
                with self.assertRaises(ValueError) as context:
                    interpolate_1d(self.abscissa, self.values, value)
                self.assertIn("query", str(context.exception))


class InterpolateTwoDimensionTests(unittest.TestCase):
    """Check the two-dimensional interpolation."""

    def setUp(self) -> None:
        self.abscissa = np.array([0.0, 10.0, 20.0])
        self.ordinate = np.array([0.0, 1.0, 4.0, 8.0])
        self.values = np.array(
            [
                [0.0, 2.0, 3.0, 4.0],
                [5.0, 7.0, 8.0, 9.0],
                [10.0, 12.0, 13.0, 14.0],
            ]
        )

    def test_exact_at_every_node(self) -> None:
        for row, first in enumerate(self.abscissa):
            for column, second in enumerate(self.ordinate):
                result = interpolate_2d(
                    self.abscissa,
                    self.ordinate,
                    self.values,
                    first,
                    second,
                )
                self.assertAlmostEqual(
                    float(result),
                    float(self.values[row, column]),
                    places=12,
                )

    def test_exact_at_the_centre_of_a_cell(self) -> None:
        result = interpolate_2d(self.abscissa, self.ordinate, self.values, 5.0, 0.5)
        # The cell corners hold 0, 2, 5, and 7. The centre is their mean.
        self.assertAlmostEqual(float(result), 3.5, places=12)

    def test_query_below_range_raises(self) -> None:
        with self.assertRaises(ValueError) as context:
            interpolate_2d(self.abscissa, self.ordinate, self.values, -1.0, 1.0)
        self.assertIn("query_abscissa", str(context.exception))

    def test_query_above_range_raises(self) -> None:
        with self.assertRaises(ValueError) as context:
            interpolate_2d(self.abscissa, self.ordinate, self.values, 1.0, 9.0)
        self.assertIn("query_ordinate", str(context.exception))

    def test_shapes_are_preserved(self) -> None:
        scalar = interpolate_2d(self.abscissa, self.ordinate, self.values, 5.0, 0.5)
        self.assertEqual(np.shape(scalar), ())

        first = np.array([5.0, 15.0])
        second = np.array([[0.5, 2.0], [3.0, 6.0]])
        result = interpolate_2d(
            self.abscissa,
            self.ordinate,
            self.values,
            first,
            second,
        )
        self.assertEqual(result.shape, (2, 2))

    def test_mismatched_values_raise(self) -> None:
        with self.assertRaises(ValueError):
            interpolate_2d(
                self.abscissa,
                self.ordinate,
                np.zeros((2, 2)),
                5.0,
                1.0,
            )

    def test_decreasing_ordinate_raises(self) -> None:
        decreasing = np.array([0.0, 4.0, 1.0, 8.0])
        with self.assertRaises(ValueError) as context:
            interpolate_2d(self.abscissa, decreasing, self.values, 5.0, 1.0)
        message = str(context.exception)
        self.assertIn("ordinate", message)
        self.assertIn("1", message)

    def test_duplicate_ordinate_raises(self) -> None:
        repeated = np.array([0.0, 1.0, 1.0, 8.0])
        with self.assertRaises(ValueError) as context:
            interpolate_2d(self.abscissa, repeated, self.values, 5.0, 1.0)
        self.assertIn("ordinate", str(context.exception))

    def test_non_finite_query_raises(self) -> None:
        with self.assertRaises(ValueError) as context:
            interpolate_2d(self.abscissa, self.ordinate, self.values, np.nan, 1.0)
        self.assertIn("query_abscissa", str(context.exception))
        with self.assertRaises(ValueError) as context:
            interpolate_2d(self.abscissa, self.ordinate, self.values, 5.0, np.inf)
        self.assertIn("query_ordinate", str(context.exception))


class SmoothTests(unittest.TestCase):
    """Check the centred moving average."""

    def test_keeps_the_length(self) -> None:
        values = np.arange(10.0)
        for window in (1, 3, 5, 9):
            self.assertEqual(smooth(values, window).shape, values.shape)

    def test_leaves_a_constant_array_unchanged(self) -> None:
        values = np.full(9, 3.5)
        result = smooth(values, 5)
        self.assertTrue(np.allclose(result, values, rtol=0.0, atol=1.0e-12))

    def test_averages_a_centred_window(self) -> None:
        values = np.array([0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0])
        result = smooth(values, 3)
        expected = np.array([0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0])
        self.assertTrue(np.allclose(result, expected, rtol=0.0, atol=1.0e-12))

    def test_even_window_raises(self) -> None:
        with self.assertRaises(ValueError):
            smooth(np.arange(5.0), 4)

    def test_window_below_one_raises(self) -> None:
        with self.assertRaises(ValueError):
            smooth(np.arange(5.0), 0)

    def test_window_above_length_raises(self) -> None:
        with self.assertRaises(ValueError):
            smooth(np.arange(5.0), 7)

    def test_window_one_returns_a_copy(self) -> None:
        values = np.arange(5.0)
        result = smooth(values, 1)
        self.assertTrue(np.array_equal(result, values))
        self.assertIsNot(result, values)


if __name__ == "__main__":
    unittest.main()
