"""Tests for the two IEM24 five-channel correlation figures."""
import unittest

import numpy as np

from tools.reports.plot_iem24_endurance_correlation import fold_measured, model_channels


class CorrelationFigureTests(unittest.TestCase):
    def test_power_is_equal_per_rear_motor_and_converts_to_kw(self):
        data = {
            "apps_fraction": np.array([0., 1.]),
            "velocity_mps": np.array([10., 10.]),
            "longitudinal_acceleration_mps2": np.zeros(2),
            "lateral_acceleration_mps2": np.zeros(2),
        }
        curves = model_channels(data)
        np.testing.assert_array_equal(curves["rl_power_kw"], curves["rr_power_kw"])
        self.assertEqual(curves["rl_power_kw"][0], 0.)
        self.assertGreater(curves["rl_power_kw"][1], 0.)

    def test_time_grid_does_not_stretch_short_laps(self):
        t = np.array([0., 1., 2., 3., 4., 5., 10., 11., 12., 13.])
        measured = {
            "time_s": t, "lap_index": np.array([1.] * 6 + [2.] * 4),
            "speed_mps": np.ones(10),
            "longitudinal_acceleration_mps2": np.zeros(10),
            "lateral_acceleration_mps2": np.zeros(10),
            "rl_power_kw": np.ones(10), "rr_power_kw": np.ones(10),
        }
        grid, folded = fold_measured(measured, np.array([1., 2.]), t, 10., "time", 6)
        self.assertEqual(grid[-1], 5.)
        self.assertTrue(np.isnan(folded["velocity_mps"][1, -1]))
        self.assertFalse(np.isnan(folded["velocity_mps"][0, -1]))

    def test_unknown_axis_fails(self):
        with self.assertRaisesRegex(ValueError, "unknown axis"):
            fold_measured({}, np.array([]), np.array([]), 10., "bad")


if __name__ == "__main__":
    unittest.main()
