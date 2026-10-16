from __future__ import annotations

import unittest

import numpy as np

from tools.reports.compare_iem24_three_way import _zero_each_lap_voltage


class ThreeWayVoltageReferenceTests(unittest.TestCase):
    def test_each_lap_starts_at_zero_without_mutating_source_data(self):
        measured = {
            "lap_index": np.array([0, 0, 1, 1, 1], dtype=float),
            "pack_voltage_v": np.array([600.0, 598.0, 580.0, 579.0, 577.0]),
            "speed_mps": np.array([0.0, 10.0, 11.0, 12.0, 13.0]),
        }
        original_voltage = measured["pack_voltage_v"].copy()

        adjusted = _zero_each_lap_voltage(measured, np.array([0.0, 1.0]))

        np.testing.assert_allclose(adjusted["pack_voltage_v"], [0.0, -2.0, 0.0, -1.0, -3.0])
        np.testing.assert_array_equal(measured["pack_voltage_v"], original_voltage)
        np.testing.assert_array_equal(adjusted["speed_mps"], measured["speed_mps"])


if __name__ == "__main__":
    unittest.main()
