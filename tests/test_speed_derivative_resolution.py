import json
import math
import subprocess
import unittest
from pathlib import Path

from tools.diagnostics.audit_speed_derivative_resolution import run_analytic_study


class DerivativeResolutionTest(unittest.TestCase):
    def test_analytic_study_records_error_and_uncertainty(self):
        result = run_analytic_study()
        self.assertIs(result["acceptance_authority"], False)
        row = next(r for r in result["rows"]
                   if r["function"] == "rapid_sine" and r["side"] == "central"
                   and r["step"] == 0.001)
        self.assertLess(abs(row["exact"] - 10 * math.cos(100 * 0.123)), 1e-12)
        self.assertLess(row["absolute_error"], 2e-7)
        self.assertGreater(row["uncertainty"], 1e-4)
        self.assertEqual(row["exact_source"], "analytic_definition")

    def test_adversarial_rows_and_json_safe_machine_envelope(self):
        result = run_analytic_study()
        rows = result["rows"]
        self.assertTrue(any(r["case"] == "large_station_collapse" and not r["representable"] for r in rows))
        self.assertTrue(any(r["case"] == "knot_jump" and r["side"] == "left" for r in rows))
        self.assertTrue(any(r["case"] == "lying_rate" for r in rows))
        self.assertTrue(any(r["case"] == "nonfinite" and not r["resolved"] for r in rows))
        self.assertTrue(any(r["case"] == "incompatible_domain" and not r["resolved"] for r in rows))
        json.dumps(result, allow_nan=False)

    def test_physical_residual_propagation_is_separate(self):
        result = run_analytic_study()
        row = next(r for r in result["rows"] if r["case"] == "lying_rate")
        self.assertAlmostEqual(row["propagated_uncertainty"], abs(row["speed"] / row["metric_G"]) * row["uncertainty"])
        self.assertGreater(row["physical_implied_acceleration_residual"], row["propagated_uncertainty"])
        self.assertGreater(row["exact_physical_residual"], row["propagated_uncertainty"])
        self.assertAlmostEqual(row["speed"], 20 + 0.1 * row["station"])


if __name__ == "__main__":
    unittest.main()
