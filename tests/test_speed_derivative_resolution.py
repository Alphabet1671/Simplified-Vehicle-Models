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
        self.assertTrue(any(r["case"] == "lying_rate_good_control" for r in rows))
        self.assertTrue(any(r["case"] == "lying_rate_bad_control" for r in rows))
        left = next(r for r in rows if r["case"] == "knot_jump" and r["side"] == "left")
        right = next(r for r in rows if r["case"] == "knot_jump" and r["side"] == "right")
        self.assertLess(abs(left["estimate"] - 0.2), 1e-10)
        self.assertLess(abs(right["estimate"] - 0.8), 1e-10)
        self.assertTrue(any(r["case"] == "nonfinite" and not r["resolved"] for r in rows))
        self.assertTrue(any(r["case"] == "incompatible_domain" and not r["resolved"] for r in rows))
        json.dumps(result, allow_nan=False)

    def test_physical_residual_propagation_is_separate(self):
        result = run_analytic_study()
        good = next(r for r in result["rows"] if r["case"] == "lying_rate_good_control")
        bad = next(r for r in result["rows"] if r["case"] == "lying_rate_bad_control")
        self.assertAlmostEqual(good["propagated_uncertainty"], abs(good["speed"] / good["metric_G"]) * good["uncertainty"])
        self.assertLess(good["physical_implied_acceleration_residual"], 1e-10)
        self.assertLess(good["exact_physical_residual"], 1e-12)
        self.assertGreater(bad["physical_implied_acceleration_residual"], bad["propagated_uncertainty"])
        self.assertGreater(bad["exact_physical_residual"], bad["propagated_uncertainty"])
        self.assertAlmostEqual(good["speed"], 20 + 0.1 * good["station"])


if __name__ == "__main__":
    unittest.main()
