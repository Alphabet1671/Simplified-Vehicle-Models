import unittest
from pathlib import Path

from tools.run.benchmark_rollout import benchmark

ROOT = Path(__file__).resolve().parents[1]


class BenchmarkTests(unittest.TestCase):
    def test_short_planar_benchmark_reports_rates(self):
        result = benchmark(
            ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml",
            ROOT / "models/tracks/fsc_may_endurance_2026_width.xml",
            start_distance=0.0,
            duration=0.5,
            method="radau-baseline",
            wall_limit=30.0,
        )
        self.assertTrue(result["completed"], result.get("failure"))
        self.assertGreater(result["distance"], 1.0)
        self.assertGreater(result["rhs_evaluations"], 0)
        self.assertAlmostEqual(
            result["wall_per_sim_second"],
            result["wall_seconds"] / result["simulated_seconds"],
        )


if __name__ == "__main__":
    unittest.main()
