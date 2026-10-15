import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class RetiredOcpEvidenceTests(unittest.TestCase):
    def test_three_way_summary_marks_ocp_unverified(self) -> None:
        summary = json.loads(
            (ROOT / "docs/validation/iem24-murphy/comparison_three_way_summary.json")
            .read_text(encoding="utf-8")
        )
        planar = summary["simulations"]["planar_7dof"]

        self.assertEqual(summary["verification_status"], "historical_unverified")
        self.assertFalse(summary["7dof_metrics_valid"])
        self.assertIsNone(summary["7dof_converged"])
        self.assertEqual(planar["verification_status"], "historical_unverified")
        self.assertIsNone(planar["converged"])

    def test_recheck_is_explicitly_nonvalidated(self) -> None:
        directory = ROOT / "docs/validation/iem24-murphy/three_way_recheck_2026-10-15"
        summary = json.loads(
            (directory / "comparison_three_way_summary.json")
            .read_text(encoding="utf-8")
        )
        ocp_summary = json.loads(
            (directory / "simulated_lap_7dof_summary.json")
            .read_text(encoding="utf-8")
        )
        report = (directory / "comparison_three_way_report.md").read_text(
            encoding="utf-8"
        )
        self.assertEqual(summary["verification_status"], "nonvalidated_historical_run")
        self.assertEqual(ocp_summary["verification_status"], "nonvalidated_historical_run")
        self.assertIn("Not validation evidence", report)
        self.assertIn("illustrative only", report)

    def test_report_discloses_retired_ocp_policy(self) -> None:
        report = (
            ROOT / "docs/validation/iem24-murphy/comparison_three_way_report.md"
        ).read_text(encoding="utf-8")
        self.assertIn("historical/unverified", report)
        self.assertIn("not a current three-way comparison", report)


if __name__ == "__main__":
    unittest.main()
