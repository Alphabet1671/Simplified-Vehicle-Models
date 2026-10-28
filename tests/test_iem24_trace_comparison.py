import io
import json
import shutil
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.tracks.profile import load_track
from tools.reports.compare_iem24_murphy_stint import (
    _periodic_interp,
    fit_position_offset,
    flying_lap_indices,
    load_table,
    main as compare_main,
    unwrap_track_distance,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = REPOSITORY_ROOT / "docs/validation/iem24-murphy"
CLOSED_TRACK = REPOSITORY_ROOT / "models/tracks/mis_ev_endurance_2024_linear.xml"


class ComparisonHelperTests(unittest.TestCase):
    def test_periodic_interp_wraps_across_the_lap_boundary(self):
        phase = np.array([0.0, 0.25, 0.5, 0.75])
        values = np.array([0.0, 1.0, 2.0, 3.0])
        grid = np.array([0.0, 0.125, 0.875, 0.999999])
        result = _periodic_interp(phase, values, grid, 1.0)
        self.assertAlmostEqual(result[0], 0.0, places=4)
        self.assertAlmostEqual(result[1], 0.5, places=4)
        # 0.875 lies halfway between (0.75, 3.0) and the wrapped (1.0, 0.0).
        self.assertAlmostEqual(result[2], 1.5, places=4)
        self.assertAlmostEqual(result[3], 0.0, places=3)

    def test_unwrap_track_distance_is_monotonic_across_the_wrap(self):
        track_distance = np.array([900.0, 950.0, 0.0, 50.0, 0.0, 60.0])
        lap_index = np.array([5.0, 5.0, 5.0, 5.0, 6.0, 6.0])
        unwrapped = unwrap_track_distance(track_distance, 1000.0, lap_index)
        np.testing.assert_allclose(unwrapped, [900.0, 950.0, 1000.0, 1050.0, 0.0, 60.0])

    def test_flying_lap_indices_drops_the_standing_start(self):
        measured = {
            "lap_index": np.array([0.0, 0.0, 1.0, 1.0, 2.0, 2.0]),
            "speed_mps": np.array([0.0, 5.0, 12.0, 13.0, 11.0, 12.0]),
        }
        np.testing.assert_array_equal(flying_lap_indices(measured), [1.0, 2.0])

    def test_fit_position_offset_recovers_a_known_translation(self):
        path = load_track(CLOSED_TRACK).path_coordinates
        assert path is not None
        stride = 3
        measured = {
            "position_north_m": path.x_north[:-1][::stride] - 12.0,
            "position_east_m": path.y_east[:-1][::stride] + 7.0,
        }
        offset, residual = fit_position_offset(measured, path)
        self.assertAlmostEqual(offset[0], 12.0, delta=0.6)
        self.assertAlmostEqual(offset[1], -7.0, delta=0.6)
        self.assertLess(residual, 0.5)


class ComparisonIntegrationTests(unittest.TestCase):
    """Run the comparison against the committed stint and lap exports."""

    @classmethod
    def setUpClass(cls):
        cls.measured = load_table(OUTPUT_DIR / "measured_stint.csv")
        cls.summary = json.loads(
            (OUTPUT_DIR / "comparison_summary.json").read_text(encoding="utf-8")
        )

    def test_measured_export_covers_signed_body_channels(self):
        for name in (
            "speed_mps",
            "longitudinal_acceleration_mps2",
            "lateral_acceleration_mps2",
            "yaw_rate_rad_s",
            "motor_torque_request_nm",
            "pack_voltage_v",
            "position_north_m",
            "position_east_m",
        ):
            self.assertIn(name, self.measured)
        self.assertLess(float(np.min(self.measured["yaw_rate_rad_s"])), -0.5)
        self.assertGreater(float(np.max(self.measured["yaw_rate_rad_s"])), 0.5)
        self.assertLess(float(np.min(self.measured["lateral_acceleration_mps2"])), -5.0)

    def test_measured_pack_voltage_covers_the_stint_discharge(self):
        voltage = self.measured["pack_voltage_v"]
        flying = self.measured["lap_index"] > 0
        self.assertGreater(float(np.max(voltage[flying])), 560.0)
        self.assertLess(float(np.min(voltage[flying])), 530.0)

    def test_simulated_lap_starts_from_the_measured_pack_voltage(self):
        simulated = self.summary["simulated_lap"]
        self.assertAlmostEqual(simulated["initial_bus_voltage_v"], 574.42, delta=1.0)
        self.assertLess(
            simulated["minimum_bus_voltage_v"], simulated["initial_bus_voltage_v"]
        )

    def test_pack_voltage_is_compared(self):
        metrics = self.summary["metrics"]
        self.assertIn("pack_voltage_v", metrics)
        self.assertGreater(metrics["pack_voltage_v"]["correlation"], 0.4)

    def test_committed_comparison_correlates_yaw_and_speed(self):
        metrics = self.summary["metrics"]
        self.assertGreater(metrics["yaw_rate_rad_s"]["correlation"], 0.9)
        self.assertGreater(metrics["speed_mps"]["correlation"], 0.9)
        self.assertGreater(metrics["lateral_acceleration_mps2"]["correlation"], 0.9)
        self.assertLess(abs(metrics["yaw_rate_rad_s"]["bias"]), 0.05)

    def test_measured_path_is_close_to_the_centreline(self):
        fit = self.summary["position_fit"]
        self.assertLess(fit["residual_median_m"], 2.0)
        self.assertLess(fit["residual_p95_m"], 4.0)
        self.assertEqual(
            self.summary["measured_stint"]["flying_lap_count"], 10
        )

    def test_comparison_tool_regenerates_report_and_figures(self):
        with tempfile.TemporaryDirectory() as directory:
            output_dir = Path(directory)
            for name in (
                "measured_stint.csv",
                "simulated_lap.csv",
                "simulated_lap_summary.json",
                "telemetry_extraction.json",
            ):
                shutil.copyfile(OUTPUT_DIR / name, output_dir / name)
            with patch(
                "sys.argv",
                [
                    "compare_iem24_murphy_stint",
                    "--output-dir",
                    str(output_dir),
                    "--bins",
                    "120",
                    # The copied committed simulated_lap.csv was built on the
                    # original track (its summary records it), so reproduce it there;
                    # the tool's own default is now the canonical multilap track.
                    "--track",
                    "models/tracks/mis_ev_endurance_2024_linear.xml",
                ],
            ), redirect_stdout(io.StringIO()):
                compare_main()
            report = (output_dir / "report.md").read_text(encoding="utf-8")
            self.assertIn("![Measured and simulated traces](comparison.png)", report)
            self.assertIn("![Per-lap speed spread", report)
            self.assertTrue((output_dir / "comparison.png").is_file())
            self.assertTrue((output_dir / "lap_spread.png").is_file())
            regenerated = json.loads(
                (output_dir / "comparison_summary.json").read_text(encoding="utf-8")
            )
            self.assertGreater(
                regenerated["metrics"]["yaw_rate_rad_s"]["correlation"], 0.85
            )


if __name__ == "__main__":
    unittest.main()
