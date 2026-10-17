"""Four-way MIS 2024 report tool: metrics, alignment and blocker checks.

Every test uses small synthetic inputs (a few samples, a two-lap fake stint) or
the synthetic attempt fixture built here; none needs the 334 MB measured MAT.
That file is only used by the report tool for a provenance hash and the
optional logged-power channels, and ``test_source_line_*`` pins both branches of
the sentence that reports the file's presence or absence.
"""
from __future__ import annotations

import hashlib
import json
import shutil
import tempfile
import unittest
from pathlib import Path

import numpy as np

from tools.reports import compare_iem24_four_way as four_way
from tools.reports import export_forward_attempt_channels as exporter
from tools.reports.compare_iem24_four_way import ReportBlocker

ROOT = Path(__file__).resolve().parents[1]
TRACK = ROOT / four_way.TRACK
TRACK_CSV = TRACK.with_suffix(".csv")
VEHICLE_XML = ROOT / "models/vehicles/iem24_murphy_real_planar.xml"


def channel_table(count: int = 12, *, distance_end: float = 100.0, speed: float = 10.0) -> dict:
    """A minimal, well-formed channels table on one synthetic distance axis."""
    distance = np.linspace(0.0, distance_end, count)
    time = distance / speed
    return {
        "distance_m": distance,
        "elapsed_time_s": time,
        "velocity_mps": np.full(count, speed),
        "longitudinal_acceleration_mps2": np.zeros(count),
        "lateral_acceleration_mps2": np.linspace(-2.0, 2.0, count),
        "yaw_rate_rad_s": np.linspace(-0.1, 0.1, count),
        "motor_torque_request_nm": np.linspace(0.0, 10.0, count),
        "pack_voltage_v": np.linspace(575.0, 565.0, count),
        "rl_power_kw": np.linspace(0.0, 5.0, count),
        "rr_power_kw": np.linspace(0.0, 5.0, count),
        "apps_fraction": np.linspace(0.0, 1.0, count),
        "soc": np.linspace(0.829, 0.80, count),
    }


class ChannelTableTests(unittest.TestCase):
    def test_well_formed_table_passes_the_distance_and_finiteness_gates(self):
        table = channel_table(distance_end=100.0)
        four_way.check_channels(table, where="synthetic", track_length=100.0,
                                required=four_way.required_columns("planar_7dof"))

    def test_point_mass_and_forward_models_use_different_speed_columns(self):
        forward = four_way.required_columns("planar_7dof")
        point = four_way.required_columns("point_mass")
        self.assertIn("velocity_mps", forward)
        self.assertIn("speed_mps", point)
        self.assertNotIn("velocity_mps", point)

    def test_missing_column_is_a_blocker(self):
        table = channel_table()
        del table["yaw_rate_rad_s"]
        with self.assertRaisesRegex(ReportBlocker, "lacks"):
            four_way.check_channels(table, where="synthetic", track_length=100.0,
                                    required=four_way.required_columns("planar_7dof"))

    def test_non_finite_value_is_a_blocker(self):
        table = channel_table()
        table["pack_voltage_v"][5] = np.nan
        with self.assertRaisesRegex(ReportBlocker, "non-finite"):
            four_way.check_channels(table, where="synthetic", track_length=100.0,
                                    required=four_way.required_columns("planar_7dof"))

    def test_non_monotonic_clock_or_distance_is_a_blocker(self):
        table = channel_table()
        table["elapsed_time_s"][4] = table["elapsed_time_s"][3]
        with self.assertRaisesRegex(ReportBlocker, "elapsed time"):
            four_way.check_channels(table, where="synthetic", track_length=100.0,
                                    required=four_way.required_columns("planar_7dof"))
        table = channel_table()
        table["distance_m"][7] = table["distance_m"][6] - 1.0
        with self.assertRaisesRegex(ReportBlocker, "distance is not strictly increasing"):
            four_way.check_channels(table, where="synthetic", track_length=100.0,
                                    required=four_way.required_columns("planar_7dof"))

    def test_trace_that_misses_the_track_length_is_a_blocker(self):
        table = channel_table(distance_end=98.0)
        with self.assertRaisesRegex(ReportBlocker, "canonical"):
            four_way.check_channels(table, where="synthetic", track_length=100.0,
                                    required=four_way.required_columns("planar_7dof"))

    def test_normalise_channels_reports_the_six_comparison_channels(self):
        table = channel_table()
        data = four_way.normalise_channels(table, "planar_7dof")
        for channel in four_way.CHANNEL_KEYS:
            self.assertIn(channel, data)
        np.testing.assert_array_equal(data["speed_mps"], table["velocity_mps"])
        point = four_way.normalise_channels({**table, "speed_mps": table["velocity_mps"]},
                                            "point_mass")
        np.testing.assert_array_equal(point["speed_mps"], table["velocity_mps"])


class MetricsTests(unittest.TestCase):
    """The metric table is a pure function of a profile and the measured median."""

    def setUp(self):
        self.median = {"speed_mps": np.linspace(10.0, 20.0, 50)}
        self.columns = ("speed_mps",)

    def test_identical_profile_scores_zero_error_and_unit_correlation(self):
        metrics = four_way.score_profile(dict(self.median), self.median, self.columns)["speed_mps"]
        self.assertEqual(metrics["rmse"], 0.0)
        self.assertEqual(metrics["mae"], 0.0)
        self.assertEqual(metrics["bias"], 0.0)
        self.assertAlmostEqual(metrics["correlation"], 1.0, places=12)
        self.assertEqual(metrics["normalised_rmse"], 0.0)

    def test_constant_offset_moves_only_the_bias_and_mae(self):
        profile = {"speed_mps": self.median["speed_mps"] + 1.5}
        metrics = four_way.score_profile(profile, self.median, self.columns)["speed_mps"]
        self.assertAlmostEqual(metrics["bias"], 1.5, places=12)
        self.assertAlmostEqual(metrics["mae"], 1.5, places=12)
        self.assertAlmostEqual(metrics["rmse"], 1.5, places=12)
        self.assertAlmostEqual(metrics["correlation"], 1.0, places=12)

    def test_bias_sign_is_simulated_minus_measured(self):
        profile = {"speed_mps": self.median["speed_mps"] - 0.25}
        metrics = four_way.score_profile(profile, self.median, self.columns)["speed_mps"]
        self.assertLess(metrics["bias"], 0.0)
        self.assertAlmostEqual(metrics["bias"], -0.25, places=12)

    def test_mutation_of_one_sample_moves_rmse_but_keeps_the_sign(self):
        profile = {"speed_mps": self.median["speed_mps"].copy()}
        profile["speed_mps"][10] += 4.0
        metrics = four_way.score_profile(profile, self.median, self.columns)["speed_mps"]
        self.assertGreater(metrics["rmse"], 0.5)
        self.assertGreater(metrics["mae"], 0.0)
        self.assertGreater(metrics["bias"], 0.0)
        self.assertLess(metrics["rmse"], 0.6)

    def test_normalised_rmse_divides_by_the_measured_standard_deviation(self):
        profile = {"speed_mps": self.median["speed_mps"] + 2.0}
        metrics = four_way.score_profile(profile, self.median, self.columns)["speed_mps"]
        self.assertAlmostEqual(metrics["normalised_rmse"],
                               2.0 / float(np.std(self.median["speed_mps"])), places=12)

    def test_flat_measured_trace_has_no_normalised_rmse(self):
        median = {"speed_mps": np.full(20, 12.0)}
        with np.errstate(invalid="ignore", divide="ignore"):
            metrics = four_way.score_profile({"speed_mps": np.full(20, 13.0)}, median,
                                             ("speed_mps",))["speed_mps"]
        self.assertIsNone(metrics["normalised_rmse"])


class CumulativeTimeTests(unittest.TestCase):
    """Measured and simulated laps must land on one physical distance axis."""

    def setUp(self):
        self.lap_length = 100.0
        bins = 10
        self.laps = np.array([1.0, 2.0])
        self.grid = (np.arange(bins, dtype=float) + 0.5) / bins * self.lap_length
        # Two identical laps on one 100 m line at 10 m/s. The projected distance
        # stops at 90 m so no lap has a duplicate phase at the seam (a real
        # projection never repeats distance 0 at both ends of one lap).
        phase = np.arange(10, dtype=float) * 10.0
        self.projected = np.concatenate((phase, phase))
        clock = 100.0 + phase / 10.0
        self.measured = {"lap_index": np.array([1] * 10 + [2] * 10, dtype=float),
                         "time_s": np.concatenate((clock, clock + 500.0))}

    def test_each_lap_is_referenced_to_its_own_start(self):
        times = four_way.measured_cumulative_time(self.measured, self.laps, self.projected,
                                                  self.grid, self.lap_length)
        np.testing.assert_allclose(times, self.grid / 10.0, atol=1e-9)

    def test_mutating_one_lap_clock_shifts_the_median_by_half_the_offset(self):
        measured = {key: value.copy() for key, value in self.measured.items()}
        # The second lap is 50 % slower within the lap (its own start is unchanged).
        measured["time_s"][10:] = 600.0 + 1.5 * (measured["time_s"][10:] - 600.0)
        times = four_way.measured_cumulative_time(measured, self.laps, self.projected,
                                                  self.grid, self.lap_length)
        np.testing.assert_allclose(times[:9], 1.25 * self.grid[:9] / 10.0, atol=1e-9)
        self.assertGreater(float(times[-1]), 11.5)  # the last bin extrapolates past 90 m

    def test_a_clock_that_stops_short_of_the_seam_is_extrapolated_not_wrapped(self):
        phase, tau = np.arange(10, dtype=float) * 10.0, np.arange(10, dtype=float)
        folded = four_way.fold_lap_clock(phase, tau, self.grid, 100.0)
        self.assertTrue(np.all(np.diff(folded) > 0.0))
        np.testing.assert_allclose(folded, self.grid / 10.0, atol=1e-9)

    def test_time_gap_is_model_time_minus_measured_median_time(self):
        median_time = four_way.measured_cumulative_time(self.measured, self.laps, self.projected,
                                                        self.grid, self.lap_length)
        model_time = self.grid / 10.0 - 1.0  # a model one second faster everywhere
        gap = model_time - median_time
        self.assertTrue(np.allclose(gap, -1.0))
        self.assertLess(float(gap[-1]), 0.0)

    def test_folded_speed_profile_is_the_median_of_the_two_laps(self):
        measured = {"lap_index": self.measured["lap_index"],
                    "speed_mps": np.concatenate((np.full(10, 10.0), np.full(10, 14.0)))}
        grid, folded = four_way.fold_laps(measured, self.laps, ("speed_mps",),
                                          track_distance=self.projected,
                                          lap_length=self.lap_length, bins=10)
        np.testing.assert_allclose(grid, self.grid)
        np.testing.assert_allclose(np.median(folded["speed_mps"], axis=0), 12.0)


class SourceLineTests(unittest.TestCase):
    """The report never depends on the 334 MB MAT being present."""

    def test_present_file_is_hashed_and_sized(self):
        text = four_way.source_line({"source": "/tmp/mat"}, {
            "source_sha256": "ab" * 32, "source_bytes": 123, "mat_present": True})
        self.assertIn("ab" * 32, text)
        self.assertIn("123 bytes", text)

    def test_absent_file_falls_back_to_the_recorded_hash(self):
        text = four_way.source_line({"source": "/tmp/mat"}, {
            "source_sha256": None, "source_bytes": None, "mat_present": False,
            "source_recorded_sha256": "cd" * 32, "source_recorded_bytes": 334513192})
        self.assertIn("cd" * 32, text)
        self.assertIn("334513192", text)
        self.assertIn("not present", text)


class PointMassProvenanceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp)

    def make_run(self, *, track=four_way.TRACK, converged=True, energy_converged=True,
                 formulation=four_way.POINT_MASS_FORMULATION, lap_time=65.0) -> Path:
        """A synthetic run whose trace clock always ends at 65 s (the recorded lap
        time is separate, so a provenance mismatch can be constructed)."""
        run = self.tmp / "point_mass"
        run.mkdir()
        table = channel_table(count=11, distance_end=1009.195)
        table["elapsed_time_s"] = np.linspace(0.0, 65.0, 11)
        np.savetxt(run / "simulated_lap.csv",
                   np.column_stack([table["distance_m"], table["elapsed_time_s"],
                                    table["velocity_mps"], table["longitudinal_acceleration_mps2"],
                                    table["lateral_acceleration_mps2"], table["yaw_rate_rad_s"],
                                    table["motor_torque_request_nm"], table["pack_voltage_v"],
                                    table["rl_power_kw"], table["rr_power_kw"],
                                    table["apps_fraction"]]),
                   delimiter=",", comments="",
                   header="distance_m,elapsed_time_s,speed_mps,longitudinal_acceleration_mps2,"
                          "lateral_acceleration_mps2,yaw_rate_rad_s,motor_torque_request_nm,"
                          "pack_voltage_v,rl_power_kw,rr_power_kw,throttle_fraction")
        summary = {"method": "energy-coupled minimum-time point-mass flying lap",
                   "converged": converged, "energy_converged": energy_converged,
                   "regen_enabled": False, "track": str(track), "lap_time_s": lap_time,
                   "track_length_m": 1009.195, "formulation": str(formulation),
                   "track_sha256": four_way.sha256_file(ROOT / track),
                   "geometry_sha256": four_way.sha256_file(ROOT / track.replace(".xml", ".csv"))}
        (run / "summary.json").write_text(json.dumps(summary))
        return run

    def test_current_model_and_track_is_accepted(self):
        info = four_way.verify_point_mass_run(self.make_run())
        self.assertEqual(info["track"], four_way.TRACK)
        for role in ("track_xml", "track_csv", "tire_xml", "formulation_xml", "vehicle_xml"):
            self.assertIn(role, info["hashes"])

    def test_wrong_track_is_a_blocker(self):
        with self.assertRaisesRegex(ReportBlocker, "not"):
            four_way.verify_point_mass_run(self.make_run(track=four_way.LEGACY_TRACK))

    def test_unconverged_solve_is_a_blocker(self):
        with self.assertRaisesRegex(ReportBlocker, "did not converge"):
            four_way.verify_point_mass_run(self.make_run(converged=False))

    def test_tampered_track_hash_is_a_blocker(self):
        run = self.make_run()
        summary = json.loads((run / "summary.json").read_text())
        summary["track_sha256"] = "0" * 64
        (run / "summary.json").write_text(json.dumps(summary))
        with self.assertRaisesRegex(ReportBlocker, "track hash"):
            four_way.verify_point_mass_run(run)

    def test_wrong_formulation_is_a_blocker(self):
        with self.assertRaisesRegex(ReportBlocker, "formulation"):
            four_way.verify_point_mass_run(self.make_run(formulation="models/other.xml"))

    def test_trace_clock_that_disagrees_with_the_summary_is_a_blocker(self):
        with self.assertRaisesRegex(ReportBlocker, "does not end"):
            four_way.verify_point_mass_run(self.make_run(lap_time=70.0))

    def test_gate_failure_is_not_an_export_blocker(self):
        """A bad lap may still be reported; only provenance and convergence bar it."""
        run = self.make_run()
        summary = json.loads((run / "summary.json").read_text())
        summary["converged"] = True
        (run / "summary.json").write_text(json.dumps(summary))
        four_way.verify_point_mass_run(run)  # must not raise


class ForwardExportTests(unittest.TestCase):
    """The established planar exporter must stay byte-identical; this one is additive."""

    #: Frozen hash of tools/reports/export_iem24_forward_correlation.py. A change here
    #: means the established exporter moved: update this constant only deliberately,
    #: because the report claims its output is unchanged.
    ESTABLISHED_EXPORTER_SHA256 = (
        "c6833dffc877be88fc84ae843c835db9e4609d8d1820eea6314ed9f00e66e7c6")

    def test_established_planar_exporter_source_is_unchanged(self):
        path = ROOT / "tools/reports/export_iem24_forward_correlation.py"
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        self.assertEqual(digest, self.ESTABLISHED_EXPORTER_SHA256,
                         "tools/reports/export_iem24_forward_correlation.py changed; its "
                         "planar export is the established reference for the four-way report")

    def test_export_module_does_not_import_the_established_exporter(self):
        import ast
        source = (ROOT / "tools/reports/export_forward_attempt_channels.py").read_text()
        imported = set()
        for node in ast.walk(ast.parse(source)):
            if isinstance(node, ast.Import):
                imported.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module)
        self.assertNotIn("tools.reports.export_iem24_forward_correlation", imported)
        self.assertFalse([name for name in imported
                          if "export_iem24_forward_correlation" in name])

    def test_channel_columns_cover_every_report_channel_plus_soc(self):
        columns = set(exporter.CHANNELS)
        for channel in four_way.CHANNEL_KEYS:
            self.assertIn(four_way.source_columns("planar_7dof")[channel], columns)
        self.assertIn("soc", columns)
        self.assertIn("apps_fraction", columns)

    def test_wheel_speed_columns_follow_either_plant_layout(self):
        self.assertEqual(exporter.wheel_speed_columns(("omega_fl", "omega_fr", "omega_rl", "omega_rr")),
                         (0, 1, 2, 3))
        names = ("s", "soc", "wheel_speed_fl", "wheel_speed_rr", "wheel_speed_rl", "wheel_speed_fr")
        self.assertEqual(exporter.wheel_speed_columns(names), (2, 5, 4, 3))
        with self.assertRaises(exporter.ExportBlocker):
            exporter.wheel_speed_columns(("s", "soc"))


if __name__ == "__main__":
    unittest.main()
