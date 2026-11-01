"""Mocked contract tests for the dedicated fixed-plan input study."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from tools.run import study_linear_driver_inputs as study


class StudyContracts(unittest.TestCase):
    def test_candidate_matrix_has_exact_map_and_slew_contrasts(self):
        expected = {
            "baseline": ([0., 6.], [0., 1.], None),
            "demand_only": ([0., 1., 2.5], [0., .5, 1.], None),
            "slew_only": ([0., 6.], [0., 1.], study.RATES),
            "combined": ([0., 1., 2.5], [0., .5, 1.], study.RATES),
        }
        for name, (axis, values, rates) in expected.items():
            payload, parsed = study._map_config(name)
            self.assertEqual(payload["demand_map"]["acceleration_to_throttle"], {"axis": axis, "value": values})
            self.assertIsNone(payload["physical_calibration"])
            self.assertEqual(parsed.demand_map.deceleration_to_brake.axis, (0., 6.))
            self.assertEqual(None if name not in ("slew_only", "combined") else study.RATES, rates)

    def test_metrics_use_left_interval_weights_and_terminal_zero_weight(self):
        rows = [
            {"time": 0., "current_speed": 0., "target_speed": 2., "pedal_lagged_applied_throttle": .5,
             "pedal_lagged_applied_brake_force": 0., "pedal": 0., "distance": 0.},
            {"time": 1., "current_speed": 2., "target_speed": 2., "pedal_lagged_applied_throttle": 1.,
             "pedal_lagged_applied_brake_force": 0., "pedal": .5, "distance": 2.},
            {"time": 3., "current_speed": 100., "target_speed": 2., "pedal_lagged_applied_throttle": 0.,
             "pedal_lagged_applied_brake_force": 0., "pedal": 0., "distance": 4.},
        ]
        m = study._metrics(rows)
        self.assertEqual(m["duration_s"], 3.)
        self.assertAlmostEqual(m["speed_error_mae_mps"], 2. / 3.)
        self.assertAlmostEqual(m["full_throttle_ge_0_95_fraction"], 2. / 3.)
        self.assertAlmostEqual(m["variation_per_simulated_second"], 1. / 3.)
        self.assertIn("left interval", m["weighting"])

    def test_prepare_serializes_full_config_and_rejects_tamper(self):
        arrays = {
            "distance": np.array([0., 150.]), "speed": np.array([6., 6.]),
            "curvature": np.zeros(2), "acceleration": np.zeros(2),
            "precursor_distance": np.array([0., 250.]), "precursor_speed": np.array([6., 6.]),
            "precursor_curvature": np.zeros(2), "precursor_acceleration": np.zeros(2),
        }
        frozen = study.fixed.FrozenPlan(arrays["distance"], arrays["speed"], arrays["curvature"], arrays["acceleration"],
            study.fixed.freeze_plan_arrays(*[arrays[n] for n in ("distance", "speed", "curvature", "acceleration")]).digest,
            arrays["precursor_distance"], arrays["precursor_speed"], 0., arrays["precursor_curvature"], arrays["precursor_acceleration"])
        compiled = mock.Mock(manifest=mock.Mock(as_text=lambda: "canonical"), model=object())
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "run"
            with mock.patch.object(study.fixed, "compile_selected_model", return_value=compiled), \
                 mock.patch.object(study, "_source_manifest", return_value={"hashes": {"x": "y"}}), \
                 mock.patch.object(study.fixed, "make_plan", return_value=frozen), \
                 mock.patch.object(study, "_arrays", return_value=arrays):
                study.prepare_study(out)
                manifest = json.loads((out / "manifest.json").read_text())
                self.assertEqual(set(manifest["candidates"]), set(study.NAMES))
                self.assertEqual(manifest["candidates"]["combined"]["pedal_map"]["physical_calibration"], None)
                manifest["candidates"]["baseline"]["pedal_time_constant_s"] = .03
                (out / "manifest.json").write_text(json.dumps(manifest))
                with self.assertRaises(ValueError): study.run_arm(out, "baseline")
            with self.assertRaises(FileExistsError): study.prepare_study(out)

    def test_arm_rejects_unknown_candidate_before_artifacts(self):
        with self.assertRaises(ValueError): study.run_arm("missing", "other")


if __name__ == "__main__":
    unittest.main()
