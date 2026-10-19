import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from tools.gates.certify_iem24_forward_between_step import main, vertical_dense_domain


class BetweenStepDispatchTests(unittest.TestCase):
    def test_unknown_model_writes_fail_closed_report(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            summary = {"model": "unknown", "usable": True}
            (root / "summary.json").write_text(json.dumps(summary))
            self.assertEqual(main([str(root)]), 1)
            report = json.loads((root / "between_step_certificate.json").read_text())
            updated = json.loads((root / "summary.json").read_text())
            self.assertFalse(report["between_step_validation_complete"])
            self.assertFalse(updated["usable"])

    def test_live_runner_planner_label_selects_live_reconstruction(self):
        settings = {"planner": "live_live_acceleration_envelope", "track": "track.xml",
                    "spacing": .5, "maximum_speed": 8., "lateral_acceleration": None}
        # The plan generator is exercised through mocks only; the exact runner label
        # must select the live path and must pass the compiled planar vehicle.
        class Vehicle:
            planar = object()
        class Planar:
            pass
        class Bounds:
            shape = (1, 39, 2)
        import numpy as np
        class Data(dict):
            pass
        fake_data = Data(bounds=np.zeros((1, 39, 2)))
        fake_track = type("Track", (), {"lap_length": 10., "path_coordinates": object()})()
        with patch("numpy.load", return_value=fake_data), \
             patch("simplified_vehicle_models.tracks.profile.load_track", return_value=fake_track), \
             patch("simplified_vehicle_models.tracks.path_coordinates.evaluate_path",
                   return_value=(None, None, None, np.zeros(21))), \
             patch("simplified_vehicle_models.solvers.forward_driver.build_speed_plan",
                   return_value=object()) as build, \
             patch("simplified_vehicle_models.solvers.vertical_dense_certificate.certify_vertical_dense_domain",
                   return_value={"certified": True}) as certify, \
             patch("simplified_vehicle_models.solvers.vertical_between_step.certify_vertical_between_step",
                   return_value={"certified": True}) as between:
            with TemporaryDirectory() as tmp:
                Path(tmp, "dense_polynomials.npz").touch()
                result = vertical_dense_domain(Vehicle(), Path(tmp), {"settings": settings})
            self.assertTrue(result["certified"])
            self.assertTrue(build.call_args.kwargs["live_limit"])
            self.assertIs(build.call_args.kwargs["vehicle"], Vehicle.planar)
            self.assertTrue(certify.call_args.kwargs["pedal_dynamics"])
            self.assertTrue(between.call_args.kwargs["pedal_dynamics"])

    def test_failed_evaluation_domain_certificate_blocks_the_combined_result(self):
        settings = {"planner": "live", "track": "track.xml", "spacing": .5, "maximum_speed": 8.}
        import numpy as np
        fake_track = type("Track", (), {"lap_length": 10., "path_coordinates": object()})()
        class Vehicle:
            planar = object()
        with patch("numpy.load", return_value={"bounds": np.zeros((1, 39, 2))}), \
             patch("simplified_vehicle_models.tracks.profile.load_track", return_value=fake_track), \
             patch("simplified_vehicle_models.tracks.path_coordinates.evaluate_path",
                   return_value=(None, None, None, np.zeros(21))), \
             patch("simplified_vehicle_models.solvers.forward_driver.build_speed_plan",
                   return_value=object()), \
             patch("simplified_vehicle_models.solvers.vertical_dense_certificate.certify_vertical_dense_domain",
                   return_value={"certified": True}), \
             patch("simplified_vehicle_models.solvers.vertical_between_step.certify_vertical_between_step",
                   return_value={"certified": False}):
            with TemporaryDirectory() as tmp:
                Path(tmp, "dense_polynomials.npz").touch()
                result = vertical_dense_domain(Vehicle(), Path(tmp), {"settings": settings})
        self.assertFalse(result["certified"])


if __name__ == "__main__":
    unittest.main()
