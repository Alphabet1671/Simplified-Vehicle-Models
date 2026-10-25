"""Integration tests for validated maneuver-animation bridge."""
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import MappingProxyType, SimpleNamespace

import numpy as np

from simplified_vehicle_models.controls.maneuvers.types import RobotCommand
from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant
from tools.maneuver_gui.artifacts import write_artifacts
from tools.maneuver_gui.export_animation import main


class ManeuverAnimationBridgeTests(unittest.TestCase):
    def artifact(self, root: Path, *, outcome="completed"):
        names = tuple(VerticalForwardPlant.state_names)
        definition = {
            "schema": "maneuver-v1", "name": "bridge fixture", "preset": "step_steer",
            "backend": "simulation",
            "vehicle": {"formulation": "models/formulations/iem24_murphy_real_14dof_simple.xml"},
            "initial": {"speed_mps": 16.6666666667, "x_m": 0., "y_m": 0., "heading_rad": 0.},
            "clock": {"controller_tick_s": .01},
            "steering": {"mode": "peak_swa", "amplitude_rad": .01},
            "longitudinal": {"mode": "constant_accelerator", "accelerator": .1},
            "profile": {"duration_s": .1}, "limits": {}, "termination": {"duration_s": .1},
            "calibration": {}, "csv": None,
        }
        time = np.array([0., .1])
        state = np.zeros((2, 38))
        state[:, 3] = 16.6666666667
        rows = tuple(MappingProxyType({"position-x": [float(i)], "position-y": [0.],
                                       "heading-world": [0.], "speed": [16.6666666667]}) for i in range(2))
        result = SimpleNamespace(time_s=time, state=state, state_rate=np.zeros_like(state),
            applied_commands=(RobotCommand(0., .1, 0.), RobotCommand(.01, .1, 0.)),
            channels=rows, execution_outcome=outcome, termination_reason="duration")
        artifact = root / "artifact"; artifact.mkdir()
        write_artifacts(artifact, definition, result, {}, model_kind="14dof-simple")
        return artifact

    def test_dry_run_reports_plan_without_writing(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); artifact = self.artifact(root); output = root / "video.mp4"
            capture = io.StringIO()
            with contextlib.redirect_stdout(capture):
                code = main(["--artifact", str(artifact), "--output", str(output), "--dry-run"])
            self.assertEqual(code, 0)
            text = capture.getvalue()
            self.assertIn(str(artifact), text)
            self.assertIn("run record:", text)
            self.assertIn("animator command:", text)
            self.assertFalse(output.exists())
            self.assertFalse((root / "video.runrec").exists())

    def test_noncompleted_artifact_rejected_without_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); artifact = self.artifact(root, outcome="failed"); output = root / "video.mp4"
            with contextlib.redirect_stderr(io.StringIO()):
                code = main(["--artifact", str(artifact), "--output", str(output)])
            self.assertEqual(code, 2)
            self.assertFalse(output.exists())
            self.assertFalse((root / "video.runrec").exists())

    def test_png_sequence_smoke(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); artifact = self.artifact(root); output = root / "placeholder.mp4"; frames = root / "frames"
            code = main(["--artifact", str(artifact), "--output", str(output), "--png-sequence", str(frames),
                         "--frame-rate", "2", "--width", "320", "--height", "180"])
            self.assertEqual(code, 0)
            self.assertTrue((root / "placeholder.runrec").is_file())
            self.assertTrue(any(frames.glob("*.png")))


if __name__ == "__main__":
    unittest.main()
