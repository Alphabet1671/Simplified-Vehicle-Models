"""Vertical (39-column) refinement checker on synthetic attempts."""
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant
from tools.gates.check_iem24_forward_refinement_vertical import compare

NAMES = tuple(VerticalForwardPlant.state_names)
COL = {name: 1 + i for i, name in enumerate(NAMES)}


def make_attempt(root, rtol, step, shift=None, **setting_overrides):
    path = Path(root)
    path.mkdir(parents=True, exist_ok=True)
    count = 200
    trace = np.zeros((count, 1 + len(NAMES)))
    trace[:, 0] = np.linspace(0., 60., count)
    trace[:, COL["s"]] = np.linspace(0., 1000., count)
    trace[:, COL["v_x"]] = 15.
    trace[:, COL["steering"]] = 0.02
    for name, value in (shift or {}).items():
        trace[1:, COL[name]] += value  # the initial state must stay identical
    np.savetxt(path / "trace.csv", trace, delimiter=",", header=",".join(("time",) + NAMES))
    np.savetxt(path / "pedal.csv", np.zeros((3, 2)), delimiter=",", header="time,pedal", comments="")
    settings = {"planner": "live", "maximum_speed": 60., "lateral_acceleration": None,
                "pedal_travel_time": .175, "spacing": .5, "track": "t.xml", "rtol": rtol,
                "maximum_step": step, "preview_seconds": .2, "heading_gain": 2.5, "lateral_gain": 1.5}
    settings.update(setting_overrides)
    (path / "summary.json").write_text(json.dumps({
        "model": "14dof-simple", "lap_completed": True, "source_hashes": {"a": "1"},
        "settings": settings}))
    return path


class VerticalRefinementCheckerTests(unittest.TestCase):
    def check(self, shift=None, **overrides):
        with tempfile.TemporaryDirectory() as root:
            reference = make_attempt(Path(root) / "a", 1e-5, .01)
            refined = make_attempt(Path(root) / "b", 5e-6, .005, shift, **overrides)
            return compare(reference, refined)

    def test_identical_trajectories_validate(self):
        self.assertTrue(self.check()["refinement_validated"])

    def test_small_tire_state_difference_within_tolerance_validates(self):
        self.assertTrue(self.check({"lateral_deformation_fl": 1e-9})["refinement_validated"])

    def test_tire_state_beyond_periodic_tolerance_is_rejected(self):
        result = self.check({"trail_angle_rr": 1.0})
        self.assertFalse(result["refinement_validated"])
        self.assertEqual(result["worst_tire_state"], "trail_angle_rr")

    def test_body_speed_beyond_limit_is_rejected(self):
        self.assertFalse(self.check({"v_x": .05})["refinement_validated"])

    def test_different_steering_gain_is_refused(self):
        with self.assertRaises(ValueError):
            self.check(heading_gain=1.5)

    def test_non_tighter_settings_are_refused(self):
        with tempfile.TemporaryDirectory() as root:
            reference = make_attempt(Path(root) / "a", 1e-5, .01)
            refined = make_attempt(Path(root) / "b", 1e-5, .01)
            with self.assertRaises(ValueError):
                compare(reference, refined)

    def test_planar_trace_layout_is_refused(self):
        with tempfile.TemporaryDirectory() as root:
            reference = make_attempt(Path(root) / "a", 1e-5, .01)
            refined = make_attempt(Path(root) / "b", 5e-6, .005)
            np.savetxt(refined / "trace.csv", np.zeros((5, 14)), delimiter=",", header="x")
            with self.assertRaises(ValueError):
                compare(reference, refined)


if __name__ == "__main__":
    unittest.main()
