import tempfile
import unittest
from pathlib import Path

from tools.sim_gui.maneuver_execute import (
    ManeuverStudyDefinition, create_maneuver_study, expand_maneuvers,
    prepare_maneuver_case, maneuver_child_command,
)


def payload():
    return {
        "schema": "maneuver-v1", "name": "queue maneuver", "preset": "step_steer",
        "backend": "simulation", "vehicle": {"model": "models/formulations/iem24_murphy_real_14dof_simple.xml"},
        "initial": {"speed_mps": 10., "x_m": 0., "y_m": 0., "heading_rad": 0.},
        "clock": {"controller_tick_s": .01},
        "steering": {"mode": "peak_swa", "amplitude_rad": .1, "direction": 1},
        "longitudinal": {"mode": "constant_accelerator", "accelerator": .1},
        "profile": {"duration_s": .1}, "limits": {"max_swa_rad": .2},
        "termination": {"duration_s": .1}, "calibration": None, "csv": None,
    }


class ManeuverStudyTests(unittest.TestCase):
    def test_native_study_index_and_case_layout(self):
        study = ManeuverStudyDefinition("maneuvers", "models/formulations/iem24_murphy_real_14dof_simple.xml", (payload(),))
        cases = expand_maneuvers(study)
        with tempfile.TemporaryDirectory() as tmp:
            index = create_maneuver_study(Path(tmp) / "maneuvers", study, cases)
            self.assertEqual(index._payload["schema"], "sim-study-v1")
            self.assertEqual(index.cases[0]["id"], "case_000")
            prepared = prepare_maneuver_case(index.directory, study, cases[0])
            self.assertEqual(prepared.directory, index.directory / "cases" / "case_000")
            self.assertTrue((prepared.directory / "case.json").is_file())
            self.assertEqual(maneuver_child_command(prepared, prepared.directory / "attempt_1")[2:4],
                             ["-m", "tools.maneuver_gui.runner"])


if __name__ == "__main__":
    unittest.main()
