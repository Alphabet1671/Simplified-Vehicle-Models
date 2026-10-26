import tempfile
import unittest
from pathlib import Path
import shutil

from tools.sim_gui.maneuver_execute import (
    ManeuverStudyDefinition, create_maneuver_study, expand_maneuvers,
    prepare_maneuver_case, maneuver_child_command, finish_maneuver_case,
)
from simplified_vehicle_models.results.run_record import load_run_record


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

    def test_completed_14dof_artifact_becomes_standard_attempt_run_record(self):
        repo = Path(__file__).resolve().parents[1]
        source = repo / 'docs/validation/iem24-murphy/maneuver-14dof/step/983ad114-143b-419b-8836-9103abc27d19'
        self.assertTrue((source / 'result' / 'manifest.json').is_file())
        study = ManeuverStudyDefinition('maneuvers', 'models/formulations/iem24_murphy_real_14dof_simple.xml', (payload(),))
        with tempfile.TemporaryDirectory() as tmp:
            cases = expand_maneuvers(study)
            index = create_maneuver_study(Path(tmp) / 'study', study, cases)
            prepared = prepare_maneuver_case(index.directory, study, cases[0])
            attempt = prepared.directory / 'attempt_fixture'; attempt.mkdir()
            shutil.copytree(source / 'result', attempt / 'result')
            shutil.copyfile(source / 'summary.json', attempt / 'summary.json')
            fields = finish_maneuver_case(prepared, attempt, return_code=0, timed_out=False,
                                          cancelled=False, wall_seconds=.2, command=['python'])
            self.assertEqual(fields['status'], 'completed')
            record = index.directory / fields['run_record']
            self.assertEqual(record, attempt / 'run_record.npz')
            loaded = load_run_record(record, verify_sources=False)
            self.assertEqual(loaded.manifest.schema, 'run-record-v1')
            self.assertIn('tire-vertical-force', loaded.manifest.channel_names)


if __name__ == "__main__":
    unittest.main()
