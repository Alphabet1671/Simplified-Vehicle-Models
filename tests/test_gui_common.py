import json
import os
import tempfile
import unittest
from pathlib import Path

from tools.gui_common import REPOSITORY_ROOT, child_environment
from tools.gui_common.files import (atomic_write_json, atomic_write_text, check_name,
                                    generated_by, is_inside, protected_directory_problem)

MODELS = REPOSITORY_ROOT / "models"
GENERATED = {
    "anti-roll-bars/iem24_murphy_real_suspension_arb.xml",
    "dampers/multimatic_dssv_vc01_front.xml",
    "dampers/multimatic_dssv_vc01_rear.xml",
    "hydraulics/iem24_roll_decoupled_circuit.xml",
    "springs/iem24_murphy_real_suspension_springs.xml",
    "suspension-kinematics/iem24_murphy_real_suspension_kinematics.xml",
    "suspensions/iem24_murphy_real_suspension.xml",
    "tracks/fsae_2026_acceleration.xml",
    "tracks/fsae_2026_skidpad.xml",
}


class EnvironmentTests(unittest.TestCase):
    def test_child_environment_puts_src_first(self):
        parts = child_environment()["PYTHONPATH"].split(os.pathsep)
        self.assertEqual(parts[:2], [str(REPOSITORY_ROOT / "src"), str(REPOSITORY_ROOT)])

    def test_child_environment_sets_a_matplotlib_folder(self):
        self.assertTrue(child_environment()["MPLCONFIGDIR"])


class NameTests(unittest.TestCase):
    def test_accepts_the_sweep_name_rule(self):
        self.assertEqual(check_name("a_b-9"), "a_b-9")

    def test_rejects_bad_names(self):
        for bad in ("", "a b", "a/b", "x" * 41, "é"):
            with self.assertRaises(ValueError, msg=bad):
                check_name(bad)


class WriteTests(unittest.TestCase):
    def test_atomic_write_replaces_and_leaves_no_temporary(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "sub" / "a.txt"
            atomic_write_text(path, "one\n")
            atomic_write_text(path, "two\n")
            self.assertEqual(path.read_text(), "two\n")
            self.assertEqual([p.name for p in path.parent.iterdir()], ["a.txt"])

    def test_atomic_json_is_sorted_and_rejects_nan(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "a.json"
            atomic_write_json(path, {"b": 1, "a": 2})
            self.assertEqual(path.read_text(), '{\n  "a": 2,\n  "b": 1\n}\n')
            with self.assertRaises(ValueError):
                atomic_write_json(path, {"x": float("nan")})
            self.assertEqual(json.loads(path.read_text()), {"a": 2, "b": 1})


class ProtectionTests(unittest.TestCase):
    def test_generated_marker_finds_exactly_the_known_files(self):
        found = {p.relative_to(MODELS).as_posix() for p in MODELS.rglob("*.xml")
                 if p.relative_to(MODELS).parts[0] != "sweeps" and generated_by(p)}
        self.assertEqual(found, GENERATED)

    def test_generated_marker_names_the_tool(self):
        self.assertEqual(generated_by(MODELS / "tracks" / "fsae_2026_skidpad.xml"),
                         "tools/tracks/build_fsae_dynamic_tracks.py")

    def test_a_hand_written_file_has_no_marker(self):
        self.assertIsNone(generated_by(MODELS / "vehicles" / "iem24_murphy_real.xml"))
        self.assertIsNone(generated_by(MODELS / "does_not_exist.xml"))

    def test_is_inside(self):
        self.assertTrue(is_inside(MODELS / "tracks", MODELS))
        self.assertFalse(is_inside(MODELS.parent / "tools", MODELS))

    def test_models_folder_needs_permission_but_sweeps_does_not(self):
        self.assertIsNotNone(protected_directory_problem(MODELS / "tracks", allow_models=False))
        self.assertIsNone(protected_directory_problem(MODELS / "tracks", allow_models=True))
        self.assertIsNone(protected_directory_problem(MODELS / "sweeps" / "x", allow_models=False))
        with tempfile.TemporaryDirectory() as folder:
            self.assertIsNone(protected_directory_problem(Path(folder), allow_models=False))


if __name__ == "__main__":
    unittest.main()
