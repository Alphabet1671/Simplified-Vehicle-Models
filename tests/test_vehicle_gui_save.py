import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from simplified_vehicle_models.compiler.compile import compile_formulation
from tools.gui_common import REPOSITORY_ROOT
from tools.vehicle_gui.model import ASSEMBLY, VehicleEdit
from tools.vehicle_gui.save import SaveRefused, check_files, render_files, save_vehicle

FORMULATION = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"


class SaveTests(unittest.TestCase):
    def setUp(self):
        self.edit = VehicleEdit.open(FORMULATION)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.folder = Path(self.tmp.name) / "out"

    def test_save_and_reopen(self):
        saved = save_vehicle(self.edit.with_parameter(ASSEMBLY, "cg_height", .315), self.folder, "t1")
        self.assertTrue(saved.validation.ok)
        self.assertEqual(sorted(p.name for p in saved.files), ["t1__formulation.xml", "t1__vehicle.xml"])
        self.assertEqual(VehicleEdit.open(saved.entry).file(ASSEMBLY).name, "t1__vehicle.xml")
        compile_formulation(saved.entry)

    def test_invalid_edit_writes_nothing(self):
        edit = self.edit.with_parameter(ASSEMBLY, "total_mass", -5.0)
        self.assertFalse(check_files(edit).ok)
        with self.assertRaises(ValueError):
            save_vehicle(edit, self.folder, "bad")
        self.assertFalse(self.folder.exists())

    def test_staging_failure_writes_nothing(self):
        with mock.patch("tools.vehicle_gui.save.atomic_write_text", side_effect=OSError("injected staging write failure")):
            with self.assertRaisesRegex(ValueError, "injected staging write failure"):
                save_vehicle(self.edit, self.folder, "fault")
        self.assertFalse(self.folder.exists())

    def test_commit_failure_restores_backups(self):
        self.folder.mkdir()
        for name in ("fault__vehicle.xml", "fault__formulation.xml"):
            (self.folder / name).write_bytes(b"original")
        real_replace, calls = os.replace, []
        def replace(source, target):
            calls.append((Path(source), Path(target)))
            if len(calls) == 6:
                raise OSError("injected replace failure")
            return real_replace(source, target)
        with mock.patch("os.replace", side_effect=replace):
            with self.assertRaisesRegex(ValueError, "injected replace failure"):
                save_vehicle(self.edit, self.folder, "fault", overwrite=True)
        self.assertEqual(len(calls), 8)
        self.assertEqual((self.folder / "fault__vehicle.xml").read_bytes(), b"original")
        self.assertEqual((self.folder / "fault__formulation.xml").read_bytes(), b"original")

    def test_rollback_failure_retains_recovery(self):
        self.folder.mkdir()
        for name in ("mask__vehicle.xml", "mask__formulation.xml"):
            (self.folder / name).write_bytes(b"original")
        real_replace, calls = os.replace, []
        def replace(source, target):
            calls.append((Path(source), Path(target)))
            if len(calls) == 6:
                raise OSError("original commit failure")
            if len(calls) == 8:
                raise OSError("rollback cleanup failure")
            return real_replace(source, target)
        with mock.patch("os.replace", side_effect=replace):
            with self.assertRaisesRegex(ValueError, "original commit failure") as caught:
                save_vehicle(self.edit, self.folder, "mask", overwrite=True)
        message = "\n".join(caught.exception.__notes__)
        self.assertIn("rollback incomplete; recover backups from", message)
        recovery = Path(message.rsplit(" ", 1)[-1])
        self.addCleanup(lambda: __import__("shutil").rmtree(recovery, ignore_errors=True))
        self.assertEqual((recovery / ".backup" / "mask__vehicle.xml").read_bytes(), b"original")
        self.assertTrue((self.folder / "mask__formulation.xml").exists())
        self.assertFalse((self.folder / "mask__vehicle.xml").exists())


if __name__ == "__main__":
    unittest.main()
