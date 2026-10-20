import hashlib
import os
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.xml.loader import load_model
from tools.gui_common import REPOSITORY_ROOT
from tools.sim_gui.catalogue import candidates, read_entry, scan_catalogue
from tools.vehicle_gui import xmltext
from tools.vehicle_gui.model import ASSEMBLY, VehicleEdit
from tools.vehicle_gui.save import (SaveRefused, check_files, render_files,
                                    save_component, save_vehicle)

MODELS = REPOSITORY_ROOT / "models"
FORMULATION = MODELS / "formulations" / "iem24_murphy_real_planar_7dof.xml"
CATALOGUE = scan_catalogue(MODELS)


def tree_hash():
    return {p.relative_to(MODELS).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in MODELS.rglob("*") if p.is_file() and p.relative_to(MODELS).parts[0] != "sweeps"}


def other_candidate(edit, key):
    current = edit.file(key)
    return next(c.path for c in candidates(read_entry(current), CATALOGUE) if c.path != current)


class SaveTests(unittest.TestCase):
    def setUp(self):
        self.edit = VehicleEdit.open(FORMULATION)
        self._folder = tempfile.TemporaryDirectory()
        self.addCleanup(self._folder.cleanup)
        self.folder = Path(self._folder.name) / "out"
        self.before = tree_hash()
        self.addCleanup(lambda: self.assertEqual(tree_hash(), self.before, "models/ changed"))

    def test_formulation_edit_saves_assembly_and_formulation(self):
        saved = save_vehicle(self.edit.with_parameter(ASSEMBLY, "cg_height", 0.315), self.folder, "t1")
        self.assertEqual(sorted(p.name for p in saved.files), ["t1__formulation.xml", "t1__vehicle.xml"])
        self.assertTrue(saved.validation.ok)
        reopened = VehicleEdit.open(saved.entry)
        self.assertEqual(next(r for r in reopened.rows(ASSEMBLY) if r.name == "cg_height").original, "0.315")
        self.assertEqual(reopened.configuration.choices["tire"], self.edit.configuration.choices["tire"])
        compile_formulation(saved.entry)

    def test_copy_keeps_comments_and_line_count(self):
        edit = self.edit.with_parameter(ASSEMBLY, "cg_height", 0.315)
        text = dict(render_files(edit, self.folder, "t1"))[self.folder.resolve() / "t1__vehicle.xml"]
        source = self.edit.file(ASSEMBLY).read_text(encoding="utf-8")
        self.assertIn("Shared IEM24 assembly", text)
        self.assertEqual(len(text.splitlines()), len(source.splitlines()) + 1)
        self.assertIn("Saved by tools/vehicle_gui from models/vehicles/iem24_murphy_real_planar.xml", text)

    def test_nested_edit_copies_the_chain(self):
        row = next(r for r in self.edit.rows("powertrain/power-plant") if r.name == "gear_ratio")
        edit = self.edit.with_parameter("powertrain/power-plant", "gear_ratio", float(row.original) * 1.05)
        saved = save_vehicle(edit, self.folder, "n1")
        self.assertEqual(sorted(p.name for p in saved.files), ["n1__formulation.xml", "n1__powertrain.xml", "n1__powertrain__power-plant.xml", "n1__vehicle.xml"])
        reopened = VehicleEdit.open(saved.entry)
        self.assertEqual(reopened.file("powertrain/power-plant").name, "n1__powertrain__power-plant.xml")
        self.assertEqual(reopened.file("tire"), self.edit.file("tire"))

    def test_the_compiler_rejects_a_fractional_unit_count(self):
        edit = self.edit.with_parameter("powertrain/power-plant", "unit_count", 2.1)
        self.assertIn("unit_count must be a positive whole number", check_files(edit).error)
        with self.assertRaises(ValueError):
            save_vehicle(edit, self.folder, "frac")
        self.assertFalse(self.folder.exists())

    def test_swap_references_the_original_file(self):
        target = other_candidate(self.edit, "aero")
        saved = save_vehicle(self.edit.with_choice("aero", target), self.folder, "s1")
        self.assertEqual(VehicleEdit.open(saved.entry).file("aero"), target)
        self.assertEqual(sorted(p.name for p in saved.files), ["s1__formulation.xml", "s1__vehicle.xml"])

    def test_every_suspension_swap_saves_and_compiles(self):
        for candidate in candidates(read_entry(self.edit.file("suspension")), CATALOGUE):
            with self.subTest(candidate=candidate.path.name):
                saved = save_vehicle(self.edit.with_choice("suspension", candidate.path), self.folder / candidate.path.stem, "w")
                self.assertEqual(VehicleEdit.open(saved.entry).file("suspension"), candidate.path)

    def test_copy_of_a_generated_file_is_not_marked_generated(self):
        edit = self.edit.with_choice("suspension/spring", self.edit.file("suspension/spring"))
        self.assertFalse(edit.changed)
        files = dict(render_files(self.edit.with_parameter(ASSEMBLY, "cg_height", 0.3), self.folder, "g2"))
        for text in files.values():
            self.assertNotIn("Do not edit", text)

    def test_a_value_the_compiler_rejects_writes_nothing(self):
        edit = self.edit.with_parameter(ASSEMBLY, "total_mass", -5.0)
        verdict = check_files(edit)
        self.assertFalse(verdict.ok)
        self.assertIn("total_mass must be positive", verdict.error)
        with self.assertRaisesRegex(ValueError, "total_mass must be positive"):
            save_vehicle(edit, self.folder, "bad")
        self.assertFalse(self.folder.exists())

    def test_validation_reports_unconsumed_parameters(self):
        verdict = check_files(self.edit)
        self.assertTrue(verdict.ok)
        self.assertIn("pitch_radius_of_gyration = 0.386 m", verdict.unconsumed)
        self.assertEqual(verdict.unused_roles, ("suspension", "tire-transient"))

    def test_existing_files_need_overwrite(self):
        edit = self.edit.with_parameter(ASSEMBLY, "cg_height", 0.315)
        save_vehicle(edit, self.folder, "o1")
        with self.assertRaisesRegex(SaveRefused, "already exists"):
            save_vehicle(edit, self.folder, "o1")
        save_vehicle(edit.with_parameter(ASSEMBLY, "cg_height", 0.32), self.folder, "o1", overwrite=True)

    def test_models_folder_is_protected(self):
        with self.assertRaisesRegex(SaveRefused, "models/"):
            save_vehicle(self.edit.with_parameter(ASSEMBLY, "cg_height", 0.315), MODELS / "vehicles", "zz_probe")
        self.assertFalse(list((MODELS / "vehicles").glob("zz_probe*")))

    def test_bad_file_name_is_refused(self):
        with self.assertRaises(ValueError):
            save_vehicle(self.edit.with_parameter(ASSEMBLY, "cg_height", 0.315), self.folder, "bad name")

    def test_later_replace_failure_rolls_back_without_partial_files(self):
        edit = self.edit.with_parameter(ASSEMBLY, "cg_height", 0.315)
        self.folder.mkdir(parents=True, exist_ok=True)
        old = {}
        for name in ("fault__vehicle.xml", "fault__formulation.xml"):
            path = self.folder / name
            path.write_bytes(b"original:" + name.encode())
            old[path] = path.read_bytes()
        real_replace, calls = os.replace, []
        def fail(source, target):
            calls.append((Path(source), Path(target)))
            if len(calls) == 6:
                raise OSError("injected replace failure")
            return real_replace(source, target)
        with mock.patch("os.replace", side_effect=fail):
            with self.assertRaisesRegex(ValueError, "injected replace failure"):
                save_vehicle(edit, self.folder, "fault", overwrite=True)
        self.assertEqual(len(calls), 8)
        self.assertEqual({path: path.read_bytes() for path in old}, old)
        self.assertEqual(sorted(p.name for p in self.folder.glob("fault__*.xml")), ["fault__formulation.xml", "fault__vehicle.xml"])

    def test_stale_same_stem_outputs_are_removed(self):
        self.folder.mkdir(parents=True, exist_ok=True)
        stale = self.folder / "stale__old-slot.xml"
        stale.write_text("old", encoding="utf-8")
        save_vehicle(self.edit.with_parameter(ASSEMBLY, "cg_height", 0.315), self.folder, "stale", overwrite=True)
        self.assertFalse(stale.exists())

    def test_staging_write_failure_leaves_no_target_files(self):
        def fail_stage(*args, **kwargs):
            raise OSError("injected staging write failure")
        with mock.patch("tools.vehicle_gui.save.atomic_write_text", side_effect=fail_stage):
            with self.assertRaisesRegex(ValueError, "injected staging write failure"):
                save_vehicle(self.edit, self.folder, "write-fault")
        self.assertFalse(self.folder.exists())

    def test_rollback_failure_does_not_mask_commit_failure(self):
        self.folder.mkdir(parents=True, exist_ok=True)
        for name in ("mask__vehicle.xml", "mask__formulation.xml"):
            (self.folder / name).write_bytes(b"original")
        real_replace, calls = os.replace, []
        def fail(source, target):
            calls.append((Path(source), Path(target)))
            if len(calls) == 6:
                raise OSError("original commit failure")
            if len(calls) == 8:
                raise OSError("rollback cleanup failure")
            return real_replace(source, target)
        with mock.patch("os.replace", side_effect=fail):
            with self.assertRaisesRegex(ValueError, "original commit failure") as caught:
                save_vehicle(self.edit, self.folder, "mask", overwrite=True)
        self.assertEqual(len(calls), 8)
        message = "\n".join(caught.exception.__notes__)
        self.assertIn("rollback incomplete; recover backups from", message)
        recovery = Path(message.rsplit(" ", 1)[-1])
        self.addCleanup(lambda: shutil.rmtree(recovery, ignore_errors=True))
        self.assertTrue((self.folder / "mask__formulation.xml").exists())
        self.assertFalse((self.folder / "mask__vehicle.xml").exists())
        self.assertEqual((recovery / ".backup" / "mask__vehicle.xml").read_bytes(), b"original")

    def test_an_unchanged_save_is_a_faithful_copy(self):
        saved = save_vehicle(self.edit, self.folder, "same")
        copy = saved.files[0].read_text(encoding="utf-8")
        source = self.edit.file(ASSEMBLY).read_text(encoding="utf-8")
        stripped = "\n".join(line for line in copy.splitlines() if "Saved by tools/vehicle_gui" not in line)
        self.assertEqual(xmltext.set_model_id(stripped, xmltext.model_id(source)).splitlines(), xmltext.reanchor(source, self.edit.file(ASSEMBLY).parent, self.folder).splitlines())


class ComponentTests(unittest.TestCase):
    def test_save_component_with_new_id(self):
        source = MODELS / "tires" / "hoosier-43075-r20-simple.xml"
        rows = xmltext.parameter_values(source.read_text(encoding="utf-8"))
        name = next(n for n, v in rows.items() if v.replace(".", "", 1).isdigit())
        with tempfile.TemporaryDirectory() as folder:
            target = save_component(source, {name: float(rows[name]) * 1.1}, Path(folder), "my_tire", "my-tire")
            self.assertEqual(load_model(target).model_id, "my-tire")
            self.assertAlmostEqual(float(load_model(target).parameters[name]), float(rows[name]) * 1.1)
            with self.assertRaises(SaveRefused):
                save_component(source, {}, Path(folder), "my_tire", "my-tire")

    def test_generated_component_cannot_be_edited(self):
        source = MODELS / "springs" / "iem24_murphy_real_suspension_springs.xml"
        with tempfile.TemporaryDirectory() as folder, self.assertRaisesRegex(ValueError, "generated"):
            save_component(source, {"x": 1.0}, Path(folder), "s", "s")

    def test_component_into_models_is_protected(self):
        with self.assertRaises(SaveRefused):
            save_component(MODELS / "tires" / "hoosier-43075-r20-simple.xml", {}, MODELS / "tires", "zz", "zz")


if __name__ == "__main__":
    unittest.main()
