import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock
from xml.etree import ElementTree

import numpy as np

from tools.sim_gui import REPOSITORY_ROOT
from tools.sim_gui.vehicle import VehicleConfiguration
from tools.sim_gui.variant import write_variant
from tools.model_data.generate_component_sweep import generate_sweep, main
from simplified_vehicle_models.compiler.compile import compile_formulation

CORNERS = REPOSITORY_ROOT / "models/data/iem24_murphy_real_corners.npz"
AERO = REPOSITORY_ROOT / "models/aero/iem24_murphy_real_aero.xml"
FORMULATION_14 = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"


def corner_table_owner() -> Path:
    owners = [path for path in (REPOSITORY_ROOT / "models").rglob("*.xml")
              if "sweeps" not in path.relative_to(REPOSITORY_ROOT / "models").parts[:1]
              and any(element.get("role") == "corner-table"
                      and (path.parent / element.get("path")).resolve() == CORNERS.resolve()
                      for element in ElementTree.parse(path).getroot().findall("./data"))]
    assert len(owners) == 1, owners
    return owners[0]


SUSPENSION = corner_table_owner()


class ComponentSweepTests(unittest.TestCase):
    def test_offset_changes_only_the_target_array(self):
        with tempfile.TemporaryDirectory() as d:
            index = json.loads(generate_sweep(SUSPENSION, "data:corner-table:rc_height_front", "offset",
                                              [-0.01, 0.0, 0.01], Path(d) / "rch", name="rch", unit="m").read_text())
            with np.load(CORNERS) as base, np.load(Path(d) / "rch" / index["members"][2]["data"][0]) as member:
                self.assertEqual(member.files, base.files)
                np.testing.assert_allclose(member["rc_height_front"], base["rc_height_front"] + 0.01)
                for key in base.files:
                    if key != "rc_height_front":
                        np.testing.assert_array_equal(member[key], base[key])

    def test_member_compiles_inside_the_vehicle(self):
        configuration = VehicleConfiguration.open(FORMULATION_14)
        key = next(key for key, path in configuration.choices.items() if path.resolve() == SUSPENSION.resolve())
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d) / "sweep"
            index = json.loads(generate_sweep(SUSPENSION, "data:corner-table:rc_height_front", "offset",
                                              [-0.01, 0.0, 0.01], folder, name="rch", unit="m").read_text())
            member = folder / index["members"][2]["file"]
            changed = configuration.with_choice(key, member)
            with tempfile.TemporaryDirectory() as variant_dir:
                files = write_variant(changed, Path(variant_dir), "case_001")
                compiled = compile_formulation(files.formulation)
                loaded = [record.path.resolve() for record in compiled.manifest.loaded_components]
                self.assertIn(member.resolve(), loaded)
                # The compiled model's table is reachable through the loaded suspension component.
                self.assertAlmostEqual(compiled.model.suspension.front.roll_centre_height(0.0), 0.051275)

    def test_member_points_at_its_own_table_with_a_matching_hash(self):
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d) / "sweep"
            index = json.loads(generate_sweep(SUSPENSION, "data:corner-table:rc_height_front", "offset",
                                              [-0.01, 0.0, 0.01], folder, name="rch", unit="m").read_text())
            member = folder / index["members"][2]["file"]
            data = folder / index["members"][2]["data"][0]
            element = ElementTree.parse(member).getroot().find("./data[@role='corner-table']")
            self.assertEqual((member.parent / element.get("path")).resolve(), data.resolve())
            self.assertEqual(element.get("sha256"), hashlib.sha256(data.read_bytes()).hexdigest())

    def test_parameter_target_takes_its_unit_from_the_xml(self):
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d) / "aero"
            index = json.loads(generate_sweep(AERO, "parameter:downforce_area", "value", [3.0, 4.0], folder, name="area").read_text())
            self.assertEqual(index["unit"], "m^2")
            root = ElementTree.parse(folder / index["members"][1]["file"]).getroot()
            value = root.find("./parameters/parameter[@name='downforce_area']").get("value")
            self.assertEqual(value, "4.0")
            with self.assertRaises(ValueError):
                generate_sweep(AERO, "parameter:downforce_area", "value", [3.0], Path(d) / "bad", name="bad", unit="m")

    def test_values_on_a_curve_is_refused(self):
        with tempfile.TemporaryDirectory() as d, self.assertRaisesRegex(ValueError, "scalar"):
            generate_sweep(SUSPENSION, "data:corner-table:rc_height_front", "value", [0.05], Path(d) / "x", name="x", unit="m")

    def test_existing_folder_needs_force_and_a_sweep_json(self):
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d) / "x"; folder.mkdir(); (folder / "keep").write_text("x")
            with self.assertRaises(ValueError):
                generate_sweep(AERO, "parameter:downforce_area", "value", [3], folder, name="x")
            (folder / "sweep.json").write_text("{}")
            generate_sweep(AERO, "parameter:downforce_area", "value", [3], folder, name="x", force=True)
            self.assertTrue((folder / "sweep.json").is_file())
            folder2 = Path(d) / "y"; folder2.mkdir(); (folder2 / "keep").write_text("x")
            with self.assertRaises(ValueError):
                generate_sweep(AERO, "parameter:downforce_area", "value", [3], folder2, name="y", force=True)
            self.assertTrue((folder2 / "keep").is_file())

    def test_assembly_base_is_refused(self):
        for base in (REPOSITORY_ROOT / "models/vehicles/iem24_murphy_real_planar.xml", REPOSITORY_ROOT / "models/powertrains/murphy_2024.xml"):
            with self.subTest(base=base), tempfile.TemporaryDirectory() as d, self.assertRaisesRegex(ValueError, "base must be a leaf component"):
                generate_sweep(base, "parameter:x", "value", [1], Path(d) / "x", name="x")

    def test_output_is_byte_reproducible(self):
        with tempfile.TemporaryDirectory() as d:
            a = Path(d) / "a"; b = Path(d) / "b"
            ia = json.loads(generate_sweep(SUSPENSION, "data:corner-table:rc_height_front", "offset", [0.01], a, name="same", unit="m").read_text())
            ib = json.loads(generate_sweep(SUSPENSION, "data:corner-table:rc_height_front", "offset", [0.01], b, name="same", unit="m").read_text())
            self.assertEqual((a / ia["members"][0]["file"]).read_bytes(), (b / ib["members"][0]["file"]).read_bytes())
            self.assertEqual((a / ia["members"][0]["data"][0]).read_bytes(), (b / ib["members"][0]["data"][0]).read_bytes())
            self.assertEqual({k: v for k, v in ia.items() if k != "created_utc"}, {k: v for k, v in ib.items() if k != "created_utc"})

    def test_bad_inputs(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d)
            csv_base = out / "csv-base.xml"
            csv_base.write_text('<model schema-version="1.0" id="csv-base" type="aero-model" subtype="fixed-aero"><data role="table" path="table.csv" format="csv"/></model>')
            (out / "table.csv").write_text("x,y\\n0,1\\n")
            with self.assertRaisesRegex(ValueError, "only npz data arrays"):
                generate_sweep(csv_base, "data:table:any", "offset", [1], out / "csv", name="csv", unit="m")
            for name in ("bad name", "!", "a" * 41):
                with self.assertRaises(ValueError): generate_sweep(AERO, "parameter:downforce_area", "value", [3], out / name.replace(" ", "_"), name=name)
            for nums in ([], list(range(101)), [float("inf")]):
                with self.assertRaises(ValueError): generate_sweep(AERO, "parameter:downforce_area", "value", nums, out / ("n" + str(len(nums))), name="n" + str(len(nums)))
            with self.assertRaisesRegex(ValueError, "unit"):
                generate_sweep(SUSPENSION, "data:corner-table:rc_height_front", "offset", [1], out / "unit", name="unit")

    def test_cli_writes_folder(self):
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d) / "cli"
            argv = ["--base", str(AERO), "--set", "parameter:downforce_area", "--values", "4", "--name", "cli", "--output-dir", str(folder)]
            self.assertEqual(main(argv), 0)
            payload = json.loads((folder / "sweep.json").read_text())
            self.assertEqual(payload["command"], argv)

    def test_failed_check_removes_folder(self):
        with tempfile.TemporaryDirectory() as d, mock.patch("tools.model_data.generate_component_sweep.load_model", side_effect=RuntimeError("bad")):
            folder = Path(d) / "x"
            with self.assertRaisesRegex(RuntimeError, "bad"):
                generate_sweep(AERO, "parameter:downforce_area", "value", [3], folder, name="x")
            self.assertFalse(folder.exists())


if __name__ == "__main__":
    unittest.main()
