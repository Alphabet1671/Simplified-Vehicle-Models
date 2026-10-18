import os
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree

from tools.sim_gui import REPOSITORY_ROOT, child_environment
from tools.sim_gui.catalogue import candidates, scan_catalogue, tracks
from tools.sim_gui.vehicle import VehicleConfiguration


class CatalogueTests(unittest.TestCase):
    def setUp(self):
        self.catalogue = scan_catalogue()
        models = REPOSITORY_ROOT / "models"
        self.by_rel = {e.path.relative_to(models).as_posix(): e for e in self.catalogue}

    def rel(self, entries):
        return {e.path.relative_to(REPOSITORY_ROOT / "models").as_posix() for e in entries}

    def test_every_model_xml_is_listed(self):
        models = REPOSITORY_ROOT / "models"
        expected = [p for p in models.rglob("*.xml") if p.relative_to(models).parts[0] != "sweeps"]
        self.assertEqual(len(self.catalogue), len(expected))

    def test_tire_candidates_share_the_capability(self):
        slot = self.by_rel["tires/iem24_murphy_real_unitire.xml"]
        self.assertEqual(self.rel(candidates(slot, self.catalogue)),
                         {"tires/iem24_murphy_real_unitire.xml", "tires/hoosier-43075-r20-simple.xml"})

    def test_capability_none_matches_type_and_subtype(self):
        slot = self.by_rel["controllers/placeholder_2024_pedal_controller.xml"]
        self.assertEqual(self.rel(candidates(slot, self.catalogue)),
                         {"controllers/iem24_murphy_real.xml", "controllers/placeholder_2024_pedal_controller.xml"})

    def test_only_signed_tracks_are_offered(self):
        self.assertNotIn("tracks/2024_michigan_endurance_v11.xml", self.rel(tracks(self.catalogue)))

    def test_child_environment_puts_src_first(self):
        parts = child_environment()["PYTHONPATH"].split(os.pathsep)
        self.assertEqual(parts[:2], [str(REPOSITORY_ROOT / "src"), str(REPOSITORY_ROOT)])


class VehicleTests(unittest.TestCase):
    FORMULATION_14 = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"

    def test_open_formulation_lists_roles_and_children(self):
        configuration = VehicleConfiguration.open(self.FORMULATION_14)
        self.assertEqual(configuration.model, "14dof-simple")
        keys = [s.key for s in configuration.slots()]
        vehicle = REPOSITORY_ROOT / "models/vehicles/iem24_murphy_real_planar.xml"
        roles = [c.get("role") for c in ElementTree.parse(vehicle).findall("./components/component")]
        self.assertEqual([k for k in keys if "/" not in k], roles)
        self.assertEqual(keys[keys.index("powertrain") + 1:keys.index("powertrain") + 3],
                         ["powertrain/power-plant", "powertrain/energy-storage"])

    def test_nesting_is_read_from_the_xml_at_any_depth(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            vehicle = root / "vehicle.xml"
            outer = root / "outer.xml"
            inner = root / "inner.xml"
            leaf = root / "leaf.xml"
            vehicle.write_text('<model type="vehicle-assembly" subtype="four-wheel"><components><component role="drivetrain" ref="outer.xml"/></components></model>')
            outer.write_text('<model type="drivetrain-assembly" subtype="generic"><components><component role="controller" ref="inner.xml"/></components></model>')
            inner.write_text('<model type="controller" subtype="generic"><components><component role="leaf" ref="leaf.xml"/></components></model>')
            leaf.write_text('<model type="controller" subtype="generic"/>')
            configuration = VehicleConfiguration.open(vehicle)
            self.assertEqual([s.key for s in configuration.slots()], ["drivetrain", "drivetrain/controller", "drivetrain/controller/leaf"])

    def test_open_assembly_defaults_to_planar(self):
        configuration = VehicleConfiguration.open(REPOSITORY_ROOT / "models/vehicles/iem24_murphy_real_planar.xml")
        self.assertIsNone(configuration.formulation)
        self.assertEqual(configuration.model, "planar-7dof")

    def test_swap_rejects_a_wrong_capability(self):
        configuration = VehicleConfiguration.open(self.FORMULATION_14)
        with self.assertRaises(ValueError):
            configuration.with_choice("tire", REPOSITORY_ROOT / "models/aero/murphy_2024.xml")

    def test_swapping_a_parent_refreshes_its_children(self):
        configuration = VehicleConfiguration.open(self.FORMULATION_14).with_choice(
            "powertrain", REPOSITORY_ROOT / "models/powertrains/murphy_2024.xml")
        child = ElementTree.parse(REPOSITORY_ROOT / "models/powertrains/murphy_2024.xml").find(
            "./components/component[@role='power-plant']").get("ref")
        expected = (REPOSITORY_ROOT / "models/powertrains" / child).resolve()
        self.assertEqual(configuration.choices["powertrain/power-plant"], expected)
        self.assertEqual(configuration.changed_from(VehicleConfiguration.open(self.FORMULATION_14))[0], "powertrain")

    def test_point_mass_formulation_opens_but_names_no_runner(self):
        configuration = VehicleConfiguration.open(REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_point_mass.xml")
        self.assertEqual(configuration.model, "point-mass")
        self.assertFalse(configuration.runnable)


if __name__ == "__main__":
    unittest.main()
