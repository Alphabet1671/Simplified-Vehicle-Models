import os
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree

from tools.sim_gui import REPOSITORY_ROOT, child_environment
from tools.sim_gui.catalogue import candidates, scan_catalogue, tracks
from tools.sim_gui.vehicle import VehicleConfiguration
from tools.sim_gui.variant import VariantFiles, write_variant
from tools.sim_gui.settings import RunSettings, runner_argv, validate
from tools.sim_gui.study import StudyDefinition, StudyIndex, TooManyCases, Dimension, expand, dimension_from_sweep
from simplified_vehicle_models.compiler.compile import compile_formulation


class SettingsTests(unittest.TestCase):
    def test_reference_preset_arguments(self):
        settings = RunSettings(timeout_seconds=900.0)
        argv = runner_argv(settings, "14dof-simple", REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_closed.xml")
        from tools.run.run_iem24_forward_driver import parse_args
        parsed = parse_args(argv)
        self.assertEqual(parsed.model, "14dof-simple")
        self.assertEqual(parsed.planner, "live")
        self.assertEqual(parsed.maximum_speed, 60.0)
        self.assertEqual(parsed.timeout_seconds, 900.0)
        self.assertEqual(parsed.duration, 300.0)
        self.assertEqual(parsed.pedal_travel_time, 0.175)
        self.assertEqual(parsed.maximum_step, 0.008)
        self.assertEqual(parsed.rtol, 1e-5)
        self.assertEqual(parsed.method, "BDF")
        self.assertEqual(parsed.spacing, 0.5)
        self.assertTrue(parsed.record)

    def test_model_defaults_and_unknown_json(self):
        self.assertEqual(RunSettings.for_model("planar-7dof").timeout_seconds, 300.0)
        self.assertEqual(RunSettings.for_model("14dof-simple").timeout_seconds, 3600.0)
        with self.assertRaisesRegex(ValueError, "unknown run setting"):
            RunSettings.from_json({"unknown": 1})

    def test_validation(self):
        track = REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_closed.xml"
        with self.assertRaisesRegex(ValueError, "wall limit"):
            validate(RunSettings(timeout_seconds=301), "planar-7dof", track)
        with self.assertRaisesRegex(ValueError, "quasi-static attitude applies to planar-7dof only"):
            validate(RunSettings(quasi_static_attitude=True), "14dof-simple", track)

    def test_json_round_trip(self):
        self.assertEqual(RunSettings.from_json(RunSettings().to_json()), RunSettings())


class StudyTests(unittest.TestCase):
    def definition(self, mode):
        return StudyDefinition("test", "", "track-a", "planar-7dof", {"tire": "a"}, RunSettings(), mode, (Dimension("tire", ("a", "b")), Dimension("model", ("planar-7dof", "14dof-simple"))))

    def test_expansion_modes_and_labels(self):
        one = expand(self.definition("one-at-a-time"))
        self.assertEqual(len(one), 3)
        self.assertEqual(one[0].label, "baseline")
        self.assertEqual(one[1].label, "tire=b")
        self.assertEqual(one[2].label, "model=14dof-simple")
        self.assertEqual(len(expand(self.definition("full-factorial"))), 4)

    def test_five_cases_when_no_dimension_matches_baseline(self):
        definition = StudyDefinition("x", "", "t", "point-mass", {"tire": "x"}, RunSettings(), "one-at-a-time", (Dimension("tire", ("a", "b")), Dimension("model", ("planar-7dof", "14dof-simple"))))
        self.assertEqual(len(expand(definition)), 5)

    def test_case_cap(self):
        definition = StudyDefinition("x", "", "t", "m", {}, RunSettings(), "full-factorial", (Dimension("x", tuple(str(i) for i in range(501))),))
        with self.assertRaises(TooManyCases):
            expand(definition)

    def test_duplicate_key(self):
        with self.assertRaisesRegex(ValueError, "slot tire is already swept"):
            StudyDefinition("x", "", "t", "m", {}, RunSettings(), "one-at-a-time", (Dimension("tire", ("a",)), Dimension("tire", ("b",))))

    def test_index_update_is_atomic(self):
        with tempfile.TemporaryDirectory() as d:
            index = StudyIndex.create(Path(d), self.definition("one-at-a-time"), expand(self.definition("one-at-a-time")))
            old = index.cases[1]
            index.update("case_001", status="completed")
            self.assertEqual(index.cases[0]["status"] if "status" in index.cases[0] else None, None)
            self.assertEqual(index.cases[1]["status"], "completed")
            self.assertFalse((Path(d) / "study.json.tmp").exists())

    def test_parent_dimension_refreshes_children(self):
        source = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"
        baseline = VehicleConfiguration.open(source)
        choices = {key: path.relative_to(REPOSITORY_ROOT).as_posix()
                   for key, path in baseline.choices.items()}
        from tools.model_data.xml_references import rewrite_references

        with tempfile.TemporaryDirectory() as d:
            folder = Path(d)
            powertrain = folder / "powertrain.xml"
            powerplant = folder / "powerplant.xml"
            for original, output, model_id in (
                (REPOSITORY_ROOT / "models/powertrains/murphy_2024.xml", powertrain, "test-powertrain"),
                (REPOSITORY_ROOT / "models/power-plants/amk_dd5_100c.xml", powerplant, "test-powerplant"),
            ):
                tree = ElementTree.parse(original)
                tree.getroot().set("id", model_id)
                rewrite_references(tree, original.parent, folder,
                                   {"power-plant": powerplant} if output == powertrain else None)
                tree.write(output, encoding="unicode")
            definition = StudyDefinition(
                "parent", source.as_posix(), "track", "14dof-simple", choices,
                RunSettings(), "one-at-a-time", (Dimension("powertrain", (choices["powertrain"], str(powertrain))),),
            )
            cases = expand(definition)
            self.assertEqual(len(cases), 2)
            self.assertEqual(cases[0].choices, choices)
            self.assertEqual(cases[1].choices["powertrain/power-plant"], str(powerplant.resolve()))
            self.assertEqual(cases[1].choices["powertrain/energy-storage"],
                             "models/energy-storages/murphy_2024_battery.xml")
            for key, value in cases[0].choices.items():
                if not key.startswith("powertrain"):
                    self.assertEqual(cases[1].choices[key], value)

    def test_dimension_from_sweep_reads_members(self):
        source = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"
        baseline = VehicleConfiguration.open(source)
        tire = Path(baseline.choices["tire"])
        from tools.model_data.xml_references import rewrite_references
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d)
            files = []
            for index, x in enumerate((0.9, 1.1)):
                output = folder / f"tire__0{index}.xml"
                tree = ElementTree.parse(tire)
                tree.getroot().set("id", f"sweep-tire-{index}")
                rewrite_references(tree, tire.parent, folder)
                tree.write(output, encoding="unicode")
                files.append(output.name)
            (folder / "sweep.json").write_text(__import__("json").dumps({
                "schema": "component-sweep-v1", "target": "parameter:mu_0_x",
                "operation": "scale", "unit": "1", "members": [
                    {"index": 0, "file": files[0], "data": [], "label": "mu x0.9", "x": 0.9},
                    {"index": 1, "file": files[1], "data": [], "label": "mu x1.1", "x": 1.1},
                ],
            }))
            dimension = dimension_from_sweep(folder, baseline)
            self.assertEqual(dimension.key, "tire")
            self.assertEqual(dimension.values, tuple(str(path.resolve()) for path in (folder / f for f in files)))
            self.assertEqual(dimension.labels, ("mu x0.9", "mu x1.1"))
            self.assertEqual(dimension.sweep, str(folder.resolve()))
            self.assertEqual(dimension.points[1], {"sweep": str(folder.resolve()), "index": 1,
                                                   "target": "parameter:mu_0_x", "operation": "scale", "x": 1.1, "unit": "1"})
            choices = {key: path.relative_to(REPOSITORY_ROOT).as_posix() for key, path in baseline.choices.items()}
            definition = StudyDefinition("sweep", source.as_posix(), "track", "14dof-simple", choices,
                                         RunSettings(), "one-at-a-time", (dimension,))
            cases = expand(definition)
            self.assertEqual(len(cases), 3)
            self.assertEqual(cases[2].label, "tire=mu x1.1")
            self.assertEqual(cases[2].sweep_points["tire"]["x"], 1.1)
            self.assertEqual(cases[0].sweep_points, {})

    def test_sweep_that_fits_no_slot_is_refused(self):
        configuration = VehicleConfiguration.open(REPOSITORY_ROOT / "models/vehicles/iem24_murphy_real_planar.xml")
        # A friction-ellipse tire has no matching slot; the vehicle requires steady-combined-slip.
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d)
            source = REPOSITORY_ROOT / "models/tires/iem24_murphy_real_tire.xml"
            member = folder / "member.xml"
            member.write_text(source.read_text())
            (folder / "sweep.json").write_text(__import__("json").dumps({
                "schema": "component-sweep-v1", "members": [{"index": 0, "file": "member.xml", "label": "x", "x": 1}],
            }))
            with self.assertRaises(ValueError):
                dimension_from_sweep(folder, configuration)


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


class VariantTests(unittest.TestCase):
    FORMULATION_14 = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"
    PLANAR_FORMULATION = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"

    def test_baseline_variant_compiles_to_same_components(self):
        configuration = VehicleConfiguration.open(self.FORMULATION_14)
        with tempfile.TemporaryDirectory() as d:
            files = write_variant(configuration, Path(d), "case_000")
            base = compile_formulation(self.FORMULATION_14).manifest
            case = compile_formulation(files.formulation).manifest
            self.assertEqual([(c.role, c.path) for c in case.loaded_components if c.role != "powertrain_drivetrain"],
                             [(c.role, c.path) for c in base.loaded_components if c.role != "powertrain_drivetrain"])

    def test_tire_swap_reaches_compiled_model(self):
        hoosier = REPOSITORY_ROOT / "models/tires/hoosier-43075-r20-simple.xml"
        configuration = VehicleConfiguration.open(self.PLANAR_FORMULATION).with_choice("tire", hoosier)
        with tempfile.TemporaryDirectory() as d:
            files = write_variant(configuration, Path(d), "case_001")
            roles = {c.role: c.path for c in compile_formulation(files.formulation).manifest.loaded_components}
            self.assertEqual(roles["tire"], hoosier.resolve())
            self.assertEqual(ElementTree.parse(files.vehicle).getroot().get("id"), "iem24-murphy-real-planar.case-001")

    def test_changed_child_writes_nested_assembly(self):
        configuration = VehicleConfiguration.open(self.PLANAR_FORMULATION).with_choice(
            "powertrain/power-plant", REPOSITORY_ROOT / "models/power-plants/placeholder_2024_motor.xml")
        with tempfile.TemporaryDirectory() as d:
            write_variant(configuration, Path(d), "case_002")
            self.assertTrue((Path(d) / "powertrain.xml").is_file())
            self.assertFalse((Path(d) / "drivetrain.xml").exists())

    def test_two_level_nested_swap_writes_all_changed_ancestors(self):
        with tempfile.TemporaryDirectory() as source_dir:
            source = Path(source_dir)
            vehicle = source / "vehicle.xml"
            outer = source / "outer.xml"
            inner = source / "inner.xml"
            old_leaf = source / "old-leaf.xml"
            new_leaf = source / "new-leaf.xml"
            sibling = source / "sibling.xml"
            vehicle.write_text(
                '<model type="vehicle-assembly" subtype="four-wheel"><components>'
                '<component role="drivetrain" ref="outer.xml"/>'
                '<component role="unchanged" ref="sibling.xml"/>'
                '</components></model>')
            outer.write_text(
                '<model type="drivetrain-assembly" subtype="electric-four-wheel"><components>'
                '<component role="controller" ref="inner.xml"/>'
                '<component role="sibling" ref="sibling.xml"/>'
                '</components></model>')
            inner.write_text(
                '<model type="controller-model" subtype="pedal-map"><components>'
                '<component role="leaf" ref="old-leaf.xml"/>'
                '</components></model>')
            old_leaf.write_text('<model type="controller-model" subtype="pedal-map" id="old-leaf"/>')
            new_leaf.write_text('<model type="controller-model" subtype="pedal-map" id="new-leaf"/>')
            sibling.write_text('<model type="controller-model" subtype="pedal-map" id="sibling"/>')
            configuration = VehicleConfiguration.open(vehicle).with_choice(
                "drivetrain/controller/leaf", new_leaf)
            with tempfile.TemporaryDirectory() as output_dir:
                output = Path(output_dir)
                files = write_variant(configuration, output, "case_004")
                self.assertTrue((output / "drivetrain.xml").is_file())
                self.assertTrue((output / "drivetrain__controller.xml").is_file())
                self.assertFalse((output / "unchanged.xml").exists())
                vehicle_root = ElementTree.parse(files.vehicle).getroot()
                drivetrain_ref = vehicle_root.find("./components/component[@role='drivetrain']").get("ref")
                drivetrain_path = (output / drivetrain_ref).resolve()
                self.assertEqual(drivetrain_path, (output / "drivetrain.xml").resolve())
                outer_root = ElementTree.parse(drivetrain_path).getroot()
                controller_ref = outer_root.find("./components/component[@role='controller']").get("ref")
                controller_path = (output / controller_ref).resolve()
                self.assertEqual(controller_path, (output / "drivetrain__controller.xml").resolve())
                inner_root = ElementTree.parse(controller_path).getroot()
                leaf_ref = inner_root.find("./components/component[@role='leaf']").get("ref")
                self.assertEqual((controller_path.parent / leaf_ref).resolve(), new_leaf.resolve())
                sibling_ref = outer_root.find("./components/component[@role='sibling']").get("ref")
                self.assertEqual((drivetrain_path.parent / sibling_ref).resolve(), sibling.resolve())
                self.assertTrue(vehicle_root.get("id").endswith(".case-004"))
                self.assertTrue(outer_root.get("id").endswith(".case-004"))
                self.assertTrue(inner_root.get("id").endswith(".case-004"))

    def test_model_change_drops_requires(self):
        configuration = VehicleConfiguration.open(self.FORMULATION_14).with_model("planar-7dof")
        with tempfile.TemporaryDirectory() as d:
            root = ElementTree.parse(write_variant(configuration, Path(d), "case_003").formulation).getroot()
            self.assertEqual(root.get("subtype"), "planar-7dof")
            self.assertEqual(root.findall("requires"), [])

if __name__ == "__main__":
    unittest.main()
