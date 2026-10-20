import tempfile
import unittest
from pathlib import Path

from simplified_vehicle_models.xml.loader import load_model
from tools.gui_common import REPOSITORY_ROOT
from tools.vehicle_gui import xmltext as X

MODELS = REPOSITORY_ROOT / "models"
FILES = sorted(p for p in MODELS.rglob("*.xml") if p.relative_to(MODELS).parts[0] != "sweeps")
SAMPLE = """<?xml version="1.0" encoding="UTF-8"?>
<!-- header <parameter name="total_mass" value="1" unit="kg" /> -->
<model schema-version="1.0" id="one" type="vehicle-assembly">
    <parameters>
        <parameter name="total_mass" value="287.0" unit="kg" />
        <parameter name="cg_height" value="0.300" unit="m"/>
    </parameters>
    <components>
        <component role="tire" ref="../tires/a.xml" />
        <component role="aero" ref="../aero/b.xml"/>
    </components>
    <data path="x.npz" />
</model>
"""


class EditorTests(unittest.TestCase):
    def test_reads_values_and_units_outside_comments(self):
        self.assertEqual(X.parameter_values(SAMPLE), {"total_mass": "287.0", "cg_height": "0.300"})
        self.assertEqual(X.parameter_units(SAMPLE), {"total_mass": "kg", "cg_height": "m"})

    def test_set_parameters_changes_only_the_value(self):
        new = X.set_parameters(SAMPLE, {"total_mass": 300.0})
        old_lines, new_lines = SAMPLE.splitlines(), new.splitlines()
        changed = [i for i, (a, b) in enumerate(zip(old_lines, new_lines)) if a != b]
        self.assertEqual(len(changed), 1)
        self.assertEqual(new_lines[changed[0]], '        <parameter name="total_mass" value="300.0" unit="kg" />')
        self.assertIn('<parameter name="cg_height" value="0.300" unit="m"/>', new)
        self.assertIn('<parameter name="total_mass" value="1" unit="kg" />', new)

    def test_unknown_and_non_finite_are_refused(self):
        with self.assertRaisesRegex(ValueError, "no such parameter: nope"):
            X.set_parameters(SAMPLE, {"nope": 1.0})
        for bad in (float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                X.set_parameters(SAMPLE, {"total_mass": bad})

    def test_numbers_round_trip_exactly(self):
        for value in (0.1, 1 / 3, 1e-9, 12345.678901234567, 1e22):
            self.assertEqual(float(X.parameter_values(X.set_parameters(SAMPLE, {"total_mass": value}))["total_mass"]), value)

    def test_model_id(self):
        self.assertEqual(X.model_id(SAMPLE), "one")
        self.assertIn('id="two"', X.set_model_id(SAMPLE, "two"))
        for bad in ("", 'a"b', "a<b", "a&b"):
            with self.assertRaises(ValueError):
                X.set_model_id(SAMPLE, bad)

    def test_component_and_source_refs(self):
        new = X.set_component_ref(SAMPLE, "aero", "../aero/c.xml")
        self.assertIn('<component role="aero" ref="../aero/c.xml"/>', new)
        self.assertIn('<component role="tire" ref="../tires/a.xml" />', new)
        with self.assertRaisesRegex(ValueError, "no component with role"):
            X.set_component_ref(SAMPLE, "zzz", "a.xml")
        with self.assertRaises(ValueError):
            X.set_component_ref(SAMPLE, "aero", 'a".xml')
        with self.assertRaisesRegex(ValueError, "no source"):
            X.set_source_ref(SAMPLE, "a.xml")
        self.assertIn('<source ref="v.xml" />', X.set_source_ref('<model id="f"><source ref="o.xml" /></model>', "v.xml"))

    def test_edge_cases_comments_missing_refs_quotes_and_root(self):
        text = """<!-- <model id='comment'/> <component role='x' ref='bad'/> -->
<model id='root'> <component role='x' ref = '../a.xml'/> <source ref = 's.xml'/> <parameters><parameter name='p' value = '1' unit = 'm'/></parameters>
  <wrapper><model id='nested'/></wrapper>
</model>"""
        self.assertEqual(X.model_id(text), "root")
        with self.assertRaises(ValueError):
            X.model_id("<models id='wrong'/>")
        self.assertIn("<model id='changed'>", X.set_model_id(text, "changed"))
        self.assertEqual(X.parameter_values(text), {"p": "1"})
        self.assertIn("value = '2.0'", X.set_parameters(text, {"p": 2.0}))
        self.assertIn("ref = '../b.xml'", X.set_component_ref(text, "x", "../b.xml"))
        with self.assertRaisesRegex(ValueError, "no ref"):
            X.set_component_ref("<model id='x'><component role='x'/></model>", "x", "a.xml")
        with self.assertRaisesRegex(ValueError, "no ref"):
            X.set_source_ref("<model id='x'><source/></model>", "a.xml")

    def test_reanchor_keeps_targets(self):
        with tempfile.TemporaryDirectory() as folder:
            source, target = Path(folder) / "models" / "vehicles", Path(folder) / "out" / "deep"
            new = X.reanchor(SAMPLE, source, target)
            self.assertIn('ref="../../models/tires/a.xml"', new)
            self.assertIn('path="../../models/vehicles/x.npz"', new)
            self.assertEqual(X.reanchor(SAMPLE, source, source), SAMPLE)


class CorpusTests(unittest.TestCase):
    def test_every_model_file_is_a_no_op_for_the_editor(self):
        for path in FILES:
            text = path.read_text(encoding="utf-8")
            with self.subTest(path=path.name):
                self.assertEqual(X.reanchor(text, path.parent, path.parent), text)
                self.assertEqual(X.set_model_id(text, X.model_id(text)), text)

    def test_every_numeric_parameter_keeps_its_value_when_rewritten(self):
        checked = 0
        for path in FILES:
            text = path.read_text(encoding="utf-8")
            numeric = {}
            for name, value in X.parameter_values(text).items():
                try:
                    numeric[name] = float(value)
                except ValueError:
                    continue
            if not numeric:
                continue
            new = X.set_parameters(text, numeric)
            after = X.parameter_values(new)
            for name, value in numeric.items():
                self.assertEqual(float(after[name]), value, f"{path.name}:{name}")
                checked += 1
            self.assertEqual(len(new.splitlines()), len(text.splitlines()), path.name)
        self.assertGreater(checked, 300)

    def test_edited_file_loads_with_the_model_loader(self):
        source = MODELS / "vehicles" / "iem24_murphy_real_planar.xml"
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder) / "v.xml"
            text = X.reanchor(source.read_text(encoding="utf-8"), source.parent, target.parent)
            target.write_text(X.set_parameters(X.set_model_id(text, "edited"), {"total_mass": 300.0}), encoding="utf-8")
            model = load_model(target)
            self.assertEqual(model.model_id, "edited")
            self.assertEqual(float(model.parameters["total_mass"]), 300.0)


if __name__ == "__main__":
    unittest.main()
