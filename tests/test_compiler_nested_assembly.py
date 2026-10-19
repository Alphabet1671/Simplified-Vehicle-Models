"""Nested components: a registered model type may hold child models.

The compiler used to special-case the powertrain and drivetrain assemblies. A
model type now declares ``nests_components`` and ``child_roles`` at
registration, and the compiler compiles the children, hands their values to the
parent's compile function, and records each child in the manifest under a role
path such as ``widget/alpha``.

The fixture types ``test-widget`` and ``test-leaf`` are test only. The
production bootstrap does not know them.
"""

from __future__ import annotations

import hashlib
import os
import tempfile
import unittest
from pathlib import Path

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.compiler.registry import (
    component_descriptor,
    register_component,
)
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import require_model_type

ROOT = Path(__file__).resolve().parents[1]
POINT_MASS_ASSEMBLY = ROOT / "models/vehicles/iem24_murphy_real.xml"

WIDGET_CAPABILITY = "test-widget-capability"


def compile_leaf(model: Model) -> float:
    require_model_type(model, "test-leaf", "scalar")
    return float(model.parameters["gain"])


def compile_widget(model: Model, children: dict[str, object]) -> dict[str, object]:
    require_model_type(model, "test-widget", "pair")
    return dict(children)


register_component("test-leaf", "scalar", compile_function=compile_leaf)
register_component(
    "test-widget",
    "pair",
    capability=WIDGET_CAPABILITY,
    compile_function=compile_widget,
    nests_components=True,
    child_roles=("alpha", "beta"),
)


def _leaf_xml(model_id: str, gain: float) -> str:
    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<model schema-version="1.0" id="{model_id}" type="test-leaf" subtype="scalar">\n'
        f'  <parameters><parameter name="gain" value="{gain}" unit="1"/></parameters>\n'
        "</model>\n"
    )


def _widget_xml(children: list[tuple[str, str]]) -> str:
    lines = "".join(f'  <component role="{role}" ref="{ref}"/>\n' for role, ref in children)
    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<model schema-version="1.0" id="test-widget-pair" type="test-widget" subtype="pair">\n'
        f"{lines}</model>\n"
    )


class NestedAssemblyTests(unittest.TestCase):
    def setUp(self):
        self._directory = tempfile.TemporaryDirectory()
        self.addCleanup(self._directory.cleanup)
        # Resolve the directory: on macOS the temp root is a symlink, and the
        # relative paths below must climb from the real location.
        self.directory = Path(self._directory.name).resolve()

    def _write(self, name: str, text: str) -> Path:
        path = self.directory / name
        path.write_text(text)
        return path

    def _formulation(self, widget_children: list[tuple[str, str]]) -> Path:
        """Write the IEM24 point-mass car plus one nested ``widget`` component."""

        self._write("alpha.xml", _leaf_xml("test-leaf-alpha", 2.0))
        self._write("beta.xml", _leaf_xml("test-leaf-beta", 3.0))
        self._write("widget.xml", _widget_xml(widget_children))
        source = POINT_MASS_ASSEMBLY.read_text()
        prefix = os.path.relpath(POINT_MASS_ASSEMBLY.parent, self.directory)
        source = source.replace('ref="../', f'ref="{prefix}/../')
        source = source.replace(
            "</components>", '<component role="widget" ref="widget.xml"/>\n    </components>'
        )
        self.assertIn('role="widget"', source)
        assembly = self._write("assembly.xml", source)
        return self._write(
            "formulation.xml",
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            '<model schema-version="1.0" id="test-nested-formulation" '
            'type="vehicle-formulation" subtype="point-mass">\n'
            f'  <source ref="{assembly.name}"/>\n'
            '  <requires capability="tire-force-envelope"/>\n'
            '  <requires capability="aero-loads"/>\n'
            '  <requires capability="drive-force-limit"/>\n'
            f'  <requires capability="{WIDGET_CAPABILITY}"/>\n'
            "</model>\n",
        )

    def test_registered_flags_are_exposed_on_the_descriptor(self):
        widget = component_descriptor("test-widget", "pair")
        self.assertTrue(widget.nests_component)
        self.assertFalse(widget.nests_assembly)
        self.assertEqual(widget.child_roles, ("alpha", "beta"))

    def test_powertrain_and_drivetrain_are_registered_as_nesting_assemblies(self):
        from simplified_vehicle_models.powertrain import assembly

        for key in (
            (assembly.POWERTRAIN_ASSEMBLY_TYPE, assembly.POWERTRAIN_SUBTYPE),
            (assembly.DRIVETRAIN_ASSEMBLY_TYPE, assembly.DRIVETRAIN_SUBTYPE),
        ):
            descriptor = component_descriptor(*key)
            self.assertTrue(descriptor.nests_assembly, key)

    def test_nested_child_values_reach_the_parent_compile_function(self):
        formulation = self._formulation([("alpha", "alpha.xml"), ("beta", "beta.xml")])
        compiled = compile_formulation(formulation)
        self.assertEqual(compiled.manifest.capability(WIDGET_CAPABILITY).component_role, "widget")
        widget = next(
            component
            for component in compile_formulation_loaded(compiled)
            if component.role == "widget"
        )
        self.assertEqual(widget.value, {"alpha": 2.0, "beta": 3.0})

    def test_manifest_lists_every_child_file_and_hash(self):
        formulation = self._formulation([("alpha", "alpha.xml"), ("beta", "beta.xml")])
        manifest = compile_formulation(formulation).manifest
        roles = [record.role for record in manifest.loaded_components]
        self.assertIn("widget", roles)
        self.assertIn("widget/alpha", roles)
        self.assertIn("widget/beta", roles)
        for role, name in (("widget/alpha", "alpha.xml"), ("widget/beta", "beta.xml")):
            record = manifest.component(role)
            self.assertEqual(record.path, (self.directory / name).resolve())
            self.assertEqual(record.model_type, "test-leaf")
            digest = hashlib.sha256(record.path.read_bytes()).hexdigest()
            self.assertEqual(len(digest), 64)
        self.assertEqual(manifest.unused_components, ())

    def test_a_duplicate_child_role_fails(self):
        formulation = self._formulation(
            [("alpha", "alpha.xml"), ("alpha", "beta.xml"), ("beta", "beta.xml")]
        )
        with self.assertRaisesRegex(ValueError, "duplicate child role 'alpha'"):
            compile_formulation(formulation)

    def test_a_missing_child_role_fails_and_names_the_role(self):
        formulation = self._formulation([("alpha", "alpha.xml")])
        with self.assertRaisesRegex(ValueError, "missing child role 'beta'"):
            compile_formulation(formulation)

    def test_an_unknown_child_role_fails(self):
        formulation = self._formulation(
            [("alpha", "alpha.xml"), ("beta", "beta.xml"), ("gamma", "beta.xml")]
        )
        with self.assertRaisesRegex(ValueError, "unknown child role 'gamma'"):
            compile_formulation(formulation)


class RegistrationValidationTests(unittest.TestCase):
    def test_child_roles_require_the_nesting_flag(self):
        with self.assertRaisesRegex(ValueError, "child_roles require nests_components"):
            register_component("test-bad", "a", child_roles=("x",))

    def test_a_nesting_component_must_name_its_children(self):
        with self.assertRaisesRegex(ValueError, "must name its child roles"):
            register_component("test-bad", "b", nests_components=True)

    def test_duplicate_child_roles_in_a_registration_fail(self):
        with self.assertRaisesRegex(ValueError, "duplicate child role"):
            register_component("test-bad", "c", nests_components=True, child_roles=("x", "x"))

    def test_a_formulation_cannot_nest(self):
        with self.assertRaisesRegex(ValueError, "cannot be registered as nesting"):
            register_component(
                "test-bad", "d", kind="formulation", nests_components=True, child_roles=("x",)
            )


def compile_formulation_loaded(compiled):
    """Return the loaded components of a compile through the manifest-free path.

    ``CompiledFormulation`` keeps only the manifest, so the test recompiles the
    nested component to read its value.
    """

    from simplified_vehicle_models.compiler.compile import compile_component
    from simplified_vehicle_models.compiler.registry import ComponentCandidate

    manifest = compiled.manifest
    loaded = []
    for record in manifest.loaded_components:
        if record.role != "widget":
            continue
        from simplified_vehicle_models.xml.loader import load_model

        model = load_model(record.path)
        descriptor = component_descriptor(model.model_type, model.subtype)
        loaded.append(compile_component(ComponentCandidate("widget", model, descriptor)))
    return loaded


if __name__ == "__main__":
    unittest.main()
