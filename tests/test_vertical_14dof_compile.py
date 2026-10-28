"""Test that the 14dof-simple formulation compiles the vertical vehicle parameters."""

from __future__ import annotations

import shutil
import tempfile
import unittest
from pathlib import Path

from simplified_vehicle_models.compiler.adapters import CapabilityResolutionError
from simplified_vehicle_models.compiler.capabilities import (
    AERO_LOADS,
    DRIVE_FORCE_LIMIT,
    STEADY_COMBINED_SLIP,
    SUSPENSION_FORCES,
    TRANSIENT_COMBINED_SLIP,
)
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.vertical_14dof import (
    VERTICAL_14DOF_ONLY_PARAMETER_UNITS,
    VERTICAL_14DOF_REQUESTED_CAPABILITIES,
    VerticalVehicleParameters,
)
from simplified_vehicle_models.parameters.vehicle import PlanarVehicleParameters
from simplified_vehicle_models.suspension.components import CompiledDamper
from simplified_vehicle_models.suspension.network import CompiledSuspension
from simplified_vehicle_models.tires.unitire_transient import TransientTireParameters
from simplified_vehicle_models.tires.unitire_simple import loaded_radius

ROOT = Path(__file__).resolve().parents[1]
FORMULATION = ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"
PLANAR_FORMULATION = ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
NEW_ROLES = ("suspension",)


class Vertical14DofCompileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.compiled = compile_formulation(FORMULATION)
        cls.vehicle = cls.compiled.model
        cls.manifest = cls.compiled.manifest

    def test_compiles_to_composed_vertical_parameters(self) -> None:
        vehicle = self.vehicle
        self.assertIsInstance(vehicle, VerticalVehicleParameters)
        self.assertIsInstance(vehicle.planar, PlanarVehicleParameters)
        self.assertIsInstance(vehicle.suspension, CompiledSuspension)
        self.assertIsInstance(vehicle.transient_tire, TransientTireParameters)
        dampers = vehicle.suspension.network.dampers
        self.assertEqual(len(dampers), 2)
        self.assertTrue(all(isinstance(d, CompiledDamper) for d in dampers))

    def test_planar_part_equals_the_planar_formulation(self) -> None:
        planar = compile_formulation(PLANAR_FORMULATION).model
        for field in PlanarVehicleParameters.__dataclass_fields__:
            if field == "manifest":
                continue
            ours, theirs = getattr(self.vehicle.planar, field), getattr(planar, field)
            if isinstance(ours, (int, float)):
                self.assertEqual(ours, theirs, field)
        self.assertEqual(self.vehicle.planar.tire, planar.tire)
        self.assertEqual(self.vehicle.planar.aero, planar.aero)
        self.assertEqual(self.vehicle.planar.yaw_inertia, planar.yaw_inertia)

    def test_manifest_is_attached_to_both_layers(self) -> None:
        self.assertIs(self.vehicle.manifest, self.manifest)
        self.assertIs(self.vehicle.planar.manifest, self.manifest)

    def test_sprung_mass_and_centre_of_gravity(self) -> None:
        vehicle = self.vehicle
        self.assertEqual(vehicle.sprung_mass, 235.0)
        self.assertAlmostEqual(vehicle.sprung_cg_height, 0.3211, delta=5.0e-5)
        self.assertAlmostEqual(vehicle.sprung_front_fraction, (287 * 0.45 - 26) / 235, places=12)
        # Moment balances reproduce the total mass, front share and height.
        unsprung = vehicle.total_unsprung_mass
        r_free = float(loaded_radius(vehicle.planar.tire, 0.0))
        self.assertEqual(vehicle.unsprung_cg_height, r_free)
        self.assertAlmostEqual(
            vehicle.sprung_mass * vehicle.sprung_cg_height + unsprung * r_free,
            vehicle.planar.total_mass * vehicle.planar.cg_height,
            places=10,
        )
        self.assertAlmostEqual(
            vehicle.sprung_mass * vehicle.sprung_front_fraction + 26.0,
            vehicle.planar.total_mass * vehicle.planar.front_mass_fraction,
            places=10,
        )

    def test_inertias(self) -> None:
        self.assertAlmostEqual(self.vehicle.roll_inertia, 8.75, delta=0.01)
        self.assertAlmostEqual(self.vehicle.pitch_inertia, 35.01, delta=0.01)
        self.assertAlmostEqual(self.vehicle.roll_inertia, 235.0 * 0.193**2, places=12)
        self.assertAlmostEqual(self.vehicle.pitch_inertia, 235.0 * 0.386**2, places=12)

    def test_assembly_scalars(self) -> None:
        vehicle = self.vehicle
        self.assertEqual(vehicle.unsprung_mass_front, 13.0)
        self.assertEqual(vehicle.unsprung_mass_rear, 13.0)
        self.assertEqual(vehicle.roll_radius_of_gyration, 0.193)
        self.assertEqual(vehicle.pitch_radius_of_gyration, 0.386)
        self.assertEqual(vehicle.tire_vertical_damping, 400.0)
        self.assertEqual(vehicle.planar.wheel_inertia, 0.2064512)

    def test_requested_capabilities_are_complete_and_resolved(self) -> None:
        self.assertEqual(
            VERTICAL_14DOF_REQUESTED_CAPABILITIES,
            (
                STEADY_COMBINED_SLIP,
                AERO_LOADS,
                DRIVE_FORCE_LIMIT,
                SUSPENSION_FORCES,
                TRANSIENT_COMBINED_SLIP,
            ),
        )
        resolved = tuple(record.capability for record in self.manifest.capabilities)
        self.assertEqual(resolved, VERTICAL_14DOF_REQUESTED_CAPABILITIES)

    def test_suspension_is_one_capability_with_nested_children(self) -> None:
        self.assertEqual(self.manifest.capability(SUSPENSION_FORCES).component_role, "suspension")
        roles = {record.role: record for record in self.manifest.loaded_components}
        for child in ("kinematics", "spring", "arb", "damper-front", "damper-rear"):
            self.assertIn(f"suspension/{child}", roles)
        self.assertNotEqual(roles["suspension/damper-front"].model_id,
                            roles["suspension/damper-rear"].model_id)
        dampers = self.vehicle.suspension.network.dampers
        self.assertIsNot(dampers[0], dampers[1])

    def test_steady_and_transient_tires_share_role_via_adapter(self) -> None:
        self.assertEqual(self.manifest.capability(STEADY_COMBINED_SLIP).component_role, "tire")
        self.assertEqual(self.manifest.capability(TRANSIENT_COMBINED_SLIP).component_role, "tire")
        self.assertEqual(self.manifest.capability(STEADY_COMBINED_SLIP).adapter_names, ("transient_to_steady",))
        self.assertEqual(sum(record.role == "tire" for record in self.manifest.loaded_components), 1)

    def test_no_unused_components_or_unconsumed_parameters(self) -> None:
        self.assertEqual(self.manifest.unused_components, ())
        self.assertEqual(self.manifest.unconsumed_parameters, ())
        self.assertEqual(
            {record.role for record in self.manifest.loaded_components} >= set(NEW_ROLES), True
        )

    def test_parameter_units_cover_the_new_scalars(self) -> None:
        self.assertEqual(
            VERTICAL_14DOF_ONLY_PARAMETER_UNITS,
            {
                "unsprung_mass_front": "kg",
                "unsprung_mass_rear": "kg",
                "roll_radius_of_gyration": "m",
                "pitch_radius_of_gyration": "m",
                "tire_vertical_damping": "N*s/m",
            },
        )


class PlanarFormulationUnaffectedTests(unittest.TestCase):
    def test_planar_lists_exactly_the_vertical_only_names_and_compiles_none_of_them(self) -> None:
        compiled = compile_formulation(PLANAR_FORMULATION)
        manifest = compiled.manifest
        self.assertEqual(manifest.unused_roles(), NEW_ROLES)
        self.assertEqual(
            set(manifest.unconsumed_names()), set(VERTICAL_14DOF_ONLY_PARAMETER_UNITS)
        )
        self.assertEqual(
            {capability.capability for capability in manifest.capabilities},
            {STEADY_COMBINED_SLIP, AERO_LOADS, DRIVE_FORCE_LIMIT},
        )
        loaded = {record.role for record in manifest.loaded_components}
        self.assertTrue(loaded.isdisjoint(NEW_ROLES))
        self.assertIsInstance(compiled.model, PlanarVehicleParameters)


class RejectionTests(unittest.TestCase):
    """Copy the model tree and break one input at a time."""

    def setUp(self) -> None:
        self.directory = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.directory, ignore_errors=True)
        self.models = self.directory / "models"
        shutil.copytree(ROOT / "models", self.models)

    def edit(self, relative: str, old: str, new: str) -> None:
        path = self.models / relative
        text = path.read_text()
        self.assertIn(old, text)
        path.write_text(text.replace(old, new))

    def compile(self):
        return compile_formulation(self.models / "formulations/iem24_murphy_real_14dof_simple.xml")

    def test_copy_compiles_unchanged(self) -> None:
        self.assertEqual(self.compile().model.sprung_mass, 235.0)

    def test_nonzero_lateral_radius_correction_is_rejected(self) -> None:
        for name, old in (
            ("k_rl_1", '<parameter name="k_rl_1" value="0.0"'),
            ("f_y_shift_fz", '<parameter name="f_y_shift_fz" value="0.0"'),
        ):
            with self.subTest(name=name):
                self.setUp()
                self.edit("tires/iem24_murphy_real_unitire_transient.xml", old, old.replace('"0.0"', '"1e-9"'))
                with self.assertRaisesRegex(ValueError, name):
                    self.compile()

    def test_unsprung_mass_that_exhausts_the_car_is_rejected(self) -> None:
        self.edit("vehicles/iem24_murphy_real_planar.xml", 'name="unsprung_mass_front" value="13.0"', 'name="unsprung_mass_front" value="80.0"')
        with self.assertRaises(ValueError):
            self.compile()

    def test_nonpositive_scalars_are_rejected(self) -> None:
        for name, old in (
            ("tire_vertical_damping", "400.0"),
            ("roll_radius_of_gyration", "0.193"),
            ("pitch_radius_of_gyration", "0.386"),
        ):
            with self.subTest(name=name):
                self.setUp()
                self.edit("vehicles/iem24_murphy_real_planar.xml", f'name="{name}" value="{old}"', f'name="{name}" value="0.0"')
                with self.assertRaisesRegex(ValueError, name):
                    self.compile()

    def test_missing_vertical_scalar_is_rejected(self) -> None:
        self.edit(
            "vehicles/iem24_murphy_real_planar.xml",
            '<parameter name="tire_vertical_damping" value="400.0" unit="N*s/m" />',
            "",
        )
        with self.assertRaisesRegex(ValueError, "tire_vertical_damping"):
            self.compile()

    def test_wrong_unit_is_rejected(self) -> None:
        self.edit("vehicles/iem24_murphy_real_planar.xml", 'value="400.0" unit="N*s/m"', 'value="400.0" unit="N/m"')
        with self.assertRaises(ValueError):
            self.compile()

    def test_missing_suspension_role_is_rejected(self) -> None:
        self.edit(
            "vehicles/iem24_murphy_real_planar.xml",
            '<component role="suspension" ref="../suspensions/iem24_murphy_real_suspension.xml" />',
            "",
        )
        with self.assertRaisesRegex(CapabilityResolutionError, SUSPENSION_FORCES):
            self.compile()

    def test_formulation_without_complete_capabilities_is_rejected_for_vertical_use(self) -> None:
        # An incomplete requires list compiles the vertical model without the suspension.
        self.edit(
            "formulations/iem24_murphy_real_14dof_simple.xml",
            '<requires capability="suspension-forces" />',
            "",
        )
        with self.assertRaises(ValueError):
            self.compile()

    def test_nonpositive_relaxation_length_is_rejected_at_compile(self) -> None:
        # A negative lateral stiffness polynomial makes sigma_y nonpositive in the load range.
        text = (self.models / "tires/iem24_murphy_real_unitire_transient.xml").read_text()
        import re
        match = re.search(r'<parameter name="k_y_0" value="([^"]+)"', text)
        self.assertIsNotNone(match)
        self.edit("tires/iem24_murphy_real_unitire_transient.xml", match.group(0), '<parameter name="k_y_0" value="-1.0"')
        with self.assertRaises(ValueError):
            self.compile()


if __name__ == "__main__":
    unittest.main()
