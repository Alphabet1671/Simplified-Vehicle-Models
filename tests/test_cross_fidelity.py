from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from simplified_vehicle_models.compiler.adapters import (
    CapabilityResolutionError,
    plan_capability,
    register_adapter,
    resolve_capability,
    unregister_adapter,
)
from simplified_vehicle_models.compiler.capabilities import (
    AERO_LOADS,
    BRAKE_LIMITS,
    DRIVE_FORCE_LIMIT,
    SUSPENSION_LOAD_TRANSFER,
    TIRE_FORCE_ENVELOPE,
    TRANSIENT_COMBINED_SLIP,
)
from simplified_vehicle_models.compiler.compile import (
    compile_component,
    compile_formulation,
)
from simplified_vehicle_models.compiler.registry import (
    ComponentCandidate,
    component_descriptor,
    register_component,
)
from simplified_vehicle_models.parameters.tire import TireEnvelopeParameters
from simplified_vehicle_models.parameters.vehicle import PointMassVehicleParameters
from simplified_vehicle_models.results.traces import LapMode
from simplified_vehicle_models.solvers.forward_backward import solve_lap
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.xml.loader import load_model, load_point_mass_vehicle
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    closed_parameters,
    require_model_type,
    require_no_references,
    require_nonnegative,
    require_positive,
)


MINIMAL_FORMULATION = REPOSITORY_ROOT / "models/formulations/murphy_point_mass.xml"
FIXTURE_MODELS = REPOSITORY_ROOT / "tests/fixtures/models"
FIXTURE_FORMULATION = FIXTURE_MODELS / "fixture_point_mass.xml"
EXTENDED_FORMULATION = FIXTURE_MODELS / "fixture_point_mass_extended.xml"
REDUCED_FORMULATION = FIXTURE_MODELS / "fixture_point_mass_reduced.xml"
FIXTURE_ASSEMBLY = FIXTURE_MODELS / "fixture_four_wheel.xml"
TRANSIENT_TIRE = FIXTURE_MODELS / "transient_comp_2024.xml"
SUSPENSION_LOOKUP = FIXTURE_MODELS / "suspension_lookup_2024.xml"
TRACK_PATH = REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_v11.xml"
TIE_BREAK_TARGET = "capability-tie-break"

TRANSIENT_TIRE_SUBTYPE = "transient-combined-slip"
SUSPENSION_LOOKUP_SUBTYPE = "lookup-table"

_TRANSIENT_TIRE_UNITS = {
    "relaxation_length": "m",
    "cornering_stiffness": "N/rad",
    "longitudinal_slip_stiffness": "N",
    "effective_radius": "m",
}

_SUSPENSION_LOOKUP_UNITS = {
    "front_roll_stiffness": "N*m/rad",
    "rear_roll_stiffness": "N*m/rad",
    "front_ride_height": "m",
    "rear_ride_height": "m",
    "motion_ratio": "1",
}


@dataclass(frozen=True, slots=True)
class TransientTireParameters:
    """Store the parameters of a fixture transient tire."""

    relaxation_length: float
    cornering_stiffness: float
    longitudinal_slip_stiffness: float
    effective_radius: float


@dataclass(frozen=True, slots=True)
class SuspensionLookupParameters:
    """Store the parameters of a fixture suspension lookup table."""

    front_roll_stiffness: float
    rear_roll_stiffness: float
    front_ride_height: float
    rear_ride_height: float
    motion_ratio: float


def compile_transient_tire(model: Model) -> TransientTireParameters:
    """Compile one fixture transient tire. This schema is test only."""

    require_model_type(model, "tire-model", TRANSIENT_TIRE_SUBTYPE)
    values = closed_parameters(model, _TRANSIENT_TIRE_UNITS)
    require_no_references(model)
    require_positive(
        model,
        values,
        "relaxation_length",
        "cornering_stiffness",
        "longitudinal_slip_stiffness",
        "effective_radius",
    )
    return TransientTireParameters(**values)


def compile_suspension_lookup(model: Model) -> SuspensionLookupParameters:
    """Compile one fixture suspension lookup table. This schema is test only."""

    require_model_type(model, "suspension-model", SUSPENSION_LOOKUP_SUBTYPE)
    values = closed_parameters(model, _SUSPENSION_LOOKUP_UNITS)
    require_no_references(model)
    require_positive(
        model,
        values,
        "front_roll_stiffness",
        "rear_roll_stiffness",
        "motion_ratio",
    )
    require_nonnegative(model, values, "front_ride_height", "rear_ride_height")
    return SuspensionLookupParameters(**values)


# The fixture schema types are test only.
# The production bootstrap does not know them.
register_component(
    "tire-model",
    TRANSIENT_TIRE_SUBTYPE,
    capability=TRANSIENT_COMBINED_SLIP,
    compile_function=compile_transient_tire,
)
register_component(
    "suspension-model",
    SUSPENSION_LOOKUP_SUBTYPE,
    capability=SUSPENSION_LOAD_TRANSFER,
    compile_function=compile_suspension_lookup,
)


def _reduce_transient_tire(
    transient: TransientTireParameters,
) -> TireEnvelopeParameters:
    """Reduce a transient tire to the steady envelope. This adapter is test only."""

    return TireEnvelopeParameters(
        longitudinal_mu_scale=0.25,
        lateral_mu_scale=0.4745,
        load_sensitivity_slope=-0.000404656098,
        load_sensitivity_intercept=2.9,
        effective_radius=transient.effective_radius,
    )


def _replace_transient_tire(value: object) -> object:
    """Return a marker value. This adapter is test only."""

    return ("transient", value)


def _replace_suspension(value: object) -> object:
    """Return a marker value. This adapter is test only."""

    return ("suspension", value)


def assert_shared_vehicle_contract(test_case, actual, expected):
    """Compare the shared point-mass contract across drivetrain runtimes."""
    for name in (
        "total_mass",
        "wheelbase",
        "track_width",
        "cg_height",
        "front_mass_fraction",
        "lateral_load_transfer_distribution",
        "brake_bias",
        "tire",
        "aero",
    ):
        test_case.assertEqual(getattr(actual, name), getattr(expected, name))
    for name in (
        "motor_count",
        "gear_ratio",
        "peak_motor_torque",
        "power_limit",
        "efficiency",
        "maximum_motor_speed",
    ):
        test_case.assertEqual(
            getattr(actual.drivetrain, name), getattr(expected.drivetrain, name)
        )


class CrossFidelityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        adapter = register_adapter(
            TRANSIENT_COMBINED_SLIP,
            TIRE_FORCE_ENVELOPE,
            _reduce_transient_tire,
        )
        cls.addClassCleanup(unregister_adapter, adapter)
        cls.minimal = load_point_mass_vehicle(MINIMAL_FORMULATION)
        cls.fixture = load_point_mass_vehicle(FIXTURE_FORMULATION)
        cls.extended = load_point_mass_vehicle(EXTENDED_FORMULATION)
        cls.reduced = load_point_mass_vehicle(REDUCED_FORMULATION)
        cls.track = load_track(TRACK_PATH)
        cls.minimal_lap = solve_lap(
            cls.minimal,
            cls.track,
            LapMode.FLYING,
            tolerance=1.0e-5,
        )
        cls.fixture_lap = solve_lap(
            cls.fixture,
            cls.track,
            LapMode.FLYING,
            tolerance=1.0e-5,
        )
        cls.extended_lap = solve_lap(
            cls.extended,
            cls.track,
            LapMode.FLYING,
            tolerance=1.0e-5,
        )

    def test_promise_one_reproduces_the_minimal_lap(self) -> None:
        assert_shared_vehicle_contract(self, self.fixture, self.minimal)
        self.assertAlmostEqual(
            self.fixture_lap.lap_time,
            self.minimal_lap.lap_time,
            delta=1.0e-9,
        )
        np.testing.assert_allclose(
            self.fixture_lap.speed,
            self.minimal_lap.speed,
            rtol=0.0,
            atol=1.0e-9,
        )

    def test_promise_two_keeps_the_lap_after_an_added_component(self) -> None:
        self.assertEqual(self.extended, self.fixture)
        self.assertAlmostEqual(
            self.extended_lap.lap_time,
            self.fixture_lap.lap_time,
            delta=1.0e-9,
        )
        np.testing.assert_allclose(
            self.extended_lap.speed,
            self.fixture_lap.speed,
            rtol=0.0,
            atol=1.0e-9,
        )

    def test_promise_two_manifest_lists_the_added_component(self) -> None:
        fixture_manifest = self.fixture.manifest
        extended_manifest = self.extended.manifest
        assert fixture_manifest is not None
        assert extended_manifest is not None

        self.assertEqual(
            extended_manifest.unused_roles(),
            ("transient_tire", "suspension", "transient_tire_rear"),
        )
        self.assertNotIn("transient_tire_rear", fixture_manifest.unused_roles())
        added = extended_manifest.unused_components[-1]
        self.assertEqual(added.model_id, "fixture-transient-comp-2025")
        self.assertEqual(added.subtype, TRANSIENT_TIRE_SUBTYPE)

    def test_manifest_lists_both_fixture_components_as_unused(self) -> None:
        manifest = self.fixture.manifest
        assert manifest is not None

        self.assertEqual(
            manifest.unused_roles(),
            ("transient_tire", "suspension"),
        )
        self.assertEqual(
            {record.capability for record in manifest.unused_components},
            {TRANSIENT_COMBINED_SLIP, SUSPENSION_LOAD_TRANSFER},
        )
        self.assertNotIn(
            "transient_tire",
            {record.role for record in manifest.loaded_components},
        )

    def test_manifest_lists_the_unconsumed_assembly_parameter(self) -> None:
        manifest = self.fixture.manifest
        assert manifest is not None

        self.assertEqual(
            manifest.unconsumed_names(),
            ("suspension_roll_axis_height",),
        )
        record = manifest.unconsumed_parameters[0]
        self.assertEqual(record.unit, "m")
        self.assertEqual(record.value, 0.06)

    def test_manifest_records_direct_capabilities(self) -> None:
        manifest = self.minimal.manifest
        assert manifest is not None

        self.assertEqual(manifest.unused_roles(), ())
        self.assertEqual(manifest.unconsumed_names(), ())
        for capability in (TIRE_FORCE_ENVELOPE, AERO_LOADS):
            record = manifest.capability(capability)
            assert record is not None
            self.assertTrue(record.is_direct)
            self.assertEqual(record.adapter_names, ())
        record = manifest.capability(DRIVE_FORCE_LIMIT)
        assert record is not None
        self.assertTrue(record.is_reduced)
        self.assertEqual(record.source_capability, "powertrain-and-drivetrain")
        self.assertEqual(record.adapter_names, ("reduce_powertrain_and_drivetrain",))

    def test_manifest_reports_a_reduced_capability(self) -> None:
        manifest = self.reduced.manifest
        assert manifest is not None

        record = manifest.capability(TIRE_FORCE_ENVELOPE)
        assert record is not None
        self.assertTrue(record.is_reduced)
        self.assertEqual(record.source_capability, TRANSIENT_COMBINED_SLIP)
        self.assertEqual(record.component_role, "tire")
        self.assertEqual(record.adapter_names, ("_reduce_transient_tire",))
        loaded = manifest.component("tire")
        assert loaded is not None
        self.assertEqual(loaded.subtype, TRANSIENT_TIRE_SUBTYPE)
        self.assertEqual(
            self.reduced.tire,
            TireEnvelopeParameters(
                longitudinal_mu_scale=0.25,
                lateral_mu_scale=0.4745,
                load_sensitivity_slope=-0.000404656098,
                load_sensitivity_intercept=2.9,
                effective_radius=0.193675,
            ),
        )
        assert_shared_vehicle_contract(self, self.reduced, self.minimal)

    def test_assembly_with_an_unused_component_compiles(self) -> None:
        self.assertIsInstance(self.fixture, PointMassVehicleParameters)
        assert self.fixture.manifest is not None
        self.assertGreater(len(self.fixture.manifest.unused_components), 0)

    def test_unknown_parameter_in_an_unused_component_compiles(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            unused = root / "unused.xml"
            unused.write_text(
                '<model schema-version="1.0" id="fixture-unused-transient" '
                f'type="tire-model" subtype="{TRANSIENT_TIRE_SUBTYPE}">'
                "<parameters>"
                '<parameter name="relaxation_length" value="0.15" unit="m"/>'
                '<parameter name="torque_map" '
                'value="amk_dd5_100c_torque_map.csv" unit="1"/>'
                '<parameter name="speling_mistake" value="1" unit="1"/>'
                "</parameters>"
                "</model>",
                encoding="utf-8",
            )
            assembly = root / "assembly.xml"
            self._write_fixture_assembly(assembly, unused)
            formulation = root / "formulation.xml"
            formulation.write_text(
                '<model schema-version="1.0" id="fixture-unused-parameter" '
                'type="vehicle-formulation" subtype="point-mass">'
                f'<source ref="{assembly}"/>'
                "</model>",
                encoding="utf-8",
            )

            vehicle = load_point_mass_vehicle(formulation)

            manifest = vehicle.manifest
            assert manifest is not None
            self.assertIn("unused", manifest.unused_roles())
            record = next(
                item
                for item in manifest.unused_components
                if item.role == "unused"
            )
            self.assertEqual(record.model_id, "fixture-unused-transient")
            self.assertEqual(record.capability, TRANSIENT_COMBINED_SLIP)
            assert_shared_vehicle_contract(self, vehicle, self.minimal)

    def test_unknown_type_in_an_unused_component_is_an_error(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            unused = root / "unused.xml"
            unused.write_text(
                '<model schema-version="1.0" id="fixture-unknown-type" '
                'type="unknown-model" subtype="unknown">'
                "<parameters>"
                '<parameter name="anything" value="1" unit="1"/>'
                "</parameters>"
                "</model>",
                encoding="utf-8",
            )
            assembly = root / "assembly.xml"
            self._write_fixture_assembly(assembly, unused)
            formulation = root / "formulation.xml"
            formulation.write_text(
                '<model schema-version="1.0" id="fixture-unknown-type-formulation" '
                'type="vehicle-formulation" subtype="point-mass">'
                f'<source ref="{assembly}"/>'
                "</model>",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "unknown model type"):
                load_point_mass_vehicle(formulation)

    def test_missing_capability_names_the_capability_and_the_assembly(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "missing.xml"
            path.write_text(
                '<model schema-version="1.0" id="missing" '
                'type="vehicle-formulation" subtype="point-mass">'
                f'<source ref="{FIXTURE_ASSEMBLY.resolve()}"/>'
                f'<requires capability="{BRAKE_LIMITS}"/>'
                "</model>",
                encoding="utf-8",
            )

            with self.assertRaises(CapabilityResolutionError) as context:
                compile_formulation(path)

            message = str(context.exception)
            self.assertIn(BRAKE_LIMITS, message)
            self.assertIn(FIXTURE_ASSEMBLY.name, message)

    def test_unknown_parameter_in_a_requested_component_is_an_error(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "models"
            shutil.copytree(REPOSITORY_ROOT / "models", target)
            tire_path = target / "tires/correlated_comp_2024.xml"
            text = tire_path.read_text(encoding="utf-8")
            tire_path.write_text(
                text.replace(
                    "</parameters>",
                    '        <parameter name="speling_mistake" value="1" unit="1"/>\n'
                    "    </parameters>",
                ),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "speling_mistake"):
                load_point_mass_vehicle(
                    target / "formulations/murphy_point_mass.xml"
                )

    def test_reference_to_a_formulation_type_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            nested = root / "nested.xml"
            nested.write_text(
                '<model schema-version="1.0" id="nested" '
                'type="vehicle-formulation" subtype="point-mass">'
                f'<source ref="{FIXTURE_ASSEMBLY.resolve()}"/>'
                "</model>",
                encoding="utf-8",
            )
            assembly = root / "assembly.xml"
            self._write_fixture_assembly(assembly, nested)
            formulation = root / "formulation.xml"
            formulation.write_text(
                '<model schema-version="1.0" id="bad-reference" '
                'type="vehicle-formulation" subtype="point-mass">'
                f'<source ref="{assembly}"/>'
                "</model>",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "not a component type"):
                load_point_mass_vehicle(formulation)

    def test_production_bootstrap_does_not_register_fixture_types(self) -> None:
        script = (
            "from simplified_vehicle_models.compiler.registry import "
            "component_descriptor\n"
            "try:\n"
            "    component_descriptor('tire-model', 'transient-combined-slip')\n"
            "except ValueError:\n"
            "    print('unregistered')\n"
            "else:\n"
            "    print('registered')\n"
        )
        environment = dict(os.environ)
        environment["PYTHONPATH"] = str(REPOSITORY_ROOT / "src")
        result = subprocess.run(
            [sys.executable, "-c", script],
            check=True,
            capture_output=True,
            text=True,
            env=environment,
        )

        self.assertEqual(result.stdout.strip(), "unregistered")

    def test_adapter_choice_follows_registration_order(self) -> None:
        first = register_adapter(
            SUSPENSION_LOAD_TRANSFER,
            TIE_BREAK_TARGET,
            _replace_suspension,
        )
        self.addCleanup(unregister_adapter, first)
        second = register_adapter(
            TRANSIENT_COMBINED_SLIP,
            TIE_BREAK_TARGET,
            _replace_transient_tire,
        )
        self.addCleanup(unregister_adapter, second)
        candidates = self._reduction_candidates()

        plan = plan_capability(TIE_BREAK_TARGET, candidates)

        self.assertEqual(plan.provider.role, "suspension")
        self.assertEqual(plan.chain[0].adapter_function, _replace_suspension)
        self.assertEqual(plan.chain[0].adapter_name, "_replace_suspension")

        loaded = tuple(compile_component(item) for item in candidates)
        result = resolve_capability(TIE_BREAK_TARGET, loaded)
        self.assertEqual(result.component.role, "suspension")
        self.assertTrue(result.chain)
        self.assertEqual(result.value, ("suspension", loaded[1].value))

    def test_adapter_cycle_is_rejected(self) -> None:
        first = register_adapter("capability-a", "capability-b", _replace_suspension)
        self.addCleanup(unregister_adapter, first)
        second = register_adapter("capability-b", "capability-c", _replace_suspension)
        self.addCleanup(unregister_adapter, second)

        with self.assertRaisesRegex(ValueError, "cycle"):
            register_adapter("capability-c", "capability-a", _replace_suspension)

    def _write_fixture_assembly(self, path: Path, unused: Path) -> None:
        tire = (REPOSITORY_ROOT / "models/tires/correlated_comp_2024.xml").resolve()
        aero = (REPOSITORY_ROOT / "models/aero/murphy_2024.xml").resolve()
        drivetrain = (
            REPOSITORY_ROOT / "models/drivetrains/murphy_4wd.xml"
        ).resolve()
        lines = [
            '<model schema-version="1.0" id="fixture-unused-assembly"',
            ' type="vehicle-assembly" subtype="four-wheel">',
            "    <parameters>",
            '        <parameter name="total_mass" value="299.0" unit="kg"/>',
            '        <parameter name="wheelbase" value="1.535" unit="m"/>',
            '        <parameter name="track_width" value="1.016" unit="m"/>',
            '        <parameter name="cg_height" value="0.3" unit="m"/>',
            '        <parameter name="front_mass_fraction" value="0.47" unit="1"/>',
            '        <parameter name="lateral_load_transfer_distribution"',
            ' value="0.44" unit="1"/>',
            '        <parameter name="brake_bias" value="0.65" unit="1"/>',
            "    </parameters>",
            "    <components>",
            f'        <component role="tire" ref="{tire}"/>',
            f'        <component role="aero" ref="{aero}"/>',
            f'        <component role="drivetrain" ref="{drivetrain}"/>',
            f'        <component role="unused" ref="{unused.resolve()}"/>',
            "    </components>",
            "</model>",
        ]
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _reduction_candidates(self) -> tuple[ComponentCandidate, ...]:
        transient = load_model(TRANSIENT_TIRE)
        suspension = load_model(SUSPENSION_LOOKUP)
        return (
            ComponentCandidate(
                role="transient",
                model=transient,
                descriptor=component_descriptor(
                    transient.model_type,
                    transient.subtype,
                ),
            ),
            ComponentCandidate(
                role="suspension",
                model=suspension,
                descriptor=component_descriptor(
                    suspension.model_type,
                    suspension.subtype,
                ),
            ),
        )


if __name__ == "__main__":
    unittest.main()
