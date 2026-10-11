from __future__ import annotations

import shutil
import sys
import tempfile
import unittest
from dataclasses import replace
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
)
from simplified_vehicle_models.parameters.tire import TireEnvelopeParameters
from simplified_vehicle_models.parameters.vehicle import PointMassVehicleParameters
from simplified_vehicle_models.results.traces import LapMode
from simplified_vehicle_models.solvers.forward_backward import solve_lap
from simplified_vehicle_models.tires.transient_slip import TransientTireParameters
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.xml.loader import load_model, load_point_mass_vehicle


MINIMAL_FORMULATION = REPOSITORY_ROOT / "models/formulations/murphy_point_mass.xml"
FIXTURE_FORMULATION = (
    REPOSITORY_ROOT / "models/formulations/fixture_point_mass.xml"
)
EXTENDED_FORMULATION = (
    REPOSITORY_ROOT / "models/formulations/fixture_point_mass_extended.xml"
)
REDUCED_FORMULATION = (
    REPOSITORY_ROOT / "models/formulations/fixture_point_mass_reduced.xml"
)
FIXTURE_ASSEMBLY = REPOSITORY_ROOT / "models/vehicles/fixture_four_wheel.xml"
TRANSIENT_TIRE = REPOSITORY_ROOT / "models/tires/transient_comp_2024.xml"
SUSPENSION_LOOKUP = REPOSITORY_ROOT / "models/suspension/lookup_2024.xml"
TRACK_PATH = REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_v11.xml"
TIE_BREAK_TARGET = "capability-tie-break"


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


def _vehicle_fields(vehicle: PointMassVehicleParameters) -> tuple[object, ...]:
    """Return every compiled field except the manifest."""

    return (
        vehicle.total_mass,
        vehicle.wheelbase,
        vehicle.track_width,
        vehicle.cg_height,
        vehicle.front_mass_fraction,
        vehicle.lateral_load_transfer_distribution,
        vehicle.brake_bias,
        vehicle.tire,
        vehicle.aero,
        vehicle.drivetrain,
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
        self.assertEqual(_vehicle_fields(self.fixture), _vehicle_fields(self.minimal))
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
        self.assertEqual(
            _vehicle_fields(self.extended),
            _vehicle_fields(self.fixture),
        )
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
        self.assertEqual(added.subtype, "transient-combined-slip")

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
        for capability in (
            TIRE_FORCE_ENVELOPE,
            AERO_LOADS,
            DRIVE_FORCE_LIMIT,
        ):
            record = manifest.capability(capability)
            assert record is not None
            self.assertTrue(record.is_direct)
            self.assertEqual(record.adapter_names, ())

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
        self.assertEqual(loaded.subtype, "transient-combined-slip")
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
        self.assertEqual(
            replace(self.reduced, manifest=None),
            replace(self.minimal, manifest=None),
        )

    def test_assembly_with_an_unused_component_compiles(self) -> None:
        self.assertIsInstance(self.fixture, PointMassVehicleParameters)
        self.assertGreater(len(self.fixture.manifest.unused_components), 0)

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
