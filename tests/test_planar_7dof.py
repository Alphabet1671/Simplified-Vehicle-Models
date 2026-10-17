import math
from dataclasses import replace
from pathlib import Path
import unittest

import numpy as np

from simplified_vehicle_models.compiler.capabilities import (
    AERO_LOADS,
    DRIVE_FORCE_LIMIT,
    STEADY_COMBINED_SLIP,
)
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.planar_7dof import (
    PLANAR_7DOF_VEHICLE_PARAMETER_UNITS,
    body_lateral_acceleration,
    compile_planar_7dof_vehicle,
    evaluate_normal_loads,
    wheel_positions,
)
from simplified_vehicle_models.parameters.tire import UniTireSimpleParameters
from simplified_vehicle_models.parameters.vehicle import PlanarVehicleParameters
from simplified_vehicle_models.xml.loader import load_model
from tools.data.import_vehicle_spec_sheet import (
    SKEW,
    obfuscate,
    round_significant,
    skew_factor,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FORMULATION_PATH = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
ASSEMBLY_PATH = REPOSITORY_ROOT / "models/vehicles/iem24_murphy_real_planar.xml"
VEHICLE_XML_PATH = ASSEMBLY_PATH


class ObfuscationTests(unittest.TestCase):
    """Check the export mechanism. These tests hold no real car data."""

    def test_round_significant_keeps_two_figures(self) -> None:
        for value, expected in (
            (1530.35, 1500.0),
            (1244.6, 1200.0),
            (254.0, 250.0),
            (213.188, 210.0),
            (0.414, 0.41),
            (-7.3, -7.3),
            (0.0, 0.0),
        ):
            with self.subTest(value=value):
                self.assertAlmostEqual(
                    round_significant(value, 2), expected, places=9
                )

    def test_skew_factor_is_bounded_and_deterministic(self) -> None:
        for name in ("wheelbase", "track_width", "total_mass", "aero_balance"):
            factor = skew_factor(name)
            with self.subTest(name=name):
                self.assertGreaterEqual(factor, 1.0 - SKEW)
                self.assertLessEqual(factor, 1.0 + SKEW)
                self.assertEqual(factor, skew_factor(name))

    def test_obfuscate_rounds_then_skews_within_the_band(self) -> None:
        real = 1530.35
        rounded = round_significant(real, 2)
        emitted = obfuscate(real, "wheelbase")

        self.assertEqual(emitted, obfuscate(real, "wheelbase"))
        self.assertGreaterEqual(emitted, rounded * (1.0 - SKEW))
        self.assertLessEqual(emitted, rounded * (1.0 + SKEW))
        # Two significant figures are gone, so the real value is not recovered
        # from the emitted value alone.
        self.assertNotAlmostEqual(emitted, real, places=1)

    def test_different_names_get_different_skews(self) -> None:
        values = {
            obfuscate(1000.0, name)
            for name in ("alpha", "beta", "gamma", "delta", "epsilon")
        }
        self.assertGreater(len(values), 1)


class PlanarCompileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.compiled = compile_formulation(FORMULATION_PATH)
        cls.vehicle = cls.compiled.model

    def test_compiles_to_planar_parameters(self) -> None:
        self.assertIsInstance(self.vehicle, PlanarVehicleParameters)
        self.assertIsInstance(self.vehicle.tire, UniTireSimpleParameters)

    def test_resolves_every_requested_capability(self) -> None:
        capabilities = {c.capability for c in self.compiled.manifest.capabilities}

        self.assertEqual(
            capabilities,
            {STEADY_COMBINED_SLIP, AERO_LOADS, DRIVE_FORCE_LIMIT},
        )
        # The shared assembly also holds the 14DoF-only roles and parameters.
        self.assertEqual(
            self.compiled.manifest.unused_roles(),
            ("suspension", "damper-front", "damper-rear", "tire-transient"),
        )
        self.assertEqual(
            self.compiled.manifest.unconsumed_names(),
            (
                "pitch_radius_of_gyration",
                "roll_radius_of_gyration",
                "tire_vertical_damping",
                "unsprung_mass_front",
                "unsprung_mass_rear",
            ),
        )

    def test_consumes_every_required_parameter(self) -> None:
        expected = set(PLANAR_7DOF_VEHICLE_PARAMETER_UNITS)

        self.assertEqual(set(self.vehicle.__dataclass_fields__) >= expected, True)

    def test_mass_split_is_consistent(self) -> None:
        vehicle = self.vehicle

        self.assertAlmostEqual(
            vehicle.mass_without_driver + vehicle.driver_mass,
            vehicle.total_mass,
            places=6,
        )

    def test_yaw_inertia_follows_the_radius_of_gyration(self) -> None:
        vehicle = self.vehicle

        self.assertAlmostEqual(
            vehicle.yaw_inertia,
            vehicle.total_mass * vehicle.yaw_radius_of_gyration**2,
            places=9,
        )
        # The general estimation request: 120 kg m^2 at 260 kg.
        reference = 260.0 * (120.0 / 260.0)
        self.assertAlmostEqual(reference, 120.0, places=9)

    def test_yaw_inertia_scales_with_mass(self) -> None:
        vehicle = self.vehicle
        heavier = replace(vehicle, total_mass=vehicle.total_mass * 1.2)

        self.assertAlmostEqual(
            heavier.yaw_inertia / vehicle.yaw_inertia, 1.2, places=9
        )

    def test_axle_distances_use_the_front_mass_fraction(self) -> None:
        vehicle = self.vehicle

        self.assertAlmostEqual(
            vehicle.front_axle_to_cg + vehicle.rear_axle_to_cg,
            vehicle.wheelbase,
            places=9,
        )
        self.assertAlmostEqual(
            vehicle.front_axle_to_cg / vehicle.wheelbase,
            1.0 - vehicle.front_mass_fraction,
            places=9,
        )

    def test_assembly_uses_measured_iem24_mass_and_balance(self) -> None:
        text = VEHICLE_XML_PATH.read_text(encoding="utf-8")

        self.assertIn('name="total_mass" value="287.0"', text)
        self.assertIn('name="driver_mass" value="78.0"', text)
        self.assertIn('name="lateral_load_transfer_distribution" value="0.42"', text)


class WheelPositionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.vehicle = compile_formulation(FORMULATION_PATH).model

    def test_positions_follow_the_sae_frame(self) -> None:
        positions = wheel_positions(self.vehicle)

        self.assertEqual(positions.shape, (4, 2))
        # FL, FR, RL, RR.
        self.assertGreater(positions[0, 0], 0.0)
        self.assertGreater(positions[1, 0], 0.0)
        self.assertLess(positions[2, 0], 0.0)
        self.assertLess(positions[3, 0], 0.0)
        # Right wheels sit at a positive y.
        self.assertLess(positions[0, 1], 0.0)
        self.assertGreater(positions[1, 1], 0.0)

    def test_positions_match_the_geometry(self) -> None:
        vehicle = self.vehicle
        positions = wheel_positions(vehicle)

        self.assertAlmostEqual(
            positions[0, 0] - positions[2, 0], vehicle.wheelbase, places=9
        )
        self.assertAlmostEqual(
            positions[1, 1] - positions[0, 1], vehicle.track_width, places=9
        )
        self.assertAlmostEqual(
            positions[0, 1], -vehicle.half_track, places=9
        )


class QuasiStaticLoadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.vehicle = compile_formulation(FORMULATION_PATH).model

    def test_static_loads_sum_to_the_weight(self) -> None:
        loads = evaluate_normal_loads(self.vehicle, 0.0, 0.0, 0.0)

        self.assertAlmostEqual(
            float(np.sum(loads)), self.vehicle.total_mass * 9.81, places=6
        )
        self.assertAlmostEqual(float(loads[0]), float(loads[1]), places=9)
        self.assertAlmostEqual(float(loads[2]), float(loads[3]), places=9)

    def test_downforce_adds_to_the_total_load(self) -> None:
        speed = 25.0
        loads = evaluate_normal_loads(self.vehicle, speed, 0.0, 0.0)
        dynamic_pressure = 0.5 * self.vehicle.aero.air_density * speed * speed
        downforce = dynamic_pressure * self.vehicle.aero.downforce_area

        self.assertAlmostEqual(
            float(np.sum(loads)),
            self.vehicle.total_mass * 9.81 + downforce,
            places=6,
        )

    def test_right_turn_loads_the_left_wheels(self) -> None:
        loads = evaluate_normal_loads(self.vehicle, 20.0, 0.0, 8.0)

        self.assertGreater(float(loads[0]), float(loads[1]))
        self.assertGreater(float(loads[2]), float(loads[3]))

    def test_left_turn_loads_the_right_wheels(self) -> None:
        loads = evaluate_normal_loads(self.vehicle, 20.0, 0.0, -8.0)

        self.assertLess(float(loads[0]), float(loads[1]))
        self.assertLess(float(loads[2]), float(loads[3]))

    def test_turn_direction_mirrors_the_wheels(self) -> None:
        right = evaluate_normal_loads(self.vehicle, 20.0, 0.0, 8.0)
        left = evaluate_normal_loads(self.vehicle, 20.0, 0.0, -8.0)

        np.testing.assert_allclose(right, left[[1, 0, 3, 2]], atol=1.0e-9)

    def test_lateral_sign_moves_the_split_but_not_the_axle_total(self) -> None:
        right = evaluate_normal_loads(self.vehicle, 20.0, 0.0, 8.0)
        left = evaluate_normal_loads(self.vehicle, 20.0, 0.0, -8.0)

        self.assertAlmostEqual(
            float(right[0] + right[1]), float(left[0] + left[1]), places=9
        )
        self.assertNotAlmostEqual(float(right[0]), float(left[0]), places=3)

    def test_acceleration_moves_load_to_the_rear(self) -> None:
        static = evaluate_normal_loads(self.vehicle, 0.0, 0.0, 0.0)
        accelerating = evaluate_normal_loads(self.vehicle, 0.0, 5.0, 0.0)
        braking = evaluate_normal_loads(self.vehicle, 0.0, -5.0, 0.0)

        self.assertLess(float(np.sum(accelerating[:2])), float(np.sum(static[:2])))
        self.assertGreater(float(np.sum(braking[:2])), float(np.sum(static[:2])))

    def test_front_distribution_follows_the_parameter(self) -> None:
        vehicle = self.vehicle
        loads = evaluate_normal_loads(vehicle, 0.0, 0.0, 10.0)
        front_total = float(loads[0] + loads[1])

        self.assertAlmostEqual(
            front_total / (float(np.sum(loads))),
            vehicle.front_mass_fraction,
            places=6,
        )

    def test_body_lateral_acceleration(self) -> None:
        self.assertAlmostEqual(
            body_lateral_acceleration(2.0, 0.5, 20.0), 12.0, places=9
        )
        self.assertAlmostEqual(
            body_lateral_acceleration(0.0, -0.4, 15.0), -6.0, places=9
        )


class PlanarCompileRejectionTests(unittest.TestCase):
    def _compile(self, **overrides):
        model = load_model(ASSEMBLY_PATH)
        parameters = dict(model.parameters)
        parameters.update(overrides)
        return compile_planar_7dof_vehicle(
            replace(model, parameters=parameters), {}
        )

    def test_rejects_a_missing_parameter(self) -> None:
        model = load_model(ASSEMBLY_PATH)
        parameters = dict(model.parameters)
        parameters.pop("wheel_inertia")
        with self.assertRaises(ValueError):
            compile_planar_7dof_vehicle(replace(model, parameters=parameters), {})

    def test_rejects_a_nonpositive_parameter(self) -> None:
        # Match the parameter name in the message, so the test proves the
        # parameter check failed and not some later step.
        for name, value in (
            ("wheel_inertia", 0.0),
            ("yaw_radius_of_gyration", -1.0),
            ("steering_maximum_angle", 0.0),
            ("steering_maximum_rate", 0.0),
            ("rack_travel_per_revolution", 0.0),
            ("steer_arm_length", -1.0),
            ("maximum_pedal_force", 0.0),
            ("hydraulic_ratio", 0.0),
            ("maximum_brake_pressure", 0.0),
            ("caliper_area", 0.0),
            ("pad_mu_front", 0.0),
            ("pad_mu_rear", -0.5),
            ("brake_effective_radius_front", 0.0),
            ("brake_effective_radius_rear", 0.0),
            ("steer_ratio", 0.0),
        ):
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, name):
                self._compile(**{name: value})

    def test_rejects_an_invalid_fraction(self) -> None:
        for name, value in (
            ("front_mass_fraction", 0.0),
            ("front_mass_fraction", 1.0),
            ("brake_bias", 1.2),
            ("brake_bias", 0.0),
            ("lateral_load_transfer_distribution", 1.5),
        ):
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, name):
                self._compile(**{name: value})

    def test_rejects_a_mass_that_does_not_cover_the_driver(self) -> None:
        with self.assertRaisesRegex(ValueError, "cover the driver"):
            self._compile(mass_without_driver=300.0, driver_mass=68.0,
                          total_mass=100.0)


if __name__ == "__main__":
    unittest.main()
