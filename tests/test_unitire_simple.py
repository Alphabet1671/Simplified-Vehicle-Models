import csv
from dataclasses import replace
from pathlib import Path
import unittest

import numpy as np

from simplified_vehicle_models.compiler.adapters import (
    CapabilityResolutionError,
    plan_capability,
)
from simplified_vehicle_models.compiler.capabilities import (
    STEADY_COMBINED_SLIP,
    TIRE_FORCE_ENVELOPE,
    SteadyCombinedSlip,
)
from simplified_vehicle_models.compiler.registry import (
    ComponentCandidate,
    component_descriptor,
)
from simplified_vehicle_models.parameters.tire import UniTireSimpleParameters
from simplified_vehicle_models.tires.unitire_simple import (
    compile_unitire_simple,
    evaluate_unitire,
)
from simplified_vehicle_models.xml.loader import load_model


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
TIRE_PATH = REPOSITORY_ROOT / "models/tires/hoosier-43075-r20-simple.xml"
ENVELOPE_PATH = REPOSITORY_ROOT / "models/tires/correlated_comp_2024.xml"
FIXTURE_PATH = REPOSITORY_ROOT / "tests/fixtures/hoosier_43075_r20_reference.csv"

#: SHA-256 of the fitted MATLAB parameter file behind the exported XML.
SOURCE_SHA256 = (
    "ed46ac5c5eb19ceeca359a10a5c4b279b26e468dec00bec229df3033848f1500"
)


def _load_reference() -> tuple[list[str], np.ndarray]:
    """Read the MATLAB reference fixture and skip its provenance comments."""

    with FIXTURE_PATH.open(newline="", encoding="utf-8") as stream:
        body = [
            line for line in stream if not line.lstrip().startswith("#")
        ]
    reader = csv.reader(body)
    columns = next(reader)
    values = np.asarray(
        [[float(item) for item in row] for row in reader],
        dtype=np.float64,
    )
    return columns, values


def _evaluate_rows(
    parameters: UniTireSimpleParameters,
    rows: np.ndarray,
):
    return evaluate_unitire(
        parameters,
        rows[:, 0],
        rows[:, 1],
        rows[:, 2],
        rows[:, 3],
    )


class UnitireSimpleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.parameters = compile_unitire_simple(load_model(TIRE_PATH))
        cls.columns, cls.reference = _load_reference()
        cls.reference_index = {
            name: index for index, name in enumerate(cls.columns)
        }

    def _column(self, name: str) -> np.ndarray:
        return self.reference[:, self.reference_index[name]]

    def test_source_provenance_is_pinned(self) -> None:
        text = TIRE_PATH.read_text(encoding="utf-8")

        self.assertIn(SOURCE_SHA256, text)
        self.assertIn('subtype="unitire-simple"', text)
        self.assertIn('type="tire-model"', text)

    def test_registry_descriptor_and_protocol(self) -> None:
        descriptor = component_descriptor("tire-model", "unitire-simple")

        self.assertEqual(descriptor.capability, STEADY_COMBINED_SLIP)
        self.assertIs(descriptor.compile_function, compile_unitire_simple)
        self.assertIsInstance(self.parameters, SteadyCombinedSlip)
        self.assertIsInstance(self.parameters, UniTireSimpleParameters)

    def test_fitted_coefficients_round_trip_exactly(self) -> None:
        parameters = self.parameters

        self.assertEqual(parameters.f_z_rated, 660.0)
        self.assertEqual(
            parameters.k_x,
            (7660.409078173158, 20000.0, 8191.007990757677),
        )
        self.assertEqual(
            parameters.k_y,
            (654.9807055362467, 19999.998862085806, 10000.0),
        )
        self.assertEqual(parameters.e_1, -0.01547791924434061)
        self.assertEqual(parameters.mu_0_x, 2.3361624600071096)
        self.assertEqual(parameters.mu_0_y, 2.3803804777987394)
        self.assertEqual(parameters.r_1, 0.2045)
        # The fit leaves these groups at zero.
        self.assertEqual(
            (parameters.k_11, parameters.k_12, parameters.k_13),
            (0.0, 0.0, 0.0),
        )
        self.assertEqual(
            (parameters.m_x_r_1, parameters.m_x_r_2, parameters.m_x_r_3),
            (0.0, 0.0, 0.0),
        )
        self.assertEqual(
            (parameters.f_rr_0, parameters.f_rr_1, parameters.f_rr_2),
            (0.0, 0.0, 0.0),
        )

    def test_python_port_reproduces_the_matlab_reference(self) -> None:
        rows = self.reference
        result = _evaluate_rows(self.parameters, rows)

        comparisons = {
            "longitudinal_force_n": result.longitudinal_force,
            "lateral_force_n": result.lateral_force,
            "vertical_force_n": result.vertical_force,
            "overturning_moment_nm": result.overturning_moment,
            "rolling_resistance_moment_nm": result.rolling_resistance_moment,
            "aligning_moment_nm": result.aligning_moment,
            "effective_radius_m": result.effective_radius,
            "loaded_radius_m": result.loaded_radius,
            "mu_x": result.mu_x,
            "mu_y": result.mu_y,
            "pneumatic_trail_m": result.pneumatic_trail,
            "relaxation_length_m": result.relaxation_length,
            "phi": result.phi,
            "f_bar": result.f_bar,
            "lambda": result.lambda_factor,
            "sx": result.s_x,
            "sy": result.s_y,
        }
        self.assertEqual(
            set(comparisons) | {"slip_angle_rad", "slip_ratio",
                                "vertical_load_n", "wheel_speed_rad_s"},
            set(self.columns),
        )
        for name, actual in comparisons.items():
            with self.subTest(column=name):
                # The measured agreement is at round-off level: the largest
                # absolute difference is 4.6e-13 N and the largest relative
                # difference is 1.6e-15. Keep a wide margin for another BLAS.
                np.testing.assert_allclose(
                    np.asarray(actual, dtype=np.float64),
                    self._column(name),
                    rtol=1.0e-12,
                    atol=1.0e-11,
                    err_msg=f"column {name} disagrees with MATLAB",
                )

    def test_positive_slip_angle_is_a_rightward_force(self) -> None:
        result = evaluate_unitire(self.parameters, 0.1, 0.0, 1000.0, 50.0)

        # The vehicle frame is SAE, so a positive lateral force acts right.
        self.assertGreater(float(result.lateral_force), 0.0)
        # The source reports the aligning moment in SAE tire axes.
        self.assertLess(float(result.aligning_moment), 0.0)
        self.assertGreater(float(result.overturning_moment), 0.0)

    def test_negative_slip_angle_mirrors_the_lateral_outputs(self) -> None:
        right = evaluate_unitire(self.parameters, 0.2, 0.05, 1000.0, 50.0)
        left = evaluate_unitire(self.parameters, -0.2, 0.05, 1000.0, 50.0)

        self.assertAlmostEqual(
            float(right.longitudinal_force),
            float(left.longitudinal_force),
        )
        self.assertAlmostEqual(
            float(right.lateral_force),
            -float(left.lateral_force),
        )
        self.assertAlmostEqual(
            float(right.aligning_moment),
            -float(left.aligning_moment),
        )
        self.assertAlmostEqual(
            float(right.overturning_moment),
            -float(left.overturning_moment),
        )

    def test_free_rolling_has_no_slip_force(self) -> None:
        result = evaluate_unitire(self.parameters, 0.0, 0.0, 1000.0, 50.0)

        self.assertEqual(float(result.longitudinal_force), 0.0)
        self.assertEqual(float(result.lateral_force), 0.0)
        self.assertEqual(float(result.aligning_moment), 0.0)
        self.assertEqual(float(result.phi), 0.0)
        self.assertEqual(float(result.f_bar), 0.0)

    def test_friction_moves_from_peak_to_sliding(self) -> None:
        at_rest = evaluate_unitire(self.parameters, 0.0, 0.0, 1000.0, 50.0)
        self.assertAlmostEqual(
            float(at_rest.mu_x), self.parameters.mu_0_x, places=12
        )
        self.assertAlmostEqual(
            float(at_rest.mu_y), self.parameters.mu_0_y, places=12
        )

        # The law falls from the peak value to the sliding value as slip grows.
        slips = np.asarray([0.0, 0.01, 0.05, 0.2, 0.5, 1.0])
        friction = evaluate_unitire(
            self.parameters, 0.0, slips, 1000.0, 50.0
        ).mu_x
        self.assertTrue(np.all(np.diff(friction) < 0.0))
        self.assertGreater(float(friction[0]), float(friction[-1]))
        self.assertGreaterEqual(float(friction[-1]), self.parameters.mu_s_x)

        # A very large slip reaches the sliding value.
        saturated = evaluate_unitire(self.parameters, 0.0, 1.0e6, 1000.0, 50.0)
        self.assertAlmostEqual(
            float(saturated.mu_x), self.parameters.mu_s_x, places=12
        )

    def test_low_speed_uses_the_rolling_speed_floor(self) -> None:
        floor_speed = self.parameters.vroll_min / float(
            evaluate_unitire(
                self.parameters, 0.1, 0.0, 1000.0, 50.0
            ).effective_radius
        )
        at_floor = evaluate_unitire(
            self.parameters, 0.1, 0.0, 1000.0, floor_speed
        )
        below_floor = evaluate_unitire(self.parameters, 0.1, 0.0, 1000.0, 1.0e-6)

        self.assertAlmostEqual(float(at_floor.mu_x), float(below_floor.mu_x))
        self.assertAlmostEqual(
            float(at_floor.lateral_force), float(below_floor.lateral_force)
        )

    def test_zero_load_is_finite_and_force_free(self) -> None:
        result = evaluate_unitire(self.parameters, 0.1, 0.05, 0.0, 50.0)

        for name in (
            "longitudinal_force",
            "lateral_force",
            "vertical_force",
            "overturning_moment",
            "rolling_resistance_moment",
            "aligning_moment",
            "effective_radius",
        ):
            value = np.asarray(getattr(result, name), dtype=np.float64)
            with self.subTest(field=name):
                self.assertTrue(np.all(np.isfinite(value)))
        self.assertEqual(float(result.longitudinal_force), 0.0)
        self.assertEqual(float(result.lateral_force), 0.0)
        self.assertEqual(float(result.aligning_moment), 0.0)
        self.assertAlmostEqual(float(result.effective_radius), 0.2045)

    def test_broadcasts_over_array_inputs(self) -> None:
        result = evaluate_unitire(
            self.parameters,
            np.asarray([0.0, 0.1, -0.1]),
            0.0,
            1000.0,
            50.0,
        )

        self.assertEqual(np.shape(result.lateral_force), (3,))
        self.assertAlmostEqual(float(result.lateral_force[0]), 0.0)
        self.assertAlmostEqual(
            float(result.lateral_force[1]), -float(result.lateral_force[2])
        )

    def test_steady_capability_is_not_reachable_from_an_envelope(self) -> None:
        envelope = load_model(ENVELOPE_PATH)
        descriptor = component_descriptor(
            envelope.model_type, envelope.subtype
        )
        self.assertEqual(descriptor.capability, TIRE_FORCE_ENVELOPE)

        candidates = (
            ComponentCandidate(
                role="tire",
                model=envelope,
                descriptor=descriptor,
            ),
        )
        with self.assertRaises(CapabilityResolutionError):
            plan_capability(STEADY_COMBINED_SLIP, candidates)

    def test_rejects_invalid_parameters(self) -> None:
        model = load_model(TIRE_PATH)

        for name, value in (
            ("f_z_rated", 0.0),
            ("f_z_rated", -1.0),
            ("k_cx", 0.0),
            ("k_cy", -1.0),
            ("v_m_x", 0.0),
            ("vroll_min", 0.0),
            ("force_eps", 0.0),
            ("stiff_eps", 0.0),
            ("slip_eps", -1.0),
        ):
            parameters = dict(model.parameters)
            parameters[name] = value
            invalid = replace(model, parameters=parameters)
            with self.subTest(name=name, value=value), self.assertRaises(
                ValueError
            ):
                compile_unitire_simple(invalid)

    def test_rejects_the_wrong_subtype(self) -> None:
        with self.assertRaises(ValueError):
            compile_unitire_simple(load_model(ENVELOPE_PATH))


if __name__ == "__main__":
    unittest.main()
