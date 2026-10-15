from pathlib import Path
import unittest

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.tires.unitire_envelope import build_unitire_peak_envelope
from simplified_vehicle_models.tires.unitire_simple import evaluate_unitire, loaded_radius


ROOT = Path(__file__).resolve().parents[1]
FORMULATION = ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"


class UnitirePeakEnvelopeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.vehicle = compile_formulation(FORMULATION).model

    def test_peak_lookup_is_finite_nonnegative_and_load_indexed(self) -> None:
        grid = np.array([0.0, 250.0, 660.0, 1500.0, 3000.0])
        envelope = build_unitire_peak_envelope(
            self.vehicle.tire, grid, slip_samples=301
        )

        fx, fy = envelope.force_capacities(grid)
        self.assertEqual(fx.shape, grid.shape)
        self.assertEqual(fy.shape, grid.shape)
        self.assertTrue(np.all(np.isfinite(fx)))
        self.assertTrue(np.all(np.isfinite(fy)))
        self.assertTrue(np.all(fx >= 0.0))
        self.assertTrue(np.all(fy >= 0.0))
        self.assertEqual(float(fx[0]), 0.0)
        self.assertEqual(float(fy[0]), 0.0)
        self.assertTrue(np.all(np.diff(fx) > 0.0))
        self.assertTrue(np.all(np.diff(fy) > 0.0))
        self.assertAlmostEqual(envelope.effective_radius, float(loaded_radius(self.vehicle.tire, 660.0)))

    def test_rated_load_peak_comes_from_simplified_unitire(self) -> None:
        parameters = self.vehicle.tire
        envelope = build_unitire_peak_envelope(
            parameters, np.array([0.0, parameters.f_z_rated]), slip_samples=1201
        )
        alpha = np.linspace(0.0, 0.6, 4001)
        kappa = np.linspace(0.0, 1.5, 4001)
        load = parameters.f_z_rated
        omega = 15.0 / float(loaded_radius(parameters, load))
        lateral = evaluate_unitire(
            parameters,
            alpha,
            np.zeros_like(alpha),
            np.full_like(alpha, load),
            np.full_like(alpha, omega),
        )
        longitudinal = evaluate_unitire(
            parameters,
            np.zeros_like(kappa),
            kappa,
            np.full_like(kappa, load),
            np.full_like(kappa, omega),
        )
        expected_fx = float(np.max(np.abs(longitudinal.longitudinal_force)))
        expected_fy = float(np.max(np.abs(lateral.lateral_force)))
        actual_fx, actual_fy = envelope.force_capacities(np.array([load]))

        self.assertAlmostEqual(float(actual_fx[0]) / expected_fx, 1.0, delta=0.002)
        self.assertAlmostEqual(float(actual_fy[0]) / expected_fy, 1.0, delta=0.002)

    def test_lookup_clamps_above_range_and_zeroes_negative_contact_load(self) -> None:
        grid = np.array([0.0, 1000.0, 2000.0])
        envelope = build_unitire_peak_envelope(self.vehicle.tire, grid, slip_samples=101)
        low = envelope.force_capacities(np.array([-100.0]))
        high = envelope.force_capacities(np.array([5000.0]))
        edge = envelope.force_capacities(np.array([2000.0]))

        self.assertEqual(float(low[0][0]), 0.0)
        self.assertEqual(float(low[1][0]), 0.0)
        self.assertAlmostEqual(float(high[0][0]), float(edge[0][0]))
        self.assertAlmostEqual(float(high[1][0]), float(edge[1][0]))

    def test_rejects_bad_grid_and_speed(self) -> None:
        with self.assertRaisesRegex(ValueError, "increasing"):
            build_unitire_peak_envelope(self.vehicle.tire, np.array([0.0, 1.0, 1.0]))
        with self.assertRaisesRegex(ValueError, "reference_speed"):
            build_unitire_peak_envelope(self.vehicle.tire, reference_speed=0.0)
