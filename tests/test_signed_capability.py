from pathlib import Path
import tempfile
import unittest

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import evaluate_equilibrium, point_mass_view_of_planar
from simplified_vehicle_models.formulations.signed_capability import (
    SignedTransientCapability, build_signed_transient_capability,
    load_signed_transient_capability, save_signed_transient_capability,
    signed_feasible_interval, symbolic_lookup,
)

ROOT = Path(__file__).resolve().parents[1]
FORMULATION = ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"


def _chart():
    return SignedTransientCapability(
        np.array([0., .5, 1.]), np.array([-.1, 0., .1]),
        np.array([[2., 2., 2.], [1., 1., 1.], [-2., -2., -2.]]),
        np.array([[-8., -8., -8.], [-7., -7., -7.], [-6., -6., -6.]]),
        np.array([15., 20., 15.]), .5, "fixture",
        {"scan_points": 101}, {"sampled": True},
    )


class SignedCapabilityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = point_mass_view_of_planar(compile_formulation(str(FORMULATION)).model)

    def test_live_signed_negative_upper_regressions(self):
        cases = ((33.33908018475074, 0., -0.05282909), (35.067452826, 0., -4.0512410))
        for speed, curvature, expected_upper in cases:
            with self.subTest(speed=speed):
                self.assertFalse(evaluate_equilibrium(self.vehicle, speed, curvature, 0.).is_feasible)
                interval = signed_feasible_interval(
                    lambda acceleration: evaluate_equilibrium(self.vehicle, speed, curvature, acceleration),
                    acceleration_min=-30., acceleration_max=10., scan_points=401, refinement_steps=30,
                )
                self.assertIsNotNone(interval)
                self.assertAlmostEqual(interval[1], expected_upper, delta=4.e-3)
                self.assertLess(interval[1], 0.)

    def test_live_curved_invalid_cell_is_excluded(self):
        curvature = -.0606381561585
        invalid = signed_feasible_interval(
            lambda acceleration: evaluate_equilibrium(self.vehicle, 17.495664306, curvature, acceleration),
            acceleration_min=-30., acceleration_max=10., scan_points=401, refinement_steps=24,
        )
        valid = signed_feasible_interval(
            lambda acceleration: evaluate_equilibrium(self.vehicle, 16.5, curvature, acceleration),
            acceleration_min=-30., acceleration_max=10., scan_points=401, refinement_steps=24,
        )
        self.assertIsNone(invalid)
        self.assertIsNotNone(valid)
        self.assertAlmostEqual(valid[0], -2.42649, delta=.01)
        self.assertAlmostEqual(valid[1], .47861, delta=.01)

    def test_real_builder_conservative_dense_validation(self):
        chart = build_signed_transient_capability(
            self.vehicle, curvature=np.array([-.01, 0., .01]),
            u=np.linspace(0., 1., 9), maximum_speed=35.067452826,
            scan_points=161, refinement_steps=20, speed_scan_points=61,
            interior_subdivisions=2, endpoint_buffer=.002, domain_backoff=.002,
            refinement_attempts=0,
        )
        self.assertGreater(chart.validation["construction_stencil_count"], 0)
        self.assertGreater(chart.validation["validation_stencil_count"], 0)
        self.assertLess(chart.limits(chart.domain_speed(0.), 0.)[1], 0.)
        # Independent stencil: includes full domain edges, u interiors and both
        # curvature-cell interiors; it is deliberately denser than construction.
        for coordinate in np.linspace(0., 1., 37):
            for curvature in np.linspace(-.01, .01, 19):
                speed = chart.speed_floor + coordinate*(chart.domain_speed(curvature)-chart.speed_floor)
                low, high = chart.limits(speed, curvature)
                self.assertTrue(evaluate_equilibrium(self.vehicle, speed, curvature, low).is_feasible)
                self.assertTrue(evaluate_equilibrium(self.vehicle, speed, curvature, high).is_feasible)

    def test_physical_normalized_query_and_no_extrapolation(self):
        chart = _chart()
        self.assertEqual(chart.domain_speed(0.), 20.)
        self.assertAlmostEqual(chart.normalized_speed(10.25, 0.), .5)
        self.assertEqual(chart.limits(10.25, 0.), (-7., 1.))
        with self.assertRaisesRegex(ValueError, "capability domain"):
            chart.limits(20.001, 0.)
        with self.assertRaisesRegex(ValueError, "capability domain"):
            chart.limits(.4, 0.)

    def test_exclusive_versioned_serialization_preserves_legacy_files(self):
        chart = _chart()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "signed.npz"
            save_signed_transient_capability(chart, path)
            loaded = load_signed_transient_capability(path)
            self.assertEqual(loaded.representation, "signed_transient_normalized_v1")
            np.testing.assert_allclose(loaded.domain_speed_values, chart.domain_speed_values)
            with self.assertRaises(FileExistsError):
                save_signed_transient_capability(chart, path)

    def test_numeric_symbolic_lookup_parity(self):
        try:
            import casadi as ca
        except ImportError:
            self.skipTest("CasADi unavailable")
        chart = _chart(); lookup = symbolic_lookup(chart, "parity")
        v, k = ca.MX.sym("v"), ca.MX.sym("k")
        lower, upper, domain = lookup(v, k)
        function = ca.Function("lookup", [v, k], [lower, upper, domain])
        numeric = chart.limits(10.25, .04)
        actual = function(10.25, .04)
        self.assertAlmostEqual(float(actual[0]), numeric[0], places=12)
        self.assertAlmostEqual(float(actual[1]), numeric[1], places=12)
        self.assertAlmostEqual(float(actual[2]), chart.domain_speed(.04), places=12)

    def test_disconnected_acceleration_sets_are_refused(self):
        with self.assertRaisesRegex(ValueError, "disconnected"):
            signed_feasible_interval(lambda a: (-4 < a < -3) or a > 2,
                                     acceleration_min=-5., acceleration_max=5., scan_points=101)

class LinearProgramSignedCapabilityTest(unittest.TestCase):
    def test_lp_fit_preserves_width_where_common_interval_would_shrink(self):
        """A spatially varying signed band is feasible for LP vertex fitting.

        A legacy all-probe common interval is only [-.5, 2], while the LP retains
        the four-unit local interval; this guards against silently routing the
        opt-in mode through the reviewed common-interval builder.
        """
        from unittest.mock import patch
        import simplified_vehicle_models.formulations.signed_capability as signed
        class Vehicle: pass
        def fake_equilibrium(vehicle, speed, curvature, acceleration):
            coordinate = (speed-.5)/9.5
            low = -2. + 1.5*coordinate + .25*curvature
            return type("State", (), {"is_feasible": low <= acceleration <= low + 4.})()
        with patch.object(signed, "maximum_vehicle_speed", return_value=10.), \
             patch.object(signed, "_speed_domain", return_value=10.), \
             patch.object(signed, "capability_hash", return_value="synthetic"), \
             patch.object(signed, "evaluate_equilibrium", side_effect=fake_equilibrium):
            chart = signed.build_signed_transient_capability(
                Vehicle(), interval_fit="linear_program", curvature=np.array([-.5, .5]),
                u=np.array([0., 1.]), maximum_speed=10., acceleration_range=(-5., 5.),
                scan_points=41, refinement_steps=4, speed_scan_points=3,
                endpoint_buffer=.1, domain_backoff=.0, minimum_interval_width=.02,
                construction_subdivisions=2, validation_grid=(5, 5),
                heldout_validation_grid=(4, 4), cut_plane_attempts=0)
        self.assertEqual(chart.build_settings["interval_fit"], "linear_program")
        self.assertGreater(float((chart.acceleration_max-chart.acceleration_min).min()), 3.7)
        self.assertEqual(chart.validation["heldout_endpoint_checks"], 32)

    def test_lp_mode_rejects_bad_options_and_default_stays_common(self):
        import simplified_vehicle_models.formulations.signed_capability as signed
        with self.assertRaisesRegex(ValueError, "interval_fit"):
            signed.build_signed_transient_capability(object(), interval_fit="unreviewed", curvature=np.array([0., .1]))
        self.assertEqual(signed.build_signed_transient_capability.__kwdefaults__["interval_fit"], "common_interval")
