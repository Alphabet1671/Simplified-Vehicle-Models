"""Focused behavior tests for the independent fixed-path energy NLP."""
from dataclasses import dataclass
import math
import unittest
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.formulations.signed_capability import SignedTransientCapability
from simplified_vehicle_models.solvers.fixed_path_energy_nlp import fixed_path_energy_profile
from simplified_vehicle_models.tracks.types import PathCoordinates


@dataclass
class Path:
    distance: object
    curvature: object
    is_closed: bool = False


def chart(*, upper=2., lower=-4., caps=(30., 30.)):
    return SignedTransientCapability(np.array([0., 1.]), np.array([0., 1.]),
        np.full((2, 2), upper), np.full((2, 2), lower), np.asarray(caps, dtype=float),
        .5, "fixed-path-nlp-test")


@unittest.skipUnless(__import__("importlib").util.find_spec("casadi"), "CasADi unavailable")
class FixedPathEnergyNlpTest(unittest.TestCase):
    def _mocked_result(self, energy, *, success=True, return_status="Solve_Succeeded"):
        class Solver:
            def __call__(self, **kwargs):
                return {"x": np.asarray(energy, dtype=float), "f": 1.}
            def stats(self):
                return {"success": success, "return_status": return_status, "iter_count": 1}
        with patch("casadi.nlpsol", return_value=Solver()):
            return fixed_path_energy_profile(Path(np.array([0., 1.]), np.zeros(2)), chart(),
                                             boundary="rolling", start_speed=1., step=1.)

    def test_negative_energy_and_nonexact_status_cannot_become_converged(self):
        negative = self._mocked_result([1., -1.])
        self.assertFalse(negative.converged)
        self.assertTrue(np.all(np.isnan(negative.speed)))
        nonexact = self._mocked_result([1., 5.], return_status="Solved_To_Acceptable_Level")
        self.assertFalse(nonexact.converged)
        self.assertEqual(nonexact.diagnostics["return_status"], "Solved_To_Acceptable_Level")

    def test_default_seed_honours_lower_analytic_midpoint_cap(self):
        # Nodes lie at the generous curvature-chart endpoints, while the exact
        # segment midpoint reaches an interior cap dip.  Capture x0 rather than
        # feigning a physically feasible solve from the rolling pinned endpoint.
        dipped = SignedTransientCapability(np.array([0., 1.]), np.array([0., .5, 1.]),
            np.full((2, 3), 2.), np.full((2, 3), -4.), np.array([30., 1., 30.]),
            .5, "midpoint-cap-dip")
        captured = {}
        class Solver:
            def __call__(self, **kwargs):
                captured["x0"] = np.asarray(kwargs["x0"], dtype=float)
                return {"x": np.array([25., 1.]), "f": 1.}
            def stats(self):
                return {"success": False, "return_status": "Maximum_Iterations_Exceeded", "iter_count": 1}
        with patch("casadi.nlpsol", return_value=Solver()):
            fixed_path_energy_profile(Path(np.array([0., 1.]), np.array([0., 1.])), dipped,
                                      boundary="rolling", start_speed=5., step=1.)
        self.assertLessEqual(math.sqrt(captured["x0"][1]), 1.)

    def test_rolling_straight_constant_acceleration_matches_analytic_time(self):
        result = fixed_path_energy_profile(Path(np.array([0., 10.]), np.zeros(2)), chart(upper=2.),
                                           boundary="rolling", start_speed=3., step=1.)
        expected = (math.sqrt(3.**2 + 2.*2.*10.)-3.)/2.
        self.assertTrue(result.converged, result.diagnostics)
        self.assertAlmostEqual(result.lap_time, expected, places=6)
        self.assertEqual(result.speed[0], 3.)
        self.assertEqual(result.diagnostics["claim"], "local NLP candidate only")

    def test_negative_maximum_acceleration_brakes_without_hold_lock(self):
        result = fixed_path_energy_profile(Path(np.array([0., 1.]), np.zeros(2)), chart(upper=-1., lower=-3.),
                                           boundary="rolling", start_speed=2., step=1.)
        self.assertTrue(result.converged, result.diagnostics)
        self.assertAlmostEqual(result.speed[-1], math.sqrt(2.), places=6)
        self.assertLess(result.speed[-1], result.speed[0])

    def test_corner_uses_anticipatory_braking(self):
        result = fixed_path_energy_profile(Path(np.array([0., 15., 16.]), np.array([0., 0., 1.])),
                                           chart(upper=4., lower=-2., caps=(30., 3.)),
                                           boundary="rolling", start_speed=8., step=1.)
        self.assertTrue(result.converged, result.diagnostics)
        self.assertLessEqual(result.speed[-1], 3.+1.e-8)
        # The best profile may initially accelerate; it must nevertheless begin
        # braking before the final capped corner segment because that last metre
        # alone cannot dissipate the required energy within the signed lower bound.
        energy_drop = np.diff(result.speed**2)
        self.assertTrue(np.any(energy_drop[:-1] < -1.e-7))
        signed = chart(upper=4., lower=-2., caps=(30., 3.))
        for index, (left, right) in enumerate(zip(result.speed[:-1], result.speed[1:])):
            acceleration = (right**2-left**2)/2.
            midpoint_speed = math.sqrt(.5*(left**2+right**2))
            midpoint_curvature = 0. if index < 15 else .5
            left_curvature = 0.
            right_curvature = 0. if index < 15 else 1.
            for speed, curvature in ((left, left_curvature), (right, right_curvature),
                                     (midpoint_speed, midpoint_curvature)):
                lower, upper = signed.limits(speed, curvature)
                self.assertGreaterEqual(acceleration, lower-1.e-7)
                self.assertLessEqual(acceleration, upper+1.e-7)

    def test_endpoint_notch_constrains_segment_even_when_midpoint_is_legal(self):
        # The acceleration ceiling at the right endpoint is 1, whereas the
        # midpoint ceiling is 2.5.  A midpoint-only transcription could return
        # a=2.5; node constraints require the physically legal a=1.
        notched = SignedTransientCapability(np.array([0., 1.]), np.array([0., 1.]),
            np.array([[4., 1.], [4., 1.]]), np.full((2, 2), -4.),
            np.full(2, 30.), .5, "endpoint-notch")
        result = fixed_path_energy_profile(Path(np.array([0., 1.]), np.array([0., 1.])), notched,
                                           boundary="rolling", start_speed=1., step=1.)
        self.assertTrue(result.converged, result.diagnostics)
        self.assertAlmostEqual(result.speed[-1], math.sqrt(3.), places=6)
        acceleration = (result.speed[-1]**2-result.speed[0]**2)/2.
        self.assertLessEqual(acceleration, notched.limits(result.speed[-1], 1.)[1]+1.e-8)
        self.assertGreater(acceleration, notched.limits(math.sqrt(2.), .5)[1]-2.)

    def test_closed_flying_requires_physical_endpoint_and_validates_periodic(self):
        closed = PathCoordinates(np.array([0., 5., 10.]), np.array([0., 1., 0.]),
                                 np.zeros(3), np.zeros(3), np.zeros(3), True)
        result = fixed_path_energy_profile(closed, chart(), boundary="flying", step=1.)
        self.assertTrue(result.converged, result.diagnostics)
        self.assertAlmostEqual(result.speed[0], result.speed[-1], places=8)
        malformed = PathCoordinates(np.array([0., 5., 10.]), np.array([0., 1., 2.]),
                                    np.zeros(3), np.zeros(3), np.zeros(3), True)
        with self.assertRaisesRegex(ValueError, "closed path"):
            fixed_path_energy_profile(malformed, chart(), boundary="flying")



class FixedPathEnergyNlpValidationTest(unittest.TestCase):
    def test_standing_and_legacy_surface_are_explicitly_unsupported(self):
        path = Path(np.array([0., 1.]), np.zeros(2))
        with self.assertRaisesRegex(ValueError, "standing is unsupported"):
            fixed_path_energy_profile(path, chart(), boundary="standing")
        legacy = type("Legacy", (), {"speed": np.array([.5, 2.]), "curvature": np.array([0., 1.]),
                 "speed_ceiling": np.array([2., 2.]), "acceleration_min": np.ones((2, 2)),
                 "acceleration_max": np.ones((2, 2))})()
        with self.assertRaisesRegex(TypeError, "legacy tables"):
            fixed_path_energy_profile(path, legacy, boundary="rolling", start_speed=1.)


if __name__ == "__main__":
    unittest.main()
