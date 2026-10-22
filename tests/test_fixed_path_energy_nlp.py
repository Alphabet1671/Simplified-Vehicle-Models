"""Focused behavior tests for the independent fixed-path energy NLP."""
from dataclasses import dataclass
import math
import unittest

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
        self.assertLess(result.speed[1], result.speed[0])
        self.assertLessEqual(result.speed[-1], 3.+1.e-8)

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
