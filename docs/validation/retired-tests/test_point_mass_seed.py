from types import SimpleNamespace
import unittest

import numpy as np

from simplified_vehicle_models.solvers.point_mass_seed import (
    point_mass_planar_primal_seed,
)


class PointMassPlanarSeedTests(unittest.TestCase):
    def test_maps_speed_acceleration_time_and_kinematic_steering(self) -> None:
        lap = SimpleNamespace(
            speed=np.array([5.0, 6.0, 7.0, 6.0, 5.0]),
            longitudinal_acceleration=np.array([1.0, 2.0, -2.0, -1.0]),
        )
        source_distance = np.array([0.0, 1.0, 2.0, 3.0, 4.0])
        target = {
            "distance": np.array([0.0, 0.5, 1.5, 2.5, 3.5, 4.0]),
            "curvature": np.array([0.0, 0.1, 0.1, -0.1, -0.1, 0.0]),
        }

        seed = point_mass_planar_primal_seed(
            lap,
            source_distance,
            target,
            SimpleNamespace(
                wheelbase=1.5,
                storage=SimpleNamespace(initial_state_of_charge=0.93),
            ),
        )

        self.assertEqual(seed["state"].shape, (7, 6))
        self.assertEqual(seed["control"].shape, (3, 6))
        np.testing.assert_allclose(seed["state"][2], [5.0, 5.5, 6.5, 6.5, 5.5, 5.0])
        np.testing.assert_array_equal(seed["state"][3], np.zeros(6))
        np.testing.assert_allclose(seed["state"][4], seed["state"][2] * target["curvature"])
        np.testing.assert_allclose(seed["state"][5], 0.93)
        self.assertEqual(seed["state"][6, 0], 0.0)
        self.assertTrue(np.all(np.diff(seed["state"][6]) > 0.0))
        self.assertTrue(np.all(seed["control"][0] >= 0.0))
        self.assertTrue(np.all(seed["control"][1] >= 0.0))
        self.assertAlmostEqual(seed["control"][2, 1], np.arctan(0.15))

    def test_rejects_target_path_outside_source_coverage(self) -> None:
        lap = SimpleNamespace(
            speed=np.array([5.0, 5.0]),
            longitudinal_acceleration=np.array([0.0]),
        )
        target = {"distance": np.array([0.0, 2.0]), "curvature": np.zeros(2)}

        with self.assertRaisesRegex(ValueError, "coverage"):
            point_mass_planar_primal_seed(
                lap,
                np.array([0.0, 1.0]),
                target,
                SimpleNamespace(wheelbase=1.5),
            )
