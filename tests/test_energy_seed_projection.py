"""Tests for bounded mechanical energy-seed control projection."""
from __future__ import annotations

import unittest

import numpy as np

from simplified_vehicle_models.solvers.energy_seed_projection import (
    EnergySeedProjectionSettings,
    project_energy_seed_controls,
)


class EnergySeedProjectionTest(unittest.TestCase):
    def test_constant_metric_exact_hs_and_feasible_global_bounds(self):
        energy = np.array([16.0, 20.0, 29.0])
        mesh = np.array([0.0, 2.0, 5.0])
        nodes = np.array([1.0, 1.5, 2.0])
        middle = np.array([.875, 1.375])
        controls, report = project_energy_seed_controls(
            energy, mesh, np.ones(3), np.ones(2), nodes, middle, -3.0, 3.0,
        )
        self.assertIsNotNone(controls)
        np.testing.assert_allclose(controls, np.r_[nodes, middle])
        projected_node, projected_middle = controls[:energy.size], controls[energy.size:]
        residual = (2.0 * projected_node[:-1] + 8.0 * projected_middle
                    + 2.0 * projected_node[1:] - 6.0 * np.diff(energy) / np.diff(mesh))
        np.testing.assert_allclose(residual, 0.0, atol=1.0e-12)
        self.assertLessEqual(np.max(controls), 3.0)
        self.assertGreaterEqual(np.min(controls), -3.0)
        self.assertTrue(report["available"])
        self.assertLess(report["maximum_hs_energy_defect"], 1.0e-12)
        self.assertEqual(report["maximum_global_bound_violation"], 0.0)
        self.assertTrue(report["projection_only"])
        self.assertFalse(report["solver_feasibility_claimed"])
        self.assertFalse(report["capability_checked"])

    def test_out_of_bound_source_is_repaired_without_energy_clipping(self):
        energy = np.array([25.0, 25.0])
        node_metric = np.array([1.0, 1.4])
        midpoint_metric = np.array([1.2])
        source_node = np.array([4.02, 0.0])
        # This source is exact for fixed W but exceeds the global nodal bound.
        source_mid = np.array([-(2.0 * node_metric[0] * source_node[0]
                                 + 2.0 * node_metric[1] * source_node[1])
                                / (8.0 * midpoint_metric[0])])
        self.assertGreater(source_node[0], 3.71)
        controls, report = project_energy_seed_controls(
            energy, [0.0, 1.0], node_metric, midpoint_metric, source_node, source_mid,
            -3.71, 3.71,
        )
        self.assertIsNotNone(controls)
        self.assertLessEqual(np.max(controls), 3.71)
        self.assertGreaterEqual(np.min(controls), -3.71)
        self.assertAlmostEqual(controls[0], 3.71)
        node, middle = controls[:2], controls[2:]
        residual = (2.0 * node_metric[0] * node[0] + 8.0 * midpoint_metric[0] * middle[0]
                    + 2.0 * node_metric[1] * node[1])
        self.assertAlmostEqual(residual, 0.0, places=12)
        self.assertLess(report["maximum_hs_energy_defect"], 1.0e-12)
        # The function receives energy as read-only physical data and does not
        # return substitute/clipped node energy or speed values.
        np.testing.assert_array_equal(energy, [25.0, 25.0])
        self.assertTrue(report["energy_unchanged"])

    def test_impossible_energy_deficit_returns_none(self):
        controls, report = project_energy_seed_controls(
            [10.0, 100.0], [0.0, 1.0], [1.0, 1.0], [1.0], [0.0, 0.0], [0.0],
            -1.0, 1.0,
        )
        self.assertIsNone(controls)
        self.assertFalse(report["available"])
        self.assertEqual(report["reason"], "infeasible")
        self.assertTrue(report["energy_unchanged"])

    def test_periodic_endpoint_equality_is_enforced(self):
        controls, report = project_energy_seed_controls(
            [9.0, 9.0, 9.0], [0.0, 1.0, 2.0], [1.0, 1.0, 1.0], [1.0, 1.0],
            [1.0, -2.0, 3.0], [.25, -.25], -5.0, 5.0,
            settings=EnergySeedProjectionSettings(periodic_endpoint=True),
        )
        self.assertIsNotNone(controls)
        self.assertAlmostEqual(controls[0], controls[2])
        self.assertTrue(report["periodic_endpoint"])
        self.assertLess(report["maximum_hs_energy_defect"], 1.0e-12)

    def test_invalid_nan_and_settings_fail_clearly(self):
        arguments = ([9.0, np.nan], [0.0, 1.0], [1.0, 1.0], [1.0],
                     [0.0, 0.0], [0.0], -1.0, 1.0)
        with self.assertRaisesRegex(ValueError, "energy must contain only finite"):
            project_energy_seed_controls(*arguments)
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            project_energy_seed_controls([9.0, 9.0], [0.0, 0.0], [1.0, 1.0], [1.0],
                                         [0.0, 0.0], [0.0], -1.0, 1.0)
        with self.assertRaisesRegex(TypeError, "settings must be"):
            project_energy_seed_controls([9.0, 9.0], [0.0, 1.0], [1.0, 1.0], [1.0],
                                         [0.0, 0.0], [0.0], -1.0, 1.0, settings=object())
        for field, invalid in (("minimum_spacing", np.nan), ("minimum_spacing", True),
                               ("minimum_spacing", "small"), ("equality_tolerance", np.nan),
                               ("equality_tolerance", True), ("equality_tolerance", "tight")):
            with self.subTest(field=field, invalid=invalid), \
                 self.assertRaisesRegex(ValueError, f"settings.{field}"):
                project_energy_seed_controls(
                    [9.0, 9.0], [0.0, 1.0], [1.0, 1.0], [1.0], [0.0, 0.0], [0.0], -1.0, 1.0,
                    settings=EnergySeedProjectionSettings(**{field: invalid}),
                )
        with self.assertRaisesRegex(ValueError, "required length"):
            project_energy_seed_controls([9.0, 9.0], [0.0, 1.0], [1.0], [1.0],
                                         [0.0, 0.0], [0.0], -1.0, 1.0)
        with self.assertRaisesRegex(ValueError, "positive"):
            project_energy_seed_controls([0.0, 9.0], [0.0, 1.0], [1.0, 1.0], [1.0],
                                         [0.0, 0.0], [0.0], -1.0, 1.0)


if __name__ == "__main__":
    unittest.main()
