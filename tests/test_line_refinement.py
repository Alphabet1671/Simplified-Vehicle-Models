"""Unit tests for numerical line-refinement helpers; no CasADi required."""
from __future__ import annotations

from types import SimpleNamespace
import unittest

import numpy as np

from simplified_vehicle_models.solvers.line_refinement import (
    merge_refinement_mesh,
    project_energy_seed,
)


class _LinearSpline:
    """Small test basis exposing the OffsetSpline-compatible surface."""

    def __init__(self, control_distance, *, closed=False):
        self.control_distance = np.asarray(control_distance, dtype=float)
        self.control_count = self.control_distance.size
        self.is_closed = closed

    def basis(self, station):
        station = np.asarray(station, dtype=float)
        # Piecewise-linear hat functions are sufficient to verify basis transfer.
        matrix = np.zeros((station.size, self.control_count))
        for row, value in enumerate(station):
            if value <= self.control_distance[0]:
                matrix[row, 0] = 1.0
            elif value >= self.control_distance[-1]:
                matrix[row, -1] = 1.0
            else:
                index = np.searchsorted(self.control_distance, value) - 1
                left, right = self.control_distance[index:index + 2]
                matrix[row, index] = (right - value) / (right - left)
                matrix[row, index + 1] = (value - left) / (right - left)
        return matrix, matrix, matrix


class _Trajectory:
    def __init__(self, spline, coefficients, metric):
        self.spline = spline
        self._coefficients = np.asarray(coefficients, dtype=float)
        self._metric = metric

    def evaluate(self, station):
        s = np.asarray(station, dtype=float)
        return SimpleNamespace(
            metric=np.asarray(self._metric(s), dtype=float),
            offset=self.spline.basis(s)[0] @ self._coefficients,
        )


class _Evaluator:
    state_representation = "energy_hs_v1"

    def __init__(self, trajectory, energy, acceleration):
        self.trajectory = trajectory
        self._energy = energy
        self._acceleration = acceleration

    def evaluate(self, station):
        s = np.asarray(station, dtype=float)
        return {"energy": self._energy(s), "acceleration": self._acceleration(s)}


class LineRefinementTest(unittest.TestCase):
    def _solution(self, *, spline=None, coefficients=None, metric=lambda s: 1.0 + .2 * s,
                  energy=lambda s: 16.0 + 3.0 * s, acceleration=lambda s: .5 + 0.1 * s):
        spline = _LinearSpline([0.0, 1.0, 2.0] if spline is None else spline)
        coefficients = np.array([.1, -.2, .3]) if coefficients is None else coefficients
        trajectory = _Trajectory(spline, coefficients, metric)
        return SimpleNamespace(
            distance=np.array([0.0, 1.0, 2.0]), coefficients=np.asarray(coefficients),
            trajectory=trajectory, state_evaluator=_Evaluator(trajectory, energy, acceleration),
            state_representation="energy_hs_v1",
        )

    def test_merge_nonuniform_mixed_arithmetic_preserves_mesh_and_source_knots(self):
        mesh = np.array([0.0, 1.25, 2.5, 5.0])
        source = np.r_[np.arange(0.0, 5.0 + .025, .025), 1.25 + 1.e-14]
        merged = merge_refinement_mesh(mesh, source)
        np.testing.assert_array_equal(merged[np.searchsorted(merged, mesh)], mesh)
        self.assertTrue(np.any(np.isclose(merged, 1.275)))
        self.assertTrue(np.any(np.isclose(merged, 4.975)))
        self.assertGreaterEqual(np.min(np.diff(merged)), 1.e-7)
        self.assertEqual(merged[0], 0.0)
        self.assertEqual(merged[-1], 5.0)

    def test_merge_rejects_malformed_mesh_and_extra_stations(self):
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            merge_refinement_mesh([0.0, 1.0, 1.0], [])
        with self.assertRaisesRegex(ValueError, "finite"):
            merge_refinement_mesh([0.0, np.nan], [])
        with self.assertRaisesRegex(ValueError, "inside"):
            merge_refinement_mesh([0.0, 1.0], [1.1])
        with self.assertRaisesRegex(ValueError, "finite"):
            merge_refinement_mesh([0.0, 1.0], [np.inf])

    def test_projection_uses_exact_energy_and_hs_repair_on_nonunit_metric(self):
        solution = self._solution()
        mesh = np.array([0.0, .4, 1.3, 2.0])
        seed, report = project_energy_seed(solution, mesh)
        controls, nodes = solution.coefficients.size, mesh.size
        speed = seed[controls:controls + nodes]
        acceleration = seed[controls + nodes:controls + 2 * nodes]
        midpoint = seed[controls + 2 * nodes:]
        np.testing.assert_allclose(speed**2, 16.0 + 3.0 * mesh)
        np.testing.assert_allclose(acceleration, .5 + .1 * mesh)
        metric = 1.0 + .2 * mesh
        middle = 1.0 + .2 * (.5 * (mesh[:-1] + mesh[1:]))
        expected = (6 * np.diff(speed**2) / np.diff(mesh) - 2 * metric[:-1] * acceleration[:-1]
                    - 2 * metric[1:] * acceleration[1:]) / (8 * middle)
        np.testing.assert_allclose(midpoint, expected)
        self.assertLess(report["maximum_hs_energy_defect"], 1.e-13)
        self.assertTrue(report["projection_only"])
        self.assertFalse(report["solver_feasibility_claimed"])
        self.assertFalse(report["capability_checked"])

    def test_projection_transfers_offset_to_different_basis(self):
        from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory, ReferenceCurve
        from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
        from simplified_vehicle_models.tracks.types import PathCoordinates

        station = np.linspace(0.0, 2.0, 9)
        track = SimpleNamespace(
            lap_length=2.0, is_closed=False,
            path_coordinates=PathCoordinates(station, station, .1 * station**2,
                                             np.zeros_like(station), np.zeros_like(station), False),
        )
        source_spline = OffsetSpline(track, control_spacing=1.0)
        coefficients = np.linspace(-.1, .1, source_spline.control_count)
        trajectory = OffsetTrajectory(ReferenceCurve(track), source_spline, coefficients)
        solution = SimpleNamespace(
            distance=np.array([0.0, 1.0, 2.0]), coefficients=coefficients,
            trajectory=trajectory,
            state_evaluator=_Evaluator(trajectory, lambda s: 16.0 + 3.0 * s,
                                       lambda s: .5 + .1 * s),
            state_representation="energy_hs_v1",
        )
        new = OffsetSpline(track, control_spacing=.5)
        seed, report = project_energy_seed(solution, [0.0, 1.0, 2.0], new)
        sample = np.linspace(0.0, 2.0, 41)
        np.testing.assert_allclose(new.basis(sample)[0] @ seed[:new.control_count],
                                   trajectory.evaluate(sample).offset, atol=2.e-4)
        self.assertLess(report["maximum_hs_energy_defect"], 1.e-13)
        self.assertGreaterEqual(report["maximum_offset_projection_error"], 0.0)

    def test_projection_rejects_tiny_source_or_target_elements(self):
        solution = self._solution()
        with self.assertRaisesRegex(ValueError, "solution distance has stations closer"):
            project_energy_seed(
                SimpleNamespace(**{**solution.__dict__, "distance": np.array([0.0, 1.e-14, 2.0])}),
                [0.0, 1.0, 2.0],
            )
        with self.assertRaisesRegex(ValueError, "new_mesh has stations closer"):
            project_energy_seed(solution, [0.0, 1.e-14, 2.0])

    def test_projection_rejects_wrong_topology_domain_and_no_implicit_fallback(self):
        solution = self._solution()
        with self.assertRaisesRegex(ValueError, "domain"):
            project_energy_seed(solution, [0.0, 1.0, 2.1])
        with self.assertRaisesRegex(ValueError, "topology"):
            project_energy_seed(solution, [0.0, 1.0, 2.0], _LinearSpline([0.0, 1.0, 2.0], closed=True))
        solution.state_evaluator = None
        with self.assertRaisesRegex(ValueError, "evaluator"):
            project_energy_seed(solution, [0.0, 1.0, 2.0])


if __name__ == "__main__":
    unittest.main()
