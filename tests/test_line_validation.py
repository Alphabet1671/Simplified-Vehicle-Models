import unittest
from types import SimpleNamespace

import numpy as np

from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface
from simplified_vehicle_models.solvers.line_validation import (
    REQUIRED_NUMERICAL_CHECKS, strict_validation, validate_line, validation_status,
)
from simplified_vehicle_models.solvers.qss_spline_ocp import _SplineStateEvaluator
from simplified_vehicle_models.tracks.corridor import Corridor
from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory, ReferenceCurve
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.tracks.types import PathCoordinates


class _State:
    def __init__(self, acceleration=0.0):
        self.acceleration = acceleration

    def evaluate(self, station):
        station = np.asarray(station, dtype=float)
        speed = np.full(station.shape, 10.0)
        acceleration = np.full(station.shape, self.acceleration)
        # Reference metric is exactly 2 m/source-metre in all fixture tests.
        return {"speed": speed, "acceleration": acceleration,
                "speed_rate": acceleration * .2,
                "rhs_speed_rate": acceleration * .2}


def _fixture(coefficients=None, *, state=None, corridor_width=1.0):
    distance = np.linspace(0., 10., 5)
    coordinates = PathCoordinates(distance, 2. * distance, np.zeros(5),
                                  np.zeros(5), np.zeros(5), False)
    track = SimpleNamespace(path_coordinates=coordinates, is_closed=False, lap_length=10.)
    spline = OffsetSpline(track, 3.)
    trajectory = OffsetTrajectory(ReferenceCurve(track), spline,
                                  np.zeros(spline.control_count) if coefficients is None else coefficients)
    corridor = Corridor(distance, np.full(5, -corridor_width), np.full(5, corridor_width), "synthetic", False)
    surface = CapabilitySurface(np.array([.5, 15., 30.]), np.array([-.1, 0., .1]),
                                np.full((3, 3), 3.), np.full((3, 3), -8.),
                                np.full(3, 25.), "fixture")
    mesh = np.array([0., 5., 10.])
    evaluator = state or _SplineStateEvaluator(
        trajectory, mesh, np.full(3, 10.), np.zeros(3), np.zeros(2),
    )
    solution = SimpleNamespace(distance=mesh, trajectory=trajectory,
                               state_evaluator=evaluator, status="Solve_Succeeded",
                               lap_time=2.0)
    return solution, corridor, surface


class LineValidationTest(unittest.TestCase):
    def test_constant_speed_analytic_fixture_passes_dense_checks(self):
        solution, corridor, surface = _fixture()
        report = validate_line(solution, corridor, surface, samples_per_element=4)
        self.assertTrue(report["sampling_converged"])
        self.assertTrue(report["checks"]["geometry_consistency"])
        self.assertTrue(report["checks"]["dense_feasibility"])
        self.assertFalse(report["physically_validated"])
        self.assertTrue(report["surrogate_validated"])

    def test_between_node_capability_excursion_is_detected(self):
        solution, corridor, surface = _fixture(state=_State(3.45))
        report = validate_line(solution, corridor, surface, samples_per_element=4)
        acceleration = next(item for item in report["diagnostics"] if item["name"] == "acceleration_max")
        self.assertFalse(report["checks"]["dense_feasibility"])
        self.assertGreater(acceleration["maximum_violation"], .44)
        self.assertEqual(acceleration["unit"], "m/s^2")

    def test_dense_trajectory_not_supplied_nodes_detects_corridor_excursion(self):
        # The supplied solution has no nodal offset field: only the analytic
        # trajectory may establish containment. A cubic interior peak is > .5m.
        probe, _, _ = _fixture()
        coefficients = np.zeros(probe.trajectory.spline.control_count)
        coefficients[2] = 2.0
        solution, corridor, surface = _fixture(coefficients, corridor_width=.5)
        report = validate_line(solution, corridor, surface, samples_per_element=8)
        self.assertFalse(report["checks"]["dense_feasibility"])
        corridor_diag = next(item for item in report["diagnostics"] if item["name"] == "corridor")
        self.assertGreater(corridor_diag["maximum_violation"], 0.0)

    def test_missing_evaluator_is_incomplete_evidence(self):
        solution, corridor, surface = _fixture()
        solution.state_evaluator = None
        report = validate_line(solution, corridor, surface)
        self.assertIsNone(report["checks"]["dense_feasibility"])
        self.assertIn("dense state evaluator", report["missing_evidence"])

    def test_domains_and_zero_speed_fail_dense_check(self):
        solution, corridor, surface = _fixture()
        # Unsupported speed forces capability-domain failure rather than extrapolation.
        class Fast(_State):
            def evaluate(self, station):
                value = super().evaluate(station)
                value["speed"][:] = 31.
                return value
        solution.state_evaluator = Fast()
        report = validate_line(solution, corridor, surface)
        self.assertFalse(report["checks"]["dense_feasibility"])
        self.assertTrue(any(x["name"] == "capability_domain" for x in report["diagnostics"]))
        solution.state_evaluator = _State()
        class Zero(_State):
            def evaluate(self, station):
                value = super().evaluate(station)
                value["speed"][:] = 0.
                return value
        solution.state_evaluator = Zero()
        report = validate_line(solution, corridor, surface)
        self.assertFalse(report["checks"]["dense_feasibility"])
        self.assertFalse(next(x for x in report["diagnostics"] if x["name"] == "speed_domain")["passed"])

    def test_strict_tiers_fail_closed_and_never_imply_physical_validation(self):
        self.assertEqual(validation_status(True, {}, REQUIRED_NUMERICAL_CHECKS), "incomplete")
        checks = {name: True for name in REQUIRED_NUMERICAL_CHECKS}
        strict = strict_validation(True, checks)
        self.assertEqual(strict["validation_status"], "numerically_validated")
        self.assertTrue(strict["accepted"])
        self.assertFalse(strict["physically_validated"])
        checks["basis_convergence"] = None
        self.assertEqual(validation_status(True, checks), "incomplete")
        checks["basis_convergence"] = False
        self.assertEqual(validation_status(True, checks), "failed")
        self.assertEqual(validation_status(False, {name: True for name in REQUIRED_NUMERICAL_CHECKS}), "solver_failed")


if __name__ == "__main__":
    unittest.main()
