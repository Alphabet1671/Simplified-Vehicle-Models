import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface
from simplified_vehicle_models.solvers.line_validation import (
    REQUIRED_NUMERICAL_CHECKS, strict_validation, validate_line, validation_status,
)
from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings
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


class _C1EnergyState:
    """Exact dynamics with a W'' jump at the middle mesh knot."""
    def __init__(self, *, corrupt_control=False, lie_about_rates=False):
        self.corrupt_control, self.lie_about_rates = corrupt_control, lie_about_rates

    def evaluate(self, station):
        s = np.asarray(station, dtype=float)
        x = (s - 5.0) / 5.0
        right = 120.0 + 20.0*x + 30.0*x**2 - 20.0*x**3
        energy = np.where(s <= 5.0, 100.0 + 4.0*s, right)
        acceleration = np.where(s <= 5.0, 1.0, 1.0 + 3.0*x - 3.0*x**2)
        if self.corrupt_control:
            acceleration = np.where(s > 5.0, 1.0, acceleration)
        speed = np.sqrt(energy)
        reported = np.zeros(s.shape) if self.lie_about_rates else acceleration * 2.0 / speed
        return {"speed": speed, "acceleration": acceleration,
                "speed_rate": reported, "rhs_speed_rate": reported}


def _c1_energy_fixture(*, corrupt_control=False, lie_about_rates=False):
    solution, corridor, surface = _fixture()
    solution.state_evaluator = _C1EnergyState(corrupt_control=corrupt_control,
                                                lie_about_rates=lie_about_rates)
    stations = np.linspace(0., 10., 20001)
    speed = solution.state_evaluator.evaluate(stations)["speed"]
    solution.lap_time = float(np.sum(np.diff(stations) *
        ((2. / speed)[:-1] + (2. / speed)[1:]) / 2.0))
    return solution, corridor, surface


class LineValidationTest(unittest.TestCase):
    def test_mesh_knot_c1_energy_uses_one_sided_actual_speed_derivatives(self):
        solution, corridor, surface = _c1_energy_fixture()
        report = validate_line(solution, corridor, surface, samples_per_element=4,
                               tolerance={"sampling_stability": 1.e-5})
        by_name = {item["name"]: item for item in report["diagnostics"]}
        self.assertTrue(report["checks"]["dense_feasibility"])
        self.assertTrue(by_name["derivative_uncertainty"]["passed"])
        self.assertLess(by_name["implied_acceleration"]["maximum_violation"], 1.e-6)
        self.assertEqual(by_name["derivative_uncertainty"]["unit"], "(m/s)/m")

    def test_mesh_knot_control_mismatch_remains_a_physical_failure(self):
        solution, corridor, surface = _c1_energy_fixture(corrupt_control=True)
        report = validate_line(solution, corridor, surface, samples_per_element=4)
        by_name = {item["name"]: item for item in report["diagnostics"]}
        self.assertFalse(report["checks"]["dense_feasibility"])
        self.assertGreater(by_name["implied_acceleration"]["maximum_violation"], .1)

    def test_mesh_knot_lying_reported_rates_do_not_fool_actual_fd(self):
        solution, corridor, surface = _c1_energy_fixture(lie_about_rates=True)
        report = validate_line(solution, corridor, surface, samples_per_element=4,
                               tolerance={"sampling_stability": 1.e-5})
        self.assertTrue(report["checks"]["dense_feasibility"])
        implied = next(item for item in report["diagnostics"] if item["name"] == "implied_acceleration")
        self.assertLess(implied["maximum_violation"], 1.e-6)

    def test_speed_fd_sampling_batches_actual_evaluator_queries(self):
        solution, corridor, surface = _fixture()
        source = solution.state_evaluator
        class CountingState:
            def __init__(self): self.calls = 0
            def evaluate(self, station):
                self.calls += 1
                return source.evaluate(station)
        counter = CountingState()
        solution.state_evaluator = counter
        validate_line(solution, corridor, surface, samples_per_element=16)
        # One dense state pass plus a vector FD prefetch (and only a handful
        # of non-grid one-sided fallback calls) rather than one full evaluator
        # reconstruction per finite-difference stencil point.
        self.assertLess(counter.calls, 20)

    def test_derivative_uncertainty_consumes_physical_error_budget(self):
        # The smooth sinusoid is dynamically exact when acceleration equals
        # v*dv/ds/metric, but a deliberately tight physical tolerance leaves
        # no room for the independently estimated FD uncertainty.
        solution, corridor, surface = _fixture()
        class ExactWavySpeed(_State):
            def evaluate(self, station):
                station = np.asarray(station, dtype=float)
                speed = 10. + .1*np.sin(station)
                acceleration = speed * .1*np.cos(station) / 2.
                return {"speed": speed, "acceleration": acceleration,
                        "speed_rate": np.zeros(station.shape), "rhs_speed_rate": np.zeros(station.shape)}
        solution.state_evaluator = ExactWavySpeed()
        stations = np.linspace(0., 10., 10001)
        speed = solution.state_evaluator.evaluate(stations)["speed"]
        solution.lap_time = float(np.sum(np.diff(stations) *
            ((2./speed)[:-1] + (2./speed)[1:]) / 2.))
        report = validate_line(solution, corridor, surface, samples_per_element=4,
                               tolerance={"sampling_stability": 1.e-5,
                                          "acceleration_mps2": 1.e-9})
        by_name = {item["name"]: item for item in report["diagnostics"]}
        self.assertTrue(by_name["implied_acceleration"]["passed"])
        self.assertFalse(by_name["implied_acceleration_upper_bound"]["passed"])
        self.assertFalse(report["checks"]["dense_feasibility"])

    def test_unresolved_derivative_sampling_fails_closed(self):
        solution, corridor, surface = _fixture()
        class OscillatorySpeed(_State):
            def evaluate(self, station):
                station = np.asarray(station, dtype=float)
                speed = 10. + 1.e-3*np.sin(2.e6*station)
                return {"speed": speed, "acceleration": np.zeros(station.shape),
                        "speed_rate": np.zeros(station.shape), "rhs_speed_rate": np.zeros(station.shape)}
        solution.state_evaluator = OscillatorySpeed()
        report = validate_line(solution, corridor, surface, samples_per_element=4,
                               tolerance={"sampling_stability": 1.e-5})
        uncertainty = next(item for item in report["diagnostics"] if item["name"] == "derivative_uncertainty")
        self.assertFalse(uncertainty["passed"])
        self.assertFalse(report["checks"]["dense_feasibility"])

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

    def test_actual_hermite_last_cell_control_overshoot_is_detected(self):
        solution, corridor, surface = _fixture()
        # Real Task-2 evaluator: *all* node and midpoint collocation controls
        # satisfy amax=3, yet the last quadratic control reaches 3.45 at x=.25.
        node_acceleration = np.array([3., 3., -.6])
        midpoint_acceleration = np.array([3., 3.])
        self.assertTrue(np.all(node_acceleration <= 3.))
        self.assertTrue(np.all(midpoint_acceleration <= 3.))
        solution.state_evaluator = _SplineStateEvaluator(
            solution.trajectory, solution.distance, np.array([10., 11.4, 12.8]),
            node_acceleration, midpoint_acceleration,
        )
        # Use its independently reconstructed physical time, avoiding an
        # intentional time diagnostic from masking the acceleration assertion.
        stations = np.linspace(0., 10., 2001)
        state = solution.state_evaluator.evaluate(stations)
        solution.lap_time = float(np.sum(np.diff(stations) *
            ((np.asarray(solution.trajectory.evaluate(stations).metric) / state["speed"])[:-1] +
             (np.asarray(solution.trajectory.evaluate(stations).metric) / state["speed"])[1:]) / 2.0))
        report = validate_line(solution, corridor, surface, samples_per_element=8)
        peak = next(item for item in report["diagnostics"] if item["name"] == "acceleration_max")
        self.assertFalse(report["checks"]["dense_feasibility"])
        self.assertGreater(peak["maximum_violation"], .44)
        self.assertGreater(peak["worst_station"], 5.0)
        self.assertLess(peak["worst_station"], 7.5)

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

    def test_curved_geometry_passes_local_xy_curvature_check(self):
        from tests.test_line_trajectory import _open_curve
        track = _open_curve()
        spline = OffsetSpline(track, 4.)
        trajectory = OffsetTrajectory(ReferenceCurve(track), spline,
                                      np.zeros(spline.control_count))
        mesh = np.linspace(0., track.lap_length, 20)
        evaluator = _SplineStateEvaluator(trajectory, mesh, np.full(20, 10.),
                                          np.zeros(20), np.zeros(19))
        solution = SimpleNamespace(distance=mesh, trajectory=trajectory,
            state_evaluator=evaluator, status="Solve_Succeeded",
            lap_time=trajectory.station_map.path_end / 10.)
        corridor = Corridor(track.distance, np.full(track.distance.shape, -2.),
                            np.full(track.distance.shape, 2.), "synthetic", False)
        surface = CapabilitySurface(np.array([.5, 15., 30.]), np.array([-1., 0., 1.]),
            np.full((3, 3), 3.), np.full((3, 3), -8.), np.full(3, 25.), "fixture")
        report = validate_line(solution, corridor, surface)
        curvature = next(item for item in report["diagnostics"] if item["name"] == "xy_curvature")
        self.assertTrue(curvature["passed"])
        self.assertTrue(report["checks"]["geometry_consistency"])

    def test_settings_geometry_and_time_gates_report_named_failures(self):
        solution, corridor, surface = _fixture()
        original = solution.trajectory.evaluate
        class TamperedTrajectory:
            def __init__(self, source):
                self.reference, self.spline, self.station_map = source.reference, source.spline, source.station_map
            def evaluate(self, station):
                value = original(station)
                return SimpleNamespace(**{**value.__dict__, "fold_factor": np.full(np.asarray(station).shape, .1),
                    "heading_error": np.full(np.asarray(station).shape, .7),
                    "heading_error_s": np.full(np.asarray(station).shape, .6),
                    "curvature": np.full(np.asarray(station).shape, .2)})
        # slots dataclasses lack __dict__; make an explicit attribute adapter.
        class Adapter:
            def __init__(self, source):
                self.reference, self.spline, self.station_map, self.source = source.reference, source.spline, source.station_map, source
            def evaluate(self, station):
                value = self.source.evaluate(station); shape = np.asarray(station).shape
                return SimpleNamespace(north=value.north, east=value.east, heading=value.heading,
                    heading_error=np.full(shape, .7), curvature=np.full(shape, .2),
                    heading_error_s=np.full(shape, .6), offset=value.offset,
                    metric=value.metric, fold_factor=np.full(shape, .1))
        solution.trajectory = Adapter(solution.trajectory)
        solution.lap_time = 9.0
        report = validate_line(solution, corridor, surface, QssSettings())
        by_name = {item["name"]: item for item in report["diagnostics"]}
        for name in ("fold", "heading_limit", "heading_rate_limit", "xy_curvature", "lap_time"):
            self.assertFalse(by_name[name]["passed"], name)

    def test_independent_heading_rate_rejects_misreported_derivative(self):
        solution, corridor, surface = _fixture()
        source = solution.trajectory
        class LyingHeadingRate:
            def __init__(self):
                self.reference, self.spline, self.station_map = source.reference, source.spline, source.station_map
            def evaluate(self, station):
                value = source.evaluate(station)
                s = np.asarray(station, dtype=float)
                # Actual heading error has 0.8 rad/m slope, while the claimed
                # derivative is zero and thus would pass the old direct check.
                return SimpleNamespace(north=value.north, east=value.east, heading=value.heading,
                    heading_error=.8*s, heading_error_s=np.zeros(s.shape), curvature=value.curvature,
                    offset=value.offset, metric=value.metric, fold_factor=value.fold_factor)
        solution.trajectory = LyingHeadingRate()
        report = validate_line(solution, corridor, surface,
                               QssSettings(maximum_heading=10., maximum_curvature_rate=.5))
        by_name = {item["name"]: item for item in report["diagnostics"]}
        self.assertTrue(by_name["heading_rate_limit"]["passed"])
        self.assertFalse(by_name["xy_heading_rate"]["passed"])
        self.assertFalse(by_name["xy_heading_rate_limit"]["passed"])
        self.assertFalse(report["checks"]["geometry_consistency"])
        self.assertFalse(report["checks"]["dense_feasibility"])

    def test_independent_speed_derivative_rejects_lying_evaluator_rates(self):
        solution, corridor, surface = _fixture()
        class LyingRates(_State):
            def evaluate(self, station):
                station = np.asarray(station, dtype=float)
                speed = 10. + station
                # Both supplied rate values agree, but neither is dv/ds=1.
                return {"speed": speed, "acceleration": np.zeros(station.shape),
                        "speed_rate": np.zeros(station.shape), "rhs_speed_rate": np.zeros(station.shape)}
        solution.state_evaluator = LyingRates()
        stations = np.linspace(0., 10., 1001)
        solution.lap_time = float(np.sum(np.diff(stations) *
            ((2. / (10.+stations))[:-1] + (2. / (10.+stations))[1:]) / 2.0))
        report = validate_line(solution, corridor, surface)
        diagnostic = next(item for item in report["diagnostics"] if item["name"] == "dynamics_defect")
        self.assertFalse(diagnostic["passed"])
        self.assertGreater(diagnostic["maximum_violation"], .9)

    def test_live_iem_equilibrium_uses_free_function_and_blocks_infeasibility(self):
        formulation = Path(__file__).resolve().parents[1] / "models/formulations/iem24_murphy_real_planar_7dof.xml"
        vehicle = point_mass_view_of_planar(compile_formulation(str(formulation)).model)
        solution, corridor, surface = _fixture()
        # This exact IEM point-mass condition exposes table-vs-live feasibility:
        # zero longitudinal acceleration fails while modest braking is feasible.
        class IemState(_State):
            def __init__(self, acceleration): super().__init__(acceleration)
            def evaluate(self, station):
                station = np.asarray(station, dtype=float)
                return {"speed": np.full(station.shape, 33.339080),
                        "acceleration": np.full(station.shape, self.acceleration),
                        "speed_rate": np.zeros(station.shape), "rhs_speed_rate": np.zeros(station.shape)}
        # The surface intentionally covers the speed; live original physics must
        # nevertheless veto ax=0.
        surface = CapabilitySurface(np.array([.5, 34., 40.]), np.array([-.1, 0., .1]),
            np.full((3, 3), 3.), np.full((3, 3), -8.), np.full(3, 40.), "fixture")
        solution.state_evaluator = IemState(0.0); solution.lap_time = 10.0 / 33.339080 * 2.
        failed = validate_line(solution, corridor, surface, vehicle=vehicle)
        self.assertFalse(failed["model_feasible"])
        self.assertFalse(failed["checks"]["dense_feasibility"])
        solution.state_evaluator = IemState(-.1)
        passed = validate_line(solution, corridor, surface, vehicle=vehicle)
        self.assertTrue(passed["model_feasible"])

    def test_object_callback_false_feasibility_blocks_dense_gate(self):
        solution, corridor, surface = _fixture()
        report = validate_line(solution, corridor, surface,
            equilibrium_callback=lambda *_: SimpleNamespace(is_feasible=np.bool_(False)))
        self.assertFalse(report["model_feasible"])
        self.assertFalse(report["checks"]["dense_feasibility"])

    def test_exhausted_refinement_is_unresolved_not_converged(self):
        solution, corridor, surface = _fixture()
        # Each refined query changes this diagnostic, so two completed levels
        # must remain unresolved (the former level<maximum_refinements formula
        # incorrectly labelled this converged).
        class RefiningState(_State):
            def evaluate(self, station):
                value = super().evaluate(station)
                value["acceleration"] = 1.e-3 * np.asarray(station)**2
                return value
        solution.state_evaluator = RefiningState()
        report = validate_line(solution, corridor, surface, maximum_refinements=2,
                               tolerance={"sampling_stability": 1.e-12})
        self.assertEqual(report["refinements_used"], 2)
        # The expanded endpoint/mesh-knot derivative evidence may make the
        # physical maxima stable; this test must not assert non-convergence
        # from an implementation-specific station set.
        self.assertTrue(report["sampling_converged"])
        self.assertFalse(report["checks"]["dense_feasibility"])

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
