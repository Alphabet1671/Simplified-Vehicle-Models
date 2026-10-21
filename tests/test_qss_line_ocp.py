import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface
from simplified_vehicle_models.tracks.corridor import corridor_from_track
from simplified_vehicle_models.tracks.profile import load_track
from tests.test_track_width import _write_circle_track


def _has_casadi() -> bool:
    try:
        import casadi  # noqa: F401
    except ImportError:
        return False
    return True


CASADI = _has_casadi()


def _flat_surface(ceiling: float, *, acceleration: float = 3.0, deceleration: float = 8.0) -> CapabilitySurface:
    speed = np.linspace(0.5, 40.0, 81)
    curvature = np.linspace(-0.3, 0.3, 61)
    return CapabilitySurface(speed, curvature, np.full((speed.size, curvature.size), acceleration), np.full((speed.size, curvature.size), -deceleration), np.full(curvature.size, ceiling), "synthetic")


@unittest.skipUnless(CASADI, "CasADi is not installed")
class TranscriptionTest(unittest.TestCase):
    def setUp(self):
        self._directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._directory.name)

    def tearDown(self):
        self._directory.cleanup()

    def _track(self, **kwargs):
        defaults = {"radius": 40.0, "nodes": 360, "half_width": 3.0}
        defaults.update(kwargs)
        return load_track(_write_circle_track(self.directory, **defaults))

    def test_mesh_ends_on_the_lap_length(self):
        from simplified_vehicle_models.solvers.qss_line_ocp import build_mesh
        track = self._track(); mesh = build_mesh(track, spacing=2.5)
        self.assertAlmostEqual(float(mesh[0]), 0.0)
        self.assertAlmostEqual(float(mesh[-1]), track.lap_length, places=9)
        self.assertTrue(np.all(np.diff(mesh) > 0.0))

    def test_scaled_solver_returns_physical_solution_values(self):
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh, solve_line

        track = self._track(radius=40.0, nodes=72, half_width=3.0)
        corridor = corridor_from_track(track)
        solution = solve_line(
            track, corridor, _flat_surface(25.0), build_mesh(track, 4.0),
            QssSettings(tolerance=1.0e-7, maximum_iterations=1000),
        )
        self.assertEqual(solution.status, "Solve_Succeeded")
        self.assertTrue(np.all(np.isfinite(solution.speed)))
        self.assertGreaterEqual(float(solution.speed.min()), 0.5)
        self.assertLessEqual(float(solution.speed.max()), 40.0)
        self.assertLessEqual(solution.maximum_violation, 1.0e-7)
        self.assertIsNotNone(solution.mid_acceleration)
        self.assertIsNotNone(solution.mid_curvature_rate)
        self.assertEqual(solution.mid_acceleration.size, solution.distance.size - 1)
        self.assertEqual(solution.mid_curvature_rate.size, solution.distance.size - 1)

    def test_nonfinite_seed_is_rejected_before_solver_construction(self):
        from unittest.mock import patch
        from simplified_vehicle_models.solvers import qss_line_ocp

        track = self._track()
        mesh = qss_line_ocp.build_mesh(track, 4.0)
        corridor = corridor_from_track(track)
        seed = qss_line_ocp.initial_guess(track, corridor, _flat_surface(25.0), mesh, qss_line_ocp.QssSettings())
        seed[0] = np.nan
        with patch("casadi.nlpsol", side_effect=AssertionError("solver must not be constructed")):
            with self.assertRaisesRegex(ValueError, "finite values"):
                qss_line_ocp.solve_line(track, corridor, _flat_surface(25.0), mesh,
                                        qss_line_ocp.QssSettings(), seed=seed)

    def test_open_track_is_rejected_at_periodic_solver_boundary(self):
        from simplified_vehicle_models.solvers.qss_line_ocp import build_mesh, build_ocp, QssSettings
        track=self._track(); mesh=build_mesh(track,4.0); corridor=corridor_from_track(track)
        object.__setattr__(track, "is_closed", False)
        with self.assertRaisesRegex(ValueError, "closed track"):
            build_ocp(track,corridor,_flat_surface(25.0),mesh,QssSettings())

    def test_acceleration_variable_bounds_are_asymmetric(self):
        from simplified_vehicle_models.solvers.qss_line_ocp import build_mesh, build_ocp, QssSettings
        track = self._track()
        surface = _flat_surface(25.0, acceleration=3.0, deceleration=8.0)
        built = build_ocp(track, corridor_from_track(track), surface, build_mesh(track, 4.0), QssSettings())
        nodes = built["mesh"].size
        np.testing.assert_allclose(built["x_lower"][3 * nodes:4 * nodes], -8.0)
        np.testing.assert_allclose(built["x_upper"][3 * nodes:4 * nodes], 3.0)
        np.testing.assert_allclose(built["x_lower"][5 * nodes:5 * nodes + nodes - 1], -8.0)
        np.testing.assert_allclose(built["x_upper"][5 * nodes:5 * nodes + nodes - 1], 3.0)

    def test_decision_scaling_rejects_invalid_node_count(self):
        from simplified_vehicle_models.solvers.qss_line_ocp import _decision_scaling

        with self.assertRaisesRegex(ValueError, "at least two"):
            _decision_scaling(1)
        for invalid in (True, 3.5):
            with self.subTest(invalid=invalid), self.assertRaisesRegex(ValueError, "must be an integer"):
                _decision_scaling(invalid)

    def test_decision_scaling_preserves_grouped_bounds_and_centers_speed(self):
        from simplified_vehicle_models.solvers.qss_line_ocp import _decision_scaling

        nodes = 4
        center, scale = _decision_scaling(nodes)
        elements = nodes - 1
        self.assertEqual(center.size, 5 * nodes + 2 * elements)
        np.testing.assert_array_equal(center[2 * nodes:3 * nodes], np.full(nodes, 20.0))
        for start, size, expected in (
            (0, nodes, 1.0), (nodes, nodes, 0.15), (2 * nodes, nodes, 10.0),
            (3 * nodes, nodes, 5.0), (4 * nodes, nodes, 0.1),
            (5 * nodes, elements, 5.0), (5 * nodes + elements, elements, 0.1),
        ):
            np.testing.assert_array_equal(scale[start:start + size], np.full(size, expected))
        physical = np.linspace(-3.0, 35.0, center.size)
        scaled = (physical - center) / scale
        np.testing.assert_allclose(center + scale * scaled, physical)

    def test_decision_layout_matches_the_declared_contract(self):
        from simplified_vehicle_models.solvers.qss_line_ocp import build_mesh, build_ocp, QssSettings
        track=self._track(); mesh=build_mesh(track,4.0); corridor=corridor_from_track(track)
        built=build_ocp(track,corridor,_flat_surface(25.0),mesh,QssSettings())
        count=mesh.size-1; nodes=count+1; x=built["nlp"]["x"]
        self.assertEqual(x.shape[0],5*nodes+2*count)
        self.assertEqual([x[i*nodes].name() for i in range(5)], ["n_0","psi_0","v_0","a_0","q_0"])
        self.assertEqual(x[5*nodes].name(),"am_0")
        self.assertEqual(x[5*nodes+count].name(),"qm_0")
        from simplified_vehicle_models.solvers.qss_line_ocp import unpack
        blocks=unpack(np.arange(5*nodes+2*count),count)
        np.testing.assert_array_equal(blocks["offset"],np.arange(nodes))
        np.testing.assert_array_equal(blocks["heading"],np.arange(nodes,2*nodes))
        np.testing.assert_array_equal(blocks["speed"],np.arange(2*nodes,3*nodes))

    def test_invalid_settings_and_mesh_are_rejected(self):
        from simplified_vehicle_models.solvers.qss_line_ocp import build_mesh, build_ocp, QssSettings
        track=self._track(); corridor=corridor_from_track(track); surface=_flat_surface(25.0); mesh=build_mesh(track,4.0)
        for settings in (QssSettings(fold_guard=float("nan")), QssSettings(minimum_speed=0.0), QssSettings(maximum_iterations=0)):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                build_ocp(track,corridor,surface,mesh,settings)
        with self.assertRaisesRegex(ValueError,"span exactly"):
            build_ocp(track,corridor,surface,mesh[:-1],QssSettings())

    def test_midpoint_offset_has_corridor_constraint(self):
        from simplified_vehicle_models.solvers.qss_line_ocp import build_mesh, build_ocp, QssSettings
        from dataclasses import replace
        track=self._track(); mesh=build_mesh(track,4.0); corridor=corridor_from_track(track)
        corridor=replace(corridor,minimum=np.full(corridor.distance.shape,-1.7),maximum=np.full(corridor.distance.shape,2.3))
        built=build_ocp(track,corridor,_flat_surface(25.0),mesh,QssSettings())
        # These asymmetric sentinels are unique to corridor bounds (not the
        # symmetric surface-curvature domain) and one pair is added per midpoint.
        count=mesh.size-1
        rows=np.flatnonzero((built["g_lower"] == -1.7) & (built["g_upper"] == 2.3))
        self.assertEqual(rows.size,3*count)  # shared HS/Hermite midpoint plus quarter and three-quarter samples

    def test_signed_asymmetric_curvature_axis_rejects_outside_track_curvature(self):
        from dataclasses import replace
        from simplified_vehicle_models.solvers.qss_line_ocp import build_mesh, build_ocp, QssSettings
        track=self._track(); track=replace(track,curvature=np.full(track.curvature.shape,-0.15)); corridor=corridor_from_track(self._track()); base=_flat_surface(25.0)
        curvature=np.linspace(-0.1,0.2,61)
        surface=replace(base,curvature=curvature,acceleration_max=np.full((base.speed.size,curvature.size),3.0),acceleration_min=np.full((base.speed.size,curvature.size),-8.0),speed_ceiling=np.full(curvature.size,25.0))
        with self.assertRaisesRegex(ValueError,"track curvature lies outside"):
            build_ocp(track,corridor,surface,build_mesh(track,4.0),QssSettings())

    def test_solver_result_rejects_failure_nonfinite_and_violation(self):
        from simplified_vehicle_models.solvers.qss_line_ocp import _validate_solver_result
        valid=np.array([1.0]); _validate_solver_result("Solve_Succeeded",valid,valid,1.0,0.0,1e-6)
        with self.assertRaisesRegex(RuntimeError,"failed"):
            _validate_solver_result("Maximum_Iterations_Exceeded",valid,valid,1.0,0.0,1e-6)
        with self.assertRaisesRegex(RuntimeError,"non-finite"):
            _validate_solver_result("Solve_Succeeded",np.array([np.nan]),valid,1.0,0.0,1e-6)
        with self.assertRaisesRegex(RuntimeError,"violates constraints"):
            _validate_solver_result("Solve_Succeeded",valid,valid,1.0,1e-3,1e-6)
        with self.assertRaisesRegex(RuntimeError,"violates constraints"):
            _validate_solver_result("Solve_Succeeded",np.array([1.1]),valid,1.0,0.0,1e-6,np.array([0.0]),np.array([1.0]))

    def test_zero_corridor_recovers_the_steady_circle_time(self):
        from simplified_vehicle_models.solvers.qss_line_ocp import build_mesh, solve_line, QssSettings
        radius=40.0; track=self._track(radius=radius,half_width=2.25); corridor=corridor_from_track(track,margin=2.25)
        solution=solve_line(track,corridor,_flat_surface(20.0),build_mesh(track,2.0),QssSettings(maximum_iterations=4000))
        self.assertEqual(solution.status,"Solve_Succeeded")
        expected=2*math.pi*radius/20.0
        self.assertAlmostEqual(solution.lap_time,expected,delta=2e-3*expected)
        self.assertLess(float(np.max(np.abs(solution.offset))),1e-9)

    def test_right_turn_free_line_hugs_the_positive_edge(self):
        from simplified_vehicle_models.solvers.qss_line_ocp import build_mesh, solve_line, QssSettings
        radius,width=40.0,3.0; track=self._track(radius=radius,half_width=width,direction=1.0); corridor=corridor_from_track(track)
        solution=solve_line(track,corridor,_flat_surface(25.0),build_mesh(track,2.0),QssSettings(maximum_iterations=6000))
        self.assertEqual(solution.status,"Solve_Succeeded")
        self.assertGreater(float(np.min(solution.offset)),.9*width)
        expected=2*math.pi*(radius-width)/25.0
        self.assertAlmostEqual(solution.lap_time,expected,delta=5e-3*expected)

    def test_left_turn_free_line_hugs_the_negative_edge(self):
        from simplified_vehicle_models.solvers.qss_line_ocp import build_mesh, solve_line, QssSettings
        radius,width=40.0,3.0; track=self._track(radius=radius,half_width=width,direction=-1.0); corridor=corridor_from_track(track)
        solution=solve_line(track,corridor,_flat_surface(25.0),build_mesh(track,2.0),QssSettings(maximum_iterations=6000))
        self.assertEqual(solution.status,"Solve_Succeeded")
        self.assertLess(float(np.max(solution.offset)),-.9*width)
        expected=2*math.pi*(radius-width)/25.0
        self.assertAlmostEqual(solution.lap_time,expected,delta=5e-3*expected)


@unittest.skipUnless(CASADI, "CasADi is not installed")
class LadderTest(unittest.TestCase):
    def setUp(self) -> None:
        self._directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._directory.name)

    def tearDown(self) -> None:
        self._directory.cleanup()

    def test_ladder_converges_and_is_mesh_independent(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, solve_ladder

        track = load_track(_write_circle_track(self.directory, radius=40.0, nodes=360, half_width=3.0))
        corridor = corridor_from_track(track)
        stages = solve_ladder(track, corridor, _flat_surface(25.0), QssSettings(maximum_iterations=6000), spacings=(4.0, 2.0, 1.0))
        self.assertEqual(len(stages), 3)
        for stage in stages:
            self.assertEqual(stage.status, "Solve_Succeeded")
        coarse, finest = stages[-2], stages[-1]
        self.assertAlmostEqual(finest.lap_time, coarse.lap_time, delta=5.0e-3 * coarse.lap_time)

    def test_next_stage_receives_next_mesh_projection(self) -> None:
        from unittest.mock import patch
        from simplified_vehicle_models.solvers import qss_line_ocp

        track = load_track(_write_circle_track(self.directory, radius=40.0, nodes=360, half_width=3.0))
        corridor = corridor_from_track(track)
        settings = qss_line_ocp.QssSettings()
        meshes = [qss_line_ocp.build_mesh(track, s) for s in (8.0, 4.0)]
        captured = []
        first_stage = []
        callbacks = []
        def fake_callback(spacing, solution, wall_seconds):
            callbacks.append((spacing, solution, wall_seconds))

        def fake_solve(track, corridor, surface, mesh, settings, *, seed=None):
            captured.append((np.asarray(mesh).copy(), seed))
            values = qss_line_ocp.initial_guess(track, corridor, surface, mesh, settings)
            blocks = qss_line_ocp.unpack(values, len(mesh)-1)
            solution = qss_line_ocp.QssSolution(np.asarray(mesh), blocks["offset"], blocks["heading"], blocks["speed"], blocks["acceleration"], blocks["curvature_rate"], 1.0, "Solve_Succeeded", 1, 0.0)
            captured[-1] = (*captured[-1], solution)
            if not first_stage:
                first_stage.append(solution)
            return solution

        with patch.object(qss_line_ocp, "solve_line", side_effect=fake_solve):
            stages = qss_line_ocp.solve_ladder(
                track, corridor, _flat_surface(25.0), settings,
                spacings=(8.0, 4.0), stage_callback=fake_callback,
            )
        expected = qss_line_ocp.project_seed(first_stage[0], meshes[1])
        self.assertEqual(captured[1][0].size, meshes[1].size)
        self.assertEqual(captured[1][1].size, 5 * meshes[1].size + 2 * (meshes[1].size - 1))
        np.testing.assert_array_equal(captured[1][1], expected)
        self.assertEqual([entry[0] for entry in callbacks], [8.0, 4.0])
        self.assertIs(callbacks[0][1], stages[0])
        self.assertIs(callbacks[1][1], stages[1])
        self.assertTrue(all(np.isfinite(entry[2]) and entry[2] >= 0.0 for entry in callbacks))

    def test_project_seed_preserves_midpoint_controls_on_same_mesh(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSolution, project_seed

        mesh = np.array([0.0, 2.0, 4.0, 6.0])
        nodes = np.arange(mesh.size, dtype=float)
        mid_a = np.array([10.0, 20.0, 30.0])
        mid_q = np.array([-1.0, -2.0, -3.0])
        solution = QssSolution(mesh, nodes, nodes, nodes, nodes, nodes, 1.0,
                               "Solve_Succeeded", 1, 0.0, mid_a, mid_q)
        projected = project_seed(solution, mesh)
        np.testing.assert_array_equal(projected[-6:-3], mid_a)
        np.testing.assert_array_equal(projected[-3:], mid_q)

    def test_project_seed_periodically_interpolates_midpoint_controls(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSolution, project_seed

        source = np.array([0.0, 2.0, 4.0, 6.0])
        nodes = np.zeros(source.size)
        solution = QssSolution(source, nodes, nodes, nodes, nodes, nodes, 1.0,
                               "Solve_Succeeded", 1, 0.0,
                               np.array([0.0, 2.0, 4.0]), np.array([1.0, 3.0, 5.0]))
        target = np.array([0.0, 3.0, 6.0])
        projected = project_seed(solution, target)
        np.testing.assert_allclose(projected[-4:-2], [0.5, 3.5])
        np.testing.assert_allclose(projected[-2:], [1.5, 4.5])

    def test_project_seed_rejects_malformed_midpoint_controls(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSolution, project_seed

        mesh = np.array([0.0, 2.0, 4.0])
        nodes = np.zeros(mesh.size)
        malformed = (
            np.array([[1.0, 2.0]]),
            np.array([1.0]),
            np.array([1.0, np.nan]),
        )
        for values in malformed:
            solution = QssSolution(mesh, nodes, nodes, nodes, nodes, nodes, 1.0,
                                   "Solve_Succeeded", 1, 0.0, values, np.zeros(2))
            with self.subTest(values=values), self.assertRaisesRegex(ValueError, "finite and match"):
                project_seed(solution, mesh)

    def test_project_seed_rejects_malformed_meshes(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSolution, project_seed

        mesh = np.array([0.0, 2.0, 4.0])
        nodes = np.zeros(mesh.size)
        solution = QssSolution(mesh, nodes, nodes, nodes, nodes, nodes, 1.0,
                               "Solve_Succeeded", 1, 0.0, np.zeros(2), np.zeros(2))
        for target in (np.array([0.0]), np.array([0.0, 2.0, 1.0]), np.array([0.0, np.inf, 4.0])):
            with self.subTest(target=target), self.assertRaisesRegex(ValueError, "target mesh"):
                project_seed(solution, target)
        bad_source = QssSolution(np.array([0.0, np.nan, 4.0]), nodes, nodes, nodes,
                                 nodes, nodes, 1.0, "Solve_Succeeded", 1, 0.0,
                                 np.zeros(2), np.zeros(2))
        with self.assertRaisesRegex(ValueError, "solution distance"):
            project_seed(bad_source, mesh)

    def test_project_seed_legacy_solution_defaults_midpoint_controls_to_zero(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSolution, project_seed

        mesh = np.array([0.0, 2.0, 4.0])
        nodes = np.zeros(mesh.size)
        solution = QssSolution(mesh, nodes, nodes, nodes, nodes, nodes, 1.0,
                               "Solve_Succeeded", 1, 0.0)
        projected = project_seed(solution, mesh)
        np.testing.assert_array_equal(projected[-4:], np.zeros(4))

    def test_empty_spacing_list_is_rejected(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, solve_ladder

        track = load_track(_write_circle_track(self.directory, radius=40.0, nodes=360, half_width=3.0))
        with self.assertRaisesRegex(ValueError, "at least one"):
            solve_ladder(track, corridor_from_track(track), _flat_surface(25.0), QssSettings(), spacings=())
