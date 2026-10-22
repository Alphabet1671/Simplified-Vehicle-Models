"""Tests for the consistent cubic-spline minimum-time OCP."""

from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

import numpy as np

from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface
from simplified_vehicle_models.solvers.qss_line_ocp import track_sample
from simplified_vehicle_models.tracks.corridor import Corridor, corridor_from_track
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.tracks.profile import load_track
from tests.test_track_width import _write_circle_track


def _has_casadi() -> bool:
    try:
        import casadi  # noqa: F401
    except ImportError:
        return False
    return True


def _has_scipy() -> bool:
    try:
        import scipy.interpolate  # noqa: F401
    except ImportError:
        return False
    return True


CASADI = _has_casadi()
SCIPY = _has_scipy()


def _surface() -> CapabilitySurface:
    speed = np.linspace(0.5, 40.0, 81)
    curvature = np.linspace(-0.3, 0.3, 61)
    return CapabilitySurface(
        speed, curvature, np.full((81, 61), 3.0), np.full((81, 61), -8.0),
        np.full(61, 25.0), "synthetic",
    )


def _zero_corridor(track, *, is_closed: bool) -> Corridor:
    base = corridor_from_track(track)
    return Corridor(
        base.distance, np.zeros_like(base.minimum), np.zeros_like(base.maximum),
        base.provenance, is_closed,
    )


def _track_corridor(track):
    """Use the written 3 m edges, or assume them for an edgeless track."""
    if getattr(track, "has_edges", False):
        return corridor_from_track(track)
    return corridor_from_track(track, half_width=3.0)


@unittest.skipUnless(SCIPY, "SciPy is required to evaluate offset splines")
class SplineGeometryTest(unittest.TestCase):
    def test_zero_offset_reproduces_the_centreline_geometry(self) -> None:
        from simplified_vehicle_models.solvers.qss_spline_ocp import (
            _curvature_pair, _geometry,
        )

        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(
                Path(directory), radius=40.0, nodes=72, half_width=3.0,
            ))
            spline = OffsetSpline(track, control_spacing=10.0)
            coefficients = np.zeros(spline.control_count)
            positions = np.array([0.0, 17.0, 90.0, 251.0])
            offset, heading, rate, curvature, den = _geometry(
                track, spline, coefficients, positions,
            )
            centreline, rate_track = _curvature_pair(track, positions)
            np.testing.assert_allclose(offset, np.zeros(positions.size), atol=1.0e-12)
            np.testing.assert_allclose(heading, np.zeros(positions.size), atol=1.0e-12)
            np.testing.assert_allclose(rate, np.zeros(positions.size), atol=1.0e-12)
            np.testing.assert_allclose(den, np.ones(positions.size), atol=1.0e-12)
            np.testing.assert_allclose(curvature, centreline, atol=1.0e-12)
            np.testing.assert_allclose(
                centreline, np.full(positions.size, 1.0 / 40.0), atol=1.0e-12,
            )
            np.testing.assert_allclose(rate_track, np.zeros(positions.size), atol=1.0e-9)


@unittest.skipUnless(CASADI, "CasADi is not installed")
class SplineTranscriptionTest(unittest.TestCase):
    def setUp(self) -> None:
        self._directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._directory.name)

    def tearDown(self) -> None:
        self._directory.cleanup()

    def _track(self, **kwargs):
        defaults = {"radius": 40.0, "nodes": 72, "half_width": 3.0}
        defaults.update(kwargs)
        return load_track(_write_circle_track(self.directory, **defaults))

    def test_zero_corridor_open_circle_matches_capability_envelope(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import (
            QssSettings, build_mesh, capability_envelope_time,
        )
        from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line

        track = self._track()
        view = replace(track, is_closed=False)
        corridor = _zero_corridor(view, is_closed=False)
        settings = QssSettings(maximum_iterations=2000, start_speed=0.5)
        surface = _surface()
        # The 2 m mesh in the brief leaves a 0.019 s launch-transient
        # discretization error. A 0.5 m mesh resolves the standing start and
        # exercises the same transcription at the requested 2e-3 s parity.
        solution = solve_spline_line(
            view, corridor, surface, build_mesh(view, 0.5),
            OffsetSpline(view, control_spacing=10.0), settings,
        )
        self.assertEqual(solution.status, "Solve_Succeeded")
        self.assertLessEqual(solution.maximum_violation, settings.tolerance)
        expected = capability_envelope_time(view, surface, settings)
        self.assertAlmostEqual(solution.lap_time, expected, delta=2.0e-3)
        np.testing.assert_allclose(solution.offset, np.zeros(solution.offset.size), atol=1.0e-9)
        np.testing.assert_allclose(
            solution.heading_error, np.zeros(solution.heading_error.size), atol=1.0e-9,
        )
        np.testing.assert_allclose(
            solution.curvature, np.full(solution.curvature.size, 1.0 / 40.0), atol=1.0e-9,
        )

    def test_closed_zero_corridor_matches_per_node_solver(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import (
            QssSettings, build_mesh, solve_line,
        )
        from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line

        track = self._track()
        corridor = _zero_corridor(track, is_closed=True)
        mesh = build_mesh(track, 2.0)
        settings = QssSettings(maximum_iterations=2000)
        surface = _surface()
        reference = solve_line(track, corridor, surface, mesh, settings)
        solution = solve_spline_line(
            track, corridor, surface, mesh,
            OffsetSpline(track, control_spacing=10.0), settings,
        )
        self.assertEqual(reference.status, "Solve_Succeeded")
        self.assertEqual(solution.status, "Solve_Succeeded")
        self.assertAlmostEqual(solution.lap_time, reference.lap_time, delta=2.0e-3)
        self.assertTrue(np.allclose(solution.speed, solution.speed[0], atol=1.0e-3))
        self.assertTrue(np.all(solution.speed <= 25.0 + 1.0e-6))

    def test_open_and_closed_use_the_same_state_layout(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh
        from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line

        track = self._track()
        for closed in (True, False):
            with self.subTest(closed=closed):
                view = track if closed else replace(track, is_closed=False)
                corridor = _track_corridor(view)
                spline = OffsetSpline(view, control_spacing=10.0)
                mesh = build_mesh(view, 3.0)
                solution = solve_spline_line(
                    view, corridor, _surface(), mesh,
                    spline, QssSettings(maximum_iterations=2000),
                )
                self.assertEqual(solution.status, "Solve_Succeeded")
                self.assertEqual(solution.distance.size, mesh.size)
                self.assertEqual(solution.speed.size, solution.distance.size)
                self.assertEqual(solution.acceleration.size, solution.distance.size)
                self.assertEqual(solution.coefficients.size, spline.control_count)
                self.assertTrue(np.all(np.isfinite(solution.speed)))
                self.assertTrue(np.all(solution.speed >= 0.5 - 1.0e-6))
                self.assertTrue(np.all(solution.offset <= 3.0 + 1.0e-6))
                self.assertTrue(np.all(solution.offset >= -3.0 - 1.0e-6))
                if closed:
                    self.assertAlmostEqual(
                        float(solution.speed[0]), float(solution.speed[-1]), places=6,
                    )
                else:
                    self.assertAlmostEqual(float(solution.speed[0]), 0.5, places=6)

    def test_speed_cap_limits_the_closed_solution(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh
        from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line

        track = self._track()
        corridor = _zero_corridor(track, is_closed=True)
        mesh = build_mesh(track, 5.0)
        cap = np.full(mesh.size, 15.0)
        settings = QssSettings(maximum_iterations=2000)
        solution = solve_spline_line(
            track, corridor, _surface(), mesh,
            OffsetSpline(track, control_spacing=10.0), settings, speed_cap=cap,
        )
        self.assertEqual(solution.status, "Solve_Succeeded")
        self.assertLessEqual(float(np.max(solution.speed)), 15.0 + 1.0e-6)
        self.assertAlmostEqual(
            solution.lap_time, float(track.lap_length) / 15.0, delta=5.0e-3,
        )

    def test_closed_corridor_offset_shortens_the_path(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh
        from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line

        for direction in (1.0, -1.0):
            with self.subTest(direction=direction):
                track = self._track(direction=direction)
                corridor = corridor_from_track(track)
                mesh = build_mesh(track, 3.0)
                solution = solve_spline_line(
                    track, corridor, _surface(), mesh,
                    OffsetSpline(track, control_spacing=10.0),
                    QssSettings(maximum_iterations=2000),
                )
                self.assertEqual(solution.status, "Solve_Succeeded")
                # The inside edge is 3 m from the centreline, so the line runs
                # on a 37 m radius at the 25 m/s ceiling: 232.478 m / 25 m/s.
                self.assertAlmostEqual(solution.lap_time, 9.299, delta=5.0e-3)
                inside = 3.0 * direction
                np.testing.assert_allclose(
                    solution.offset,
                    np.full(solution.offset.size, inside), atol=1.0e-2,
                )
                np.testing.assert_allclose(
                    solution.curvature,
                    np.full(solution.curvature.size, direction / 37.0), atol=1.0e-3,
                )
                np.testing.assert_allclose(
                    solution.heading_error,
                    np.zeros(solution.heading_error.size), atol=1.0e-3,
                )
                self.assertLess(
                    solution.lap_time, float(track.lap_length) / 25.0,
                )

    def test_excessive_heading_and_curvature_rate_are_bounded(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh
        from simplified_vehicle_models.solvers.qss_spline_ocp import (
            _geometry, solve_spline_line,
        )

        track = self._track()
        base = corridor_from_track(track)
        minimum = np.array(base.minimum, copy=True)
        maximum = np.array(base.maximum, copy=True)
        # A zero-width pinch forces the spline away from the constant inside
        # edge; the free solution then needs a visible lateral slew.
        minimum[15:25] = 0.0
        maximum[15:25] = 0.0
        corridor = Corridor(base.distance, minimum, maximum, base.provenance, True)
        mesh = build_mesh(track, 3.0)
        spline = OffsetSpline(track, control_spacing=10.0)
        surface = _surface()

        free = solve_spline_line(
            track, corridor, surface, mesh, spline,
            QssSettings(maximum_iterations=2000),
        )
        self.assertEqual(free.status, "Solve_Succeeded")
        _, free_heading, free_rate, _, _ = _geometry(
            track, spline, free.coefficients, mesh,
        )
        self.assertGreater(float(np.max(np.abs(free_heading))), 1.0e-2)
        self.assertGreater(float(np.max(np.abs(free_rate))), 1.0e-3)

        bound = 1.0e-6
        tight = QssSettings(
            maximum_iterations=3000,
            maximum_heading=bound,
            maximum_curvature_rate=bound,
        )
        bounded = solve_spline_line(track, corridor, surface, mesh, spline, tight)
        self.assertEqual(bounded.status, "Solve_Succeeded")
        _, heading, rate, _, _ = _geometry(track, spline, bounded.coefficients, mesh)
        self.assertLessEqual(
            float(np.max(np.abs(heading))), bound + tight.tolerance,
        )
        self.assertLessEqual(
            float(np.max(np.abs(rate))), bound + tight.tolerance,
        )
        # The heading limit leaves no room to leave the centreline, so the
        # bounded solve collapses to the zero-offset line instead of diverging.
        self.assertLess(float(np.max(np.abs(bounded.offset))), 1.0e-2)

    def test_decision_layout_and_scaling_match_the_declared_contract(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh
        from simplified_vehicle_models.solvers.qss_spline_ocp import (
            _decision_scaling, build_spline_ocp, unpack,
        )

        track = self._track()
        corridor = _zero_corridor(track, is_closed=True)
        mesh = build_mesh(track, 10.0)
        built = build_spline_ocp(
            track, corridor, _surface(), mesh,
            OffsetSpline(track, control_spacing=20.0), QssSettings(),
        )
        count = built["count"]
        nodes = mesh.size
        controls = built["control_count"]
        self.assertEqual(int(built["nlp"]["x"].numel()), controls + 3 * nodes - 1)
        values = np.arange(float(controls + 3 * nodes - 1))
        blocks = unpack(values, count, controls)
        np.testing.assert_array_equal(blocks["coefficients"], values[:controls])
        np.testing.assert_array_equal(
            blocks["speed"], values[controls:controls + nodes],
        )
        np.testing.assert_array_equal(
            blocks["acceleration"], values[controls + nodes:controls + 2 * nodes],
        )
        np.testing.assert_array_equal(
            blocks["mid_acceleration"], values[controls + 2 * nodes:],
        )
        center, scale = _decision_scaling(controls, nodes)
        self.assertEqual(center.size, values.size)
        np.testing.assert_array_equal(center[:controls], np.zeros(controls))
        np.testing.assert_array_equal(
            center[controls:controls + nodes], np.full(nodes, 20.0),
        )
        np.testing.assert_array_equal(
            scale[controls:controls + nodes], np.full(nodes, 10.0),
        )
        np.testing.assert_array_equal(
            scale[controls + nodes:], np.full(values.size - controls - nodes, 5.0),
        )
        with self.assertRaisesRegex(ValueError, "layout needs"):
            unpack(np.zeros(values.size - 1), count, controls)

    def test_invalid_speed_cap_and_spline_are_rejected(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp

        track = self._track()
        corridor = _zero_corridor(track, is_closed=True)
        mesh = build_mesh(track, 10.0)
        spline = OffsetSpline(track, control_spacing=20.0)
        with self.assertRaisesRegex(ValueError, "speed_cap"):
            build_spline_ocp(
                track, corridor, _surface(), mesh, spline, QssSettings(),
                speed_cap=np.ones(mesh.size + 1),
            )
        with self.assertRaisesRegex(ValueError, "speed_cap"):
            build_spline_ocp(
                track, corridor, _surface(), mesh, spline, QssSettings(),
                speed_cap=np.zeros(mesh.size),
            )
        with self.assertRaisesRegex(ValueError, "closure"):
            open_view = replace(track, is_closed=False)
            build_spline_ocp(
                track, corridor, _surface(), mesh,
                OffsetSpline(open_view, control_spacing=20.0), QssSettings(),
            )

    def test_spline_solution_respects_dense_corridor(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh
        from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line

        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40.0, nodes=72,
                                                   half_width=3.0))
            base = corridor_from_track(track)
            corridor = Corridor(base.distance, base.minimum, base.maximum, base.provenance, True)
            mesh = build_mesh(track, 3.0)
            spline = OffsetSpline(track, control_spacing=10.0)
            solution = solve_spline_line(track, corridor, _surface(), mesh, spline,
                                         QssSettings(maximum_iterations=2000))
        self.assertEqual(solution.status, 'Solve_Succeeded')
        self.assertLessEqual(solution.maximum_violation, 1.0e-6)
        self.assertTrue(np.any(base.maximum > base.minimum))
        positions = np.linspace(0.0, track.lap_length, 25 * (mesh.size - 1) + 1)
        self.assertEqual(positions.size, 25 * (mesh.size - 1) + 1)
        offset = spline.evaluate(solution.coefficients, positions)[0]
        lower = track_sample(corridor.minimum, corridor.distance, positions,
                             closed=track.is_closed, period=track.lap_length)
        upper = track_sample(corridor.maximum, corridor.distance, positions,
                             closed=track.is_closed, period=track.lap_length)
        self.assertTrue(np.all(offset >= lower - 1e-6))
        self.assertTrue(np.all(offset <= upper + 1e-6))


if __name__ == "__main__":
    unittest.main()
