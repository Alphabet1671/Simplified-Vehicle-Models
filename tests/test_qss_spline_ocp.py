"""Tests for the consistent cubic-spline minimum-time OCP."""

from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

import numpy as np

from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface
from simplified_vehicle_models.formulations.signed_capability import SignedTransientCapability
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


def _signed_surface() -> SignedTransientCapability:
    """Safe, nonconstant-domain fixture used to exercise the numeric seed path."""
    u = np.array([0., .5, 1.]); curvature = np.array([-.1, 0., .1])
    return SignedTransientCapability(
        u, curvature, np.full((3, 3), 2.), np.full((3, 3), -6.),
        np.array([12., 18., 12.]), .5, "signed-fixture",
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

    def test_signed_surface_solves_physical_seed(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh
        from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line
        track = self._track(radius=40.0)
        view = replace(track, is_closed=False)
        solution = solve_spline_line(
            view, _zero_corridor(view, is_closed=False), _signed_surface(),
            build_mesh(view, 10.0), OffsetSpline(view, control_spacing=10.0),
            QssSettings(maximum_iterations=400, start_speed=.5),
        )
        self.assertEqual(solution.status, "Solve_Succeeded")
        self.assertLessEqual(solution.maximum_violation, QssSettings().tolerance)

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
        # Global continuous-control and off-site physical protections alter the
        # coarse optimum slightly.  A 0.25 m mesh resolves the launch transient
        # and retains the declared 2e-3 s analytic-envelope parity without
        # changing an objective or acceptance tolerance.
        solution = solve_spline_line(
            view, corridor, surface, build_mesh(view, 0.25),
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
            # OCP uses the shared interpolated XY reference; this coarse source
            # circle has finite reference-fit curvature error, not an exact
            # stored-curvature identity.
            solution.curvature, np.full(solution.curvature.size, 1.0 / 40.0), atol=1.0e-6,
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

    def test_invalid_off_collocation_fractions_are_rejected(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp

        track = self._track()
        corridor = _zero_corridor(track, is_closed=True)
        mesh = build_mesh(track, 10.0)
        spline = OffsetSpline(track, control_spacing=20.0)
        for fractions in (("quarter",), (0.25, 0.25), (0.5,), (), [0.25]):
            with self.subTest(fractions=fractions):
                with self.assertRaisesRegex(ValueError, "off_collocation_fractions"):
                    build_spline_ocp(
                        track, corridor, _surface(), mesh, spline,
                        QssSettings(off_collocation_fractions=fractions),
                    )

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

    def test_zero_corridor_detection_covers_midpoints_and_quarters(self) -> None:
        from simplified_vehicle_models.solvers.qss_spline_ocp import (
            _is_zero_corridor,
        )

        zeros = np.zeros(5)
        self.assertTrue(_is_zero_corridor(zeros, zeros, zeros, zeros, zeros, zeros))
        bump = np.zeros(5)
        bump[2] = -0.25
        self.assertFalse(_is_zero_corridor(zeros, zeros, zeros, zeros, bump, zeros))
        self.assertFalse(_is_zero_corridor(zeros, zeros, zeros, bump, zeros, zeros))
        self.assertFalse(_is_zero_corridor(zeros, bump, zeros, zeros, zeros, zeros))

    def test_nonzero_corridor_between_nodes_keeps_the_free_path(self) -> None:
        from simplified_vehicle_models.solvers.qss_line_ocp import (
            QssSettings, build_mesh, track_sample,
        )
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp

        track = self._track()
        base = corridor_from_track(track)
        mesh = build_mesh(track, 10.0)
        midpoints = 0.5 * (mesh[:-1] + mesh[1:])
        chosen = None
        for index in range(track.distance.size):
            minimum = np.zeros_like(base.minimum)
            maximum = np.zeros_like(base.maximum)
            minimum[index] = -0.5
            maximum[index] = 0.5
            node_lower = track_sample(
                minimum, base.distance, mesh, closed=True,
                period=float(track.lap_length),
            )
            node_upper = track_sample(
                maximum, base.distance, mesh, closed=True,
                period=float(track.lap_length),
            )
            mid_lower = track_sample(
                minimum, base.distance, midpoints, closed=True,
                period=float(track.lap_length),
            )
            if (np.all(node_lower == 0.0) and np.all(node_upper == 0.0)
                    and np.any(mid_lower != 0.0)):
                chosen = (minimum, maximum)
                break
        self.assertIsNotNone(
            chosen, "fixture must be zero at mesh nodes but nonzero at a midpoint",
        )
        corridor = Corridor(
            base.distance, chosen[0], chosen[1], base.provenance, True,
        )
        built = build_spline_ocp(
            track, corridor, _surface(), mesh,
            OffsetSpline(track, control_spacing=20.0), QssSettings(),
        )
        controls = built["control_count"]
        self.assertTrue(np.any(built["x_lower"][:controls] < 0.0))
        self.assertTrue(np.any(built["x_upper"][:controls] > 0.0))

    def test_bernstein_middle_coefficient_boxes_quadratic_control_globally(self) -> None:
        """The old node/midpoint-only [3, 3, -.6]/am=3 peak is rejected."""
        import casadi as ca

        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp

        track = replace(self._track(), is_closed=False)
        mesh = np.array([0.0, 0.5 * track.lap_length, track.lap_length])
        built = build_spline_ocp(
            track, _zero_corridor(track, is_closed=False), _surface(), mesh,
            OffsetSpline(track, control_spacing=20.0), QssSettings(start_speed=10.0),
        )
        controls, nodes = built["control_count"], mesh.size
        values = np.zeros(controls + 3 * nodes - 1)
        values[controls:controls + nodes] = 10.0
        values[controls + nodes:controls + 2 * nodes] = [3.0, 3.0, -0.6]
        values[controls + 2 * nodes:] = [3.0, 3.0]
        constraints = np.asarray(ca.Function(
            "bernstein_constraints", [built["nlp"]["x"]], [built["nlp"]["g"]],
        )(values), dtype=float).ravel()
        # Last-element b1=2*3-.5*(3-.6)=4.8.  It used to have no row;
        # now a global amax=3 bound applies independent of surface variation.
        matching = np.isclose(constraints, 4.8)
        self.assertTrue(np.any(matching))
        self.assertTrue(np.any(built["g_upper"][matching] == 3.0))

    def test_capability_rows_reserve_strict_interior_nlp_margin(self) -> None:
        """Chart faces retain an eight-residual interpolation budget."""
        import casadi as ca

        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp

        track = replace(self._track(), is_closed=False)
        mesh = np.array([0.0, track.lap_length])
        settings = QssSettings(start_speed=10.0, tolerance=2.5e-7)
        built = build_spline_ocp(
            track, _zero_corridor(track, is_closed=False), _surface(), mesh,
            OffsetSpline(track, control_spacing=20.0), settings,
        )
        controls, nodes = built["control_count"], mesh.size
        values = np.zeros(controls + 3 * nodes - 1)
        values[controls:controls + nodes] = 10.0
        values[controls + nodes:controls + 2 * nodes] = 3.0
        values[controls + 2 * nodes:] = 3.0
        g = np.asarray(ca.Function("capability_margin", [built["nlp"]["x"]],
                                   [built["nlp"]["g"]])(values), dtype=float).ravel()
        upper_indices = np.r_[built["node_acceleration_upper_indices"],
                              built["midpoint_acceleration_upper_indices"],
                              built["off_site_acceleration_upper_indices"]]
        np.testing.assert_allclose(g[upper_indices], 0.0, atol=1.0e-12)
        np.testing.assert_allclose(
            built["g_lower"][upper_indices],
            np.full(upper_indices.size, 8.0 * settings.tolerance),
        )
        # Repeat at the real lower chart face to cover all stored lower rows.
        values[controls + nodes:controls + 2 * nodes] = -8.0
        values[controls + 2 * nodes:] = -8.0
        g = np.asarray(ca.Function("capability_lower_margin", [built["nlp"]["x"]],
                                   [built["nlp"]["g"]])(values), dtype=float).ravel()
        lower_indices = np.r_[built["node_acceleration_lower_indices"],
                              built["midpoint_acceleration_lower_indices"],
                              built["off_site_acceleration_lower_indices"]]
        np.testing.assert_allclose(g[lower_indices], 0.0, atol=1.0e-12)
        np.testing.assert_allclose(
            built["g_lower"][lower_indices],
            np.full(lower_indices.size, 8.0 * settings.tolerance),
        )

    def test_capability_margin_rejects_narrow_chart_range(self) -> None:
        """Never silently make a chart whose band cannot fit both margins."""
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp

        track = replace(self._track(), is_closed=False)
        surface = CapabilitySurface(
            np.array([.5, 30.]), np.array([-.3, .3]),
            np.full((2, 2), 1.0), np.full((2, 2), 1.0 - 15.0e-6),
            np.full(2, 30.0), "narrow-band",
        )
        with self.assertRaisesRegex(ValueError, "acceleration range"):
            build_spline_ocp(
                track, _zero_corridor(track, is_closed=False), surface,
                build_mesh(track, 10.0), OffsetSpline(track, control_spacing=20.0),
                QssSettings(start_speed=10.0),
            )

    def test_off_collocation_sites_add_physical_rows_without_extra_defects(self) -> None:
        """Eighth/quarter rows constrain capability, not more HS equalities."""
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp

        track = replace(self._track(), is_closed=False)
        mesh = np.array([0.0, 0.5 * track.lap_length, track.lap_length])
        base = build_spline_ocp(
            track, _zero_corridor(track, is_closed=False), _surface(), mesh,
            OffsetSpline(track, control_spacing=20.0),
            QssSettings(start_speed=10.0, off_collocation_fractions=(0.25,)),
        )
        protected = build_spline_ocp(
            track, _zero_corridor(track, is_closed=False), _surface(), mesh,
            OffsetSpline(track, control_spacing=20.0), QssSettings(start_speed=10.0),
        )
        # Four default interior sites replace the old two quarters: their
        # additional rows are inequalities only, so the sole HS defect per
        # element remains the only new equality family.
        self.assertGreater(protected["g_lower"].size, base["g_lower"].size)
        self.assertEqual(
            int(np.count_nonzero(protected["g_lower"] == protected["g_upper"])),
            int(np.count_nonzero(base["g_lower"] == base["g_upper"])),
        )
        # The added rows remain physical inequalities; no off-site nonlinear
        # dynamics equality is introduced into the HS transcription.
        self.assertTrue(np.all(
            protected["g_lower"][protected["off_site_acceleration_upper_indices"]]
            < protected["g_upper"][protected["off_site_acceleration_upper_indices"]]
        ))

    def test_quarter_capability_row_rejects_varying_surface_violation(self) -> None:
        """A physical amax notch missed by nodes/midpoint is constrained at 1/4."""
        import casadi as ca

        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp

        from types import SimpleNamespace
        from simplified_vehicle_models.tracks.types import PathCoordinates

        distance = np.linspace(0.0, 20.0, 81)
        track = SimpleNamespace(
            distance=distance, curvature=np.zeros_like(distance),
            path_coordinates=PathCoordinates(distance, distance, np.zeros_like(distance),
                                             np.zeros_like(distance), np.zeros_like(distance), False),
            is_closed=False, lap_length=20.0,
        )
        mesh = np.array([0.0, 10.0, 20.0])
        # Energy-HS gives v(.25)=10.4358 while v(.5)=10.5119. The narrow
        # table notch leaves all node and midpoint capability rows legal but
        # makes only the quarter candidate fail (the global Bernstein a=3 box
        # remains legal).
        surface = CapabilitySurface(
            np.array([.5, 10., 10.42, 10.43, 10.45, 10.46, 11., 12., 30.]),
            np.array([-.3, .3]),
            np.array([[3., 3.], [3., 3.], [3., 3.], [2., 2.], [2., 2.],
                      [3., 3.], [3., 3.], [3., 3.], [3., 3.]]),
            np.full((9, 2), -8.), np.full(2, 30.), "varying-quarter",
        )
        corridor = Corridor(distance, np.zeros_like(distance), np.zeros_like(distance),
                            "straight", False)
        midpoint_only = build_spline_ocp(
            track, corridor, surface, mesh,
            OffsetSpline(track, control_spacing=20.0),
            # Eighth sites are deliberately away from the quarter notch.
            QssSettings(start_speed=10.0, off_collocation_fractions=(0.125, 0.875)),
        )
        built = build_spline_ocp(
            track, corridor, surface, mesh,
            OffsetSpline(track, control_spacing=20.0),
            QssSettings(start_speed=10.0, off_collocation_fractions=(0.25,)),
        )
        controls, nodes = built["control_count"], mesh.size
        values = np.zeros(controls + 3 * nodes - 1)
        values[controls:controls + nodes] = [10.0, 11.0, 12.0]
        values[controls + nodes:controls + 2 * nodes] = 3.0
        values[controls + 2 * nodes:] = 3.0
        evaluate_midpoint_only = ca.Function(
            "midpoint_only_capability_constraints", [midpoint_only["nlp"]["x"]],
            [midpoint_only["nlp"]["g"]],
        )
        midpoint_only_constraints = np.asarray(
            evaluate_midpoint_only(values), dtype=float,
        ).ravel()
        evaluate_quarter = ca.Function(
            "quarter_capability_constraints", [built["nlp"]["x"]], [built["nlp"]["g"]],
        )
        constraints = np.asarray(evaluate_quarter(values), dtype=float).ravel()
        # All acceleration-capability rows at nodes, midpoints, and the paired
        # non-quarter sites pass. (The deliberately hand-built candidate need
        # not satisfy the separate HS defect equality.) At x=.25, energy-HS
        # gives v=10.43582..., a=3, and the table gives amax=2, so the stored physical
        # upper-capability row for element 0 / fraction .25 is the only
        # demonstrated acceleration-capability failure.
        self.assertTrue(np.all(
            midpoint_only_constraints[
                midpoint_only["node_acceleration_upper_indices"]
            ] >= -1.e-9,
        ))
        self.assertTrue(np.all(
            midpoint_only_constraints[
                midpoint_only["midpoint_acceleration_upper_indices"]
            ] >= -1.e-9,
        ))
        self.assertTrue(np.all(
            midpoint_only_constraints[
                midpoint_only["off_site_acceleration_upper_indices"]
            ] >= -1.e-9,
        ))
        self.assertTrue(np.all(
            constraints[built["node_acceleration_upper_indices"]] >= -1.e-9,
        ))
        self.assertTrue(np.all(
            constraints[built["midpoint_acceleration_upper_indices"]] >= -1.e-9,
        ))
        bernstein_indices = built["bernstein_middle_indices"]
        self.assertEqual(bernstein_indices.size, mesh.size - 1)
        np.testing.assert_allclose(constraints[bernstein_indices], 3.0, atol=1.e-12)
        self.assertTrue(np.all(
            constraints[bernstein_indices] >= built["g_lower"][bernstein_indices],
        ))
        self.assertTrue(np.all(
            constraints[bernstein_indices] <= built["g_upper"][bernstein_indices],
        ))
        quarter_indices = built["off_site_acceleration_upper_indices"]
        self.assertEqual(quarter_indices.size, mesh.size - 1)
        self.assertLess(constraints[quarter_indices[0]], -0.9)
        self.assertGreaterEqual(constraints[quarter_indices[1]], -1.e-9)

    def test_midpoint_speed_has_a_minimum_speed_lower_bound(self) -> None:
        import casadi as ca

        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp

        track = self._track()
        corridor = _zero_corridor(track, is_closed=True)
        mesh = build_mesh(track, 5.0)
        settings = QssSettings()
        built = build_spline_ocp(
            track, corridor, _surface(), mesh,
            OffsetSpline(track, control_spacing=20.0), settings,
        )
        nodes = mesh.size
        count = built["count"]
        controls = built["control_count"]
        values = np.zeros(controls + 3 * nodes - 1)
        values[controls:controls + nodes] = settings.minimum_speed
        # Alternate node accelerations so the Hermite midpoint speed is driven
        # far below the node floor while every node remains within its limits.
        values[controls + nodes:controls + 2 * nodes] = np.where(
            np.arange(nodes) % 2 == 0, -8.0, 3.0,
        )
        evaluate = ca.Function("g", [built["nlp"]["x"]], [built["nlp"]["g"]])
        g = np.asarray(evaluate(values), dtype=float).ravel()
        lower_only = (
            (built["g_lower"] == 0.0) & (built["g_upper"] == np.inf)
        )
        self.assertTrue(np.any(lower_only))
        self.assertLess(
            float(np.min(g[lower_only])), -1.0,
        )

    def test_energy_hs_is_exact_for_straight_quadratic_acceleration(self) -> None:
        """W=v² is cubic exactly when a(s) is quadratic on a straight."""
        from types import SimpleNamespace
        from simplified_vehicle_models.solvers.qss_spline_ocp import _SplineStateEvaluator
        from simplified_vehicle_models.tracks.types import PathCoordinates

        distance = np.linspace(0.0, 2.0, 5)
        track = SimpleNamespace(
            distance=distance, curvature=np.zeros(5), lap_length=2.0,
            path_coordinates=PathCoordinates(distance, distance, np.zeros(5),
                                             np.zeros(5), np.zeros(5), False),
            is_closed=False,
        )
        spline = OffsetSpline(track, control_spacing=2.0)
        from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory, ReferenceCurve
        trajectory = OffsetTrajectory(ReferenceCurve(track), spline, np.zeros(spline.control_count))
        # a(s)=1 + 2s - .5s², hence W(s)=100+2s+2s²-s³/3.
        stations = np.linspace(0.0, 2.0, 29)
        acceleration = np.array([1.0, 3.0])
        midpoint_acceleration = np.array([2.5])
        energy = lambda s: 100.0 + 2.0*s + 2.0*s**2 - s**3 / 3.0
        mesh = np.array([0.0, 2.0])
        evaluator = _SplineStateEvaluator(
            trajectory, mesh, np.sqrt(energy(mesh)), acceleration,
            midpoint_acceleration,
        )
        dense = evaluator.evaluate(stations)
        expected_a = 1.0 + 2.0*stations - .5*stations**2
        np.testing.assert_allclose(dense["energy"], energy(stations), atol=2.e-12)
        np.testing.assert_allclose(dense["acceleration"], expected_a, atol=2.e-12)
        np.testing.assert_allclose(
            dense["speed_rate"], expected_a / np.sqrt(energy(stations)), atol=3.e-12,
        )
        # ∫ ds/v has the analytic constant-acceleration special case below;
        # here independently verify high-order physical quadrature convergence.
        coarse = np.trapezoid(1.0 / np.sqrt(energy(np.linspace(0., 2., 1001))),
                              np.linspace(0., 2., 1001))
        fine = np.trapezoid(1.0 / np.sqrt(energy(np.linspace(0., 2., 2001))),
                            np.linspace(0., 2., 2001))
        self.assertLess(abs(fine - coarse), 1.e-7)

    def test_energy_hs_variable_metric_residual_converges(self) -> None:
        """Nonlinear metric remains an HS approximation whose dense residual falls."""
        from types import SimpleNamespace
        from simplified_vehicle_models.solvers.qss_spline_ocp import _SplineStateEvaluator

        class VariableMetricTrajectory:
            def evaluate(self, station):
                s = np.asarray(station, dtype=float)
                return SimpleNamespace(metric=1.0 + .05 * s**3)

        trajectory = VariableMetricTrajectory()
        # a=1 and G=1+.05s³ give W=100+2s+.025s⁴, deliberately not cubic.
        energy = lambda s: 100.0 + 2.0*s + .025*s**4
        residuals = []
        for intervals in (4, 8, 16):
            mesh = np.linspace(0.0, 2.0, intervals + 1)
            evaluator = _SplineStateEvaluator(
                trajectory, mesh, np.sqrt(energy(mesh)), np.ones(mesh.size),
                np.ones(intervals),
            )
            stations = np.linspace(0.0, 2.0, 16 * intervals + 1)[1:-1]
            state = evaluator.evaluate(stations)
            residuals.append(float(np.max(np.abs(
                state["speed_rate"] - state["rhs_speed_rate"],
            ))))
        self.assertLess(residuals[1], residuals[0] / 2.0)
        self.assertLess(residuals[2], residuals[1] / 2.0)

    def test_energy_bernstein_floor_rejects_interior_braking_dip(self) -> None:
        """The energy cubic's Bernstein coefficients protect all x in [0, 1]."""
        import casadi as ca
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp

        track = replace(self._track(), is_closed=False)
        mesh = np.array([0.0, track.lap_length])
        settings = QssSettings(start_speed=2.0, minimum_speed=0.5)
        built = build_spline_ocp(
            track, _zero_corridor(track, is_closed=False), _surface(), mesh,
            OffsetSpline(track, control_spacing=20.0), settings,
        )
        controls, nodes = built["control_count"], mesh.size
        values = np.zeros(controls + 3 * nodes - 1)
        values[controls:controls + nodes] = [2.0, 2.0]
        # Endpoint W=4, but W's first Bernstein interior coefficient is deeply
        # negative under a long braking slope; the explicit floor must reject it.
        values[controls + nodes:controls + 2 * nodes] = [-8.0, 3.0]
        values[controls + 2 * nodes:] = 0.0
        constraints = np.asarray(ca.Function(
            "energy_floor_constraints", [built["nlp"]["x"]], [built["nlp"]["g"]],
        )(values), dtype=float).ravel()
        lower_only = (built["g_lower"] == 0.0) & (built["g_upper"] == np.inf)
        self.assertLess(float(np.min(constraints[lower_only])), -1.0)

    def test_open_straight_acceleration_is_dense_bounded_and_time_is_analytic(self) -> None:
        """Task-5 20 m launch regression: no terminal control overshoot."""
        from types import SimpleNamespace
        from simplified_vehicle_models.solvers.line_validation import validate_line
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings
        from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line
        from simplified_vehicle_models.tracks.types import PathCoordinates

        distance = np.linspace(0.0, 20.0, 81)
        track = SimpleNamespace(
            distance=distance, curvature=np.zeros_like(distance),
            path_coordinates=PathCoordinates(distance, distance, np.zeros_like(distance),
                                             np.zeros_like(distance), np.zeros_like(distance), False),
            is_closed=False, lap_length=20.0,
        )
        corridor = Corridor(distance, np.zeros_like(distance), np.zeros_like(distance),
                            "straight", False)
        surface = CapabilitySurface(
            np.array([.5, 15., 30.]), np.array([-.1, 0., .1]),
            np.full((3, 3), 3.), np.full((3, 3), -8.), np.full(3, 30.), "straight",
        )
        mesh = np.linspace(0., 20., 17)
        settings = QssSettings(start_speed=10., maximum_iterations=2000)
        solution = solve_spline_line(track, corridor, surface, mesh,
                                     OffsetSpline(track, control_spacing=5.), settings)
        self.assertEqual(solution.status, "Solve_Succeeded")
        stations = np.linspace(0., 20., 257)
        dense = solution.state_evaluator.evaluate(stations)
        self.assertLessEqual(float(np.max(dense["acceleration"])), 3.0 + 2.e-6)
        self.assertGreaterEqual(float(np.min(dense["acceleration"])), -8.0 - 2.e-6)
        analytic = (np.sqrt(10.0**2 + 2.0 * 3.0 * 20.0) - 10.0) / 3.0
        self.assertAlmostEqual(solution.lap_time, analytic, delta=2.0e-3)
        report = validate_line(solution, corridor, surface, samples_per_element=8)
        peak = next(item for item in report["diagnostics"] if item["name"] == "acceleration_max")
        self.assertTrue(peak["passed"])

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
        # The status above already implies the solver's own violation bound;
        # this test checks the represented line, so assert the corridor is
        # non-degenerate and sample it densely between nodes.
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
