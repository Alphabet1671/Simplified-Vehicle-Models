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

    def test_open_track_is_rejected_at_periodic_solver_boundary(self):
        from simplified_vehicle_models.solvers.qss_line_ocp import build_mesh, build_ocp, QssSettings
        track=self._track(); mesh=build_mesh(track,4.0); corridor=corridor_from_track(track)
        object.__setattr__(track, "is_closed", False)
        with self.assertRaisesRegex(ValueError, "closed track"):
            build_ocp(track,corridor,_flat_surface(25.0),mesh,QssSettings())

    def test_decision_layout_matches_the_declared_contract(self):
        from simplified_vehicle_models.solvers.qss_line_ocp import build_mesh, build_ocp, QssSettings
        track=self._track(); mesh=build_mesh(track,4.0); corridor=corridor_from_track(track)
        built=build_ocp(track,corridor,_flat_surface(25.0),mesh,QssSettings())
        self.assertEqual(built["nlp"]["x"].shape[0],5*mesh.size+2*(mesh.size-1))

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
