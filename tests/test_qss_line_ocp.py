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
        self.assertEqual(rows.size,4*count)  # midpoint plus Hermite samples at 1/4, 1/2, 3/4

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
