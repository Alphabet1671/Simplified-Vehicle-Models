"""Controlled topology/length views for real-track QSS diagnostics."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.tracks.profile import load_track
from tools.diagnostics.compare_qss_track_boundaries import track_view, make_cases

ROOT = Path(__file__).resolve().parents[1]


class TrackBoundaryViewsTest(unittest.TestCase):
    def test_real_tracks_cut_open_without_changing_geometry(self):
        for name, relative in (
            ('MIS', 'models/tracks/2024_michigan_endurance_closed_edged.xml'),
            ('FSC', 'tracks_out/fsc_may_2026.xml'),
        ):
            with self.subTest(track=name):
                source = ROOT / relative
                original_bytes = source.read_bytes()
                original = load_track(source)
                opened = track_view(original, is_closed=False)
                self.assertTrue(original.is_closed)
                self.assertTrue(original.path_coordinates.is_closed)
                self.assertFalse(opened.is_closed)
                self.assertFalse(opened.path_coordinates.is_closed)
                self.assertEqual(opened.lap_length, original.lap_length)
                for field in ('distance', 'segment_length', 'radius', 'curvature'):
                    np.testing.assert_array_equal(getattr(opened, field), getattr(original, field))
                for field in ('distance', 'x_north', 'y_east', 'heading', 'curvature'):
                    np.testing.assert_array_equal(getattr(opened.path_coordinates, field),
                                                  getattr(original.path_coordinates, field))
                self.assertEqual(opened.width_provenance, original.width_provenance)
                if original.has_edges:
                    np.testing.assert_array_equal(opened.half_width_left, original.half_width_left)
                    np.testing.assert_array_equal(opened.half_width_right, original.half_width_right)
                self.assertEqual(source.read_bytes(), original_bytes)

    def test_open_prefix_preserves_source_samples_and_grid(self):
        track = load_track(ROOT / 'models/tracks/2024_michigan_endurance_closed_edged.xml')
        prefix = track_view(track, is_closed=False, length=200.0)
        self.assertFalse(prefix.is_closed)
        self.assertLessEqual(prefix.lap_length, 200.0)
        self.assertLess(prefix.lap_length, track.lap_length)
        count = prefix.distance.size
        np.testing.assert_array_equal(prefix.distance, track.distance[:count])
        np.testing.assert_array_equal(prefix.curvature, track.curvature[:count])
        np.testing.assert_array_equal(prefix.segment_length, np.diff(prefix.distance))
        self.assertEqual(prefix.half_width_left.shape, prefix.distance.shape)
        self.assertEqual(prefix.path_coordinates.distance.shape, prefix.distance.shape)

    def test_cases_keep_full_pair_identical_in_size(self):
        track = load_track(ROOT / 'models/tracks/2024_michigan_endurance_closed_edged.xml')
        cases = dict(make_cases(track, 200.0))
        self.assertEqual(set(cases), {'closed_full', 'open_full', 'open_prefix'})
        self.assertEqual(cases['closed_full'].distance.size, cases['open_full'].distance.size)
        self.assertLess(cases['open_prefix'].distance.size, cases['open_full'].distance.size)

    def test_existing_output_is_rejected_without_modification(self):
        from types import SimpleNamespace
        from tools.diagnostics.compare_qss_track_boundaries import run
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'prior.json'
            contents = b'previous diagnostic: must survive\n'
            output.write_bytes(contents)
            with self.assertRaises(FileExistsError):
                run(SimpleNamespace(output=output))
            self.assertEqual(output.read_bytes(), contents)

    def test_invalid_lengths_or_closed_crops_are_rejected(self):
        track = load_track(ROOT / 'models/tracks/2024_michigan_endurance_closed_edged.xml')
        for length in (0.0, -1.0, float('nan'), track.lap_length + 1):
            with self.subTest(length=length), self.assertRaises(ValueError):
                track_view(track, is_closed=False, length=length)
        with self.assertRaises(ValueError):
            track_view(track, is_closed=True, length=200.0)


def has_casadi():
    try:
        import casadi
    except ImportError:
        return False
    return True


@unittest.skipUnless(has_casadi(), 'CasADi is not installed')
class MatchedTranscriptionTest(unittest.TestCase):
    @staticmethod
    def surface():
        from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface
        speed = np.array([0.5, 40.0])
        curvature = np.array([-1.0, 1.0])
        return CapabilitySurface(speed, curvature, np.full((2, 2), 3.0),
                                 np.full((2, 2), -8.0), np.full(2, 10.0), 'synthetic')

    def test_full_real_pairs_have_identical_interior_nlp_and_seed(self):
        import casadi as ca
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_ocp, build_mesh
        from simplified_vehicle_models.tracks.corridor import corridor_from_track
        from tools.diagnostics.compare_qss_track_boundaries import common_seed

        for name, relative in (
            ('MIS', 'models/tracks/2024_michigan_endurance_closed_edged.xml'),
            ('FSC', 'tracks_out/fsc_may_2026.xml'),
        ):
            with self.subTest(track=name):
                source = load_track(ROOT / relative)
                models, vectors = [], []
                settings = QssSettings(start_speed=10.0, open_regularisation=0.0)
                for closed in (True, False):
                    view = track_view(source, is_closed=closed)
                    corridor = corridor_from_track(view, margin=.1,
                                      half_width=None if view.has_edges else 3.0)
                    mesh = build_mesh(view, 5.0)
                    built = build_ocp(view, corridor, self.surface(), mesh, settings)
                    seed = common_seed(mesh, 10.0)
                    evaluate = ca.Function('matched', [built['nlp']['x']],
                                           [built['nlp']['f'], built['nlp']['g']])
                    objective, constraints = evaluate(seed)
                    models.append(built)
                    vectors.append((seed, float(objective), np.asarray(constraints).ravel()))
                np.testing.assert_array_equal(vectors[0][0], vectors[1][0])
                self.assertAlmostEqual(vectors[0][1], vectors[1][1], places=12)
                np.testing.assert_allclose(vectors[0][2][:-3], vectors[1][2][:-3], atol=1e-12)
                for field in ('x_lower', 'x_upper', 'g_lower', 'g_upper'):
                    np.testing.assert_array_equal(models[0][field], models[1][field])

    def test_synthetic_positive_controls_succeed_in_both_topologies(self):
        from tests.test_track_width import _write_circle_track
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings
        from simplified_vehicle_models.tracks.corridor import corridor_from_track
        from tools.diagnostics.compare_qss_track_boundaries import solve_diagnostic

        with tempfile.TemporaryDirectory() as directory:
            source = load_track(_write_circle_track(Path(directory), radius=160., nodes=720, half_width=3.))
            self.assertGreater(source.lap_length, 1000.0)
            for closed in (True, False):
                with self.subTest(closed=closed):
                    view = track_view(source, is_closed=closed)
                    result = solve_diagnostic(view, corridor_from_track(view), self.surface(),
                                 QssSettings(start_speed=10., open_regularisation=0., maximum_iterations=200), 5.)
                    self.assertEqual(result['status'], 'Solve_Succeeded')
                    self.assertTrue(result['accepted'])
                    self.assertLessEqual(result['constraint_violation'], 1e-6)
                    self.assertLessEqual(result['bound_violation'], 1e-6)

    def test_diagnostic_result_exposes_the_documented_schema(self):
        from tests.test_track_width import _write_circle_track
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings
        from simplified_vehicle_models.tracks.corridor import corridor_from_track
        from tools.diagnostics.compare_qss_track_boundaries import solve_diagnostic
        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40., nodes=72, half_width=3.))
            result = solve_diagnostic(track, corridor_from_track(track), self.surface(),
                         QssSettings(start_speed=10., open_regularisation=0., maximum_iterations=100), 5.)
        required = {'status', 'accepted', 'iterations', 'wall_seconds', 'objective_s',
                    'nodes', 'variables', 'constraints', 'is_closed', 'length_m',
                    'seed_sha256', 'constraint_violation', 'bound_violation',
                    'equality_violation', 'hs_defect_violation', 'worst_constraint'}
        self.assertLessEqual(required, set(result))
        self.assertIsInstance(result['worst_constraint'], str)

    def test_synthetic_control_is_explicitly_nonphysical(self):
        from tools.diagnostics.compare_qss_track_boundaries import synthetic_capability_control
        control = synthetic_capability_control()
        self.assertIn('NONPHYSICAL', control.provenance_hash)
        np.testing.assert_array_equal(control.speed_ceiling, 10.0)
        np.testing.assert_array_equal(control.acceleration_max, 3.0)

    def test_speed_cap_cannot_conflict_with_start_speed(self):
        from tests.test_track_width import _write_circle_track
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings
        from simplified_vehicle_models.tracks.corridor import corridor_from_track
        from tools.diagnostics.compare_qss_track_boundaries import solve_diagnostic
        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40., nodes=72, half_width=3.))
            with self.assertRaisesRegex(ValueError, 'speed cap'):
                solve_diagnostic(track, corridor_from_track(track), self.surface(),
                                 QssSettings(start_speed=10.), 5., speed_cap=9.)

    def test_failed_status_is_not_accepted_even_with_finite_returned_values(self):
        import casadi as ca
        from tests.test_track_width import _write_circle_track
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings
        from simplified_vehicle_models.tracks.corridor import corridor_from_track
        from tools.diagnostics.compare_qss_track_boundaries import solve_diagnostic

        def fake_solver(name, backend, nlp, options):
            evaluate = ca.Function('fake_diagnostic', [nlp['x']], [nlp['f'], nlp['g']])
            def invoke(**arguments):
                objective, constraints = evaluate(arguments['x0'])
                return dict(x=arguments['x0'], f=objective, g=constraints)
            invoke.stats = lambda: dict(return_status='Maximum_Iterations_Exceeded', iter_count=1)
            return invoke

        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40., nodes=72, half_width=3.))
            with patch('casadi.nlpsol', side_effect=fake_solver):
                result = solve_diagnostic(track, corridor_from_track(track), self.surface(),
                             QssSettings(start_speed=10., open_regularisation=0., maximum_iterations=1), 5.)
            self.assertFalse(result['accepted'])
            self.assertIn('Maximum_Iterations_Exceeded', result['rejection'])


if __name__ == '__main__':
    unittest.main()
