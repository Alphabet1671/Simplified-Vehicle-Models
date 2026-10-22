"""Tests for closed-loop replay correction of the spline minimum-time line."""

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.solvers import closed_loop_line
from simplified_vehicle_models.solvers.closed_loop_line import (
    CAP_MARGIN,
    ReplaySample,
    _matches,
    _next_cap,
    correct_to_replay,
)
from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh
from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line
from simplified_vehicle_models.tracks.corridor import corridor_from_track
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.tracks.profile import load_track
from tests.test_qss_spline_ocp import _surface
from tests.test_track_width import _write_circle_track


def _case(directory: str, *, spacing: float = 3.0):
    """Build the circle track, corridor, spline, mesh, and settings for a test."""
    track = load_track(_write_circle_track(
        Path(directory), radius=40.0, nodes=72, half_width=3.0,
    ))
    corridor = corridor_from_track(track)
    spline = OffsetSpline(track, control_spacing=10.0)
    mesh = build_mesh(track, spacing)
    return track, corridor, spline, mesh, QssSettings(maximum_iterations=2000)


class ClosedLoopCorrectionTest(unittest.TestCase):
    def test_caps_are_lowered_when_replay_is_infeasible(self):
        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40.0, nodes=72,
                                                   half_width=3.0))
            corridor = corridor_from_track(track)
            spline = OffsetSpline(track, control_spacing=10.0)
            mesh = build_mesh(track, 3.0)
            calls = []

            def replay(path, controller):
                calls.append(path.distance.copy())
                return ReplaySample(path.distance, np.full(path.distance.size, 8.0), False,
                                    float(path.lap_length / 8.0), ('speed',))

            result = correct_to_replay(track, corridor, _surface(), mesh, spline,
                                       QssSettings(maximum_iterations=2000), None, replay,
                                       maximum_rounds=2)
            self.assertEqual(result.rounds, 2)
            self.assertFalse(result.accepted)
            self.assertFalse(result.feasible)
            self.assertGreaterEqual(len(calls), 2)
            self.assertTrue(np.all(result.speed_caps[-1] <= result.speed_caps[0]))
            self.assertTrue(np.any(result.speed_caps[-1] < result.speed_caps[0]))

    def test_matching_feasible_replay_is_accepted_on_the_first_round(self):
        with tempfile.TemporaryDirectory() as directory:
            track, corridor, spline, mesh, settings = _case(directory, spacing=5.0)
            surface = _surface()
            baseline = solve_spline_line(
                track, corridor, surface, mesh, spline, settings,
            )

            def replay(path, controller):
                # The speed trace is deliberately inconsistent with the reported
                # time to prove that the adapter's own lap time is authoritative.
                return ReplaySample(
                    path.distance, np.full(path.distance.size, 8.0), True,
                    baseline.lap_time,
                )

            result = correct_to_replay(
                track, corridor, surface, mesh, spline, settings, None, replay,
            )

            self.assertTrue(result.accepted)
            self.assertTrue(result.feasible)
            self.assertEqual(result.rounds, 1)
            self.assertEqual(len(result.speed_caps), 1)
            np.testing.assert_allclose(
                result.speed_caps[0], baseline.speed, rtol=0.0, atol=1.0e-9,
            )
            self.assertFalse(result.speed_caps[0].flags.writeable)
            self.assertEqual(result.cap_margin, CAP_MARGIN)
            self.assertAlmostEqual(result.replay_time, baseline.lap_time)

    def test_time_disagreement_lowers_caps_until_replay_matches(self):
        with tempfile.TemporaryDirectory() as directory:
            track, corridor, spline, mesh, settings = _case(directory, spacing=5.0)
            surface = _surface()
            baseline = solve_spline_line(
                track, corridor, surface, mesh, spline, settings,
            )
            slow_speed = 0.9 * np.asarray(baseline.speed, dtype=float)
            cap = np.maximum(
                np.minimum(np.asarray(baseline.speed, dtype=float), slow_speed)
                - CAP_MARGIN,
                settings.minimum_speed,
            )
            capped = solve_spline_line(
                track, corridor, surface, mesh, spline, settings, speed_cap=cap,
            )
            calls = []

            def replay(path, controller):
                calls.append(path.distance.copy())
                if len(calls) == 1:
                    return ReplaySample(
                        path.distance,
                        np.interp(path.reference_distance, baseline.distance, slow_speed),
                        True, baseline.lap_time / 0.9,
                    )
                return ReplaySample(
                    path.distance,
                    np.interp(path.reference_distance, capped.distance, capped.speed),
                    True, capped.lap_time,
                )

            result = correct_to_replay(
                track, corridor, surface, mesh, spline, settings, None, replay,
                maximum_rounds=3,
            )

            self.assertTrue(result.accepted)
            self.assertEqual(result.rounds, 2)
            self.assertEqual(len(calls), 2)
            self.assertEqual(len(result.speed_caps), 2)
            self.assertTrue(np.all(result.speed_caps[1] <= result.speed_caps[0]))
            self.assertTrue(np.any(result.speed_caps[1] < result.speed_caps[0]))
            self.assertAlmostEqual(result.replay_time, capped.lap_time)

    def test_replay_faster_within_tolerance_is_accepted_without_correction(self):
        with tempfile.TemporaryDirectory() as directory:
            track, corridor, spline, mesh, settings = _case(directory, spacing=5.0)
            surface = _surface()
            baseline = solve_spline_line(
                track, corridor, surface, mesh, spline, settings,
            )
            faster = 0.995 * baseline.lap_time

            def replay(path, controller):
                return ReplaySample(
                    path.distance, np.full(path.distance.size, 8.0), True, faster,
                )

            result = correct_to_replay(
                track, corridor, surface, mesh, spline, settings, None, replay,
            )

            self.assertTrue(result.accepted)
            self.assertTrue(result.feasible)
            self.assertEqual(result.rounds, 1)
            self.assertEqual(len(result.speed_caps), 1)
            self.assertAlmostEqual(result.replay_time, faster)
            # An in-tolerance replay must not tighten the caps for another solve.
            np.testing.assert_allclose(
                result.speed_caps[0], baseline.speed, rtol=0.0, atol=1.0e-9,
            )

    def test_matches_is_two_sided_within_tolerance(self):
        solution = SimpleNamespace(lap_time=100.0)
        def sample(replay_time, feasible=True):
            return ReplaySample(
                np.array([0.0, 1.0]), np.array([5.0, 5.0]), feasible,
                replay_time,
            )

        self.assertTrue(_matches(solution, sample(99.5), 0.01))
        self.assertTrue(_matches(solution, sample(100.5), 0.01))
        # A materially faster replay is not silently accepted as agreement.
        self.assertFalse(_matches(solution, sample(98.9), 0.01))
        self.assertFalse(_matches(solution, sample(101.1), 0.01))
        self.assertFalse(_matches(solution, sample(100.0, feasible=False), 0.01))

    def test_next_cap_only_lowers_where_replay_is_slower(self):
        mesh = np.array([0.0, 1.0, 2.0])
        ocp = np.array([10.0, 10.0, 10.0])
        # faster, slower, equal to the OCP speed
        replay = ReplaySample(
            mesh, np.array([12.0, 8.0, 10.0]), True, 1.0,
        )
        cap = _next_cap(mesh, replay, ocp, None, 0.5, CAP_MARGIN)
        np.testing.assert_allclose(cap, [10.0, 8.0 - CAP_MARGIN, 10.0])
        # A previous lower cap still wins and is never raised.
        previous = np.array([9.0, 9.0, 9.0])
        capped = _next_cap(mesh, replay, ocp, previous, 0.5, CAP_MARGIN)
        np.testing.assert_allclose(capped, [9.0, 8.0 - CAP_MARGIN, 9.0])
        # A replay that is faster at every node keeps the OCP speed.
        fast = ReplaySample(mesh, np.array([11.0, 12.0, 13.0]), True, 1.0)
        np.testing.assert_allclose(
            _next_cap(mesh, fast, ocp, None, 0.5, CAP_MARGIN), ocp,
        )

    def test_materially_faster_replay_does_not_reduce_caps(self):
        with tempfile.TemporaryDirectory() as directory:
            track, corridor, spline, mesh, settings = _case(directory, spacing=5.0)
            surface = _surface()
            baseline = solve_spline_line(
                track, corridor, surface, mesh, spline, settings,
            )
            calls = []

            def replay(path, controller):
                calls.append(path.distance.copy())
                # Faster than the OCP at every node, paired with a materially
                # faster reported lap. This must not lower any node's cap.
                speed = np.full(path.distance.size, 30.0)
                reported = (
                    baseline.lap_time if len(calls) > 1
                    else 0.9 * baseline.lap_time
                )
                return ReplaySample(path.distance, speed, True, reported)

            result = correct_to_replay(
                track, corridor, surface, mesh, spline, settings, None, replay,
                maximum_rounds=2,
            )

            self.assertTrue(result.accepted)
            self.assertEqual(len(calls), 2)
            self.assertEqual(len(result.speed_caps), 2)
            # The faster replay is not a reason to lower any node's cap, so the
            # second solve uses exactly the first solve's speed profile.
            np.testing.assert_allclose(
                result.speed_caps[1], result.speed_caps[0],
                rtol=0.0, atol=1.0e-9,
            )

    def test_solver_failure_under_a_cap_returns_an_unaccepted_result(self):
        with tempfile.TemporaryDirectory() as directory:
            track, corridor, spline, mesh, settings = _case(directory, spacing=5.0)
            surface = _surface()
            real_solve = closed_loop_line.solve_spline_line
            calls = []
            replay_times = []

            def replay(path, controller):
                calls.append(path.distance.copy())
                replay_times.append(float(path.lap_length / 8.0))
                return ReplaySample(
                    path.distance, np.full(path.distance.size, 8.0), False,
                    replay_times[-1], ('speed',),
                )

            def fail_under_cap(*args, **kwargs):
                if kwargs.get('speed_cap') is None:
                    return real_solve(*args, **kwargs)
                raise RuntimeError("infeasible under the corrected cap")

            with patch.object(
                closed_loop_line, 'solve_spline_line', side_effect=fail_under_cap,
            ):
                result = correct_to_replay(
                    track, corridor, surface, mesh, spline, settings, None, replay,
                )

            self.assertFalse(result.accepted)
            self.assertEqual(result.rounds, 2)
            self.assertEqual(len(calls), 1)
            self.assertEqual(len(result.speed_caps), 1)
            self.assertTrue(np.isfinite(result.replay_time))
            self.assertAlmostEqual(result.replay_time, replay_times[-1])

    def test_closed_loop_rejects_a_replay_that_never_becomes_feasible(self):
        with tempfile.TemporaryDirectory() as directory:
            track = load_track(_write_circle_track(Path(directory), radius=40.0, nodes=72,
                                                   half_width=3.0))
            corridor = corridor_from_track(track)
            spline = OffsetSpline(track, control_spacing=10.0)
            mesh = build_mesh(track, 3.0)

            def always_infeasible(path, controller):
                return ReplaySample(path.distance, np.full(path.distance.size, 8.0), False,
                                    float(path.lap_length / 8.0), ('speed',))

            result = correct_to_replay(track, corridor, _surface(), mesh, spline,
                                       QssSettings(maximum_iterations=2000), None,
                                       always_infeasible, maximum_rounds=3)
            self.assertFalse(result.accepted)
            self.assertEqual(result.rounds, 3)
            # Every round reached a replay, so the budget was exhausted rather
            # than the loop stopping early on an accepted or failed solve.
            self.assertEqual(len(result.speed_caps), 3)
            self.assertTrue(np.isfinite(result.replay_time))

    def test_rejects_invalid_correction_controls(self):
        def replay(path, controller):
            raise AssertionError("validation must run before any replay")

        arguments = (None, None, None, None, None, None, None, replay)
        with self.assertRaises(ValueError):
            correct_to_replay(*arguments, tolerance=-0.01)
        with self.assertRaises(TypeError):
            correct_to_replay(*arguments, tolerance=True)
        with self.assertRaises(ValueError):
            correct_to_replay(*arguments, maximum_rounds=0)
        with self.assertRaises(TypeError):
            correct_to_replay(*arguments, maximum_rounds=True)
        with self.assertRaises(TypeError):
            correct_to_replay(None, None, None, None, None, None, None, None)
