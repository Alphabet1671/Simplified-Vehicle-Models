"""Load-bearing Task 2 analytic-line pipeline regressions."""
from __future__ import annotations

import csv
import importlib.util
import math
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface
from simplified_vehicle_models.solvers.closed_loop_line import ReplaySample, _next_cap, correct_to_replay
from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh
from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line
from simplified_vehicle_models.tracks.corridor import Corridor
from simplified_vehicle_models.tracks.line_trajectory import OffsetTrajectory, ReferenceCurve
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.racing_line import path_from_solution
from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile
from tests.test_track_width import _write_circle_track
from tools.run.solve_minimum_time_line import _write_rebuilt_track


def _metric_straight() -> TrackProfile:
    """Source station s in [0, 10] with physical XY=(2s, 0)."""
    station = np.linspace(0.0, 10.0, 11)
    coordinates = PathCoordinates(
        station, 2.0 * station, np.zeros_like(station), np.zeros_like(station),
        np.zeros_like(station), False,
    )
    return TrackProfile(
        station, np.diff(station), np.full_like(station, 1.0e5),
        np.zeros_like(station), False, "metric straight", coordinates,
    )


def _surface(cap: float = 10.0) -> CapabilitySurface:
    speed = np.linspace(0.5, cap, 20)
    curvature = np.linspace(-0.1, 0.1, 5)
    return CapabilitySurface(
        speed, curvature, np.full((speed.size, curvature.size), 3.0),
        np.full((speed.size, curvature.size), -8.0),
        np.full(curvature.size, cap), "synthetic",
    )


@unittest.skipUnless(importlib.util.find_spec("scipy") is not None, "SciPy is required")
class AnalyticLinePipelineTest(unittest.TestCase):
    def test_attached_trajectory_is_authoritative_and_rejects_smoothing(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            track = load_track(_write_circle_track(Path(name), radius=30.0, nodes=72))
            spline = OffsetSpline(track, control_spacing=10.0)
            trajectory = OffsetTrajectory(
                ReferenceCurve(track), spline, np.full(spline.control_count, 3.0),
            )
            # Deliberately contradictory OCP-node offsets must never cause a refit.
            solution = SimpleNamespace(
                trajectory=trajectory, distance=track.distance,
                offset=np.zeros(track.distance.size),
            )
            rebuilt = path_from_solution(track, solution)
            expected = trajectory.sample_path(track.distance)
            np.testing.assert_allclose(rebuilt.x_north, expected.x_north, atol=1e-12)
            np.testing.assert_allclose(rebuilt.y_east, expected.y_east, atol=1e-12)
            np.testing.assert_allclose(rebuilt.reference_distance, expected.reference_distance, atol=0.0)
            with self.assertRaisesRegex(ValueError, "smoothing is not permitted"):
                path_from_solution(track, solution, smoothing=0.1)

    def test_rebuilt_widths_use_reference_station_on_denser_path_grid(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            source = _write_circle_track(root, radius=40.0, nodes=9, width_rows=9)
            track = load_track(source)
            source_station = track.distance
            with (root / "width.csv").open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(("distance", "half_width_left", "half_width_right"))
                for s in source_station:
                    writer.writerow((s, 2.0 + s / 100.0, 3.0 + s / 200.0))
            track = load_track(source)
            reference = np.linspace(0.0, track.lap_length, 17)
            geometry = track.path_coordinates
            path = PathCoordinates(
                # These deliberately differ from source station / true path length.
                1.2 * reference,
                np.interp(reference, track.distance, geometry.x_north),
                np.interp(reference, track.distance, geometry.y_east),
                np.interp(reference, track.distance, np.unwrap(geometry.heading)),
                np.interp(reference, track.distance, geometry.curvature),
                True, reference,
            )
            _, geometry_csv = _write_rebuilt_track(source, root / "out", path)
            self.assertTrue(geometry_csv.is_file())
            with (root / "out" / "width_rebuilt.csv").open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            left = np.array([float(row["half_width_left"]) for row in rows])
            right = np.array([float(row["half_width_right"]) for row in rows])
            # Geometry and heading are sampled on reference station, so this also
            # removes the normal-projection factor: only reference interpolation
            # can produce these nonconstant values on the denser grid.
            np.testing.assert_allclose(left, 2.0 + reference / 100.0, atol=2e-10)
            np.testing.assert_allclose(right, 3.0 + reference / 200.0, atol=2e-10)

    @unittest.skipUnless(importlib.util.find_spec("casadi") is not None, "CasADi is required")
    def test_nonunit_reference_metric_controls_time_and_path_length(self) -> None:
        track = _metric_straight()
        corridor = Corridor(
            track.distance, np.zeros_like(track.distance), np.zeros_like(track.distance),
            "synthetic", False,
        )
        settings = QssSettings(maximum_iterations=2000, start_speed=10.0)
        mesh = build_mesh(track, 1.0)
        solution = solve_spline_line(
            track, corridor, _surface(10.0), mesh,
            OffsetSpline(track, control_spacing=2.5), settings,
            speed_cap=np.full(mesh.size, 10.0),
        )
        self.assertEqual(solution.status, "Solve_Succeeded")
        self.assertIsNotNone(solution.trajectory)
        self.assertAlmostEqual(solution.trajectory.station_map.path_end, 20.0, places=7)
        self.assertAlmostEqual(solution.lap_time, 2.0, places=6)
        np.testing.assert_allclose(solution.offset, 0.0, atol=1e-10)
        np.testing.assert_allclose(solution.speed, 10.0, atol=1e-5)

    def test_slowdown_at_ell_90_maps_to_reference_s_100(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            track = load_track(_write_circle_track(Path(name), radius=30.0, nodes=121))
            spline = OffsetSpline(track, control_spacing=10.0)
            trajectory = OffsetTrajectory(
                ReferenceCurve(track), spline, np.full(spline.control_count, 3.0),
            )
            path_end = trajectory.station_map.path_end
            self.assertAlmostEqual(float(trajectory.station_map.to_path(100.0)), 90.0, places=5)
            replay = ReplaySample(
                np.array([0.0, 90.0, path_end]), np.array([20.0, 5.0, 20.0]),
                True, 1.0, reference_distance=np.array([0.0, 100.0, track.lap_length]),
            )
            cap = _next_cap(
                np.array([0.0, 100.0]), replay, np.array([20.0, 20.0]),
                None, 0.5, 0.1,
            )
            # The narrow cap applies at source s=100 (ell=90), not s=90.
            np.testing.assert_allclose(cap, [20.0, 4.9], atol=1e-12)

    def test_trajectory_metadata_persists_station_map_and_controls(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            track = load_track(_write_circle_track(Path(name), radius=30.0, nodes=72))
            spline = OffsetSpline(track, control_spacing=10.0)
            coefficients = np.linspace(-0.2, 0.3, spline.control_count)
            trajectory = OffsetTrajectory(ReferenceCurve(track), spline, coefficients)
            solution = SimpleNamespace(
                trajectory=trajectory, coefficients=coefficients,
                distance=np.linspace(0.0, track.lap_length, 11),
                speed=np.linspace(8.0, 10.0, 11),
                acceleration=np.linspace(-1.0, 1.0, 11),
                midpoint_acceleration=np.linspace(-0.8, 0.8, 10),
            )
            from tools.run.solve_minimum_time_line import _write_trajectory_metadata
            artifact = Path(name) / "trajectory.npz"
            _write_trajectory_metadata(artifact, solution)
            with np.load(artifact) as data:
                self.assertEqual(set(data.files), {
                    "metadata", "source_reference_station", "reference_north", "reference_east",
                    "map_reference_station", "path_station", "control_station", "coefficients", "node_station",
                    "node_speed", "node_acceleration", "midpoint_acceleration",
                })
                metadata = __import__("json").loads(str(data["metadata"]))
                self.assertEqual(metadata["schema_version"], 1)
                self.assertTrue(metadata["is_closed"])
                self.assertEqual(metadata["reference_interpolant"], "scipy.make_interp_spline")
                np.testing.assert_allclose(data["source_reference_station"], trajectory.reference.source_station)
                np.testing.assert_allclose(data["map_reference_station"], trajectory.station_map.reference_station)
                reference = trajectory.reference.evaluate(trajectory.reference.source_station)
                np.testing.assert_allclose(data["reference_north"], reference.north)
                np.testing.assert_allclose(data["reference_east"], reference.east)
                np.testing.assert_allclose(data["path_station"], trajectory.station_map.path_station)
                np.testing.assert_allclose(data["control_station"], spline.control_distance)
                np.testing.assert_allclose(data["coefficients"], coefficients)
                np.testing.assert_allclose(data["node_speed"], solution.speed)
                np.testing.assert_allclose(data["midpoint_acceleration"], solution.midpoint_acceleration)
            with self.assertRaises(FileExistsError):
                _write_trajectory_metadata(artifact, solution)

    def test_incomplete_analytic_replay_mapping_rejects_before_time_match(self) -> None:
        mesh = np.array([0.0, 10.0])
        solution = SimpleNamespace(
            distance=mesh, speed=np.array([10.0, 10.0]), lap_time=1.0,
            trajectory=object(),
        )
        path = PathCoordinates(
            np.array([0.0, 20.0]), np.array([0.0, 20.0]), np.zeros(2),
            np.zeros(2), np.zeros(2), False, reference_distance=mesh,
        )

        def replay(_path, _controller):
            # Time agrees exactly, but the last OCP station is not covered.
            return ReplaySample(
                np.array([0.0, 18.0]), np.array([10.0, 10.0]), True, 1.0,
                reference_distance=np.array([0.0, 9.0]),
            )

        with (
            patch("simplified_vehicle_models.solvers.closed_loop_line.solve_spline_line", return_value=solution),
            patch("simplified_vehicle_models.solvers.closed_loop_line.path_from_solution", return_value=path),
        ):
            with self.assertRaisesRegex(ValueError, "does not cover the OCP mesh"):
                correct_to_replay(
                    None, None, None, mesh, None, QssSettings(), None, replay,
                )


if __name__ == "__main__":
    unittest.main()
