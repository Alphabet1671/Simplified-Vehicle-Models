import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

from tools.run.solve_minimum_time_line import _validate_output_paths, _write_rebuilt_track
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.types import PathCoordinates

from tools.gates.check_minimum_time_line import (
    check_corridor_containment,
    check_free_line_not_slower,
    check_mesh_convergence,
    check_replay_agreement,
    check_zero_corridor_matches_envelope,
)


class GateCheckTest(unittest.TestCase):
    def test_zero_corridor_check_uses_relative_tolerance(self) -> None:
        self.assertTrue(check_zero_corridor_matches_envelope(open_loop=61.14, ocp=61.30, tolerance=0.01))
        self.assertFalse(check_zero_corridor_matches_envelope(open_loop=61.14, ocp=65.00, tolerance=0.01))

    def test_free_line_check_rejects_a_slower_line(self) -> None:
        self.assertTrue(check_free_line_not_slower(centreline=61.14, free=58.90))
        self.assertFalse(check_free_line_not_slower(centreline=61.14, free=61.60))
        self.assertFalse(check_free_line_not_slower(centreline=61.14, free=61.20))
        self.assertTrue(check_free_line_not_slower(centreline=61.14, free=61.20, tolerance=0.002))
        self.assertFalse(check_free_line_not_slower(centreline=61.14, free=61.14, tolerance=-0.001))

    def test_containment_check_tests_both_edges(self) -> None:
        minimum = np.array([-1.0, -2.0, -1.0])
        maximum = np.array([1.0, 2.0, 1.0])
        self.assertTrue(check_corridor_containment(np.array([0.0, 1.9, -0.9]), minimum, maximum))
        self.assertFalse(check_corridor_containment(np.array([0.0, 2.1, -0.9]), minimum, maximum))

    def test_mesh_convergence_check_uses_relative_change(self) -> None:
        self.assertTrue(check_mesh_convergence(coarse=58.90, fine=58.85, tolerance=0.005))
        self.assertFalse(check_mesh_convergence(coarse=58.90, fine=59.40, tolerance=0.005))

    def test_replay_check_compares_lap_times(self) -> None:
        self.assertTrue(check_replay_agreement(ocp=58.85, replay=58.80, tolerance=0.01))
        self.assertFalse(check_replay_agreement(ocp=58.85, replay=59.60, tolerance=0.01))

    def test_checks_reject_nonfinite_inputs_and_invalid_tolerances(self) -> None:
        self.assertFalse(check_zero_corridor_matches_envelope(open_loop=float("nan"), ocp=1.0, tolerance=0.01))
        self.assertFalse(check_free_line_not_slower(centreline=10.0, free=float("inf")))
        self.assertFalse(check_mesh_convergence(coarse=10.0, fine=10.0, tolerance=float("nan")))
        self.assertFalse(check_replay_agreement(ocp=10.0, replay=10.0, tolerance=-0.01))
        self.assertFalse(check_corridor_containment([0.0], [float("nan")], [1.0]))
        self.assertFalse(check_corridor_containment([], [], []))
        self.assertFalse(check_corridor_containment([0.0], [1.0], [-1.0]))

    def test_source_hashes_use_role_keys_and_include_referenced_assets(self) -> None:
        from tools.run.solve_minimum_time_line import _source_hashes

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "track.xml"
            source.write_text(
                '<model schema-version="1.0" type="track-model" subtype="closed-xy-profile">'
                '<metadata><title>fixture</title></metadata>'
                '<data ref="geometry.csv" /><data role="width" ref="width.csv" />'
                '</model>', encoding="utf-8",
            )
            (root / "geometry.csv").write_text("distance,x_north,y_east\n",
                                               encoding="utf-8")
            (root / "width.csv").write_text("distance,left,right\n",
                                            encoding="utf-8")
            hashes = _source_hashes(source)
        self.assertEqual(set(hashes), {"track_xml", "track_csv", "width_csv"})
        self.assertTrue(all(len(value) == 64 for value in hashes.values()))

    def test_rebuilt_track_rejects_duplicate_width_roles(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.xml"
            source.write_text(
                '<model schema-version="1.0" type="track-model" subtype="closed-xy-profile">'
                '<metadata><title>fixture</title></metadata><data role="width" ref="a.csv" />'
                '<data role="width" ref="b.csv" /><data ref="geometry.csv" />'
                '<parameters><parameter name="is_closed" value="true" />'
                '<parameter name="terminal_segment_is_duplicate" value="true" />'
                '<parameter name="maximum_radius" value="100" /></parameters></model>',
                encoding="utf-8",
            )
            path = PathCoordinates(
                distance=np.array([0., 1., 2.]), x_north=np.array([0., 1., 0.]),
                y_east=np.array([0., 0., 1.]), heading=np.zeros(3),
                curvature=np.ones(3), is_closed=True,
            )
            with self.assertRaisesRegex(ValueError, "multiple width-role"):
                _write_rebuilt_track(source, root / "out", path)

    def test_rebuilt_track_rejects_large_heading_change_for_width_projection(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.xml"
            source.write_text(
                '<model schema-version="1.0" type="track-model" subtype="closed-xy-profile">'
                '<metadata><title>fixture</title></metadata><data role="width" ref="width.csv" />'
                '<data ref="geometry.csv" /><parameters><parameter name="is_closed" value="true" />'
                '<parameter name="terminal_segment_is_duplicate" value="true" />'
                '<parameter name="maximum_radius" value="100" />'
                '<parameter name="width_provenance" value="surveyed" /></parameters></model>',
                encoding="utf-8",
            )
            (root / "geometry.csv").write_text(
                "distance,x_north,y_east,heading_rad,curvature\n"
                "0,0,0,0,1\n1,0.841470984808,0.459697694132,1,1\n"
                "2,0.909297426826,1.41614683655,2,1\n3,0.14112000806,1.9899924966,3,1\n"
                "4,0,0,0,1\n", encoding="utf-8",
            )
            (root / "width.csv").write_text(
                "distance,half_width_left,half_width_right\n0,2,3\n1,2,3\n"
                "2,2,3\n3,2,3\n4,2,3\n", encoding="utf-8",
            )
            path = PathCoordinates(
                distance=np.array([0., 1., 2., 3., 4.]),
                x_north=np.array([0., 0.8, 0.7, 0.1, 0.]),
                y_east=np.array([0., 0.6, 1.6, 2., 0.]),
                heading=np.array([0., 1.4, 2.4, 3.4, 0.]),
                curvature=np.ones(5), is_closed=True,
            )
            with self.assertRaisesRegex(ValueError, "heading differs by more than 0.25"):
                _write_rebuilt_track(source, root / "out", path)

    def test_rebuilt_track_wraps_equivalent_heading_angles(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.xml"
            source.write_text(
                '<model schema-version="1.0" type="track-model" subtype="closed-xy-profile">'
                '<metadata><title>fixture</title></metadata><data role="width" ref="width.csv" />'
                '<data ref="geometry.csv" /><parameters><parameter name="is_closed" value="true" />'
                '<parameter name="terminal_segment_is_duplicate" value="true" />'
                '<parameter name="maximum_radius" value="100" />'
                '<parameter name="width_provenance" value="surveyed" /></parameters></model>',
                encoding="utf-8",
            )
            (root / "geometry.csv").write_text(
                "distance,x_north,y_east,heading_rad,curvature\n"
                "0,0,0,0,1\n1,0.841470984808,0.459697694132,1,1\n"
                "2,0.909297426826,1.41614683655,2,1\n3,0.14112000806,1.9899924966,3,1\n"
                "4,0,0,0,1\n", encoding="utf-8",
            )
            (root / "width.csv").write_text(
                "distance,half_width_left,half_width_right\n"
                "0,2,3\n1,2,3\n2,2,3\n3,2,3\n4,2,3\n", encoding="utf-8",
            )
            path = PathCoordinates(
                distance=np.array([0., 1.1, 2.2, 3.3, 4.4]),
                x_north=np.array([0., 0.9, 0.9, 0.1, 0.]),
                y_east=np.array([0., 0.4, 1.4, 2., 0.]),
                heading=np.array([2. * np.pi, 1. + 2. * np.pi, 2. + 2. * np.pi, 3. + 2. * np.pi, 2. * np.pi]),
                curvature=np.ones(5), is_closed=True,
            )
            xml_path, _ = _write_rebuilt_track(source, root / "out", path)
            self.assertTrue(xml_path.is_file())

    def test_rebuilt_track_converts_constant_width_parameter(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.xml"
            source.write_text(
                '<model schema-version="1.0" type="track-model" subtype="closed-xy-profile">'
                '<metadata><title>fixture</title></metadata><data ref="geometry.csv" />'
                '<parameters><parameter name="is_closed" value="true" />'
                '<parameter name="terminal_segment_is_duplicate" value="true" />'
                '<parameter name="maximum_radius" value="100" />'
                '<parameter name="half_width" value="3" />'
                '<parameter name="width_provenance" value="assumed" /></parameters></model>',
                encoding="utf-8",
            )
            (root / "geometry.csv").write_text(
                "distance,x_north,y_east,heading_rad,curvature\n"
                "0,0,0,0,1\n1,0.841470984808,0.459697694132,1,1\n"
                "2,0.909297426826,1.41614683655,2,1\n3,0.14112000806,1.9899924966,3,1\n"
                "4,0,0,0,1\n", encoding="utf-8",
            )
            path = PathCoordinates(
                distance=np.array([0., 1.1, 2.2, 3.3, 4.4]),
                x_north=np.array([0., 0.9, 0.9, 0.1, 0.]),
                y_east=np.array([0., 0.4, 1.4, 2., 0.]),
                heading=np.array([0., 1., 2., 3., 0.]),
                curvature=np.ones(5), is_closed=True,
            )
            xml_path, _ = _write_rebuilt_track(source, root / "out", path)
            rebuilt = load_track(xml_path)
            self.assertEqual(rebuilt.width_provenance, "assumed")
            self.assertIsNone(ET.parse(xml_path).getroot().find("./parameters/parameter[@name='half_width']"))
            self.assertIsNotNone(ET.parse(xml_path).getroot().find("./data[@role='width']"))
            self.assertEqual(rebuilt.half_width_left.shape, path.distance.shape)
            self.assertEqual(rebuilt.half_width_right.shape, path.distance.shape)

    def test_rebuilt_track_round_trips_widths_on_new_distance_grid(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.xml"
            source.write_text(
                '<model schema-version="1.0" type="track-model" subtype="closed-xy-profile">'
                '<metadata><title>fixture</title></metadata>'
                '<data role="width" ref="width.csv" />'
                '<data ref="geometry.csv" />'
                '<parameters><parameter name="is_closed" value="true" />'
                '<parameter name="terminal_segment_is_duplicate" value="true" />'
                '<parameter name="maximum_radius" value="100" />'
                '<parameter name="width_provenance" value="surveyed" /></parameters></model>',
                encoding="utf-8",
            )
            (root / "geometry.csv").write_text(
                "distance,x_north,y_east,heading_rad,curvature\n"
                "0,0,0,0,1\n1,0.841470984808,0.459697694132,1,1\n"
                "2,0.909297426826,1.41614683655,2,1\n"
                "3,0.14112000806,1.9899924966,3,1\n"
                "4,0,0,0,1\n", encoding="utf-8",
            )
            (root / "width.csv").write_text(
                "distance,half_width_left,half_width_right\n"
                "0,2,3\n1,2.1,3.1\n2,2.2,3.2\n3,2.3,3.3\n4,2.4,3.4\n", encoding="utf-8",
            )
            path = PathCoordinates(
                distance=np.array([0., 1.1, 2.2, 3.3, 4.4]),
                x_north=np.array([0., 0.9, 0.9, 0.1, 0.]),
                y_east=np.array([0., 0.4, 1.4, 2., 0.]),
                heading=np.array([0., 1., 2., 3., 0.]),
                curvature=np.ones(5),
                is_closed=True,
            )
            xml_path, _ = _write_rebuilt_track(source, root / "out", path)
            rebuilt = load_track(xml_path)
            np.testing.assert_allclose(rebuilt.distance, path.distance)
            original_heading = np.array([0., 1., 2., 3., 0.])
            original_north = np.array([0., 0.841470984808, 0.909297426826, 0.14112000806, 0.])
            original_east = np.array([0., 0.459697694132, 1.41614683655, 1.9899924966, 0.])
            source_left = np.array([2.0, 2.1, 2.2, 2.3, 2.4])
            source_right = np.array([3.0, 3.1, 3.2, 3.3, 3.4])
            old_left_north = original_north + source_left * np.sin(original_heading)
            old_left_east = original_east - source_left * np.cos(original_heading)
            new_left_north = path.x_north + rebuilt.half_width_left * np.sin(path.heading)
            new_left_east = path.y_east - rebuilt.half_width_left * np.cos(path.heading)
            rebuilt_left_normal = np.column_stack((np.sin(path.heading), -np.cos(path.heading)))
            old_left_vector = np.column_stack((old_left_north - path.x_north, old_left_east - path.y_east))
            np.testing.assert_allclose(rebuilt.half_width_left, np.sum(old_left_vector * rebuilt_left_normal, axis=1))
            old_right_north = original_north - source_right * np.sin(original_heading)
            old_right_east = original_east + source_right * np.cos(original_heading)
            new_right_north = path.x_north - rebuilt.half_width_right * np.sin(path.heading)
            new_right_east = path.y_east + rebuilt.half_width_right * np.cos(path.heading)
            rebuilt_right_normal = np.column_stack((-np.sin(path.heading), np.cos(path.heading)))
            old_right_vector = np.column_stack((old_right_north - path.x_north, old_right_east - path.y_east))
            np.testing.assert_allclose(rebuilt.half_width_right, np.sum(old_right_vector * rebuilt_right_normal, axis=1))
            tree = ET.parse(xml_path)
            rows = tree.getroot().findall("./data")
            self.assertEqual(rows[0].get("role"), "width")
            self.assertEqual(rows[1].get("role"), None)


class ExactSplineExportTest(unittest.TestCase):
    def test_adaptive_zero_offset_skidpad_round_trips_loader(self) -> None:
        """Exact arc-length export is resampled, not replaced by chord lengths."""
        from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve, OffsetTrajectory
        from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
        source = Path("models/tracks/fsae_2026_skidpad.xml")
        track = load_track(source)
        spline = OffsetSpline(track, control_spacing=10.0)
        trajectory = OffsetTrajectory(ReferenceCurve(track), spline,
                                      np.zeros(spline.control_count))
        path = trajectory.sample_path(track.distance)
        self.assertGreater(path.distance.size, track.distance.size)
        self.assertIsNotNone(path.reference_distance)
        with tempfile.TemporaryDirectory() as directory:
            xml_path, _ = _write_rebuilt_track(source, Path(directory), path)
            rebuilt = load_track(xml_path)
        self.assertGreater(rebuilt.path_coordinates.distance[-1], 0.0)
        np.testing.assert_allclose(path.distance[-1], trajectory.station_map.path_end,
                                   rtol=0.0, atol=1e-7)


class RunnerOrchestrationTest(unittest.TestCase):
    def test_requested_tracks_match_current_runner_scope(self) -> None:
        from tools.run.solve_minimum_time_line import _validate_track_for_line_pipeline

        tracks = (
            ("MIS", Path("models/tracks/2024_michigan_endurance_closed_edged.xml"), True, True),
            ("FSC", Path("tracks_out/fsc_may_2026.xml"), True, False),
            ("skidpad", Path("models/tracks/fsae_2026_skidpad.xml"), False, False),
            ("acceleration", Path("models/tracks/fsae_2026_acceleration.xml"), False, False),
        )
        for name, path, closed, has_edges in tracks:
            with self.subTest(track=name):
                track = load_track(path)
                self.assertEqual(track.is_closed, closed)
                self.assertEqual(track.has_edges, has_edges)
                if has_edges:
                    _validate_track_for_line_pipeline(track)
                    with self.assertRaisesRegex(ValueError, "only valid for a track without edges"):
                        _validate_track_for_line_pipeline(track, half_width=2.0)
                else:
                    with self.assertRaisesRegex(ValueError, "corridor or half_width"):
                        _validate_track_for_line_pipeline(track)
                    _validate_track_for_line_pipeline(track, half_width=2.0)

    def test_output_collision_is_rejected_before_source_bytes_change(self) -> None:
        from tools.run import solve_minimum_time_line as runner

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            geometry = root / "racing_line_track.csv"
            geometry_bytes = b"original source geometry\n"
            geometry.write_bytes(geometry_bytes)
            source = root / "source.xml"
            source.write_text(
                '<model schema-version="1.0" type="track-model" subtype="closed-xy-profile">'
                '<metadata><title>collision fixture</title></metadata>'
                '<data ref="racing_line_track.csv" />'
                '</model>',
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "overwrite source track asset"):
                _validate_output_paths(source, root)
            self.assertEqual(geometry.read_bytes(), geometry_bytes)

            with self.assertRaisesRegex(ValueError, "overwrite source track asset"):
                _write_rebuilt_track(source, root, SimpleNamespace())
            self.assertEqual(geometry.read_bytes(), geometry_bytes)
            self.assertFalse((root / "racing_line_track.xml").exists())

            alias_directory = root / "alias_output"
            alias_directory.mkdir()
            (alias_directory / "line.csv").symlink_to(geometry)
            with self.assertRaisesRegex(ValueError, "overwrite source track asset"):
                _validate_output_paths(source, alias_directory)
            self.assertEqual(geometry.read_bytes(), geometry_bytes)

            hardlink_directory = root / "hardlink_output"
            hardlink_directory.mkdir()
            hardlink_target = hardlink_directory / "line.csv"
            try:
                hardlink_target.hardlink_to(geometry)
            except (OSError, NotImplementedError):
                pass
            else:
                with self.assertRaisesRegex(ValueError, "overwrite source track asset"):
                    _validate_output_paths(source, hardlink_directory)
                self.assertEqual(geometry.read_bytes(), geometry_bytes)

            args = SimpleNamespace(track=source, output=root, margin=.1)
            with self.assertRaisesRegex(ValueError, "overwrite source track asset"):
                runner.run(args)
            self.assertEqual(geometry.read_bytes(), geometry_bytes)

            generated_names = (
                "line.csv", "racing_line.csv", "racing_line_track.csv",
                "racing_line_track.xml", "rebuilt_width.csv", "summary.json",
            )
            for name in generated_names:
                asset = root / name
                asset_bytes = f"source asset {name}\n".encode()
                asset.write_bytes(asset_bytes)
                xml = root / f"ref_{name}.xml"
                xml.write_text(
                    '<model schema-version="1.0" type="track-model" subtype="closed-xy-profile">'
                    f'<metadata><title>fixture</title></metadata><data ref="{name}" />'
                    '</model>',
                    encoding="utf-8",
                )
                with self.subTest(generated_name=name):
                    with self.assertRaisesRegex(ValueError, "overwrite source track asset"):
                        _validate_output_paths(xml, root)
                    self.assertEqual(asset.read_bytes(), asset_bytes)

            width_asset = root / "track.csv"
            width_bytes = b"original width data\n"
            width_asset.write_bytes(width_bytes)
            geometry_collision = root / "track_rebuilt.csv"
            geometry_bytes = b"original geometry data\n"
            geometry_collision.write_bytes(geometry_bytes)
            width_source = root / "width_source.xml"
            width_source.write_text(
                '<model schema-version="1.0" type="track-model" subtype="closed-xy-profile">'
                '<metadata><title>width collision</title></metadata>'
                '<data ref="track_rebuilt.csv" /><data role="width" ref="track.csv" />'
                '</model>',
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "overwrite source track asset"):
                _validate_output_paths(width_source, root)
            self.assertEqual(width_asset.read_bytes(), width_bytes)
            self.assertEqual(geometry_collision.read_bytes(), geometry_bytes)

            source_named_as_output = root / "summary.json"
            source_xml_bytes = (
                b'<model schema-version="1.0" type="track-model" subtype="closed-xy-profile">'
                b'<metadata><title>xml collision</title></metadata></model>'
            )
            source_named_as_output.write_bytes(source_xml_bytes)
            with self.assertRaisesRegex(ValueError, "overwrite source track asset"):
                _validate_output_paths(source_named_as_output, root)
            self.assertEqual(source_named_as_output.read_bytes(), source_xml_bytes)

    def test_run_rejects_open_track_without_width_before_solver_or_output_creation(self) -> None:
        from tools.run import solve_minimum_time_line as runner

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "open.xml"
            source.write_text(
                '<model schema-version="1.0" type="track-model" subtype="open-xy-profile">'
                '<metadata><title>open fixture</title></metadata><data ref="geometry.csv" />'
                '</model>', encoding="utf-8",
            )
            output = root / "output"
            track = SimpleNamespace(is_closed=False, has_edges=False)
            args = SimpleNamespace(track=source, output=output, margin=.1, half_width=None)
            with (
                patch.object(runner, "load_track", return_value=track),
                patch.object(runner, "solve_lap") as solve_lap,
                patch.object(runner, "solve_line") as solve_line,
                patch.object(runner, "solve_ladder") as solve_ladder,
                patch.object(runner, "compile_formulation") as compile_formulation,
                patch.object(runner, "load_capability_surface") as load_surface,
            ):
                with self.assertRaisesRegex(ValueError, "corridor or half_width"):
                    runner.run(args)
            solve_lap.assert_not_called()
            solve_line.assert_not_called()
            solve_ladder.assert_not_called()
            compile_formulation.assert_not_called()
            load_surface.assert_not_called()
            self.assertFalse(output.exists())

    def test_run_rejects_missing_corridor_before_solver_or_output_creation(self) -> None:
        from tools.run import solve_minimum_time_line as runner

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "closed.xml"
            source.write_text(
                '<model schema-version="1.0" type="track-model" subtype="closed-xy-profile">'
                '<metadata><title>no edges fixture</title></metadata><data ref="geometry.csv" />'
                '</model>', encoding="utf-8",
            )
            output = root / "output"
            track = SimpleNamespace(is_closed=True, has_edges=False)
            args = SimpleNamespace(track=source, output=output, margin=.1, half_width=None)
            with (
                patch.object(runner, "load_track", return_value=track),
                patch.object(runner, "solve_lap") as solve_lap,
                patch.object(runner, "solve_line") as solve_line,
                patch.object(runner, "solve_ladder") as solve_ladder,
                patch.object(runner, "compile_formulation") as compile_formulation,
                patch.object(runner, "load_capability_surface") as load_surface,
            ):
                with self.assertRaisesRegex(ValueError, "corridor or half_width"):
                    runner.run(args)
            solve_lap.assert_not_called()
            solve_line.assert_not_called()
            solve_ladder.assert_not_called()
            compile_formulation.assert_not_called()
            load_surface.assert_not_called()
            self.assertFalse(output.exists())

    def test_open_track_uses_standing_start_and_fallback_corridor(self) -> None:
        from tools.run import solve_minimum_time_line as runner
        from simplified_vehicle_models.results.traces import LapMode

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "open.xml"
            source.write_text(
                '<model schema-version="1.0" type="track-model" subtype="open-xy-profile">'
                '<metadata><title>open fixture</title></metadata><data ref="geometry.csv" />'
                '</model>', encoding="utf-8",
            )
            track = SimpleNamespace(
                is_closed=False, has_edges=False, width_provenance=None,
            )
            corridor = SimpleNamespace(
                distance=np.array([0., 1.]), minimum=np.array([-1.9, -1.9]),
                maximum=np.array([1.9, 1.9]), provenance="assumed", is_closed=False,
            )
            args = SimpleNamespace(
                track=source, output=root / "output", margin=.1, half_width=2.0,
                formulation=Path("formulation.xml"), surface=Path("surface.npz"),
            )
            with (
                patch.object(runner, "load_track", return_value=track),
                patch.object(runner, "corridor_from_track", return_value=corridor) as build_corridor,
                patch.object(runner, "compile_formulation", return_value=SimpleNamespace(model=object())),
                patch.object(runner, "point_mass_view_of_planar", return_value=object()),
                patch.object(runner, "load_capability_surface", return_value=SimpleNamespace(provenance_hash="hash")),
                patch.object(runner, "capability_hash", return_value="hash"),
                patch.object(runner, "solve_lap", side_effect=RuntimeError("stop")) as solve_lap,
            ):
                with self.assertRaisesRegex(RuntimeError, "stop"):
                    runner.run(args)
            build_corridor.assert_called_once_with(track, margin=.1, half_width=2.0)
            solve_lap.assert_called_once()
            self.assertIs(solve_lap.call_args.args[2], LapMode.STANDING_START)
            self.assertTrue((root / "output").is_dir())

    def test_invalid_spacings_fail_before_solver_or_output_creation(self) -> None:
        from tools.run import solve_minimum_time_line as runner

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "closed.xml"
            source.write_text(
                '<model schema-version="1.0" type="track-model" subtype="closed-xy-profile">'
                '<metadata><title>fixture</title></metadata><data ref="geometry.csv" />'
                '</model>', encoding="utf-8",
            )
            output = root / "output"
            track = SimpleNamespace(is_closed=True, has_edges=True)
            with (
                patch.object(runner, "load_track", return_value=track),
                patch.object(runner, "solve_lap") as solve_lap,
            ):
                for bad in ("5", "5,5", "1,2"):
                    with self.subTest(spacings=bad):
                        with self.assertRaises(ValueError):
                            runner.run(SimpleNamespace(
                                track=source, output=output, margin=.1,
                                half_width=None, spacings=bad,
                            ))
            solve_lap.assert_not_called()
            self.assertFalse(output.exists())

    def test_open_envelope_seed_matches_reduced_basis_layout(self) -> None:
        from tools.run.solve_minimum_time_line import _open_envelope_seed
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, _offset_basis

        distance = np.linspace(0.0, 10.0, 5)
        track = SimpleNamespace(lap_length=10.0, distance=distance, is_closed=False)
        envelope = SimpleNamespace(speed=np.linspace(0.0, 10.0, 5))
        settings = QssSettings(offset_control_spacing=5.0)
        mesh = np.linspace(0.0, 10.0, 3)
        seed = _open_envelope_seed(track, mesh, envelope, settings)
        basis, _ = _offset_basis(track, mesh, 5.0)
        self.assertEqual(seed.size, basis.shape[1] + 4 * mesh.size + 2 * (mesh.size - 1))
        self.assertTrue(np.all(np.isfinite(seed)))

    def test_mesh_spacings_are_validated(self) -> None:
        from tools.run.solve_minimum_time_line import _parse_spacings

        self.assertEqual(_parse_spacings("5,2,1"), (5.0, 2.0, 1.0))
        self.assertEqual(_parse_spacings([10, 5]), (10.0, 5.0))
        for bad in ("5", "5,5", "1,2", "5,0,1", "5,nan", "5,,1"):
            with self.subTest(spacings=bad):
                with self.assertRaises(ValueError):
                    _parse_spacings(bad)

    def test_open_launch_time_enters_gate_inputs_and_summary(self) -> None:
        from tools.run import solve_minimum_time_line as runner

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "open.xml"
            source.write_text(
                '<model schema-version="1.0" type="track-model" subtype="open-xy-profile">'
                '<metadata><title>open fixture</title></metadata><data ref="geometry.csv" />'
                '</model>', encoding="utf-8",
            )
            distance = np.array([0., 1., 2.])
            coordinates = PathCoordinates(
                distance=distance, x_north=np.zeros(3), y_east=np.zeros(3),
                heading=np.zeros(3), curvature=np.zeros(3), is_closed=False,
            )
            track = SimpleNamespace(
                is_closed=False, has_edges=False, width_provenance=None,
                lap_length=2.0, distance=distance,
            )
            corridor = SimpleNamespace(
                distance=distance, minimum=np.full(3, -2.), maximum=np.full(3, 2.),
                provenance="assumed", is_closed=False,
            )
            envelope = SimpleNamespace(lap_time=10.5, speed=np.array([0., 5., 6.]))
            zero = SimpleNamespace(lap_time=9.9, iterations=1, maximum_violation=0.0)
            coarse = SimpleNamespace(lap_time=9.84, iterations=1, maximum_violation=0.0)
            fine = SimpleNamespace(
                distance=distance, offset=np.zeros(3), heading_error=np.zeros(3),
                speed=np.full(3, 5.), acceleration=np.zeros(3),
                curvature_rate=np.zeros(3), lap_time=9.8, iterations=1,
                maximum_violation=0.0,
            )
            replay = SimpleNamespace(lap_time=10.3)
            args = SimpleNamespace(
                track=source, output=root / "output", margin=.1, half_width=2.0,
                spacings="5,2", formulation=Path("formulation.xml"),
                surface=Path("surface.npz"),
            )
            with (
                patch.object(runner, "load_track", side_effect=[track, track]),
                patch.object(runner, "corridor_from_track", return_value=corridor),
                patch.object(runner, "compile_formulation", return_value=SimpleNamespace(model=object())),
                patch.object(runner, "point_mass_view_of_planar", return_value=object()),
                patch.object(runner, "load_capability_surface", return_value=SimpleNamespace(provenance_hash="hash")),
                patch.object(runner, "capability_hash", return_value="hash"),
                patch.object(runner, "solve_lap", side_effect=[envelope, envelope, replay]),
                patch.object(runner, "launch_time", return_value=0.5),
                patch.object(runner, "capability_envelope_time", return_value=10.5),
                patch.object(runner, "solve_line", return_value=zero),
                patch.object(runner, "solve_ladder", return_value=[coarse, fine]),
                patch.object(runner, "path_from_solution", return_value=coordinates),
                patch.object(runner, "_write_rebuilt_track", return_value=(root / "rebuilt.xml", root / "geometry.csv")),
                patch.object(runner, "check_zero_corridor_matches_envelope", return_value=True) as zero_gate,
                patch.object(runner, "check_free_line_not_slower", return_value=True) as free_gate,
                patch.object(runner, "check_replay_agreement", return_value=True) as replay_gate,
            ):
                summary = runner.run(args)
            zero_gate.assert_called_once_with(open_loop=10.5, ocp=10.4, tolerance=0.01)
            free_gate.assert_called_once_with(centreline=10.5, free=10.3, tolerance=0.002)
            replay_gate.assert_called_once_with(ocp=10.3, replay=10.3, tolerance=0.01)
            self.assertEqual(summary["launch_time_s"], 0.5)
            self.assertEqual(summary["ocp_zero_corridor_including_launch_s"], 10.4)
            self.assertEqual(summary["ocp_free_line_including_launch_s"], 10.3)

    def test_run_reports_all_five_checks_and_preserves_false_gate(self) -> None:
        from tools.run import solve_minimum_time_line as runner

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            distance = np.array([0., 1., 2., 3., 4.])
            path = PathCoordinates(
                distance=distance, x_north=np.zeros(5), y_east=np.zeros(5),
                heading=np.zeros(5), curvature=np.ones(5), is_closed=True,
            )
            stage = SimpleNamespace(
                distance=distance, offset=np.zeros(5), heading_error=np.zeros(5),
                speed=np.full(5, 20.), acceleration=np.zeros(5),
                curvature_rate=np.zeros(5), lap_time=57.9, iterations=12,
                maximum_violation=0.0,
            )
            stages = [stage, stage, stage]
            track = SimpleNamespace(
                lap_length=4.0, distance=distance, width_provenance="assumed",
                is_closed=True, has_edges=True,
            )
            corridor = SimpleNamespace(
                distance=distance, minimum=np.full(5, -2.), maximum=np.full(5, 2.),
                provenance="assumed", is_closed=True,
            )
            source_track = root / "source_track.xml"
            source_track.write_text(
                '<model schema-version="1.0" type="track-model" subtype="closed-xy-profile">'
                '<metadata><title>runner fixture</title></metadata><data ref="source_geometry.csv" />'
                '</model>',
                encoding="utf-8",
            )
            args = SimpleNamespace(
                track=source_track, formulation=Path("formulation.xml"),
                surface=Path("surface.npz"), output=root / "output", margin=.1,
            )
            with (
                patch.object(runner, "corridor_from_track", return_value=corridor),
                patch.object(runner, "compile_formulation", return_value=SimpleNamespace(model=object())),
                patch.object(runner, "point_mass_view_of_planar", return_value=object()),
                patch.object(runner, "load_capability_surface", return_value=SimpleNamespace(provenance_hash="matching-hash")),
                patch.object(runner, "capability_hash", return_value="matching-hash"),
                patch.object(runner, "solve_lap", side_effect=[SimpleNamespace(lap_time=58.0), SimpleNamespace(lap_time=57.9)]),
                patch.object(runner, "solve_line", return_value=stage) as solve_zero_corridor,
                patch.object(
                    runner, "solve_ladder",
                    side_effect=lambda *_args, stage_callback=None, **_kwargs: [
                        (stage_callback(spacing, item, 0.1) if stage_callback else None) or item
                        for spacing, item in zip((5.0, 2.0, 1.0), stages)
                    ],
                ) as solve_ladder,
                patch.object(runner, "path_from_solution", return_value=path),
                patch.object(runner, "_write_rebuilt_track", return_value=(root / "rebuilt.xml", root / "geometry.csv")),
                patch.object(runner, "load_track", side_effect=[track, track, track]),
                patch.object(runner, "check_zero_corridor_matches_envelope", return_value=True),
                patch.object(runner, "check_free_line_not_slower", return_value=True),
                patch.object(runner, "check_corridor_containment", return_value=False),
                patch.object(runner, "check_mesh_convergence", return_value=True),
                patch.object(runner, "check_replay_agreement", return_value=True),
            ):
                summary = runner.run(args)

            solve_zero_corridor.assert_called_once()
            zero_corridor_arg = solve_zero_corridor.call_args.args[1]
            np.testing.assert_array_equal(zero_corridor_arg.minimum, 0.0)
            np.testing.assert_array_equal(zero_corridor_arg.maximum, 0.0)
            solve_ladder.assert_called_once()
            self.assertEqual(solve_ladder.call_args.kwargs["spacings"], (5.0, 2.0, 1.0))
            self.assertEqual(set(summary["checks"]), {
                "zero_corridor_matches_envelope", "free_line_not_slower", "corridor_containment",
                "mesh_convergence", "replay_agreement",
            })
            self.assertFalse(summary["checks"]["corridor_containment"])
            self.assertEqual(summary["stages"][0]["spacing_m"], 5.0)
            self.assertEqual(summary["stages"][-1]["spacing_m"], 1.0)
            self.assertTrue(summary["conditional_result"])
            self.assertTrue((root / "output" / "summary.json").is_file())

    def test_cli_passes_open_track_options(self) -> None:
        from tools.run import solve_minimum_time_line as runner

        with patch.object(runner, "run", return_value={"checks": {}}) as run:
            self.assertEqual(
                runner.main([
                    "--track", "track.xml", "--formulation", "formulation.xml",
                    "--surface", "surface.npz", "--output", "out",
                    "--half-width", "2.45", "--spacings", "10,5",
                    "--offset-spacing", "35", "--path-smoothing", "3",
                ]),
                0,
            )
        args = run.call_args.args[0]
        self.assertEqual(args.half_width, 2.45)
        self.assertEqual(args.spacings, "10,5")
        self.assertEqual(args.offset_spacing, 35.0)
        self.assertEqual(args.path_smoothing, 3.0)

    def test_cli_exit_codes_reflect_failed_gates_and_pipeline_errors(self) -> None:
        from tools.run import solve_minimum_time_line as runner

        failed_summary = {"checks": {"required_gate": False}}
        with patch.object(runner, "run", return_value=failed_summary):
            self.assertEqual(
                runner.main(["--track", "track.xml", "--formulation", "formulation.xml",
                             "--surface", "surface.npz", "--output", "out"]),
                1,
            )
        with patch.object(runner, "run", side_effect=RuntimeError("solver failed")):
            self.assertEqual(
                runner.main(["--track", "track.xml", "--formulation", "formulation.xml",
                             "--surface", "surface.npz", "--output", "out"]),
                2,
            )
