import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from tools.run.solve_minimum_time_line import _write_rebuilt_track
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
