import csv
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from simplified_vehicle_models.tracks.profile import load_track


def _write_circle_track(
    directory: Path,
    *,
    radius: float = 25.0,
    nodes: int = 240,
    direction: float = 1.0,
    half_width: float | None = None,
    width_rows: int | None = None,
    provenance: str | None = "assumed",
    side: str = "both",
    width_first: bool = False,
) -> Path:
    """Write a closed circular track with an optional width source.

    ``direction`` is +1 for a right-turning circle (positive curvature) and -1
    for a left-turning one.
    """

    distance = np.linspace(0.0, 2.0 * math.pi * radius, nodes)
    angle = direction * distance / radius
    north = radius * np.sin(angle)
    east = radius * (1.0 - np.cos(angle))
    heading = angle.copy()
    curvature = np.full(nodes, direction / radius)

    with (directory / "centreline.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["distance", "x_north", "y_east", "heading_rad", "curvature"])
        for row in zip(distance, north, east, heading, curvature):
            writer.writerow([f"{value:.17g}" for value in row])

    parameters = [
        '<parameter name="is_closed" value="true" />',
        '<parameter name="terminal_segment_is_duplicate" value="true" />',
        '<parameter name="maximum_radius" value="300" unit="m" />',
    ]
    data = ['<data ref="centreline.csv" />']
    if half_width is not None:
        parameters.append(f'<parameter name="half_width" value="{half_width}" unit="m" />')
        parameters.append(
            f'<parameter name="width_provenance" value="{provenance or ""}" />'
        )
    if width_rows is not None:
        with (directory / "width.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["distance", "half_width_left", "half_width_right"])
            for index in range(width_rows):
                left = 3.0 if side in ("both", "left") else 1.0
                right = 3.0 if side in ("both", "right") else 1.0
                writer.writerow([f"{distance[index]:.17g}", f"{left}", f"{right}"])
        width_data = '<data role="width" ref="width.csv" />'
        if width_first:
            data.insert(0, width_data)
        else:
            data.append(width_data)
        parameters.append('<parameter name="width_provenance" value="surveyed" />')

    xml = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<model schema-version="1.0" id="circle" type="track-model" '
        'subtype="closed-xy-profile">\n'
        "  <metadata><title>Synthetic circle</title></metadata>\n"
        f"  {' '.join(data)}\n"
        "  <parameters>\n"
        f"    {' '.join(parameters)}\n"
        "  </parameters>\n"
        "</model>\n"
    )
    path = directory / "track.xml"
    path.write_text(xml, encoding="utf-8")
    return path


class WidthParameterTest(unittest.TestCase):
    def setUp(self) -> None:
        self._directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._directory.name)

    def tearDown(self) -> None:
        self._directory.cleanup()

    def test_constant_width_is_broadcast_to_the_whole_grid(self) -> None:
        track = load_track(_write_circle_track(self.directory, half_width=2.25))
        self.assertTrue(track.has_edges)
        self.assertEqual(track.width_provenance, "assumed")
        np.testing.assert_allclose(track.half_width_right, 2.25)
        np.testing.assert_allclose(track.half_width_left, 2.25)
        self.assertEqual(track.half_width_right.shape, track.distance.shape)

    def test_absent_width_leaves_edges_undefined(self) -> None:
        track = load_track(_write_circle_track(self.directory))
        self.assertFalse(track.has_edges)
        self.assertIsNone(track.half_width_right)
        self.assertIsNone(track.width_provenance)

    def test_width_data_role_is_read_per_node(self) -> None:
        track = load_track(
            _write_circle_track(self.directory, width_rows=240, side="left")
        )
        self.assertEqual(track.width_provenance, "surveyed")
        np.testing.assert_allclose(track.half_width_left, 3.0)
        np.testing.assert_allclose(track.half_width_right, 1.0)

    def test_width_role_before_geometry_still_loads_geometry(self) -> None:
        track = load_track(
            _write_circle_track(self.directory, width_rows=240, width_first=True)
        )
        self.assertTrue(track.has_signed_geometry)
        self.assertEqual(track.width_provenance, "surveyed")

    def test_width_data_row_count_must_match_the_grid(self) -> None:
        with self.assertRaises(ValueError):
            load_track(_write_circle_track(self.directory, width_rows=100))

    def test_missing_provenance_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            load_track(
                _write_circle_track(self.directory, half_width=2.25, provenance=None)
            )

    def test_unknown_provenance_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            load_track(
                _write_circle_track(self.directory, half_width=2.25, provenance="guessed")
            )

    def test_nonpositive_half_width_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            load_track(_write_circle_track(self.directory, half_width=0.0))

from tools.tracks.build_width_profile import write_width_profile


class WidthProfileToolTest(unittest.TestCase):
    def setUp(self) -> None:
        self._directory = tempfile.TemporaryDirectory()
        self.directory = Path(self._directory.name)

    def tearDown(self) -> None:
        self._directory.cleanup()

    def test_symmetric_width_writes_the_expected_header_and_rows(self) -> None:
        source = load_track(_write_circle_track(self.directory, half_width=2.0))
        destination = self.directory / "written.csv"
        write_width_profile(source, destination, half_width=2.25, provenance="assumed")
        lines = destination.read_text(encoding="utf-8").splitlines()
        self.assertEqual(lines[0], "distance,half_width_left,half_width_right")
        self.assertEqual(len(lines), source.distance.size + 1)
        left, right = (float(value) for value in lines[1].split(",")[1:])
        self.assertAlmostEqual(left, 2.25)
        self.assertAlmostEqual(right, 2.25)

    def test_asymmetric_width_is_written_per_side(self) -> None:
        source = load_track(_write_circle_track(self.directory, half_width=2.0))
        destination = self.directory / "asymmetric.csv"
        write_width_profile(
            source,
            destination,
            half_width_left=2.0,
            half_width_right=3.0,
            provenance="assumed",
        )
        first = destination.read_text(encoding="utf-8").splitlines()[1].split(",")
        self.assertAlmostEqual(float(first[1]), 2.0)
        self.assertAlmostEqual(float(first[2]), 3.0)

    def test_both_a_half_width_and_per_side_widths_is_rejected(self) -> None:
        source = load_track(_write_circle_track(self.directory, half_width=2.0))
        with self.assertRaises(ValueError):
            write_width_profile(
                source,
                self.directory / "bad.csv",
                half_width=2.0,
                half_width_left=2.0,
                provenance="assumed",
            )

    def test_unknown_provenance_is_rejected(self) -> None:
        source = load_track(_write_circle_track(self.directory, half_width=2.0))
        with self.assertRaises(ValueError):
            write_width_profile(
                source, self.directory / "bad.csv", half_width=2.0, provenance="guessed"
            )

    def test_the_written_file_loads_back_through_the_track_loader(self) -> None:
        source = load_track(_write_circle_track(self.directory, half_width=2.0))
        write_width_profile(
            source, self.directory / "width.csv", half_width=1.75, provenance="assumed"
        )
        track_dir = self.directory
        xml = (
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            '<model schema-version="1.0" id="round-trip" type="track-model" '
            'subtype="closed-xy-profile">\n'
            '  <metadata><title>Round trip</title></metadata>\n'
            '  <data ref="centreline.csv" />\n'
            '  <data role="width" ref="width.csv" />\n'
            '  <parameters>\n'
            '    <parameter name="is_closed" value="true" />\n'
            '    <parameter name="terminal_segment_is_duplicate" value="true" />\n'
            '    <parameter name="maximum_radius" value="300" unit="m" />\n'
            '    <parameter name="width_provenance" value="assumed" />\n'
            '  </parameters>\n'
            '</model>\n'
        )
        (track_dir / "round_trip.xml").write_text(xml, encoding="utf-8")
        restored = load_track(track_dir / "round_trip.xml")
        np.testing.assert_allclose(restored.half_width_left, 1.75)
        np.testing.assert_allclose(restored.half_width_right, 1.75)
