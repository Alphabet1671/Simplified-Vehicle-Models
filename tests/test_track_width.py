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
        data.append('<data role="width" ref="width.csv" />')
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
