import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from simplified_vehicle_models.tracks.path_coordinates import (
    evaluate_path,
    heading_modulo_two_pi,
)
from simplified_vehicle_models.tracks.profile import load_track
from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile
from tools.tracks.build_closed_track import (
    arc_length,
    close_by_linear_correction,
    integrate_rates,
    minimum_self_distance,
    periodic_gaussian,
    signed_geometry,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CLOSED_PATH = REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_closed.xml"
LEGACY_PATH = REPOSITORY_ROOT / "models/tracks/2024_michigan_endurance_v11.xml"


def _circle(
    radius: float,
    samples: int = 720,
    direction: float = 1.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return one closed circle as timestamps and two velocity channels.

    The trace covers a full turn, so the last sample returns to the start.
    A positive ``direction`` turns right, so the curvature is positive.
    """

    steps = samples + 1
    spacing = 2.0 * math.pi * radius / samples
    speed = 10.0
    timestamps = np.arange(steps, dtype=np.float64) * (spacing / speed)
    heading = direction * np.arange(steps, dtype=np.float64) * (spacing / radius)
    north_rate = speed * np.cos(heading)
    east_rate = speed * np.sin(heading)
    return timestamps, north_rate, east_rate


def _closed_nodes(
    radius: float,
    direction: float = 1.0,
    samples: int = 360,
) -> tuple[np.ndarray, np.ndarray]:
    """Return a circle sampled at uniform arc length without a repeated node.

    A negative ``direction`` mirrors the path, so the turn becomes a left turn.
    """

    spacing = 2.0 * math.pi * radius / samples
    distance = np.arange(samples, dtype=np.float64) * spacing
    angle = distance / radius
    return (
        radius * np.sin(angle),
        direction * radius * (1.0 - np.cos(angle)),
    )


def _append_closing_node(
    north: np.ndarray,
    east: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Repeat the first node, so the polyline is closed."""

    return np.append(north, north[0]), np.append(east, east[0])


class ClosedTrackBuildTests(unittest.TestCase):
    def test_integration_recovers_a_known_circle(self) -> None:
        radius = 40.0
        timestamps, north_rate, east_rate = _circle(radius)

        north, east = integrate_rates(timestamps, north_rate, east_rate)
        distance = arc_length(north, east)

        # The chord sum understates the arc by the usual sampling error.
        self.assertAlmostEqual(
            float(distance[-1]), 2.0 * math.pi * radius, delta=0.6
        )
        # The trapezoid rule on a closed circle returns to the start.
        self.assertAlmostEqual(float(north[-1]), 0.0, places=6)
        self.assertAlmostEqual(float(east[-1]), 0.0, places=6)

    def test_linear_drift_correction_closes_the_loop(self) -> None:
        north, east = _closed_nodes(40.0)
        north, east = _append_closing_node(north, east)
        # Open the loop with a known offset that grows to the far end.
        north = north + np.linspace(0.0, 3.0, north.size)
        east = east + np.linspace(0.0, -4.0, east.size)
        distance = arc_length(north, east)

        closed_north, closed_east, gap = close_by_linear_correction(
            north, east, distance
        )

        self.assertAlmostEqual(gap, 5.0, places=9)
        self.assertAlmostEqual(float(closed_north[0]), float(north[0]), places=12)
        self.assertAlmostEqual(float(closed_east[0]), float(east[0]), places=12)
        # The correction makes the last sample land on the first sample.
        self.assertAlmostEqual(
            float(closed_north[-1]), float(closed_north[0]), places=9
        )
        self.assertAlmostEqual(
            float(closed_east[-1]), float(closed_east[0]), places=9
        )

    def test_periodic_smoothing_wraps_at_the_boundary(self) -> None:
        samples = 128
        sigma = 3.0
        values = np.sin(2.0 * math.pi * np.arange(samples) / samples * 3.0)

        smoothed = periodic_gaussian(values, sigma)

        # Reproduce the wrapped convolution directly.
        half = int(math.ceil(4.0 * sigma))
        offsets = np.arange(-half, half + 1, dtype=np.float64)
        weights = np.exp(-0.5 * (offsets / sigma) ** 2)
        weights /= weights.sum()
        expected = sum(
            weight * np.roll(values, int(offset))
            for offset, weight in zip(offsets, weights)
        )
        np.testing.assert_allclose(smoothed, expected, rtol=1.0e-12, atol=1.0e-12)
        # The kernel is normalised, so a constant signal is unchanged.
        constant = np.full(64, 7.0)
        np.testing.assert_allclose(periodic_gaussian(constant, 2.5), constant)

    def test_signed_geometry_uses_the_right_turn_convention(self) -> None:
        radius = 25.0
        samples = 720
        spacing = 2.0 * math.pi * radius / samples
        distance = np.arange(samples, dtype=np.float64) * spacing
        for direction, label in ((1.0, "right"), (-1.0, "left")):
            north, east = _closed_nodes(radius, direction=direction, samples=samples)
            _, curvature = signed_geometry(distance, north, east)
            with self.subTest(turn=label):
                if direction > 0.0:
                    self.assertTrue(np.all(curvature > 0.0))
                else:
                    self.assertTrue(np.all(curvature < 0.0))
                np.testing.assert_allclose(
                    np.abs(curvature), 1.0 / radius, rtol=1.0e-3
                )

    def test_winding_of_a_closed_circle_is_one_turn(self) -> None:
        samples = 400
        for direction, expected in ((1.0, 2.0 * math.pi), (-1.0, -2.0 * math.pi)):
            radius = 30.0
            spacing = 2.0 * math.pi * radius / samples
            distance = np.arange(samples, dtype=np.float64) * spacing
            north, east = _closed_nodes(radius, direction=direction, samples=samples)
            _, curvature = signed_geometry(distance, north, east)
            with self.subTest(direction=direction):
                self.assertAlmostEqual(
                    float(np.sum(curvature) * spacing), expected, places=2
                )

    def test_minimum_self_distance_finds_a_crossing(self) -> None:
        straight_north, straight_east = _closed_nodes(40.0, samples=120)
        self.assertGreater(
            minimum_self_distance(straight_north, straight_east), 10.0
        )

        # A figure of eight crosses itself at the origin.
        parameter = np.linspace(0.0, 2.0 * math.pi, 240, endpoint=False)
        north = np.sin(parameter)
        east = np.sin(parameter) * np.cos(parameter)
        self.assertLess(minimum_self_distance(north, east), 0.2)


class ClosedTrackArtifactTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.track = load_track(CLOSED_PATH)
        cls.coordinates = cls.track.path_coordinates

    def test_carries_signed_geometry(self) -> None:
        self.assertTrue(self.track.has_signed_geometry)
        self.assertTrue(self.track.is_closed)
        self.assertIsInstance(self.coordinates, PathCoordinates)
        self.assertEqual(
            self.coordinates.distance.size, self.track.distance.size
        )

    def test_distance_and_segments_are_consistent(self) -> None:
        distance = self.track.distance

        self.assertAlmostEqual(float(distance[0]), 0.0, places=12)
        self.assertTrue(np.all(np.diff(distance) > 0.0))
        self.assertEqual(
            self.track.segment_length.size, self.track.distance.size - 1
        )
        np.testing.assert_allclose(
            self.track.segment_length, np.diff(distance), rtol=0.0, atol=1.0e-9
        )
        self.assertAlmostEqual(
            self.track.lap_length, self.coordinates.lap_length, places=9
        )

    def test_position_is_exactly_closed(self) -> None:
        distance = self.track.distance
        spacing = float(distance[1] - distance[0])

        self.assertAlmostEqual(
            float(distance[-1] - distance[-2]), spacing, places=6
        )
        np.testing.assert_allclose(
            self.coordinates.x_north[-1], self.coordinates.x_north[0], atol=1.0e-9
        )
        np.testing.assert_allclose(
            self.coordinates.y_east[-1], self.coordinates.y_east[0], atol=1.0e-9
        )

    def test_winding_is_one_full_turn(self) -> None:
        distance = self.track.distance
        curvature = self.coordinates.curvature
        winding = float(np.trapezoid(curvature, distance))

        self.assertAlmostEqual(abs(winding), 2.0 * math.pi, places=2)
        # The stored heading falls by one turn across the lap. The trapezoid
        # of a piecewise linear curvature differs from the central difference
        # heading by a small quadrature term.
        turn = float(self.coordinates.heading[-1] - self.coordinates.heading[0])
        self.assertAlmostEqual(turn, winding, delta=5.0e-3)

    def test_carries_both_turn_signs(self) -> None:
        curvature = self.coordinates.curvature

        self.assertLess(float(np.min(curvature)), 0.0)
        self.assertGreater(float(np.max(curvature)), 0.0)

    def test_radius_magnitude_is_finite_and_capped(self) -> None:
        radius = self.track.radius

        self.assertTrue(np.all(np.isfinite(radius)))
        self.assertTrue(np.all(radius > 0.0))
        self.assertLessEqual(float(np.max(radius)), 300.0 + 1.0e-9)

    def test_has_no_self_intersection(self) -> None:
        self.assertGreater(
            minimum_self_distance(
                self.coordinates.x_north, self.coordinates.y_east
            ),
            5.0,
        )

    def test_path_evaluation_is_periodic(self) -> None:
        query = np.asarray([0.0, 100.0, 250.0])
        x0, y0, h0, k0 = evaluate_path(self.coordinates, query)
        x1, y1, h1, k1 = evaluate_path(
            self.coordinates, query + self.track.lap_length
        )

        np.testing.assert_allclose(x1, x0, atol=1.0e-9)
        np.testing.assert_allclose(y1, y0, atol=1.0e-9)
        np.testing.assert_allclose(k1, k0, atol=1.0e-12)
        turn = float(self.coordinates.heading[-1] - self.coordinates.heading[0])
        np.testing.assert_allclose(h1 - h0, turn, atol=1.0e-9)

    def test_negative_distance_wraps_into_the_lap(self) -> None:
        forward = evaluate_path(self.coordinates, np.asarray([self.track.lap_length - 40.0]))
        backward = evaluate_path(self.coordinates, np.asarray([-40.0]))

        np.testing.assert_allclose(backward[0], forward[0], atol=1.0e-9)
        np.testing.assert_allclose(backward[1], forward[1], atol=1.0e-9)
        np.testing.assert_allclose(backward[3], forward[3], atol=1.0e-12)

    def test_evaluation_reproduces_the_stored_nodes(self) -> None:
        x, y, heading, curvature = evaluate_path(
            self.coordinates, self.coordinates.distance
        )

        np.testing.assert_allclose(x, self.coordinates.x_north, atol=0.0)
        np.testing.assert_allclose(y, self.coordinates.y_east, atol=0.0)
        np.testing.assert_allclose(heading, self.coordinates.heading, atol=0.0)
        np.testing.assert_allclose(curvature, self.coordinates.curvature, atol=0.0)

    def test_heading_folding_is_bounded(self) -> None:
        folded = heading_modulo_two_pi(self.coordinates.heading)

        self.assertTrue(np.all(np.abs(folded) <= math.pi + 1.0e-12))

    def test_the_legacy_track_keeps_its_unsigned_contract(self) -> None:
        legacy = load_track(LEGACY_PATH)

        self.assertFalse(legacy.has_signed_geometry)
        self.assertIsNone(legacy.path_coordinates)
        self.assertEqual(legacy.distance.size, 1407)
        self.assertAlmostEqual(legacy.lap_length, 1014.32575, places=8)
        self.assertTrue(np.all(legacy.curvature > 0.0))


class ClosedTrackLoaderRejectionTests(unittest.TestCase):
    """Reject a malformed closed centerline."""

    def _write(self, directory: Path, rows: str, parameters: str) -> Path:
        (directory / "geometry.csv").write_text(
            "distance,x_north,y_east,heading_rad,curvature\n" + rows,
            encoding="utf-8",
        )
        model = directory / "track.xml"
        model.write_text(
            '<model schema-version="1.0" id="t" type="track-model" '
            'subtype="closed-xy-profile">\n'
            "  <metadata><title>bad</title></metadata>\n"
            '  <data ref="geometry.csv"/>\n'
            "  <parameters>\n" + parameters + "  </parameters>\n"
            "</model>\n",
            encoding="utf-8",
        )
        return model

    def _valid_rows(self) -> str:
        return "0,0,0,0,0\n1,1,0,0,0\n2,0,0,0,0\n"

    def _valid_parameters(self) -> str:
        return (
            '    <parameter name="is_closed" value="true"/>\n'
            '    <parameter name="terminal_segment_is_duplicate" value="true"/>\n'
            '    <parameter name="maximum_radius" value="300" unit="m"/>\n'
        )

    def test_rejects_an_open_profile(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            parameters = self._valid_parameters().replace(
                'name="is_closed" value="true"', 'name="is_closed" value="false"'
            )
            model = self._write(Path(name), self._valid_rows(), parameters)
            with self.assertRaisesRegex(ValueError, "must be closed"):
                load_track(model)

    def test_rejects_a_missing_terminal_duplicate(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            parameters = self._valid_parameters().replace(
                'name="terminal_segment_is_duplicate" value="true"',
                'name="terminal_segment_is_duplicate" value="false"',
            )
            model = self._write(Path(name), self._valid_rows(), parameters)
            with self.assertRaisesRegex(ValueError, "must repeat the first node"):
                load_track(model)

    def test_rejects_a_missing_maximum_radius(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            parameters = self._valid_parameters().replace(
                '    <parameter name="maximum_radius" value="300" unit="m"/>\n', ""
            )
            model = self._write(Path(name), self._valid_rows(), parameters)
            with self.assertRaisesRegex(ValueError, "maximum_radius"):
                load_track(model)

    def test_rejects_a_non_monotone_distance(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            rows = "0,0,0,0,0\n1,1,0,0,0\n1,0,0,0,0\n"
            model = self._write(Path(name), rows, self._valid_parameters())
            with self.assertRaisesRegex(ValueError, "distance must increase"):
                load_track(model)

    def test_rejects_a_broken_position_closure(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            rows = "0,0,0,0,0\n1,1,0,0,0\n2,5,0,0,0\n"
            model = self._write(Path(name), rows, self._valid_parameters())
            with self.assertRaisesRegex(ValueError, "closing node"):
                load_track(model)

    def test_rejects_a_broken_curvature_closure(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            rows = "0,0,0,0,0.1\n1,1,0,0,0.1\n2,0,0,0,0.2\n"
            model = self._write(Path(name), rows, self._valid_parameters())
            with self.assertRaisesRegex(ValueError, "closing node"):
                load_track(model)

    def test_rejects_unknown_columns(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            (directory / "geometry.csv").write_text(
                "distance,x,y\n0,0,0\n", encoding="utf-8"
            )
            model = directory / "track.xml"
            model.write_text(
                '<model schema-version="1.0" id="t" type="track-model" '
                'subtype="closed-xy-profile">\n'
                "  <metadata><title>bad</title></metadata>\n"
                '  <data ref="geometry.csv"/>\n'
                "  <parameters>\n" + self._valid_parameters() + "  </parameters>\n"
                "</model>\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "geometry columns"):
                load_track(model)

    def test_unknown_subtype_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            (directory / "geometry.csv").write_text("a,b\n0,0\n", encoding="utf-8")
            model = directory / "track.xml"
            model.write_text(
                '<model schema-version="1.0" id="t" type="track-model" '
                'subtype="mystery"><metadata><title>x</title></metadata>'
                '<data ref="geometry.csv"/></model>\n',
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "unsupported track model"):
                load_track(model)


if __name__ == "__main__":
    unittest.main()
