"""Tests for the exported legacy powertrain data.

The tests read the committed CSV files in ``models/data``.
They do not read the legacy repositories at run time, except in one test
that skips when a legacy source file is absent.
"""

from __future__ import annotations

import csv
import hashlib
import importlib.util
import math
import re
import struct
import unittest
import zlib
from pathlib import Path
from typing import Any

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DATA_DIRECTORY = REPOSITORY_ROOT / "models" / "data"

TORQUE_MAP_PATH = DATA_DIRECTORY / "amk_dd5_100c_torque_map.csv"
LOSS_MAP_PATH = DATA_DIRECTORY / "amk_dd5_100c_loss_map.csv"
IDENTIFICATION_PATH = DATA_DIRECTORY / "amk_dd5_100c_identification.csv"
CELL_CURVE_PATH = DATA_DIRECTORY / "cosmx_cell_curves.csv"
PROVENANCE_PATH = DATA_DIRECTORY / "provenance.md"

# The legacy sources are read-only. The digest test reads them when they
# are present, and skips when they are absent.
MOTOR_SOURCE = Path(
    "/Users/alphabet1671/Dev/lap-sim-2025/Models/Legacy Models/"
    "Drivetrain Models/AMK_Characteristic_100C.mat"
)
MOTOR_COPY = Path(
    "/Users/alphabet1671/Dev/lap-sim/EV_Points_Sim/Input Files/"
    "Latest Input Files/AMK_Characteristic_100C.mat"
)
CELL_SOURCE = Path(
    "/Users/alphabet1671/Dev/lap-sim/EV_Points_Sim/Input Files/"
    "Battery Modelling/cosmx_cell_data_curves.csv"
)

MOTOR_SHA256 = "1bff99223cb4537245350e6b15f2a669d2da85ac44154453146ab00f252802fd"
MOTOR_BYTES = 488025
CELL_SHA256 = "4f9cf03ecb298c2e153cfadd7b57ff83f8e4a9f4f27559ea9ec8f508d7c47314"
CELL_BYTES = 732

SPEED_COUNT = 201
TORQUE_COUNT = 21
GRID_POINT_COUNT = SPEED_COUNT * TORQUE_COUNT
CELL_ROW_COUNT = 18


def read_commented_csv(path: Path) -> list[dict[str, str]]:
    """Read a CSV file that starts with comment lines."""
    lines = [
        line
        for line in path.read_text(encoding="utf-8").splitlines()
        if line and not line.startswith("#")
    ]
    return list(csv.DictReader(lines))


def read_column(path: Path, name: str) -> np.ndarray:
    """Read one numeric column from a commented CSV file."""
    rows = read_commented_csv(path)
    return np.array([float(row[name]) for row in rows], dtype=float)


def read_provenance() -> str:
    """Read the provenance text."""
    return PROVENANCE_PATH.read_text(encoding="utf-8")


def calculate_sha256(path: Path) -> str:
    """Return the SHA-256 digest of one file."""
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def derive_phase_peak_voltage(columns: dict[str, np.ndarray]) -> np.ndarray:
    """Return the phase peak voltage from the d/q peak voltages.

    The map follows the space-vector relation
    ``voltage_phase_peak = hypot(vd_peak, vq_peak)``.
    """
    return np.hypot(columns["vd_peak"], columns["vq_peak"])


def derive_line_rms_voltage(columns: dict[str, np.ndarray]) -> np.ndarray:
    """Return the line RMS voltage from the d/q peak voltages.

    The map follows the space-vector relation
    ``voltage_line_rms = voltage_phase_peak / sqrt(2)``.
    """
    return derive_phase_peak_voltage(columns) / math.sqrt(2.0)


class TorqueMapTests(unittest.TestCase):
    """Check the torque envelope file."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.rows = read_commented_csv(TORQUE_MAP_PATH)
        cls.speed_rpm = read_column(TORQUE_MAP_PATH, "speed_rpm")
        cls.maximum_torque = read_column(TORQUE_MAP_PATH, "maximum_shaft_torque")

    def test_columns(self) -> None:
        self.assertEqual(
            list(self.rows[0].keys()),
            ["speed_rpm", "maximum_shaft_torque"],
        )

    def test_grid_shape(self) -> None:
        self.assertEqual(self.speed_rpm.size, SPEED_COUNT)
        self.assertEqual(self.maximum_torque.size, SPEED_COUNT)

    def test_speed_range(self) -> None:
        self.assertAlmostEqual(float(self.speed_rpm[0]), 1.0, places=6)
        self.assertAlmostEqual(float(self.speed_rpm[-1]), 20000.0, places=6)
        self.assertTrue(np.all(np.diff(self.speed_rpm) > 0.0))
        self.assertEqual(np.unique(self.speed_rpm).size, SPEED_COUNT)

    def test_peak_shaft_torque(self) -> None:
        self.assertAlmostEqual(
            float(np.max(self.maximum_torque)),
            21.834945614425717,
            places=12,
        )


class LossMapTests(unittest.TestCase):
    """Check the total loss map file."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.rows = read_commented_csv(LOSS_MAP_PATH)
        cls.speed_rpm = read_column(LOSS_MAP_PATH, "speed_rpm")
        cls.shaft_torque = read_column(LOSS_MAP_PATH, "shaft_torque")
        cls.total_loss = read_column(LOSS_MAP_PATH, "total_loss")

    def test_columns(self) -> None:
        self.assertEqual(
            list(self.rows[0].keys()),
            ["speed_rpm", "shaft_torque", "total_loss"],
        )

    def test_grid_shape(self) -> None:
        self.assertEqual(self.speed_rpm.size, GRID_POINT_COUNT)
        self.assertEqual(np.unique(self.speed_rpm).size, SPEED_COUNT)
        speeds, counts = np.unique(self.speed_rpm, return_counts=True)
        self.assertTrue(np.all(counts == TORQUE_COUNT))

    def test_grid_matches_the_torque_map(self) -> None:
        torque_speed = read_column(TORQUE_MAP_PATH, "speed_rpm")
        self.assertTrue(np.allclose(np.unique(self.speed_rpm), torque_speed))

    def test_maximum_total_loss(self) -> None:
        self.assertAlmostEqual(float(np.max(self.total_loss)), 6205.05, places=2)


class IdentificationTests(unittest.TestCase):
    """Check the identification file and its anchors."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.rows = read_commented_csv(IDENTIFICATION_PATH)
        cls.columns = {
            name: read_column(IDENTIFICATION_PATH, name)
            for name in cls.rows[0]
        }

    def test_columns(self) -> None:
        self.assertEqual(
            list(self.rows[0].keys()),
            [
                "speed_rpm",
                "shaft_torque",
                "id_peak",
                "iq_peak",
                "vd_peak",
                "vq_peak",
                "stator_copper_loss",
                "iron_loss",
                "magnet_loss",
                "mechanical_loss",
                "total_loss",
                "input_power",
            ],
        )

    def test_grid_shape(self) -> None:
        self.assertEqual(self.columns["speed_rpm"].size, GRID_POINT_COUNT)
        self.assertEqual(
            self.columns["speed_rpm"].size,
            read_column(LOSS_MAP_PATH, "speed_rpm").size,
        )

    def test_grid_matches_the_loss_map(self) -> None:
        self.assertTrue(
            np.allclose(
                self.columns["speed_rpm"],
                read_column(LOSS_MAP_PATH, "speed_rpm"),
            )
        )
        self.assertTrue(
            np.allclose(
                self.columns["shaft_torque"],
                read_column(LOSS_MAP_PATH, "shaft_torque"),
            )
        )

    def test_maximum_voltage_line_rms(self) -> None:
        maximum = float(derive_line_rms_voltage(self.columns).max())
        # Use a relative tolerance, so the check keeps a wide margin.
        self.assertLess(abs(maximum - 424.2641) / 424.2641, 1.0e-6)
        self.assertLess(abs(maximum - 600.0 / math.sqrt(2.0)), 0.01)

    def test_maximum_voltage_phase_peak(self) -> None:
        maximum = float(derive_phase_peak_voltage(self.columns).max())
        self.assertLess(abs(maximum - 600.0) / 600.0, 1.0e-8)
        self.assertLess(abs(maximum - 600.0), 0.01)

    def test_maximum_iq_peak(self) -> None:
        self.assertAlmostEqual(
            float(self.columns["iq_peak"].max()),
            83.813,
            places=3,
        )

    def test_id_peak_range(self) -> None:
        minimum = float(self.columns["id_peak"].min())
        maximum = float(self.columns["id_peak"].max())
        self.assertAlmostEqual(minimum, -43.306, places=3)
        self.assertAlmostEqual(maximum, 0.0, delta=1.0e-9)

    def test_loss_component_anchors(self) -> None:
        self.assertAlmostEqual(
            float(self.columns["stator_copper_loss"].max()),
            2831.85,
            places=2,
        )
        self.assertAlmostEqual(
            float(self.columns["iron_loss"].max()),
            2843.02,
            places=2,
        )
        self.assertAlmostEqual(
            float(self.columns["mechanical_loss"].max()),
            166.67,
            places=2,
        )
        self.assertAlmostEqual(
            float(self.columns["total_loss"].max()),
            6205.05,
            places=2,
        )

    def test_total_loss_is_the_sum_of_the_components(self) -> None:
        component_sum = (
            self.columns["stator_copper_loss"]
            + self.columns["iron_loss"]
            + self.columns["magnet_loss"]
            + self.columns["mechanical_loss"]
        )
        self.assertTrue(np.allclose(component_sum, self.columns["total_loss"]))

    def test_input_power_relation(self) -> None:
        # The map covers the motor loss only.
        # The inverter efficiency applies on top of the map.
        angular_speed = self.columns["speed_rpm"] * 2.0 * math.pi / 60.0
        shaft_power = self.columns["shaft_torque"] * angular_speed
        motor_input = shaft_power + self.columns["total_loss"]
        mask = np.abs(motor_input) > 1.0e-9
        relative_error = np.abs(
            self.columns["input_power"][mask] - motor_input[mask]
        ) / np.abs(motor_input[mask])
        self.assertLess(float(np.median(relative_error)), 1.0e-6)
        inverter_relative_error = np.abs(
            self.columns["input_power"][mask] - motor_input[mask] / 0.95
        ) / np.abs(motor_input[mask] / 0.95)
        self.assertAlmostEqual(
            float(np.median(inverter_relative_error)),
            0.05,
            places=4,
        )


class CellCurveTests(unittest.TestCase):
    """Check the cell curve file and its anchors."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.rows = read_commented_csv(CELL_CURVE_PATH)
        cls.state_of_charge = read_column(CELL_CURVE_PATH, "state_of_charge")
        cls.voltage = read_column(CELL_CURVE_PATH, "open_circuit_voltage")
        cls.resistance = read_column(CELL_CURVE_PATH, "internal_resistance")
        cls.energy = read_column(CELL_CURVE_PATH, "cell_energy")

    def test_columns(self) -> None:
        self.assertEqual(
            list(self.rows[0].keys()),
            [
                "state_of_charge",
                "open_circuit_voltage",
                "internal_resistance",
                "cell_energy",
            ],
        )

    def test_row_count(self) -> None:
        self.assertEqual(len(self.rows), CELL_ROW_COUNT)
        self.assertEqual(self.state_of_charge.size, CELL_ROW_COUNT)

    def test_voltage_range(self) -> None:
        self.assertAlmostEqual(float(self.voltage[0]), 4.227, places=4)
        self.assertAlmostEqual(float(self.voltage[-1]), 3.117, places=4)

    def test_state_of_charge_range(self) -> None:
        self.assertAlmostEqual(float(self.state_of_charge[0]), 1.0, places=9)
        self.assertAlmostEqual(float(self.state_of_charge[-1]), 0.0, places=9)
        self.assertTrue(np.all(np.diff(self.state_of_charge) < 0.0))

    def test_energy_at_full_charge(self) -> None:
        self.assertAlmostEqual(float(self.energy[0]), 50.7606, places=4)
        self.assertAlmostEqual(float(self.energy[-1]), 0.0, places=9)

    def test_internal_resistance_range(self) -> None:
        self.assertAlmostEqual(float(self.resistance.min()), 0.00192, places=5)
        self.assertAlmostEqual(float(self.resistance.max()), 0.01001, places=5)

    def test_derived_capacity(self) -> None:
        # Integrate sum(delta_energy / midpoint_voltage) over the curve.
        # The energy is in Wh, so the result is in A*h.
        delta_energy = self.energy[:-1] - self.energy[1:]
        midpoint_voltage = 0.5 * (self.voltage[:-1] + self.voltage[1:])
        capacity = float(np.sum(delta_energy / midpoint_voltage))
        self.assertAlmostEqual(capacity, 14.146, delta=14.146 * 0.001)


class ProvenanceTests(unittest.TestCase):
    """Check the provenance record."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.text = read_provenance()

    def test_records_every_source_path(self) -> None:
        for path in (MOTOR_SOURCE, MOTOR_COPY, CELL_SOURCE):
            self.assertIn(str(path), self.text)

    def test_records_every_digest(self) -> None:
        self.assertIn(MOTOR_SHA256, self.text)
        self.assertIn(CELL_SHA256, self.text)

    def test_records_every_size(self) -> None:
        self.assertIn(f"{MOTOR_BYTES} bytes", self.text)
        self.assertIn(f"{CELL_BYTES} bytes", self.text)

    def test_records_the_inverter_efficiency(self) -> None:
        match = re.search(r"Inverter efficiency \| ([0-9.]+)", self.text)
        self.assertIsNotNone(match, "provenance.md records no inverter efficiency")
        self.assertAlmostEqual(float(match.group(1)), 0.95, places=9)

    def test_records_the_map_ratio(self) -> None:
        match = re.search(r"Ratio inside the map \| ([0-9.]+)", self.text)
        self.assertIsNotNone(match, "provenance.md records no map ratio")
        self.assertAlmostEqual(float(match.group(1)), 11.9, places=9)
        self.assertIn("10.9", self.text)

    def test_records_the_loss_scale_conclusion(self) -> None:
        self.assertIn("1.60", self.text)
        self.assertIn("motorLossCorrectionFactor = 1.60", self.text)


class SourceDigestTests(unittest.TestCase):
    """Recompute the source digests so a source change is detected."""

    def test_legacy_source_digests_match_the_recorded_values(self) -> None:
        missing = [
            path for path in (MOTOR_SOURCE, CELL_SOURCE) if not path.exists()
        ]
        if missing:
            self.skipTest(
                "legacy source file is absent: " + ", ".join(str(p) for p in missing)
            )
        text = read_provenance()
        for path, expected_digest, expected_size in (
            (MOTOR_SOURCE, MOTOR_SHA256, MOTOR_BYTES),
            (CELL_SOURCE, CELL_SHA256, CELL_BYTES),
        ):
            with self.subTest(path=str(path)):
                self.assertIn(expected_digest, text)
                self.assertEqual(calculate_sha256(path), expected_digest)
                self.assertEqual(path.stat().st_size, expected_size)


class InMemoryPath:
    """Stand in for a Path in the reader tests."""

    def __init__(self, content: bytes) -> None:
        self.content = content

    def read_bytes(self) -> bytes:
        """Return the stored content."""
        return self.content

    def __str__(self) -> str:
        return "<in-memory MAT-file>"


def load_exporter_module() -> Any:
    """Load the exporter module from the tools directory."""
    specification = importlib.util.spec_from_file_location(
        "export_legacy_powertrain_data",
        REPOSITORY_ROOT / "tools" / "export_legacy_powertrain_data.py",
    )
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def build_mat_header() -> bytes:
    """Build the 128-byte MATLAB v5 header."""
    header = bytearray(128)
    header[0 : len(b"MATLAB 5.0 MAT-file")] = b"MATLAB 5.0 MAT-file"
    struct.pack_into("<H", header, 124, 256)
    header[126:128] = b"IM"
    return bytes(header)


def build_matrix_element(name: str = "a") -> bytes:
    """Build one minimal miMATRIX element that holds a 1x1 double."""
    flags = struct.pack("<II", 6, 8) + struct.pack("<I", 6) + b"\x00" * 4
    dims = struct.pack("<II", 5, 8) + struct.pack("<ii", 1, 1)
    name_payload = name.encode("ascii") + b"\x00" * (8 - len(name))
    name_element = struct.pack("<II", 1, len(name)) + name_payload
    data = struct.pack("<II", 9, 8) + struct.pack("<d", 1.0)
    body = flags + dims + name_element + data
    return struct.pack("<II", 14, len(body)) + body


def build_mat_file(payload: bytes) -> bytes:
    """Build a compressed MATLAB v5 file from a decompressed payload."""
    compressed = zlib.compress(payload)
    return build_mat_header() + struct.pack("<II", 15, len(compressed)) + compressed


class ReaderTests(unittest.TestCase):
    """Check that the MATLAB v5 reader rejects a damaged file clearly."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.exporter = load_exporter_module()

    def test_rejects_a_header_with_no_data_element(self) -> None:
        with self.assertRaises(ValueError) as context:
            self.exporter.load_mat_v5(InMemoryPath(build_mat_header()))
        self.assertIn("no data element", str(context.exception))

    def test_rejects_a_file_with_no_header(self) -> None:
        with self.assertRaises(ValueError) as context:
            self.exporter.load_mat_v5(InMemoryPath(b"not a MAT-file"))
        self.assertIn("not a MATLAB 5.0 MAT-file", str(context.exception))

    def test_rejects_a_truncated_data_element(self) -> None:
        content = build_mat_header() + struct.pack("<II", 15, 4096)
        with self.assertRaises(ValueError) as context:
            self.exporter.load_mat_v5(InMemoryPath(content))
        self.assertIn("runs past the end of the buffer", str(context.exception))

    def test_reads_a_minimal_compressed_matrix(self) -> None:
        variables = self.exporter.load_mat_v5(
            InMemoryPath(build_mat_file(build_matrix_element("speed")))
        )
        self.assertEqual(list(variables), ["speed"])
        self.assertEqual(variables["speed"]["shape"], (1, 1))
        self.assertAlmostEqual(float(variables["speed"]["value"][0, 0]), 1.0)

    def test_rejects_a_non_matrix_element_after_a_matrix(self) -> None:
        # Trailing garbage after a valid matrix must raise, not pass.
        payload = build_matrix_element() + struct.pack("<II", 1, 0)
        with self.assertRaises(ValueError) as context:
            self.exporter.load_mat_v5(InMemoryPath(build_mat_file(payload)))
        self.assertIn("not a matrix element", str(context.exception))


if __name__ == "__main__":
    unittest.main()
