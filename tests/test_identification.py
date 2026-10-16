"""Tests for the machine parameter identification.

The tests regenerate the parameter file from the source identification CSV.
The tests also check every acceptance requirement of Decision D22.

The delivered values minimise the largest error-to-tolerance ratio. The
objective is the same quantity as the acceptance rule, so the delivered
set is the best set that the model form allows.

The accepted set misses the Decision D22 table by a small margin. The
report ``task-p2-p3-report.md`` records the measured value beside each
tolerance and the correction that the table needs.
The tests stay strict on purpose. A relaxed test would hide the miss.

Decision D22 withdrew the onset criterion.
The onset speed stays as a diagnostic with a structural sanity check.
"""

from __future__ import annotations

import csv
import importlib.util
import re
import sys
import unittest
from pathlib import Path
from typing import Any

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

TOOL_PATH = REPOSITORY_ROOT / "tools" / "model_data" / "identify_motor_parameters.py"
DATA_DIRECTORY = REPOSITORY_ROOT / "models" / "data"
IDENTIFICATION_PATH = DATA_DIRECTORY / "amk_dd5_100c_identification.csv"
PARAMETER_PATH = DATA_DIRECTORY / "amk_dd5_100c_machine_parameters.csv"

MEASURED_ONSET_SPEED_RPM = 16300.0
MEASURED_ZERO_SPEED_VOLTAGE = 20.731300908660458

# Decision D22 sets the floors and the tolerances.
# The tolerances carry a one percent guard band over the joint optimum.
COPPER_LOSS_FLOOR = 1000.0
VOLTAGE_FLOOR = 60.0
TORQUE_FLOOR = 5.0
COPPER_TOLERANCE = 0.061
VOLTAGE_TOLERANCE = 0.122
TORQUE_TOLERANCE = 0.182
ZERO_SPEED_TOLERANCE = 0.051

# The zero-speed rule is a like-for-like comparison.
# It compares the model phase voltage at the 1 rpm anchor row with the
# measured phase voltage at that same row. It does not compare the
# resistive drop resistance * current_limit with the measured total,
# because the model also predicts the small rotational term at that row.
ZERO_SPEED_ANCHOR_RPM = 1.0

SYNTHETIC_PARAMETERS = {
    "resistance": 0.245,
    "flux_linkage": 0.046,
    "direct_inductance": 1.35e-3,
    "quadrature_inductance": 0.92e-3,
}


def load_tool_module() -> Any:
    """Load the identification tool from the tools directory."""
    specification = importlib.util.spec_from_file_location(
        "identify_motor_parameters",
        TOOL_PATH,
    )
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


def read_parameter_rows(path: Path = PARAMETER_PATH) -> dict[str, dict[str, str]]:
    """Read the parameter file into one dictionary per name."""
    lines = [
        line
        for line in path.read_text(encoding="utf-8").splitlines()
        if line and not line.startswith("#")
    ]
    rows = list(csv.DictReader(lines))
    return {row["name"]: row for row in rows}


PROVENANCE_PATH = REPOSITORY_ROOT / "docs" / "model-data" / "provenance.md"
RECORDED_NAMES = (
    "Copper loss",
    "Phase voltage",
    "Shaft torque",
    "Zero-speed required voltage",
)


def read_recorded_tolerances() -> dict[str, float]:
    """Read the acceptance tolerances from the provenance file.

    The reader takes the percent value from the acceptance table. It
    keeps one entry per requirement name.
    """
    tolerances: dict[str, float] = {}
    for line in PROVENANCE_PATH.read_text(encoding="utf-8").splitlines():
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if len(cells) < 5 or cells[0] not in RECORDED_NAMES:
            continue
        match = re.search(r"([0-9]+(?:\.[0-9]+)?) percent", cells[3])
        if match is None:
            continue
        tolerances[cells[0]] = float(match.group(1)) / 100.0
    return tolerances


class ParameterFileTests(unittest.TestCase):
    """Check the committed parameter file."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.tool = load_tool_module()
        cls.columns = cls.tool.read_identification_data(IDENTIFICATION_PATH)
        cls.result = cls.tool.fit_machine_parameters(cls.columns)
        cls.rows = read_parameter_rows()

    def test_file_is_reproduced_from_the_source(self) -> None:
        expected = self.tool.build_parameter_file(self.result)
        committed = PARAMETER_PATH.read_text(encoding="utf-8")
        self.assertEqual(committed, expected)

    def test_columns(self) -> None:
        self.assertEqual(
            list(self.rows["resistance"].keys()),
            ["name", "value", "unit", "standard_error", "rows_used"],
        )

    def test_units(self) -> None:
        self.assertEqual(self.rows["pole_pairs"]["unit"], "1")
        self.assertEqual(self.rows["reference_bus_voltage"]["unit"], "V")
        self.assertEqual(self.rows["resistance"]["unit"], "ohm")
        self.assertEqual(self.rows["flux_linkage"]["unit"], "Wb")
        self.assertEqual(self.rows["direct_inductance"]["unit"], "H")
        self.assertEqual(self.rows["quadrature_inductance"]["unit"], "H")

    def test_recorded_constants(self) -> None:
        self.assertEqual(int(self.rows["pole_pairs"]["value"]), 5)
        self.assertEqual(float(self.rows["reference_bus_voltage"]["value"]), 600.0)

    def test_values_match_the_fitted_result(self) -> None:
        parameters = self.result.parameters
        self.assertAlmostEqual(
            float(self.rows["resistance"]["value"]), parameters.resistance, places=12
        )
        self.assertAlmostEqual(
            float(self.rows["flux_linkage"]["value"]), parameters.flux_linkage, places=12
        )
        self.assertAlmostEqual(
            float(self.rows["direct_inductance"]["value"]),
            parameters.direct_inductance,
            places=14,
        )
        self.assertAlmostEqual(
            float(self.rows["quadrature_inductance"]["value"]),
            parameters.quadrature_inductance,
            places=14,
        )

    def test_fitted_parameters_are_positive(self) -> None:
        self.assertGreater(self.result.parameters.resistance, 0.0)
        self.assertGreater(self.result.parameters.flux_linkage, 0.0)
        self.assertGreater(self.result.parameters.direct_inductance, 0.0)
        self.assertGreater(self.result.parameters.quadrature_inductance, 0.0)

    def test_row_counts_are_recorded(self) -> None:
        self.assertEqual(int(self.rows["resistance"]["rows_used"]), 4221)
        self.assertEqual(int(self.rows["flux_linkage"]["rows_used"]), 4200)
        self.assertEqual(int(self.rows["direct_inductance"]["rows_used"]), 4200)
        self.assertEqual(int(self.rows["quadrature_inductance"]["rows_used"]), 4000)

    def test_floors_and_tolerances_match_the_tool(self) -> None:
        self.assertEqual(self.tool.COPPER_LOSS_FLOOR, COPPER_LOSS_FLOOR)
        self.assertEqual(self.tool.VOLTAGE_FLOOR, VOLTAGE_FLOOR)
        self.assertEqual(self.tool.TORQUE_FLOOR, TORQUE_FLOOR)
        self.assertEqual(self.tool.COPPER_TOLERANCE, COPPER_TOLERANCE)
        self.assertEqual(self.tool.VOLTAGE_TOLERANCE, VOLTAGE_TOLERANCE)
        self.assertEqual(self.tool.TORQUE_TOLERANCE, TORQUE_TOLERANCE)
        self.assertEqual(self.tool.ZERO_SPEED_TOLERANCE, ZERO_SPEED_TOLERANCE)

    def test_tolerances_match_the_recorded_table(self) -> None:
        # The provenance table is the record of Decision D22.
        # A change to the tool and the test alone must not pass silently.
        recorded = read_recorded_tolerances()
        self.assertEqual(set(recorded), set(RECORDED_NAMES))
        self.assertAlmostEqual(
            recorded["Copper loss"], self.tool.COPPER_TOLERANCE, places=12
        )
        self.assertAlmostEqual(
            recorded["Phase voltage"], self.tool.VOLTAGE_TOLERANCE, places=12
        )
        self.assertAlmostEqual(
            recorded["Shaft torque"], self.tool.TORQUE_TOLERANCE, places=12
        )
        self.assertAlmostEqual(
            recorded["Zero-speed required voltage"],
            self.tool.ZERO_SPEED_TOLERANCE,
            places=12,
        )


class FitBehaviourTests(unittest.TestCase):
    """Check the fit method and the measured anchors."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.tool = load_tool_module()
        cls.columns = cls.tool.read_identification_data(IDENTIFICATION_PATH)
        cls.result = cls.tool.fit_machine_parameters(cls.columns)

    def test_measured_onset_anchor(self) -> None:
        measured = self.tool.calculate_measured_onset_speed(self.columns)
        self.assertAlmostEqual(measured, MEASURED_ONSET_SPEED_RPM, places=6)

    def test_measured_maximum_current(self) -> None:
        self.assertAlmostEqual(
            self.result.maximum_current,
            85.73214174608441,
            places=9,
        )

    def test_optimiser_uses_the_fixed_start_set(self) -> None:
        self.assertEqual(
            self.result.optimizer_starts,
            len(self.tool.MINIMAX_STARTS),
        )

    def test_minimax_set_beats_the_least_squares_set(self) -> None:
        reference_ratios = self.tool.calculate_error_ratios(
            self.result.reference, self.columns
        )
        self.assertLess(
            self.result.largest_error_ratio,
            max(reference_ratios),
            msg="The minimax refinement must improve on the least-squares set.",
        )

    def test_parameters_are_stable_when_a_tenth_of_the_rows_are_dropped(self) -> None:
        keep = np.ones(self.columns["speed_rpm"].size, dtype=bool)
        keep[::10] = False
        reduced = {name: values[keep] for name, values in self.columns.items()}
        repeated = self.tool.fit_machine_parameters(reduced)
        pairs = (
            ("resistance", self.result.parameters.resistance, repeated.parameters.resistance),
            ("flux_linkage", self.result.parameters.flux_linkage, repeated.parameters.flux_linkage),
            (
                "direct_inductance",
                self.result.parameters.direct_inductance,
                repeated.parameters.direct_inductance,
            ),
            (
                "quadrature_inductance",
                self.result.parameters.quadrature_inductance,
                repeated.parameters.quadrature_inductance,
            ),
        )
        for name, full, reduced_value in pairs:
            relative_change = abs(reduced_value - full) / abs(full)
            self.assertLess(
                relative_change,
                0.05,
                msg=f"{name} moved by {relative_change:.6f} when a tenth of the rows were dropped",
            )

    def test_synthetic_parameters_are_recovered(self) -> None:
        parameters = self.tool.MachineParameters(
            pole_pairs=5,
            reference_bus_voltage=600.0,
            resistance=SYNTHETIC_PARAMETERS["resistance"],
            flux_linkage=SYNTHETIC_PARAMETERS["flux_linkage"],
            direct_inductance=SYNTHETIC_PARAMETERS["direct_inductance"],
            quadrature_inductance=SYNTHETIC_PARAMETERS["quadrature_inductance"],
        )
        speed = np.repeat(np.arange(1000.0, 18001.0, 1000.0), 9)
        quadrature = np.tile(np.linspace(10.0, 80.0, 9), 18)
        direct = -0.2 * quadrature
        direct_voltage, quadrature_voltage = self.tool.calculate_phase_voltage(
            parameters, speed, direct, quadrature
        )
        columns = {
            "speed_rpm": speed,
            "id_peak": direct,
            "iq_peak": quadrature,
            "vd_peak": direct_voltage,
            "vq_peak": quadrature_voltage,
            "stator_copper_loss": 1.5 * parameters.resistance * (direct**2 + quadrature**2),
            "shaft_torque": 1.5
            * 5
            * (
                parameters.flux_linkage * quadrature
                + (parameters.direct_inductance - parameters.quadrature_inductance)
                * direct
                * quadrature
            ),
        }
        anchor = self.tool.calculate_zero_speed_voltage(columns, parameters)
        recovered = self.tool.fit_machine_parameters(columns, zero_speed_anchor=anchor)
        self.assertAlmostEqual(
            recovered.parameters.resistance, parameters.resistance, places=9
        )
        self.assertAlmostEqual(
            recovered.parameters.flux_linkage, parameters.flux_linkage, places=9
        )
        self.assertAlmostEqual(
            recovered.parameters.direct_inductance,
            parameters.direct_inductance,
            places=11,
        )
        self.assertAlmostEqual(
            recovered.parameters.quadrature_inductance,
            parameters.quadrature_inductance,
            places=11,
        )


class AcceptanceTests(unittest.TestCase):
    """Check every acceptance requirement of Decision D22.

    The delivered set is the joint optimum of the model form.
    It misses the table by a small margin on every rule.
    The report records the measured value beside each tolerance.
    """

    @classmethod
    def setUpClass(cls) -> None:
        cls.tool = load_tool_module()
        cls.columns = cls.tool.read_identification_data(IDENTIFICATION_PATH)
        cls.result = cls.tool.fit_machine_parameters(cls.columns)

    def test_copper_loss_is_reproduced(self) -> None:
        measured = self.result.copper_residual.maximum
        self.assertLessEqual(
            measured,
            COPPER_TOLERANCE,
            msg=(
                f"Copper loss is outside {COPPER_TOLERANCE:.0%} above the "
                f"{COPPER_LOSS_FLOOR:g} W floor. Measured maximum relative "
                f"error {measured:.9f}."
            ),
        )

    def test_phase_voltage_is_reproduced(self) -> None:
        measured = self.result.phase_voltage_residual.maximum
        self.assertLessEqual(
            measured,
            VOLTAGE_TOLERANCE,
            msg=(
                f"Phase voltage is outside {VOLTAGE_TOLERANCE:.0%} above the "
                f"{VOLTAGE_FLOOR:g} V floor. Measured maximum relative error "
                f"{measured:.9f}."
            ),
        )

    def test_shaft_torque_is_reproduced(self) -> None:
        measured = self.result.torque_residual.maximum
        self.assertLessEqual(
            measured,
            TORQUE_TOLERANCE,
            msg=(
                f"Shaft torque is outside {TORQUE_TOLERANCE:.0%} above the "
                f"{TORQUE_FLOOR:g} N*m floor. Measured maximum relative error "
                f"{measured:.9f}."
            ),
        )

    def test_zero_speed_voltage_is_reproduced(self) -> None:
        # The comparison is like for like.
        # The anchor is the maximum-current point of the 1 rpm row.
        # The model value comes from that same row, so it holds the small
        # rotational term as well as the resistive drop.
        anchor_row = ZERO_SPEED_ANCHOR_RPM
        mask = self.columns["speed_rpm"] == anchor_row
        self.assertEqual(int(mask.sum()), 21)
        magnitude = np.hypot(
            self.columns["id_peak"][mask], self.columns["iq_peak"][mask]
        )
        index = int(np.argmax(magnitude))
        row_voltage = float(
            np.hypot(
                self.columns["vd_peak"][mask][index],
                self.columns["vq_peak"][mask][index],
            )
        )
        self.assertAlmostEqual(
            row_voltage, MEASURED_ZERO_SPEED_VOLTAGE, places=9
        )
        self.assertAlmostEqual(
            self.result.zero_speed_voltage,
            self.tool.calculate_zero_speed_voltage(
                self.columns, self.result.parameters
            ),
            places=12,
        )

        measured = (
            abs(self.result.zero_speed_voltage - MEASURED_ZERO_SPEED_VOLTAGE)
            / MEASURED_ZERO_SPEED_VOLTAGE
        )
        self.assertLessEqual(
            measured,
            ZERO_SPEED_TOLERANCE,
            msg=(
                f"The zero-speed voltage is outside {ZERO_SPEED_TOLERANCE:.0%}. "
                f"Measured {self.result.zero_speed_voltage:.9f} V against "
                f"{MEASURED_ZERO_SPEED_VOLTAGE:.9f} V at the "
                f"{anchor_row:g} rpm row."
            ),
        )


class JointGuardTests(unittest.TestCase):
    """Check the largest error-to-tolerance ratio of the delivered set.

    The minimax objective is this same ratio, so the delivered set is the
    best set that the model form allows. The ratio must stay at or below
    one against the Decision D22 table.
    """

    @classmethod
    def setUpClass(cls) -> None:
        cls.tool = load_tool_module()
        cls.columns = cls.tool.read_identification_data(IDENTIFICATION_PATH)
        cls.result = cls.tool.fit_machine_parameters(cls.columns)

    def test_ratios_match_the_fitted_result(self) -> None:
        repeated = self.tool.calculate_error_ratios(
            self.result.parameters, self.columns
        )
        self.assertEqual(len(repeated), 4)
        for value, recorded in zip(repeated, self.result.error_ratios):
            self.assertAlmostEqual(value, recorded, places=12)

    def test_largest_ratio_is_within_one(self) -> None:
        ratios = self.result.error_ratios
        names = ("copper loss", "phase voltage", "shaft torque", "zero speed")
        vector = ", ".join(
            f"{name} {value:.6f}" for name, value in zip(names, ratios)
        )
        self.assertLessEqual(
            self.result.largest_error_ratio,
            1.0,
            msg=(
                "The achieved largest error-to-tolerance ratio is "
                f"{self.result.largest_error_ratio:.6f}. The vector is {vector}."
            ),
        )


class OnsetDiagnosticTests(unittest.TestCase):
    """Check the onset speed as a diagnostic.

    Decision D22 withdrew the onset criterion. The envelope comes from
    the map. This test keeps the computation and a loose sanity bound.
    """

    @classmethod
    def setUpClass(cls) -> None:
        cls.tool = load_tool_module()
        cls.columns = cls.tool.read_identification_data(IDENTIFICATION_PATH)
        cls.result = cls.tool.fit_machine_parameters(cls.columns)

    def test_onset_speed_is_a_structural_diagnostic(self) -> None:
        onset = self.result.onset_speed_rpm
        split = self.result.onset_speed_at_measured_split_rpm
        self.assertGreaterEqual(
            onset,
            split,
            msg="A free current angle must hold the maximum current at least as long.",
        )
        self.assertGreater(onset, 0.5 * MEASURED_ONSET_SPEED_RPM)
        self.assertLess(onset, 2.0 * MEASURED_ONSET_SPEED_RPM)

    def test_onset_matches_the_recorded_diagnostic(self) -> None:
        provenance = (PROVENANCE_PATH).read_text(encoding="utf-8")
        recorded = f"{self.result.onset_speed_rpm:,.3f}".replace(",", " ")
        self.assertIn(recorded, provenance)


if __name__ == "__main__":
    unittest.main()
