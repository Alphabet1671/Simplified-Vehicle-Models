"""Tests for the machine parameter identification.

The tests regenerate the parameter file from the source identification CSV.
The tests also check every acceptance requirement of the task brief.

Four of the five acceptance requirements are NOT met by the fitted set.
The report ``task-p2-p3-report.md`` records the measured value beside each
tolerance and explains why the model cannot meet them.
The tests stay strict on purpose. A relaxed test would hide the miss.
"""

from __future__ import annotations

import csv
import importlib.util
import sys
import unittest
from pathlib import Path
from typing import Any

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

TOOL_PATH = REPOSITORY_ROOT / "tools" / "identify_motor_parameters.py"
DATA_DIRECTORY = REPOSITORY_ROOT / "models" / "data"
IDENTIFICATION_PATH = DATA_DIRECTORY / "amk_dd5_100c_identification.csv"
PARAMETER_PATH = DATA_DIRECTORY / "amk_dd5_100c_machine_parameters.csv"

MEASURED_ONSET_SPEED_RPM = 16300.0
MEASURED_ZERO_SPEED_VOLTAGE = 20.731300908660458

COPPER_TOLERANCE = 0.03
VOLTAGE_TOLERANCE = 0.05
TORQUE_TOLERANCE = 0.05
ONSET_TOLERANCE = 0.05

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
                0.01,
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
        recovered = self.tool.fit_machine_parameters(columns)
        self.assertAlmostEqual(
            recovered.parameters.resistance, parameters.resistance, places=10
        )
        self.assertAlmostEqual(
            recovered.parameters.flux_linkage, parameters.flux_linkage, places=10
        )
        self.assertAlmostEqual(
            recovered.parameters.direct_inductance,
            parameters.direct_inductance,
            places=12,
        )
        self.assertAlmostEqual(
            recovered.parameters.quadrature_inductance,
            parameters.quadrature_inductance,
            places=12,
        )


class AcceptanceTests(unittest.TestCase):
    """Check every acceptance requirement of the task brief.

    The fitted set meets the zero-speed requirement only.
    The other four requirements are outside tolerance.
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
                "Copper loss is outside 3 percent at some point. "
                f"Measured maximum relative error {measured:.6f}."
            ),
        )

    def test_voltage_is_reproduced(self) -> None:
        requirements = (
            ("direct voltage", self.result.direct_voltage_residual.maximum),
            ("quadrature voltage", self.result.quadrature_voltage_residual.maximum),
            ("phase voltage", self.result.phase_voltage_residual.maximum),
        )
        for name, measured in requirements:
            with self.subTest(quantity=name):
                self.assertLessEqual(
                    measured,
                    VOLTAGE_TOLERANCE,
                    msg=(
                        f"The {name} is outside 5 percent at some point. "
                        f"Measured maximum relative error {measured:.6f}."
                    ),
                )

    def test_torque_is_reproduced(self) -> None:
        measured = self.result.torque_residual.maximum
        self.assertLessEqual(
            measured,
            TORQUE_TOLERANCE,
            msg=(
                "Shaft torque is outside 5 percent above 5 N*m at some point. "
                f"Measured maximum relative error {measured:.6f}."
            ),
        )

    def test_zero_speed_voltage_is_reproduced(self) -> None:
        measured = (
            abs(self.result.zero_speed_voltage - MEASURED_ZERO_SPEED_VOLTAGE)
            / MEASURED_ZERO_SPEED_VOLTAGE
        )
        self.assertLessEqual(
            measured,
            VOLTAGE_TOLERANCE,
            msg=(
                "The zero-speed voltage is outside 5 percent. "
                f"Measured {self.result.zero_speed_voltage:.6f} V against "
                f"{MEASURED_ZERO_SPEED_VOLTAGE:.6f} V."
            ),
        )

    def test_onset_speed_is_reproduced(self) -> None:
        measured = (
            abs(self.result.onset_speed_rpm - MEASURED_ONSET_SPEED_RPM)
            / MEASURED_ONSET_SPEED_RPM
        )
        self.assertLessEqual(
            measured,
            ONSET_TOLERANCE,
            msg=(
                "The voltage-limit onset speed is outside 5 percent. "
                f"The fit gives {self.result.onset_speed_rpm:.3f} rpm against "
                f"the measured {MEASURED_ONSET_SPEED_RPM:.1f} rpm. "
                "The joint fit does not close the gap. The brief forbids a "
                "forced fit."
            ),
        )


if __name__ == "__main__":
    unittest.main()
