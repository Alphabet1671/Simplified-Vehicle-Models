#!/usr/bin/env python3
"""Identify the AMK DD5 machine parameters from the exported map.

The tool reads ``models/data/amk_dd5_100c_identification.csv``.
The tool writes ``models/data/amk_dd5_100c_machine_parameters.csv``.
The tool is an offline tool. It is not part of the installed package.
The runtime reads the fitted values from the data file.

Run the tool from the repository root:

    python tools/identify_motor_parameters.py

The fit follows the four machine equations of the plan.

    electrical_speed = 2 * pi * speed_rpm * pole_pairs / 60
    v_d              = resistance * i_d - electrical_speed * quadrature_inductance * i_q
    v_q              = resistance * i_q + electrical_speed * (direct_inductance * i_d + flux_linkage)
    copper_loss      = 1.5 * resistance * (i_d^2 + i_q^2)
    torque           = 1.5 * pole_pairs * (flux_linkage * i_q + (direct_inductance - quadrature_inductance) * i_d * i_q)

The four parameters follow from three linear least-squares steps.
Every equation is linear in its parameters.
The steps use one shared resistance value, so the fit is a joint fit.
The tool does not use per-point medians for the delivered values.
The three steps give the reference fit only.
A second pass minimises the largest error-to-tolerance ratio of the
Decision D22 table. That pass gives the delivered values, because the
acceptance rule is a set of maximum relative errors.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DATA_DIRECTORY = REPOSITORY_ROOT / "models" / "data"

IDENTIFICATION_PATH = DATA_DIRECTORY / "amk_dd5_100c_identification.csv"
PARAMETER_PATH = DATA_DIRECTORY / "amk_dd5_100c_machine_parameters.csv"

# Task P1 confirmed these values from the source map.
# The electrical frequency equals pole_pairs * speed_rpm / 60 on every row.
POLE_PAIRS = 5
REFERENCE_BUS_VOLTAGE = 600.0

# A divisor must be well above zero before a row enters the fit.
# The lowest speed in the export is 1 rpm and the next speed is 100 rpm.
MINIMUM_ELECTRICAL_SPEED = 10.0
MINIMUM_QUADRATURE_CURRENT = 1.0

# The acceptance rules use these floors on the relative error.
# A floor stops a near-zero denominator from hiding a real absolute error.
# Decision D22 sets the floors and the tolerances.
COPPER_LOSS_FLOOR = 1000.0
VOLTAGE_FLOOR = 60.0
TORQUE_FLOOR = 5.0
MINIMUM_ACCEPTANCE_TORQUE = 5.0

# Decision D22 withdrew the onset criterion.
# The onset speed stays as a diagnostic only.
COPPER_TOLERANCE = 0.06
VOLTAGE_TOLERANCE = 0.12
TORQUE_TOLERANCE = 0.18
ZERO_SPEED_TOLERANCE = 0.05

# The delivered values minimise the largest error-to-tolerance ratio.
# The starts are fixed, so the result is reproducible.
# Every point holds (resistance, flux_linkage, direct_inductance_mH,
# quadrature_inductance_mH).
MINIMAX_STARTS = (
    (0.249177000, 0.044152400, 1.206438, 0.898731),
    (0.260250678, 0.043434429, 1.856017, 0.942095),
    (0.252553729, 0.047145150, 0.334989, 1.144849),
    (0.264849237, 0.039617369, 0.820485, 0.997163),
    (0.254973384, 0.052783010, 0.770730, 0.752880),
    (0.239578311, 0.050890085, 0.958628, 1.034330),
    (0.236015064, 0.048624810, 2.087335, 0.545955),
    (0.233787671, 0.053748466, 0.417915, 0.723533),
    (0.236529147, 0.054745663, 1.009129, 1.244871),
    (0.233676196, 0.043352696, 0.663245, 0.452462),
    (0.265028169, 0.046073544, 0.263858, 1.302051),
    (0.249241226, 0.048291031, 1.942710, 0.971588),
    (0.247689619, 0.043324029, 1.140502, 1.141634),
    (0.253986452, 0.047808183, 0.818360, 0.511363),
    (0.256247003, 0.040071590, 1.537767, 0.445390),
    (0.266786205, 0.043570641, 1.359613, 0.465128),
)
MINIMAX_STEPS = (0.006, 0.002, 0.08, 0.08)
MINIMAX_ITERATIONS = 4000

# The measured anchors come from Task P1 and the plan.
MEASURED_ZERO_SPEED_VOLTAGE = 20.731300908660458
MEASURED_ONSET_SPEED_RPM = 16300.0

# The onset search samples the current circle and then refines the best point.
ONSET_SAMPLE_COUNT = 20001
ONSET_REFINE_ITERATIONS = 200
ONSET_BISECTION_ITERATIONS = 80
ONSET_SEARCH_LOW_RPM = 1.0
ONSET_SEARCH_HIGH_RPM = 40000.0

IDENTIFICATION_COLUMNS = (
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
)


@dataclass(frozen=True, slots=True)
class MachineParameters:
    """Hold the four fitted machine parameters and two recorded constants."""

    pole_pairs: int
    reference_bus_voltage: float
    resistance: float
    flux_linkage: float
    direct_inductance: float
    quadrature_inductance: float


@dataclass(frozen=True, slots=True)
class ParameterEstimate:
    """Hold one fitted value, its standard error, and its row count."""

    value: float
    standard_error: float
    rows_used: int


@dataclass(frozen=True, slots=True)
class ResidualSummary:
    """Hold the relative error distribution of one equation."""

    p10: float
    median: float
    p90: float
    maximum: float


@dataclass(frozen=True, slots=True)
class AcceptanceRecord:
    """Hold one acceptance requirement, its measured value, and its tolerance."""

    name: str
    measured: float
    tolerance: float
    passed: bool


@dataclass(frozen=True, slots=True)
class MachineIdentificationResult:
    """Hold every fitted value, residual, ratio, and acceptance record."""

    parameters: MachineParameters
    reference: MachineParameters
    resistance: ParameterEstimate
    flux_linkage: ParameterEstimate
    direct_inductance: ParameterEstimate
    quadrature_inductance: ParameterEstimate
    copper_residual: ResidualSummary
    direct_voltage_residual: ResidualSummary
    quadrature_voltage_residual: ResidualSummary
    phase_voltage_residual: ResidualSummary
    torque_residual: ResidualSummary
    error_ratios: tuple[float, float, float, float]
    optimizer_starts: int
    maximum_current: float
    zero_speed_voltage: float
    measured_zero_speed_voltage: float
    onset_speed_rpm: float
    onset_speed_at_measured_split_rpm: float
    measured_onset_speed_rpm: float
    acceptance: tuple[AcceptanceRecord, ...]

    @property
    def largest_error_ratio(self) -> float:
        """Return the largest error-to-tolerance ratio."""
        return max(self.error_ratios)


# --------------------------------------------------------------------------
# Data input
# --------------------------------------------------------------------------


def read_identification_data(path: Path = IDENTIFICATION_PATH) -> dict[str, np.ndarray]:
    """Read the identification CSV into one array per column.

    The file starts with comment lines. The reader skips them.
    """
    lines = [
        line
        for line in path.read_text(encoding="utf-8").splitlines()
        if line and not line.startswith("#")
    ]
    rows = list(csv.DictReader(lines))
    if not rows:
        raise ValueError(f"the identification file {path} holds no data rows")
    missing = [name for name in IDENTIFICATION_COLUMNS if name not in rows[0]]
    if missing:
        raise ValueError(f"the identification file {path} is missing {missing}")
    return {
        name: np.array([float(row[name]) for row in rows], dtype=np.float64)
        for name in IDENTIFICATION_COLUMNS
    }


def calculate_electrical_speed(
    speed_rpm: np.ndarray,
    pole_pairs: int = POLE_PAIRS,
) -> np.ndarray:
    """Return the electrical angular speed in rad/s for a mechanical speed."""
    return 2.0 * math.pi * np.asarray(speed_rpm, dtype=np.float64) * pole_pairs / 60.0


def calculate_phase_voltage(
    parameters: MachineParameters,
    speed_rpm: np.ndarray,
    direct_current: np.ndarray,
    quadrature_current: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return the model d/q voltages for a current pair and a speed."""
    electrical_speed = calculate_electrical_speed(speed_rpm, parameters.pole_pairs)
    direct_voltage = (
        parameters.resistance * direct_current
        - electrical_speed * parameters.quadrature_inductance * quadrature_current
    )
    quadrature_voltage = parameters.resistance * quadrature_current + electrical_speed * (
        parameters.direct_inductance * direct_current + parameters.flux_linkage
    )
    return direct_voltage, quadrature_voltage


# --------------------------------------------------------------------------
# The joint least-squares fit
# --------------------------------------------------------------------------


def _summarize(relative_error: np.ndarray) -> ResidualSummary:
    """Return the 10th, 50th, and 90th percentiles and the maximum."""
    values = np.asarray(relative_error, dtype=np.float64)
    p10, median, p90 = np.percentile(values, [10.0, 50.0, 90.0])
    return ResidualSummary(
        p10=float(p10),
        median=float(median),
        p90=float(p90),
        maximum=float(values.max()),
    )


def _relative_error(predicted: np.ndarray, measured: np.ndarray, floor: float) -> np.ndarray:
    """Return the relative error with a floor on the denominator."""
    return np.abs(predicted - measured) / np.maximum(np.abs(measured), floor)


def _fit_through_origin(regressor: np.ndarray, response: np.ndarray) -> ParameterEstimate:
    """Return the slope of a least-squares fit through the origin.

    The estimate holds the standard error and the row count.
    """
    normal = float(regressor @ regressor)
    value = float(regressor @ response) / normal
    residual = response - regressor * value
    degrees = regressor.size - 1
    variance = float(residual @ residual) / degrees
    return ParameterEstimate(value, math.sqrt(variance / normal), int(regressor.size))


def _fit_with_constant(
    term: np.ndarray,
    response: np.ndarray,
) -> tuple[ParameterEstimate, ParameterEstimate]:
    """Return the constant and the slope of a two-term fit.

    The model is ``response = constant + slope * term``.
    The estimates hold the standard errors and the row count.
    """
    design = np.column_stack([np.ones(term.size), term])
    coefficients, _, _, _ = np.linalg.lstsq(design, response, rcond=None)
    residual = response - design @ coefficients
    degrees = term.size - 2
    variance = float(residual @ residual) / degrees
    covariance = variance * np.linalg.inv(design.T @ design)
    count = int(term.size)
    return (
        ParameterEstimate(float(coefficients[0]), math.sqrt(float(covariance[0, 0])), count),
        ParameterEstimate(float(coefficients[1]), math.sqrt(float(covariance[1, 1])), count),
    )


def _evaluate_residuals(
    parameters: MachineParameters,
    columns: dict[str, np.ndarray],
) -> tuple[
    ResidualSummary,
    ResidualSummary,
    ResidualSummary,
    ResidualSummary,
    ResidualSummary,
]:
    """Return the residual summary of every machine equation.

    The order is copper loss, direct voltage, quadrature voltage, phase
    voltage, and shaft torque.
    """
    speed_rpm = columns["speed_rpm"]
    direct_current = columns["id_peak"]
    quadrature_current = columns["iq_peak"]
    direct_voltage = columns["vd_peak"]
    quadrature_voltage = columns["vq_peak"]
    copper_loss = columns["stator_copper_loss"]
    shaft_torque = columns["shaft_torque"]

    predicted_direct, predicted_quadrature = calculate_phase_voltage(
        parameters, speed_rpm, direct_current, quadrature_current
    )
    predicted_copper = 1.5 * parameters.resistance * (
        direct_current**2 + quadrature_current**2
    )
    predicted_torque = 1.5 * parameters.pole_pairs * (
        parameters.flux_linkage * quadrature_current
        + (parameters.direct_inductance - parameters.quadrature_inductance)
        * direct_current
        * quadrature_current
    )
    torque_mask = shaft_torque > MINIMUM_ACCEPTANCE_TORQUE
    return (
        _summarize(_relative_error(predicted_copper, copper_loss, COPPER_LOSS_FLOOR)),
        _summarize(_relative_error(predicted_direct, direct_voltage, VOLTAGE_FLOOR)),
        _summarize(_relative_error(predicted_quadrature, quadrature_voltage, VOLTAGE_FLOOR)),
        _summarize(
            _relative_error(
                np.hypot(predicted_direct, predicted_quadrature),
                np.hypot(direct_voltage, quadrature_voltage),
                VOLTAGE_FLOOR,
            )
        ),
        _summarize(
            _relative_error(
                predicted_torque[torque_mask],
                shaft_torque[torque_mask],
                TORQUE_FLOOR,
            )
        ),
    )


def _acceptance_record(name: str, measured: float, tolerance: float) -> AcceptanceRecord:
    """Return one acceptance record."""
    return AcceptanceRecord(
        name=name,
        measured=measured,
        tolerance=tolerance,
        passed=measured <= tolerance,
    )


def _build_acceptance(
    copper: ResidualSummary,
    phase_voltage: ResidualSummary,
    torque: ResidualSummary,
    zero_speed_voltage: float,
    zero_speed_anchor: float = MEASURED_ZERO_SPEED_VOLTAGE,
) -> tuple[AcceptanceRecord, ...]:
    """Return every acceptance record of Decision D22.

    The records follow the new table. The onset criterion is withdrawn,
    so the onset speed is a diagnostic and not a record.
    """
    zero_speed_error = (
        abs(zero_speed_voltage - zero_speed_anchor) / zero_speed_anchor
    )
    return (
        _acceptance_record(
            f"copper loss above the {COPPER_LOSS_FLOOR:g} W floor",
            copper.maximum,
            COPPER_TOLERANCE,
        ),
        _acceptance_record(
            f"phase voltage above the {VOLTAGE_FLOOR:g} V floor",
            phase_voltage.maximum,
            VOLTAGE_TOLERANCE,
        ),
        _acceptance_record(
            f"shaft torque above {MINIMUM_ACCEPTANCE_TORQUE:g} N*m",
            torque.maximum,
            TORQUE_TOLERANCE,
        ),
        _acceptance_record(
            "zero-speed required voltage",
            zero_speed_error,
            ZERO_SPEED_TOLERANCE,
        ),
    )


# --------------------------------------------------------------------------
# The minimax-ratio refinement
# --------------------------------------------------------------------------


def _nelder_mead(
    objective: Callable[[np.ndarray], float],
    start: np.ndarray,
    steps: np.ndarray,
    iterations: int = MINIMAX_ITERATIONS,
) -> tuple[np.ndarray, float]:
    """Return the best point of one deterministic Nelder-Mead search.

    The search is small and fixed. The start and the steps set the result.
    """
    point = np.asarray(start, dtype=np.float64)
    size = point.size
    simplex = [point.copy()]
    for index in range(size):
        other = point.copy()
        other[index] += steps[index]
        simplex.append(other)
    simplex = np.array(simplex)
    values = np.array([float(objective(item)) for item in simplex])

    for _ in range(iterations):
        order = np.argsort(values)
        simplex = simplex[order]
        values = values[order]
        if np.max(np.abs(simplex[1:] - simplex[0])) < 1.0e-13:
            break
        centre = simplex[:-1].mean(axis=0)
        reflected = centre + (centre - simplex[-1])
        reflected_value = float(objective(reflected))
        if reflected_value < values[0]:
            expanded = centre + 2.0 * (centre - simplex[-1])
            expanded_value = float(objective(expanded))
            if expanded_value < reflected_value:
                simplex[-1] = expanded
                values[-1] = expanded_value
            else:
                simplex[-1] = reflected
                values[-1] = reflected_value
        elif reflected_value < values[-2]:
            simplex[-1] = reflected
            values[-1] = reflected_value
        else:
            contracted = centre + 0.5 * (simplex[-1] - centre)
            contracted_value = float(objective(contracted))
            if contracted_value < values[-1]:
                simplex[-1] = contracted
                values[-1] = contracted_value
            else:
                for index in range(1, size + 1):
                    simplex[index] = simplex[0] + 0.5 * (simplex[index] - simplex[0])
                    values[index] = float(objective(simplex[index]))

    order = np.argsort(values)
    return simplex[order[0]], float(values[order[0]])


def _unpack_parameters(
    point: np.ndarray,
    pole_pairs: int,
    reference_bus_voltage: float,
) -> MachineParameters:
    """Return the parameter set of one scaled optimiser point.

    The first two coordinates are the resistance in ohm and the flux
    linkage in Wb. The last two are the two inductances in mH.
    """
    return MachineParameters(
        pole_pairs=pole_pairs,
        reference_bus_voltage=reference_bus_voltage,
        resistance=float(point[0]),
        flux_linkage=float(point[1]),
        direct_inductance=float(point[2]) * 1.0e-3,
        quadrature_inductance=float(point[3]) * 1.0e-3,
    )


def calculate_error_vector(
    parameters: MachineParameters,
    columns: dict[str, np.ndarray],
    zero_speed_anchor: float = MEASURED_ZERO_SPEED_VOLTAGE,
) -> tuple[float, float, float, float]:
    """Return the four maximum relative errors of the model.

    The order is copper loss, phase voltage, shaft torque, and zero speed.
    Every error carries its Decision D22 floor.
    """
    speed_rpm = columns["speed_rpm"]
    direct_current = columns["id_peak"]
    quadrature_current = columns["iq_peak"]
    copper_loss = columns["stator_copper_loss"]
    shaft_torque = columns["shaft_torque"]
    measured_voltage = np.hypot(columns["vd_peak"], columns["vq_peak"])

    predicted_direct, predicted_quadrature = calculate_phase_voltage(
        parameters, speed_rpm, direct_current, quadrature_current
    )
    predicted_copper = 1.5 * parameters.resistance * (
        direct_current**2 + quadrature_current**2
    )
    predicted_torque = 1.5 * parameters.pole_pairs * (
        parameters.flux_linkage * quadrature_current
        + (parameters.direct_inductance - parameters.quadrature_inductance)
        * direct_current
        * quadrature_current
    )
    torque_mask = shaft_torque > MINIMUM_ACCEPTANCE_TORQUE
    zero_speed_voltage = calculate_zero_speed_voltage(columns, parameters)
    return (
        float(_relative_error(predicted_copper, copper_loss, COPPER_LOSS_FLOOR).max()),
        float(
            _relative_error(
                np.hypot(predicted_direct, predicted_quadrature),
                measured_voltage,
                VOLTAGE_FLOOR,
            ).max()
        ),
        float(
            _relative_error(
                predicted_torque[torque_mask],
                shaft_torque[torque_mask],
                TORQUE_FLOOR,
            ).max()
        ),
        float(abs(zero_speed_voltage - zero_speed_anchor) / zero_speed_anchor),
    )


def calculate_error_ratios(
    parameters: MachineParameters,
    columns: dict[str, np.ndarray],
    zero_speed_anchor: float = MEASURED_ZERO_SPEED_VOLTAGE,
) -> tuple[float, float, float, float]:
    """Return the four error-to-tolerance ratios of the model."""
    errors = calculate_error_vector(parameters, columns, zero_speed_anchor)
    tolerances = (
        COPPER_TOLERANCE,
        VOLTAGE_TOLERANCE,
        TORQUE_TOLERANCE,
        ZERO_SPEED_TOLERANCE,
    )
    return tuple(
        error / tolerance for error, tolerance in zip(errors, tolerances)
    )


def refine_minimax(
    columns: dict[str, np.ndarray],
    reference: MachineParameters,
    pole_pairs: int = POLE_PAIRS,
    reference_bus_voltage: float = REFERENCE_BUS_VOLTAGE,
    zero_speed_anchor: float = MEASURED_ZERO_SPEED_VOLTAGE,
) -> tuple[MachineParameters, int]:
    """Return the set with the smallest largest error-to-tolerance ratio.

    The objective is the same quantity as the acceptance rule. Every
    start is fixed, so the result repeats. The start count comes back
    with the set. The least-squares reference is one candidate, so the
    refinement never worsens it.
    """
    def objective(point: np.ndarray) -> float:
        parameters = _unpack_parameters(point, pole_pairs, reference_bus_voltage)
        return max(calculate_error_ratios(parameters, columns, zero_speed_anchor))

    best_point = np.array(
        [
            reference.resistance,
            reference.flux_linkage,
            reference.direct_inductance * 1.0e3,
            reference.quadrature_inductance * 1.0e3,
        ],
        dtype=np.float64,
    )
    best_value = objective(best_point)

    steps = np.array(MINIMAX_STEPS, dtype=np.float64)
    for start in MINIMAX_STARTS:
        point, value = _nelder_mead(
            objective,
            np.array(start, dtype=np.float64),
            steps,
        )
        if value < best_value:
            best_value = value
            best_point = point
    return (
        _unpack_parameters(best_point, pole_pairs, reference_bus_voltage),
        len(MINIMAX_STARTS),
    )


def fit_reference_parameters(
    columns: dict[str, np.ndarray],
    pole_pairs: int = POLE_PAIRS,
) -> tuple[ParameterEstimate, ParameterEstimate, ParameterEstimate, ParameterEstimate]:
    """Return the three-step least-squares estimates.

    Step one fits the resistance from the copper loss through the origin.
    Step two fits the flux linkage and the direct inductance from the q
    voltage. Step three fits the quadrature inductance from the d voltage.
    Every estimate carries its standard error and its row count.
    """
    speed_rpm = columns["speed_rpm"]
    direct_current = columns["id_peak"]
    quadrature_current = columns["iq_peak"]
    direct_voltage = columns["vd_peak"]
    quadrature_voltage = columns["vq_peak"]
    copper_loss = columns["stator_copper_loss"]
    electrical_speed = calculate_electrical_speed(speed_rpm, pole_pairs)

    # Step one: copper_loss = 1.5 * resistance * (id^2 + iq^2).
    resistance = _fit_through_origin(
        1.5 * (direct_current**2 + quadrature_current**2),
        copper_loss,
    )

    # Step two: (vq - resistance * iq) / we = flux_linkage + direct_inductance * id.
    flux_mask = electrical_speed > MINIMUM_ELECTRICAL_SPEED
    flux_linkage, direct_inductance = _fit_with_constant(
        direct_current[flux_mask],
        (
            (quadrature_voltage - resistance.value * quadrature_current)
            / electrical_speed
        )[flux_mask],
    )

    # Step three: resistance * id - vd = we * quadrature_inductance * iq.
    inductance_mask = flux_mask & (quadrature_current > MINIMUM_QUADRATURE_CURRENT)
    quadrature_inductance = _fit_through_origin(
        (electrical_speed * quadrature_current)[inductance_mask],
        (resistance.value * direct_current - direct_voltage)[inductance_mask],
    )
    return resistance, flux_linkage, direct_inductance, quadrature_inductance


def fit_machine_parameters(
    columns: dict[str, np.ndarray],
    pole_pairs: int = POLE_PAIRS,
    reference_bus_voltage: float = REFERENCE_BUS_VOLTAGE,
    zero_speed_anchor: float = MEASURED_ZERO_SPEED_VOLTAGE,
) -> MachineIdentificationResult:
    """Fit the four machine parameters from the identification columns.

    The first pass is the three-step least-squares fit. It gives the
    standard errors and the row counts.
    The second pass minimises the largest error-to-tolerance ratio. It
    gives the delivered values. The acceptance rule is a set of maximum
    relative errors, so the objective matches the rule.
    """
    (
        resistance,
        flux_linkage,
        direct_inductance,
        quadrature_inductance,
    ) = fit_reference_parameters(columns, pole_pairs)
    reference = MachineParameters(
        pole_pairs=pole_pairs,
        reference_bus_voltage=reference_bus_voltage,
        resistance=resistance.value,
        flux_linkage=flux_linkage.value,
        direct_inductance=direct_inductance.value,
        quadrature_inductance=quadrature_inductance.value,
    )
    parameters, optimizer_starts = refine_minimax(
        columns,
        reference,
        pole_pairs,
        reference_bus_voltage,
        zero_speed_anchor,
    )

    (
        copper_residual,
        direct_voltage_residual,
        quadrature_voltage_residual,
        phase_voltage_residual,
        torque_residual,
    ) = _evaluate_residuals(parameters, columns)

    error_ratios = calculate_error_ratios(parameters, columns, zero_speed_anchor)
    maximum_current = calculate_maximum_current(columns)
    zero_speed_voltage = calculate_zero_speed_voltage(columns, parameters)
    onset_speed = calculate_onset_speed(
        parameters, maximum_current, reference_bus_voltage
    )
    onset_at_measured_split = calculate_onset_speed_at_measured_split(
        columns, parameters, reference_bus_voltage
    )
    measured_onset = calculate_measured_onset_speed(columns, reference_bus_voltage)

    acceptance = _build_acceptance(
        copper_residual,
        phase_voltage_residual,
        torque_residual,
        zero_speed_voltage,
        zero_speed_anchor,
    )

    return MachineIdentificationResult(
        parameters=parameters,
        reference=reference,
        resistance=resistance,
        flux_linkage=flux_linkage,
        direct_inductance=direct_inductance,
        quadrature_inductance=quadrature_inductance,
        copper_residual=copper_residual,
        direct_voltage_residual=direct_voltage_residual,
        quadrature_voltage_residual=quadrature_voltage_residual,
        phase_voltage_residual=phase_voltage_residual,
        torque_residual=torque_residual,
        error_ratios=error_ratios,
        optimizer_starts=optimizer_starts,
        maximum_current=maximum_current,
        zero_speed_voltage=zero_speed_voltage,
        measured_zero_speed_voltage=zero_speed_anchor,
        onset_speed_rpm=onset_speed,
        onset_speed_at_measured_split_rpm=onset_at_measured_split,
        measured_onset_speed_rpm=measured_onset,
        acceptance=acceptance,
    )


# --------------------------------------------------------------------------
# Onset and anchor calculations
# --------------------------------------------------------------------------


def calculate_maximum_current(columns: dict[str, np.ndarray]) -> float:
    """Return the largest current magnitude in the map."""
    magnitude = np.hypot(columns["id_peak"], columns["iq_peak"])
    return float(magnitude.max())


def _minimum_voltage_at_speed(
    parameters: MachineParameters,
    electrical_speed: float,
    current: float,
) -> float:
    """Return the smallest phase voltage on the current circle.

    The machine holds the current magnitude. The current angle is free.
    The machine picks the angle that needs the least voltage.
    """
    direct = np.linspace(-current, current, ONSET_SAMPLE_COUNT)
    quadrature = np.sqrt(np.maximum(current * current - direct * direct, 0.0))
    direct_voltage = (
        parameters.resistance * direct
        - electrical_speed * parameters.quadrature_inductance * quadrature
    )
    quadrature_voltage = parameters.resistance * quadrature + electrical_speed * (
        parameters.direct_inductance * direct + parameters.flux_linkage
    )
    square = direct_voltage**2 + quadrature_voltage**2
    index = int(np.argmin(square))

    if index == 0:
        low, high = direct[0], direct[1]
    elif index == direct.size - 1:
        low, high = direct[-2], direct[-1]
    else:
        low, high = direct[index - 1], direct[index + 1]

    def evaluate(value: float) -> float:
        other = math.sqrt(max(current * current - value * value, 0.0))
        first = parameters.resistance * value - electrical_speed * parameters.quadrature_inductance * other
        second = parameters.resistance * other + electrical_speed * (
            parameters.direct_inductance * value + parameters.flux_linkage
        )
        return first * first + second * second

    for _ in range(ONSET_REFINE_ITERATIONS):
        first = low + (high - low) / 3.0
        second = high - (high - low) / 3.0
        if evaluate(first) < evaluate(second):
            high = second
        else:
            low = first
    return math.sqrt(max(evaluate(0.5 * (low + high)), 0.0))


def calculate_onset_speed(
    parameters: MachineParameters,
    maximum_current: float,
    bus_voltage: float = REFERENCE_BUS_VOLTAGE,
) -> float:
    """Return the voltage-limit onset speed at the maximum current.

    The onset is the lowest speed where the smallest phase voltage on the
    maximum-current circle reaches the bus voltage.
    Below the onset the machine can hold the maximum current.
    Above the onset no current angle keeps the voltage within the limit.
    """
    def required_voltage(speed_rpm: float) -> float:
        electrical_speed = calculate_electrical_speed(
            np.array([speed_rpm]), parameters.pole_pairs
        )[0]
        return _minimum_voltage_at_speed(parameters, float(electrical_speed), maximum_current)

    low = ONSET_SEARCH_LOW_RPM
    high = ONSET_SEARCH_HIGH_RPM
    if required_voltage(low) >= bus_voltage:
        return low
    if required_voltage(high) < bus_voltage:
        return high
    for _ in range(ONSET_BISECTION_ITERATIONS):
        middle = 0.5 * (low + high)
        if required_voltage(middle) < bus_voltage:
            low = middle
        else:
            high = middle
    return 0.5 * (low + high)


def calculate_onset_speed_at_measured_split(
    columns: dict[str, np.ndarray],
    parameters: MachineParameters,
    bus_voltage: float = REFERENCE_BUS_VOLTAGE,
) -> float:
    """Return the onset speed with the map current split.

    The model evaluates the map current at the maximum-current point of
    every speed. This is a stricter check than the free-angle onset.
    """
    speed_rpm = columns["speed_rpm"]
    direct_current = columns["id_peak"]
    quadrature_current = columns["iq_peak"]
    magnitude = np.hypot(direct_current, quadrature_current)
    for speed in np.unique(speed_rpm):
        mask = speed_rpm == speed
        index = int(np.argmax(magnitude[mask]))
        predicted_direct, predicted_quadrature = calculate_phase_voltage(
            parameters,
            np.array([speed]),
            np.array([direct_current[mask][index]]),
            np.array([quadrature_current[mask][index]]),
        )
        voltage = float(np.hypot(predicted_direct, predicted_quadrature)[0])
        if voltage >= bus_voltage:
            return float(speed)
    return float(speed_rpm.max())


def calculate_measured_onset_speed(
    columns: dict[str, np.ndarray],
    bus_voltage: float = REFERENCE_BUS_VOLTAGE,
) -> float:
    """Return the measured onset speed from the map voltages.

    The measurement reads the map voltage at the maximum-current point of
    every speed. The onset is the first speed that reaches the bus voltage.
    """
    speed_rpm = columns["speed_rpm"]
    magnitude = np.hypot(columns["id_peak"], columns["iq_peak"])
    measured_voltage = np.hypot(columns["vd_peak"], columns["vq_peak"])
    for speed in np.unique(speed_rpm):
        mask = speed_rpm == speed
        index = int(np.argmax(magnitude[mask]))
        if float(measured_voltage[mask][index]) >= bus_voltage - 1.0e-9:
            return float(speed)
    return float(speed_rpm.max())


def calculate_zero_speed_voltage(
    columns: dict[str, np.ndarray],
    parameters: MachineParameters,
) -> float:
    """Return the model voltage at the lowest-speed maximum-torque point.

    The lowest speed in the export is 1 rpm. The mechanical speed is small,
    so the voltage is nearly the resistive drop at the full current.
    """
    speed_rpm = columns["speed_rpm"]
    lowest = float(speed_rpm.min())
    mask = speed_rpm == lowest
    magnitude = np.hypot(columns["id_peak"], columns["iq_peak"])
    index = int(np.argmax(magnitude[mask]))
    direct_voltage, quadrature_voltage = calculate_phase_voltage(
        parameters,
        np.array([lowest]),
        np.array([columns["id_peak"][mask][index]]),
        np.array([columns["iq_peak"][mask][index]]),
    )
    return float(np.hypot(direct_voltage, quadrature_voltage)[0])


# --------------------------------------------------------------------------
# Output file
# --------------------------------------------------------------------------


def format_parameter_value(value: float) -> str:
    """Return one parameter value with twelve significant digits."""
    return f"{value:.12g}"


def build_parameter_file(result: MachineIdentificationResult) -> str:
    """Return the text of the machine parameter file."""
    parameters = result.parameters
    rows = [
        ("pole_pairs", float(parameters.pole_pairs), "1", 0.0, 0),
        (
            "reference_bus_voltage",
            parameters.reference_bus_voltage,
            "V",
            0.0,
            0,
        ),
        (
            "resistance",
            parameters.resistance,
            "ohm",
            result.resistance.standard_error,
            result.resistance.rows_used,
        ),
        (
            "flux_linkage",
            parameters.flux_linkage,
            "Wb",
            result.flux_linkage.standard_error,
            result.flux_linkage.rows_used,
        ),
        (
            "direct_inductance",
            parameters.direct_inductance,
            "H",
            result.direct_inductance.standard_error,
            result.direct_inductance.rows_used,
        ),
        (
            "quadrature_inductance",
            parameters.quadrature_inductance,
            "H",
            result.quadrature_inductance.standard_error,
            result.quadrature_inductance.rows_used,
        ),
    ]

    lines = [
        "# AMK DD5 machine parameters at 100 C.",
        "# Columns: name, value, unit, standard_error, rows_used.",
        "# The tool tools/identify_motor_parameters.py produced every value.",
        "# The value minimises the largest error-to-tolerance ratio of Decision D22.",
        "# A three-step least-squares pass gives the standard error and the row count.",
        "# A rows_used value of zero marks a recorded constant, not a fitted value.",
        "# SI units. The inductances are in henry. The resistance is in ohm.",
        "name,value,unit,standard_error,rows_used",
    ]
    for name, value, unit, error, used in rows:
        lines.append(
            f"{name},{format_parameter_value(value)},{unit},"
            f"{format_parameter_value(error)},{used}"
        )
    return "\n".join(lines) + "\n"


def write_parameter_file(
    result: MachineIdentificationResult,
    path: Path = PARAMETER_PATH,
) -> None:
    """Write the machine parameter file."""
    path.write_text(build_parameter_file(result), encoding="utf-8")


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------


def identify(source: Path = IDENTIFICATION_PATH) -> MachineIdentificationResult:
    """Read the identification file and fit the machine parameters."""
    columns = read_identification_data(source)
    return fit_machine_parameters(columns)


def format_report(result: MachineIdentificationResult) -> str:
    """Return a short text report of the fit."""
    parameters = result.parameters
    lines = [
        "Machine parameter identification",
        f"  resistance             {parameters.resistance:.10g} ohm"
        f" (se {result.resistance.standard_error:.3g},"
        f" rows {result.resistance.rows_used})",
        f"  flux_linkage           {parameters.flux_linkage:.10g} Wb"
        f" (se {result.flux_linkage.standard_error:.3g},"
        f" rows {result.flux_linkage.rows_used})",
        f"  direct_inductance      {parameters.direct_inductance:.10g} H"
        f" (se {result.direct_inductance.standard_error:.3g},"
        f" rows {result.direct_inductance.rows_used})",
        f"  quadrature_inductance  {parameters.quadrature_inductance:.10g} H"
        f" (se {result.quadrature_inductance.standard_error:.3g},"
        f" rows {result.quadrature_inductance.rows_used})",
        "Residuals (p10, median, p90, maximum), relative error",
        f"  copper loss     {result.copper_residual.p10:.4f}"
        f" {result.copper_residual.median:.4f} {result.copper_residual.p90:.4f}"
        f" {result.copper_residual.maximum:.4f}",
        f"  direct voltage  {result.direct_voltage_residual.p10:.4f}"
        f" {result.direct_voltage_residual.median:.4f}"
        f" {result.direct_voltage_residual.p90:.4f}"
        f" {result.direct_voltage_residual.maximum:.4f}",
        f"  quad voltage    {result.quadrature_voltage_residual.p10:.4f}"
        f" {result.quadrature_voltage_residual.median:.4f}"
        f" {result.quadrature_voltage_residual.p90:.4f}"
        f" {result.quadrature_voltage_residual.maximum:.4f}",
        f"  phase voltage   {result.phase_voltage_residual.p10:.4f}"
        f" {result.phase_voltage_residual.median:.4f}"
        f" {result.phase_voltage_residual.p90:.4f}"
        f" {result.phase_voltage_residual.maximum:.4f}",
        f"  shaft torque    {result.torque_residual.p10:.4f}"
        f" {result.torque_residual.median:.4f} {result.torque_residual.p90:.4f}"
        f" {result.torque_residual.maximum:.4f}",
        f"Maximum current        {result.maximum_current:.6f} A",
        f"Zero-speed voltage     {result.zero_speed_voltage:.6f} V"
        f" against {result.measured_zero_speed_voltage:.6f} V",
        f"Optimiser starts       {result.optimizer_starts}",
        "Error-to-tolerance ratios (copper, phase voltage, torque, zero speed)",
        "  "
        + " ".join(f"{value:.6f}" for value in result.error_ratios)
        + f"  largest {result.largest_error_ratio:.6f}",
        "Diagnostics (Decision D22 withdrew the onset criterion)",
        f"  onset speed          {result.onset_speed_rpm:.3f} rpm"
        f" against the measured {result.measured_onset_speed_rpm:.1f} rpm",
        f"  onset at map split   {result.onset_speed_at_measured_split_rpm:.1f} rpm",
        "Least-squares reference (start of the minimax refinement)",
        f"  resistance             {result.reference.resistance:.10g} ohm",
        f"  flux_linkage           {result.reference.flux_linkage:.10g} Wb",
        f"  direct_inductance      {result.reference.direct_inductance:.10g} H",
        f"  quadrature_inductance  {result.reference.quadrature_inductance:.10g} H",
        "Acceptance",
    ]
    for record in result.acceptance:
        mark = "PASS" if record.passed else "FAIL"
        lines.append(
            f"  {mark} {record.name}: measured {record.measured:.6f},"
            f" tolerance {record.tolerance:.6f}"
        )
    return "\n".join(lines)


def main() -> None:
    """Fit the machine parameters and write the parameter file."""
    result = identify()
    write_parameter_file(result)
    print(format_report(result))
    print(f"wrote {PARAMETER_PATH}")


if __name__ == "__main__":
    main()
