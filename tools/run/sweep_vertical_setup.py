"""Damper, anti-roll and inertance setup sweep for the vertical 14-DoF model.

A spring-*rate* override is **not available**: the wheel-rate curve is a spline
inside the sha256-pinned corner NPZ, not an XML scalar, so a spring sweep needs a
LUT regeneration or a new suspension parameter (see the Task 10 report's Open
issue 1). The accepted override names are listed below.

One *setup* is an override mapping over the parameters the compiled 14-DoF
vehicle actually exposes: one click per damper valve (``click_compression``,
``click_rebound``; a four-valve damper exposes ``click_<valve>``; an axle-qualified
``front.click_<valve>`` / ``rear.click_<valve>`` name is also accepted) and the
compiled suspension scalars (``arb_wheel_rate_front``, ``arb_wheel_rate_rear``,
``inertance_front``, ``inertance_rear``, ``static_toe_front``,
``static_toe_rear``). An unknown name raises ``ValueError`` listing the valid
names; a click outside ``0..clicks-1``, a fractional click, or a negative ARB
rate or inertance raises ``ValueError``. Overrides are applied with
``dataclasses.replace`` on the compiled vehicle, so the damper *tables* (the NPZ
LUTs pinned by sha256) are never rewritten: a click selects a different slice of
the same table, which is exactly how the component compiles it.

Every result comes from the model, not from a table of literals:

* **Modal frequency convention.** A reported mode frequency is
  ``|lambda| / (2*pi)`` -- the undamped-equivalent natural frequency of the damped
  pole, *not* ``Im(lambda) / (2*pi)`` (the baseline heave mode is 3.9893 Hz by
  ``|lambda|`` and 3.9196 Hz by ``Im(lambda)``). The ``*_damped_frequency_hz``
  fields report ``Im(lambda) / (2*pi)`` alongside, and are 0 for an overdamped
  (real) eigenvalue.
* **Damper branch convention at zero velocity.** The linearized damping matrix
  uses each damper table's **compression-branch** slope at zero velocity for both
  directions, because the compiled table is kinked at ``v = 0`` (rebound 2346.7
  vs compression 2126.7 N s/m at click 5). That is one of two defensible
  conventions: averaging the two branch slopes instead moves the stiff-click
  first peak to 26.43 mm at 0.0767 s (click 11) instead of 25.87 mm at 0.3038 s,
  so the click-10/11 peak and time-to-peak are convention-sensitive.
* **Linearization (Task 6).** At the speed-``S`` static equilibrium the
  seven-coordinate vertical subsystem ``[heave, roll, pitch, jounce x4]`` is
  linearized. The mass matrix is the evaluator's own
  :func:`~simplified_vehicle_models.formulations.vertical_14dof.vertical_mass_block`;
  the stiffness is the central-difference Jacobian of the evaluator's
  ``generalized_forces``; the damping matrix is the tire vertical damping acting
  between each hub and the ground plus ``slope / MR^2`` from each compiled
  damper table at zero velocity. Heave/roll/pitch modal frequency and damping
  ratio come from the eigenvalues of the 14x14 first-order form. The per-axle
  ride frequency is the lower mode of the two-DoF body-fixed quarter car built
  from the tangent spring and tire rates, and the corner damping ratio is the
  secant ``|F(v_ref)|/v_ref`` of the compiled damper at the 25 mm/s reference
  velocity, divided by ``MR^2`` and by ``2 m_s omega_ride``.
* **``step-response``.** A 2 000 N downward step on the front axle's sprung-body
  points (``[F, 0, -F*x_front, 0, 0, 0, 0]``), the same input the Task 7 step
  test uses. The response is solved in closed form with the eigenvalues of the
  first-order system -- NumPy only, no SciPy, no integrator -- and reported as
  the front body-point displacement. Metrics: first peak, time to first peak,
  overshoot fraction above the steady value, steady state, and settling time
  (the first sample after which ``|y - y_steady|`` stays inside
  ``band * |y_steady|`` for the rest of the window; the band defaults to 2 %).
* **``frequency-response``.** The displacement-per-generalized-force transfer
  ``H_ii(omega) = e_i' (K - omega^2 M + i omega C)^-1 e_i`` for heave, roll, and
  pitch on a fixed grid (0.5-30 Hz, 3 001 points by default), reported as the
  maximum-magnitude grid point and its frequency. Units: m/N for heave,
  rad/(N*m) for roll and pitch.

The tool never runs a lap. Laps go through ``run_iem24_forward_driver.py``, one
attempt per setup.

CLI::

    PYTHONPATH=src:. python3 tools/run/sweep_vertical_setup.py \\
        --study step-response \\
        --grid '{"click_compression": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11], "click_rebound": [7]}' \\
        --output-csv /tmp/click_sweep.csv --output-json /tmp/click_sweep.json
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
import sys
import time
import xml.etree.ElementTree as ElementTree
from dataclasses import dataclass, replace
from pathlib import Path
from types import MappingProxyType
from typing import Iterable, Literal, Mapping, Sequence

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.vertical_14dof import (
    VerticalState,
    evaluate_vertical_dynamics,
    sprung_wheel_positions,
    vertical_mass_block,
)
from simplified_vehicle_models.solvers.vertical_equilibrium import solve_static_equilibrium
from simplified_vehicle_models.suspension.components import CompiledDamper
from simplified_vehicle_models.suspension.damper_table import DamperSetting
from simplified_vehicle_models.suspension.elements import AxleArbRates
from simplified_vehicle_models.suspension.layouts.independent_corner import IndependentCornerNetwork
from simplified_vehicle_models.tires.unitire_transient import (
    TransientTireState,
    steady_rolling_state,
)
from simplified_vehicle_models.tires.vertical import tire_deflection_load

TOOL_VERSION = "vertical-setup-sweep-v1"

#: Repository root, derived from this file, so the default formulation resolves
#: the same way wherever the tool is invoked from.
REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
#: The 14-DoF-simple formulation the runner and Task 9 evidence use.
DEFAULT_FORMULATION = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"

#: Rows of the 21-value ``VerticalState`` vector for the seven vertical coordinates.
POSITION_ROWS: tuple[int, ...] = (6, 7, 8, 13, 14, 15, 16)

#: Coordinate index of each named vertical mode in the seven-coordinate vector.
MODE_INDICES: Mapping[str, int] = MappingProxyType({"heave": 0, "roll": 1, "pitch": 2})

#: Damper velocity [m/s] the corner damping ratio and the plan D7 reference use.
REFERENCE_DAMPER_VELOCITY = 0.025
#: Task 7's step: a 2 000 N downward step on the front axle's body points.
DEFAULT_STEP_FORCE_N = 2000.0
DEFAULT_STEP_DURATION_S = 1.2
DEFAULT_STEP_SAMPLES = 12001
#: Settling band as a fraction of the steady response.
DEFAULT_SETTLING_BAND = 0.02
#: Frequency-response grid (minimum Hz, maximum Hz, samples).
DEFAULT_FREQUENCY_GRID_HZ = (0.5, 30.0, 3001)

STUDIES = ("step-response", "frequency-response")

#: Library sources the sweep's numbers depend on (hashed into the JSON summary).
_TOOL_SOURCES = (
    Path("tools/run/sweep_vertical_setup.py"),
    Path("src/simplified_vehicle_models/compiler/compile.py"),
    Path("src/simplified_vehicle_models/formulations/vertical_14dof.py"),
    Path("src/simplified_vehicle_models/solvers/vertical_equilibrium.py"),
    Path("src/simplified_vehicle_models/suspension/components.py"),
    Path("src/simplified_vehicle_models/suspension/corner.py"),
    Path("src/simplified_vehicle_models/suspension/damper_table.py"),
    Path("src/simplified_vehicle_models/suspension/elements.py"),
    Path("src/simplified_vehicle_models/suspension/kinematics.py"),
    Path("src/simplified_vehicle_models/suspension/layouts/independent_corner.py"),
    Path("src/simplified_vehicle_models/suspension/monotone_spline.py"),
    Path("src/simplified_vehicle_models/suspension/network.py"),
    Path("src/simplified_vehicle_models/tires/unitire_simple.py"),
    Path("src/simplified_vehicle_models/tires/unitire_transient.py"),
    Path("src/simplified_vehicle_models/tires/vertical.py"),
)

#: Metric columns of the CSV, in order (``study`` and the override names come first).
_METRIC_FIELDS = (
    "front_ride_frequency_hz",
    "rear_ride_frequency_hz",
    "heave_frequency_hz",
    "heave_damping_ratio",
    "roll_frequency_hz",
    "roll_damping_ratio",
    "pitch_frequency_hz",
    "pitch_damping_ratio",
    "heave_damped_frequency_hz",
    "roll_damped_frequency_hz",
    "pitch_damped_frequency_hz",
    "front_corner_damping_ratio",
    "front_corner_damping_ratio_rebound",
    "rear_corner_damping_ratio",
    "rear_corner_damping_ratio_rebound",
    "step_peak_m",
    "step_time_to_peak_s",
    "step_overshoot_fraction",
    "step_settling_time_s",
    "step_steady_state_m",
    "heave_peak_magnitude",
    "heave_peak_frequency_hz",
    "roll_peak_magnitude",
    "roll_peak_frequency_hz",
    "pitch_peak_magnitude",
    "pitch_peak_frequency_hz",
)


@dataclass(frozen=True, slots=True)
class SweepResult:
    """One setup's linearized ride, modal, and study metrics.

    ``overrides`` echoes the mapping that produced the result. Modal fields are
    always present; the ``step_*`` fields are populated only for the
    ``step-response`` study and the ``*_peak_*`` fields only for the
    ``frequency-response`` study (``None`` otherwise).

    ``*_frequency_hz`` is ``|lambda| / (2*pi)``, the undamped-equivalent natural
    frequency of the damped pole; ``*_damped_frequency_hz`` is
    ``Im(lambda) / (2*pi)`` (0 for an overdamped mode). The modal damping matrix
    uses each damper table's compression-branch slope at zero velocity for both
    directions (the compiled table is kinked at zero).
    """

    overrides: Mapping[str, float]
    study: str
    front_ride_frequency_hz: float
    rear_ride_frequency_hz: float
    heave_frequency_hz: float
    heave_damping_ratio: float
    roll_frequency_hz: float
    roll_damping_ratio: float
    pitch_frequency_hz: float
    pitch_damping_ratio: float
    heave_damped_frequency_hz: float
    roll_damped_frequency_hz: float
    pitch_damped_frequency_hz: float
    front_corner_damping_ratio: float
    front_corner_damping_ratio_rebound: float
    rear_corner_damping_ratio: float
    rear_corner_damping_ratio_rebound: float
    step_peak_m: float | None = None
    step_time_to_peak_s: float | None = None
    step_overshoot_fraction: float | None = None
    step_settling_time_s: float | None = None
    step_steady_state_m: float | None = None
    heave_peak_magnitude: float | None = None
    heave_peak_frequency_hz: float | None = None
    roll_peak_magnitude: float | None = None
    roll_peak_frequency_hz: float | None = None
    pitch_peak_magnitude: float | None = None
    pitch_peak_frequency_hz: float | None = None

    def as_row(self) -> dict[str, object]:
        """Return the metric fields as a flat mapping (override mapping included)."""

        row: dict[str, object] = {"study": self.study, "overrides": dict(self.overrides)}
        for name in _METRIC_FIELDS:
            row[name] = getattr(self, name)
        return row


@dataclass(frozen=True, slots=True)
class _StudySettings:
    """Everything a study needs besides the vehicle."""

    speed: float = 0.0
    step_force_n: float = DEFAULT_STEP_FORCE_N
    step_duration_s: float = DEFAULT_STEP_DURATION_S
    step_samples: int = DEFAULT_STEP_SAMPLES
    settling_band: float = DEFAULT_SETTLING_BAND
    frequency_grid_hz: tuple[float, float, int] = DEFAULT_FREQUENCY_GRID_HZ


@dataclass(frozen=True, slots=True)
class _LinearSubsystem:
    """The linearized seven-coordinate vertical subsystem of one setup."""

    position: np.ndarray
    mass: np.ndarray
    stiffness: np.ndarray
    damping: np.ndarray
    spring_rate: np.ndarray
    tire_rate: np.ndarray
    ride_frequency_hz: tuple[float, float]
    body_positions: np.ndarray

    @property
    def front_x(self) -> float:
        """Forward arm of the front axle from the sprung CG [m]."""

        return float(self.body_positions[0, 0])


# --------------------------------------------------------------------------- #
# Overrides
# --------------------------------------------------------------------------- #
def _corner_network(vehicle) -> IndependentCornerNetwork:
    """Return the independent-corner network; other layouts are not swept yet."""

    network = vehicle.suspension.network
    if not isinstance(network, IndependentCornerNetwork):
        raise ValueError(
            f"the setup sweep supports the independent-corner layout only, "
            f"not {vehicle.suspension.layout!r}"
        )
    return network


def _axle_scalar(vehicle, name: str, axle: int) -> float:
    """Return one axle scalar (``arb_wheel_rate``, ``inertance`` or ``static_toe``)."""

    network = _corner_network(vehicle)
    if name == "arb_wheel_rate":
        return float(network.arb.axle(axle))
    if name == "inertance":
        return float(network.inertance[axle])
    if name == "static_toe":
        return float(vehicle.suspension.kinematics.axle(axle).static_toe)
    raise ValueError(f"unknown axle scalar {name!r}")


def valid_override_names(vehicle) -> dict[str, str]:
    """Return ``{name: description}`` of every override the compiled model exposes.

    Click names come from each compiled damper table's valve names: a bare
    ``click_<valve>`` applies to every damper that has that valve, and
    ``front.click_<valve>`` / ``rear.click_<valve>`` address one damper. The
    suspension scalars are the ARB rate and inertance of the compiled
    independent-corner network and the static toe of its kinematics.
    """

    catalogue: dict[str, str] = {}
    common_valves: set[str] | None = None
    for index, axle_name in enumerate(("front", "rear")):
        damper = _corner_network(vehicle).dampers[index]
        valves = {valve.name: valve for valve in damper.table.valves}
        common_valves = set(valves) if common_valves is None else common_valves & set(valves)
        for name, valve in sorted(valves.items()):
            catalogue[f"{axle_name}.click_{name}"] = (
                f"{axle_name} damper {name!r} click (0..{valve.clicks - 1})"
            )
    for name in sorted(common_valves or ()):
        catalogue[f"click_{name}"] = f"both dampers, {name!r} click"
    for name in ("arb_wheel_rate", "inertance", "static_toe"):
        for axle, axle_name in enumerate(("front", "rear")):
            catalogue[f"{name}_{axle_name}"] = (
                f"compiled {axle_name} axle {name} ({_axle_scalar(vehicle, name, axle)!r})"
            )
    return catalogue


def _coerce_override(name: object, value: object) -> tuple[str, float]:
    """Return one ``(name, value)`` override with both parts validated as scalars."""

    if not isinstance(name, str) or not name:
        raise ValueError(f"override names must be non-empty strings, got {name!r}")
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as error:
        raise ValueError(f"override {name!r}: value {value!r} is not a number") from error
    if not np.isfinite(number):
        raise ValueError(f"override {name!r}: value {value!r} must be finite")
    return name, number


def apply_overrides(vehicle, overrides: Mapping[str, float]):
    """Return ``vehicle`` with the override mapping applied, or raise ``ValueError``.

    The compiled damper *tables* are shared and untouched: a click override only
    rebuilds the ``DamperSetting`` (``dataclasses.replace``), so the NPZ LUT
    pinned by sha256 is never rewritten. A click must be an integer inside
    ``0..clicks-1``; an ARB rate or inertance must be non-negative.
    """

    catalogue = valid_override_names(vehicle)
    requested = dict(overrides)
    unknown = sorted(name for name in requested if name not in catalogue)
    if unknown:
        raise ValueError(
            f"unknown override name(s) {unknown}; valid names are {sorted(catalogue)}"
        )
    network = _corner_network(vehicle)
    clicks = [dict(damper.setting.clicks) for damper in network.dampers]
    scalars = {
        (name, axle): _axle_scalar(vehicle, name, axle)
        for name in ("arb_wheel_rate", "inertance", "static_toe") for axle in (0, 1)
    }
    for raw_name, raw_value in requested.items():
        name, value = _coerce_override(raw_name, raw_value)
        axle_name, _, field = name.partition(".")
        if not field:
            axle_name, field = "", name
        if field.startswith("click_"):
            valve = field[len("click_"):]
            indices = (
                [0, 1] if not axle_name else ([0] if axle_name == "front" else [1])
            )
            for index in indices:
                clicks[index][valve] = value
            continue
        attribute = field
        if attribute.endswith("_front") or attribute.endswith("_rear"):
            axle_name = attribute.rsplit("_", 1)[1]
            attribute = attribute[: -len(axle_name) - 1]
        # Every catalogued non-click name carries the _front/_rear suffix, so
        # axle_name is always one of the two axles here: the catalogue check at
        # the top of this function rejected everything else. The former
        # `axle_name not in (...)` guard was unreachable and was removed.
        if attribute in ("arb_wheel_rate", "inertance") and value < 0.0:
            raise ValueError(f"override {name!r}: value {value} must be non-negative")
        scalars[(attribute, 0 if axle_name == "front" else 1)] = value
    dampers = tuple(
        CompiledDamper(table=damper.table, setting=DamperSetting(clicks[index]))
        for index, damper in enumerate(network.dampers)
    )
    for index, damper in enumerate(dampers):
        try:
            damper.setting.validate(damper.table)
        except ValueError as error:
            raise ValueError(f"override click for damper {index}: {error}") from error
    kinematics = vehicle.suspension.kinematics
    for axle in (0, 1):
        kinematics = kinematics.with_axle(axle, static_toe=scalars[("static_toe", axle)])
    network = replace(
        network,
        kinematics=kinematics,
        dampers=dampers,
        arb=AxleArbRates(scalars[("arb_wheel_rate", 0)], scalars[("arb_wheel_rate", 1)]),
        inertance=(scalars[("inertance", 0)], scalars[("inertance", 1)]),
    )
    return replace(
        vehicle,
        suspension=replace(vehicle.suspension, kinematics=kinematics, network=network),
    )


# --------------------------------------------------------------------------- #
# Linearization (Task 6) and the two studies
# --------------------------------------------------------------------------- #
def _static_vertical_state(vehicle, speed: float):
    """Return ``(q0, tires)`` at the speed-``speed`` static equilibrium."""

    q0 = solve_static_equilibrium(vehicle, float(speed)).as_vector()
    q0 = np.asarray(q0, dtype=float)
    zero = (TransientTireState(0.0, 0.0, 0.0),) * 4
    loads = evaluate_vertical_dynamics(
        vehicle, VerticalState.from_vector(q0), np.zeros(4), 0.0, zero
    ).normal_loads
    tires = tuple(
        steady_rolling_state(
            vehicle.planar.tire, vehicle.transient_tire, float(speed), 0.0, float(load)
        )
        for load in loads
    )
    return q0, tires


def _generalized_forces(vehicle, position: np.ndarray, tires) -> np.ndarray:
    """Return the seven vertical generalized forces at ``position`` with zero rates."""

    q = np.zeros(21)
    q[list(POSITION_ROWS)] = np.asarray(position, dtype=float)
    out = evaluate_vertical_dynamics(
        vehicle, VerticalState.from_vector(q), np.zeros(4), 0.0, tires
    )
    return np.asarray(out.generalized_forces, dtype=float)


def _tire_deflection(vehicle, position: np.ndarray) -> np.ndarray:
    """Return the four hub deflections [m] at the given vertical position."""

    positions = sprung_wheel_positions(vehicle)
    body = position[0] + positions[:, 1] * position[1] - positions[:, 0] * position[2]
    return body - np.asarray(position, dtype=float)[3:]


def _quarter_car_ride_frequency(vehicle, spring_rate: np.ndarray, tire_rate: np.ndarray) -> tuple[float, float]:
    """Return the two (front, rear) body-fixed quarter-car ride frequencies [Hz].

    The lower mode of ``[[k_w, -k_w], [-k_w, k_w + k_t]]`` on the corner masses
    is the ride mode; the higher mode is wheel hop. The unsprung mass is kept,
    so this is the exact two-DoF ride frequency rather than the series-spring
    estimate the plan's reference numbers quote (about 3 % higher).
    """

    frequencies = []
    for index in (0, 1):
        front = index == 0
        fraction = vehicle.sprung_front_fraction if front else 1.0 - vehicle.sprung_front_fraction
        sprung = vehicle.sprung_mass * fraction / 2.0
        unsprung = vehicle.unsprung_mass_front if front else vehicle.unsprung_mass_rear
        corner = 0 if front else 2
        stiffness = np.array(
            [
                [spring_rate[corner], -spring_rate[corner]],
                [-spring_rate[corner], spring_rate[corner] + tire_rate[corner]],
            ]
        )
        modes = np.sort(
            np.linalg.eigvals(
                np.linalg.solve(np.diag([sprung, unsprung]), stiffness)
            ).real
        )
        if modes[0] <= 0.0:
            raise ValueError(f"axle {index}: non-positive quarter-car ride frequency")
        frequencies.append(float(np.sqrt(modes[0])) / (2.0 * np.pi))
    return tuple(frequencies)  # type: ignore[return-value]


def _linear_subsystem(vehicle, speed: float) -> _LinearSubsystem:
    """Linearize the seven-coordinate vertical subsystem at the static equilibrium."""

    q0, tires = _static_vertical_state(vehicle, speed)
    position = np.asarray(q0)[list(POSITION_ROWS)]
    stiffness = np.empty((7, 7))
    for column in range(7):
        step = 1e-7 * (1.0 + abs(position[column]))
        plus, minus = position.copy(), position.copy()
        plus[column] += step
        minus[column] -= step
        stiffness[:, column] = -(
            _generalized_forces(vehicle, plus, tires)
            - _generalized_forces(vehicle, minus, tires)
        ) / (2.0 * step)
    positions = sprung_wheel_positions(vehicle)
    damping = np.zeros((7, 7))
    for index in range(4):
        column = np.zeros(7)
        column[0] = 1.0
        column[1] = positions[index, 1]
        column[2] = -positions[index, 0]
        column[3 + index] = -1.0
        damping += vehicle.tire_vertical_damping * np.outer(column, column)
    for index in range(4):
        damper = _corner_network(vehicle).dampers[0 if index < 2 else 1]
        ratio = vehicle.suspension.kinematics.motion_ratio(index, position[3 + index])
        slope = float(np.asarray(damper.force(np.zeros(1)).slope)[0])
        damping[3 + index, 3 + index] += slope / ratio**2
    spring_rate = np.empty(4)
    for axle, (left, right) in ((0, (0, 1)), (1, (2, 3))):
        spring_rate[[left, right]] = _corner_network(vehicle).springs.axle(axle).derivative(
            position[3:][[left, right]])
    tire_rate = np.asarray(
        tire_deflection_load(
            vehicle.planar.tire, _tire_deflection(vehicle, position), np.zeros(4), 0.0
        ).stiffness,
        dtype=float,
    )
    ride = _quarter_car_ride_frequency(vehicle, spring_rate, tire_rate)
    mass = np.asarray(vertical_mass_block(vehicle, position[3:]), dtype=float)
    return _LinearSubsystem(
        position=position,
        mass=mass,
        stiffness=stiffness,
        damping=damping,
        spring_rate=spring_rate,
        tire_rate=tire_rate,
        ride_frequency_hz=ride,
        body_positions=positions,
    )


def _corner_damping_ratio(vehicle, subsystem: _LinearSubsystem, index: int, velocity: float) -> float:
    """Return one axle's secant corner damping ratio at ``velocity`` [m/s].

    ``zeta = c_wheel / (2 m_s_corner omega_ride)`` with
    ``c_wheel = (|F(v_ref)| / |v_ref|) / MR^2`` from the compiled damper table.
    """

    front = index == 0
    damper = _corner_network(vehicle).dampers[0 if front else 1]
    force = float(np.asarray(damper.force(np.array([velocity])).force)[0])
    ratio = vehicle.suspension.kinematics.motion_ratio(0 if front else 2, 0.0)
    wheel_rate = (abs(force) / abs(velocity)) / ratio**2
    fraction = vehicle.sprung_front_fraction if front else 1.0 - vehicle.sprung_front_fraction
    sprung = vehicle.sprung_mass * fraction / 2.0
    omega = 2.0 * np.pi * subsystem.ride_frequency_hz[index]
    if sprung <= 0.0 or omega <= 0.0:
        raise ValueError(f"axle {index}: non-positive corner mass or ride frequency")
    return float(wheel_rate / (2.0 * sprung * omega))


def _first_order_form(subsystem: _LinearSubsystem) -> np.ndarray:
    """Return the 14x14 first-order matrix of ``M xddot + C xdot + K x = 0``."""

    n = 7
    matrix = np.zeros((2 * n, 2 * n))
    matrix[:n, n:] = np.eye(n)
    inverse = np.linalg.inv(subsystem.mass)
    matrix[n:, :n] = -inverse @ subsystem.stiffness
    matrix[n:, n:] = -inverse @ subsystem.damping
    return matrix


def _damped_modes(subsystem: _LinearSubsystem) -> dict[str, tuple[float, float, float]]:
    """Return ``{name: (frequency_hz, damping_ratio, damped_frequency_hz)}``.

    ``frequency_hz`` is ``|lambda| / (2*pi)``, the undamped-equivalent natural
    frequency of the damped pole, and ``damped_frequency_hz`` is
    ``Im(lambda) / (2*pi)`` (0 for an overdamped, real eigenvalue). Each named
    mode is the distinct eigenmode with the largest mass-weighted participation
    in that coordinate, assigned greedily by descending participation so the
    three names always resolve to three different modes.
    """

    matrix = _first_order_form(subsystem)
    eigenvalues, eigenvectors = np.linalg.eig(matrix)
    keep = eigenvalues.imag >= -1e-9
    eigenvalues, eigenvectors = eigenvalues[keep], eigenvectors[:, keep]
    shares = np.abs(eigenvectors[:7]) ** 2 * np.diag(subsystem.mass)[:, None]
    shares = shares / shares.sum(axis=0)
    # Keeping one eigenvalue per conjugate pair plus every real (overdamped)
    # eigenvalue gives 7 + r/2 modes for r real eigenvalues (r is always even):
    # 9 at the baseline (r = 4), 10 at click 8 (r = 6) and 11 at clicks 9-11
    # (r = 8). The mode range therefore comes from the data, never a fixed 7.
    candidates = sorted(
        (
            (float(shares[coordinate, mode]), coordinate, mode)
            for coordinate in range(3)
            for mode in range(eigenvalues.size)
        ),
        reverse=True,
    )
    assigned: dict[int, int] = {}
    used: set[int] = set()
    for _share, coordinate, mode in candidates:
        if coordinate in assigned or mode in used:
            continue
        assigned[coordinate] = mode
        used.add(mode)
    result: dict[str, tuple[float, float, float]] = {}
    for name, coordinate in MODE_INDICES.items():
        mode = assigned[coordinate]
        eigenvalue = eigenvalues[mode]
        magnitude = float(abs(eigenvalue))
        if magnitude <= 0.0:
            raise ValueError(f"{name}: zero eigenvalue in the linearized subsystem")
        result[name] = (
            magnitude / (2.0 * np.pi),
            float(-eigenvalue.real / magnitude),
            float(eigenvalue.imag) / (2.0 * np.pi),
        )
    return result


def _step_solution(subsystem: _LinearSubsystem, generalized: np.ndarray, times: np.ndarray) -> np.ndarray:
    """Return the ``(14, N)`` state trajectory of a constant generalized-force step.

    For ``z' = A z + B F`` with ``z(0) = 0`` and constant ``F``,
    ``z(t) = V diag((exp(lambda t) - 1) / lambda) V^-1 B F``. Closed form, NumPy
    only: no integrator and no SciPy.
    """

    n = 7
    matrix = _first_order_form(subsystem)
    forcing = np.zeros(2 * n)
    forcing[n:] = np.linalg.inv(subsystem.mass) @ np.asarray(generalized, dtype=float)
    eigenvalues, eigenvectors = np.linalg.eig(matrix)
    coefficients = np.linalg.solve(eigenvectors, forcing)
    exponentials = np.exp(np.outer(np.asarray(times, dtype=float), eigenvalues))
    small = np.abs(eigenvalues) <= 1e-12
    ratio = np.where(
        small,
        np.asarray(times, dtype=float)[:, None],
        (exponentials - 1.0) / np.where(small, 1.0, eigenvalues),
    )
    return np.real(eigenvectors @ (ratio * coefficients).T)


def _front_body_point(subsystem: _LinearSubsystem, states: np.ndarray) -> np.ndarray:
    """Return the front body-point displacement [m] of a ``(14, N)`` trajectory."""

    x_arm = float(subsystem.body_positions[0, 0])
    y_arm = float(subsystem.body_positions[0, 1])
    return states[0] + y_arm * states[1] - x_arm * states[2]


def step_response(
    vehicle,
    *,
    speed: float = 0.0,
    step_force_n: float = DEFAULT_STEP_FORCE_N,
    duration_s: float = DEFAULT_STEP_DURATION_S,
    samples: int = DEFAULT_STEP_SAMPLES,
) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(times, response)`` of the front body-point step response.

    The 2 000 N step is applied as the generalized force
    ``[F, 0, -F*x_front, 0, 0, 0, 0]`` on ``[heave, roll, pitch, jounce x4]``,
    exactly as Task 7's step test does.
    """

    subsystem = _linear_subsystem(vehicle, float(speed))
    times = np.linspace(0.0, float(duration_s), int(samples))
    generalized = np.zeros(7)
    generalized[0] = float(step_force_n)
    generalized[2] = -float(step_force_n) * subsystem.front_x
    states = _step_solution(subsystem, generalized, times)
    return times, _front_body_point(subsystem, states)


def _step_metrics(times: np.ndarray, response: np.ndarray, band: float) -> dict[str, float | None]:
    """Return peak, time to peak, overshoot, settling time, and steady state."""

    steady = float(response[-1])
    peak_index = int(np.argmax(response))
    peak = float(response[peak_index])
    tolerance = float(band) * abs(steady)
    outside = np.nonzero(np.abs(response - steady) > tolerance)[0]
    settled_index = int(outside[-1]) + 1 if outside.size else 0
    settled = float(times[settled_index]) if settled_index < times.size else None
    overshoot = peak / steady - 1.0 if steady != 0.0 else float("nan")
    return {
        "step_peak_m": peak,
        "step_time_to_peak_s": float(times[peak_index]),
        "step_overshoot_fraction": float(overshoot),
        "step_settling_time_s": settled,
        "step_steady_state_m": steady,
    }


def _frequency_peaks(subsystem: _LinearSubsystem, grid_hz: Sequence[float]) -> dict[str, tuple[float, float]]:
    """Return ``{name: (peak_magnitude, peak_frequency_hz)}`` on the given grid."""

    grid = np.asarray(grid_hz, dtype=float)
    omega = 2.0 * np.pi * grid
    matrices = (
        subsystem.stiffness[None, :, :]
        - (omega**2)[:, None, None] * subsystem.mass[None, :, :]
        + (1j * omega)[:, None, None] * subsystem.damping[None, :, :]
    )
    right_hand_side = np.broadcast_to(np.eye(7), (grid.size, 7, 7))
    response = np.linalg.solve(matrices, right_hand_side)
    peaks: dict[str, tuple[float, float]] = {}
    for name, index in MODE_INDICES.items():
        magnitude = np.abs(response[:, index, index])
        best = int(np.argmax(magnitude))
        peaks[name] = (float(magnitude[best]), float(grid[best]))
    return peaks


def _evaluate_setup(
    vehicle,
    overrides: Mapping[str, float],
    study: str,
    settings: _StudySettings,
) -> SweepResult:
    """Return one setup's :class:`SweepResult`, derived from the model."""

    subsystem = _linear_subsystem(vehicle, settings.speed)
    modes = _damped_modes(subsystem)
    compression_front = _corner_damping_ratio(vehicle, subsystem, 0, REFERENCE_DAMPER_VELOCITY)
    compression_rear = _corner_damping_ratio(vehicle, subsystem, 1, REFERENCE_DAMPER_VELOCITY)
    rebound_front = _corner_damping_ratio(vehicle, subsystem, 0, -REFERENCE_DAMPER_VELOCITY)
    rebound_rear = _corner_damping_ratio(vehicle, subsystem, 1, -REFERENCE_DAMPER_VELOCITY)
    fields: dict[str, float | None] = {
        "step_peak_m": None,
        "step_time_to_peak_s": None,
        "step_overshoot_fraction": None,
        "step_settling_time_s": None,
        "step_steady_state_m": None,
        "heave_peak_magnitude": None,
        "heave_peak_frequency_hz": None,
        "roll_peak_magnitude": None,
        "roll_peak_frequency_hz": None,
        "pitch_peak_magnitude": None,
        "pitch_peak_frequency_hz": None,
    }
    if study == "step-response":
        times = np.linspace(0.0, settings.step_duration_s, settings.step_samples)
        generalized = np.zeros(7)
        generalized[0] = settings.step_force_n
        generalized[2] = -settings.step_force_n * subsystem.front_x
        response = _front_body_point(
            subsystem, _step_solution(subsystem, generalized, times)
        )
        fields.update(_step_metrics(times, response, settings.settling_band))
    elif study == "frequency-response":
        minimum, maximum, samples = settings.frequency_grid_hz
        grid = np.linspace(float(minimum), float(maximum), int(samples))
        peaks = _frequency_peaks(subsystem, grid)
        fields["heave_peak_magnitude"], fields["heave_peak_frequency_hz"] = peaks["heave"]
        fields["roll_peak_magnitude"], fields["roll_peak_frequency_hz"] = peaks["roll"]
        fields["pitch_peak_magnitude"], fields["pitch_peak_frequency_hz"] = peaks["pitch"]
    else:  # pragma: no cover - guarded by the public entry point
        raise ValueError(f"unknown study {study!r}; choose one of {STUDIES}")
    return SweepResult(
        overrides=MappingProxyType(dict(overrides)),
        study=study,
        front_ride_frequency_hz=float(subsystem.ride_frequency_hz[0]),
        rear_ride_frequency_hz=float(subsystem.ride_frequency_hz[1]),
        heave_frequency_hz=float(modes["heave"][0]),
        heave_damping_ratio=float(modes["heave"][1]),
        roll_frequency_hz=float(modes["roll"][0]),
        roll_damping_ratio=float(modes["roll"][1]),
        pitch_frequency_hz=float(modes["pitch"][0]),
        pitch_damping_ratio=float(modes["pitch"][1]),
        heave_damped_frequency_hz=float(modes["heave"][2]),
        roll_damped_frequency_hz=float(modes["roll"][2]),
        pitch_damped_frequency_hz=float(modes["pitch"][2]),
        front_corner_damping_ratio=float(compression_front),
        front_corner_damping_ratio_rebound=float(rebound_front),
        rear_corner_damping_ratio=float(compression_rear),
        rear_corner_damping_ratio_rebound=float(rebound_rear),
        **fields,
    )


# --------------------------------------------------------------------------- #
# Study definition and source hashes
# --------------------------------------------------------------------------- #
def study_definition(study: str, settings: _StudySettings) -> dict:
    """Return the JSON description of what one study measures."""

    common = {
        "study": study,
        "speed_m_s": float(settings.speed),
        "linearization": (
            "seven-coordinate vertical subsystem at the static equilibrium: "
            "mass from vertical_mass_block, stiffness from the central-difference "
            "Jacobian of the evaluator generalized forces, damping from the tire "
            "vertical damping and each compiled damper's slope/MR^2 at zero velocity"
        ),
        "modal_frequency_convention": (
            "*_frequency_hz is |lambda|/(2*pi), the undamped-equivalent natural "
            "frequency of the damped pole; *_damped_frequency_hz is "
            "Im(lambda)/(2*pi) (0 for an overdamped mode)"
        ),
        "damping_branch_convention": (
            "the linearized damping matrix uses each damper table's "
            "compression-branch slope at zero velocity for both directions (the "
            "compiled table is kinked at v = 0: rebound 2346.7 vs compression "
            "2126.7 N s/m at click 5); averaging the two branch slopes instead "
            "moves the stiff-click first peak to 26.43 mm at 0.0767 s (click 11) "
            "instead of 25.87 mm at 0.3038 s, so the click-10/11 step peak and "
            "time-to-peak are convention-sensitive"
        ),
    }
    if study == "step-response":
        common.update(
            {
                "observable": (
                    "front-axle body-point vertical displacement "
                    "heave + y_front*roll - x_front*pitch, positive down [m]"
                ),
                "input": (
                    "2000 N downward step on the two front sprung-body points; "
                    "generalized force [F, 0, -F*x_front, 0, 0, 0, 0]"
                ),
                "step_force_n": float(settings.step_force_n),
                "duration_s": float(settings.step_duration_s),
                "samples": int(settings.step_samples),
                "settling_band_fraction": float(settings.settling_band),
                "settling_band_definition": (
                    "first sample after which |y - y_steady| <= band*|y_steady| for "
                    "every remaining sample of the window, else null"
                ),
                "solver": (
                    "pure-NumPy closed-form state-space step: "
                    "z(t) = V diag((exp(lambda t)-1)/lambda) V^-1 B F on the 14x14 "
                    "first-order form"
                ),
            }
        )
    elif study == "frequency-response":
        minimum, maximum, samples = settings.frequency_grid_hz
        resolution = (float(maximum) - float(minimum)) / (int(samples) - 1)
        common.update(
            {
                "observable": (
                    "displacement per generalized force "
                    "H_ii(omega) = e_i' (K - omega^2 M + i omega C)^-1 e_i; "
                    "units m/N (heave) and rad/(N*m) (roll, pitch)"
                ),
                "frequency_min_hz": float(minimum),
                "frequency_max_hz": float(maximum),
                "frequency_samples": int(samples),
                "frequency_resolution_hz": float(resolution),
                "peak_rule": "maximum-magnitude point of the fixed grid",
                "mode_assignment": (
                    "each named mode is the distinct eigenmode with the largest "
                    "mass-weighted participation in that coordinate, assigned "
                    "greedily by descending participation"
                ),
            }
        )
    else:
        raise ValueError(f"unknown study {study!r}; choose one of {STUDIES}")
    return common


def _collect_component_paths(
    model_path: Path, root: ElementTree.Element, paths: list[Path], seen: set[Path]
) -> None:
    """Append the data tables of ``model_path`` and, recursively, its component XMLs.

    Nested assemblies (a suspension assembly holds its kinematics, spring, ARB and
    damper children) are followed to any depth; ``seen`` stops a reference cycle.
    """

    if model_path in seen:
        return
    seen.add(model_path)
    for data in root.iter("data"):
        if data.get("path"):
            paths.append((model_path.parent / data.get("path")).resolve())
    for component in root.iter("component"):
        reference = component.get("ref")
        if not reference:
            continue
        component_path = (model_path.parent / reference).resolve()
        paths.append(component_path)
        try:
            component_root = ElementTree.parse(component_path).getroot()
        except (OSError, ElementTree.ParseError):
            continue
        _collect_component_paths(component_path, component_root, paths, seen)


def _model_input_paths(formulation_xml: str | Path) -> tuple[Path, ...]:
    """Return the formulation, assembly, component XMLs, and their data tables."""

    formulation = Path(formulation_xml).expanduser().resolve()
    paths: list[Path] = [formulation]
    try:
        root = ElementTree.parse(formulation).getroot()
    except (OSError, ElementTree.ParseError):
        return tuple(paths)
    for source in root.iter("source"):
        reference = source.get("ref")
        if not reference:
            continue
        assembly = (formulation.parent / reference).resolve()
        paths.append(assembly)
        try:
            assembly_root = ElementTree.parse(assembly).getroot()
        except (OSError, ElementTree.ParseError):
            continue
        _collect_component_paths(assembly, assembly_root, paths, set())
    unique: list[Path] = []
    for path in paths:
        if path not in unique:
            unique.append(path)
    return tuple(unique)


def source_hashes(formulation_xml: str | Path) -> dict[str, str]:
    """Return ``{absolute path: sha256}`` for the model inputs and tool sources.

    The model inputs are the formulation, the assembly, every component XML and
    every data table they declare (resolved through the XML graph, so a tmp
    synthetic formulation hashes the tmp files it actually compiled); the tool
    sources are the library modules the sweep's numbers depend on. Missing files
    are skipped rather than hashed as absent.
    """

    paths = list(_model_input_paths(formulation_xml))
    paths.extend(REPOSITORY_ROOT / relative for relative in _TOOL_SOURCES)
    digests: dict[str, str] = {}
    for path in paths:
        if not path.is_file():
            continue
        digests[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    return digests


# --------------------------------------------------------------------------- #
# Public entry point
# --------------------------------------------------------------------------- #
def sweep_vertical_setup(
    formulation_xml: str | Path,
    overrides: Iterable[Mapping[str, float]],
    study: Literal["step-response", "frequency-response"],
    *,
    speed: float = 0.0,
    step_force_n: float = DEFAULT_STEP_FORCE_N,
    step_duration_s: float = DEFAULT_STEP_DURATION_S,
    step_samples: int = DEFAULT_STEP_SAMPLES,
    settling_band: float = DEFAULT_SETTLING_BAND,
    frequency_grid_hz: Sequence[float] = DEFAULT_FREQUENCY_GRID_HZ,
    output_csv: str | Path | None = None,
    output_json: str | Path | None = None,
) -> list[SweepResult]:
    """Run one study for each override mapping and return one result per setup.

    ``overrides`` is an iterable of mappings of parameter names to values, in the
    order the results come back. ``study`` selects the step-response or the
    frequency-response metrics. When ``output_csv`` / ``output_json`` are given,
    the CSV and the JSON summary (inputs, results, tool version, source hashes,
    and the study definition) are written as well.
    """

    if study not in STUDIES:
        raise ValueError(f"unknown study {study!r}; choose one of {STUDIES}")
    settings = _StudySettings(
        speed=float(speed),
        step_force_n=float(step_force_n),
        step_duration_s=float(step_duration_s),
        step_samples=int(step_samples),
        settling_band=float(settling_band),
        frequency_grid_hz=tuple(float(value) for value in frequency_grid_hz),  # type: ignore[arg-type]
    )
    vehicle = compile_formulation(formulation_xml).model
    results: list[SweepResult] = []
    for mapping in overrides:
        requested = dict(_validated_mapping(mapping))
        results.append(
            _evaluate_setup(apply_overrides(vehicle, requested), requested, study, settings)
        )
    if output_csv is not None:
        write_csv(results, output_csv)
    if output_json is not None:
        write_json(
            results,
            output_json,
            formulation_xml=formulation_xml,
            study=study,
            study_definition=study_definition(study, settings),
            source_hashes=source_hashes(formulation_xml),
        )
    return results


def _validated_mapping(mapping: Mapping[str, float]) -> list[tuple[str, float]]:
    """Return one override mapping as validated ``(name, value)`` pairs."""

    if not isinstance(mapping, Mapping):
        raise ValueError(f"each setup must be a mapping of override names, got {mapping!r}")
    return [_coerce_override(name, value) for name, value in mapping.items()]


# --------------------------------------------------------------------------- #
# Output
# --------------------------------------------------------------------------- #
def _cell(value: object) -> str:
    """Return the CSV cell of one metric value (``""`` for a missing study field)."""

    if value is None:
        return ""
    if isinstance(value, str):
        return value
    return repr(float(value))  # type: ignore[arg-type]


def write_csv(results: Sequence[SweepResult], path: str | Path) -> Path:
    """Write one CSV row per setup (override columns, then study and metrics)."""

    names = sorted({name for result in results for name in result.overrides})
    fieldnames = names + ["study"] + list(_METRIC_FIELDS)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for result in results:
            row = {name: _cell(result.overrides.get(name)) for name in names}
            row["study"] = result.study
            for field in _METRIC_FIELDS:
                row[field] = _cell(getattr(result, field))
            writer.writerow(row)
    return path


def write_json(
    results: Sequence[SweepResult],
    path: str | Path,
    *,
    formulation_xml: str | Path,
    study: str,
    study_definition: dict,
    source_hashes: Mapping[str, str],
) -> Path:
    """Write the JSON summary: inputs, results, hashes, and the study definition."""

    document = {
        "tool": "sweep_vertical_setup",
        "tool_version": TOOL_VERSION,
        "formulation": str(Path(formulation_xml).expanduser().resolve()),
        "study": study,
        "study_definition": study_definition,
        "source_hashes": {str(name): str(value) for name, value in source_hashes.items()},
        "overrides": [dict(result.overrides) for result in results],
        "results": [
            {**{name: getattr(result, name) for name in _METRIC_FIELDS},
             "study": result.study,
             "overrides": dict(result.overrides)}
            for result in results
        ],
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
    return path


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def _overrides_from_arguments(args) -> list[dict[str, float]]:
    """Return the setup list from ``--overrides-json`` or ``--grid``."""

    if args.grid is not None:
        try:
            spec = json.loads(args.grid)
        except json.JSONDecodeError as error:
            raise ValueError(f"--grid is not valid JSON: {error}") from error
        if not isinstance(spec, dict) or not spec:
            raise ValueError("--grid must be a non-empty JSON object")
        names = sorted(spec)
        values = []
        for name in names:
            entry = spec[name]
            if isinstance(entry, (list, tuple)):
                if not entry:
                    raise ValueError(f"--grid entry {name!r} is an empty list")
                values.append(list(entry))
            else:
                values.append([entry])
        return [
            {name: value for name, value in zip(names, combination)}
            for combination in itertools.product(*values)
        ]
    if args.overrides_json is not None:
        text = args.overrides_json
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            candidate = Path(text)
            if not candidate.is_file():
                raise ValueError(
                    "--overrides-json must be a JSON list or a path to a JSON file"
                ) from None
            payload = json.loads(candidate.read_text())
        if not isinstance(payload, list) or not payload:
            raise ValueError("--overrides-json must be a non-empty JSON list of objects")
        for entry in payload:
            if not isinstance(entry, dict):
                raise ValueError(f"--overrides-json entries must be objects, got {entry!r}")
        return payload
    return [{}]


def build_parser() -> argparse.ArgumentParser:
    """Return the command-line parser of the sweep tool."""

    parser = argparse.ArgumentParser(
        prog="sweep_vertical_setup.py",
        description=(
            "Damper click, ARB, inertance and static-toe setup sweep over the "
            "vertical 14-DoF linearization. A spring-rate override is not available "
            "(the spring curve is inside the pinned corner NPZ). It writes a CSV row "
            "per setup and a JSON summary. It does not run laps."
        ),
    )
    parser.add_argument("--formulation", default=str(DEFAULT_FORMULATION),
                        help="vehicle formulation XML (default: 14dof-simple)")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--overrides-json", default=None,
                       help="JSON list of override mappings, or a path to such a file")
    group.add_argument("--grid", default=None,
                       help="JSON object mapping override name to one value or a list")
    parser.add_argument("--study", required=True, choices=STUDIES)
    parser.add_argument("--speed", type=float, default=0.0,
                        help="speed [m/s] of the static equilibrium the tool linearizes")
    parser.add_argument("--step-force-n", type=float, default=DEFAULT_STEP_FORCE_N)
    parser.add_argument("--step-duration-s", type=float, default=DEFAULT_STEP_DURATION_S)
    parser.add_argument("--step-samples", type=int, default=DEFAULT_STEP_SAMPLES)
    parser.add_argument("--settling-band", type=float, default=DEFAULT_SETTLING_BAND)
    parser.add_argument("--frequency-min-hz", type=float, default=DEFAULT_FREQUENCY_GRID_HZ[0])
    parser.add_argument("--frequency-max-hz", type=float, default=DEFAULT_FREQUENCY_GRID_HZ[1])
    parser.add_argument("--frequency-samples", type=int, default=DEFAULT_FREQUENCY_GRID_HZ[2])
    parser.add_argument("--output-csv", default=None)
    parser.add_argument("--output-json", default=None)
    return parser


def _print_report(results: Sequence[SweepResult], study: str, elapsed: float) -> None:
    """Print a compact per-setup table."""

    print(f"study: {study}")
    print(f"setups: {len(results)}")
    for result in results:
        overrides = ", ".join(f"{name}={value:g}" for name, value in sorted(result.overrides.items()))
        if study == "step-response":
            headline = (
                f"peak={result.step_peak_m * 1e3:.3f} mm t_peak={result.step_time_to_peak_s:.4f} s "
                f"overshoot={result.step_overshoot_fraction:.4f} "
                f"settling={result.step_settling_time_s if result.step_settling_time_s is None else round(result.step_settling_time_s, 4)} s"
            )
        else:
            headline = (
                f"heave={result.heave_peak_magnitude:.4g} m/N @ {result.heave_peak_frequency_hz:.3f} Hz "
                f"roll={result.roll_peak_magnitude:.4g} @ {result.roll_peak_frequency_hz:.3f} Hz "
                f"pitch={result.pitch_peak_magnitude:.4g} @ {result.pitch_peak_frequency_hz:.3f} Hz"
            )
        print(
            f"  [{overrides or 'baseline'}] zeta_corner F/R="
            f"{result.front_corner_damping_ratio:.4f}/{result.rear_corner_damping_ratio:.4f} {headline}"
        )
    print(f"elapsed: {elapsed:.2f} s")


def main(argv: Sequence[str] | None = None) -> int:
    """Run the CLI; return 0 on success and 2 on an invalid override or spec."""

    parser = build_parser()
    args = parser.parse_args(argv)
    started = time.perf_counter()
    try:
        setups = _overrides_from_arguments(args)
        results = sweep_vertical_setup(
            args.formulation,
            setups,
            args.study,
            speed=args.speed,
            step_force_n=args.step_force_n,
            step_duration_s=args.step_duration_s,
            step_samples=args.step_samples,
            settling_band=args.settling_band,
            frequency_grid_hz=(args.frequency_min_hz, args.frequency_max_hz,
                               args.frequency_samples),
            output_csv=args.output_csv,
            output_json=args.output_json,
        )
    except ValueError as error:
        print(f"sweep_vertical_setup: {error}", file=sys.stderr)
        return 2
    elapsed = time.perf_counter() - started
    _print_report(results, args.study, elapsed)
    if args.output_csv is not None:
        print(f"csv: {args.output_csv}")
    if args.output_json is not None:
        print(f"json: {args.output_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
