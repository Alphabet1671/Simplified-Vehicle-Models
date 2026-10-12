"""Model a voltage-dependent electric motor.

The identification voltages are line-to-line peak values. Runtime equations
use per-phase peak values, so fitted flux and inductance values are divided by
square root of three before the voltage solve. Resistance remains physical.
"""

from __future__ import annotations

import csv
import hashlib
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from simplified_vehicle_models.compiler.capabilities import TORQUE_SOURCE
from simplified_vehicle_models.compiler.registry import register_component
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    closed_parameters,
    require_no_references,
)

_ROOT = Path(__file__).resolve().parents[3]
_DATA = _ROOT / "models" / "data"
_SQRT_THREE = math.sqrt(3.0)


@dataclass(frozen=True, slots=True)
class DirectCurrentResult:
    """Report the direct current and voltage feasibility."""

    direct_current: float
    feasible: bool


@dataclass(frozen=True, slots=True)
class ShaftPowerResult:
    """Report shaft power, losses, and the active operating limit."""

    bus_power: float
    loss_power: float
    field_weakening_correction: float
    direct_current: float
    efficiency: float
    active_limit: str
    feasible: bool = True


@dataclass(frozen=True, slots=True)
class MotorOperatingMap:
    """Validated immutable normalized-load motor operating table."""

    voltage: np.ndarray
    speed: np.ndarray
    load: np.ndarray
    drive_torque: np.ndarray
    regen_torque: np.ndarray
    bus_power: np.ndarray
    loss_power: np.ndarray
    regen_feasible: np.ndarray


@dataclass(frozen=True, slots=True)
class PowerPlantParameters:
    """Store motor maps and identified machine parameters."""

    unit_count: int
    maximum_motor_speed: float
    unit_power_limit: float
    inverter_efficiency: float
    device_drop: float
    pole_pairs: int
    reference_bus_voltage: float
    resistance: float
    flux_linkage: float
    direct_inductance: float
    quadrature_inductance: float
    torque_speed: tuple[float, ...]
    torque_values: tuple[float, ...]
    loss_speed: tuple[float, ...]
    loss_torque: tuple[float, ...]
    loss_values: tuple[float, ...]
    measured_reference_speed: float
    measured_reference_direct_current: float
    measured_reference_quadrature_current: float
    measured_reference_direct_voltage: float
    measured_reference_quadrature_voltage: float
    measured_reference_voltage: float
    operating_map: MotorOperatingMap | None = None

    @property
    def phase_flux_linkage(self) -> float:
        """Return flux linkage on the per-phase voltage basis."""

        return self.flux_linkage / _SQRT_THREE

    @property
    def phase_direct_inductance(self) -> float:
        """Return direct inductance on the per-phase voltage basis."""

        return self.direct_inductance / _SQRT_THREE

    @property
    def phase_quadrature_inductance(self) -> float:
        """Return quadrature inductance on the per-phase voltage basis."""

        return self.quadrature_inductance / _SQRT_THREE


def _read(name):
    with (_DATA / name).open(newline="") as stream:
        return [
            row
            for row in csv.reader(stream)
            if row and not row[0].startswith("#")
            and row[0] not in {"name", "speed_rpm"}
        ]


def _parameter_file():
    return {
        row[0]: float(row[1])
        for row in _read("amk_dd5_100c_machine_parameters.csv")
    }


def _interp(x_axis, y_axis, value):
    if value < x_axis[0] or value > x_axis[-1]:
        raise ValueError("motor speed is outside the map")
    return float(np.interp(value, x_axis, y_axis))


_LOSS_GRID_CACHE = {}


def _loss_grid(parameters):
    key = id(parameters)
    grid = _LOSS_GRID_CACHE.get(key)
    if grid is not None:
        return grid
    rows = {}
    for row_speed, torque_value, loss_value in zip(
        parameters.loss_speed, parameters.loss_torque, parameters.loss_values
    ):
        rows.setdefault(float(row_speed), []).append((float(torque_value), float(loss_value)))
    speeds = np.asarray(sorted(rows), dtype=float)
    torque_rows = []
    loss_rows = []
    for speed in speeds:
        pairs = sorted(rows[float(speed)])
        torque_rows.append(np.asarray([pair[0] for pair in pairs], dtype=float))
        loss_rows.append(np.asarray([pair[1] for pair in pairs], dtype=float))
    grid = (speeds, tuple(torque_rows), tuple(loss_rows))
    _LOSS_GRID_CACHE[key] = grid
    return grid


def _loss_interpolate(parameters, speed, torque):
    speeds, torque_rows, loss_rows = _loss_grid(parameters)
    speed = min(float(speeds[-1]), max(float(speeds[0]), speed))
    speed_index = int(np.searchsorted(speeds, speed, side="right"))
    upper = min(len(speeds) - 1, max(1, speed_index))
    lower = upper - 1
    lower_value = float(np.interp(torque, torque_rows[lower], loss_rows[lower]))
    upper_value = float(np.interp(torque, torque_rows[upper], loss_rows[upper]))
    if upper == lower:
        return lower_value
    fraction = (speed - speeds[lower]) / (speeds[upper] - speeds[lower])
    return lower_value + fraction * (upper_value - lower_value)


def _map_values(speed, torque, parameters):
    if speed > parameters.maximum_motor_speed:
        raise ValueError("motor speed exceeds maximum_motor_speed")
    speed = max(speed, parameters.torque_speed[0])
    envelope = _interp(parameters.torque_speed, parameters.torque_values, speed)
    loss = _loss_interpolate(
        parameters, max(speed, parameters.loss_speed[0]), abs(torque)
    )
    return envelope, loss


def calculate_available_voltage(parameters, bus_voltage, current):
    """Return available per-phase peak voltage.

    ``current`` is the signed q-axis current estimate. It provides the
    resistive drop before the direct-axis solve and is refined by that solve.
    """

    return (
        bus_voltage / _SQRT_THREE
        - parameters.resistance * current
        - parameters.device_drop
    )


def maximum_motor_torque(parameters, motor_speed, available_voltage):
    """Scale the map envelope along its speed axis."""

    _map_values(motor_speed, 0.0, parameters)
    if available_voltage <= 0.0:
        return 0.0
    reference_speed = (
        motor_speed * parameters.reference_bus_voltage / _SQRT_THREE / available_voltage
    )
    if reference_speed > parameters.torque_speed[-1]:
        return 0.0
    return _interp(
        parameters.torque_speed,
        parameters.torque_values,
        max(reference_speed, parameters.torque_speed[0]),
    )


def solve_direct_current(
    parameters,
    motor_speed,
    quadrature_current,
    available_voltage,
):
    """Solve the d-axis voltage equation and report infeasibility."""

    electrical_speed = (
        parameters.pole_pairs * 2.0 * math.pi * max(motor_speed, 0.0) / 60.0
    )
    resistance = parameters.resistance
    direct_inductance = parameters.phase_direct_inductance
    quadrature_inductance = parameters.phase_quadrature_inductance
    flux_linkage = parameters.phase_flux_linkage
    direct_term = electrical_speed * direct_inductance
    quadrature_term = electrical_speed * quadrature_inductance
    a = resistance**2 + direct_term**2
    b = (
        -2.0 * quadrature_term * quadrature_current * resistance
        + 2.0 * direct_term * (
            resistance * quadrature_current + electrical_speed * flux_linkage
        )
    )
    c = (
        (-quadrature_term * quadrature_current) ** 2
        + (resistance * quadrature_current + electrical_speed * flux_linkage) ** 2
        - available_voltage**2
    )
    if abs(a) < 1.0e-30:
        return DirectCurrentResult(0.0, c <= 0.0)
    discriminant = b**2 - 4.0 * a * c
    if discriminant < 0.0:
        return DirectCurrentResult(0.0, False)
    roots = (
        (-b - math.sqrt(discriminant)) / (2.0 * a),
        (-b + math.sqrt(discriminant)) / (2.0 * a),
    )
    return DirectCurrentResult(min(roots, key=abs), True)


def evaluate_shaft_power_exact(parameters, motor_speed, motor_torque, bus_voltage):
    """Analytical motor oracle: evaluate power or report voltage infeasibility."""

    quadrature_current = motor_torque / (
        1.5 * parameters.pole_pairs * parameters.phase_flux_linkage
    )
    available_voltage = calculate_available_voltage(
        parameters, bus_voltage, quadrature_current
    )
    current = solve_direct_current(
        parameters, motor_speed, quadrature_current, available_voltage
    )
    _, map_loss = _map_values(motor_speed, motor_torque, parameters)
    shaft_power = motor_speed * 2.0 * math.pi / 60.0 * motor_torque
    if not current.feasible:
        return ShaftPowerResult(
            0.0,
            map_loss,
            0.0,
            current.direct_current,
            0.0,
            "voltage",
            False,
        )
    reference_voltage = calculate_available_voltage(
        parameters, parameters.reference_bus_voltage, quadrature_current
    )
    reference = solve_direct_current(
        parameters, motor_speed, quadrature_current, reference_voltage
    )
    if not reference.feasible:
        return ShaftPowerResult(
            0.0,
            map_loss,
            0.0,
            current.direct_current,
            0.0,
            "voltage",
            False,
        )
    correction = (
        0.0
        if abs(bus_voltage - parameters.reference_bus_voltage) < 1.0e-12
        else parameters.resistance
        * 1.5
        * (current.direct_current**2 - reference.direct_current**2)
    )
    # Voltage correction represents an added physical loss. A lower current
    # estimate cannot make total loss negative.
    loss_power = map_loss + max(correction, 0.0)
    raw_bus_power = shaft_power + loss_power
    if raw_bus_power >= 0.0:
        bus_power = raw_bus_power / parameters.inverter_efficiency
        efficiency = shaft_power / bus_power if bus_power else 1.0
    else:
        bus_power = raw_bus_power * parameters.inverter_efficiency
        efficiency = bus_power / shaft_power if shaft_power else 1.0
    active_limit = (
        "power" if abs(shaft_power) >= parameters.unit_power_limit else "torque"
    )
    return ShaftPowerResult(
        bus_power,
        loss_power,
        correction,
        current.direct_current,
        efficiency,
        active_limit,
        True,
    )


_LOOKUP_COUNTERS = {"forward": 0, "inverse": 0, "fallback": 0}


@dataclass(frozen=True, slots=True)
class TorqueLimits:
    maximum_drive_torque: float
    maximum_regen_torque: float
    used_fallback: bool


@dataclass(frozen=True, slots=True)
class TorqueSolveResult:
    torque: float
    operating_point: ShaftPowerResult
    residual: float
    feasible: bool
    active_limit: str
    used_fallback: bool


def reset_motor_lut_counters():
    """Reset production LUT lookup counters used by validation benchmarks."""
    for key in _LOOKUP_COUNTERS:
        _LOOKUP_COUNTERS[key] = 0


def motor_lut_counters():
    """Return a snapshot of forward, inverse, and fallback lookup counts."""
    return dict(_LOOKUP_COUNTERS)


def _bracket(axis, value):
    index = int(np.searchsorted(axis, value, side="right")) - 1
    index = max(0, min(index, len(axis) - 2))
    fraction = (value - axis[index]) / (axis[index + 1] - axis[index])
    return index, fraction


def _bilinear(values, voltage_index, speed_index, vf, sf):
    return float(
        (1.0 - vf) * (1.0 - sf) * values[voltage_index, speed_index]
        + vf * (1.0 - sf) * values[voltage_index + 1, speed_index]
        + (1.0 - vf) * sf * values[voltage_index, speed_index + 1]
        + vf * sf * values[voltage_index + 1, speed_index + 1]
    )


def _interpolated_envelopes(mapping, voltage, speed):
    vi, vf = _bracket(mapping.voltage, voltage)
    si, sf = _bracket(mapping.speed, speed)
    return (
        _bilinear(mapping.drive_torque, vi, si, vf, sf),
        _bilinear(mapping.regen_torque, vi, si, vf, sf),
        vi, si, vf, sf,
    )


_ROW_MASK_TOLERANCE = 1.0e-12


def _row_power_tolerance(magnitude):
    """Return the tolerated row/forward bus-power disagreement in watts.

    ``_validate_operating_map`` admits up to 1.0e-9 W of monotonicity slack per
    drive node and 1.0e-9 W of productive-regen slack per node.  Bilinear
    interpolation over the four voltage/speed corners keeps that bound, and
    float64 accumulation at map magnitudes near 1.0e5 W adds roughly 1.0e-11 W.
    A tolerance of 1.0e-6 W is therefore several orders of magnitude above the
    map's own numerical slack, while still negligible against the ~100 W
    interpolation error of the underlying map.  It exists so an inverse solved
    on the row is never rejected merely because the trilinear forward evaluator
    reconstructs the load from a rounded torque.
    """

    return 1.0e-6 + 1.0e-12 * abs(float(magnitude))


def _interpolated_row(array, vi, si, vf, sf):
    """Return one load row using one vectorized four-corner interpolation.

    This is the same convex combination as ``_bilinear`` evaluated over the
    whole load axis at once, replacing a per-node scalar corner accessor.
    """

    return (
        (1.0 - vf) * (1.0 - sf) * array[vi, si, :]
        + vf * (1.0 - sf) * array[vi + 1, si, :]
        + (1.0 - vf) * sf * array[vi, si + 1, :]
        + vf * sf * array[vi + 1, si + 1, :]
    )


def _interpolated_regen_mask(mapping, vi, si, vf, sf):
    """Return the conservative regen feasibility mask for one requested point."""

    weighted = _interpolated_row(
        mapping.regen_feasible.astype(np.float64), vi, si, vf, sf
    )
    return weighted >= 1.0 - _ROW_MASK_TOLERANCE


def _segment_load(row, load, k):
    """Return the load-axis fraction at which row segment ``k`` reaches ``load``."""

    if row[k + 1] == row[k]:
        return 0.0
    return (load - row[k]) / (row[k + 1] - row[k])


def _exact_fallback(parameters, motor_speed, shaft_torque, bus_voltage):
    """Evaluate the analytical oracle and account for one exact fallback."""
    _LOOKUP_COUNTERS["fallback"] += 1
    return evaluate_shaft_power_exact(parameters, motor_speed, shaft_torque, bus_voltage)


def _lut_forward(parameters, motor_speed, shaft_torque, bus_voltage):
    mapping = parameters.operating_map
    if mapping is None:
        return _exact_fallback(parameters, motor_speed, shaft_torque, bus_voltage)
    if not (mapping.voltage[0] <= bus_voltage <= mapping.voltage[-1] and
            mapping.speed[0] <= motor_speed <= mapping.speed[-1]):
        return _exact_fallback(parameters, motor_speed, shaft_torque, bus_voltage)
    drive, regen, vi, si, vf, sf = _interpolated_envelopes(
        mapping, bus_voltage, motor_speed
    )
    if shaft_torque == 0.0:
        load = 0.0
    else:
        envelope = drive if shaft_torque > 0.0 else regen
        if envelope <= 0.0:
            return _exact_fallback(parameters, motor_speed, shaft_torque, bus_voltage)
        load = (
            shaft_torque / drive
            if shaft_torque > 0.0
            else -abs(shaft_torque) / regen
        )
    if load < mapping.load[0] or load > mapping.load[-1]:
        return _exact_fallback(parameters, motor_speed, shaft_torque, bus_voltage)
    li, lf = _bracket(mapping.load, load)
    def interp3(array):
        a = array[:, :, li]
        b = array[:, :, li + 1]
        return (1.0 - lf) * _bilinear(a, vi, si, vf, sf) + lf * _bilinear(b, vi, si, vf, sf)
    bus = float(interp3(mapping.bus_power))
    loss = float(interp3(mapping.loss_power))
    shaft = motor_speed * 2.0 * math.pi / 60.0 * shaft_torque
    if abs(bus) < 1.0e-12:
        efficiency = 1.0
    elif shaft >= 0.0:
        efficiency = shaft / bus
    else:
        efficiency = bus / shaft
    _LOOKUP_COUNTERS["forward"] += 1
    return ShaftPowerResult(bus, loss, 0.0, 0.0, efficiency,
                            "power" if abs(shaft) >= parameters.unit_power_limit else "torque", True)


def evaluate_motor(parameters, motor_speed_rpm, shaft_torque, bus_voltage):
    """Evaluate a motor through the persisted LUT, with exact fallback."""
    return _lut_forward(parameters, motor_speed_rpm, shaft_torque, bus_voltage)


_EXACT_LIMIT_CACHE = {}
_EXACT_LIMIT_CACHE_MAX = 65536
_LIMIT_CACHE_QUANTUM = 1.0e-3
_MAP_IDENTITY_CACHE = {}


def _operating_map_identity(mapping):
    """Return a content identity for one compiled operating map.

    Caching the exact limit needs a key that cannot collide when a discarded
    motor's Python address is reused by a freshly compiled one.  A content
    digest of the map arrays is such a key.  The digest is computed once per
    distinct map object and kept in a bounded table so repeated calls during a
    lap stay cheap.
    """

    identity = _MAP_IDENTITY_CACHE.get(id(mapping))
    if identity is not None:
        return identity
    digest = hashlib.sha256()
    for name in ("voltage", "speed", "load", "drive_torque", "regen_torque",
                 "bus_power", "loss_power", "regen_feasible"):
        array = np.ascontiguousarray(getattr(mapping, name))
        digest.update(name.encode("ascii"))
        digest.update(str(array.shape).encode("ascii"))
        digest.update(array.tobytes())
    identity = digest.hexdigest()
    if len(_MAP_IDENTITY_CACHE) >= 64:
        _MAP_IDENTITY_CACHE.clear()
    _MAP_IDENTITY_CACHE[id(mapping)] = identity
    return identity


def _exact_limit(parameters, speed, voltage, regen=False):
    """Return the exact directional torque limit at one operating point.

    The analytical bisection costs dozens of oracle evaluations and a lap
    revisits the same voltage-coupling operating point thousands of times, so
    results are memoized on a 1.0e-3 V/rpm-quantized key.  That quantum is far
    below the maps' own accuracy while keeping the cache bounded and observable.
    Entries are keyed by the map's content identity, so a cache entry can never
    be served to a differently compiled motor.
    """

    mapping = parameters.operating_map
    if mapping is None:
        return _exact_limit_uncached(parameters, speed, voltage, regen)
    map_identity = _operating_map_identity(mapping)
    values = _EXACT_LIMIT_CACHE.get(map_identity)
    if values is None:
        if len(_EXACT_LIMIT_CACHE) >= _EXACT_LIMIT_CACHE_MAX:
            _EXACT_LIMIT_CACHE.clear()
        values = {}
        _EXACT_LIMIT_CACHE[map_identity] = values
    key = (
        regen,
        round(float(speed) / _LIMIT_CACHE_QUANTUM),
        round(float(voltage) / _LIMIT_CACHE_QUANTUM),
    )
    cached = values.get(key)
    if cached is None:
        cached = _exact_limit_uncached(parameters, speed, voltage, regen)
        values[key] = cached
    return cached


def _exact_limit_uncached(parameters, speed, voltage, regen=False):
    envelope = _map_values(max(0.0, speed), 0.0, parameters)[0]
    if envelope <= 0.0:
        return 0.0
    lo, hi = (-envelope, 0.0) if regen else (0.0, envelope)
    feasible_limit = 0.0
    # 24 bisections resolve the limit to ~1e-7 of the directional envelope,
    # far below the maps' own accuracy, at half the analytical cost.
    for _ in range(24):
        mid = (lo + hi) * 0.5
        if _exact_fallback(parameters, speed, mid, voltage).feasible:
            feasible_limit = mid
            if regen:
                hi = mid
            else:
                lo = mid
        elif regen:
            lo = mid
        else:
            hi = mid
    return abs(feasible_limit)


def lut_bus_power_range(parameters, motor_speed_rpm, bus_voltage):
    """Return the achievable drive bus-power row range at one requested point.

    The returned ``(zero_load, maximum)`` pair comes from the same bilinear row
    the inverse solves on, so callers can decide whether a requested storage
    power crosses the mapped motor capability without invoking the inverse (and
    its exact fallback) for an unreachable target.  ``None`` is returned when no
    LUT is compiled.
    """

    mapping = parameters.operating_map
    if mapping is None:
        return None
    if not (
        mapping.voltage[0] <= bus_voltage <= mapping.voltage[-1]
        and mapping.speed[0] <= motor_speed_rpm <= mapping.speed[-1]
    ):
        return None
    vi, vf = _bracket(mapping.voltage, bus_voltage)
    si, sf = _bracket(mapping.speed, motor_speed_rpm)
    row = _interpolated_row(mapping.bus_power, vi, si, vf, sf)
    zero = int(np.searchsorted(mapping.load, 0.0))
    return float(row[zero]), float(np.max(row))


def motor_torque_limits(parameters, motor_speed_rpm, bus_voltage):
    mapping = parameters.operating_map
    # The boundary tolerance accepts a query a few ulps outside the axis.  The
    # assembled motor speed lands exactly on 18,600 rpm, which is simultaneously
    # the map's last speed node and ``maximum_motor_speed``; without the
    # tolerance that single operating point would run a full analytical
    # bisection on every voltage-coupling pass.
    boundary = 1.0e-6 * max(1.0, abs(mapping.speed[-1]) if mapping is not None else 1.0)
    in_map = (
        mapping is not None
        and mapping.voltage[0] - 1.0e-9 <= bus_voltage <= mapping.voltage[-1] + 1.0e-9
        and mapping.speed[0] - boundary <= motor_speed_rpm <= mapping.speed[-1] + boundary
    )
    if not in_map:
        return TorqueLimits(
            _exact_limit(parameters, motor_speed_rpm, bus_voltage),
            _exact_limit(parameters, motor_speed_rpm, bus_voltage, True),
            True,
        )
    clamped_speed = min(max(motor_speed_rpm, mapping.speed[0]), mapping.speed[-1])
    clamped_voltage = min(max(bus_voltage, mapping.voltage[0]), mapping.voltage[-1])
    drive, regen, *_ = _interpolated_envelopes(mapping, clamped_voltage, clamped_speed)
    return TorqueLimits(drive, regen, False)


def _fallback_fractions(parameters):
    """Return deterministic dimensionless sample points along a limit.

    A uniform grid resolves the source envelope, combined with the normalized
    breakpoints of the compact torque-envelope map.  The loss map is sampled row
    by row and stores thousands of torque knots, so it is deliberately excluded:
    using every loss knot made one exact fallback thousands of analytical
    evaluations long.  The grid is coarse enough to keep the total fallback cost
    bounded, and the bracketed bisection removes any residual sampling error.
    """

    fractions = {index / 16.0 for index in range(17)}
    envelope_axis = parameters.torque_values
    last = max(abs(float(envelope_axis[-1])), 1.0e-12)
    for value in envelope_axis:
        fraction = abs(float(value)) / last
        if 0.0 < fraction < 1.0:
            fractions.add(fraction)
    return sorted(fractions)


_DRIVE_FALLBACK_STEPS = 24


def _drive_fallback_torque(parameters, speed, voltage, target, maximum):
    """Bisect the monotone drive bus-power curve for one exact fallback.

    Drive bus power is monotone increasing in torque up to the directional
    envelope, so a single bounded bisection finds the largest feasible torque
    without sampling the whole range.
    """

    lo, hi = 0.0, maximum
    best = 0.0
    for _ in range(_DRIVE_FALLBACK_STEPS):
        mid = 0.5 * (lo + hi)
        op = _exact_fallback(parameters, speed, mid, voltage)
        if op.feasible and op.bus_power <= target + 1.0e-9:
            best, lo = mid, mid
        else:
            hi = mid
    return best


def _inverse_fallback(parameters, speed, voltage, target, direction):
    """Exact analytical fallback for genuinely out-of-domain targets."""
    maximum = _exact_limit(parameters, speed, voltage, direction == "regen")
    if maximum <= 0.0:
        op = _exact_fallback(parameters, speed, 0.0, voltage)
        return TorqueSolveResult(0.0, op, op.bus_power - target, False, op.active_limit, True)
    if direction == "drive":
        torque = _drive_fallback_torque(parameters, speed, voltage, target, maximum)
        op = _exact_fallback(parameters, speed, torque, voltage)
        tolerance = _row_power_tolerance(target)
        feasible = op.feasible and op.bus_power <= target + tolerance
        return TorqueSolveResult(torque, op, op.bus_power - target, feasible, op.active_limit, True)
    # Regeneration bus power is genuinely non-monotone, so sample the complete
    # negative-torque range, then refine every segment that straddles the target
    # and keep the most-negative admissible torque.
    lo, hi = -maximum, 0.0
    fractions = _fallback_fractions(parameters)
    points = [lo + (hi - lo) * fraction for fraction in fractions]
    values = [
        _exact_fallback(parameters, speed, torque, voltage) for torque in points
    ]
    admissible = [
        torque
        for torque, op in zip(points, values)
        if op.feasible and op.bus_power >= target - 1.0e-9
    ]
    stop = 1.0e-7 + 1.0e-9 * abs(target)
    for left, right, left_op, right_op in zip(points, points[1:], values, values[1:]):
        if not left_op.feasible or not right_op.feasible:
            continue
        if left_op.bus_power == right_op.bus_power:
            continue
        span = right_op.bus_power - left_op.bus_power
        fraction = min(1.0, max(0.0, (target - left_op.bus_power) / span))
        torque = left + (right - left) * fraction
        op = _exact_fallback(parameters, speed, torque, voltage)
        if op.feasible and op.bus_power >= target - 1.0e-9 and abs(op.bus_power - target) <= stop:
            admissible.append(torque)
            continue
        if not (
            min(left_op.bus_power, right_op.bus_power) - 1.0e-9
            <= target
            <= max(left_op.bus_power, right_op.bus_power) + 1.0e-9
        ):
            continue
        a, b = left, right
        f_left = left_op.bus_power - target
        for _ in range(24):
            mid = 0.5 * (a + b)
            mid_op = _exact_fallback(parameters, speed, mid, voltage)
            if not mid_op.feasible:
                break
            if (mid_op.bus_power - target) * f_left <= 0.0:
                b = mid
            else:
                a, f_left = mid, mid_op.bus_power - target
        admissible.append(0.5 * (a + b))
    torque = min(admissible) if admissible else 0.0
    op = _exact_fallback(parameters, speed, torque, voltage)
    tolerance = _row_power_tolerance(target)
    feasible = op.feasible and op.bus_power >= target - tolerance
    return TorqueSolveResult(torque, op, op.bus_power - target, feasible, op.active_limit, True)


def torque_for_bus_power(parameters, motor_speed_rpm, bus_voltage, target_bus_power, direction):
    """Invert LUT bus power, scanning every regen segment inclusively.

    The drive row is monotone, so the inverse is solved directly on the same
    piecewise-linear row the forward evaluator reconstructs; the acceptance
    check is the row's own range plus the map's discretization tolerance rather
    than sub-1e-7 W agreement with a differently ordered interpolation.  The
    regen row is genuinely non-monotone, so every valid negative-load segment is
    scanned and the most-negative feasible torque is selected.
    """

    if direction not in ("drive", "regen"):
        raise ValueError("direction must be 'drive' or 'regen'")
    mapping = parameters.operating_map
    in_map = (
        mapping is not None
        and mapping.voltage[0] <= bus_voltage <= mapping.voltage[-1]
        and mapping.speed[0] <= motor_speed_rpm <= mapping.speed[-1]
    )
    if not in_map:
        _LOOKUP_COUNTERS["inverse"] += 1
        return _inverse_fallback(parameters, motor_speed_rpm, bus_voltage, target_bus_power, direction)
    drive, regen, vi, si, vf, sf = _interpolated_envelopes(
        mapping, bus_voltage, motor_speed_rpm
    )
    if (direction == "drive" and drive <= 0.0) or (
        direction == "regen" and regen <= 0.0
    ):
        _LOOKUP_COUNTERS["inverse"] += 1
        return _inverse_fallback(
            parameters, motor_speed_rpm, bus_voltage, target_bus_power, direction
        )
    li0 = int(np.searchsorted(mapping.load, 0.0))
    row = _interpolated_row(mapping.bus_power, vi, si, vf, sf)
    if direction == "drive":
        loads, vals = mapping.load[li0:], row[li0:]
        tolerance = _row_power_tolerance(target_bus_power)
        if target_bus_power > vals[-1] + tolerance:
            # The requested storage power is above the motor's achievable row
            # maximum, so the motor's own envelope is the limit.  This is a
            # bounded row answer rather than an out-of-domain request: the exact
            # analytical fallback costs dozens of oracle evaluations and can even
            # report an infeasible operating point at the map's voltage ceiling.
            # The returned operating point is still observable and, when the row
            # maximum already charges at least the target, feasible.
            _LOOKUP_COUNTERS["inverse"] += 1
            load = float(loads[-1])
            torque = float(drive * load)
            op = evaluate_motor(parameters, motor_speed_rpm, torque, bus_voltage)
            if not op.feasible or op.bus_power > target_bus_power + tolerance:
                return _inverse_fallback(
                    parameters, motor_speed_rpm, bus_voltage, target_bus_power, direction
                )
            return TorqueSolveResult(
                torque, op, op.bus_power - target_bus_power, True, op.active_limit, False
            )
        if target_bus_power < vals[0] - tolerance:
            # The request is weaker than the mapped zero-load power, so the
            # exact oracle must answer it.
            _LOOKUP_COUNTERS["inverse"] += 1
            return _inverse_fallback(parameters, motor_speed_rpm, bus_voltage, target_bus_power, direction)
        j = min(len(vals) - 2, max(0, int(np.searchsorted(vals, target_bus_power, side="right") - 1)))
        fraction = _segment_load(vals, target_bus_power, j)
        load = float(loads[j] + fraction * (loads[j + 1] - loads[j]))
        found = True
    else:
        mask = _interpolated_regen_mask(mapping, vi, si, vf, sf)
        # Vectorized node scan over every mapped negative-load node.  A node is
        # admissible only when its row bus power already satisfies the charge
        # constraint.  The zero-load anchor is included because it bounds the
        # shallow end of the mapped range: a request weaker than the zero-load
        # bus power needs no braking at all.
        node_hit = mask[:li0 + 1] & (row[:li0 + 1] >= target_bus_power - 1.0e-9)
        candidates = [float(value) for value in mapping.load[:li0 + 1][node_hit]]
        # Zero regen is always physically available: coasting draws no charge
        # power, so a target weaker than the row's zero-load point resolves to
        # no braking even when that mapped node reports a small loss.
        if row[li0] >= target_bus_power - 1.0e-9 or target_bus_power <= 0.0:
            candidates.append(float(mapping.load[li0]))
        # Vectorized segment scan for interior crossings.
        if li0 >= 1:
            y_low = row[:li0]
            y_high = row[1:li0 + 1]
            valid_low = mask[:li0]
            valid_high = mask[1:li0 + 1]
            crossing = (
                valid_low
                & valid_high
                & ((y_low - target_bus_power) * (y_high - target_bus_power) <= 0.0)
                & (y_high != y_low)
            )
            if np.any(crossing):
                x_low = mapping.load[:li0][crossing]
                y_cross_low = y_low[crossing]
                y_cross_high = y_high[crossing]
                fractions = (target_bus_power - y_cross_low) / (y_cross_high - y_cross_low)
                crossing_loads = x_low + fractions * (
                    mapping.load[1:li0 + 1][crossing] - x_low
                )
                candidates.extend(float(value) for value in crossing_loads)
        if not candidates:
            _LOOKUP_COUNTERS["inverse"] += 1
            return _inverse_fallback(parameters, motor_speed_rpm, bus_voltage, target_bus_power, direction)
        load, found = min(candidates), True
    torque = float(drive * load if load >= 0.0 else regen * load)
    # The row solved the load.  The reported operating point comes from the
    # trilinear forward evaluator at the selected torque so consumers see the
    # same power they would compute themselves.  Acceptance is the row's own
    # signed bus-power inequality within the map tolerance; demanding sub-1e-7 W
    # agreement between the row's interpolation order and the forward order is
    # what previously routed almost every in-map query into the expensive exact
    # analytical fallback.
    if torque == 0.0:
        # Zero regenerative torque is always physically available and draws no
        # charge power, so the request is satisfied without braking.  The row's
        # own zero-load power reports the mapped loss at that node.
        _LOOKUP_COUNTERS["inverse"] += 1
        op = ShaftPowerResult(
            float(row[li0]), float(mapping.loss_power[vi, si, li0]), 0.0, 0.0,
            1.0,
            "power" if abs(motor_speed_rpm * 2.0 * math.pi / 60.0) >= parameters.unit_power_limit else "torque",
            True,
        )
        return TorqueSolveResult(0.0, op, float(row[li0]) - target_bus_power, True, op.active_limit, False)
    fallback_before = _LOOKUP_COUNTERS["fallback"]
    op = evaluate_motor(parameters, motor_speed_rpm, torque, bus_voltage)
    stayed_in_domain = _LOOKUP_COUNTERS["fallback"] == fallback_before
    tolerance = _row_power_tolerance(target_bus_power)
    feasible = stayed_in_domain and op.feasible and (
        (op.bus_power <= target_bus_power + tolerance)
        if direction == "drive"
        else (op.bus_power >= target_bus_power - tolerance)
    )
    _LOOKUP_COUNTERS["inverse"] += 1
    if not feasible:
        return _inverse_fallback(parameters, motor_speed_rpm, bus_voltage, target_bus_power, direction)
    return TorqueSolveResult(torque, op, op.bus_power - target_bus_power, True, op.active_limit, False)


# Existing assembly/solver callers retain the analytical oracle's strict
# fixed-point behavior; new production callers use evaluate_motor explicitly.
evaluate_shaft_power = evaluate_shaft_power_exact


def scalar_efficiency_bus_power(shaft_power, efficiency):
    """Convert shaft power to bus power using one scalar efficiency."""

    if not 0.0 < efficiency <= 1.0:
        raise ValueError("efficiency must be above zero and no greater than one")
    if shaft_power >= 0.0:
        return shaft_power / efficiency
    return shaft_power * efficiency


def _reference_row():
    rows = _read("amk_dd5_100c_identification.csv")
    candidates = [row for row in rows if abs(float(row[0]) - 16300.0) < 1.0e-9]
    return max(candidates, key=lambda row: math.hypot(float(row[2]), float(row[3])))


_UNITS = {
    "unit_count": "1",
    "maximum_motor_speed": "rpm",
    "unit_power_limit": "W",
    "inverter_efficiency": "1",
    "device_drop": "V",
}


def _compile_motor_source(model: Model, operating_map=None) -> PowerPlantParameters:
    values = closed_parameters(model, _UNITS)
    require_no_references(model)
    if not values["unit_count"].is_integer() or values["unit_count"] < 1:
        raise ValueError("unit_count must be a positive integer")
    for name in ("maximum_motor_speed", "unit_power_limit"):
        if values[name] <= 0.0:
            raise ValueError(f"{name} must be positive")
    if not 0.0 < values["inverter_efficiency"] <= 1.0:
        raise ValueError("inverter_efficiency must be from zero to one")
    parameters = _parameter_file()
    if parameters["resistance"] <= 0.0:
        raise ValueError("motor resistance must be positive")
    torque_map = _read("amk_dd5_100c_torque_map.csv")
    loss_map = _read("amk_dd5_100c_loss_map.csv")
    reference = _reference_row()
    return PowerPlantParameters(
        int(values["unit_count"]), values["maximum_motor_speed"], values["unit_power_limit"],
        values["inverter_efficiency"], values["device_drop"], int(parameters["pole_pairs"]),
        parameters["reference_bus_voltage"], parameters["resistance"], parameters["flux_linkage"],
        parameters["direct_inductance"], parameters["quadrature_inductance"],
        tuple(float(row[0]) for row in torque_map), tuple(float(row[1]) for row in torque_map),
        tuple(float(row[0]) for row in loss_map), tuple(float(row[1]) for row in loss_map),
        tuple(float(row[2]) for row in loss_map), float(reference[0]), float(reference[2]),
        float(reference[3]), float(reference[4]), float(reference[5]),
        math.hypot(float(reference[4]), float(reference[5])), operating_map,
    )


def compile_analytical_motor_source(model: Model) -> PowerPlantParameters:
    """Private generator-only construction path without a persisted LUT."""
    return _compile_motor_source(model)


def _validate_operating_map(data: dict[str, np.ndarray]) -> MotorOperatingMap:
    names = ("voltage", "speed", "load", "drive_torque", "regen_torque", "bus_power", "loss_power")
    if any(name not in data for name in (*names, "regen_feasible")):
        raise ValueError("operating map is missing required arrays")
    arrays = {name: np.array(data[name], dtype=np.float64, order="C", copy=True) for name in names}
    feasible = np.array(data["regen_feasible"], dtype=bool, order="C", copy=True)
    arrays["regen_feasible"] = feasible
    v, s, l = arrays["voltage"], arrays["speed"], arrays["load"]
    if v.ndim != 1 or s.ndim != 1 or l.ndim != 1 or v.size < 2 or s.size < 2 or l.size < 3:
        raise ValueError("operating map axes have invalid shape")
    if not all(np.all(np.isfinite(a)) for a in arrays.values()):
        raise ValueError("operating map contains nonfinite values")
    if not np.all(np.diff(v) > 0) or not np.all(np.diff(s) > 0) or not np.all(np.diff(l) > 0):
        raise ValueError("operating map axes must be strictly increasing")
    if l[0] != -1.0 or l[-1] != 1.0:
        raise ValueError("operating map load axis must span -1 to 1")
    shape2, shape3 = (v.size, s.size), (v.size, s.size, l.size)
    if arrays["drive_torque"].shape != shape2 or arrays["regen_torque"].shape != shape2:
        raise ValueError("operating map torque envelope shape mismatch")
    if arrays["bus_power"].shape != shape3 or arrays["loss_power"].shape != shape3 or feasible.shape != shape3:
        raise ValueError("operating map power shape mismatch")
    if np.any(arrays["drive_torque"] < 0) or np.any(arrays["regen_torque"] < 0):
        raise ValueError("operating map torque envelopes must be nonnegative")
    if np.any(arrays["loss_power"] < 0):
        raise ValueError("operating map loss must be nonnegative")
    zero_indices = np.flatnonzero(np.isclose(l, 0.0, atol=1.0e-9, rtol=0.0))
    if zero_indices.size != 1:
        raise ValueError("operating map load axis requires one zero anchor")
    zero = int(zero_indices[0])
    power_rows = arrays["bus_power"].reshape(-1, l.size)
    feasible_rows = feasible.reshape(-1, l.size)
    for row, valid in zip(power_rows, feasible_rows):
        if row[zero] < -1.0e-9:
            raise ValueError("operating map zero-load bus power has invalid sign")
        if np.any(row[zero + 1:] < -1.0e-9):
            raise ValueError("operating map drive bus power has invalid sign")
        if np.any(valid[zero:]):
            raise ValueError("operating map regen feasibility is limited to negative loads")
        negative_row = row[:zero]
        regen = valid[:zero]
        productive = negative_row < -1.0e-9
        if not np.array_equal(regen, productive):
            raise ValueError("operating map regen productive mask is inconsistent")
        if np.any(np.diff(row[zero:]) < -1.0e-9):
            raise ValueError("operating map drive bus power is not monotone")
    for array in arrays.values():
        array.setflags(write=False)
    return MotorOperatingMap(**arrays)


def _load_operating_map(model: Model) -> MotorOperatingMap:
    references = [item for item in model.data if item.role == "operating-map"]
    if len(references) != 1 or references[0].format != "npz":
        raise ValueError("electric motor requires one operating-map NPZ declaration")
    reference = references[0]
    if not reference.sha256:
        raise ValueError("operating-map declaration requires sha256")
    digest = hashlib.sha256(reference.path.read_bytes()).hexdigest()
    if digest.lower() != reference.sha256.lower():
        raise ValueError("operating-map digest mismatch")
    with np.load(reference.path, allow_pickle=False) as archive:
        return _validate_operating_map({name: archive[name] for name in archive.files})


def compile_electric_motor(model: Model) -> PowerPlantParameters:
    """Compile the XML motor and require its declared, validated LUT."""
    return _compile_motor_source(model, _load_operating_map(model))


register_component(
    "power-plant",
    "electric-motor",
    capability=TORQUE_SOURCE,
    compile_function=compile_electric_motor,
)
