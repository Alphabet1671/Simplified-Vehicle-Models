"""Deterministically generate the canonical motor operating map."""
from __future__ import annotations

import argparse
import hashlib
import os
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT / "src"))

from simplified_vehicle_models.powertrain.power_plant import (  # noqa: E402
    compile_analytical_motor_source,
    evaluate_shaft_power_exact,
)
from simplified_vehicle_models.xml.loader import load_model  # noqa: E402

GENERATOR_VERSION = "motor-lut-v2"

# Default grid resolution.  These values are also the documented canonical
# command in ``models/data/provenance.md``; changing them changes the artifact
# digest and therefore the XML declaration.
DEFAULT_VOLTAGE_POINTS = 9
DEFAULT_SPEED_POINTS = 129
DEFAULT_LOAD_POINTS = 65

# The persisted drive row must be monotone to within the load-time validator's
# tolerance (``_validate_operating_map`` rejects a decrease below -1.0e-9 W).
_MONOTONE_TOLERANCE = 1.0e-9

# Relative width at which the envelope-reduction bisection stops.  A relative
# width of 1.0e-12 resolves the boundary to ~40 bisections, far below the
# 1.0e-3 Nm scale at which map error moves lap time by a millisecond.
_REDUCTION_RELATIVE_TOLERANCE = 1.0e-12
_REDUCTION_MAX_STEPS = 64


class EnvelopeReduction:
    """Accumulated statistics for the minimal drive-envelope reductions."""

    def __init__(self) -> None:
        self.reduced_nodes = 0
        self.max_reduction = 0.0
        self.max_steps = 0
        self.bisection_evaluations = 0

    def record(self, absolute: float, reduced: float, steps: int, row: int) -> None:
        self.reduced_nodes += 1
        self.max_reduction = max(self.max_reduction, absolute - reduced)
        self.max_steps = max(self.max_steps, steps)
        self.bisection_evaluations += steps * row


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _maximum_feasible(parameters, speed, voltage, sign, envelope):
    if envelope <= 0.0:
        return 0.0

    def feasible(torque_magnitude):
        result = evaluate_shaft_power_exact(
            parameters, speed, sign * torque_magnitude, voltage
        )
        return result.feasible

    if not feasible(0.0):
        return 0.0
    if feasible(envelope):
        return envelope
    lo, hi = 0.0, envelope
    for _ in range(80):
        mid = (lo + hi) * 0.5
        if feasible(mid):
            lo = mid
        else:
            hi = mid
    return lo


def _source_paths(xml_path: Path) -> tuple[Path, ...]:
    data = _ROOT / "models" / "data"
    return (xml_path, data / "amk_dd5_100c_machine_parameters.csv",
            data / "amk_dd5_100c_torque_map.csv", data / "amk_dd5_100c_loss_map.csv",
            data / "amk_dd5_100c_identification.csv")


def _regen_envelope(parameters, speed, voltage, envelope):
    """Return the complete feasible negative-torque envelope.

    Regenerative bus power is genuinely non-monotone at some operating points,
    so neither a productive-power crossing nor bus-power ordering may reduce
    this analytical torque boundary.
    """
    if envelope <= 0.0:
        return 0.0
    endpoint = evaluate_shaft_power_exact(parameters, speed, -float(envelope), voltage)
    if not endpoint.feasible:
        raise ValueError(
            f"analytical regen envelope is infeasible at voltage={voltage}, speed={speed}"
        )
    return float(envelope)


def _drive_row_is_monotone(parameters, speed, voltage, magnitude, fractions):
    """Return whether the sampled drive bus-power row is non-decreasing.

    ``fractions`` are the positive normalized loads of the persisted drive row,
    so the predicate evaluates exactly the row ``generate`` stores and exactly
    the row ``_validate_operating_map`` re-checks.  One predicate therefore
    keeps generation and load-time validation in agreement.  The scan
    short-circuits on the first decrease, which keeps a rejected magnitude
    cheap.  Only the positive-load half is checked: the negative-load half is
    the regen row, which is deliberately not monotone.
    """
    previous = None
    for fraction in fractions:
        value = evaluate_shaft_power_exact(
            parameters, speed, fraction * magnitude, voltage
        ).bus_power
        if previous is not None and value - previous < -_MONOTONE_TOLERANCE:
            return False
        previous = value
    return True


def _monotone_drive_envelope(parameters, speed, voltage, envelope, drive_fractions):
    """Return the largest feasible drive envelope with a monotone sampled row.

    The exact analytical feasibility boundary found by ``_maximum_feasible`` is
    kept whenever its sampled row is already monotone; only a genuine
    monotonicity violation reduces it.  A reduction bisects the torque magnitude
    to the largest monotone value, so it is minimal rather than one of a coarse
    set of discrete candidate steps: a 257-candidate scan snapped the boundary
    down by up to ``envelope / 256`` (about 0.05 Nm, sometimes twice that) even
    when the exact boundary was itself feasible and monotone.
    """
    magnitude = float(envelope)
    if magnitude <= 0.0:
        return 0.0, False, 0
    fractions = np.asarray(drive_fractions, dtype=np.float64)
    if _drive_row_is_monotone(parameters, speed, voltage, magnitude, fractions):
        return magnitude, False, 0
    lo, hi = 0.0, magnitude
    steps = 0
    tolerance = magnitude * _REDUCTION_RELATIVE_TOLERANCE
    while hi - lo > tolerance and steps < _REDUCTION_MAX_STEPS:
        midpoint = 0.5 * (lo + hi)
        steps += 1
        if _drive_row_is_monotone(parameters, speed, voltage, midpoint, fractions):
            lo = midpoint
        else:
            hi = midpoint
    return float(lo), True, steps


def validate_grid(voltage_points: int, speed_points: int, load_points: int) -> None:
    """Reject grid resolutions the map format cannot represent faithfully."""
    if voltage_points < 2:
        raise ValueError("voltage_points must be at least 2")
    if speed_points < 2:
        raise ValueError("speed_points must be at least 2")
    if load_points < 3:
        raise ValueError("load_points must be at least 3")
    if load_points % 2 == 0:
        # An odd load axis places exactly one sample on the zero-torque anchor,
        # which ``_validate_operating_map`` requires.
        raise ValueError("load_points must be odd so the load axis has one zero anchor")


def generate(path, voltage_points=DEFAULT_VOLTAGE_POINTS,
             speed_points=DEFAULT_SPEED_POINTS, load_points=DEFAULT_LOAD_POINTS,
             xml_path=None, voltage_min=448.848, voltage_max=608.688) -> str:
    """Generate the canonical map and return its SHA-256 digest."""
    validate_grid(voltage_points, speed_points, load_points)
    path = Path(path)
    xml_path = Path(xml_path) if xml_path is not None else _ROOT / "models" / "power-plants" / "amk_dd5_100c.xml"
    parameters = compile_analytical_motor_source(load_model(xml_path))
    source_digests = {str(source): _sha256(source) for source in _source_paths(xml_path)}
    voltage = np.linspace(voltage_min, voltage_max, voltage_points, dtype=np.float64)
    speed = np.linspace(0.0, 18600.0, speed_points, dtype=np.float64)
    load = np.linspace(-1.0, 1.0, load_points, dtype=np.float64)
    zero = load_points // 2
    # The persisted drive row is exactly ``load[zero:]``; the monotonicity
    # predicate therefore samples the same normalized loads the validator sees.
    drive_fractions = load[zero:]
    drive_torque = np.empty((voltage_points, speed_points), dtype=np.float64)
    regen_torque = np.empty_like(drive_torque)
    bus_power = np.empty((voltage_points, speed_points, load_points), dtype=np.float64)
    loss_power = np.empty_like(bus_power)
    regen_feasible = np.zeros_like(bus_power, dtype=np.bool_)
    max_drive_violation = 0.0
    reductions = EnvelopeReduction()
    for iv, bus_voltage in enumerate(voltage):
        for ix, motor_speed in enumerate(speed):
            envelope = float(np.interp(
                min(motor_speed, parameters.torque_speed[-1]),
                parameters.torque_speed, parameters.torque_values,
            ))
            exact_drive = _maximum_feasible(
                parameters, motor_speed, bus_voltage, 1.0, envelope,
            )
            drive_limit, reduced, steps = _monotone_drive_envelope(
                parameters, motor_speed, bus_voltage, exact_drive, drive_fractions,
            )
            if reduced:
                reductions.record(exact_drive, drive_limit, steps, load_points)
            drive_torque[iv, ix] = drive_limit
            regen_torque[iv, ix] = _regen_envelope(
                parameters, motor_speed, bus_voltage,
                _maximum_feasible(parameters, motor_speed, bus_voltage, -1.0, envelope),
            )
            for il, normalized_load in enumerate(load):
                torque_limit = drive_torque[iv, ix] if normalized_load >= 0 else regen_torque[iv, ix]
                result = evaluate_shaft_power_exact(parameters, motor_speed, normalized_load * torque_limit, bus_voltage)
                if not result.feasible:
                    raise ValueError(
                        f"infeasible point inside analytical torque envelope at "
                        f"voltage={bus_voltage}, speed={motor_speed}, load={normalized_load}"
                    )
                # Retain every finite analytical value.  The mask identifies
                # productive negative-load samples only; it does not truncate
                # or reorder a physically non-monotone regen curve.
                bus_power[iv, ix, il] = result.bus_power
                loss_power[iv, ix, il] = result.loss_power
                regen_feasible[iv, ix, il] = (
                    normalized_load < 0.0 and result.bus_power < -1.0e-9
                )
            row = bus_power[iv, ix]
            # Drive power remains monotone.  Regen is intentionally not subject
            # to this check: Task 3 scans every piecewise-linear regen segment.
            values = row[zero:]
            if values.size > 1:
                delta = float(np.max(np.maximum(0.0, -np.diff(values))))
                max_drive_violation = max(max_drive_violation, delta)
                if delta > 1.0e-9:
                    raise ValueError(
                        f"non-monotone analytical drive bus power at "
                        f"voltage={bus_voltage}, speed={motor_speed}: {delta} W"
                    )
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    np.savez_compressed(temporary, voltage=voltage, speed=speed, load=load,
                        drive_torque=drive_torque, regen_torque=regen_torque,
                        bus_power=bus_power, loss_power=loss_power,
                        regen_feasible=regen_feasible)
    produced = temporary if temporary.exists() else Path(str(temporary) + ".npz")
    os.replace(produced, path)
    digest = _sha256(path)
    print(f"SHA-256: {digest}")
    print(f"grid: voltage_points={voltage_points} speed_points={speed_points} load_points={load_points}")
    print(f"axes: voltage={voltage.shape}, speed={speed.shape}, load={load.shape}")
    print(f"array shapes: drive_torque={drive_torque.shape}, regen_torque={regen_torque.shape}, bus_power={bus_power.shape}, loss_power={loss_power.shape}")
    print(f"artifact bytes: {path.stat().st_size}")
    print(f"maximum drive monotonic violation: {max_drive_violation:.17g}")
    print(
        f"drive envelope reductions: nodes={reductions.reduced_nodes} "
        f"max_reduction={reductions.max_reduction:.17g} Nm "
        f"max_bisection_steps={reductions.max_steps} "
        f"extra_evaluations={reductions.bisection_evaluations}"
    )
    print(f"generator version: {GENERATOR_VERSION}")
    for source, source_digest in source_digests.items():
        print(f"source SHA-256: {source} {source_digest}")
    print(f"generation command: {' '.join(sys.argv)}")
    return digest


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("xml_path")
    parser.add_argument("output")
    parser.add_argument(
        "--voltage-points", type=int, default=DEFAULT_VOLTAGE_POINTS,
        help=f"bus-voltage grid samples (default: {DEFAULT_VOLTAGE_POINTS})",
    )
    parser.add_argument(
        "--speed-points", type=int, default=DEFAULT_SPEED_POINTS,
        help=f"motor-speed grid samples (default: {DEFAULT_SPEED_POINTS})",
    )
    parser.add_argument(
        "--load-points", type=int, default=DEFAULT_LOAD_POINTS,
        help=f"signed normalized-load samples, must be odd (default: {DEFAULT_LOAD_POINTS})",
    )
    parser.add_argument(
        "--voltage-min", type=float, default=448.848,
        help="minimum bus-voltage axis value (default: 448.848)",
    )
    parser.add_argument(
        "--voltage-max", type=float, default=608.688,
        help="maximum bus-voltage axis value (default: 608.688)",
    )
    return parser


if __name__ == "__main__":
    options = build_parser().parse_args()
    generate(
        options.output,
        voltage_points=options.voltage_points,
        speed_points=options.speed_points,
        load_points=options.load_points,
        xml_path=options.xml_path,
        voltage_min=options.voltage_min,
        voltage_max=options.voltage_max,
    )
