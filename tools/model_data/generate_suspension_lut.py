"""Deterministically generate the corner suspension table NPZ and its model XML.

The input is the IEM24 DSS workbook (sheet ``FSAE_Specs_EV``) or a CSV copy of its
rows 17-31. The generator reads these cells by fixed address:

* D17/E17 and G17/H17: front and rear compression and rebound design travel [mm];
* C23/F23: heave motion ratio (the first ``<ratio>:1 heave`` value; the roll ratio is ignored);
* C24/F24: camber gain [deg/m];
* C26/F26: static toe [deg], positive toe-in, text or number;
* C29/F29: anti values in percent, first value brake (anti-dive), second drive (anti-squat);
* C30/F30: roll-centre height, static [mm];
* D31/E31 and G31/H31: roll-centre height and lateral position at 1 g [mm].

Rows 18-19 of the DSS hold wheel and roll rates. They are intentionally NOT used:
the selected wheel rates come from the command line (lbf/in). The workbook stores
some numbers as text; the generator converts them and fails on a non-numeric value.

The jounce-steer curves are user-selected zero baseline placeholders (no measured
data). The schema allows measured nonzero knots later.

The NPZ is byte-reproducible: stored zip entries, fixed timestamps, and neither a
path nor a time enters the bytes.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import shlex
import sys
from pathlib import Path

import numpy as np

try:  # imported as a package module
    from tools.model_data.generate_damper_lut import atomic_write_bytes, npz_bytes
except ImportError:  # run as a script
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from generate_damper_lut import atomic_write_bytes, npz_bytes  # type: ignore

GENERATOR_VERSION = "suspension-lut-v1"
SCHEMA = "suspension-corner-v1"
DEFAULT_SHEET = "FSAE_Specs_EV"

LBF_IN_TO_N_M = 4.4482216152605 / 0.0254
G = 9.80665
# Reference roll gradient (rigid tire) used to map the 1 g DSS point to a roll angle.
ROLL_GRADIENT_DEG_PER_G = 0.289
# Roll-centre table extends to this roll magnitude; height and lateral are held beyond 1 g.
ROLL_TABLE_LIMIT_DEG = 5.0
# PROPOSED bump and droop stops.
BUMP_STOP_RATE_MULTIPLIER = 10.0
BUMP_STOP_ONSET_BEFORE_LIMIT_M = 0.002   # 29.8 mm for a 31.8 mm travel
BUMP_STOP_BLEND_M = 0.002                # C1 blend over 27.8-29.8 mm
SPRING_KNOT_SPACING_M = 2.0e-4
STOP_KNOT_SPACING_M = 2.0e-5   # fine knots over the blend and stop, keeps the Hermite rate ripple small
# Static sprung corner loads: 287 kg total, 45 % front static, 13 kg unsprung per corner (PROPOSED).
TOTAL_MASS_KG = 287.0
FRONT_MASS_FRACTION = 0.45
UNSPRUNG_MASS_PER_CORNER_KG = 13.0

ZERO_CURVE_SOURCE = "baseline, no measured jounce-steer data"

CELLS = {
    "front": {
        "compression": "D17", "rebound": "E17", "motion_ratio": "C23", "camber_gain": "C24",
        "static_toe": "C26", "anti": "C29", "rc_static": "C30", "rc_1g_height": "D31",
        "rc_1g_lateral": "E31",
    },
    "rear": {
        "compression": "G17", "rebound": "H17", "motion_ratio": "F23", "camber_gain": "F24",
        "static_toe": "F26", "anti": "F29", "rc_static": "F30", "rc_1g_height": "G31",
        "rc_1g_lateral": "H31",
    },
}


class SuspensionTableError(ValueError):
    """Report an invalid input or a table defect."""


# ---------------------------------------------------------------- readers
def _column_letter(index: int) -> str:
    letters = ""
    column = index + 1
    while column > 0:
        column, remainder = divmod(column - 1, 26)
        letters = chr(ord("A") + remainder) + letters
    return letters


def read_csv_cells(path):
    """Read a CSV copy of DSS rows. Column 0 is the sheet row; columns 1.. are A, B, ...

    Lines that start with ``#`` are provenance comments.
    """
    cells = {}
    with Path(path).open(newline="") as stream:
        for row in csv.reader(stream):
            if not row or row[0].lstrip().startswith("#"):
                continue
            try:
                sheet_row = int(row[0])
            except ValueError as error:
                raise SuspensionTableError(f"{path}: first column must be the sheet row, got {row[0]!r}") from error
            for index, text in enumerate(row[1:]):
                if text != "":
                    cells[f"{_column_letter(index)}{sheet_row}"] = text
    return cells


def read_xlsx_cells(path, sheet=DEFAULT_SHEET, rows=range(17, 32), columns=range(10)):
    """Read the cells of DSS rows 17-31, columns A-J, as stored."""
    try:
        import openpyxl  # lazy: only the xlsx path needs it
    except ImportError as error:  # pragma: no cover - environment dependent
        raise SuspensionTableError("reading xlsx requires openpyxl") from error
    workbook = openpyxl.load_workbook(path, data_only=True, read_only=True)
    try:
        if sheet not in workbook.sheetnames:
            raise SuspensionTableError(f"{path}: no sheet {sheet!r}; found {workbook.sheetnames}")
        worksheet = workbook[sheet]
        cells = {}
        for row in rows:
            for column in columns:
                value = worksheet.cell(row=row, column=column + 1).value
                if value is not None:
                    cells[f"{_column_letter(column)}{row}"] = value
    finally:
        workbook.close()
    return cells


def sheet_rows_csv_text(cells, rows=range(17, 32), columns=range(10)):
    """Return CSV text of the given cells in the fixture layout (for fixture creation)."""
    import io

    buffer = io.StringIO()
    writer = csv.writer(buffer, lineterminator="\n")
    for row in rows:
        writer.writerow([row] + [str(cells.get(f"{_column_letter(c)}{row}", "")) for c in columns])
    return buffer.getvalue()


# ---------------------------------------------------------------- parsing
def _number(cells, address):
    """Return a finite float from a numeric or numeric-text cell; fail otherwise."""
    value = cells.get(address)
    if value is None or isinstance(value, bool):
        raise SuspensionTableError(f"DSS cell {address} is empty or not numeric: {value!r}")
    if isinstance(value, str):
        try:
            number = float(value.strip())
        except ValueError as error:
            raise SuspensionTableError(f"DSS cell {address} is not numeric: {value!r}") from error
    else:
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise SuspensionTableError(f"DSS cell {address} is not numeric: {value!r}") from error
    if not math.isfinite(number):
        raise SuspensionTableError(f"DSS cell {address} is not finite: {value!r}")
    return number


_HEAVE_RATIO = re.compile(r"^\s*([0-9]*\.?[0-9]+)\s*:\s*1\s+heave\b", re.IGNORECASE)
_ANTI = re.compile(r"^\s*([0-9]*\.?[0-9]+)\s*/\s*([0-9]*\.?[0-9]+)\s*$")


def parse_heave_motion_ratio(cells, address):
    """Parse the first ``<ratio>:1 heave`` value of a cell such as ``1.05:1 heave / 1.2:1 roll``."""
    value = cells.get(address)
    match = _HEAVE_RATIO.match(value) if isinstance(value, str) else None
    if match is None:
        raise SuspensionTableError(f"DSS cell {address} has no '<ratio>:1 heave' value: {value!r}")
    ratio = float(match.group(1))
    if ratio <= 0.0:
        raise SuspensionTableError(f"DSS cell {address} heave motion ratio must be positive")
    return ratio


def parse_anti(cells, address):
    """Parse ``brake/drive`` percent text, return fractions ``(brake, drive)``."""
    value = cells.get(address)
    match = _ANTI.match(value) if isinstance(value, str) else None
    if match is None:
        raise SuspensionTableError(f"DSS cell {address} is not 'brake/drive' percent text: {value!r}")
    brake, drive = float(match.group(1)) / 100.0, float(match.group(2)) / 100.0
    if not (0.0 <= brake <= 1.0 and 0.0 <= drive <= 1.0):
        raise SuspensionTableError(f"DSS cell {address} anti percentages must be 0-100")
    return brake, drive


def read_axle_cells(cells, axle):
    """Return the DSS values of one axle in SI units (jounce limits, RC in m, toe in rad)."""
    a = CELLS[axle]
    compression = _number(cells, a["compression"]) * 1.0e-3
    rebound = _number(cells, a["rebound"]) * 1.0e-3
    if compression <= 0.0 or rebound <= 0.0:
        raise SuspensionTableError(f"{axle} design travel must be positive")
    brake, drive = parse_anti(cells, a["anti"])
    return {
        "jounce_limits": (-rebound, compression),
        "motion_ratio": parse_heave_motion_ratio(cells, a["motion_ratio"]),
        "camber_gain": _number(cells, a["camber_gain"]),
        "static_toe_rad": math.radians(_number(cells, a["static_toe"])),
        "anti_brake": brake,
        "anti_drive": drive,
        "rc_static": _number(cells, a["rc_static"]) * 1.0e-3,
        "rc_1g_height": _number(cells, a["rc_1g_height"]) * 1.0e-3,
        "rc_1g_lateral": _number(cells, a["rc_1g_lateral"]) * 1.0e-3,
    }


# ---------------------------------------------------------------- curves
def default_static_loads():
    """Return the sprung static corner loads (front, rear) in N."""
    front = (TOTAL_MASS_KG * FRONT_MASS_FRACTION - 2.0 * UNSPRUNG_MASS_PER_CORNER_KG) / 2.0 * G
    rear = (TOTAL_MASS_KG * (1.0 - FRONT_MASS_FRACTION) - 2.0 * UNSPRUNG_MASS_PER_CORNER_KG) / 2.0 * G
    return front, rear


def _stop_integral(z, limit):
    """Return the integral of the rate ratio (rate / base rate) from 0 to z >= 0."""
    onset = limit - BUMP_STOP_ONSET_BEFORE_LIMIT_M
    start = onset - BUMP_STOP_BLEND_M
    h = BUMP_STOP_BLEND_M
    m = BUMP_STOP_RATE_MULTIPLIER - 1.0
    z = np.asarray(z, dtype=float)
    t = np.clip((z - start) / h, 0.0, 1.0)
    blend = h * (t ** 3 - 0.5 * t ** 4)           # integral of smoothstep 3t^2-2t^3
    past = np.maximum(z - onset, 0.0)
    return z + m * (blend + past)


def spring_curve(wheel_rate, static_load, limits):
    """Return ``(jounce, force)`` knots of the wheel spring with bump and droop stops.

    The force is ``static_load`` at jounce 0 and has slope ``wheel_rate`` there. The rate
    rises to ``BUMP_STOP_RATE_MULTIPLIER`` times with a C1 blend before each stop.
    """
    low, high = limits
    pieces = []
    for sign, limit in ((-1.0, -low), (1.0, high)):
        onset = limit - BUMP_STOP_ONSET_BEFORE_LIMIT_M
        fine_start = onset - BUMP_STOP_BLEND_M - 5.0e-4
        grid = list(np.arange(0.0, fine_start, SPRING_KNOT_SPACING_M))
        grid += list(np.arange(fine_start, limit, STOP_KNOT_SPACING_M))
        grid += [onset - BUMP_STOP_BLEND_M, onset, limit]
        kept = []
        for value in sorted(grid):
            if value < 0.0 or (kept and value <= kept[-1] + 1.0e-9):
                continue
            kept.append(float(value))
        kept[-1] = float(limit)
        pieces.append((sign, np.array(kept)))
    negative = -pieces[0][1][:0:-1]
    positive = pieces[1][1]
    jounce = np.concatenate([negative, positive])
    z = np.abs(jounce)
    force = static_load + wheel_rate * np.sign(jounce) * _stop_integral_pair(z, jounce, limits)
    jounce[0], jounce[-1] = low, high
    return jounce, force


def _stop_integral_pair(z, jounce, limits):
    low, high = limits
    out = np.empty_like(z)
    neg = jounce < 0.0
    out[neg] = _stop_integral(z[neg], -low)
    out[~neg] = _stop_integral(z[~neg], high)
    return out


def roll_centre_curves(rc_static, rc_1g_height, rc_1g_lateral):
    """Return ``(roll_rad, height_m, lateral_m)`` knots, roll magnitude from zero.

    The 1 g point sits at roll = 0.289 deg/g * 1 g. Height and lateral are held constant
    above it (the DSS gives no data beyond 1 g).
    """
    roll_1g = math.radians(ROLL_GRADIENT_DEG_PER_G)
    roll_max = math.radians(ROLL_TABLE_LIMIT_DEG)
    roll = np.array([0.0, roll_1g, roll_max])
    height = np.array([rc_static, rc_1g_height, rc_1g_height])
    lateral = np.array([0.0, rc_1g_lateral, rc_1g_lateral])
    return roll, height, lateral


def validate_jounce_steer(jounce_m, toe_rad, limits):
    """Validate jounce-steer knots: strictly rising, finite, explicit 0 m/0 rad, knots at both limits."""
    jounce = np.asarray(jounce_m, dtype=float)
    toe = np.asarray(toe_rad, dtype=float)
    if jounce.ndim != 1 or toe.shape != jounce.shape or jounce.size < 3:
        raise SuspensionTableError("jounce steer needs matching 1-D knots, at least three")
    if not (np.all(np.isfinite(jounce)) and np.all(np.isfinite(toe))):
        raise SuspensionTableError("jounce steer knots must be finite")
    if np.any(np.diff(jounce) <= 0.0):
        raise SuspensionTableError("jounce steer jounce knots must be strictly rising")
    zero = np.flatnonzero(jounce == 0.0)
    if zero.size != 1 or toe[zero[0]] != 0.0:
        raise SuspensionTableError("jounce steer needs an explicit 0 mm knot with 0 deg toe change")
    if jounce[0] != limits[0] or jounce[-1] != limits[1]:
        raise SuspensionTableError(
            f"jounce steer needs knots at both travel limits {limits[0]:g} and {limits[1]:g} m"
        )
    return jounce, toe


# ---------------------------------------------------------------- assembly
def build_corner_arrays(
    cells,
    *,
    wheel_rate_front_n_m,
    wheel_rate_rear_n_m,
    static_load_front_n,
    static_load_rear_n,
    jounce_steer_front=None,
    jounce_steer_rear=None,
    source_sha256="",
):
    """Build the ``suspension-corner-v1`` arrays from DSS cells. Pure: no file access.

    ``jounce_steer_*`` is ``None`` for the zero baseline or ``(jounce_mm, toe_deg)``.
    """
    for name, value in (
        ("front wheel rate", wheel_rate_front_n_m), ("rear wheel rate", wheel_rate_rear_n_m),
        ("front static load", static_load_front_n), ("rear static load", static_load_rear_n),
    ):
        if not (math.isfinite(value) and value > 0.0):
            raise SuspensionTableError(f"{name} must be positive and finite")
    arrays = {
        "schema": np.asarray(SCHEMA),
        "generator_version": np.asarray(GENERATOR_VERSION),
        "source_sha256": np.asarray(str(source_sha256)),
    }
    axle_meta = {}
    inputs = {
        "front": (wheel_rate_front_n_m, static_load_front_n, jounce_steer_front),
        "rear": (wheel_rate_rear_n_m, static_load_rear_n, jounce_steer_rear),
    }
    for axle, (rate, load, steer) in inputs.items():
        dss = read_axle_cells(cells, axle)
        limits = dss["jounce_limits"]
        spring_z, spring_f = spring_curve(rate, load, limits)
        if steer is None:
            steer_z = np.array([limits[0], 0.0, limits[1]])
            steer_toe = np.zeros(3)
            steer_source = ZERO_CURVE_SOURCE
        else:
            steer_z = np.asarray(steer[0], dtype=float) * 1.0e-3
            steer_toe = np.radians(np.asarray(steer[1], dtype=float))
            steer_source = "user-supplied jounce-steer knots"
        steer_z, steer_toe = validate_jounce_steer(steer_z, steer_toe, limits)
        roll, rc_h, rc_l = roll_centre_curves(dss["rc_static"], dss["rc_1g_height"], dss["rc_1g_lateral"])
        values = {
            "spring_jounce": spring_z,
            "spring_force": spring_f,
            "motion_ratio_jounce": np.array([limits[0], 0.0, limits[1]]),
            "motion_ratio": np.full(3, dss["motion_ratio"]),
            "rc_roll": roll,
            "rc_height": rc_h,
            "rc_lateral": rc_l,
            "jounce_steer_jounce": steer_z,
            "jounce_steer_toe": steer_toe,
            "jounce_limits": np.array(limits),
            "anti_brake": np.asarray(dss["anti_brake"]),
            "anti_drive": np.asarray(dss["anti_drive"]),
            "camber_gain": np.asarray(dss["camber_gain"]),
            "static_sprung_load": np.asarray(float(load)),
        }
        for name, value in values.items():
            array = np.asarray(value, dtype=float)  # scalars stay 0-d
            arrays[f"{name}_{axle}"] = np.ascontiguousarray(array) if array.ndim else array
        axle_meta[axle] = {
            "cells": CELLS[axle],
            "jounce_steer_source": steer_source,
            "static_toe_rad": dss["static_toe_rad"],
            "wheel_rate_n_per_m": float(rate),
        }
    metadata = {
        "axles": axle_meta,
        "bump_stop": {
            "status": "PROPOSED",
            "rate_multiplier": BUMP_STOP_RATE_MULTIPLIER,
            "onset_before_limit_m": BUMP_STOP_ONSET_BEFORE_LIMIT_M,
            "blend_m": BUMP_STOP_BLEND_M,
            "note": "10x wheel rate from 29.8 mm, C1 smoothstep blend over 27.8-29.8 mm, both directions",
        },
        "dss_sheet": DEFAULT_SHEET,
        "jounce_steer": ZERO_CURVE_SOURCE if jounce_steer_front is None and jounce_steer_rear is None
        else "see axles.*.jounce_steer_source",
        "roll_centre": {
            "roll_gradient_deg_per_g": ROLL_GRADIENT_DEG_PER_G,
            "table_limit_deg": ROLL_TABLE_LIMIT_DEG,
            "note": "RC height and lateral are held constant above the 1 g point; lateral keeps the DSS sign "
                    "and Task 6 multiplies by the sign of the roll angle",
        },
        "wheel_rate_source": "command line (lbf/in); DSS rows 18-19 intentionally overridden",
    }
    arrays["metadata"] = np.asarray(json.dumps(metadata, sort_keys=True, separators=(",", ":")))
    return arrays


def static_toe_rad(cells, axle):
    return read_axle_cells(cells, axle)["static_toe_rad"]


DEFAULT_DAMPER_FRONT = "../dampers/multimatic_dssv_vc01_front.xml"
DEFAULT_DAMPER_REAR = "../dampers/multimatic_dssv_vc01_rear.xml"

_HEADER = (
    '<?xml version="1.0" encoding="UTF-8"?>\n'
    f"<!-- Generated by tools/model_data/generate_suspension_lut.py ({GENERATOR_VERSION}). Do not edit.\n"
    "     IEM24 corner suspension from the DSS workbook. Wheel rates are overridden by the selected\n"
    "     450/350 lbf/in. Inertance is PROPOSED 0. Jounce steer is a zero curve:\n"
    f"     {ZERO_CURVE_SOURCE}.\n"
    "     Provenance: docs/model-data/provenance.md. -->\n"
)


def kinematics_xml(model_id, npz_relative, digest, static_toe_front, static_toe_rear):
    """Return the ``suspension-kinematics/corner-table`` component XML."""

    return (
        _HEADER
        + f'<model schema-version="1.0" id="{model_id}" type="suspension-kinematics" subtype="corner-table">\n'
        "  <parameters>\n"
        f'    <parameter name="static_toe_front" value="{static_toe_front!r}" unit="rad"/>\n'
        f'    <parameter name="static_toe_rear" value="{static_toe_rear!r}" unit="rad"/>\n'
        "  </parameters>\n"
        f'  <data role="corner-table" path="{npz_relative}" format="npz" sha256="{digest}"/>\n'
        "</model>\n"
    )


def springs_xml(model_id, npz_relative, digest):
    """Return the ``spring-model/corner-table`` component XML (spring curves of both axles)."""

    return (
        _HEADER
        + f'<model schema-version="1.0" id="{model_id}" type="spring-model" subtype="corner-table">\n'
        f'  <data role="corner-table" path="{npz_relative}" format="npz" sha256="{digest}"/>\n'
        "</model>\n"
    )


def arb_xml(model_id, arb_front_n_m, arb_rear_n_m):
    """Return the ``arb-model/axle-rate`` component XML."""

    return (
        _HEADER
        + f'<model schema-version="1.0" id="{model_id}" type="arb-model" subtype="axle-rate">\n'
        "  <parameters>\n"
        f'    <parameter name="arb_wheel_rate_front" value="{arb_front_n_m!r}" unit="N/m"/>\n'
        f'    <parameter name="arb_wheel_rate_rear" value="{arb_rear_n_m!r}" unit="N/m"/>\n'
        "  </parameters>\n"
        "</model>\n"
    )


def suspension_xml(model_id, kinematics_ref, spring_ref, arb_ref, damper_front_ref, damper_rear_ref,
                   inertance_front=0.0, inertance_rear=0.0):
    """Return the ``suspension-assembly/independent-corner`` XML."""

    return (
        _HEADER
        + f'<model schema-version="1.0" id="{model_id}" type="suspension-assembly" subtype="independent-corner">\n'
        "  <parameters>\n"
        f'    <parameter name="inertance_front" value="{inertance_front!r}" unit="kg"/>\n'
        f'    <parameter name="inertance_rear" value="{inertance_rear!r}" unit="kg"/>\n'
        "  </parameters>\n"
        "  <components>\n"
        f'    <component role="kinematics" ref="{kinematics_ref}"/>\n'
        f'    <component role="spring" ref="{spring_ref}"/>\n'
        f'    <component role="arb" ref="{arb_ref}"/>\n'
        f'    <component role="damper-front" ref="{damper_front_ref}"/>\n'
        f'    <component role="damper-rear" ref="{damper_rear_ref}"/>\n'
        "  </components>\n"
        "</model>\n"
    )


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--dss", required=True, help="DSS xlsx workbook or CSV copy of rows 17-31")
    parser.add_argument("--dss-sha256", required=True, help="expected sha256 of the --dss file")
    parser.add_argument("--sheet", default=DEFAULT_SHEET)
    parser.add_argument("--wheel-rate-front-lbf-in", type=float, required=True)
    parser.add_argument("--wheel-rate-rear-lbf-in", type=float, required=True)
    parser.add_argument("--arb-rear-lbf-in", type=float, required=True)
    parser.add_argument("--arb-front-lbf-in", type=float, default=0.0)
    parser.add_argument("--sprung-corner-load-front-n", type=float)
    parser.add_argument("--sprung-corner-load-rear-n", type=float)
    parser.add_argument("--output-npz", required=True)
    parser.add_argument("--output-xml", required=True,
                        help="suspension-assembly XML; the kinematics, spring and ARB part files "
                             "are written to sibling directories of its directory")
    parser.add_argument("--damper-front-ref", default=DEFAULT_DAMPER_FRONT,
                        help="reference from the assembly to the front damper XML")
    parser.add_argument("--damper-rear-ref", default=DEFAULT_DAMPER_REAR,
                        help="reference from the assembly to the rear damper XML")
    return parser


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    args = build_parser().parse_args(argv)
    source = Path(args.dss)
    try:
        digest = _sha256(source)
        if digest != args.dss_sha256.strip().lower():
            raise SuspensionTableError(f"{source}: sha256 mismatch; expected {args.dss_sha256}, got {digest}")
        if source.suffix.lower() == ".csv":
            cells = read_csv_cells(source)
        else:
            cells = read_xlsx_cells(source, args.sheet)
        default_front, default_rear = default_static_loads()
        load_front = default_front if args.sprung_corner_load_front_n is None else args.sprung_corner_load_front_n
        load_rear = default_rear if args.sprung_corner_load_rear_n is None else args.sprung_corner_load_rear_n
        arb_rear = args.arb_rear_lbf_in * LBF_IN_TO_N_M
        arb_front = args.arb_front_lbf_in * LBF_IN_TO_N_M
        if not (math.isfinite(arb_rear) and arb_rear >= 0.0 and math.isfinite(arb_front) and arb_front >= 0.0):
            raise SuspensionTableError("anti-roll bar rates must be finite and nonnegative")
        arrays = build_corner_arrays(
            cells,
            wheel_rate_front_n_m=args.wheel_rate_front_lbf_in * LBF_IN_TO_N_M,
            wheel_rate_rear_n_m=args.wheel_rate_rear_lbf_in * LBF_IN_TO_N_M,
            static_load_front_n=load_front,
            static_load_rear_n=load_rear,
            source_sha256=digest,
        )
        toe_front, toe_rear = static_toe_rad(cells, "front"), static_toe_rad(cells, "rear")
    except (SuspensionTableError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    data = npz_bytes(arrays)
    npz_digest = hashlib.sha256(data).hexdigest()
    npz_path = Path(args.output_npz)
    xml_path = Path(args.output_xml)
    atomic_write_bytes(npz_path, data)
    # The assembly goes to ``--output-xml``. Its three part files sit in sibling
    # directories of the assembly's directory, and each file name is unique
    # because the forward-driver runner snapshots sources by base name.
    models_root = xml_path.resolve().parent.parent
    stem = xml_path.stem
    parts = {
        "kinematics": models_root / "suspension-kinematics" / f"{stem}_kinematics.xml",
        "springs": models_root / "springs" / f"{stem}_springs.xml",
        "arb": models_root / "anti-roll-bars" / f"{stem}_arb.xml",
    }

    def relative_to(path, origin):
        return Path(os.path.relpath(Path(path).resolve(), Path(origin).resolve().parent)).as_posix()

    def part_id(name):
        return parts[name].stem.replace("_", "-")

    atomic_write_bytes(
        parts["kinematics"],
        kinematics_xml(part_id("kinematics"), relative_to(npz_path, parts["kinematics"]), npz_digest,
                       toe_front, toe_rear).encode("utf-8"),
    )
    atomic_write_bytes(
        parts["springs"],
        springs_xml(part_id("springs"), relative_to(npz_path, parts["springs"]),
                    npz_digest).encode("utf-8"),
    )
    atomic_write_bytes(
        parts["arb"], arb_xml(part_id("arb"), arb_front, arb_rear).encode("utf-8"),
    )
    atomic_write_bytes(
        xml_path,
        suspension_xml(
            stem.replace("_", "-"),
            relative_to(parts["kinematics"], xml_path),
            relative_to(parts["springs"], xml_path),
            relative_to(parts["arb"], xml_path),
            args.damper_front_ref, args.damper_rear_ref,
        ).encode("utf-8"),
    )
    print(f"SHA-256: {npz_digest}")
    print(f"static corner loads front/rear: {load_front:.6f} / {load_rear:.6f} N")
    print("command: python3 tools/model_data/generate_suspension_lut.py " + shlex.join(argv))
    return 0


if __name__ == "__main__":
    sys.exit(main())
