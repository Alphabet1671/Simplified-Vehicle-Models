"""Deterministically generate a damper force table NPZ and damper model XMLs.

The input is a Multimatic DSSV style table: one row per click, one column per
damper velocity, force in the cell. The input can be the source xlsx (read with
openpyxl, imported lazily) or a CSV copy of the same layout.

The generator checks the table before it writes anything:

* force is non-decreasing with click at every velocity (ties are allowed);
* force strictly increases with click wherever |v| >= 25 mm/s;
* force strictly increases with |v| for every click;
* force is zero at v = 0 for every click.

A defect aborts the run unless a ``--correct "click,velocity,force"`` entry
supplies the value that removes it. Every applied correction is recorded in the
NPZ ``corrections`` key. The NPZ is byte-reproducible: the zip entries carry a
fixed timestamp and are stored uncompressed, and no path or time enters the
bytes.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import os
import shlex
import sys
import zipfile
from pathlib import Path

import numpy as np

GENERATOR_VERSION = "damper-lut-v1"
SCHEMA = "damper-table-v1"
DEFAULT_SHEET = "Multimatic_DSSV_VC01"
# Ties in click are physical below this |v| (the sheet has equal forces at 0 and +-10 mm/s).
STRICT_CLICK_SPEED_MM_S = 25.0
_VELOCITY_SCALE = {"mm/s": 1.0e-3, "m/s": 1.0}
_FORCE_SCALE = {"N": 1.0, "kN": 1.0e3}
_ZIP_TIME = (1980, 1, 1, 0, 0, 0)


class DamperTableError(ValueError):
    """Report a table defect or an invalid generator input."""


def _cell_name(click_index: int, velocity_index: int) -> str:
    """Return the sheet cell of the source layout (header row 1, click column A)."""
    column = velocity_index + 2  # B is the first velocity column
    letters = ""
    while column > 0:
        column, remainder = divmod(column - 1, 26)
        letters = chr(ord("A") + remainder) + letters
    return f"{letters}{click_index + 2}"


# ---------------------------------------------------------------- readers
def read_csv_table(path):
    """Read a CSV table. Lines that start with ``#`` are provenance comments.

    The first data row is ``click,<velocity>...``; later rows are ``<click>,<force>...``.
    """
    with Path(path).open(newline="") as stream:
        rows = [
            row for row in csv.reader(stream)
            if row and not row[0].lstrip().startswith("#")
        ]
    if len(rows) < 2:
        raise DamperTableError(f"{path}: no table rows")
    velocity = np.array([float(x) for x in rows[0][1:]], dtype=float)
    clicks = np.array([float(row[0]) for row in rows[1:]], dtype=float)
    force = np.array([[float(x) for x in row[1:]] for row in rows[1:]], dtype=float)
    return velocity, clicks, force


def read_xlsx_table(path, sheet):
    """Read the xlsx sheet: B1.. velocities, A2.. clicks, cells hold force."""
    try:
        import openpyxl  # lazy: only the xlsx path needs it
    except ImportError as error:  # pragma: no cover - environment dependent
        raise DamperTableError("reading xlsx requires openpyxl") from error
    workbook = openpyxl.load_workbook(path, data_only=True, read_only=True)
    try:
        if sheet not in workbook.sheetnames:
            raise DamperTableError(f"{path}: no sheet {sheet!r}; found {workbook.sheetnames}")
        rows = [list(r) for r in workbook[sheet].iter_rows(values_only=True)]
    finally:
        workbook.close()
    header = rows[0]
    columns = [i for i in range(1, len(header)) if header[i] is not None]
    body = []
    for row in rows[1:]:
        if row[0] is None:
            break
        body.append(row)
    velocity = np.array([header[i] for i in columns], dtype=float)
    clicks = np.array([row[0] for row in body], dtype=float)
    cells = [[row[i] if i < len(row) else None for i in columns] for row in body]
    if any(cell is None for line in cells for cell in line):
        raise DamperTableError(f"{path}: sheet {sheet!r} has empty table cells")
    return velocity, clicks, np.array(cells, dtype=float)


# ---------------------------------------------------------------- processing
def parse_correction(text):
    """Parse ``"click,velocity,force"`` (velocity and force in the input units)."""
    parts = [p.strip() for p in str(text).split(",")]
    if len(parts) != 3:
        raise DamperTableError(f"--correct expects 'click,velocity,force', got {text!r}")
    try:
        click, velocity, force = int(parts[0]), float(parts[1]), float(parts[2])
    except ValueError as error:
        raise DamperTableError(f"--correct expects numbers, got {text!r}") from error
    return click, velocity, force


def _defects(velocity_mm_s, force, strict_speed_mm_s):
    """Return human-readable defects of a table in sheet orientation (click, velocity)."""
    problems = []
    n_click, n_v = force.shape
    for j, v in enumerate(velocity_mm_s):
        for k in range(1, n_click):
            low, high = abs(force[k - 1, j]), abs(force[k, j])
            strict = abs(v) >= strict_speed_mm_s
            if high < low or (strict and high == low):
                problems.append(
                    f"force magnitude does not increase with click at click {k}, "
                    f"v={v:+g} mm/s (sheet cell {_cell_name(k, j)}): "
                    f"{force[k - 1, j]:g} -> {force[k, j]:g}"
                )
    zero = int(np.flatnonzero(velocity_mm_s == 0.0)[0])
    for side, order in (("compression", range(zero, n_v)), ("rebound", range(zero, -1, -1))):
        order = list(order)
        for k in range(n_click):
            for a, b in zip(order, order[1:]):
                if abs(force[k, b]) <= abs(force[k, a]):
                    problems.append(
                        f"force magnitude does not increase with |v| ({side}) at click {k}, "
                        f"v={velocity_mm_s[b]:+g} mm/s (sheet cell {_cell_name(k, b)}): "
                        f"{force[k, a]:g} -> {force[k, b]:g}"
                    )
    return problems


def process_damper_table(
    velocity,
    clicks,
    force,
    *,
    velocity_unit="mm/s",
    force_unit="N",
    rebound_sign="negative",
    corrections=(),
    source_sha256="",
    strict_click_speed_mm_s=STRICT_CLICK_SPEED_MM_S,
):
    """Validate, correct, and convert a raw table. Pure: no file access.

    ``force`` has shape (clicks, velocities) in the sheet layout. ``corrections``
    is a sequence of ``(click, velocity, force)`` in the input units. Returns the
    ``damper-table-v1`` arrays as a dict (key order is the NPZ entry order).
    Raises ``DamperTableError`` naming the cell of any defect that no
    correction removes.
    """
    if velocity_unit not in _VELOCITY_SCALE:
        raise DamperTableError(f"unsupported velocity unit {velocity_unit!r}")
    if force_unit not in _FORCE_SCALE:
        raise DamperTableError(f"unsupported force unit {force_unit!r}")
    if rebound_sign not in ("negative", "positive"):
        raise DamperTableError("rebound_sign must be 'negative' or 'positive'")
    velocity = np.array(velocity, dtype=float)
    clicks = np.array(clicks, dtype=float)
    force = np.array(force, dtype=float)
    if velocity.ndim != 1 or clicks.ndim != 1 or force.shape != (clicks.size, velocity.size):
        raise DamperTableError("table shape does not match velocity and click axes")
    if not (np.all(np.isfinite(velocity)) and np.all(np.isfinite(force))):
        raise DamperTableError("table must be finite")
    if not np.array_equal(clicks, np.arange(clicks.size)):
        raise DamperTableError("click column must be 0..n-1 in order")
    if np.any(np.diff(velocity) <= 0.0):
        raise DamperTableError("velocity axis must be strictly increasing")
    vel_mm_s = velocity * (_VELOCITY_SCALE[velocity_unit] / 1.0e-3)
    zeros = np.flatnonzero(vel_mm_s == 0.0)
    if zeros.size != 1 or zeros[0] == 0 or zeros[0] == vel_mm_s.size - 1:
        raise DamperTableError("velocity axis must contain 0 with rebound and compression knots")
    zero = int(zeros[0])
    # Work on signed force in N: compression positive, rebound negative.
    signed = force * _FORCE_SCALE[force_unit]
    if rebound_sign == "positive":
        signed[:, :zero] *= -1.0
    if np.any(signed[:, zero] != 0.0):
        raise DamperTableError("force at zero velocity must be zero for every click")
    if np.any(signed[:, zero + 1:] <= 0.0) or np.any(signed[:, :zero] >= 0.0):
        raise DamperTableError("compression force must be positive and rebound force negative")

    applied = []
    work = signed.copy()
    for click, cv, value in corrections:
        columns = np.flatnonzero(np.isclose(vel_mm_s, cv * (_VELOCITY_SCALE[velocity_unit] / 1.0e-3), rtol=0.0, atol=1.0e-9))
        if not 0 <= click < clicks.size or columns.size != 1:
            raise DamperTableError(f"correction ({click}, {cv}, {value}) names no table cell")
        j = int(columns[0])
        new = value * _FORCE_SCALE[force_unit]
        if rebound_sign == "positive" and j < zero:
            new = -new
        old = float(work[click, j])
        if old == new:
            raise DamperTableError(f"correction for click {click}, v={cv:+g} does not change the cell")
        work[click, j] = new
        applied.append(
            {
                "cell": _cell_name(click, j),
                "click": int(click),
                "from_N": old,
                "to_N": float(new),
                "velocity_m_s": float(vel_mm_s[j] * 1.0e-3),
            }
        )
    problems = _defects(vel_mm_s, work, strict_click_speed_mm_s)
    if problems:
        hint = "" if applied else " (supply --correct 'click,velocity,force' to repair a source defect)"
        raise DamperTableError(
            "damper table is not monotone: " + "; ".join(problems[:5])
            + ("" if len(problems) <= 5 else f"; and {len(problems) - 5} more") + hint
        )
    # Unused correction check: a correction must remove a defect in the raw table.
    for entry in applied:
        trial = signed.copy()
        for other in applied:
            if other is not entry:
                trial[other["click"], int(np.flatnonzero(np.isclose(vel_mm_s, other["velocity_m_s"] * 1.0e3))[0])] = other["to_N"]
        if not _defects(vel_mm_s, trial, strict_click_speed_mm_s):
            raise DamperTableError(
                f"correction at click {entry['click']}, cell {entry['cell']} is not needed"
            )

    v_m_s = vel_mm_s * 1.0e-3
    n_click = clicks.size
    arrays = {
        "schema": np.asarray(SCHEMA),
        "velocity_compression": np.ascontiguousarray(v_m_s[zero:]),
        "velocity_rebound": np.ascontiguousarray(v_m_s[: zero + 1]),
        "valve_names": np.asarray(["compression", "rebound"]),
        "valve_halves": np.asarray(["compression", "rebound"]),
        "valve_bands": np.asarray(["full", "full"]),
        "valve_clicks": np.asarray([n_click, n_click], dtype=np.int64),
        "force_compression": np.ascontiguousarray(work[:, zero:].T),
        "force_rebound": np.ascontiguousarray(work[:, : zero + 1].T),
        "source_sha256": np.asarray(str(source_sha256)),
        "generator_version": np.asarray(GENERATOR_VERSION),
        "corrections": np.asarray(json.dumps(applied, sort_keys=True, separators=(",", ":"))),
    }
    return arrays


# ---------------------------------------------------------------- output
def npz_bytes(arrays):
    """Serialize arrays to deterministic NPZ bytes (stored, fixed timestamps)."""
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w", zipfile.ZIP_STORED) as archive:
        for name, array in arrays.items():
            payload = io.BytesIO()
            np.lib.format.write_array(payload, np.asanyarray(array), allow_pickle=False)
            info = zipfile.ZipInfo(f"{name}.npy", date_time=_ZIP_TIME)
            info.compress_type = zipfile.ZIP_STORED
            info.create_system = 3
            info.external_attr = 0o600 << 16
            archive.writestr(info, payload.getvalue())
    return buffer.getvalue()


def atomic_write_bytes(path, data):
    """Write ``data`` through a sibling temp file and ``os.replace``.

    The temp file is opened explicitly, so no library appends a suffix to it.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    try:
        with temporary.open("wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def damper_xml(model_id, subtype, npz_relative, digest, click_compression, click_rebound):
    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        f"<!-- Generated by tools/model_data/generate_damper_lut.py ({GENERATOR_VERSION}). Do not edit.\n"
        "     Multimatic DSSV VC01 damper. Positive velocity is compression; the table is in the NPZ.\n"
        "     The NPZ sha256 pins the table. Provenance: docs/model-data/provenance.md. -->\n"
        f'<model schema-version="1.0" id="{model_id}" type="damper-model" subtype="{subtype}">\n'
        "  <parameters>\n"
        f'    <parameter name="click_compression" value="{click_compression}" unit="1"/>\n'
        f'    <parameter name="click_rebound" value="{click_rebound}" unit="1"/>\n'
        "  </parameters>\n"
        f'  <data role="damper-table" path="{npz_relative}" format="npz" sha256="{digest}"/>\n'
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
    parser.add_argument("source", help="xlsx workbook or CSV table")
    parser.add_argument("--sheet", default=DEFAULT_SHEET)
    parser.add_argument("--velocity-unit", default="mm/s", choices=sorted(_VELOCITY_SCALE))
    parser.add_argument("--force-unit", default="N", choices=sorted(_FORCE_SCALE))
    parser.add_argument("--rebound-sign", default="negative", choices=("negative", "positive"))
    parser.add_argument("--correct", action="append", default=[], metavar="CLICK,VELOCITY,FORCE")
    parser.add_argument("--output-npz", required=True)
    parser.add_argument("--output-xml-front")
    parser.add_argument("--output-xml-rear")
    parser.add_argument("--click-compression", type=int, default=5)
    parser.add_argument("--click-rebound", type=int, default=7)
    return parser


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    args = build_parser().parse_args(argv)
    source = Path(args.source)
    try:
        if source.suffix.lower() == ".csv":
            velocity, clicks, force = read_csv_table(source)
        else:
            velocity, clicks, force = read_xlsx_table(source, args.sheet)
        arrays = process_damper_table(
            velocity, clicks, force,
            velocity_unit=args.velocity_unit,
            force_unit=args.force_unit,
            rebound_sign=args.rebound_sign,
            corrections=[parse_correction(c) for c in args.correct],
            source_sha256=_sha256(source),
        )
        for name, value in (("compression", args.click_compression), ("rebound", args.click_rebound)):
            if not 0 <= value < int(arrays["valve_clicks"][0]):
                raise DamperTableError(f"baseline {name} click {value} outside the table")
    except (DamperTableError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    data = npz_bytes(arrays)
    digest = hashlib.sha256(data).hexdigest()
    npz_path = Path(args.output_npz)
    atomic_write_bytes(npz_path, data)
    for subtype, target in (("table-front", args.output_xml_front), ("table-rear", args.output_xml_rear)):
        if target is None:
            continue
        target = Path(target)
        relative = Path(os.path.relpath(npz_path.resolve(), target.resolve().parent)).as_posix()
        model_id = target.stem.replace("_", "-")
        atomic_write_bytes(
            target,
            damper_xml(model_id, subtype, relative, digest, args.click_compression, args.click_rebound).encode("utf-8"),
        )
    print(f"SHA-256: {digest}")
    print(f"corrections: {arrays['corrections'].item()}")
    print("command: python3 tools/model_data/generate_damper_lut.py " + shlex.join(argv))
    return 0


if __name__ == "__main__":
    sys.exit(main())
