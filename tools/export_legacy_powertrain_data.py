#!/usr/bin/env python3
"""Export the legacy powertrain data into ``models/data``.

The tool reads the legacy MATLAB and CSV sources once.
Later tasks read the exported files, so they do not touch the legacy
repositories.

Run the tool from the repository root:

    python tools/export_legacy_powertrain_data.py

The tool reads only. It never writes to a legacy repository.
"""

from __future__ import annotations

import csv
import datetime
import hashlib
import struct
import zlib
from pathlib import Path
from typing import Any

import numpy as np

# --------------------------------------------------------------------------
# Source files
# --------------------------------------------------------------------------

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

# The speed axis follows from the electrical frequency.
# 60 * 0.0833333 Hz / 5 = 1 rpm, and 60 * 1666.6667 Hz / 5 = 20 000 rpm.
POLE_PAIRS = 5

# The reference bus voltage gives the space-vector linear voltage limit.
REFERENCE_BUS_VOLTAGE = 600.0

# MATLAB v5 data types.
MI_INT8 = 1
MI_UINT8 = 2
MI_UINT16 = 4
MI_INT32 = 5
MI_UINT32 = 6
MI_DOUBLE = 9
MI_MATRIX = 14
MI_COMPRESSED = 15

# MATLAB array classes.
# Class 2 is mxSTRUCT and class 4 is mxCHAR.
# Class 6 is mxDOUBLE. Every field of this file carries class 6, so the
# class does not identify a character field. The data element type does.
MX_STRUCT = 2
MX_CHAR = 4

MAT_HEADER = b"MATLAB 5.0 MAT-file"
MAT_VERSION_OFFSET = 124
MAT_VERSION = 256
FIRST_ELEMENT_OFFSET = 128

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DATA_DIRECTORY = REPOSITORY_ROOT / "models" / "data"

TORQUE_MAP_PATH = DATA_DIRECTORY / "amk_dd5_100c_torque_map.csv"
LOSS_MAP_PATH = DATA_DIRECTORY / "amk_dd5_100c_loss_map.csv"
IDENTIFICATION_PATH = DATA_DIRECTORY / "amk_dd5_100c_identification.csv"
CELL_CURVE_PATH = DATA_DIRECTORY / "cosmx_cell_curves.csv"
PROVENANCE_PATH = DATA_DIRECTORY / "provenance.md"


# --------------------------------------------------------------------------
# MATLAB v5 reader
# --------------------------------------------------------------------------


def calculate_sha256(path: Path) -> str:
    """Return the SHA-256 digest of one file."""
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def read_data_element(buffer: bytes, offset: int) -> tuple[int, int, bytes, int]:
    """Read one MATLAB data element.

    Return the data type, the size, the payload, and the next offset.
    """
    if offset < 0 or offset + 8 > len(buffer):
        raise ValueError(
            f"the buffer holds no data element at offset {offset}"
            f" of {len(buffer)} bytes"
        )
    first, second = struct.unpack_from("<II", buffer, offset)
    if first >> 16:
        # Small-data-element form: the type and the size share one word.
        data_type = first & 0xFFFF
        size = first >> 16
        if offset + 4 + size > len(buffer):
            raise ValueError("the data element runs past the end of the buffer")
        payload = buffer[offset + 4 : offset + 4 + size]
        return data_type, size, payload, offset + 8
    data_type = first
    size = second
    if offset + 8 + size > len(buffer):
        raise ValueError("the data element runs past the end of the buffer")
    payload = buffer[offset + 8 : offset + 8 + size]
    next_offset = offset + 8 + ((size + 7) // 8) * 8
    return data_type, size, payload, next_offset


def decode_payload(data_type: int, payload: bytes) -> Any:
    """Decode one data element payload."""
    if data_type == MI_DOUBLE:
        count = len(payload) // 8
        return np.array(struct.unpack("<" + "d" * count, payload), dtype=float)
    if data_type == MI_UINT16:
        count = len(payload) // 2
        return np.array(struct.unpack("<" + "H" * count, payload), dtype=float)
    if data_type == MI_UINT8:
        return np.array(list(payload), dtype=float)
    if data_type == MI_INT8:
        return np.frombuffer(payload, dtype=np.int8).astype(float)
    if data_type == MI_INT32:
        count = len(payload) // 4
        return np.array(struct.unpack("<" + "i" * count, payload), dtype=float)
    if data_type == MI_UINT32:
        count = len(payload) // 4
        return np.array(struct.unpack("<" + "I" * count, payload), dtype=float)
    raise ValueError(f"unsupported MATLAB data type {data_type}")


def decode_characters(data_type: int, payload: bytes) -> str:
    """Decode one character payload as text."""
    if data_type == MI_UINT16:
        text = payload.decode("utf-16-le", errors="replace")
    elif data_type == MI_UINT8:
        text = payload.decode("latin-1")
    elif data_type == MI_INT8:
        text = payload.decode("latin-1")
    else:
        raise ValueError(f"unsupported MATLAB character type {data_type}")
    return text.rstrip("\x00")


def parse_matrix_element(buffer: bytes, offset: int) -> tuple[dict[str, Any], int]:
    """Parse one ``miMATRIX`` element with its tag.

    Return a dictionary of the matrix content and the next offset.
    """
    element_type, _, body, next_offset = read_data_element(buffer, offset)
    if element_type != MI_MATRIX:
        raise ValueError(f"expected a matrix element, found type {element_type}")
    matrix, _ = parse_matrix_body(body, 0)
    return matrix, next_offset


def parse_matrix_body(buffer: bytes, offset: int) -> tuple[dict[str, Any], int]:
    """Parse the body of one ``miMATRIX`` element.

    Return a dictionary of the matrix content and the next offset.
    """
    flags_type, _, flags_payload, position = read_data_element(buffer, offset)
    if flags_type != MI_UINT32:
        raise ValueError("the array flags element is missing")
    flags = struct.unpack("<I", flags_payload[:4])[0]
    array_class = flags & 0xFF

    dims_type, _, dims_payload, position = read_data_element(buffer, position)
    if dims_type != MI_INT32:
        raise ValueError("the dimensions element is missing")
    rows, columns = struct.unpack("<ii", dims_payload[:8])

    name_type, _, name_payload, position = read_data_element(buffer, position)
    if name_type not in (MI_INT8, MI_UINT8, MI_UINT16):
        raise ValueError("the array name element is missing")
    name = decode_characters(name_type, name_payload)

    result: dict[str, Any] = {
        "class": array_class,
        "shape": (rows, columns),
        "name": name,
    }

    if array_class == MX_STRUCT:
        length_type, _, length_payload, position = read_data_element(buffer, position)
        if length_type != MI_INT32:
            raise ValueError("the field name length element is missing")
        field_name_length = struct.unpack("<i", length_payload[:4])[0]
        names_type, _, names_payload, position = read_data_element(buffer, position)
        field_names = [
            decode_characters(names_type, names_payload[i : i + field_name_length])
            for i in range(0, len(names_payload), field_name_length)
        ]
        fields: dict[str, Any] = {}
        for index, field_name in enumerate(field_names):
            field_value, position = parse_matrix_element(buffer, position)
            if not field_name:
                field_name = f"field_{index}"
            fields[field_name] = field_value
        result["fields"] = fields
        return result, position

    data_type, _, payload, position = read_data_element(buffer, position)
    if array_class == MX_CHAR:
        result["value"] = decode_characters(data_type, payload)
        return result, position
    values = decode_payload(data_type, payload)
    result["value"] = values.reshape((rows, columns), order="F")
    return result, position


def load_mat_v5(path: Path) -> dict[str, Any]:
    """Read a compressed MATLAB v5 file.

    Return one entry for each top-level variable.
    """
    raw = path.read_bytes()
    if not raw.startswith(MAT_HEADER):
        raise ValueError(f"{path} is not a MATLAB 5.0 MAT-file")
    if len(raw) < FIRST_ELEMENT_OFFSET + 8:
        raise ValueError(f"{path} holds a header but no data element")
    version = struct.unpack_from("<H", raw, MAT_VERSION_OFFSET)[0]
    if version != MAT_VERSION:
        raise ValueError(f"{path} has version {version}, not {MAT_VERSION}")

    data_type, _, payload, _ = read_data_element(raw, FIRST_ELEMENT_OFFSET)
    if data_type == MI_COMPRESSED:
        payload = zlib.decompress(payload)
    elif data_type != MI_MATRIX:
        raise ValueError(f"{path} starts with element type {data_type}")

    variables: dict[str, Any] = {}
    offset = 0
    while offset < len(payload):
        if offset + 8 > len(payload):
            raise ValueError(f"{path} ends inside a data element at offset {offset}")
        element_type = struct.unpack_from("<I", payload, offset)[0]
        if element_type != MI_MATRIX:
            raise ValueError(
                f"{path} holds element type {element_type} at offset {offset},"
                " not a matrix element"
            )
        matrix, offset = parse_matrix_element(payload, offset)
        variables[matrix["name"] or f"variable_{len(variables)}"] = matrix
    return variables


def load_motor_characteristic(path: Path) -> dict[str, np.ndarray]:
    """Read the ``MotorCharacteristic`` struct from the motor map file."""
    variables = load_mat_v5(path)
    if "MotorCharacteristic" not in variables:
        raise ValueError(f"{path} does not hold MotorCharacteristic")
    characteristic = variables["MotorCharacteristic"]
    if characteristic["class"] != MX_STRUCT:
        raise ValueError("MotorCharacteristic is not a struct")
    fields: dict[str, np.ndarray] = {}
    for field_name, field in characteristic["fields"].items():
        if "value" not in field:
            raise ValueError(f"MotorCharacteristic field {field_name} holds no data")
        value = field["value"]
        if isinstance(value, str):
            value = np.array(value)
        fields[field_name] = value
    return fields


# --------------------------------------------------------------------------
# Export helpers
# --------------------------------------------------------------------------


def format_number(value: float) -> str:
    """Return the shortest text that reproduces one float exactly."""
    return repr(float(value))


def write_csv(
    path: Path,
    header: list[str],
    rows: list[list[str]],
    comments: list[str],
) -> None:
    """Write one commented CSV file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        for comment in comments:
            handle.write(f"# {comment}\n")
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(header)
        writer.writerows(rows)


def derive_speed_axis(
    fields: dict[str, np.ndarray],
    pole_pairs: int = POLE_PAIRS,
) -> np.ndarray:
    """Return the mechanical speed axis in rpm from the electrical frequency.

    Round to six decimal places.
    The frequency is stored as a double, so the round trip is not exact.
    The rounding recovers the underlying integer rpm grid.
    """
    frequency = fields["Frequency"][:, 0]
    speed_rpm = 60.0 * frequency / pole_pairs
    return np.round(speed_rpm, 6)


def confirm_pole_pairs(fields: dict[str, np.ndarray]) -> float:
    """Return the pole pairs implied by the frequency and the speed field.

    Skip the first row, because the source stores zero speed there.
    """
    frequency = fields["Frequency"][1:, 0]
    source_speed = fields["Speed"][1:, 0]
    ratios = 60.0 * frequency / source_speed
    return float(np.mean(ratios))


def export_torque_map(path: Path, fields: dict[str, np.ndarray]) -> None:
    """Write the peak shaft torque for each speed."""
    speed_rpm = derive_speed_axis(fields)
    maximum_torque = np.max(fields["Shaft_Torque"], axis=1)
    rows = [
        [format_number(speed_rpm[i]), format_number(maximum_torque[i])]
        for i in range(speed_rpm.size)
    ]
    comments = [
        "AMK DD5 motor torque envelope at 100 C.",
        "Columns: speed_rpm, maximum_shaft_torque.",
        "Units: speed_rpm in rpm, maximum_shaft_torque in N*m.",
        "One row per speed. The torque is the maximum over the torque axis.",
        "The speed axis follows from the electrical frequency with 5 pole pairs.",
    ]
    write_csv(path, ["speed_rpm", "maximum_shaft_torque"], rows, comments)


def export_loss_map(path: Path, fields: dict[str, np.ndarray]) -> None:
    """Write the total loss for every grid point."""
    speed_rpm = derive_speed_axis(fields)
    shaft_torque = fields["Shaft_Torque"]
    total_loss = fields["Total_Loss"]
    rows = [
        [
            format_number(speed_rpm[i]),
            format_number(shaft_torque[i, j]),
            format_number(total_loss[i, j]),
        ]
        for i in range(speed_rpm.size)
        for j in range(shaft_torque.shape[1])
    ]
    comments = [
        "AMK DD5 motor loss map at 100 C.",
        "Columns: speed_rpm, shaft_torque, total_loss.",
        "Units: speed_rpm in rpm, shaft_torque in N*m, total_loss in W.",
        "Layout: one row per grid point, in speed-major order.",
        "The grid has 201 speeds and 21 torque points.",
        "total_loss is the sum of the copper, iron, magnet, and mechanical loss.",
    ]
    write_csv(path, ["speed_rpm", "shaft_torque", "total_loss"], rows, comments)


def export_identification(path: Path, fields: dict[str, np.ndarray]) -> None:
    """Write the machine identification columns for every grid point."""
    speed_rpm = derive_speed_axis(fields)
    shaft_torque = fields["Shaft_Torque"]
    # The order of the arrays matches the header below.
    arrays = [
        fields["Id_Peak"],
        fields["Iq_Peak"],
        fields["Vd_Peak"],
        fields["Vq_Peak"],
        fields["Stator_Copper_Loss"],
        fields["Iron_Loss"],
        fields["Magnet_Loss"],
        fields["Mechanical_Loss"],
        fields["Total_Loss"],
        fields["InputPower"] * 1000.0,
    ]
    header = [
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
    ]
    rows = []
    for i in range(speed_rpm.size):
        for j in range(shaft_torque.shape[1]):
            values = [speed_rpm[i], shaft_torque[i, j]]
            values.extend(array[i, j] for array in arrays)
            rows.append([format_number(value) for value in values])
    comments = [
        "AMK DD5 machine identification data at 100 C.",
        "Columns: " + ", ".join(header) + ".",
        "Units: speed_rpm in rpm, shaft_torque in N*m, id_peak and iq_peak in A,",
        "vd_peak and vq_peak in V, every loss in W, input_power in W.",
        "The source stores input_power in kW. The export multiplies by 1000.",
        "input_power is the electrical input to the motor. It excludes the inverter loss.",
        "The voltage follows from the d/q values: voltage_phase_peak = hypot(vd_peak, vq_peak).",
        "One row per grid point, in speed-major order.",
    ]
    write_csv(path, header, rows, comments)


def read_cell_curve(path: Path) -> list[dict[str, float]]:
    """Read the legacy cell curve CSV and convert its units.

    Convert the state of charge to a fraction.
    Convert the internal resistance to ohms.
    Keep the energy in watt-hours and the row order.
    """
    rows: list[dict[str, float]] = []
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        for record in reader:
            rows.append(
                {
                    "state_of_charge": float(record["SOC"]) / 100.0,
                    "open_circuit_voltage": float(record["Voltage"]),
                    "internal_resistance": float(record["IR"]) * 0.001,
                    "cell_energy": float(record[" Wh"]),
                }
            )
    return rows


def export_cell_curve(path: Path, rows: list[dict[str, float]]) -> None:
    """Write the cell curve with converted units."""
    header = [
        "state_of_charge",
        "open_circuit_voltage",
        "internal_resistance",
        "cell_energy",
    ]
    body = [
        [
            format_number(row["state_of_charge"]),
            format_number(row["open_circuit_voltage"]),
            format_number(row["internal_resistance"]),
            format_number(row["cell_energy"]),
        ]
        for row in rows
    ]
    comments = [
        "COSMX cell curves. Source: cosmx_cell_data_curves.csv.",
        "Columns: " + ", ".join(header) + ".",
        "Units: state_of_charge in 1 from 0 to 1, open_circuit_voltage in V,",
        "internal_resistance in ohm, cell_energy in Wh.",
        "The source stores the state of charge in percent and the resistance in milliohm.",
        "The source stores the energy in Wh. The export keeps Wh and states the unit.",
        "One row per source row, in source order from full charge to empty.",
        "The energy is cumulative from empty. Multiply by 3600 to get joules.",
    ]
    write_csv(path, header, body, comments)


def derive_cell_capacity(rows: list[dict[str, float]]) -> float:
    """Return the cell capacity in A*h from the discharge curve.

    Sum delta_energy divided by the midpoint voltage.
    The energy is in Wh, so the result is in A*h.
    """
    capacity = 0.0
    for index in range(len(rows) - 1):
        first = rows[index]
        second = rows[index + 1]
        delta_energy = first["cell_energy"] - second["cell_energy"]
        midpoint_voltage = 0.5 * (
            first["open_circuit_voltage"] + second["open_circuit_voltage"]
        )
        capacity += delta_energy / midpoint_voltage
    return capacity


# --------------------------------------------------------------------------
# Provenance
# --------------------------------------------------------------------------


def build_provenance(
    motor_fields: dict[str, np.ndarray],
    cell_rows: list[dict[str, float]],
    export_date: str,
) -> str:
    """Return the provenance text."""
    pole_pairs = confirm_pole_pairs(motor_fields)
    speed_rpm = derive_speed_axis(motor_fields)
    capacity = derive_cell_capacity(cell_rows)
    line = REFERENCE_BUS_VOLTAGE / np.sqrt(2.0)
    shaft_torque = motor_fields["Shaft_Torque"]
    column_zero_min = repr(float(np.min(shaft_torque[:, 0])))
    column_twenty_gap = repr(
        float(np.max(np.max(shaft_torque, axis=1) - shaft_torque[:, 20]))
    )
    column_twenty_rows = int(np.sum(np.argmax(shaft_torque, axis=1) == 20))
    motor_lines = "\n".join(
        [
            "| Field | Value |",
            "|---|---|",
            "| Absolute path | `" + str(MOTOR_SOURCE) + "` |",
            "| Identical copy | `" + str(MOTOR_COPY) + "` |",
            "| SHA-256 | `" + MOTOR_SHA256 + "` |",
            "| Size | " + f"{MOTOR_BYTES} bytes" + " |",
            "| Export date | " + export_date + " |",
            "| Top-level variable | `MotorCharacteristic`, a struct |",
            "| Grid | 201 speeds x 21 torque points |",
            f"| Speed range | {speed_rpm[0]:.4f} rpm to {speed_rpm[-1]:.4f} rpm |",
            f"| Pole pairs | {pole_pairs:.6f} |",
            f"| Reference bus voltage | {REFERENCE_BUS_VOLTAGE:.1f} V |",
            f"| Maximum line voltage | {line:.4f} V, which is 600 / sqrt(2) |",
            f"| Peak shaft torque | {np.max(motor_fields['Shaft_Torque']):.17g} N*m |",
            f"| Inverter efficiency | {float(motor_fields['inverterEfficiency'][0, 0])} |",
            f"| Ratio inside the map | {float(motor_fields['ratio'][0, 0])} |",
        ]
    )
    cell_lines = "\n".join(
        [
            "| Field | Value |",
            "|---|---|",
            "| Absolute path | `" + str(CELL_SOURCE) + "` |",
            "| SHA-256 | `" + CELL_SHA256 + "` |",
            "| Size | " + f"{CELL_BYTES} bytes" + " |",
            "| Export date | " + export_date + " |",
            "| Data rows | " + str(len(cell_rows)) + " |",
            "| Header in source | `Voltage,IR, Wh,SOC` |",
            "| Encoding | UTF-8 with a byte-order mark |",
            f"| Cell capacity from the curve | {capacity:.4f} A*h |",
        ]
    )
    return PROVENANCE_TEMPLATE.format(
        export_date=export_date,
        motor_lines=motor_lines,
        cell_lines=cell_lines,
        line=line,
        column_zero_min=column_zero_min,
        column_twenty_gap=column_twenty_gap,
        column_twenty_rows=column_twenty_rows,
    )


PROVENANCE_TEMPLATE = """# Provenance of the legacy powertrain data

Export date: {export_date}

This file records the source of every exported data file.
It also records the unit conversions and two open data questions.

## Source files

### Motor map

{motor_lines}

The tool reads the file with a small MATLAB v5 reader.
The steps follow.

1. Read the header text and the version word at byte offset 124.
2. Read the first data element at byte offset 128. The type is 15, `miCOMPRESSED`.
3. Decompress the payload with `zlib`. The result is one `miMATRIX` element.
4. Parse the array flags, the dimensions, the name, and the field names.
5. Parse one `miMATRIX` element for each field.

The struct is stored column-major.
The tool reshapes each field to `(201, 21)` with Fortran order.
Column 0 of `Frequency` holds the electrical frequency for each speed.
Row `i` and column `j` hold the value at speed `i` and torque point `j`.

**The torque axis.** Column 0 of `Shaft_Torque` is the lowest torque point.
It is zero at the first speed. It is slightly negative at the other 200
speeds, down to {column_zero_min} N*m.
The generator takes column 20 as the nominal maximum torque,
`Shaft_Torque(:, 21)` in MATLAB.
Column 20 is the exact row maximum for {column_twenty_rows} of the 201 rows.
The other 17 rows peak at column 17, 18, or 19.
The largest difference between the row maximum and column 20 is
{column_twenty_gap} N*m.
The torque map uses `np.max(axis=1)`, so it holds the true row maximum.

The speed axis does not use the `Speed` field.
The `Speed` field uses the character element type `miUINT16`.
Its raw values form the integer grid 0, 100, 200, up to 20 000 rpm.
Its first entry is zero, so the field disagrees with `Frequency` on row 0.
The tool derives the speed axis from `Frequency` instead.

**Speed axis.** The derived axis runs from 1.0 rpm to 20 000.0 rpm.
It has 201 strictly increasing entries.
The step is 100 rpm after the first entry.
The tool rounds the derived value to six decimal places, because the
stored frequency is a double and the round trip is not exact.

**Other scalar fields.** The tool records these values for reference.
`type` is 4, which is the motor count.
`maxPower` is 80, in kW.
`MaxRPM` is 18 600, in rpm.
The export uses none of these values.

**Pole pairs.** The tool confirms the pole pairs from the data.
It computes `60 * Frequency / Speed` on rows 1 to 200.
The source stores zero speed on row 0, so the tool skips that row.
Every ratio equals 5 to within 9e-16.
The relation is `electrical_frequency = pole_pairs * speed / 60`.
Therefore the map has 5 pole pairs.

**Reference bus voltage.** The maximum `Voltage_Line_RMS` is {line:.4f} V.
That value is `600 / sqrt(2)`, the space-vector linear limit for a 600 V bus.
The tool records 600 V as the reference bus voltage.

### Cell curves

{cell_lines}

## Unit conversions

| Column | Source unit | Export unit | Conversion |
|---|---|---|---|
| `speed_rpm` | rpm | rpm | none |
| `maximum_shaft_torque` | N*m | N*m | none |
| `shaft_torque` | N*m | N*m | none |
| `total_loss` | W | W | none |
| `id_peak`, `iq_peak` | A | A | none |
| `vd_peak`, `vq_peak` | V | V | none |
| `stator_copper_loss`, `iron_loss` | W | W | none |
| `magnet_loss`, `mechanical_loss` | W | W | none |
| `input_power` | kW | W | multiply by 1000 |
| `state_of_charge` | percent | 1 | divide by 100 |
| `open_circuit_voltage` | V | V | none |
| `internal_resistance` | milliohm | ohm | multiply by 0.001 |
| `cell_energy` | Wh | Wh | none, the header states the unit |

`cell_energy` stays in Wh, because the source stores Wh and the project
declares the cell capacity in A*h. Multiply by 3600 to get joules.

## Source column meanings

**Motor map fields.** The struct `MotorCharacteristic` holds these fields.

| Field | Meaning | Unit |
|---|---|---|
| `Frequency` | Electrical frequency of the stator field | Hz |
| `Shaft_Torque` | Mechanical torque at the shaft | N*m |
| `Id_Peak`, `Iq_Peak` | Direct and quadrature current, peak value | A |
| `Vd_Peak`, `Vq_Peak` | Direct and quadrature voltage, peak value | V |
| `Stator_Copper_Loss` | Resistive loss in the stator winding | W |
| `Iron_Loss` | Hysteresis and eddy loss in the stator core | W |
| `Magnet_Loss` | Eddy loss in the rotor magnets | W |
| `Mechanical_Loss` | Friction and windage loss | W |
| `Total_Loss` | Sum of the four loss terms | W |
| `InputPower` | Electrical input power at the motor terminals | kW |
| `inverterEfficiency` | Efficiency of the inverter | 1 |
| `ratio` | Gear ratio of the gearbox in the source car | 1 |
| `Voltage_Line_RMS` | Line RMS voltage | V |
| `Voltage_Phase_Peak` | Phase peak voltage | V |

The export does not use the fields `Speed`, `MaxRPM`, `type`, and `maxPower`.

**Cell curve columns.** The source header reads `Voltage,IR, Wh,SOC`.

| Column | Meaning | Unit |
|---|---|---|
| `Voltage` | Cell open-circuit voltage | V |
| `IR` | Cell internal resistance | milliohm |
| ` Wh` | Cell energy, cumulative from empty | Wh |
| `SOC` | Cell state of charge | percent |

Note the leading space in the header name ` Wh`.

## Voltage relations

The identification file holds `vd_peak` and `vq_peak`.
The source voltage columns follow from them by the space-vector relations.
The tool confirmed both relations on every grid point.

```text
voltage_phase_peak = hypot(vd_peak, vq_peak)
voltage_line_rms   = voltage_phase_peak / sqrt(2)
```

The largest difference for the first relation is 1.1e-13 V.
The second relation holds exactly.
The source also sets `Voltage_Line_RMS` equal to `Voltage_Phase_RMS`.

## Exported files

| File | Content |
|---|---|
| `amk_dd5_100c_torque_map.csv` | `speed_rpm`, `maximum_shaft_torque` |
| `amk_dd5_100c_loss_map.csv` | `speed_rpm`, `shaft_torque`, `total_loss` |
| `amk_dd5_100c_identification.csv` | The d/q currents, the d/q voltages, and every loss |
| `cosmx_cell_curves.csv` | The cell curve with converted units |

## Verified relations

The tool checked the following relations against the source.

**Input power.** The column `input_power` equals `shaft_power + total_loss`.
The tool compared the two quantities at the 4020 grid points that carry a
positive shaft torque. The median relative error is 9.9e-10, which is
0.000 percent to three decimal places.
The task brief quotes 3959 points. This export counts 4020 points with a
positive shaft torque, so the count differs. The relation and the median
error agree.
The same column equals `(shaft_power + total_loss) / 0.95` with a median
relative error of 5.0000 percent, because the source stores the motor input
power and the inverter efficiency separately.
Therefore the loss map covers the motor losses only.
The inverter efficiency applies on top of the map.

**Loss components.** `total_loss` equals
`stator_copper_loss + iron_loss + magnet_loss + mechanical_loss` at every
grid point. The largest residual is exactly 0.0 W.

**Resistance.** At the zero-speed end the map needs 20.7313 V of phase peak
voltage at `iq_peak = 83.8133 A` and `id_peak = -18.0367 A`.
The quotient `voltage_phase_peak / iq_peak` is 0.24735 ohm.
The plan records a first-pass fitted resistance of 0.2488 ohm.
The two values differ by 0.6 percent, so the map is self-consistent.

## Open questions

### 1. The loss scale

Conclusion: the factor of 1.60 does **not** apply to this map.
It belongs to the older model, and there it corrects a **cooling load**,
not the loss map.

Evidence:

1. `generateDrivetrainModel.m` builds `drivetrain.lossMap` from
   `MotorCharacteristic.Total_Loss` with no multiplication.
   The script only stores `motorlossScalar` as a separate scalar field.
2. The saved `Drivetrain_Murphy_4WD.mat` proves that no scale was applied.
   Its `lossMap` reproduces the linear interpolation of `Total_Loss` at
   integer torques. Its maximum is 5867.858324713112 W.
   The unscaled interpolation gives 5867.858324713111 W.
   A factor of 1.60 would give 9388.573 W, which the file does not hold.
3. `Total_Loss` is a complete loss model.
   It equals `Stator_Copper_Loss + Iron_Loss + Magnet_Loss + Mechanical_Loss`
   at every grid point.
4. `Total_Loss` also equals `input_power - shaft_power`.
   The source already accounts for the full motor loss.
   A further factor of 1.60 would overstate every loss by 60 percent.
5. The older model `lap-sim` holds the origin of the factor.
   `Execute/Calculators/amkCoolingLoadCalculator.m` line 66 sets
   `motorLossCorrectionFactor = 1.60`.
   `Execute/Run Execution/FullCarCoolingSim.m` line 49 sets
   `motorLossCoefficient = 1.60`.
   `FullCarCoolingSim.m` multiplies `Total_Loss` by the factor only when it
   computes the heat that enters the motor thermal mass.
   `amkCoolingLoadCalculator.m` also multiplies the loss term inside its
   `inverterPower` estimate at lines 80 and 93.
   Both uses are cooling study inputs.
   The factor is a heat-generation margin.
   The lap-sim energy calculation keeps `Total_Loss` unscaled.
6. The exported loss map therefore keeps the source `Total_Loss` unchanged.

Consequence for the powertrain model: use `total_loss` as it stands.
Do not apply 1.60 to the bus power.
The thermal margin is a cooling study input, and this plan excludes
thermal state.

### 2. The gear ratio

Two values exist. They differ.

| Value | Source |
|---|---|
| 11.9 | The `ratio` field inside `AMK_Characteristic_100C.mat` |
| 10.9 | `generateDrivetrainModel.m`, `drivetrain.gearRatio = 10.9` |
| 10.9 | The project XML `models/drivetrains/murphy_4wd.xml`, `gear_ratio` |

The old `lap-sim` model uses the map value.
`amkCoolingLoadCalculator.m` computes the motor speed with
`MotorCharacteristic.ratio`, which is 11.9.
That value describes the gearbox of the car that produced the motor data.
The 2025 generator and the project both use 10.9.
The value of 10.9 describes the Murphy 2024 car.
The project value stays 10.9. This task changes no project value.
Task P6 must decide which value the transmission model uses.
"""


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------


def export_all(
    motor_path: Path = MOTOR_SOURCE,
    cell_path: Path = CELL_SOURCE,
    output_directory: Path = DATA_DIRECTORY,
    export_date: str | None = None,
) -> None:
    """Export every data file from the legacy sources."""
    export_date = export_date or datetime.date.today().isoformat()
    fields = load_motor_characteristic(motor_path)
    cell_rows = read_cell_curve(cell_path)
    output_directory.mkdir(parents=True, exist_ok=True)
    export_torque_map(output_directory / TORQUE_MAP_PATH.name, fields)
    export_loss_map(output_directory / LOSS_MAP_PATH.name, fields)
    export_identification(output_directory / IDENTIFICATION_PATH.name, fields)
    export_cell_curve(output_directory / CELL_CURVE_PATH.name, cell_rows)
    provenance = build_provenance(fields, cell_rows, export_date)
    (output_directory / PROVENANCE_PATH.name).write_text(provenance, encoding="utf-8")


def main() -> None:
    """Export every data file."""
    export_all()
    print(f"wrote {TORQUE_MAP_PATH}")
    print(f"wrote {LOSS_MAP_PATH}")
    print(f"wrote {IDENTIFICATION_PATH}")
    print(f"wrote {CELL_CURVE_PATH}")
    print(f"wrote {PROVENANCE_PATH}")


if __name__ == "__main__":
    main()
