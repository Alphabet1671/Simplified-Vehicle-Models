"""Import a Formula SAE design spec sheet as obfuscated vehicle parameters.

The source sheet describes one real car. This tool does not copy the real
values into the repository. It rounds each value to two significant figures
and then skews it, so the repository carries plausible placeholders instead of
a real car's numbers.

The skew is deterministic in the parameter name. One source file therefore
always gives the same export. The skew is not cryptography. The goal is that
the repository does not publish a real car's data, not that the source is
unrecoverable.

Replace every value with a measured value before any correlated result. The
emitted XML records this requirement.

Values that the sheet does not contain are separate. The yaw radius of
gyration and the wheel inertia come from a general estimation request, and the
steering limits, the brake bias and the peak braking deceleration are stated
placeholders. Those values are emitted unchanged and are marked as estimates.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import re
import math
from pathlib import Path
import xml.etree.ElementTree as ElementTree


SHEET_NAME = "FSAE_Specs_EV"

VEHICLE_ID = "placeholder-2024-formula"
AERO_ID = "placeholder-2024-aero"

#: Deterministic skew half width as a fraction of the rounded value.
SKEW = 0.06

#: Significant figures kept from the real value before the skew.
SIGNIFICANT_DIGITS = 2

#: Cell references in the design spec sheet.
CELLS = {
    "wheelbase_mm": "D8",
    "track_front_mm": "F8",
    "track_rear_mm": "H8",
    "cg_height_mm": "D9",
    "mass_front_kg": "D10",
    "mass_rear_kg": "F10",
    "mass_total_kg": "H10",
    "front_weight_fraction": "D11",
    "wheel_rate_front": "C18",
    "roll_rate_front": "C19",
    "roll_rate_rear": "F19",
    "caster_deg": "D32",
    "kingpin_inclination_deg": "D33",
    "steer_ratio": "D36",
    "steer_arm_mm": "H36",
    "brake_pressure_front_bar": "D43",
    "brake_pressure_rear_bar": "F43",
    "brake_pedal_force_1g_kn": "H43",
    "brake_pressure_front_1g_bar": "D43",
    "motor_max_rpm": "C84",
    "motor_base_rpm": "C85",
    "motor_peak_torque": "C86",
    "motor_continuous_torque": "C87",
    "motor_peak_power_kw": "C88",
    "motor_continuous_power_kw": "C89",
    "motor_count": "C81",
    "gear_ratio": "C111",
    "pack_configuration": "C99",
    "pack_peak_power_kw": "C100",
    "pack_continuous_power_kw": "C101",
    "pack_charge_power_kw": "C104",
    "rotor_front": "C39",
    "rotor_rear": "F39",
    "downforce_n": "D119",
    "downforce_front_fraction": "F119",
    "drag_n": "H119",
    "reference_area_m2": "F120",
}


def sha256(path: Path) -> str:
    """Return the SHA-256 digest of one file."""

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def round_significant(value: float, digits: int) -> float:
    """Round one value to a number of significant figures."""

    if value == 0.0 or not math.isfinite(value):
        return float(value)
    exponent = math.floor(math.log10(abs(value)))
    return float(round(value, -(exponent - (digits - 1))))


def skew_factor(name: str, spread: float = SKEW) -> float:
    """Return one deterministic multiplicative skew for a parameter name."""

    digest = hashlib.sha256(name.encode("utf-8")).digest()
    unit = int.from_bytes(digest[:8], "big") / float(1 << 64)
    return 1.0 + spread * (2.0 * unit - 1.0)


def obfuscate(value: float, name: str, *, digits: int = 4) -> float:
    """Round a real value to two significant figures and skew it."""

    rounded = round_significant(value, SIGNIFICANT_DIGITS)
    return float(round(rounded * skew_factor(name), digits))


#: Cells that hold text rather than a number.
TEXT_CELLS = ("rotor_front", "rotor_rear")

#: Cells that hold a number followed by text, such as ``21 Nm for 1.24 s``.
NUMBER_PREFIX_CELLS = (
    "motor_peak_torque",
    "motor_continuous_torque",
    "motor_peak_power_kw",
    "motor_continuous_power_kw",
    "gear_ratio",
    "motor_count",
    "pack_peak_power_kw",
    "pack_continuous_power_kw",
    "pack_charge_power_kw",
)

#: Matches the first number of a cell.
_LEADING_NUMBER = re.compile(r"(-?\d+(?:\.\d+)?)")

#: Matches a pack configuration such as ``144S1P``.
_PACK_CONFIGURATION = re.compile(r"(\d+)\s*S\s*(\d+)\s*P")

#: Matches the rotor diameter before the diameter symbol, so that a thickness
#: such as ``3/16"`` is not mistaken for the diameter.
_ROTOR_DIAMETER = re.compile(r"([\d.]+)\s*\"\s*\u00d8")


def read_cells(path: Path) -> dict[str, float]:
    """Read every mapped cell.

    A numeric cell returns a number. A text cell returns the rotor diameter in
    inches, parsed from its description. The spreadsheet library is imported
    here, so the pure helpers stay usable without it.
    """

    from openpyxl import load_workbook

    book = load_workbook(path, data_only=True)
    sheet = book[SHEET_NAME]
    values: dict[str, float] = {}
    for name, reference in CELLS.items():
        raw = sheet[reference].value
        if raw is None:
            raise ValueError(f"{path}: cell {reference} for {name!r} is empty")
        if name == "pack_configuration":
            found = _PACK_CONFIGURATION.search(str(raw).replace(" ", ""))
            if found is None:
                raise ValueError(
                    f"{path}: cell {reference} is not a series-parallel configuration"
                )
            values["pack_series_count"] = float(found.group(1))
            values["pack_parallel_count"] = float(found.group(2))
            continue
        if name in NUMBER_PREFIX_CELLS:
            found = _LEADING_NUMBER.search(str(raw))
            if found is None:
                raise ValueError(
                    f"{path}: cell {reference} for {name!r} holds no number: {raw!r}"
                )
            values[name] = float(found.group(1))
            continue
        if name in TEXT_CELLS:
            text = str(raw)
            found = _ROTOR_DIAMETER.search(text)
            if found is None:
                raise ValueError(
                    f"{path}: cell {reference} for {name!r} holds no diameter: {raw!r}"
                )
            values[name] = float(found.group(1))
            continue
        try:
            values[name] = float(raw)
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"{path}: cell {reference} for {name!r} is not numeric: {raw!r}"
            ) from error
    return values


def build_parameters(
    raw: dict[str, float],
    driver_mass: float,
    brake_bias: float,
    maximum_steer_deg: float,
    maximum_steer_rate_deg_s: float,
    rack_travel_per_revolution: float,
    maximum_pedal_force: float,
    hydraulic_ratio: float,
    maximum_brake_pressure: float,
    caliper_area: float,
    rotor_effective_fraction: float,
    pad_mu: float,
    wheel_mass: float,
    wheel_diameter_in: float,
    wheel_width_in: float,
    yaw_inertia_reference: float,
    yaw_mass_reference: float,
) -> tuple[dict[str, float], dict[str, float], dict[str, float], float]:
    """Return the vehicle scalars, the aero scalars, and the source values.

    The third mapping holds the real values. The caller must not write it to
    the repository.
    """

    real = dict(raw)
    real["driver_mass_kg"] = driver_mass

    mass_without_driver = obfuscate(real["mass_total_kg"], "mass_without_driver")
    total_mass = mass_without_driver + driver_mass
    wheelbase = obfuscate(real["wheelbase_mm"] / 1000.0, "wheelbase")
    track_width = obfuscate(real["track_front_mm"] / 1000.0, "track_width")
    cg_height = obfuscate(real["cg_height_mm"] / 1000.0, "cg_height")
    front_mass_fraction = obfuscate(
        real["front_weight_fraction"] / 100.0, "front_mass_fraction"
    )
    roll_total = real["roll_rate_front"] + real["roll_rate_rear"]
    load_transfer_distribution = obfuscate(
        real["roll_rate_front"] / roll_total, "lateral_load_transfer_distribution"
    )

    # The yaw radius of gyration follows a general estimation request, not the
    # sheet. It scales the yaw inertia with the loaded mass.
    yaw_radius = math.sqrt(yaw_inertia_reference / yaw_mass_reference)
    yaw_inertia = total_mass * yaw_radius**2

    # The wheel is a solid cylinder of the stated size and mass.
    wheel_radius = wheel_diameter_in * 0.0254 / 2.0
    wheel_inertia = 0.5 * wheel_mass * wheel_radius**2

    # The sheet gives a line pressure at a known pedal force. Calibrate the
    # pedal ratio from that one operating point, so the brake model reproduces
    # the sheet instead of a guess.
    pedal_force_1g = real["brake_pedal_force_1g_kn"] * 1000.0
    pressure_1g = real["brake_pressure_front_1g_bar"] * 1.0e5
    # The balance bar sends only the front master-cylinder share
    # (1 + brake_bias) / 2 to the front circuit, so the bias belongs in the
    # calibration. brake_bias is signed: 0 neutral, +1 all front, -1 all rear.
    pedal_ratio = (
        pressure_1g * caliper_area
        / (pedal_force_1g * hydraulic_ratio * 0.5 * (1.0 + brake_bias))
    )

    # The effective radius is a fraction of the rotor radius.
    front_radius = (
        rotor_effective_fraction * real["rotor_front"] * 0.0254 / 2.0
    )
    rear_radius = (
        rotor_effective_fraction * real["rotor_rear"] * 0.0254 / 2.0
    )
    obfuscated_pedal_ratio = obfuscate(pedal_ratio, "pedal_ratio")

    vehicle = {
        "total_mass": round(total_mass, 4),
        "mass_without_driver": mass_without_driver,
        "driver_mass": driver_mass,
        "wheelbase": wheelbase,
        "track_width": track_width,
        "cg_height": cg_height,
        "front_mass_fraction": front_mass_fraction,
        "lateral_load_transfer_distribution": load_transfer_distribution,
        "yaw_radius_of_gyration": round(yaw_radius, 6),
        "wheel_inertia": round(wheel_inertia, 6),
        "steer_ratio": obfuscate(real["steer_ratio"], "steer_ratio"),
        "caster_angle": math.radians(
            obfuscate(real["caster_deg"], "caster_angle")
        ),
        "kingpin_inclination_angle": math.radians(
            obfuscate(real["kingpin_inclination_deg"], "kingpin_inclination_angle")
        ),
        "rack_travel_per_revolution": rack_travel_per_revolution,
        "steering_maximum_angle": math.radians(maximum_steer_deg),
        "steering_maximum_rate": math.radians(maximum_steer_rate_deg_s),
        "steer_arm_length": obfuscate(
            real["steer_arm_mm"] / 1000.0, "steer_arm_length"
        ),
        "brake_bias": brake_bias,
        "maximum_pedal_force": round(maximum_pedal_force, 4),
        "hydraulic_ratio_front": hydraulic_ratio,
        "hydraulic_ratio_rear": hydraulic_ratio,
        "maximum_brake_pressure": maximum_brake_pressure,
        "caliper_area": caliper_area,
        "pad_mu_front": pad_mu,
        "pad_mu_rear": pad_mu,
        "brake_effective_radius_front": round(front_radius, 6),
        "brake_effective_radius_rear": round(rear_radius, 6),
    }

    reference_speed = 80.0 / 3.6
    dynamic_pressure = 0.5 * 1.162 * reference_speed**2
    aero = {
        "air_density": 1.162,
        "downforce_area": obfuscate(real["downforce_n"] / dynamic_pressure, "downforce_area"),
        "drag_area": obfuscate(real["drag_n"] / dynamic_pressure, "drag_area"),
        "aero_balance": obfuscate(
            real["downforce_front_fraction"] / 100.0, "aero_balance"
        ),
    }
    return vehicle, aero, real, obfuscated_pedal_ratio


def write_vehicle(
    path: Path,
    vehicle: dict[str, float],
    source_name: str,
    source_digest: str,
) -> None:
    """Write the vehicle assembly XML."""

    model = ElementTree.Element(
        "model",
        {
            "schema-version": "1.0",
            "id": VEHICLE_ID,
            "type": "vehicle-assembly",
            "subtype": "four-wheel",
        },
    )
    model.append(ElementTree.Comment(
        " Placeholder vehicle for the planar 7-DOF model."
        " Every sheet-derived value is rounded to two significant figures and"
        " skewed, so this file does not carry a real car's data."
        " Replace every value with a measured value before any correlated"
        " result."
    ))
    model.append(ElementTree.Comment(
        f" Source {source_name} sha256={source_digest}."
        " The source file stays outside the repository."
    ))
    model.append(ElementTree.Comment(
        " Not from the sheet, and kept exact:"
        " yaw_radius_of_gyration and wheel_inertia follow a general estimation"
        " request; steering_maximum_angle, steering_maximum_rate, brake_bias,"
        " hydraulic_ratio_front/rear, caliper_area and maximum_brake_pressure are stated"
        " placeholders. The pedal ratio is calibrated from the sheet line"
        " pressure at one g, and the Ackermann table is a zero array."
    ))

    parameters = ElementTree.SubElement(model, "parameters")
    units = {
        "total_mass": "kg",
        "mass_without_driver": "kg",
        "driver_mass": "kg",
        "wheelbase": "m",
        "track_width": "m",
        "cg_height": "m",
        "front_mass_fraction": "1",
        "lateral_load_transfer_distribution": "1",
        "yaw_radius_of_gyration": "m",
        "wheel_inertia": "kg*m^2",
        "steer_ratio": "1",
        "steering_maximum_angle": "rad",
        "steering_maximum_rate": "rad/s",
        "rack_travel_per_revolution": "m",
        "steer_arm_length": "m",
        "caster_angle": "rad",
        "kingpin_inclination_angle": "rad",
        "brake_bias": "1",
        "maximum_pedal_force": "N",
        "hydraulic_ratio_front": "1",
        "hydraulic_ratio_rear": "1",
        "maximum_brake_pressure": "Pa",
        "caliper_area": "m^2",
        "pad_mu_front": "1",
        "pad_mu_rear": "1",
        "brake_effective_radius_front": "m",
        "brake_effective_radius_rear": "m",
    }
    for name, value in vehicle.items():
        ElementTree.SubElement(
            parameters,
            "parameter",
            {"name": name, "value": repr(float(value)), "unit": units[name]},
        )

    for role, relative in (
        ("ackermann", "../data/placeholder_ackermann.csv"),
        ("pedal-ratio", "../data/placeholder_pedal_ratio.csv"),
    ):
        ElementTree.SubElement(
            model,
            "data",
            {
                "role": role,
                "path": relative,
                "format": "csv",
                "sha256": sha256((path.parent / relative).resolve()),
            },
        )

    components = ElementTree.SubElement(model, "components")
    for role, reference in (
        ("tire", "../tires/hoosier-43075-r20-simple.xml"),
        ("aero", "../aero/placeholder_2024_aero.xml"),
        ("powertrain", "../powertrains/placeholder_2024_powertrain.xml"),
        ("drivetrain", "../drivetrains/murphy_2024_drivetrain.xml"),
    ):
        ElementTree.SubElement(components, "component", {"role": role, "ref": reference})

    _write(model, path)


def write_tables(
    ackermann_path: Path,
    pedal_ratio_path: Path,
    pedal_ratio: float,
    maximum_pedal_force: float,
    inner_angles: int = 5,
) -> None:
    """Write the two placeholder lookup tables.

    The Ackermann table is a zero array, which gives parallel steering. The
    pedal ratio table is constant, because only one operating point is known.
    """

    ackermann_path.parent.mkdir(parents=True, exist_ok=True)
    with ackermann_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["inner_angle_rad", "outer_offset_rad"])
        for index in range(inner_angles):
            angle = (index + 1) / inner_angles
            writer.writerow([f"{angle:.12g}", "0"])

    with pedal_ratio_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["pedal_force_n", "pedal_ratio"])
        for index in range(inner_angles):
            force = maximum_pedal_force * (index + 1) / inner_angles
            writer.writerow([f"{force:.12g}", f"{pedal_ratio:.12g}"])


MOTOR_ID = "placeholder-2024-motor"
BATTERY_ID = "placeholder-2024-battery"
POWERTRAIN_ID = "placeholder-2024-powertrain"


def build_powertrain(
    real: dict[str, float],
    nominal_bus_voltage: float,
    inverter_efficiency: float,
    motor_efficiency: float,
    regeneration_torque_fraction: float,
    cell_capacity: float,
    minimum_cell_voltage: float,
    maximum_cell_voltage: float,
) -> tuple[dict[str, float], dict[str, float]]:
    """Return the motor scalars and the battery scalars."""

    unit_count = 2.0 * real["motor_count"]
    motor = {
        "unit_count": float(round(unit_count)),
        "gear_ratio": obfuscate(real["gear_ratio"], "gear_ratio"),
        "motor_efficiency": motor_efficiency,
        "peak_motor_torque": obfuscate(real["motor_peak_torque"], "peak_motor_torque"),
        "continuous_motor_torque": obfuscate(
            real["motor_continuous_torque"], "continuous_motor_torque"
        ),
        "peak_power": obfuscate(
            real["motor_peak_power_kw"] * 1000.0, "peak_power"
        ),
        "continuous_power": obfuscate(
            real["motor_continuous_power_kw"] * 1000.0, "continuous_power"
        ),
        "maximum_motor_speed": obfuscate(
            real["motor_max_rpm"], "maximum_motor_speed"
        ),
        "nominal_bus_voltage": nominal_bus_voltage,
        "maximum_bus_voltage": nominal_bus_voltage,
        "regeneration_torque_fraction": regeneration_torque_fraction,
        "inverter_efficiency": inverter_efficiency,
    }
    battery = {
        "series_count": real["pack_series_count"],
        "parallel_count": real["pack_parallel_count"],
        "discharge_current_limit": obfuscate(
            real["pack_peak_power_kw"] * 1000.0 / nominal_bus_voltage,
            "discharge_current_limit",
        ),
        "charge_current_limit": obfuscate(
            real["pack_charge_power_kw"] * 1000.0 / nominal_bus_voltage,
            "charge_current_limit",
        ),
        "cell_capacity": obfuscate(cell_capacity, "cell_capacity"),
        "minimum_cell_voltage": obfuscate(
            minimum_cell_voltage, "minimum_cell_voltage"
        ),
        "maximum_cell_voltage": obfuscate(
            maximum_cell_voltage, "maximum_cell_voltage"
        ),
        "initial_bus_voltage": nominal_bus_voltage,
    }
    return motor, battery


def _write_simple(
    path: Path,
    model_id: str,
    model_type: str,
    subtype: str,
    values: dict[str, float],
    units: dict[str, str],
    comments: tuple[str, ...],
) -> None:
    """Write one flat parameter model."""

    model = ElementTree.Element(
        "model",
        {
            "schema-version": "1.0",
            "id": model_id,
            "type": model_type,
            "subtype": subtype,
        },
    )
    for comment in comments:
        model.append(ElementTree.Comment(f" {comment}"))
    parameters = ElementTree.SubElement(model, "parameters")
    for name, value in values.items():
        ElementTree.SubElement(
            parameters,
            "parameter",
            {"name": name, "value": repr(float(value)), "unit": units[name]},
        )
    _write(model, path)


def write_aero(
    path: Path,
    aero: dict[str, float],
    source_name: str,
    source_digest: str,
) -> None:
    """Write the aero component XML."""

    model = ElementTree.Element(
        "model",
        {
            "schema-version": "1.0",
            "id": AERO_ID,
            "type": "aero-model",
            "subtype": "fixed-aero",
        },
    )
    model.append(ElementTree.Comment(
        " Placeholder aero. downforce_area and drag_area are derived from the"
        " measured forces at the stated reference condition, then rounded to"
        " two significant figures and skewed."
    ))
    model.append(ElementTree.Comment(
        f" Source {source_name} sha256={source_digest}."
    ))
    parameters = ElementTree.SubElement(model, "parameters")
    for name, unit in (
        ("air_density", "kg/m^3"),
        ("downforce_area", "m^2"),
        ("drag_area", "m^2"),
        ("aero_balance", "1"),
    ):
        ElementTree.SubElement(
            parameters,
            "parameter",
            {"name": name, "value": repr(float(aero[name])), "unit": unit},
        )
    _write(model, path)


def _write(model: ElementTree.Element, path: Path) -> None:
    ElementTree.indent(model, space="    ")
    path.parent.mkdir(parents=True, exist_ok=True)
    ElementTree.ElementTree(model).write(path, encoding="UTF-8", xml_declaration=True)
    text = path.read_text(encoding="utf-8").replace(
        "<?xml version='1.0' encoding='UTF-8'?>",
        '<?xml version="1.0" encoding="UTF-8"?>',
        1,
    )
    path.write_text(text, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output-vehicle", type=Path, required=True)
    parser.add_argument("--output-aero", type=Path, required=True)
    parser.add_argument("--output-motor", type=Path, required=True)
    parser.add_argument("--output-battery", type=Path, required=True)
    parser.add_argument("--output-powertrain", type=Path, required=True)
    parser.add_argument("--nominal-bus-voltage", type=float, default=600.0)
    parser.add_argument("--inverter-efficiency", type=float, default=0.95)
    parser.add_argument("--motor-efficiency", type=float, default=0.95)
    parser.add_argument("--regeneration-torque-fraction", type=float, default=0.5)
    parser.add_argument("--cell-capacity", type=float, default=14.146)
    parser.add_argument("--minimum-cell-voltage", type=float, default=3.117)
    parser.add_argument("--maximum-cell-voltage", type=float, default=4.227)
    parser.add_argument("--output-ackermann", type=Path, required=True)
    parser.add_argument("--output-pedal-ratio", type=Path, required=True)
    parser.add_argument("--driver-mass", type=float, default=68.0)
    parser.add_argument(
        "--brake-bias", type=float, default=0.3,
        help="signed balance bar: 0 neutral, +1 all front, -1 all rear "
             "(0.3 sends 65 %% of the pushrod force to the front)",
    )
    parser.add_argument("--maximum-steer-deg", type=float, default=30.0)
    parser.add_argument("--maximum-steer-rate-deg-s", type=float, default=180.0)
    parser.add_argument(
        "--rack-travel-per-revolution",
        type=float,
        default=3.5 * 0.0254,
        help="rack travel in metres per steering wheel revolution",
    )
    parser.add_argument(
        "--maximum-pedal-force",
        type=float,
        default=120.0 * 9.80665,
        help="driver pedal force limit in newtons",
    )
    parser.add_argument("--hydraulic-ratio", type=float, default=2.0)
    parser.add_argument("--maximum-brake-pressure", type=float, default=1.0e7)
    parser.add_argument(
        "--caliper-area",
        type=float,
        default=1.14e-3,
        help="effective caliper piston area in square metres",
    )
    parser.add_argument(
        "--rotor-effective-fraction",
        type=float,
        default=0.8,
        help="effective radius as a fraction of the rotor radius",
    )
    parser.add_argument("--pad-mu", type=float, default=0.35)
    parser.add_argument("--wheel-mass", type=float, default=12.0)
    parser.add_argument("--wheel-diameter-in", type=float, default=16.0)
    parser.add_argument("--wheel-width-in", type=float, default=8.0)
    parser.add_argument("--yaw-inertia-reference", type=float, default=120.0)
    parser.add_argument("--yaw-mass-reference", type=float, default=260.0)
    options = parser.parse_args()

    source = options.source.expanduser().resolve()
    raw = read_cells(source)
    vehicle, aero, real, pedal_ratio = build_parameters(
        raw,
        driver_mass=options.driver_mass,
        brake_bias=options.brake_bias,
        maximum_steer_deg=options.maximum_steer_deg,
        maximum_steer_rate_deg_s=options.maximum_steer_rate_deg_s,
        rack_travel_per_revolution=options.rack_travel_per_revolution,
        maximum_pedal_force=options.maximum_pedal_force,
        hydraulic_ratio=options.hydraulic_ratio,
        maximum_brake_pressure=options.maximum_brake_pressure,
        caliper_area=options.caliper_area,
        rotor_effective_fraction=options.rotor_effective_fraction,
        pad_mu=options.pad_mu,
        wheel_mass=options.wheel_mass,
        wheel_diameter_in=options.wheel_diameter_in,
        wheel_width_in=options.wheel_width_in,
        yaw_inertia_reference=options.yaw_inertia_reference,
        yaw_mass_reference=options.yaw_mass_reference,
    )
    motor, battery = build_powertrain(
        real,
        nominal_bus_voltage=options.nominal_bus_voltage,
        inverter_efficiency=options.inverter_efficiency,
        motor_efficiency=options.motor_efficiency,
        regeneration_torque_fraction=options.regeneration_torque_fraction,
        cell_capacity=options.cell_capacity,
        minimum_cell_voltage=options.minimum_cell_voltage,
        maximum_cell_voltage=options.maximum_cell_voltage,
    )
    _write_simple(
        options.output_motor,
        MOTOR_ID,
        "power-plant",
        "torque-speed-envelope",
        motor,
        {
            "unit_count": "1",
            "gear_ratio": "1",
            "motor_efficiency": "1",
            "peak_motor_torque": "N*m",
            "continuous_motor_torque": "N*m",
            "peak_power": "W",
            "continuous_power": "W",
            "maximum_motor_speed": "rpm",
            "nominal_bus_voltage": "V",
            "maximum_bus_voltage": "V",
            "regeneration_torque_fraction": "1",
            "inverter_efficiency": "1",
        },
        (
            "Placeholder smooth motor envelope. Every sheet-derived value is"
            " rounded to two significant figures and skewed.",
            "The envelope is smooth, so a nonlinear program can differentiate"
            " it. The fitted map in the older power plant is not.",
        ),
    )
    _write_simple(
        options.output_battery,
        BATTERY_ID,
        "energy-storage",
        "battery-pack",
        battery,
        {
            "series_count": "1",
            "parallel_count": "1",
            "discharge_current_limit": "A",
            "charge_current_limit": "A",
            "cell_capacity": "A*h",
            "minimum_cell_voltage": "V",
            "maximum_cell_voltage": "V",
            "initial_bus_voltage": "V",
        },
        (
            "Placeholder battery pack. The series and parallel counts and the"
            " initial voltage are exact; the rest is rounded and skewed.",
            "The cell curves come from models/data/cosmx_cell_curves.csv.",
        ),
    )
    powertrain = ElementTree.Element(
        "model",
        {
            "schema-version": "1.0",
            "id": POWERTRAIN_ID,
            "type": "powertrain-assembly",
            "subtype": "electric",
        },
    )
    components = ElementTree.SubElement(powertrain, "components")
    ElementTree.SubElement(
        components,
        "component",
        {
            "role": "power-plant",
            "ref": f"../power-plants/{options.output_motor.name}",
        },
    )
    ElementTree.SubElement(
        components,
        "component",
        {
            "role": "energy-storage",
            "ref": f"../energy-storages/{options.output_battery.name}",
        },
    )
    _write(powertrain, options.output_powertrain)

    write_tables(
        options.output_ackermann,
        options.output_pedal_ratio,
        pedal_ratio,
        options.maximum_pedal_force,
    )
    digest = sha256(source)
    write_vehicle(options.output_vehicle, vehicle, source.name, digest)
    write_aero(options.output_aero, aero, source.name, digest)

    print(f"source sha256 {digest}")
    print("emitted vehicle parameters (obfuscated unless marked):")
    exact = {
        "total_mass",
        "mass_without_driver",
        "driver_mass",
        "yaw_radius_of_gyration",
        "wheel_inertia",
        "steering_maximum_angle",
        "steering_maximum_rate",
        "rack_travel_per_revolution",
        "maximum_pedal_force",
        "hydraulic_ratio_front",
        "hydraulic_ratio_rear",
        "maximum_brake_pressure",
        "caliper_area",
        "pad_mu_front",
        "pad_mu_rear",
        "brake_bias",
    }
    for name, value in vehicle.items():
        mark = "estimate" if name in exact else "obfuscated"
        print(f"  {name:38s} {value!r:22s} {mark}")
    print("emitted aero parameters:")
    for name, value in aero.items():
        mark = "sheet constant" if name == "air_density" else "obfuscated"
        print(f"  {name:38s} {value!r:22s} {mark}")
    print(f"pedal ratio (calibrated, then obfuscated) {pedal_ratio!r}")
    print(f"wrote {options.output_vehicle}")
    print(f"wrote {options.output_aero}")
    print("emitted motor:", {k: motor[k] for k in ("unit_count","gear_ratio","peak_motor_torque","peak_power","maximum_motor_speed")})
    print("emitted battery:", {k: battery[k] for k in ("series_count","parallel_count","discharge_current_limit","cell_capacity")})
    print(f"wrote {options.output_motor}")
    print(f"wrote {options.output_battery}")
    print(f"wrote {options.output_powertrain}")
    print(f"wrote {options.output_ackermann}")
    print(f"wrote {options.output_pedal_ratio}")


if __name__ == "__main__":
    main()
