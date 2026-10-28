"""Build one closed signed XY track from a measured velocity trace.

The measured path does not close exactly, because the integrated velocity
drifts. This tool makes one closed centerline and records every correction.

Method:

1. Select the lap window and integrate ``VNorth`` and ``VEast`` at the logged
   timestamps with the trapezoid rule. The world frame is ``X`` north, ``Y``
   east, and ``Z`` down. That frame is right handed, so a right turn has
   positive curvature with the standard formula.
2. Remove the closure gap with a linear drift correction in arc length. The
   correction is zero at the start and at the end. The tool reports its peak.
3. Smooth the position with a periodic Gaussian kernel. The kernel wraps, so
   the result stays closed.
4. Resample at uniform arc length. Repeat the smoothing and the resample once,
   so the stored distance is the arc length of the final geometry.
5. Report heading and signed curvature from the final geometry.

The tool writes a CSV of geometry and an XML model that references it.

Run it only when the track source changes. It needs SciPy for the MAT file.
The runtime loader needs only NumPy.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ElementTree

import numpy as np
from scipy.io import loadmat

from simplified_vehicle_models.core.conventions import (
    AXIS_DOWN,
    AXIS_FORWARD,
    AXIS_RIGHT,
    RIGHT_TURN_CURVATURE_SIGN,
    WHEEL_ORDER,
)


SOURCE_MAT = Path("/Users/alphabet1671/Desktop/0615CompEndurance.mat")

MODEL_ID = "mis_ev_endurance_2024_linear"
TRACK_TITLE = "2024 Michigan Endurance Closed"
TRACK_SUBTYPE = "closed-xy-profile"

#: The legacy lap window for the fastest recorded lap.
DEFAULT_START_TIME = 628.61
DEFAULT_END_TIME = 698.94

#: Distance between stored nodes in metres.
DEFAULT_NODE_SPACING = 0.5

#: Standard deviation of the periodic smoothing kernel in metres.
DEFAULT_SMOOTHING = 2.0

#: Largest radius that the point-mass limit solver treats as a straight.
DEFAULT_MAXIMUM_RADIUS = 300.0


def _xml_comment_text(text: str) -> str:
    """Escape sequences that XML comments do not permit."""
    return text.replace("--", "- -").rstrip("-") + (" " if text.endswith("-") else "")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def integrate_rates(
    timestamps: np.ndarray,
    north_rate: np.ndarray,
    east_rate: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Integrate two velocity channels into a north and east trace.

    The integration uses the trapezoid rule at the logged timestamps.
    """

    steps = np.diff(timestamps)
    north = np.concatenate(
        ([0.0], np.cumsum(0.5 * (north_rate[1:] + north_rate[:-1]) * steps))
    )
    east = np.concatenate(
        ([0.0], np.cumsum(0.5 * (east_rate[1:] + east_rate[:-1]) * steps))
    )
    return north, east


def integrate_velocity(
    telemetry: Path,
    start_time: float,
    end_time: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Integrate the two velocity channels of a telemetry file.

    Return the timestamps, the north coordinate, and the east coordinate.
    """

    loaded = loadmat(
        telemetry,
        variable_names=["xtime", "VNorth", "VEast"],
        squeeze_me=True,
        struct_as_record=False,
    )
    time = np.asarray(loaded["xtime"].Value, dtype=np.float64)
    north_rate = np.asarray(loaded["VNorth"].Value, dtype=np.float64)
    east_rate = np.asarray(loaded["VEast"].Value, dtype=np.float64)

    if not (time.size == north_rate.size == east_rate.size):
        raise ValueError("the telemetry channels must have the same length")
    window = np.flatnonzero((time >= start_time) & (time <= end_time))
    if window.size < 2:
        raise ValueError("the lap window holds too few samples")

    stamps = time[window]
    if np.any(np.diff(stamps) <= 0.0):
        raise ValueError("the logged timestamps must increase")
    if not (
        np.all(np.isfinite(north_rate[window]))
        and np.all(np.isfinite(east_rate[window]))
    ):
        raise ValueError("the velocity channels must be finite in the window")

    north, east = integrate_rates(
        stamps, north_rate[window], east_rate[window]
    )
    return stamps, north, east


def arc_length(north: np.ndarray, east: np.ndarray) -> np.ndarray:
    """Return the cumulative arc length of a polyline."""

    steps = np.hypot(np.diff(north), np.diff(east))
    return np.concatenate(([0.0], np.cumsum(steps)))


def close_by_linear_correction(
    north: np.ndarray,
    east: np.ndarray,
    distance: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Remove the closure gap with a correction that is linear in distance."""

    length = float(distance[-1])
    if length <= 0.0:
        raise ValueError("the path must have a positive length")
    fraction = distance / length
    gap_north = float(north[-1] - north[0])
    gap_east = float(east[-1] - east[0])
    corrected_north = north - fraction * gap_north
    corrected_east = east - fraction * gap_east
    return corrected_north, corrected_east, math.hypot(gap_north, gap_east)


def periodic_gaussian(values: np.ndarray, sigma_nodes: float) -> np.ndarray:
    """Smooth a periodic signal with a wrapped Gaussian kernel."""

    if sigma_nodes <= 0.0:
        return values.copy()
    half = int(math.ceil(4.0 * sigma_nodes))
    offsets = np.arange(-half, half + 1, dtype=np.float64)
    weights = np.exp(-0.5 * (offsets / sigma_nodes) ** 2)
    weights /= weights.sum()
    wrapped = np.concatenate((values[-half:], values, values[:half]))
    smoothed = np.convolve(wrapped, weights, mode="same")
    return smoothed[half:-half]


def resample(
    distance: np.ndarray,
    north: np.ndarray,
    east: np.ndarray,
    spacing: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Resample a closed trace at uniform arc length without the end point.

    The first node is not repeated, because the track is periodic.
    """

    length = float(distance[-1])
    nodes = int(round(length / spacing))
    if nodes < 8:
        raise ValueError("the node spacing is too large for this path")
    grid = np.arange(nodes, dtype=np.float64) * (length / nodes)
    return (
        grid,
        np.interp(grid, distance, north),
        np.interp(grid, distance, east),
    )


def signed_geometry(
    distance: np.ndarray,
    north: np.ndarray,
    east: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return the heading and the signed curvature of a closed path.

    The path is periodic. The derivative uses a wrapped central difference.
    A right turn has positive curvature in this frame.
    """

    spacing = float(distance[1] - distance[0])
    north_prime = (np.roll(north, -1) - np.roll(north, 1)) / (2.0 * spacing)
    east_prime = (np.roll(east, -1) - np.roll(east, 1)) / (2.0 * spacing)
    north_second = (np.roll(north, -1) - 2.0 * north + np.roll(north, 1)) / spacing**2
    east_second = (np.roll(east, -1) - 2.0 * east + np.roll(east, 1)) / spacing**2

    heading = np.unwrap(np.arctan2(east_prime, north_prime))
    speed = np.hypot(north_prime, east_prime)
    denominator = speed**3
    if np.any(denominator <= 0.0):
        raise ValueError("the path must not stop")
    curvature = (
        north_prime * east_second - east_prime * north_second
    ) / denominator
    return heading, curvature


def minimum_self_distance(
    north: np.ndarray,
    east: np.ndarray,
    neighbours: int = 40,
) -> float:
    """Return the closest approach between distant parts of the path.

    A small value marks a self intersection.
    """

    count = north.size
    points = np.column_stack((north, east))
    closest = math.inf
    for index in range(count):
        gap = np.abs(np.arange(count) - index)
        gap = np.minimum(gap, count - gap)
        distant = gap > neighbours
        if not distant.any():
            continue
        offsets = points[distant] - points[index]
        closest = min(closest, float(np.min(np.hypot(offsets[:, 0], offsets[:, 1]))))
    return closest


def geometry_from_arrays(
    north: np.ndarray,
    east: np.ndarray,
    spacing: float,
    smoothing: float,
    *,
    stamps=None,
) -> dict[str, object]:
    """Build closed-track geometry from arrays without changing CLI defaults."""
    north = np.asarray(north, dtype=float)
    east = np.asarray(east, dtype=float)
    if (
        north.ndim != 1
        or east.shape != north.shape
        or north.size < 8
        or not np.all(np.isfinite(north))
        or not np.all(np.isfinite(east))
    ):
        raise ValueError(
            "coordinate arrays must be finite, one-dimensional, equally sized, "
            "and contain at least eight samples"
        )
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError("node spacing must be finite and positive")
    if not math.isfinite(smoothing) or smoothing < 0:
        raise ValueError("smoothing must be finite and nonnegative")
    estimated_nodes = int(math.ceil(float(arc_length(north, east)[-1]) / spacing))
    if estimated_nodes > 12000:
        raise ValueError(
            "requested spacing would create more than 12000 geometry nodes; "
            "choose a larger spacing"
        )

    raw_distance = arc_length(north, east)
    closed_north, closed_east, gap = close_by_linear_correction(
        north, east, raw_distance
    )
    distance, north_nodes, east_nodes = resample(
        raw_distance, closed_north, closed_east, spacing
    )
    correction = 0.0
    for _ in range(2):
        before_north = north_nodes.copy()
        before_east = east_nodes.copy()
        sigma = min(smoothing / spacing, len(north_nodes) / 16)
        north_nodes = periodic_gaussian(north_nodes, sigma)
        east_nodes = periodic_gaussian(east_nodes, sigma)
        correction = max(
            correction,
            float(
                np.max(
                    np.hypot(
                        north_nodes - before_north,
                        east_nodes - before_east,
                    )
                )
            ),
        )
        closed_north_nodes = np.append(north_nodes, north_nodes[0])
        closed_east_nodes = np.append(east_nodes, east_nodes[0])
        closed_distance = arc_length(closed_north_nodes, closed_east_nodes)
        distance, north_nodes, east_nodes = resample(
            closed_distance, closed_north_nodes, closed_east_nodes, spacing
        )

    node_spacing = float(distance[1] - distance[0])
    length = float(distance[-1] + node_spacing)
    heading, curvature = signed_geometry(distance, north_nodes, east_nodes)
    winding = float(np.sum(curvature) * node_spacing)
    return {
        "stamps": np.asarray(stamps if stamps is not None else []),
        "raw_distance": raw_distance,
        "raw_north": north,
        "raw_east": east,
        "closed_north": closed_north,
        "closed_east": closed_east,
        "closure_gap": gap,
        "distance": distance,
        "north": north_nodes,
        "east": east_nodes,
        "heading": heading,
        "curvature": curvature,
        "length": length,
        "winding": winding,
        "peak_smoothing_correction": correction,
        "minimum_radius": float(1 / np.max(np.abs(curvature))),
        "minimum_self_distance": minimum_self_distance(
            north_nodes, east_nodes
        ),
    }


def build_geometry(
    telemetry: Path,
    start_time: float,
    end_time: float,
    spacing: float,
    smoothing: float,
) -> dict[str, object]:
    """Build the closed geometry and the diagnostics of one lap window."""

    stamps, north, east = integrate_velocity(telemetry, start_time, end_time)
    raw_distance = arc_length(north, east)
    closed_north, closed_east, gap = close_by_linear_correction(
        north, east, raw_distance
    )

    distance, north_nodes, east_nodes = resample(
        raw_distance, closed_north, closed_east, spacing
    )
    correction = 0.0
    for _ in range(2):
        before = (north_nodes.copy(), east_nodes.copy())
        sigma_nodes = smoothing / spacing
        north_nodes = periodic_gaussian(north_nodes, sigma_nodes)
        east_nodes = periodic_gaussian(east_nodes, sigma_nodes)
        correction = max(
            correction, float(np.max(np.hypot(north_nodes - before[0], east_nodes - before[1])))
        )
        distance, north_nodes, east_nodes = resample(
            arc_length(np.append(north_nodes, north_nodes[0]),
                       np.append(east_nodes, east_nodes[0])),
            np.append(north_nodes, north_nodes[0]),
            np.append(east_nodes, east_nodes[0]),
            spacing,
        )

    length = float(distance[-1] + spacing)
    heading, curvature = signed_geometry(distance, north_nodes, east_nodes)
    winding = float(np.sum(curvature) * spacing)

    return {
        "stamps": stamps,
        "raw_distance": raw_distance,
        "raw_north": north,
        "raw_east": east,
        "closed_north": closed_north,
        "closed_east": closed_east,
        "closure_gap": gap,
        "distance": distance,
        "north": north_nodes,
        "east": east_nodes,
        "heading": heading,
        "curvature": curvature,
        "length": length,
        "winding": winding,
        "peak_smoothing_correction": correction,
        "minimum_radius": float(1.0 / np.max(np.abs(curvature))),
        "minimum_self_distance": minimum_self_distance(north_nodes, east_nodes),
    }


def write_csv(path: Path, geometry: dict[str, object]) -> None:
    """Write the geometry table.

    Repeat the first node at the end, so the stored segment count is one less
    than the stored node count. The existing track loader uses that convention.
    """

    distance = np.asarray(geometry["distance"])
    north = np.asarray(geometry["north"])
    east = np.asarray(geometry["east"])
    heading = np.asarray(geometry["heading"])
    curvature = np.asarray(geometry["curvature"])
    spacing = float(distance[1] - distance[0])
    closing = float(distance[-1] + spacing)
    # The heading is unwrapped, so it drops by one full turn across a lap.
    # Extend the last step instead of repeating the wrapped start value.
    closing_heading = float(2.0 * heading[-1] - heading[-2])
    distance = np.append(distance, closing)
    north = np.append(north, north[0])
    east = np.append(east, east[0])
    heading = np.append(heading, closing_heading)
    curvature = np.append(curvature, curvature[0])
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["distance", "x_north", "y_east", "heading_rad", "curvature"])
        for row in zip(distance, north, east, heading, curvature):
            writer.writerow([f"{value:.12g}" for value in row])


def write_xml(
    path: Path,
    csv_name: str,
    geometry: dict[str, object],
    telemetry: Path,
    maximum_radius: float,
    spacing: float,
    smoothing: float,
    *,
    model_id: str = MODEL_ID,
    title: str = TRACK_TITLE,
) -> None:
    """Write the track model that references the geometry table.

    ``model_id`` and ``title`` default to the 2024 Michigan endurance values, so a call
    without them writes the same file as before.
    """

    model = ElementTree.Element(
        "model",
        {
            "schema-version": "1.0",
            "id": model_id,
            "type": "track-model",
            "subtype": TRACK_SUBTYPE,
        },
    )
    provenance = geometry.get("provenance")
    if provenance:
        source_description = (
            f"selected {provenance['mode']} channels "
            f"{provenance['channels']['x']} / {provenance['channels']['y']}"
        )
    else:
        source_description = "measured velocity trace"
    first_comment = (
        " Closed signed centerline rebuilt from a "
        + source_description
        + "."
        f" World frame X north, Y east, Z down, so {AXIS_FORWARD} is north,"
        f" {AXIS_RIGHT} is east, and {AXIS_DOWN} is down."
        " A right turn has positive curvature."
    )
    model.append(ElementTree.Comment(_xml_comment_text(first_comment)))
    if provenance:
        provenance_comment = " Track GUI provenance " + json.dumps(
            provenance, sort_keys=True, default=str
        )
        model.append(ElementTree.Comment(_xml_comment_text(provenance_comment)))
    source_comment = (
        f" Source {telemetry.name} sha256={_sha256(telemetry)}"
        f" rows {np.asarray(geometry['raw_north']).size}"
        f" nodes {np.asarray(geometry['distance']).size}"
        f" spacing {spacing} m smoothing {smoothing} m"
        f" length {float(geometry['length']):.6f} m"
        f" closure gap {float(geometry['closure_gap']):.4f} m"
        f" winding {float(geometry['winding']):+.6f} rad"
        f" minimum radius {float(geometry['minimum_radius']):.4f} m"
        f" minimum self distance {float(geometry['minimum_self_distance']):.3f} m"
    )
    if provenance:
        source_comment += " Rebuilt from the selected channels listed above."
    else:
        source_comment += (
            " The source radius is a magnitude only, so this rebuild is the first"
            " signed geometry for this track."
        )
    model.append(ElementTree.Comment(_xml_comment_text(source_comment)))
    model.append(ElementTree.Comment(
        " The point-mass limit solver consumes radius_magnitude."
        " The planar model consumes signed curvature."
        " Wheel order is " + ", ".join(WHEEL_ORDER) + "."
    ))

    metadata = ElementTree.SubElement(model, "metadata")
    title_element = ElementTree.SubElement(metadata, "title")
    title_element.text = title
    ElementTree.SubElement(metadata, "source", {
        "sha256": _sha256(telemetry),
    }).text = telemetry.name

    ElementTree.SubElement(model, "data", {"ref": csv_name})

    parameters = ElementTree.SubElement(model, "parameters")
    for name, value, unit in (
        ("is_closed", "true", None),
        ("terminal_segment_is_duplicate", "true", None),
        ("node_spacing", f"{spacing:.12g}", "m"),
        ("smoothing_length", f"{smoothing:.12g}", "m"),
        ("maximum_radius", f"{maximum_radius:.12g}", "m"),
    ):
        attributes = {"name": name, "value": value}
        if unit is not None:
            attributes["unit"] = unit
        ElementTree.SubElement(parameters, "parameter", attributes)

    ElementTree.indent(model, space="    ")
    path.parent.mkdir(parents=True, exist_ok=True)
    ElementTree.ElementTree(model).write(path, encoding="UTF-8", xml_declaration=True)
    text = path.read_text(encoding="utf-8").replace(
        "<?xml version='1.0' encoding='UTF-8'?>",
        '<?xml version="1.0" encoding="UTF-8"?>',
        1,
    )
    path.write_text(text, encoding="utf-8")


def report(geometry: dict[str, object]) -> None:
    """Print the release diagnostics of one build."""

    distance = np.asarray(geometry["distance"])
    curvature = np.asarray(geometry["curvature"])
    radius = 1.0 / np.maximum(np.abs(curvature), 1e-12)
    print(f"raw samples          {np.asarray(geometry['raw_north']).size}")
    print(f"raw length           {float(geometry['raw_distance'][-1]):.4f} m")
    print(f"closure gap          {float(geometry['closure_gap']):.4f} m")
    print(f"nodes                {distance.size}")
    print(f"stored length        {float(geometry['length']):.4f} m")
    print(f"winding              {float(geometry['winding']):+.6f} rad"
          f"  (ideal {-2.0*math.pi:+.6f})")
    print(f"winding error        {abs(float(geometry['winding']) + 2.0*math.pi):.6f} rad")
    print(f"minimum radius       {float(geometry['minimum_radius']):.4f} m")
    print(f"radius 1st percentile {np.percentile(radius, 1):.4f} m")
    print(f"peak smoothing move  {float(geometry['peak_smoothing_correction']):.4f} m")
    print(f"minimum self distance {float(geometry['minimum_self_distance']):.4f} m")
    print(f"heading span         {np.ptp(np.asarray(geometry['heading'])):.4f} rad")
    print(f"curvature |max|      {np.max(np.abs(curvature)):.6f} 1/m")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--telemetry", type=Path, default=SOURCE_MAT)
    parser.add_argument("--start-time", type=float, default=DEFAULT_START_TIME)
    parser.add_argument("--end-time", type=float, default=DEFAULT_END_TIME)
    parser.add_argument("--spacing", type=float, default=DEFAULT_NODE_SPACING)
    parser.add_argument("--smoothing", type=float, default=DEFAULT_SMOOTHING)
    parser.add_argument("--maximum-radius", type=float, default=DEFAULT_MAXIMUM_RADIUS)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--output-xml", type=Path, required=True)
    options = parser.parse_args()

    telemetry = options.telemetry.expanduser().resolve()
    geometry = build_geometry(
        telemetry,
        options.start_time,
        options.end_time,
        options.spacing,
        options.smoothing,
    )
    report(geometry)
    write_csv(options.output_csv, geometry)
    write_xml(
        options.output_xml,
        options.output_csv.name,
        geometry,
        telemetry,
        options.maximum_radius,
        options.spacing,
        options.smoothing,
    )
    print(f"wrote {options.output_csv}")
    print(f"wrote {options.output_xml}")


if __name__ == "__main__":
    main()
