from __future__ import annotations

import csv
import math
from pathlib import Path
import xml.etree.ElementTree as ElementTree

import numpy as np

from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.tracks.types import PathCoordinates, TrackProfile


DISTANCE_RADIUS_SUBTYPE = "distance-radius-profile"
CLOSED_XY_SUBTYPE = "closed-xy-profile"
OPEN_XY_SUBTYPE = "open-xy-profile"

#: An open course must be self consistent, because it can come from any source.
#: Across one segment, the heading change must match the mean curvature times
#: the segment length to within this angle in radians.
_OPEN_HEADING_TOLERANCE = 2.0e-3
#: The chord of one segment must point along the mean heading to within this
#: angle in radians.
_OPEN_CHORD_ANGLE_TOLERANCE = 2.0e-3
#: The chord length of one segment must match the segment length to within this
#: fraction.
_OPEN_CHORD_LENGTH_TOLERANCE = 1.0e-3

#: Curvature below this value counts as a straight for a radius magnitude.
_STRAIGHT_CURVATURE = 1.0e-12


def load_track(path: str | Path) -> TrackProfile:
    """Load one track profile from XML.

    A ``distance-radius-profile`` holds a radius magnitude and no turn
    direction. A ``closed-xy-profile`` holds a signed closed centerline. An
    ``open-xy-profile`` holds a signed centerline with a start and an end, such
    as a straight acceleration course. It has no repeated first node.
    """

    source_path = Path(path).expanduser().resolve()
    try:
        root = ElementTree.parse(source_path).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise ValueError(f"cannot read track XML {source_path}: {error}") from error

    if root.get("type") != "track-model":
        raise ValueError(
            f"{source_path}: unsupported track model "
            f"{(root.get('type'), root.get('subtype'))}"
        )
    if root.get("schema-version") != "1.0":
        raise ValueError(f"{source_path}: unsupported schema version")

    subtype = root.get("subtype")
    if subtype == DISTANCE_RADIUS_SUBTYPE:
        return _load_distance_radius(source_path, root)
    if subtype == CLOSED_XY_SUBTYPE:
        return _load_closed_xy(source_path, root)
    if subtype == OPEN_XY_SUBTYPE:
        return _load_open_xy(source_path, root)
    raise ValueError(
        f"{source_path}: unsupported track model "
        f"{(root.get('type'), root.get('subtype'))}"
    )


def _title(root: ElementTree.Element, source_path: Path) -> str:
    title_element = root.find("./metadata/title")
    if title_element is None or not (title_element.text or "").strip():
        raise ValueError(f"{source_path}: missing track title")
    return (title_element.text or "").strip()


def _data_path(root: ElementTree.Element, source_path: Path) -> Path:
    data_element = root.find("./data")
    if data_element is None or not data_element.get("ref"):
        raise ValueError(f"{source_path}: missing track data reference")
    return (source_path.parent / data_element.get("ref", "")).resolve()


def _load_distance_radius(
    source_path: Path,
    root: ElementTree.Element,
) -> TrackProfile:
    """Load a legacy distance and radius profile.

    The radius is a magnitude. The stored curvature is not signed.
    """

    parameters = _read_parameters(root, source_path)
    is_closed = _parse_boolean(parameters.get("is_closed"), "is_closed", source_path)
    terminal_is_duplicate = _parse_boolean(
        parameters.get("terminal_segment_is_duplicate"),
        "terminal_segment_is_duplicate",
        source_path,
    )
    csv_path = _data_path(root, source_path)
    distance, stored_segment_length, radius = _read_csv(csv_path)
    _validate_source_arrays(distance, stored_segment_length, radius, csv_path)

    physical_segment_length = np.diff(distance)
    if not terminal_is_duplicate:
        if len(stored_segment_length) != len(physical_segment_length):
            raise ValueError(f"{csv_path}: segment count does not match node count")
        physical_segment_length = stored_segment_length
    elif not np.allclose(
        stored_segment_length[:-1],
        physical_segment_length,
        rtol=0.0,
        atol=1.0e-9,
    ):
        raise ValueError(f"{csv_path}: stored segment lengths do not match distance")

    curvature = np.reciprocal(radius)
    arrays = (distance, physical_segment_length, radius, curvature)
    for array in arrays:
        array.setflags(write=False)

    return TrackProfile(
        distance=distance,
        segment_length=physical_segment_length,
        radius=radius,
        curvature=curvature,
        is_closed=is_closed,
        title=_title(root, source_path),
    )


def _load_closed_xy(
    source_path: Path,
    root: ElementTree.Element,
) -> TrackProfile:
    """Load a signed closed centerline.

    The point-mass limit solver consumes the radius magnitude.
    The planar model consumes the signed curvature and the path coordinates.
    """

    parameters = _read_parameters(root, source_path)
    is_closed = _parse_boolean(parameters.get("is_closed"), "is_closed", source_path)
    if not is_closed:
        raise ValueError(f"{source_path}: a closed-xy-profile must be closed")
    terminal_is_duplicate = _parse_boolean(
        parameters.get("terminal_segment_is_duplicate"),
        "terminal_segment_is_duplicate",
        source_path,
    )
    if not terminal_is_duplicate:
        raise ValueError(
            f"{source_path}: a closed-xy-profile must repeat the first node"
        )
    maximum_radius = _parse_positive_float(
        parameters.get("maximum_radius"), "maximum_radius", source_path
    )

    csv_path = _data_path(root, source_path)
    distance, x_north, y_east, heading, curvature = _read_geometry_csv(csv_path)
    _validate_geometry_arrays(
        distance, x_north, y_east, heading, curvature, csv_path
    )

    physical_segment_length = np.diff(distance)
    # The point-mass limit solver needs a finite nonnegative radius magnitude.
    # Cap the magnitude so a numerical straight does not give an absurd speed.
    magnitude = np.minimum(
        np.reciprocal(np.maximum(np.abs(curvature), _STRAIGHT_CURVATURE)),
        maximum_radius,
    )
    coordinates = PathCoordinates(
        distance=distance,
        x_north=x_north,
        y_east=y_east,
        heading=heading,
        curvature=curvature,
    )
    arrays = (
        distance,
        physical_segment_length,
        magnitude,
        curvature,
        x_north,
        y_east,
        heading,
    )
    for array in arrays:
        array.setflags(write=False)

    return TrackProfile(
        distance=distance,
        segment_length=physical_segment_length,
        radius=magnitude,
        curvature=curvature,
        is_closed=is_closed,
        title=_title(root, source_path),
        path_coordinates=coordinates,
    )


def _load_open_xy(
    source_path: Path,
    root: ElementTree.Element,
) -> TrackProfile:
    """Load a signed open centerline.

    The course runs from the first node to the last node. The last node does
    not repeat the first node. The point-mass limit solver consumes the radius
    magnitude. The planar model consumes the signed curvature and the path
    coordinates.
    """

    parameters = _read_parameters(root, source_path)
    # An open track needs no closure flags. Reject a contradicting flag, so a
    # closed track cannot be mislabelled as open.
    if "is_closed" in parameters and _parse_boolean(
        parameters["is_closed"], "is_closed", source_path
    ):
        raise ValueError(f"{source_path}: an open-xy-profile must not be closed")
    if "terminal_segment_is_duplicate" in parameters and _parse_boolean(
        parameters["terminal_segment_is_duplicate"],
        "terminal_segment_is_duplicate",
        source_path,
    ):
        raise ValueError(
            f"{source_path}: an open-xy-profile must not repeat the first node"
        )
    maximum_radius = _parse_positive_float(
        parameters.get("maximum_radius"), "maximum_radius", source_path
    )

    csv_path = _data_path(root, source_path)
    distance, x_north, y_east, heading, curvature = _read_geometry_csv(csv_path)
    _validate_open_geometry_arrays(
        distance, x_north, y_east, heading, curvature, csv_path
    )

    physical_segment_length = np.diff(distance)
    magnitude = np.minimum(
        np.reciprocal(np.maximum(np.abs(curvature), _STRAIGHT_CURVATURE)),
        maximum_radius,
    )
    coordinates = PathCoordinates(
        distance=distance,
        x_north=x_north,
        y_east=y_east,
        heading=heading,
        curvature=curvature,
        is_closed=False,
    )
    arrays = (
        distance,
        physical_segment_length,
        magnitude,
        curvature,
        x_north,
        y_east,
        heading,
    )
    for array in arrays:
        array.setflags(write=False)

    return TrackProfile(
        distance=distance,
        segment_length=physical_segment_length,
        radius=magnitude,
        curvature=curvature,
        is_closed=False,
        title=_title(root, source_path),
        path_coordinates=coordinates,
    )


def _validate_open_geometry_arrays(
    distance: FloatArray,
    x_north: FloatArray,
    y_east: FloatArray,
    heading: FloatArray,
    curvature: FloatArray,
    path: Path,
) -> None:
    """Validate one signed open centerline table.

    Position, heading, and curvature must describe the same path. The check
    runs per segment, so one bad row is reported by its row number.
    """

    if not all(
        np.all(np.isfinite(array))
        for array in (distance, x_north, y_east, heading, curvature)
    ):
        raise ValueError(f"{path}: track geometry must be finite")
    if not math.isclose(float(distance[0]), 0.0, abs_tol=1.0e-12):
        raise ValueError(f"{path}: distance must start at zero")
    if np.any(np.diff(distance) <= 0.0):
        raise ValueError(f"{path}: distance must increase")

    segment = np.diff(distance)
    heading_change = np.diff(heading)
    expected_change = 0.5 * (curvature[:-1] + curvature[1:]) * segment
    mismatch = np.abs(heading_change - expected_change)
    bad = np.flatnonzero(mismatch > _OPEN_HEADING_TOLERANCE)
    if bad.size:
        # Row numbers are file lines: line 1 is the header, and a segment is
        # reported at the line of the node that ends it.
        row = int(bad[0]) + 3
        raise ValueError(
            f"{path}: row {row} heading change does not match the curvature "
            f"(off by {mismatch[bad[0]]:.3g} rad). The heading must be the "
            f"integral of the curvature"
        )

    delta_north = np.diff(x_north)
    delta_east = np.diff(y_east)
    chord = np.hypot(delta_north, delta_east)
    # A segment is a short arc. Its chord is shorter than the arc length by a
    # relative amount of about (heading change)^2 / 24, which the tolerance covers
    # for any sensible node spacing.
    length_error = np.abs(chord - segment) / segment
    bad = np.flatnonzero(length_error > _OPEN_CHORD_LENGTH_TOLERANCE)
    if bad.size:
        # Row numbers are file lines: line 1 is the header, and a segment is
        # reported at the line of the node that ends it.
        row = int(bad[0]) + 3
        raise ValueError(
            f"{path}: row {row} position step does not match the distance step "
            f"(off by {100.0 * length_error[bad[0]]:.3g} percent)"
        )
    mean_heading = 0.5 * (heading[:-1] + heading[1:])
    chord_heading = np.arctan2(delta_east, delta_north)
    angle_error = np.abs(
        (chord_heading - mean_heading + np.pi) % (2.0 * np.pi) - np.pi
    )
    bad = np.flatnonzero(angle_error > _OPEN_CHORD_ANGLE_TOLERANCE)
    if bad.size:
        # Row numbers are file lines: line 1 is the header, and a segment is
        # reported at the line of the node that ends it.
        row = int(bad[0]) + 3
        raise ValueError(
            f"{path}: row {row} position step does not follow the heading "
            f"(off by {angle_error[bad[0]]:.3g} rad)"
        )


def _read_parameters(root: ElementTree.Element, path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for element in root.findall("./parameters/parameter"):
        name = element.get("name")
        value = element.get("value")
        if not name or value is None:
            raise ValueError(f"{path}: each track parameter needs a name and value")
        if name in values:
            raise ValueError(f"{path}: duplicate track parameter {name!r}")
        values[name] = value
    return values


def _parse_positive_float(value: str | None, name: str, path: Path) -> float:
    """Parse one positive finite numeric track parameter."""

    if value is None:
        raise ValueError(f"{path}: missing track parameter {name!r}")
    try:
        number = float(value)
    except ValueError as error:
        raise ValueError(
            f"{path}: track parameter {name!r} must be numeric"
        ) from error
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(
            f"{path}: track parameter {name!r} must be positive and finite"
        )
    return number


def _parse_boolean(value: str | None, name: str, path: Path) -> bool:
    if value == "true":
        return True
    if value == "false":
        return False
    raise ValueError(f"{path}: parameter {name!r} must be true or false")


def _read_csv(path: Path) -> tuple[FloatArray, FloatArray, FloatArray]:
    rows: list[tuple[float, float, float]] = []
    try:
        with path.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            if reader.fieldnames != ["distance", "segment_length", "radius"]:
                raise ValueError(f"{path}: track columns are invalid")
            for row_number, row in enumerate(reader, start=2):
                try:
                    values = (
                        float(row["distance"]),
                        float(row["segment_length"]),
                        float(row["radius"]),
                    )
                except (KeyError, TypeError, ValueError) as error:
                    raise ValueError(
                        f"{path}: row {row_number} contains invalid numeric data"
                    ) from error
                rows.append(values)
    except OSError as error:
        raise ValueError(f"cannot read track CSV {path}: {error}") from error
    if len(rows) < 2:
        raise ValueError(f"{path}: a track needs at least two rows")
    values = np.asarray(rows, dtype=np.float64)
    return values[:, 0], values[:, 1], values[:, 2]


def _validate_source_arrays(
    distance: FloatArray,
    segment_length: FloatArray,
    radius: FloatArray,
    path: Path,
) -> None:
    if not (
        np.all(np.isfinite(distance))
        and np.all(np.isfinite(segment_length))
        and np.all(np.isfinite(radius))
    ):
        raise ValueError(f"{path}: track data must be finite")
    if np.any(np.diff(distance) <= 0.0):
        raise ValueError(f"{path}: distance must increase")
    if np.any(segment_length <= 0.0):
        raise ValueError(f"{path}: segment lengths must be positive")
    if np.any(radius <= 0.0):
        raise ValueError(f"{path}: radii must be positive")
    if not math.isclose(float(distance[0]), 0.0, abs_tol=1.0e-12):
        raise ValueError(f"{path}: distance must start at zero")


def _read_geometry_csv(
    path: Path,
) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray, FloatArray]:
    """Read a signed centerline table.

    The columns are distance, north position, east position, heading, and
    signed curvature.
    """

    expected = ["distance", "x_north", "y_east", "heading_rad", "curvature"]
    rows: list[tuple[float, ...]] = []
    try:
        with path.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            if reader.fieldnames != expected:
                raise ValueError(
                    f"{path}: geometry columns must be {', '.join(expected)}"
                )
            for row_number, row in enumerate(reader, start=2):
                try:
                    rows.append(tuple(float(row[name]) for name in expected))
                except (KeyError, TypeError, ValueError) as error:
                    raise ValueError(
                        f"{path}: row {row_number} contains invalid numeric data"
                    ) from error
    except OSError as error:
        raise ValueError(f"cannot read track geometry CSV {path}: {error}") from error
    if len(rows) < 3:
        raise ValueError(f"{path}: a closed centerline needs at least three rows")
    values = np.asarray(rows, dtype=np.float64)
    return (
        values[:, 0],
        values[:, 1],
        values[:, 2],
        values[:, 3],
        values[:, 4],
    )


def _validate_geometry_arrays(
    distance: FloatArray,
    x_north: FloatArray,
    y_east: FloatArray,
    heading: FloatArray,
    curvature: FloatArray,
    path: Path,
) -> None:
    """Validate one signed centerline table."""

    if not all(
        np.all(np.isfinite(array))
        for array in (distance, x_north, y_east, heading, curvature)
    ):
        raise ValueError(f"{path}: track geometry must be finite")
    if not math.isclose(float(distance[0]), 0.0, abs_tol=1.0e-12):
        raise ValueError(f"{path}: distance must start at zero")
    if np.any(np.diff(distance) <= 0.0):
        raise ValueError(f"{path}: distance must increase")
    if not np.isclose(
        float(x_north[-1]), float(x_north[0]), rtol=0.0, atol=1.0e-9
    ) or not np.isclose(
        float(y_east[-1]), float(y_east[0]), rtol=0.0, atol=1.0e-9
    ):
        raise ValueError(f"{path}: the closing node must repeat the first position")
    if not np.isclose(
        float(curvature[-1]), float(curvature[0]), rtol=0.0, atol=1.0e-12
    ):
        raise ValueError(f"{path}: the closing node must repeat the first curvature")
