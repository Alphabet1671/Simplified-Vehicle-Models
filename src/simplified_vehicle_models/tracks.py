from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ElementTree

import numpy as np
import numpy.typing as npt


FloatArray = npt.NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class TrackProfile:
    """Store a distance and radius track profile."""

    distance: FloatArray
    segment_length: FloatArray
    radius: FloatArray
    curvature: FloatArray
    is_closed: bool
    title: str

    @property
    def lap_length(self) -> float:
        """Return the represented lap length."""

        return float(self.distance[-1] - self.distance[0])


def load_track(path: str | Path) -> TrackProfile:
    """Load a distance and radius track profile from XML."""

    source_path = Path(path).expanduser().resolve()
    try:
        root = ElementTree.parse(source_path).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise ValueError(f"cannot read track XML {source_path}: {error}") from error

    expected_identity = ("track-model", "distance-radius-profile")
    identity = (root.get("type"), root.get("subtype"))
    if identity != expected_identity:
        raise ValueError(f"{source_path}: unsupported track model {identity}")
    if root.get("schema-version") != "1.0":
        raise ValueError(f"{source_path}: unsupported schema version")

    title_element = root.find("./metadata/title")
    if title_element is None or not (title_element.text or "").strip():
        raise ValueError(f"{source_path}: missing track title")
    data_element = root.find("./data")
    if data_element is None or not data_element.get("ref"):
        raise ValueError(f"{source_path}: missing track data reference")

    parameters = _read_parameters(root, source_path)
    is_closed = _parse_boolean(parameters.get("is_closed"), "is_closed", source_path)
    terminal_is_duplicate = _parse_boolean(
        parameters.get("terminal_segment_is_duplicate"),
        "terminal_segment_is_duplicate",
        source_path,
    )
    csv_path = (source_path.parent / data_element.get("ref", "")).resolve()
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
        title=(title_element.text or "").strip(),
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
