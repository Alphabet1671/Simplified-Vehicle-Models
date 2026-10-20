"""Build a signed open course from sections of linearly varying curvature.

An open course has a start and an end, such as a straight acceleration course
or a skidpad. This module writes it as an ``open-xy-profile`` track model.

Every section has a length and a start and end curvature. The curvature varies
linearly with arc length inside a section, so one primitive covers three
shapes:

- a straight (both curvatures zero),
- a constant-radius arc (both curvatures equal), and
- a clothoid transition (the curvatures differ).

The world frame is ``X`` north, ``Y`` east, and ``Z`` down. The heading is
measured from north toward east. A right turn has positive curvature.

Sections join end to end. The builder requires the curvature to be continuous
at each join, so every course it writes is continuous in position, heading, and
curvature. A jump in curvature asks for an infinite steering rate, so insert a
clothoid transition instead. Use ``transition`` to compute its length from a
steering-rate budget. A course that needs a jump can pass
``allow_curvature_jumps=True``, and the XML then records the jump.

The tool needs only NumPy and the standard library.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np


#: Largest curvature step at a join that counts as continuous, in 1/m.
CURVATURE_CONTINUITY_TOLERANCE = 1.0e-9

#: Curvature magnitude stored as a straight when it is converted to a radius.
MAXIMUM_RADIUS = 1.0e5

CSV_HEADER = "distance,x_north,y_east,heading_rad,curvature"


@dataclass(frozen=True, slots=True)
class Section:
    """One section with a curvature that varies linearly with arc length."""

    length: float
    start_curvature: float
    end_curvature: float
    name: str = ""

    def __post_init__(self) -> None:
        values = (self.length, self.start_curvature, self.end_curvature)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("a section needs finite values")
        if self.length <= 0.0:
            raise ValueError("a section length must be positive")


def straight(length: float, name: str = "straight") -> Section:
    """Return a straight section."""

    return Section(length, 0.0, 0.0, name)


def arc(length: float, curvature: float, name: str = "arc") -> Section:
    """Return a constant-curvature section. A right turn is positive."""

    return Section(length, curvature, curvature, name)


def arc_by_angle(radius: float, angle: float, name: str = "arc") -> Section:
    """Return a constant-radius arc that turns by ``angle`` radians.

    A positive angle is a right turn. The radius is a magnitude.
    """

    if radius <= 0.0 or not math.isfinite(radius):
        raise ValueError("the radius must be positive and finite")
    if angle == 0.0 or not math.isfinite(angle):
        raise ValueError("the angle must be finite and nonzero")
    return Section(radius * abs(angle), math.copysign(1.0 / radius, angle), math.copysign(1.0 / radius, angle), name)


def transition(
    start_curvature: float,
    end_curvature: float,
    maximum_curvature_rate: float,
    name: str = "transition",
) -> Section:
    """Return a clothoid that changes curvature at a bounded rate.

    ``maximum_curvature_rate`` is the largest ``d(curvature)/d(arc length)`` in
    1/m^2. At speed ``v`` the road-wheel steering rate is about the wheelbase
    times ``v^2`` times this rate, so choose it for the speed of the course.
    """

    if maximum_curvature_rate <= 0.0 or not math.isfinite(maximum_curvature_rate):
        raise ValueError("the curvature rate must be positive and finite")
    change = abs(end_curvature - start_curvature)
    if change == 0.0:
        raise ValueError("a transition needs different end curvatures")
    return Section(change / maximum_curvature_rate, start_curvature, end_curvature, name)


@dataclass(frozen=True, slots=True)
class OpenCourse:
    """Store one sampled open course."""

    distance: np.ndarray
    x_north: np.ndarray
    y_east: np.ndarray
    heading: np.ndarray
    curvature: np.ndarray
    #: Cumulative section boundary distances, starting at 0 and ending at the length.
    boundaries: tuple[float, ...]
    names: tuple[str, ...]
    curvature_jumps: tuple[tuple[float, float], ...]

    @property
    def length(self) -> float:
        return float(self.distance[-1])


def _section_heading(section: Section, s: np.ndarray) -> np.ndarray:
    """Return the heading change at arc length ``s`` into a section."""

    slope = (section.end_curvature - section.start_curvature) / section.length
    return section.start_curvature * s + 0.5 * slope * s * s


def _section_curvature(section: Section, s: np.ndarray) -> np.ndarray:
    slope = (section.end_curvature - section.start_curvature) / section.length
    return section.start_curvature + slope * s


# Three-point Gauss-Legendre rule on [0, 1].
_GAUSS_NODES = 0.5 * (1.0 + np.array([-math.sqrt(3.0 / 5.0), 0.0, math.sqrt(3.0 / 5.0)]))
_GAUSS_WEIGHTS = 0.5 * np.array([5.0 / 9.0, 8.0 / 9.0, 5.0 / 9.0])


def build_course(
    sections: list[Section] | tuple[Section, ...],
    maximum_spacing: float = 0.25,
    start_heading: float = 0.0,
    start_north: float = 0.0,
    start_east: float = 0.0,
    allow_curvature_jumps: bool = False,
    jump_ramp_length: float = 0.05,
) -> OpenCourse:
    """Sample a course from its sections.

    Each section is split into equal steps no longer than ``maximum_spacing``,
    so every section boundary is a node. The position is integrated with a
    three-point Gauss rule per step on the analytic heading, which is exact for
    a straight and accurate far below the step length for an arc or a clothoid.

    With ``allow_curvature_jumps`` a jump in curvature between two sections
    becomes a ramp of ``jump_ramp_length`` metres. The returned
    ``curvature_jumps`` lists each jump as (distance in the unramped course, step).
    The ramp adds ``jump_ramp_length`` to the course length for each jump.
    """

    if not sections:
        raise ValueError("a course needs at least one section")
    if maximum_spacing <= 0.0 or not math.isfinite(maximum_spacing):
        raise ValueError("the maximum spacing must be positive and finite")
    if jump_ramp_length <= 0.0 or not math.isfinite(jump_ramp_length):
        raise ValueError("the jump ramp length must be positive and finite")

    # A table node holds one curvature and the path between nodes has linearly
    # interpolated curvature. A true jump cannot be stored, so an allowed jump
    # becomes a ramp over ``jump_ramp_length``. The ramp is a real, very short
    # section and the XML records it.
    jumps: list[tuple[float, float]] = []
    expanded: list[Section] = [sections[0]]
    boundary = sections[0].length
    for previous, following in zip(sections[:-1], sections[1:]):
        step = following.start_curvature - previous.end_curvature
        if abs(step) > CURVATURE_CONTINUITY_TOLERANCE:
            if not allow_curvature_jumps:
                raise ValueError(
                    f"curvature jumps by {step:.6g} 1/m at {boundary:.4f} m between "
                    f"{previous.name!r} and {following.name!r}. Insert a transition "
                    f"section, or pass allow_curvature_jumps=True"
                )
            jumps.append((boundary, step))
            expanded.append(
                Section(
                    jump_ramp_length,
                    previous.end_curvature,
                    following.start_curvature,
                    f"jump ramp {previous.name}->{following.name}",
                )
            )
            boundary += jump_ramp_length
        expanded.append(following)
        boundary += following.length
    sections = expanded

    distance = [0.0]
    north = [start_north]
    east = [start_east]
    heading = [start_heading]
    curvature = [sections[0].start_curvature]
    boundaries = [0.0]

    offset = 0.0
    for section in sections:
        steps = max(1, int(math.ceil(section.length / maximum_spacing - 1.0e-9)))
        edges = section.length * np.arange(steps + 1) / steps
        base_heading = heading[-1]
        for index in range(steps):
            s0, s1 = float(edges[index]), float(edges[index + 1])
            width = s1 - s0
            nodes = s0 + width * _GAUSS_NODES
            theta = base_heading + _section_heading(section, nodes)
            north.append(north[-1] + width * float(np.sum(_GAUSS_WEIGHTS * np.cos(theta))))
            east.append(east[-1] + width * float(np.sum(_GAUSS_WEIGHTS * np.sin(theta))))
            heading.append(base_heading + float(_section_heading(section, np.array(s1))))
            curvature.append(float(_section_curvature(section, np.array(s1))))
            distance.append(offset + s1)
        offset += section.length
        boundaries.append(offset)

    return OpenCourse(
        distance=np.array(distance),
        x_north=np.array(north),
        y_east=np.array(east),
        heading=np.array(heading),
        curvature=np.array(curvature),
        boundaries=tuple(boundaries),
        names=tuple(section.name for section in sections),
        curvature_jumps=tuple(jumps),
    )


def csv_text(course: OpenCourse) -> str:
    """Return the geometry CSV of one course."""

    lines = [CSV_HEADER]
    for row in zip(
        course.distance, course.x_north, course.y_east, course.heading, course.curvature
    ):
        lines.append(",".join(repr(float(value)) for value in row))
    return "\n".join(lines) + "\n"


def xml_text(
    course: OpenCourse,
    model_id: str,
    title: str,
    csv_name: str,
    comments: list[str] = (),
    parameters: list[tuple[str, object, str | None]] = (),
    source: tuple[str, str] | None = None,
) -> str:
    """Return the ``open-xy-profile`` XML of one course.

    ``source`` is an optional ``(sha256, description)`` pair for the metadata.
    """

    def parameter(name: str, value: object, unit: str | None = None) -> str:
        text = value if isinstance(value, str) else repr(float(value))
        suffix = f' unit="{unit}"' if unit else ""
        return f'        <parameter name="{name}" value="{text}"{suffix} />'

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<model schema-version="1.0" id="{model_id}" type="track-model" '
        'subtype="open-xy-profile">',
        "    <!-- Open signed centerline with a start and an end. World frame X north, Y east, "
        "Z down. A right turn has positive curvature. Built by tools/tracks/open_course.py "
        "from sections of linearly varying curvature. -->",
    ]
    lines += [f"    <!-- {comment} -->" for comment in comments]
    if course.curvature_jumps:
        jumps = "; ".join(f"{step:+.6g} 1/m at {at:.4f} m" for at, step in course.curvature_jumps)
        lines.append(f"    <!-- Curvature jumps, not smoothed: {jumps}. -->")
    lines += ["    <metadata>", f"        <title>{title}</title>"]
    if source is not None:
        lines.append(f'        <source sha256="{source[0]}">{source[1]}</source>')
    lines += [
        "    </metadata>",
        f'    <data ref="{csv_name}" />',
        "    <parameters>",
        parameter("is_closed", "false"),
        parameter("terminal_segment_is_duplicate", "false"),
        parameter("maximum_radius", MAXIMUM_RADIUS, "m"),
        parameter("course_length", course.length, "m"),
    ]
    lines += [parameter(name, value, unit) for name, value, unit in parameters]
    lines += ["    </parameters>", "</model>"]
    return "\n".join(lines) + "\n"


def write_course(
    course: OpenCourse,
    directory: Path,
    stem: str,
    model_id: str,
    title: str,
    comments: list[str] = (),
    parameters: list[tuple[str, object, str | None]] = (),
    source: tuple[str, str] | None = None,
) -> tuple[Path, Path]:
    """Write ``<stem>.csv`` and ``<stem>.xml`` and return their paths."""

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    csv_path = directory / f"{stem}.csv"
    xml_path = directory / f"{stem}.xml"
    csv_path.write_text(csv_text(course), encoding="utf-8")
    xml_path.write_text(
        xml_text(course, model_id, title, csv_path.name, comments, parameters, source),
        encoding="utf-8",
    )
    return xml_path, csv_path
