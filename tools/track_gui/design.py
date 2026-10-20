"""An editable open-course design: a section list and the build settings."""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, replace
from typing import Any

from tools.tracks import build_fsae_dynamic_tracks as fsae
from tools.tracks.open_course import (OpenCourse, Section, arc, arc_by_angle, build_course,
                                      straight, transition)

KINDS = ("straight", "arc", "arc-angle", "clothoid")

#: Largest course the GUI builds. A longer course would make the preview slow.
MAXIMUM_LENGTH = 50_000.0


@dataclass(frozen=True, slots=True)
class SectionSpec:
    """One row of the section table.

    ``straight`` uses ``length``. ``arc`` uses ``length`` and ``curvature`` (right turn positive).
    ``arc-angle`` uses ``radius`` and ``angle_deg`` (right turn positive). ``clothoid`` uses
    ``curvature`` (start), ``end_curvature`` and ``rate`` (largest d(curvature)/ds in 1/m^2).
    """

    kind: str = "straight"
    name: str = ""
    length: float = 10.0
    radius: float = 10.0
    angle_deg: float = 90.0
    curvature: float = 0.0
    end_curvature: float = 0.0
    rate: float = 0.05

    def to_section(self) -> Section:
        name = self.name or self.kind
        if self.kind == "straight":
            return straight(self.length, name)
        if self.kind == "arc":
            return arc(self.length, self.curvature, name)
        if self.kind == "arc-angle":
            return arc_by_angle(self.radius, math.radians(self.angle_deg), name)
        if self.kind == "clothoid":
            return transition(self.curvature, self.end_curvature, self.rate, name)
        raise ValueError(f"unknown section kind {self.kind!r}; use one of {', '.join(KINDS)}")


@dataclass(frozen=True, slots=True)
class CourseDesign:
    """Sections plus the build settings of ``build_course``."""

    sections: tuple[SectionSpec, ...] = ()
    maximum_spacing: float = 0.25
    start_heading: float = 0.0
    start_north: float = 0.0
    start_east: float = 0.0
    allow_curvature_jumps: bool = False
    jump_ramp_length: float = 0.05

    def build(self) -> OpenCourse:
        """Build the course. Raise ``ValueError`` with a message the user can act on."""
        if not self.sections:
            raise ValueError("the course has no sections")
        if not (math.isfinite(self.maximum_spacing) and self.maximum_spacing > 0.0):
            raise ValueError("the node spacing must be positive")
        built = []
        for index, spec in enumerate(self.sections, start=1):
            try:
                built.append(spec.to_section())
            except ValueError as error:
                raise ValueError(f"section {index} ({spec.name or spec.kind}): {error}") from error
        if sum(section.length for section in built) > MAXIMUM_LENGTH:
            raise ValueError(f"the course is longer than {MAXIMUM_LENGTH:g} m")
        return build_course(
            built,
            maximum_spacing=self.maximum_spacing,
            start_heading=self.start_heading,
            start_north=self.start_north,
            start_east=self.start_east,
            allow_curvature_jumps=self.allow_curvature_jumps,
            jump_ramp_length=self.jump_ramp_length,
        )

    def with_sections(self, sections) -> "CourseDesign":
        return replace(self, sections=tuple(sections))

    def to_json(self) -> str:
        payload = {"schema": "track-design-v1", **{
            key: getattr(self, key) for key in (
                "maximum_spacing", "start_heading", "start_north", "start_east",
                "allow_curvature_jumps", "jump_ramp_length")},
            "sections": [{field_name: getattr(spec, field_name)
                          for field_name in SectionSpec.__dataclass_fields__} for spec in self.sections]}
        return json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"

    @classmethod
    def from_json(cls, text: str) -> "CourseDesign":
        try:
            payload: dict[str, Any] = json.loads(text)
        except json.JSONDecodeError as error:
            raise ValueError(f"not a track design file: {error}") from error
        if not isinstance(payload, dict) or payload.get("schema") != "track-design-v1":
            raise ValueError("not a track design file: the schema must be track-design-v1")
        names = SectionSpec.__dataclass_fields__
        sections = []
        for row in payload.get("sections", []):
            unknown = set(row) - set(names)
            if unknown:
                raise ValueError(f"unknown section field {sorted(unknown)}")
            sections.append(SectionSpec(**row))
        settings = {key: payload[key] for key in (
            "maximum_spacing", "start_heading", "start_north", "start_east",
            "allow_curvature_jumps", "jump_ramp_length") if key in payload}
        return cls(sections=tuple(sections), **settings)


def acceleration_design() -> CourseDesign:
    """The FSAE acceleration course, the same sections as ``fsae.acceleration_course``."""
    total = fsae.ACCELERATION_STAGING_OFFSET + fsae.ACCELERATION_LENGTH
    return CourseDesign(
        sections=(SectionSpec("straight", "acceleration course", length=total),),
        maximum_spacing=fsae.ACCELERATION_NODE_SPACING,
    )


def skidpad_design(transition_length: float = 0.0) -> CourseDesign:
    """The FSAE skidpad, the same sections as ``fsae.skidpad_course(transition_length)``."""
    if transition_length < 0.0 or not math.isfinite(transition_length):
        raise ValueError("the transition length must be zero or positive")
    radius = fsae.SKIDPAD_CIRCLE_RADIUS
    curvature = 1.0 / radius
    rows = [SectionSpec("straight", "entry", length=fsae.SKIDPAD_ENTRY_LENGTH)]
    rate = 2.0 * curvature / transition_length if transition_length > 0.0 else 0.0

    def link(start, end, name):
        return SectionSpec("clothoid", name, curvature=start, end_curvature=end, rate=rate)

    if transition_length > 0.0:
        rows.append(link(0.0, curvature, "entry transition"))
    rows += [SectionSpec("arc-angle", "right lap 1 (untimed)", radius=radius, angle_deg=360.0),
             SectionSpec("arc-angle", "right lap 2 (timed)", radius=radius, angle_deg=360.0)]
    if transition_length > 0.0:
        rows.append(link(curvature, -curvature, "right-to-left transition"))
    rows += [SectionSpec("arc-angle", "left lap 1 (untimed)", radius=radius, angle_deg=-360.0),
             SectionSpec("arc-angle", "left lap 2 (timed)", radius=radius, angle_deg=-360.0)]
    if transition_length > 0.0:
        rows.append(link(-curvature, 0.0, "exit transition"))
    rows.append(SectionSpec("straight", "exit", length=fsae.SKIDPAD_EXIT_LENGTH))
    return CourseDesign(
        sections=tuple(rows),
        maximum_spacing=fsae.SKIDPAD_NODE_SPACING,
        allow_curvature_jumps=True,
        jump_ramp_length=fsae.SKIDPAD_JUMP_RAMP_LENGTH,
    )


PRESETS = {
    "Acceleration (75 m)": acceleration_design,
    "Skidpad (rule geometry)": lambda: skidpad_design(0.0),
    "Skidpad (6 m clothoids)": lambda: skidpad_design(6.0),
}
