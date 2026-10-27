"""Study definitions, sweep expansion, and persistent case indexes."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from itertools import product
import json
import math
import os
from pathlib import Path
import subprocess
from typing import Any, Literal

from . import REPOSITORY_ROOT
from .catalogue import fits, read_entry
from .settings import RunSettings
from .vehicle import VehicleConfiguration

MAXIMUM_CASES = 500
WARNING_CASES = 50


class TooManyCases(ValueError):
    """Raised when expansion exceeds the case cap."""


@dataclass(frozen=True, slots=True)
class Dimension:
    """One swept study dimension."""

    key: str
    values: tuple[str, ...]
    labels: tuple[str, ...] = ()
    sweep: str | None = None
    points: tuple[dict, ...] = ()

    def __post_init__(self) -> None:
        values = tuple(self.values)
        labels = tuple(self.labels) or tuple(Path(value).stem for value in values)
        if len(labels) != len(values):
            raise ValueError("dimension labels and values differ")
        object.__setattr__(self, "values", values)
        object.__setattr__(self, "labels", labels)
        object.__setattr__(self, "points", tuple(self.points))


@dataclass(frozen=True, slots=True)
class CaseSpec:
    """One expanded study case."""

    id: str
    label: str
    model: str
    track: str
    choices: dict[str, str]
    sweep_points: dict[str, dict]


@dataclass(frozen=True, slots=True)
class StudyDefinition:
    """Serializable study input."""

    name: str
    source: str
    track: str
    model: str
    choices: dict[str, str]
    settings: RunSettings
    mode: Literal["one-at-a-time", "full-factorial"]
    dimensions: tuple[Dimension, ...]

    def __post_init__(self) -> None:
        if self.model not in {"planar-7dof", "14dof-simple", "point-mass"}:
            raise ValueError(f"unsupported GUI model: {self.model}")
        seen: set[str] = set()
        for dimension in self.dimensions:
            if dimension.key in seen:
                raise ValueError(f"slot {dimension.key} is already swept")
            seen.add(dimension.key)

    def to_json(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "source": self.source,
            "track": self.track,
            "model": self.model,
            "choices": dict(self.choices),
            "settings": self.settings.to_json(),
            "mode": self.mode,
            "dimensions": [asdict(dimension) for dimension in self.dimensions],
        }

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> "StudyDefinition":
        allowed = {"name", "source", "track", "model", "choices", "settings", "mode", "dimensions"}
        unknown = set(data) - allowed
        if unknown:
            raise ValueError(f"unknown study field(s): {', '.join(sorted(unknown))}")
        missing = allowed - set(data)
        if missing:
            raise ValueError(f"missing study field(s): {', '.join(sorted(missing))}")
        return cls(
            data["name"], data["source"], data["track"], data["model"],
            dict(data.get("choices", {})), RunSettings.from_json(data["settings"]),
            data["mode"], tuple(Dimension(**item) for item in data["dimensions"]),
        )


def _case_label(changes: list[tuple[str, str]]) -> str:
    if not changes:
        return "baseline"
    return ", ".join(f"{key}={value}" for key, value in changes)


def _resolve_value(value: str | None) -> Path:
    """Resolve repository-relative and absolute path values canonically."""
    path = Path(value or "")
    return (path if path.is_absolute() else REPOSITORY_ROOT / path).resolve()


def _relative_or_absolute(path: Path) -> str:
    path = path.resolve()
    try:
        return path.relative_to(REPOSITORY_ROOT).as_posix()
    except ValueError:
        return str(path)


def expand(definition: StudyDefinition) -> tuple[CaseSpec, ...]:
    """Expand a definition into one-at-a-time or factorial cases."""
    dimensions = definition.dimensions
    baseline = dict(definition.choices)
    baseline.update({"model": definition.model, "track": definition.track})

    def differs(dimension: Dimension, index: int) -> bool:
        value = dimension.values[index]
        baseline_value = baseline.get(dimension.key)
        if dimension.key in {"model", "track"}:
            if dimension.key == "track":
                return _resolve_value(value) != _resolve_value(baseline_value)
            return value != baseline_value
        return _resolve_value(value) != _resolve_value(baseline_value)

    if definition.mode == "full-factorial":
        case_count = math.prod(len(dimension.values) for dimension in dimensions)
        if case_count > MAXIMUM_CASES:
            raise TooManyCases(f"study has {case_count} cases; maximum is {MAXIMUM_CASES}")
        combinations = list(product(*(range(len(d.values)) for d in dimensions)))
    else:
        # Keep an explicit baseline case, then vary one dimension at a time.
        case_count = 1 + sum(
            differs(dimension, index)
            for dimension in dimensions
            for index in range(len(dimension.values))
        )
        if case_count > MAXIMUM_CASES:
            raise TooManyCases(f"study has {case_count} cases; maximum is {MAXIMUM_CASES}")
        combinations = [None]
        for position, dimension in enumerate(dimensions):
            for index in range(len(dimension.values)):
                if not differs(dimension, index):
                    continue
                combination = [None] * len(dimensions)
                combination[position] = index
                combinations.append(tuple(combination))

    baseline_configuration = None
    if definition.source:
        baseline_configuration = VehicleConfiguration.open(
            Path(definition.source)
        ).with_model(definition.model)
        for key, value in definition.choices.items():
            baseline_value = baseline_configuration.choices.get(key)
            resolved = Path(value) if Path(value).is_absolute() else REPOSITORY_ROOT / value
            if baseline_value is None:
                raise ValueError(f"unknown slot {key}")
            if resolved.resolve() != baseline_value.resolve():
                baseline_configuration = baseline_configuration.with_choice(key, resolved)
        valid_keys = set(baseline_configuration.choices) | {"model", "track"}
        for dimension in dimensions:
            if dimension.key not in valid_keys:
                raise ValueError(f"unknown slot {dimension.key}")

    cases: list[CaseSpec] = []
    for case_index, combination in enumerate(combinations):
        choices = dict(definition.choices)
        model = definition.model
        track = definition.track
        changes: list[tuple[str, str]] = []
        points: dict[str, dict] = {}
        selected: list[tuple[Dimension, int]] = [] if combination is None else [
            (dimension, index) for dimension, index in zip(dimensions, combination)
            if index is not None
        ]
        for dimension, index in selected:
            value = dimension.values[index]
            if dimension.key == "model":
                model = value
            elif dimension.key == "track":
                track = value
            else:
                choices[dimension.key] = value
            if differs(dimension, index):
                changes.append((dimension.key, dimension.labels[index]))
                if dimension.sweep is not None and dimension.points:
                    points[dimension.key] = dict(dimension.points[index])

        if baseline_configuration is not None:
            configuration = baseline_configuration
            changed = [
                (dimension.key, dimension.values[index])
                for dimension, index in selected
                if dimension.key not in {"model", "track"} and differs(dimension, index)
            ]
            for key, value in sorted(changed, key=lambda item: item[0].count("/")):
                configuration = configuration.with_choice(
                    key, Path(value) if Path(value).is_absolute() else REPOSITORY_ROOT / value
                )
            choices = {
                key: _relative_or_absolute(value) for key, value in configuration.choices.items()
            }
        cases.append(CaseSpec(
            f"case_{case_index:03d}", _case_label(changes), model, track, choices, points,
        ))
    return tuple(cases)


def dimension_from_sweep(folder: Path, configuration: VehicleConfiguration) -> Dimension:
    """Build a dimension from a component-sweep-v1 folder."""
    folder = Path(folder)
    data = json.loads((folder / "sweep.json").read_text())
    if data.get("schema") != "component-sweep-v1":
        raise ValueError("unsupported sweep schema")
    members = data.get("members", [])
    if not members:
        raise ValueError("sweep has no members")
    first = folder / members[0]["file"]
    member_entry = read_entry(first)
    matches = [
        key for key, path in configuration.choices.items()
        if fits(read_entry(path), member_entry)
    ]
    if len(matches) != 1:
        raise ValueError("sweep members fit zero or more than one slot")
    values = tuple(_relative_or_absolute(folder / member["file"]) for member in members)
    sweep = _relative_or_absolute(folder)
    points = tuple({
        "sweep": sweep,
        "index": member["index"],
        "target": data.get("target", ""),
        "operation": data.get("operation", ""),
        "x": member["x"],
        "unit": data.get("unit", ""),
    } for member in members)
    return Dimension(
        matches[0], values, tuple(member["label"] for member in members), sweep, points,
    )


def _atomic(path: Path, data: dict[str, Any]) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, sort_keys=True, allow_nan=False))
    os.replace(temporary, path)


class StudyIndex:
    """Persistent study case index."""

    def __init__(self, directory: Path, payload: dict[str, Any]):
        self.directory = Path(directory)
        self._payload = payload

    @property
    def cases(self) -> tuple[dict[str, Any], ...]:
        return tuple(self._payload["cases"])

    @classmethod
    def create(cls, directory: Path, definition: StudyDefinition,
               cases: tuple[CaseSpec, ...]) -> "StudyIndex":
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        try:
            result = subprocess.run(
                ["git", "rev-parse", "HEAD"], capture_output=True, text=True,
                cwd=REPOSITORY_ROOT,
            )
        except OSError:
            commit = None
        else:
            commit = result.stdout.strip() if result.returncode == 0 else None
        payload = {
            "schema": "sim-study-v1",
            "name": definition.name,
            "created_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "repository_commit": commit,
            "definition": "definition.study.json",
            "cases": [asdict(case) for case in cases],
        }
        _atomic(directory / "definition.study.json", definition.to_json())
        _atomic(directory / "study.json", payload)
        return cls(directory, payload)

    @classmethod
    def load(cls, directory: Path) -> "StudyIndex":
        directory = Path(directory)
        return cls(directory, json.loads((directory / "study.json").read_text()))

    def update(self, case_id: str, **fields: Any) -> None:
        """Atomically update one case."""
        for case in self._payload["cases"]:
            if case["id"] == case_id:
                case.update(fields)
                break
        else:
            raise KeyError(case_id)
        _atomic(self.directory / "study.json", self._payload)
