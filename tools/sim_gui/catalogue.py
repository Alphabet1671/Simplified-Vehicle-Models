"""Read and match registered model files."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree

from simplified_vehicle_models.compiler.registry import component_descriptor

from . import REPOSITORY_ROOT


@dataclass(frozen=True, slots=True)
class ModelEntry:
    """Describe one registered model file."""

    path: Path
    model_id: str
    model_type: str
    subtype: str
    capability: str | None
    kind: str


def read_entry(path: Path) -> ModelEntry:
    """Read root attributes and match the registry descriptor."""
    path = Path(path).resolve()
    try:
        root = ElementTree.parse(path).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise ValueError(f"cannot read {path}: {error}") from error
    if root.tag != "model":
        raise ValueError(f"{path}: the root element must be model")
    model_type = root.get("type", "")
    subtype = root.get("subtype", "")
    try:
        descriptor = component_descriptor(model_type, subtype)
    except ValueError as error:
        # Track profiles are loaded by the track reader, not the component registry.
        if model_type == "track-model":
            return ModelEntry(path, root.get("id", ""), model_type, subtype, None, "track")
        raise ValueError(f"{path}: {error}") from error
    return ModelEntry(
        path,
        root.get("id", ""),
        descriptor.model_type,
        descriptor.subtype,
        descriptor.capability,
        descriptor.kind,
    )


def skipped_files(root: Path = REPOSITORY_ROOT / "models") -> tuple[tuple[Path, str], ...]:
    """Return unreadable or unregistered XML files."""
    root = Path(root).resolve()
    skipped: list[tuple[Path, str]] = []
    for path in sorted(root.rglob("*.xml")):
        if path.relative_to(root).parts[:1] == ("sweeps",):
            continue
        try:
            read_entry(path)
        except ValueError as error:
            skipped.append((path, str(error)))
    return tuple(skipped)


def scan_catalogue(root: Path = REPOSITORY_ROOT / "models") -> tuple[ModelEntry, ...]:
    """Scan registered XML files below the model root."""
    root = Path(root).resolve()
    entries: list[ModelEntry] = []
    for path in sorted(root.rglob("*.xml")):
        if path.relative_to(root).parts[:1] == ("sweeps",):
            continue
        try:
            entries.append(read_entry(path))
        except ValueError:
            continue
    return tuple(entries)


def fits(slot_entry: ModelEntry, candidate: ModelEntry) -> bool:
    """Return whether a candidate can fill a slot."""
    if slot_entry.capability is not None:
        return candidate.capability == slot_entry.capability
    return (candidate.model_type, candidate.subtype) == (
        slot_entry.model_type,
        slot_entry.subtype,
    )


def candidates(slot_entry: ModelEntry, catalogue: tuple[ModelEntry, ...]) -> tuple[ModelEntry, ...]:
    """Return sorted catalogue entries that fit a slot."""
    return tuple(sorted((entry for entry in catalogue if fits(slot_entry, entry)), key=lambda e: e.path))


def tracks(catalogue: tuple[ModelEntry, ...]) -> tuple[ModelEntry, ...]:
    """Return signed closed track profiles."""
    return tuple(
        sorted(
            (entry for entry in catalogue if entry.model_type == "track-model" and entry.subtype == "closed-xy-profile"),
            key=lambda entry: entry.path,
        )
    )
