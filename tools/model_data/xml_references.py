"""Utilities for moving model XML files while preserving references."""
from __future__ import annotations

import os
from pathlib import Path


def relative_reference(target: Path, directory: Path) -> str:
    """Return *target* relative to *directory*, using POSIX separators."""
    target_path = Path(target).resolve()
    directory_path = Path(directory).resolve()
    try:
        return Path(os.path.relpath(target_path, directory_path)).as_posix()
    except ValueError:
        return target_path.as_posix()


def rewrite_references(tree, source_dir: Path, output_dir: Path,
                       replace: dict[str, Path] | None = None) -> None:
    """Re-anchor every XML ``ref`` and ``path`` from one directory to another.

    ``replace`` maps a component role to a replacement file.  The replacement
    applies only to the ``ref`` attribute on that component.
    """
    replacements = replace or {}
    source_dir = Path(source_dir)
    output_dir = Path(output_dir)
    for element in tree.getroot().iter():
        for attribute in ("ref", "path"):
            value = element.get(attribute)
            if value is None:
                continue
            target = None
            if attribute == "ref" and element.tag == "component":
                target = replacements.get(element.get("role", ""))
            if target is None:
                target = Path(value) if Path(value).is_absolute() else source_dir / value
            element.set(attribute, relative_reference(target, output_dir))
