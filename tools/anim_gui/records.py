"""Find run records: one file, a folder, or the cases of a ``study.json``."""
from __future__ import annotations

import json
import shutil
from dataclasses import dataclass
from pathlib import Path

from tools.gui_common.files import is_inside

RECORD_NAME = "run_record.npz"


@dataclass(frozen=True, slots=True)
class RecordChoice:
    label: str
    path: Path


def study_records(study_json: Path) -> list[RecordChoice]:
    """The finished cases of a study that wrote a record. ``ValueError`` for an unreadable study."""
    study_json = Path(study_json).resolve()
    try:
        data = json.loads(study_json.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read {study_json}: {error}") from error
    if data.get("schema") != "sim-study-v1" or not isinstance(data.get("cases"), list):
        raise ValueError(f"{study_json} is not a sim-study-v1 file")
    choices = []
    for case in data["cases"]:
        relative = case.get("run_record")
        if not relative:
            continue
        path = (study_json.parent / relative).resolve()
        if is_inside(path, study_json.parent) and path.is_file():
            choices.append(RecordChoice(str(case.get("label") or case.get("id")), path))
    return choices


def find_records(folder: Path, *, limit: int = 500) -> list[RecordChoice]:
    """Every ``run_record.npz`` below ``folder`` (at most ``limit``), nearest first."""
    folder = Path(folder).resolve()
    found = sorted(folder.rglob(RECORD_NAME), key=lambda p: (len(p.parts), str(p)))[:limit]
    return [RecordChoice(path.parent.relative_to(folder).as_posix() or ".", path) for path in found]


def remove_partials(output: Path) -> list[Path]:
    """Delete what a killed export leaves behind. Returns what was removed.

    A video export writes ``<stem>.partial.mp4`` and renames it when it is complete, so a
    killed run leaves the partial file and never a half-written final file. A PNG export
    leaves frames in its folder; they are removed only when the folder holds nothing else.
    """
    output = Path(output)
    removed: list[Path] = []
    if output.suffix.lower() == ".mp4":
        partial = output.with_name(f"{output.stem}.partial.mp4")
        if partial.exists():
            partial.unlink()
            removed.append(partial)
    elif output.is_dir():
        entries = list(output.iterdir())
        if entries and all(p.name.startswith("frame_") and p.suffix == ".png" for p in entries):
            shutil.rmtree(output)
            removed.append(output)
    return removed
