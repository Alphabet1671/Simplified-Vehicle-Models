from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True, slots=True)
class ModelReference:
    tag: str
    role: str | None
    model: Model


@dataclass(frozen=True, slots=True)
class Model:
    path: Path
    model_id: str
    model_type: str
    subtype: str
    parameters: dict[str, float]
    units: dict[str, str]
    references: tuple[ModelReference, ...]
    requires: tuple[str, ...] = ()
    driven_wheels: tuple[str, ...] = ()
