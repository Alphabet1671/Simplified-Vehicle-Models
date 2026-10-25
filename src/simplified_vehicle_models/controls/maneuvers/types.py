"""Qt-free value types for maneuver control."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True, slots=True)
class RobotCommand:
    """Canonical command: SWA radians and normalized accelerator/brake travel."""
    swa_rad: float
    accelerator: float
    brake: float

    def __post_init__(self):
        raw = (self.swa_rad, self.accelerator, self.brake)
        if any(isinstance(value, (bool, np.bool_)) for value in raw):
            raise ValueError("command values must be numeric, not boolean")
        try:
            values = np.array(raw, dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError("command values must be numeric") from exc
        if not np.all(np.isfinite(values)):
            raise ValueError("command values must be finite")
        if not (0 <= values[1] <= 1 and 0 <= values[2] <= 1):
            raise ValueError("pedals must be fractions in [0,1]")
        if values[1] > 0 and values[2] > 0:
            raise ValueError("pedal overlap is not supported")
        object.__setattr__(self, "swa_rad", float(values[0]))
        object.__setattr__(self, "accelerator", float(values[1]))
        object.__setattr__(self, "brake", float(values[2]))


@dataclass(frozen=True, slots=True)
class ManeuverDefinition:
    """Validated JSON-compatible maneuver definition with immutable boundary copy."""
    _data: dict

    @classmethod
    def from_dict(cls, payload: dict) -> "ManeuverDefinition":
        from .schema import validate_definition
        return cls(validate_definition(payload))

    def to_dict(self) -> dict:
        from copy import deepcopy
        return deepcopy(self._data)
