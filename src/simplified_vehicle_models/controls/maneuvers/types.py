"""Qt-free value types for maneuver control."""
from dataclasses import dataclass
from copy import deepcopy
from types import MappingProxyType
import numpy as np


@dataclass(frozen=True, slots=True)
class Observation:
    time_s: float
    x_m: float
    y_m: float
    heading_rad: float
    speed_mps: float
    yaw_rate_rad_s: float
    swa_rad: float
    ay_mps2: float

    def __post_init__(self):
        vals=(self.time_s,self.x_m,self.y_m,self.heading_rad,self.speed_mps,self.yaw_rate_rad_s,self.swa_rad,self.ay_mps2)
        if any(isinstance(v,(bool,np.bool_)) for v in vals): raise ValueError('observation values must be numeric, not boolean')
        try: values=np.asarray(vals,dtype=float)
        except (TypeError,ValueError) as exc: raise ValueError('observation values must be numeric') from exc
        if not np.all(np.isfinite(values)): raise ValueError('observation values must be finite')
        for name,value in zip(('time_s','x_m','y_m','heading_rad','speed_mps','yaw_rate_rad_s','swa_rad','ay_mps2'),values): object.__setattr__(self,name,float(value))


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


def _freeze(value):
    if isinstance(value, dict):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    return value


def _thaw(value):
    if isinstance(value, MappingProxyType):
        return {key: _thaw(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_thaw(item) for item in value]
    return deepcopy(value)


@dataclass(frozen=True, slots=True, init=False)
class ManeuverDefinition:
    """Validated definition held in a recursively immutable representation."""
    __data: object

    def __init__(self, data):
        object.__setattr__(self, '_ManeuverDefinition__data', _freeze(data))

    @property
    def _data(self):
        """Read-only compatibility view; internal storage cannot be replaced."""
        return self.__data

    @classmethod
    def from_dict(cls, payload: dict) -> "ManeuverDefinition":
        from .schema import validate_definition
        return cls(validate_definition(payload))

    def to_dict(self) -> dict:
        return _thaw(self.__data)
