"""Controller-facing planning contract; no vehicle equations live here."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from typing import Protocol

import numpy as np


@dataclass(frozen=True, slots=True)
class PathDemand:
    speed: float
    curvature: float
    longitudinal_acceleration: float
    lateral_acceleration: float
    yaw_rate: float
    steering_angle: float = 0.0

    def __post_init__(self):
        values = (self.speed, self.curvature, self.longitudinal_acceleration,
                  self.lateral_acceleration, self.yaw_rate, self.steering_angle)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('path demand values must be finite')
        if self.speed <= 0.0:
            raise ValueError('path demand speed must be positive')


@dataclass(frozen=True, slots=True)
class ControlResponse:
    feasible: bool
    longitudinal_acceleration: float
    yaw_moment: float
    saturation: tuple[str, ...] = ()

    def __post_init__(self):
        if not (math.isfinite(self.longitudinal_acceleration) and math.isfinite(self.yaw_moment)):
            raise ValueError('control response values must be finite')
        if not isinstance(self.feasible, bool):
            raise ValueError('feasible must be boolean')
        saturation = self.saturation
        if isinstance(saturation, str):
            raise ValueError('saturation must be a sequence of tags, not a string')
        try:
            tags = tuple(saturation)
        except TypeError as error:
            raise ValueError('saturation must be a sequence of tags') from error
        if not all(isinstance(tag, str) for tag in tags):
            raise ValueError('saturation tags must be strings')
        object.__setattr__(self, 'saturation', tags)


class DriveControlPolicy(Protocol):
    name: str
    provenance_hash: str
    is_stateful: bool

    def evaluate(self, demand: PathDemand) -> ControlResponse: ...


def _canonical(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, (list, tuple)):
        return [_canonical(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _canonical(item) for key, item in sorted(value.items())}
    raise TypeError(f'cannot canonically serialize controller parameter {type(value).__qualname__}')


def _public_parameters(policy):
    names = set(getattr(policy, '__dict__', {}))
    for cls in type(policy).__mro__:
        declared = getattr(cls, '__slots__', ())
        names.update((declared,) if isinstance(declared, str) else declared)
    names -= {'name', 'provenance_hash', 'is_stateful'}
    return {name: _canonical(getattr(policy, name)) for name in sorted(names)
            if not name.startswith('_') and hasattr(policy, name)}


def controller_hash(policy) -> str:
    """Hash the declared controller identity and canonical public parameters."""
    payload = {'name': policy.name, 'provenance': policy.provenance_hash,
               'is_stateful': policy.is_stateful,
               'parameters': _public_parameters(policy)}
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


CONTROL_POLICIES = {}


def register_policy(name, factory):
    if not isinstance(name, str) or not name:
        raise ValueError('policy name must be a non-empty string')
    CONTROL_POLICIES[name] = factory


def build_policy(name, **kwargs):
    if name not in CONTROL_POLICIES:
        raise KeyError(f'unknown control policy {name!r}')
    return CONTROL_POLICIES[name](**kwargs)
