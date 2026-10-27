from dataclasses import dataclass, field
from typing import Mapping
from types import MappingProxyType

import numpy as np


def _freeze_array(value):
    arr = np.array(value, copy=True)
    arr.setflags(write=False)
    return arr


@dataclass(frozen=True)
class RunData:
    run_id: int
    tire_id: str
    channels: Mapping[str, np.ndarray]
    metadata: Mapping[str, str]
    raw_indices: np.ndarray

    def __post_init__(self):
        object.__setattr__(self, 'channels', MappingProxyType({k: _freeze_array(v) for k, v in self.channels.items()}))
        object.__setattr__(self, 'metadata', MappingProxyType(dict(self.metadata)))
        object.__setattr__(self, 'raw_indices', _freeze_array(self.raw_indices))


@dataclass(frozen=True)
class SampleSet:
    alpha: np.ndarray
    kappa: np.ndarray
    fz: np.ndarray
    target: np.ndarray
    segment_ids: np.ndarray
    run_id: int | None = None
    tire_id: str | None = None
    raw_indices: np.ndarray = field(default_factory=lambda: np.array([], dtype=np.int64))

    def __post_init__(self):
        for name in ('alpha', 'kappa', 'fz', 'target', 'segment_ids', 'raw_indices'):
            object.__setattr__(self, name, _freeze_array(getattr(self, name)))


@dataclass(frozen=True)
class SteadySelection:
    lateral: SampleSet
    longitudinal: SampleSet
    combined_fx: SampleSet
    combined_fy: SampleSet
    mz: SampleSet
    belt_speed: float
