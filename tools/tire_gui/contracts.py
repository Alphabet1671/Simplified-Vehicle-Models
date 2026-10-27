from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping

import numpy as np

from simplified_vehicle_models.parameters.tire import UniTireSimpleParameters
from simplified_vehicle_models.tires.unitire_transient import TransientTireParameters


class FitCancelled(Exception):
    """Raised when a fitting operation is cancelled."""


@dataclass(frozen=True)
class FitResult:
    source_revision: int
    candidate: 'ModelSnapshot'
    metrics: Mapping[str, float]
    converged: bool
    warnings: tuple[str, ...] = ()
    status_by_parameter: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self):
        object.__setattr__(self, 'metrics', MappingProxyType(dict(self.metrics)))
        object.__setattr__(self, 'status_by_parameter', MappingProxyType(dict(self.status_by_parameter)))

def _freeze_array(value):
    arr = np.array(value, copy=True)
    arr.setflags(write=False)
    return arr


@dataclass(frozen=True)
class ModelSnapshot:
    revision: int
    steady: UniTireSimpleParameters
    transient: TransientTireParameters
    values: Mapping[str, float]
    provenance: Mapping[str, str]
    parameter_status: Mapping[str, str] = field(default_factory=dict)
    steady_fingerprint: str = ''
    load_domain: tuple[float, float] = (0.0, 1500.0)
    session_metadata: Mapping = field(default_factory=dict)

    def __post_init__(self):
        object.__setattr__(self, 'values', MappingProxyType(dict(self.values)))
        object.__setattr__(self, 'provenance', MappingProxyType(dict(self.provenance)))
        object.__setattr__(self, 'parameter_status', MappingProxyType(dict(self.parameter_status)))
        object.__setattr__(self, 'session_metadata', MappingProxyType(dict(self.session_metadata)))


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
