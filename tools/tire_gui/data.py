from pathlib import Path
from typing import Mapping

import numpy as np

from .contracts import RunData

ALIASES = {
    'time': ('ET', 'time'), 'alpha': ('SA', 'SLA', 'SlipAngle', 'ALPHA', 'alpha'),
    'kappa': ('SL', 'SR', 'KAPPA', 'kappa', 'LONGSLIP', 'SlipRatio', 'slip_ratio'),
    'fz': ('FZ', 'Fz', 'fz'), 'fx': ('FX', 'Fx', 'fx'), 'fy': ('FY', 'Fy', 'fy'),
    'mz': ('MZ', 'Mz', 'mz', 'MALIGN', 'AligningMoment', 'aligning_moment'),
    'speed': ('V', 'speed'), 'omega': ('N', 'omega'), 'ia': ('IA', 'ia'), 'pressure': ('P', 'pressure'),
}
UNIT_FACTORS = {
    'alpha': {'deg': np.pi/180, 'degree': np.pi/180, 'rad': 1.0},
    'fz': {'N': -1.0}, 'fx': {'N': 1.0}, 'fy': {'N': 1.0},
    'mz': {'N*m': 1.0, 'Nm': 1.0, 'N-m': 1.0},
    'speed': {'kph': 1/3.6, 'km/h': 1/3.6, 'm/s': 1.0},
    'omega': {'rpm': 2*np.pi/60, 'rad/s': 1.0},
    'kappa': {'dimensionless': 1.0, '1': 1.0, '': 1.0, 'none': 1.0},
    'time': {'s': 1.0, 'sec': 1.0}, 'ia': {'deg': np.pi/180, 'rad': 1.0},
    'pressure': {'kPa': 1000.0, 'Pa': 1.0},
}


def normalize_channels(raw: Mapping[str, np.ndarray], units: Mapping[str, str]) -> dict[str, np.ndarray]:
    """Normalize recognized descriptor-named TTC channels; SL wins over SR."""
    out = {}
    for key, candidates in ALIASES.items():
        found = [name for name in candidates if name in raw]
        if not found:
            continue
        if key == 'kappa' and 'SL' in found and 'SR' in found:
            found.remove('SR')  # source-script preference; no magnitude-based guessing
        if len(found) > 1:
            raise ValueError(f"Ambiguous channel aliases for {key}: {', '.join(found)}")
        name = found[0]
        unit = units.get(name, units.get(key))
        if unit is None:
            raise ValueError(f'Missing unit descriptor for channel {name}')
        if unit not in UNIT_FACTORS[key]:
            raise ValueError(f'Unsupported or ambiguous unit {unit!r} for channel {name}')
        values = np.asarray(raw[name], dtype=float).reshape(-1)
        if not np.all(np.isfinite(values)):
            raise ValueError(f'Nonfinite values in channel {name}')
        factor = UNIT_FACTORS[key][unit]
        out[key] = (-values * factor if key == 'alpha' else values * factor).copy()
    return out


def contiguous_slices(time: np.ndarray, maximum_gap: float = .02) -> tuple[slice, ...]:
    if not np.isfinite(maximum_gap) or maximum_gap <= 0:
        raise ValueError('maximum_gap must be finite and positive')
    t = np.asarray(time, dtype=float).reshape(-1)
    if not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
        raise ValueError('time must be finite and strictly increasing')
    breaks = np.flatnonzero(np.diff(t) > maximum_gap) + 1
    bounds = np.r_[0, breaks, len(t)]
    return tuple(slice(int(a), int(b)) for a, b in zip(bounds[:-1], bounds[1:]) if b > a)


def load_run(path: Path, *, require_monotonic_time: bool = True) -> RunData:
    """Load a TTC MAT v5 run using its explicit channel descriptor table.

    Steady sweeps may explicitly opt out of time monotonicity validation because
    time is not used by their compatibility selection. All original rows and
    the normalized time channel are retained. Transient/event consumers must
    keep the strict default.
    """
    try:
        from scipy.io import loadmat
    except ImportError as exc:
        raise RuntimeError('MAT loading requires the tire-gui scipy extra') from exc
    mat = loadmat(Path(path), squeeze_me=True, struct_as_record=False)
    fields = {k: np.asarray(v).reshape(-1) for k, v in mat.items()
              if not k.startswith('__') and isinstance(v, np.ndarray) and v.ndim > 0 and v.dtype.kind in 'fiu'}
    channel = mat.get('channel')
    if channel is None or not hasattr(channel, '_fieldnames') or not {'name', 'units'} <= set(channel._fieldnames):
        raise ValueError('MAT file is missing channel name/unit descriptors')
    names, units_array = np.asarray(channel.name).reshape(-1), np.asarray(channel.units).reshape(-1)
    if len(names) != len(units_array):
        raise ValueError('MAT channel name/unit descriptors have mismatched lengths')
    units = {str(name).strip(): str(unit).strip() for name, unit in zip(names, units_array)}
    metadata = {k: str(v) for k, v in mat.items() if k in ('tireid', 'testid', 'source') and isinstance(v, (str, np.str_))}
    if 'RUN' in fields and len(fields['RUN']):
        run_ids = np.unique(fields['RUN'])
        if len(run_ids) != 1:
            raise ValueError('RUN channel contains multiple run IDs')
        metadata['run_id'] = str(int(run_ids[0]))
    if 'tireid' in metadata:
        metadata['tire_id'] = metadata['tireid']
    normalized = normalize_channels(fields, units)
    if not normalized:
        raise ValueError('MAT file contains no recognized channels')
    lengths = {len(v) for v in normalized.values()}
    if len(lengths) != 1:
        raise ValueError('Channel vectors have mismatched lengths')
    count = len(normalized.get('time', next(iter(normalized.values()))))
    if 'time' in normalized and np.any(np.diff(normalized['time']) <= 0):
        if require_monotonic_time:
            raise ValueError('Elapsed-time channel is nonmonotonic')
        metadata['time_warning'] = 'Elapsed-time channel is nonmonotonic; retained unchanged for steady-only use'
    return RunData(int(metadata.get('run_id', 0)), metadata.get('tire_id', 'unknown'), normalized, metadata, np.arange(count))
