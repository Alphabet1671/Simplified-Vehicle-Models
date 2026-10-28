"""Resolve a run's belt speed while preserving the source measurement."""
from __future__ import annotations

import math
from collections.abc import Mapping

import numpy as np

from .contracts import RunData

_PLACEHOLDER_SPEED_MPS = 0.01
_PLACEHOLDER_TOLERANCE_MPS = 0.002
_MIN_RADIUS_M = 0.05
_MAX_RADIUS_M = 1.0
_MIN_OMEGA_RAD_S = 1.0


def _copy_run(run: RunData, channels: Mapping[str, np.ndarray], metadata: Mapping[str, str]) -> RunData:
    return RunData(run.run_id, run.tire_id, channels, metadata, run.raw_indices)


def _validated_vector(name: str, values, expected_length: int) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.ndim != 1:
        raise ValueError(f'{name} must be one-dimensional')
    if len(array) != expected_length:
        raise ValueError(f'{name} length must match raw_indices ({expected_length})')
    return array


def _wheel_speed(run: RunData) -> tuple[np.ndarray | None, str | None]:
    channels = run.channels
    missing = [name for name in ('omega', 'effective_radius', 'kappa') if name not in channels]
    if missing:
        return None, 'Wheel-speed recovery unavailable: missing ' + ', '.join(missing)
    expected = len(run.raw_indices)
    try:
        omega = _validated_vector('omega', channels['omega'], expected)
        radius = _validated_vector('effective_radius', channels['effective_radius'], expected)
        kappa = _validated_vector('kappa', channels['kappa'], expected)
    except ValueError as exc:
        return None, f'Wheel-speed recovery unavailable: {exc}'
    if not (np.all(np.isfinite(omega)) and np.all(np.isfinite(radius)) and np.all(np.isfinite(kappa))):
        return None, 'Wheel-speed recovery rejected: inputs contain nonfinite values'
    if np.any((radius <= _MIN_RADIUS_M) | (radius >= _MAX_RADIUS_M)):
        return None, 'Wheel-speed recovery rejected: effective radius must be strictly between 0.05 and 1.0 m'
    denominator = 1.0 + kappa
    if np.any(np.abs(denominator) < 1e-6):
        return None, 'Wheel-speed recovery rejected: slip-ratio denominator is zero or unsafe'
    # A stationary/quantized wheel cannot establish belt speed; do not convert
    # its small sensor noise into an apparently meaningful velocity.
    if np.any(np.abs(omega) <= _MIN_OMEGA_RAD_S):
        return None, 'Wheel-speed recovery rejected: wheel rotation is near zero or not meaningful in every sample (|omega| must exceed 1 rad/s)'
    speed = omega * radius / denominator
    if not np.all(np.isfinite(speed)):
        return None, 'Wheel-speed recovery rejected: computed speed is nonfinite'
    # Preserve sign from the declared wheel-rotation convention; require that
    # conversion has not produced an implausibly large value.
    if np.any(np.abs(speed) > 100.0):
        return None, 'Wheel-speed recovery rejected: computed speed exceeds 100 m/s plausibility limit'
    return speed, None


def _is_placeholder(speed: np.ndarray) -> bool:
    return bool(speed.size and np.all(np.isfinite(speed)) and
                np.all(speed > 0) and
                np.max(np.abs(speed - _PLACEHOLDER_SPEED_MPS)) <= _PLACEHOLDER_TOLERANCE_MPS)


def resolve_belt_speed(run: RunData, *, source: str = 'auto', override: float | None = None) -> RunData:
    """Return a copied run with selected ``speed`` and auditable provenance.

    ``auto`` keeps a usable measured belt-speed channel as-is (including zero
    and near-zero samples in a moving trace). It attempts wheel recovery only
    when measured speed is absent or is a uniform ~0.01 m/s placeholder.
    ``wheel`` explicitly requests wheel-derived speed; ``measured`` restores
    the original ``speed_measured`` channel; ``override`` uses a finite,
    positive scalar m/s value. Failed wheel recovery raises rather than silently
    falling back to another source.
    """
    if source not in {'auto', 'measured', 'wheel', 'override'}:
        raise ValueError("source must be 'auto', 'measured', 'wheel', or 'override'")
    channels = dict(run.channels)
    metadata = dict(run.metadata)
    expected = len(run.raw_indices)
    measured = channels.get('speed_measured')
    current = channels.get('speed')
    if measured is None and current is not None:
        measured = _validated_vector('speed', current, expected).copy()
        channels['speed_measured'] = measured
    elif measured is not None:
        measured = _validated_vector('speed_measured', measured, expected)
    if current is not None:
        current = _validated_vector('speed', current, expected)
    if measured is not None and not np.all(np.isfinite(measured)):
        raise ValueError('measured belt speed contains nonfinite values')

    warning = ''
    selected_source = source
    if source == 'override':
        if override is None or not math.isfinite(float(override)) or float(override) <= 0:
            raise ValueError('override must be a finite positive speed in m/s')
        if current is None:
            if measured is None:
                raise ValueError('cannot apply speed override without run channels')
            current = measured
        selected = np.full(expected, float(override), dtype=float)
    elif source == 'measured':
        if measured is None:
            raise ValueError('no original measured speed is available to restore')
        selected = measured.copy()
    elif source == 'wheel':
        selected, warning = _wheel_speed(run)
        if selected is None:
            raise ValueError(warning)
        selected_source = 'wheel'
    else:  # auto
        if current is not None and measured is None:
            measured = np.asarray(current, dtype=float).copy()
            channels['speed_measured'] = measured
        if measured is not None and not _is_placeholder(measured):
            selected = measured.copy()
            selected_source = 'measured'
            # Explicitly flag a wheel-sensor near-zero situation without
            # replacing directly measured belt velocity or true stop samples.
            omega = channels.get('omega')
            if omega is not None and np.all(np.isfinite(omega)) and not np.any(np.abs(omega) > _MIN_OMEGA_RAD_S):
                warning = 'Wheel rotation is near zero; retained measured belt speed (no wheel-derived replacement).'
        else:
            selected, warning = _wheel_speed(run)
            if selected is None:
                if measured is None:
                    raise ValueError(warning or 'no measured speed or usable wheel-speed inputs')
                selected = measured.copy()
                selected_source = 'measured'
                warning = 'Placeholder speed retained: ' + (warning or 'wheel recovery unavailable')
            else:
                selected_source = 'wheel'
                if measured is not None:
                    warning = 'Replaced uniform ~0.01 m/s placeholder with wheel-derived speed.'

    selected = _validated_vector('selected speed', selected, expected)
    channels['speed'] = selected
    metadata['belt_speed_source'] = selected_source
    if warning:
        metadata['belt_speed_warning'] = warning
    else:
        metadata.pop('belt_speed_warning', None)
    return _copy_run(run, channels, metadata)
