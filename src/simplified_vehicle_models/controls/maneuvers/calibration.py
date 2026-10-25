"""Frozen pre-run peak lateral-acceleration amplitude calibration.

This module deliberately runs trial simulations before the eventual maneuver.  It
never observes a final-run signal and never installs a feedback controller.
"""
from dataclasses import dataclass
from hashlib import sha256
import json
import math
from typing import Callable

import numpy as np


@dataclass(frozen=True, slots=True)
class AyCalibration:
    """A pre-run SWA selection and its audit evidence."""
    amplitude_rad: float | None
    target_mps2: float
    achieved_mps2: float | None
    residual_mps2: float | None
    definition_sha256: str
    trials: tuple[tuple[float, float], ...]
    status: str


def _without_ui(value):
    if isinstance(value, dict):
        return {str(k): _without_ui(v) for k, v in value.items()
                if str(k) not in {'ui', 'output', 'output_path', 'output_directory'}}
    if isinstance(value, (tuple, list)):
        return [_without_ui(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def definition_fingerprint(definition) -> str:
    """Hash expanded maneuver/model/actuator data, excluding presentation paths."""
    if hasattr(definition, 'to_dict'):
        definition = definition.to_dict()
    payload = _without_ui(definition)
    encoded = json.dumps(payload, sort_keys=True, separators=(',', ':'), default=str)
    return sha256(encoded.encode('utf-8')).hexdigest()


def _accepted_peak(result) -> float:
    """Extract peak absolute body ay from accepted simulator evidence only."""
    outcome = getattr(result, 'execution_outcome', None)
    reason = getattr(result, 'termination_reason', None)
    if outcome == 'failed' or reason in {'model_limit', 'model_error', 'nonfinite_state'}:
        raise RuntimeError('model_limit')
    observations = getattr(result, 'observations', None)
    if observations is not None:
        values = [getattr(item, 'ay_mps2', float('nan')) for item in observations]
    elif isinstance(result, dict):
        values = result.get('ay_mps2', result.get('accepted_ay_mps2', ()))
    else:
        values = result
    try:
        values = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise RuntimeError('model_limit') from exc
    values = values[np.isfinite(values)]
    if values.size == 0:
        raise RuntimeError('model_limit')
    return float(np.max(np.abs(values)))


def calibrate_peak_ay(definition, run_trial: Callable[[float], object],
                      amplitude_bounds_rad: tuple[float, float],
                      max_trials: int = 24) -> AyCalibration:
    """Scan/refine a frozen SWA amplitude for a requested peak ``ay``.

    The first eight trials uniformly scan *positive magnitudes*, including both
    endpoints.  Refinement is allowed only between adjacent monotone samples
    bracketing the target.  The signed amplitude is selected only after all
    trials, so a later final simulation cannot mutate the calibration.
    """
    if not callable(run_trial):
        raise ValueError('run_trial must be callable')
    if isinstance(max_trials, bool) or not isinstance(max_trials, int) or max_trials < 8:
        raise ValueError('max_trials must be an integer >= 8')
    try:
        lower, upper = (float(item) for item in amplitude_bounds_rad)
    except (TypeError, ValueError) as exc:
        raise ValueError('amplitude_bounds_rad must contain two numbers') from exc
    if not (math.isfinite(lower) and math.isfinite(upper) and 0 <= lower < upper):
        raise ValueError('amplitude bounds must be finite, nonnegative, increasing')
    payload = definition.to_dict() if hasattr(definition, 'to_dict') else definition
    target = None
    canonical_target = False
    steering = None
    if isinstance(payload, dict):
        target = payload.get('peak_ay_mps2', payload.get('target_ay_mps2'))
        if target is None and isinstance(payload.get('steering'), dict):
            # ``amplitude_mps2`` is the canonical maneuver-v1 *magnitude*;
            # its direction is separately represented by steering.direction.
            steering = payload['steering']
            target = steering.get('amplitude_mps2', steering.get('peak_ay_mps2'))
            canonical_target = 'amplitude_mps2' in steering
    if target is None:
        raise ValueError('definition must specify peak_ay_mps2')
    target = float(target)
    if not math.isfinite(target) or target == 0:
        raise ValueError('peak_ay_mps2 must be finite and nonzero')
    target_abs = abs(target)
    direction = steering.get('direction', 1) if canonical_target else math.copysign(1., target)
    if direction not in (-1, 1):
        raise ValueError('steering.direction must be -1 or +1')
    sign = float(direction)
    # Canonical definition amplitude is magnitude; expose a signed request in
    # this calibration record so residual and selected SWA preserve direction.
    target = sign * target_abs
    fingerprint = definition_fingerprint(payload)
    trials: list[tuple[float, float]] = []

    def sample(magnitude: float):
        try:
            achieved = _accepted_peak(run_trial(sign * magnitude))
        except RuntimeError:
            return None
        trials.append((sign * magnitude, achieved))
        return achieved

    grid = np.linspace(lower, upper, 8)
    values = [sample(float(magnitude)) for magnitude in grid]
    if any(value is None for value in values):
        return AyCalibration(None, target, None, None, fingerprint, tuple(trials), 'model_limit')
    # Any negative finite difference is an ambiguous physical response.  Do not
    # pretend bisection is valid when tire/saturation response folds over.
    if any(b + 1e-12 < a for a, b in zip(values, values[1:])):
        return AyCalibration(None, target, None, None, fingerprint, tuple(trials), 'ambiguous')
    if target_abs < values[0] - 1e-12 or target_abs > values[-1] + 1e-12:
        return AyCalibration(None, target, None, None, fingerprint, tuple(trials), 'unreachable')
    tolerance = max(.05, .02 * target_abs)
    qualifying = [(float(mag), float(value)) for mag, value in zip(grid, values)
                  if abs(value - target_abs) <= tolerance]
    # Locate the first feasible monotonic bracket; this implements smallest
    # qualifying magnitude rather than a potentially higher-response branch.
    bracket = next(((float(grid[i]), float(grid[i + 1]), values[i], values[i + 1])
                    for i in range(7) if values[i] <= target_abs <= values[i + 1]), None)
    if bracket is not None:
        lo, hi, lo_value, hi_value = bracket
        while len(trials) < max_trials and hi - lo > 1e-10:
            mid = (lo + hi) / 2
            mid_value = sample(mid)
            if mid_value is None:
                return AyCalibration(None, target, None, None, fingerprint, tuple(trials), 'model_limit')
            if mid_value + 1e-12 < lo_value or mid_value > hi_value + 1e-12:
                return AyCalibration(None, target, None, None, fingerprint, tuple(trials), 'ambiguous')
            if abs(mid_value - target_abs) <= tolerance:
                qualifying.append((mid, mid_value))
                hi, hi_value = mid, mid_value
            elif mid_value < target_abs:
                lo, lo_value = mid, mid_value
            else:
                hi, hi_value = mid, mid_value
    if not qualifying:
        # The trial budget ended between samples; report the closest evidence,
        # but never call that a match.
        nearest = min(zip(grid, values), key=lambda pair: abs(pair[1] - target_abs))
        return AyCalibration(sign * float(nearest[0]), target, float(nearest[1]),
                             float(nearest[1] - target_abs), fingerprint, tuple(trials), 'unreachable')
    magnitude, achieved = min(qualifying, key=lambda pair: pair[0])
    return AyCalibration(sign * magnitude, target, achieved, achieved - target_abs,
                         fingerprint, tuple(trials), 'matched')
