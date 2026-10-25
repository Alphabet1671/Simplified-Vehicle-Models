"""Evidence-only analysis for completed or partial maneuver simulations.

This module deliberately does not evaluate a plant or import model code.  Missing,
malformed, or poorly sampled evidence is reported as unavailable rather than guessed.
"""
from collections.abc import Mapping
import math
import numpy as np


def _data(definition):
    if hasattr(definition, 'to_dict'):
        try:
            return definition.to_dict()
        except Exception:
            return {}
    return definition if isinstance(definition, Mapping) else {}


def _get(obj, key, default=None):
    if isinstance(obj, Mapping):
        return obj.get(key, default)
    return getattr(obj, key, default)


def _finite_vector(value):
    try:
        a = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None
    return a if a.ndim == 1 and a.size and np.all(np.isfinite(a)) else None


def _rows(values):
    try:
        return tuple(values)
    except (TypeError, ValueError):
        return ()


def analyze_maneuver(definition, result) -> dict:
    """Summarize definition and ``SimulationResult`` evidence without simulation.

    Numeric derivatives/integrals are only calculated for finite, strictly
    increasing, nearly uniform time samples with matching finite array shapes.
    """
    d = _data(definition)
    preset = d.get('preset')
    steer = d.get('steering', {}) if isinstance(d.get('steering', {}), Mapping) else {}
    longitudinal = d.get('longitudinal', {}) if isinstance(d.get('longitudinal', {}), Mapping) else {}
    mode = steer.get('mode')

    raw_t = _finite_vector(_get(result, 'time_s'))
    state = None
    try:
        candidate = np.asarray(_get(result, 'state'), dtype=float)
        if candidate.ndim == 2 and candidate.shape[0] > 0 and candidate.shape[1] > 0:
            state = candidate
    except (TypeError, ValueError, OverflowError):
        pass
    times_valid = raw_t is not None and state is not None and state.shape[0] == raw_t.size
    uniform = False
    if times_valid and raw_t.size >= 2:
        dt = np.diff(raw_t)
        uniform = bool(np.all(dt > 0) and np.all(np.isfinite(dt)) and
                       np.allclose(dt, np.median(dt), rtol=1e-3, atol=1e-9))
    valid = bool(times_valid and raw_t.size >= 2 and uniform and np.all(np.isfinite(state)))
    t = raw_t if valid else None
    s = state if valid else None

    rates = None
    try:
        r = np.asarray(_get(result, 'state_rate'), dtype=float)
        if valid and r.shape == s.shape and np.all(np.isfinite(r)):
            rates = r
    except (TypeError, ValueError, OverflowError):
        pass

    observations = _rows(_get(result, 'observations', ()))
    channels = _rows(_get(result, 'channels', ()))
    applied = _rows(_get(result, 'applied_commands', ()))
    requested = _rows(_get(result, 'requested_commands', ()))
    events = _rows(_get(result, 'events', ()))

    def series(rows, field):
        vals = []
        for row in rows:
            v = _get(row, field)
            try: v = float(v)
            except (TypeError, ValueError, OverflowError): return None
            if not math.isfinite(v): return None
            vals.append(v)
        return np.asarray(vals) if vals else None

    speeds = series(observations, 'speed_mps')
    if speeds is None and valid and s.shape[1] > 3:
        speeds = s[:, 3]
    ay = series(observations, 'ay_mps2')
    swa = series(observations, 'swa_rad')
    metrics = {
        'sample_count': int(raw_t.size) if raw_t is not None else 0,
        'duration_s': float(raw_t[-1] - raw_t[0]) if valid else None,
        'peak_speed_mps': float(np.max(speeds)) if speeds is not None else None,
        'peak_abs_lateral_acceleration_mps2': float(np.max(np.abs(ay))) if ay is not None else None,
        'peak_abs_swa_rad': float(np.max(np.abs(swa))) if swa is not None else None,
        'integrated_distance_m': float(np.trapezoid(speeds, t)) if valid and speeds is not None and speeds.size == t.size else None,
    }
    if rates is not None:
        metrics['peak_abs_state_rate'] = float(np.max(np.abs(rates)))
    else:
        metrics['peak_abs_state_rate'] = None

    termination = str(_get(result, 'termination_reason', 'unknown'))
    outcome = str(_get(result, 'execution_outcome', 'unknown'))
    execution = {'outcome': outcome, 'termination_reason': termination,
                 'sampled_arrays_valid': valid, 'uniform_sampling': uniform}
    inputs = {'requested_command_count': len(requested), 'applied_command_count': len(applied),
              'command_evidence_complete': bool(applied) and len(applied) == (raw_t.size if raw_t is not None else -1)}
    interventions = []
    if termination in {'model_error', 'model_limit', 'nonfinite_state'} or outcome == 'failed':
        interventions.append({'kind': 'model_error', 'reason': termination})
    if termination in {'cancelled', 'wall_budget'} or outcome == 'interrupted':
        interventions.append({'kind': 'execution_interruption', 'reason': termination})
    for event in events:
        kind = _get(event, 'kind', 'event')
        interventions.append({'kind': str(kind), 'evidence': dict(event) if isinstance(event, Mapping) else str(event)})

    objective = {'preset': preset, 'steering_mode': mode, 'longitudinal_mode': longitudinal.get('mode')}
    if mode == 'locked_swa':
        objective['research_classification'] = 'fixed-steering-angle response; not a peak-limit estimate'
    if preset == 'circular_skidpad':
        # Require explicit per-sample geometric error evidence. Transient spikes,
        # saturation and unmet target speed are diagnostics, never limit brackets.
        path_err = None
        for key in ('radial_error_m', 'path_error_m'):
            candidate = series(channels, key)
            if candidate is not None and valid and candidate.size == t.size:
                path_err = candidate
                break
        target_speed = steer.get('target_speed_mps', longitudinal.get('target_speed_mps'))
        # Limit classifications require speed and path-error observations on the
        # same accepted sample timeline as valid time/state evidence. A shorter
        # observation series must never be implicitly paired with path samples.
        evidence_aligned = bool(valid and speeds is not None and speeds.size == t.size and
                                path_err is not None and path_err.size == t.size)
        if not evidence_aligned:
            interventions.append({'kind': 'evidence_misaligned'})
        speed_met = None
        speed_window = None
        if speeds is not None and target_speed is not None:
            try:
                target_speed = float(target_speed)
                if math.isfinite(target_speed) and target_speed > 0 and speeds.size >= 3:
                    window_n = min(5, speeds.size)
                    speed_window = speeds[-window_n:]
                    # Require target attainment persistently throughout the
                    # evidence window; a sparse mean or missing value is not proof.
                    speed_met = bool(np.all(speed_window >= target_speed * .98))
            except (TypeError, ValueError, OverflowError):
                pass
        # Persistence threshold is a conservative multi-sample condition; no
        # geometry evidence means no physical conclusion.
        persistent = None
        bad = None
        tolerance = float(steer.get('path_tolerance_m', .1)) if isinstance(steer.get('path_tolerance_m', .1), (int,float)) else .1
        if path_err is not None:
            bad = np.abs(path_err) > max(tolerance, 0.)
            tail = bad[-min(5, bad.size):]
            persistent = bool(bad.size >= 3 and np.all(tail[-3:]))
        saturated = any(bool(_get(c, 'saturated', False)) for c in channels)
        if saturated:
            interventions.append({'kind': 'actuator_saturation'})
        if speed_met is False:
            interventions.append({'kind': 'target_speed_unmet'})
        elif speed_met is None:
            interventions.append({'kind': 'target_speed_evidence_insufficient'})
        model_failed = termination in {'model_error', 'model_limit', 'nonfinite_state'} or outcome == 'failed'
        evidence_ok = speed_met is True and evidence_aligned
        bracket = persistent is True and not saturated and evidence_ok and not model_failed
        clean_sweep = (persistent is False and path_err is not None and not np.any(np.abs(path_err) > max(float(steer.get('path_tolerance_m', .1)), 0.))
                       and not saturated and evidence_ok and not model_failed)
        if bracket and speeds is not None and speeds.size == path_err.size:
            bad_indices = np.flatnonzero(bad)
            first_bad = int(bad_indices[0])
            run_start = next((int(i) for i in range(first_bad, len(bad))
                              if np.all(bad[i:min(i + 3, len(bad))])), first_bad)
            # Bracket only when the persistent invalid run is preceded by a
            # valid sample; never manufacture a same-speed bracket at index 0.
            prior_valid = np.flatnonzero(~bad[:run_start])
            if prior_valid.size:
                last_valid = float(speeds[int(prior_valid[-1])])
                first_invalid = float(speeds[run_start])
            else:
                bracket = False
                last_valid, first_invalid = None, None
        elif clean_sweep and speeds is not None and speeds.size:
            last_valid, first_invalid = float(speeds[-1]), None
        else:
            last_valid, first_invalid = None, None
        objective.update({'last_valid_speed_mps': last_valid,
                          'first_invalid_speed_mps': first_invalid,
                          'limit_status': ('bracketed_physical_limit' if bracket else
                                           'limit_not_reached' if clean_sweep else 'not_established'),
                          'persistent_path_error': persistent,
                          'target_speed_met': speed_met})
    return {'execution': execution, 'input_fidelity': inputs, 'objective': objective,
            'metrics': metrics, 'interventions': interventions}
