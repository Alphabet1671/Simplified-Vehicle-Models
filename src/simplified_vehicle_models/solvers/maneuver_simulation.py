"""Sampled-control maneuver simulation using segmented stiff integration."""
from dataclasses import dataclass
from time import monotonic
import numpy as np
from scipy.integrate import solve_ivp


@dataclass(frozen=True, slots=True)
class SimulationSettings:
    tick_s: float = .01
    rtol: float = 1e-6
    atol: float = 1e-8
    max_wall_s: float = 300.
    cancel: object = None

    def __post_init__(self):
        for name in ('tick_s', 'rtol', 'atol', 'max_wall_s'):
            value = getattr(self, name)
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')


@dataclass(frozen=True, slots=True)
class SimulationResult:
    time_s: np.ndarray
    state: np.ndarray
    state_rate: np.ndarray
    requested_commands: tuple
    applied_commands: tuple
    observations: tuple
    channels: tuple
    events: tuple
    execution_outcome: str
    termination_reason: str

    def __post_init__(self):
        for name in ('time_s', 'state', 'state_rate'):
            value = np.array(getattr(self, name), dtype=float, copy=True)
            value.setflags(write=False)
            object.__setattr__(self, name, value)
        for name in ('requested_commands','applied_commands','observations','channels','events'):
            values = tuple(_freeze(value) for value in getattr(self, name))
            object.__setattr__(self, name, values)


def _freeze(value):
    # RobotCommand and other frozen records may still contain mutable arrays;
    # rebuild dataclasses recursively so snapshots never alias caller objects.
    import dataclasses
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return type(value)(**{field.name: _freeze(getattr(value, field.name)) for field in dataclasses.fields(value)})
    if isinstance(value, dict):
        from types import MappingProxyType
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, np.ndarray):
        result = np.array(value, copy=True); result.setflags(write=False); return result
    if isinstance(value, list): return tuple(_freeze(item) for item in value)
    if isinstance(value, tuple): return tuple(_freeze(item) for item in value)
    return value


def _duration(program):
    data = getattr(program, 'data', {})
    if hasattr(data, 'to_dict'): data = data.to_dict()
    term = data.get('termination', {}) if isinstance(data, dict) else {}
    profile = data.get('profile', {}) if isinstance(data, dict) else {}
    value = term.get('duration_s', profile.get('duration_s'))
    if value is None or not np.isfinite(float(value)) or float(value) <= 0:
        raise ValueError('program must define a finite positive duration_s')
    return float(value)


def _observe(plant, t, y, command):
    try: return plant.observe(t, y, command)
    except TypeError:
        try: return plant.observe(t, y)
        except TypeError: return plant.observe(y)


def _rate(plant, t, y, command):
    result = plant.evaluate(t, y, command).solve()
    result = np.asarray(result, dtype=float)
    if result.shape != y.shape: raise ValueError('plant rates must match state shape')
    return result


class _Stop(Exception):
    pass


def simulate_maneuver(plant, program, initial_state, settings=SimulationSettings()):
    state = np.array(initial_state, dtype=float, copy=True)
    size = getattr(plant, 'state_size', state.size)
    if state.ndim != 1 or state.size != size or not np.all(np.isfinite(state)):
        raise ValueError('initial_state must be finite and match plant.state_size')
    duration, started = _duration(program), monotonic()
    requested_duration = duration
    ticks = []
    k = 0
    while k * settings.tick_s < duration:
        ticks.append(k * settings.tick_s); k += 1
    source = getattr(program, 'csv_source', None)
    source_end = None
    points = {0., duration, *ticks}
    if hasattr(program, 'breakpoints'):
        source_points = tuple(float(x) for x in source.breakpoints()) if source is not None else ()
        if source is not None and hasattr(source, 'time_s') and source_points:
            source_end = float(source.time_s[-1])
            if source_end < duration:
                duration = source_end
                ticks = [tick for tick in ticks if tick < duration]
        if source is not None:
            points.update(x for x in source_points if 0 < x < duration)
        else:
            points.update(float(x) for x in program.breakpoints() if np.isfinite(x) and 0 < float(x) < duration)
    points.add(duration)
    bounds = sorted(points)
    requested, applied, times, states, rates, observations, channels, events = [], [], [], [], [], [], [], []
    command = None
    reason, outcome = ('source_end' if source_end is not None and source_end < requested_duration else 'duration'), 'completed'
    source_truncated = source_end is not None and source_end < requested_duration
    if source_truncated:
        reason = 'source_end'
        ticks = [tick for tick in (k * settings.tick_s for k in range(int(np.ceil(duration / settings.tick_s)))) if tick < duration]
        points = {0., duration, *ticks}
        points.update(x for x in source.breakpoints() if 0 < x < duration)
        bounds = sorted(points)
    current = state.copy()

    def interrupted():
        if settings.cancel is not None and settings.cancel(): return 'cancelled'
        if monotonic() - started >= settings.max_wall_s: return 'wall_budget'
        return None

    for left, right in zip(bounds[:-1], bounds[1:]):
        stop = interrupted()
        if stop:
            outcome, reason = 'interrupted', stop; break
        if any(abs(left - tick) <= 1e-12 for tick in ticks):
            obs = _observe(plant, left, current, command)
            command = program.tick(obs, settings.tick_s)
            requested.append(command)
        elif source is not None:
            if source_end is not None and left > source_end:
                outcome, reason = 'completed', 'source_end'; break
            command = source.sample(left)
        if command is None:
            outcome, reason = 'failed', 'model_error'; break
        # A shared segment boundary is right-continuous: replace the prior
        # interval's endpoint rate/command with the newly active command.
        if times and abs(times[-1] - left) <= 1e-13:
            try:
                rates[-1] = _rate(plant, left, states[-1], command)
                applied[-1] = command
                observations[-1] = _observe(plant, left, states[-1], command)
                channels[-1] = plant.accepted_outputs(left, states[-1], command) if hasattr(plant, 'accepted_outputs') else {}
            except Exception as exc:
                outcome, reason = 'failed', ('model_limit' if type(exc).__name__ in ('TrialStateError','ModelDomainError') else 'model_error')
                break

        def rhs(t, y):
            stop = interrupted()
            if stop: raise _Stop(stop)
            if not np.all(np.isfinite(y)): raise _Stop('nonfinite_state')
            # Implicit Radau stages can probe past a terminal event before
            # localizing it. Keep those internal probes on the valid side of
            # the stop surface; the event function still roots the true state.
            eval_state = y
            if speed_index is not None and y[speed_index] <= speed_floor:
                eval_state = np.array(y, copy=True)
                eval_state[speed_index] = speed_floor
            try: result = _rate(plant, t, eval_state, command)
            except Exception as exc:
                kind = 'model_limit' if type(exc).__name__ in ('TrialStateError','ModelDomainError') else 'model_error'
                raise _Stop(kind) from exc
            if not np.all(np.isfinite(result)): raise _Stop('nonfinite_state')
            return result

        evs = []
        speed_index = 3 if current.size > 3 else None
        speed_floor = None
        if speed_index is not None:
            # Stop just above the plant's minimum valid speed: the event root
            # must not land on a boundary where the plant RHS rejects equality.
            minimum_speed = getattr(plant, 'minimum_speed', None)
            if minimum_speed is None:
                minimum_speed = getattr(plant, 'minimum_forward_speed_mps', 1e-6)
            minimum_speed = float(minimum_speed)
            if not np.isfinite(minimum_speed) or minimum_speed <= 0:
                raise ValueError('plant minimum speed must be finite and positive')
            speed_floor = minimum_speed + max(1e-6, 1e-3 * max(1., minimum_speed))
            def zero_speed(t, y): return y[speed_index] - speed_floor
            zero_speed.terminal = True; zero_speed.direction = -1
            evs.append(zero_speed)
        try:
            sol = solve_ivp(rhs, (left, right), current, method='Radau', rtol=settings.rtol, atol=settings.atol, events=evs or None)
        except _Stop as exc:
            outcome, reason = ('interrupted' if str(exc) in ('cancelled','wall_budget') else 'failed'), str(exc); break
        if not sol.success:
            outcome, reason = 'failed', 'model_error'; break
        bad = False
        for t, y in zip(sol.t, sol.y.T):
            if times and t <= times[-1] + 1e-13: continue
            if not np.all(np.isfinite(y)):
                outcome, reason, bad = 'failed', 'nonfinite_state', True; break
            try: r = _rate(plant, float(t), y, command)
            except Exception as exc:
                outcome, reason, bad = 'failed', ('model_limit' if type(exc).__name__ in ('TrialStateError','ModelDomainError') else 'model_error'), True; break
            if not np.all(np.isfinite(r)):
                outcome, reason, bad = 'failed', 'nonfinite_state', True; break
            times.append(float(t)); states.append(y.copy()); rates.append(r.copy()); applied.append(command)
            observations.append(_observe(plant, float(t), y, command))
            channels.append(plant.accepted_outputs(float(t), y, command) if hasattr(plant,'accepted_outputs') else {})
        if bad: break
        current = sol.y[:, -1].copy()
        if sol.status == 1:
            events.append({'time_s': float(sol.t[-1]), 'kind': 'zero_speed',
                           'speed_threshold_mps': speed_floor,
                           'state_speed_mps': float(sol.y[speed_index, -1])})
            outcome, reason = 'stopped', 'zero_speed'; break

    if not times:
        times, states, rates = [0.], [state.copy()], [np.full_like(state, np.nan)]
        applied, observations, channels = [command], [_observe(plant, 0., state, command) if command is not None else None], [{}]
    return SimulationResult(times, states, rates, tuple(requested), tuple(applied), tuple(observations), tuple(channels), tuple(events), outcome, reason)
