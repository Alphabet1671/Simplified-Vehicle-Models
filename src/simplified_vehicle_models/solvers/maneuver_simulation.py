"""Deterministic fixed-grid simulation for sampled maneuver programs.

This intentionally uses explicit Euler stepping; it is a small predictable core,
not a stiff/adaptive integrator.
"""
from dataclasses import dataclass
from typing import Callable
import numpy as np


@dataclass(frozen=True, slots=True)
class SimulationSettings:
    duration_s: float
    controller_tick_s: float
    max_step_s: float = 0.01
    cancel: Callable[[], bool] | None = None

    def __post_init__(self):
        for name in ("duration_s", "controller_tick_s", "max_step_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True, slots=True)
class SimulationResult:
    time_s: np.ndarray
    state: np.ndarray
    requested: tuple
    applied: tuple
    observations: tuple
    status: str
    error: str | None = None

    def __post_init__(self):
        for name in ("time_s", "state"):
            arr = np.array(getattr(self, name), dtype=float, copy=True)
            arr.setflags(write=False)
            object.__setattr__(self, name, arr)


def _command_values(command):
    if hasattr(command, "swa_rad"):
        return np.array([command.swa_rad, command.accelerator, command.brake], dtype=float)
    return np.asarray(command, dtype=float)


def _readonly_result(times, states, requested, applied, observations, status, error=None):
    return SimulationResult(np.asarray(times), np.asarray(states), tuple(requested),
                            tuple(applied), tuple(observations), status, error)


def simulate_maneuver(plant, program, initial_state, settings: SimulationSettings) -> SimulationResult:
    """Run a sampled program over a deterministic explicit-Euler grid.

    Program ticks occur at 0, tick, ... strictly before duration. Breakpoints
    from program and controller are included as integration boundaries.
    """
    y = np.array(initial_state, dtype=float, copy=True)
    if y.ndim != 1 or y.size != plant.state_size or not np.all(np.isfinite(y)):
        raise ValueError("initial_state must be finite and match plant.state_size")
    duration, tick, step = settings.duration_s, settings.controller_tick_s, settings.max_step_s
    breakpoints = {0.0, duration}
    if hasattr(program, "breakpoints"):
        breakpoints.update(float(x) for x in program.breakpoints() if 0 < float(x) < duration)
    if hasattr(plant, "breakpoints"):
        breakpoints.update(float(x) for x in plant.breakpoints() if 0 < float(x) < duration)
    tick_index = 0
    next_tick = 0.0
    times, states, requested, applied, observations = [0.0], [y.copy()], [], [], []
    command = None
    while times[-1] < duration:
        t = times[-1]
        if settings.cancel is not None and settings.cancel():
            return _readonly_result(times, states, requested, applied, observations, "cancelled")
        if tick_index < int(np.ceil(duration / tick)) and abs(t-next_tick) <= 1e-12 and next_tick < duration:
            obs = plant.observe(y)
            command = program.tick(obs, tick)
            requested.append(command)
            tick_index += 1
            next_tick = tick_index * tick
        targets = [duration, t + step]
        targets.extend(b for b in breakpoints if b > t + 1e-12)
        if next_tick < duration and next_tick > t + 1e-12:
            targets.append(next_tick)
        end = min(targets)
        try:
            rates = np.asarray(plant.evaluate(t, y, command).solve(), dtype=float)
            if rates.shape != y.shape or not np.all(np.isfinite(rates)):
                raise ValueError("plant rates must be finite and match state")
            y = y + (end-t) * rates
            if not np.all(np.isfinite(y)):
                raise ValueError("integration produced non-finite state")
        except Exception as exc:
            return _readonly_result(times, states, requested, applied, observations, "error",
                                    f"{type(exc).__name__}: {exc}")
        t = end
        times.append(t); states.append(y.copy())
        applied.append(command)
        observations.append(plant.observe(y))
    return _readonly_result(times, states, requested, applied, observations, "completed")
