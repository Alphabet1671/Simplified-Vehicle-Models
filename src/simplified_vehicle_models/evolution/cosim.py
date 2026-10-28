"""Fixed-rate co-simulation: driver + TV step once per tick, plant integrated between."""
from __future__ import annotations
import time
from dataclasses import dataclass, field
from typing import Protocol
import numpy as np
from simplified_vehicle_models.solvers.forward_driver import TrialStateError

@dataclass(frozen=True)
class TvInputs:
    time: float
    speed: float
    yaw_rate: float
    lateral_speed: float
    steering: float
    wheel_speeds: np.ndarray
    driver_torques: np.ndarray

class TorqueVectoringController(Protocol):
    sample_time: float
    def reset(self) -> None: ...
    def step(self, inputs: TvInputs) -> np.ndarray: ...

_TV_CLASSES: dict[type, type] = {}

def tv_plant_class(base_class: type) -> type:
    """Return a cached subclass applying held TV torques at the powertrain seam."""
    cached = _TV_CLASSES.get(base_class)
    if cached is not None:
        return cached
    from dataclasses import replace
    class TvPlant(base_class):
        tv_torques: np.ndarray | None = None
        def powertrain(self, state, command):
            point = super().powertrain(state, command)
            if self.tv_torques is None:
                return point
            return replace(point, wheel_torques=self.tv_torques)
        def driver_torques(self, state, command) -> np.ndarray:
            return np.asarray(super().powertrain(state, command).wheel_torques, float)
    TvPlant.__name__ = TvPlant.__qualname__ = f"Tv{base_class.__name__}"
    _TV_CLASSES[base_class] = TvPlant
    return TvPlant

def state_index(plant_class: type) -> dict[str, int | tuple[int, ...]]:
    """Physical-field indices from published state names."""
    names = tuple(plant_class.state_names)
    wheel = "omega_" if "omega_fl" in names else "wheel_speed_"
    return {"s": names.index("s"), "n": names.index("n"), "v_x": names.index("v_x"),
            "v_y": names.index("v_y"), "yaw_rate": names.index("yaw_rate"),
            "steering": names.index("steering"),
            "wheels": tuple(names.index(wheel + w) for w in ("fl", "fr", "rl", "rr"))}

@dataclass(frozen=True)
class CosimSettings:
    controller_dt: float = 0.01
    integrator: str = "radau"
    rtol: float = 1e-6
    atol: float = 1e-8
    max_step: float = 0.02

@dataclass
class CosimResult:
    time: np.ndarray
    state: np.ndarray
    tv_torque: np.ndarray
    completed: bool
    failure: str | None
    stats: dict = field(default_factory=dict)

def _tv_inputs(t, state, index, driver_torques) -> TvInputs:
    return TvInputs(t, float(state[index["v_x"]]), float(state[index["yaw_rate"]]),
                    float(state[index["v_y"]]), float(state[index["steering"]]),
                    np.asarray(state[list(index["wheels"])], float), driver_torques)

def simulate(plant, driver, tv, initial_state, settings: CosimSettings, *,
             stop_distance: float, max_time: float, deadline: float | None) -> CosimResult:
    """Integrate plant between fixed-rate driver/controller ticks."""
    from simplified_vehicle_models.evolution.tick_integrators import make_tick_integrator
    integrate_tick = make_tick_integrator(plant, settings, deadline)
    dt = float(settings.controller_dt)
    index = state_index(type(plant))
    state = np.asarray(initial_state, float).copy()
    t = 0.0
    times, states, torques = [t], [state.copy()], []
    failure, completed = None, False
    started = time.perf_counter()
    while True:
        if deadline is not None and time.monotonic() >= deadline:
            failure = "deadline"; break
        command = driver.command(plant.observe(state))
        plant.tv_torques = None
        request = plant.driver_torques(state, command)
        held = np.asarray(tv.step(_tv_inputs(t, state, index, request)), float)
        if held.shape != (4,) or not np.all(np.isfinite(held)):
            failure = "tv returned invalid torques"; break
        held.setflags(write=False)
        plant.tv_torques = held
        torques.append(held)
        try:
            state = integrate_tick(command, t, state, dt)
        except (TrialStateError, RuntimeError, ValueError) as error:
            failure = f"{type(error).__name__}: {error}"; break
        t += dt
        times.append(t); states.append(state.copy())
        if state[0] >= stop_distance:
            completed = True; break
        if t >= max_time - 1e-12:
            completed = stop_distance >= 1e8; break
    if len(torques) < len(times):
        torques.append(torques[-1] if torques else np.zeros(4))
    return CosimResult(np.array(times), np.array(states), np.array(torques), completed,
                       failure, {"wall_seconds": time.perf_counter() - started,
                                 "ticks": len(times) - 1})
