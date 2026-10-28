"""Per-tick plant integrators. Task 3 adds and selects faster methods.

``make_tick_integrator`` is called once per rollout and may keep state (for
example a reused Jacobian) across ticks; the returned callable advances one
controller tick with the command held constant.
"""
from __future__ import annotations
import time
from scipy.integrate import solve_ivp

REGISTERED = ("radau",)

def make_tick_integrator(plant, settings, deadline=None):
    if settings.integrator == "radau":
        def step(command, t0, state, dt):
            if deadline is not None and time.monotonic() >= deadline:
                raise RuntimeError("deadline")
            sol = solve_ivp(lambda t, x: plant.evaluate(t, x, command).solve(),
                            (t0, t0 + dt), state, method="Radau", rtol=settings.rtol,
                            atol=settings.atol, max_step=min(settings.max_step, dt))
            if not sol.success:
                raise RuntimeError(sol.message)
            return sol.y[:, -1]
        return step
    raise ValueError(f"unknown integrator {settings.integrator!r}")
