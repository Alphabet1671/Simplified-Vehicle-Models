"""Per-tick plant integrators for fixed-rate co-simulation."""
from __future__ import annotations
import time
import numpy as np
from scipy.integrate import solve_ivp

# E3 did not pass accuracy; do not advertise an implicit method until implemented.
REGISTERED = ("radau", "radau-loose")


def make_tick_integrator(plant, settings, deadline=None):
    """Create one tick integrator; stateful implementations live per rollout."""
    def check_deadline():
        if deadline is not None and time.monotonic() >= deadline:
            raise RuntimeError("deadline")

    if settings.integrator in ("radau", "radau-loose"):
        loose = settings.integrator == "radau-loose"
        rtol, atol = (1e-4, 1e-6) if loose else (settings.rtol, settings.atol)
        def step(command, t0, state, dt):
            check_deadline()
            sol = solve_ivp(lambda t, x: plant.evaluate(t, x, command).solve(),
                            (t0, t0 + dt), state, method="Radau", rtol=rtol,
                            atol=atol, max_step=min(settings.max_step, dt))
            check_deadline()
            if not sol.success:
                raise RuntimeError(sol.message)
            return sol.y[:, -1]
        return step

    raise ValueError(f"unknown integrator {settings.integrator!r}")
