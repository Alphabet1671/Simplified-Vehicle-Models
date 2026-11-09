# controls

Driver-input conversion and controller contracts.

## Files

| File | Contents |
|---|---|
| `brakes.py` | Convert a driver brake-pedal force into four wheel brake torques. |
| `brake_inverse.py` | Bounded scalar inversion of summed hydraulic wheel-torque magnitude; does not allocate independent wheel targets or invert acceleration. |
| `planning.py` | Controller-facing planning contract; no vehicle equations live here. |
| `replay.py` | Replay adapters for controller-in-loop correction; no TV physics here. |
| `steering.py` | Convert a steering-wheel angle into the four road-wheel angles. |

## Subdirectories

| Directory | Purpose |
|---|---|
| `maneuvers/` | Open-loop maneuver definitions and controllers. |
