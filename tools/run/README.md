# run

Run scripts for solver attempts and sweeps.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Package marker. |
| `run_forward_acceptance_chain.sh` | Shell script. |
| `run_iem24_forward_driver.py` | Run an isolated envelope-policy forward-driver attempt under a parent wall deadline. |
| `run_with_deadline.py` | Spawn and supervise one command under a parent wall deadline. |
| `run_iem24_point_mass_track_pair.py` | Run the same supported IEM24 energy-coupled point-mass solve on two signed tracks. |
| `solve_minimum_time_line.py` | Run the stage 1 minimum-time racing-line acceptance pipeline. |
| `sweep_vertical_setup.py` | Legacy-layout ride studies plus physical-strut shaft override catalogue for the vertical 14-DoF model; legacy quarter-car metrics reject additive physical layouts. |
