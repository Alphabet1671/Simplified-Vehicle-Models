# run

Run scripts for solver attempts and sweeps.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Package marker. |
| `run_forward_acceptance_chain.sh` | Shell script. |
| `run_iem24_forward_driver.py` | Run an isolated envelope-policy forward-driver attempt under a parent wall deadline. |
| `run_with_deadline.py` | Spawn and supervise one command under a parent wall deadline. |
| `compare_continuous_driver_throttle.py` | Prepare and run matched continuous-driver throttle comparisons. |
| `study_linear_driver_inputs.py` | Prepare fixed-plan pedal-demand and slew studies. |
| `run_iem24_point_mass_track_pair.py` | Run the same supported IEM24 energy-coupled point-mass solve on two signed tracks. |
| `solve_minimum_time_line.py` | Run the stage 1 minimum-time racing-line acceptance pipeline. |
| `sweep_forward_steering.py` | Bounded steering-gain sweep on the canonical forward lap. |
| `sweep_vertical_setup.py` | Damper, anti-roll and inertance setup sweep for the vertical 14-DoF model. |
