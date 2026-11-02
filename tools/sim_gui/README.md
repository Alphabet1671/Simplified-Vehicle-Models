# sim_gui

Simulation GUI.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Simulation GUI core without Qt. |
| `app.py` | Start the simulation GUI application. |
| `catalogue.py` | Read and match registered model files. |
| `execute.py` | Prepare, execute, and finish one simulation GUI case. |
| `forward_execute.py` | Run and validate the current continuous-policy forward driver with selected case inputs. |
| `maneuver_execute.py` | Native sim-study-v1 adapter for maneuver cases; validates and reuses the runner's canonical simulation RunRecord without rendering video. |
| `run_case.py` | Run one GUI study case: the runner child with the case's variant vehicle. |
| `run_study.py` | Run simulation GUI studies without Qt. |
| `settings.py` | Serializable run settings for forward-driver and point-mass workflows. |
| `study.py` | Study definitions, sweep expansion, and persistent case indexes. |
| `variant.py` | Write isolated XML model variants for simulation cases. |
| `vehicle.py` | Vehicle assembly configuration and nested component selection. |

## Subdirectories

| Directory | Purpose |
|---|---|
| `qt/` | Qt layer of the simulation GUI. |
