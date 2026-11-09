# sim_gui

Simulation GUI.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Simulation GUI core without Qt. |
| `app.py` | Start the simulation GUI application. |
| `catalogue.py` | Read and match registered model files. |
| `execute.py` | Prepare, execute, and finish one simulation GUI case. |
| `forward_execute.py` | Run the historical-preview default or opt-in newer policies on the selected planar or full vertical 14DoF plant, with actual-state RunRecord channels. |
| `maneuver_execute.py` | Native sim-study-v1 adapter for maneuver cases; validates and reuses the runner's canonical simulation RunRecord without rendering video. |
| `run_case.py` | Run one GUI study case: the runner child with the case's variant vehicle; only a fresh attempt or its parent's regular `run.log` is admitted. |
| `run_study.py` | Run simulation GUI studies without Qt. |
| `settings.py` | Strictly validated serializable run settings for forward-driver and point-mass workflows, including opt-in operational brake forecasting and transient corner planning. |
| `source_identity.py` | Canonical repository-rooted, fail-closed execution source manifest and digest helper shared by forward preparation and child execution. |
| `telemetry_schema.py` | Versioned accepted-row telemetry schema, unit/null contract, policy-row projection, hybrid provider/allocator identity, and verified-relay work/timing fields. |
| `study.py` | Study definitions, sweep expansion, and persistent case indexes. |
| `variant.py` | Write isolated XML model variants for simulation cases. |
| `vehicle.py` | Vehicle assembly configuration and nested component selection. |

## Subdirectories

| Directory | Purpose |
|---|---|
| `qt/` | Qt layer of the simulation GUI. |
