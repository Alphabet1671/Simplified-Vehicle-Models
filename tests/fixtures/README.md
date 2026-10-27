# fixtures

Data files that tests read.

## Files

| File | Contents |
|---|---|
| `damper_vc01.csv` | Damper force table fixture from real car data, retained for CI. |
| `forward_driver_runner_planar_baseline.json` | JSON record. |
| `hoosier_43075_r20_reference.csv` | Reference output of unitire_simple_solve.m, generated with MATLAB R2024b. |
| `iem24_dss_suspension_rows.csv` | Suspension parameter fixture from real car data, rows 17-31, columns A-J, for CI. |
| `planar_forward_powertrain_states.npz` | NumPy archive. |
| `suspension_baseline.json` | JSON record. |

## Subdirectories

| Directory | Purpose |
|---|---|
| `iem24_mesh_sensitivity/` | Summary files for the mesh-sensitivity tests. |
| `models/` | Small XML models for the compiler tests. |
