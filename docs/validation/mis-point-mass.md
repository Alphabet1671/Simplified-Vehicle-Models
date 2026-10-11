# MIS Point-Mass Validation

## Scope

This validation uses the unchanged legacy Murphy vehicle, tire, aero, and drivetrain values. It uses the `2024 Michigan Endurance V11` distance-radius profile. The calculation does not tune parameters to match a target lap time.

The source track has 1,407 nodes and 1,406 physical segments. Its represented lap length is 1,014.32575 m. The loader excludes the duplicated terminal `stepD` value.

## Results

| Boundary condition | Lap time | Average speed | Maximum speed | Iterations |
|---|---:|---:|---:|---:|
| Flying lap | 93.7956 s | 10.8142 m/s | 19.4278 m/s | 2 |
| Standing start | 94.8431 s | 10.6948 m/s | 19.4278 m/s | 2 |

The standing-start penalty is 1.0475 s. Both calculations met the `1e-5 m/s` convergence criterion. The final maximum speed change was `3.45e-9 m/s`.

## Checks

- Both lap times are finite and between 50 s and 100 s.
- The standing-start time is greater than the flying-lap time.
- The standing-start penalty is less than 15 s.
- The flying-lap boundary speeds are equal.
- The standing-start initial speed is zero.
- All speeds are below the motor-speed limit.
- All segment transitions satisfy the calculated live acceleration or braking limit.
- All segment states satisfy the combined-force lateral feasibility check.

## Legacy comparison

The measured Lap 8 trace spans approximately 70.30 s. A nearby legacy correlation script cites a 69.98 s fastest-lap reference and a 71.696 s median-lap reference. The legacy event workbook contains average-lap values from approximately 67.667 s to 94.253 s.

The flying-lap result is inside the broad event range and near its slow boundary. It is 23.50 s slower than the measured Lap 8 trace. The standing-start result is 0.59 s above the broad event range. The standing-start result is not directly comparable to flying event laps.

The difference from the measured trace is plausible for this first model. The model uses a load-sensitive friction ellipse instead of a correlated transient tire model. The path contains unsigned radius only, so it does not contain turn direction, banking, or elevation. The segment calculation also uses the larger curvature at adjacent nodes to keep every transition feasible. These choices make the calculation conservative.

The event values are reference observations. They are not outputs from the unfinished `lap-sim-2025` MATLAB solver. A future correlation step must use measured controls and a documented parameter-identification method. It must not tune parameters only to hit a lap-time target.

## Reproduction

Run these commands from the repository root:

```bash
PYTHONPATH=src python -m unittest discover -s tests -v
PYTHONPATH=src python examples/run_mis_point_mass.py
```
