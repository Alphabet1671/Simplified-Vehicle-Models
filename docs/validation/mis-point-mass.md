# MIS Point-Mass Validation

## Scope

This validation uses the unchanged legacy Murphy vehicle, tire, aero, and drivetrain values. It uses the `2024 Michigan Endurance V11` distance-radius profile. The calculation does not tune parameters to match a target lap time.

The source track has 1,407 nodes and 1,406 physical segments. Its represented lap length is 1,014.32575 m. The loader excludes the duplicated terminal `stepD` value.

## Results

| Boundary condition | Lap time | Average speed | Maximum speed | Iterations |
|---|---:|---:|---:|---:|
| Flying lap | 69.2265 s | 14.6523 m/s | 29.5823 m/s | 2 |
| Standing start | 70.3896 s | 14.4102 m/s | 29.5823 m/s | 2 |

The standing-start penalty is 1.1630 s. Both calculations met the `1e-5 m/s` convergence criterion. The final maximum speed change was `1.0e-9 m/s`.

## Checks

- Both lap times are finite and between 50 s and 100 s.
- The standing-start time is greater than the flying-lap time.
- The standing-start penalty is less than 15 s.
- The flying-lap boundary speeds are equal.
- The standing-start initial speed is zero.
- All speeds are below the motor-speed limit.
- The lateral friction stays between 1.0 and 1.4 over corner loads from 700 N to 1200 N.
- The longitudinal friction stays between 0.5 and 0.8 over corner loads from 700 N to 1200 N.
- Independent test calculations close longitudinal force balance to `1e-8 N`.
- Independent test calculations close yaw-moment balance to `1e-8 N*m`.
- All wheel contact loads are nonnegative.
- Independent per-wheel combined utilization is at most one within `1e-7`.
- Replayed throttle and brake commands reproduce the required wheel force to `1e-8 N`.

## Tire unit correction

The first version of this model evaluated the tire load-sensitivity coefficients against newton loads. The legacy MATLAB fit uses pound-force loads. The header of `calcTireLoad.m` states the unit, and `maxTireLongBrake.m` labels its results `%lbf`.

The model now converts the coefficient:

```text
source value        = -0.0018 per lbf
conversion          = divide by 4.4482216 N/lbf
converted value     = -0.000404656098 per N
```

The conversion raises the lateral friction from about 0.78 to about 1.24 at a 700 N corner load. It raises the corner speeds by about 20 percent and shortens the flying lap from 95.2064 s to 69.2265 s.

A regression test now requires the lateral friction to stay between 1.0 and 1.4 and the longitudinal friction to stay between 0.5 and 0.8 over corner loads from 700 N to 1200 N. The test prevents a silent unit regression.

## Legacy comparison

The measured Lap 8 trace spans approximately 70.30 s. A nearby legacy correlation script cites a 69.98 s fastest-lap reference and a 71.696 s median-lap reference. The legacy event workbook contains average-lap values from approximately 67.667 s to 94.253 s.

The flying-lap result of 69.2265 s is **1.07 s faster than the measured Lap 8 trace** and **0.75 s faster than the cited 69.98 s fastest-lap reference**. It differs from the measured trace by 1.5 percent. The standing-start result of 70.3896 s is 0.09 s above the measured flying trace. The standing-start result is not directly comparable to flying event laps.

The agreement is close enough that the remaining differences are the quantities worth examining. The model is slightly faster than the measured lap for these reasons:

- the path contains unsigned radius only, so it has no banking, elevation, or turn direction;
- the calculation is quasi steady-state, so it has no tire transient or suspension response;
- the calculation assumes a perfect driver on the limit at every point.

The model is slightly conservative in two other ways: it uses the larger curvature of each pair of adjacent nodes, and it uses a steady-state friction ellipse instead of the correlated transient UniTire model.

The event values are reference observations. They are not outputs from the unfinished `lap-sim-2025` MATLAB solver. A future correlation step must use measured controls and a documented parameter-identification method. It must not tune parameters only to hit a lap-time target.

## Reproduction

Run these commands from the repository root:

```bash
PYTHONPATH=src python -m unittest discover -s tests -v
PYTHONPATH=src python examples/run_mis_point_mass.py
```
