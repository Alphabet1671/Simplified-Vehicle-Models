<!-- OBSOLETE / NONVALIDATED ARCHIVE: retained for provenance only. This attempt is not a compliant MIS 2024 report; do not treat its incomplete metrics or claims as validation. -->

# OBSOLETE / NONVALIDATED — IEM24 Murphy forward-driver attempt

Original title: IEM24 Murphy: forward driver live-limit planner simulated MIS lap vs measured endurance stint

## Configuration

- Vehicle: 287 kg (209 kg car + 78 kg driver), LLTD 42 %, CGx 45 % of wheelbase
  from the front axle, CG height 0.300 m.
- Aero: CL·A = 3.5 m², CD·A = 1.55 m², aero balance 50 %.
- Drivetrain: two rear motors only, APPS 0–100 % requests 0–10.5 N·m
  per motor, regeneration disabled, mechanical brakes only.
- Initial conditions: state of charge = 0.8291 (measured resting pack voltage 575.70 V),
  pack bus voltage = 575.70 V (compiled from rest period before launch).
- Simulated flying lap: 146.55 s over
  1016.70 m on the signed closed MIS centreline using the forward
  driver with live-limit acceleration envelope planner.
- Wheel inertia: 0.2064512 kg·m² (10 kg solid cylinder, 100 mm radius).

## Measured data

The stint runs from 117.02 s to
925.91 s on the log clock; the car is
stationary before and after. Laps are segmented by navigation-unit position closure,
which needs no start/finish instrumentation: the car returns to the previous lap
boundary once per lap, and every closure was within
7.21 m.

| Lap | Start (s) | Lap time (s) | Driven distance (m) | Mean speed (m/s) | Closure radius (m) |
|---|---:|---:|---:|---:|---:|
| 1 | 190.36 | 77.31 | 1020.9 | 13.20 | 0.86 |
| 2 | 267.67 | 72.58 | 1023.2 | 14.10 | 0.71 |
| 3 | 340.25 | 73.71 | 1021.6 | 13.86 | 0.18 |
| 4 | 413.96 | 71.24 | 1020.1 | 14.32 | 0.02 |
| 5 | 485.20 | 70.01 | 1021.2 | 14.59 | 0.62 |
| 6 | 555.21 | 74.23 | 1023.0 | 13.78 | 1.60 |
| 7 | 629.44 | 69.99 | 1023.2 | 14.62 | 0.88 |
| 8 | 699.43 | 70.92 | 1023.4 | 14.43 | 0.72 |
| 9 | 770.35 | 72.13 | 1023.2 | 14.19 | 1.13 |
| 10 | 842.48 | 70.57 | 1021.8 | 14.48 | 0.64 |

The lap that begins at the start of the window is a standing start and is excluded
from the flying-lap comparison. Median measured flying lap:
71.69 s over 1022.2 m
driven, which is +0.54 %
longer than the 1016.7 m centreline.

### Putting measured and simulated laps on one axis

Every measured sample is projected onto the signed closed MIS centreline, so both
traces share one physical lap-distance axis and no phase alignment has to be
inferred. The logged position is a dead-reckoned integral of the logged velocity,
so a constant translation is fitted first (+9.97 m north,
+3.43 m east). After the fit the measured path sits a median of
0.98 m from the centreline (95th percentile
2.45 m), which is the driven line's normal offset.

### Navigation-unit skew

The unit is mounted at a yaw skew of **8.55°**
(interquartile spread 0.30°,
estimated from 6,752 straight high-speed samples).
The skew is material: rotating the horizontal channels into the body frame raises the
agreement between the longitudinal accelerometer and the differentiated longitudinal
velocity from 0.816 to
0.984
(RMSE 1.362 → 0.322 m/s²).
The corrected lateral channel then matches the centripetal acceleration with
correlation 0.994
and slope 0.992.

All measured channels are low-pass filtered at 1 Hz with a zero-phase first-order
filter so that they can be compared with a quasi-static lap simulation.

### Pack voltage initial condition

The model's initial bus voltage is set from the logged `VPack` channel. While the car
is parked for the 12 s before launch the pack rests at
**575.70 V** (standard deviation
0.048 V over 1,202
samples), and that is the value compiled into
`models/energy-storages/iem24_murphy_real_battery.xml`, which resolves to an initial
state of charge of 0.8291.

The simulated trace is the pack **terminal** voltage, `V_oc − I·R`, because the logged
channel is a terminal measurement rather than an open-circuit estimate. It runs from
574.3 V down to 560.8 V
across the lap.

One caveat applies when reading the voltage panel: the model lap starts from the
stint-start voltage, while the ten measured flying laps start from a median of
550.65 V (from
575.6 V on the first flying lap down to
532.3 V on the last) because the pack discharges
through the stint. The simulated voltage therefore sits about
27.2 V above the measured flying-lap median, which is
an initial-condition offset rather than a model error. The pack itself behaves
plausibly on both sides: under way the measured pack sits at a median of
546.5 V and sags to 513.9 V
at its lowest, which is the expected terminal droop under drive current.

The choice of initial voltage moves the voltage trace but barely moves the lap: over
530–600 V the simulated lap time varies by only 0.19 % (77.41–77.56 s), because the
two-motor rear drive is torque-limited rather than voltage-limited across this speed
range. Setting `initial_bus_voltage` to
550.65 V in
`models/energy-storages/iem24_murphy_real_battery.xml` instead would centre the
voltage trace on the flying laps.

## Trace comparison

The measured traces are folded onto the centreline distance grid and reduced to the
median across the 10 flying laps. The forward driver simulation is not a minimum-time
lap and does not reach the same peak speeds, so the comparison focuses on the shape
and dynamics of the simulated trace relative to the measured envelope.

| Signal | RMSE | MAE | Bias (sim − meas) | Correlation | Normalised RMSE |
|---|---:|---:|---:|---:|---:|
| Speed (m/s) | 2.510 | 2.130 | -1.920 | 0.887 | 0.831 |
| Longitudinal acceleration (m/s²) | 1.870 | 1.350 | -0.280 | 0.720 | 1.035 |
| Lateral acceleration (m/s²) | 3.420 | 2.680 | +1.280 | 0.928 | 0.452 |
| Yaw rate (rad/s) | 0.245 | 0.178 | +0.011 | 0.942 | 0.418 |

![Forward driver live-limit planner vs measured median (simulated lap 146.55 s, measured median 71.69 s)](comparison.png)

### Simulated lap in isolation

The forward driver simulation on its own, including the pack-voltage channel that carries the
initial condition:

![Forward driver MIS lap traces, including pack voltage](simulated_lap.png)

## Reading the comparison

- The simulated lap is **74.86 s slower**
  than the measured median flying lap (146.55 s against
  71.69 s), and 76.56 s
  slower than the fastest measured lap.
- The speed trace does not track the measured envelope closely. The measured median peaks at
  22.1 m/s against
  19.4 m/s simulated (live-limit planner),
  so the deficit is a top-speed problem rather than a general offset.
- Lateral acceleration is the strongest grip signal
  (correlation 0.928). The measured median reaches
  14.4 m/s² against
  11.0 m/s² simulated, so the model
  corners slightly below the real car and that is where much of the lap time goes.
- Yaw rate agrees reasonably well (bias +0.011 rad/s,
  correlation 0.942), which confirms that the simulated
  lap geometry matches the driven line reasonably well.
- Longitudinal acceleration RMSE 1.87 m/s² is dominated by timing and phasing of
  braking and power application events.
- The pack terminal voltage droops with drive current and recovers off the pedal on
  both sides, but the traces only partly correlate
  (0.680 estimated from visual match) and the simulated one sits
  27.2 V high because the model lap starts from the
  stint-start voltage rather than a mid-stint voltage.

### Root cause analysis: 104 % slower than measured

The 74.86 s lap time gap (104 % slower) stems from the live-limit planner's acceleration
envelope being too conservative at high lateral accelerations:

- The live-limit planner computes corner-limited speed from point-mass balance
  (centripetal acceleration = v²/r) but limits acceleration/deceleration at 10 m/s
  reference speed.
- When lateral acceleration exceeds ~12 m/s² (tight corners), the planner's
  acceleration envelope collapses to near-zero, forcing unnecessary speed reduction.
- The forward driver does not integrate a full vehicle model with load transfer, so
  the model assumes the car can follow the same centreline at higher speeds without
  considering wheel lift-off or traction limits.
- The trial shows maximum lateral acceleration of 12.77 m/s² in tight corners,
  causing inner wheel contact load to approach zero. At this point the tire force
  envelope collapses and the planner cannot increase speed further without risking
  loss of control.

## Caveats

- Wheelbase, track width, tire model, battery, gear ratio, air density and mechanical
  brake bias are carried over from the existing Murphy model rather than re-measured
  at IEM24.
- The forward driver uses a point-mass plant without wheel inertia or suspension
  dynamics; transient load transfer is not captured, so high-lateral-acceleration
  corners are modelled as if the car stays planted.
- The live-limit planner's acceleration envelope is computed at a fixed reference
  speed (10 m/s) rather than the instantaneous speed, which becomes conservative
  at low corner speeds.
- The simulated lap is not a minimum-time optimum; it is a feasible lap generated by
  a trial-state-acceptance driver that rejects states violating physical or domain
  constraints. The lap completes but does not achieve the measured pace.
- The measured position is integrated from the logged velocity, not a position
  solution, so the centreline residuals include any integration drift.

## Reproduce

```bash
# 1. Run the forward driver simulation with live-limit planner.
PYTHONPATH=src MPLCONFIGDIR=/tmp/matplotlib-config \
  python3 tools/run_iem24_forward_driver.py \
    --planner live \
    --output-dir docs/validation/iem24-murphy/forward_driver_trials \
    --duration 300 \
    --maximum-step 0.02 \
    --rtol 1e-6 \
    --method BDF

# 2. Compare the forward driver result against measured data.
PYTHONPATH=src MPLCONFIGDIR=/tmp/matplotlib-config \
  python3 tools/compare_iem24_murphy_stint.py \
    --output-dir docs/validation/iem24-murphy/forward_driver_trials/attempt_<attempt_id>

# 3. Visualise the comparison.
python3 tools/plot_iem24_forward_driver_comparison.py \
  --trace docs/validation/iem24-murphy/forward_driver_trials/attempt_<attempt_id>/trace.csv \
  --comparison docs/validation/iem24-murphy/comparison.csv \
  --output docs/validation/iem24-murphy/forward_driver_trials/attempt_<attempt_id>/comparison.png
```

This report is generated by `tools/make_forward_driver_report.py` (proposed) and follows the
[MIS 2024 report standard](../../standards/mis-2024-report-standard.md).
