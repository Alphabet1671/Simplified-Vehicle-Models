# FSC synthetic-3 m yaw-aware no-TV diagnostic — 2026-10-25

**Track identity:** [fsc_may_endurance_2026_width.xml](../../models/tracks/fsc_may_endurance_2026_width.xml). The centreline is the copied original FSC one-dimensional reconstruction; the corridor is a **synthetic assumed 3.0 m total width** (1.5 m left and 1.5 m right), not surveyed course boundaries. Read [the FSC corridor provenance note](fsc-synthetic-corridor-2026-10-25.md) before interpreting clearance.

## Result

A no-TV yaw-aware plan was generated at **0.05 m physical planning-grid spacing** and replayed through the planar 7DoF forward model. The forward integration completed a lap and an MP4 animation was exported. The run is diagnostic only and **failed the sampled centre-point containment check**, so it is not an accepted driver result.

| Item | Result |
|---|---:|
| Centreline lap length | 768.295854 m |
| Corridor | synthetic 3.0 m total / 1.5 m per side |
| Map | `physical_no_tv_v1` |
| Map speed domain | 0–7 m/s |
| Yaw map grid | 7 speed × 7 longitudinal acceleration × 13 lateral acceleration |
| Physical planning-grid spacing | **0.05 m** |
| Profile samples | 15,367 |
| Line control spacing | 2,000 m |
| Outer passes / per-complex iterations | 1 / 1 |
| Plan status | `single_pass` |
| Predicted profile time | 108.653973 s |
| Driven time | 110.704850 s |
| Completed lap | true |
| Maximum sampled centre-point corridor violation | **0.050087 m** |
| Sampled centre-point contained | **false** |
| MP4 | 960 × 540, H.264, 30 fps, 110.733 s, 3.9 MiB |

The coarse lateral-control/optimization settings were deliberately used only to make a fine-stationing pipeline exercise tractable. They are not a credible finalized racing line, and the observed boundary violation means this result must not be used as a path-clearance demonstration.

## Exact command

```sh
PYTHONPATH=src:. python3 tools/run/run_yaw_aware_forward_driver.py \
  --track models/tracks/fsc_may_endurance_2026_width.xml \
  --output-dir runs/yaw_aware_driver/fsc-synthetic-3m-no-tv-20261025b \
  --yaw-speed-points 7 --yaw-ax-points 7 --yaw-ay-points 13 \
  --maximum-speed 7 --lateral-margin 0.15 \
  --control-spacing 2000 --planning-grid-spacing 0.05 \
  --planner-iterations 1 --planner-passes 1 \
  --duration 120 --wall-seconds 300 --maximum-step 0.05 --method BDF \
  --record --record-precision float32
```

The ignored run directory contains `line.csv`, `plan.json`, `summary.json`, `trace.csv`, and `run_record.npz`; the record has 6,420 accepted rows. Its manifest includes model identity, four input-source hashes (formulation, track XML, centreline CSV, width CSV), map grid/identity, planning settings, and integrator settings.

The associated animation command was:

```sh
PYTHONPATH=src:. python3 tools/anim/export_animation.py \
  --record runs/yaw_aware_driver/fsc-synthetic-3m-no-tv-20261025b/run_record.npz \
  --output runs/yaw_aware_driver/fsc-synthetic-3m-no-tv-20261025b/fsc-synthetic-3m-yaw-aware-no-tv.mp4 \
  --frame-rate 30 --width 960 --height 540 --progress
```

## Resolution and map limitations

The 0.05 m profile previously exposed a discrete-transition scan failure: a uniform 80-point speed scan can skip the tiny endpoint-speed changes implied by a fine `ds`. [The profile repair](../../src/simplified_vehicle_models/solvers/yaw_aware_profile.py) now adds energy-projected candidates derived from the longitudinal-acceleration map axis and bounds station speed by the lateral-acceleration map domain. This makes the 0.05 m, 7 m/s FSC diagnostic runnable.

At an 8 m/s map/profile cap, the current strict sampled profile still has FSC transition-feasibility failures near rapidly changing curvature. The final diagnostic therefore uses a 7 m/s cap; it is not evidence that 7 m/s is physically optimal or that 8 m/s is physically impossible. A future refinement must use a more robust coupled forward/backward transition feasibility algorithm before raising that cap.

## Straight-line throttle investigation

The earlier MIS diagnostic showed little throttle because it was explicitly planned and map-capped at 8 m/s. `PreviewDriver` is a speed tracker, with requested throttle:

```text
clip((planned longitudinal feedforward + speed_gain × (target_speed − measured_speed)) / 6, 0, 1)
```

On a straight already at its capped target with zero feedforward, its correct command is zero throttle. At 7.65 m/s against an 8 m/s target and the default gain of 2/s, it requests roughly 0.12 throttle, matching the recorded MIS maximum. This is not a verified powertrain/full-throttle defect: it is a speed-envelope/controller-policy limitation. Full throttle would deliberately overshoot the fixed profile unless a higher feasible speed envelope is planned.

## Non-claims and required next work

This output does not establish minimum time, a converged line optimization, torque-vectoring/FM U performance, real FSC edge clearance, footprint/between-step containment, periodicity, refinement, U3 qualification, or measured-track validation.

Before an accepted FSC result:

1. replace synthetic width with admitted/surveyed edge geometry;
2. use meaningful lateral-control spacing and multiple converged optimization passes;
3. repair the high-cap coupled-profile transition solver rather than lowering the cap as a workaround;
4. tune/replay the tracking controller until sampled containment passes, then add footprint and between-step evidence.
