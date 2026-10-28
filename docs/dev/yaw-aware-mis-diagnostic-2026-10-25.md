# MIS yaw-aware no-TV diagnostic run — 2026-10-25

**Branch / revision:** `feature/yaw-aware-virtual-driver` at `88ddccd` for the run-record metadata fix; the bounded planning-control exposure is `bf7d239` and the yaw-grid braking-domain correction is `c1335d1`.

**Purpose:** demonstrate the new diagnostic runner, optimized-path run record, and existing animation exporter on MIS. This is **not** an MIS 2024 simulated-vs-measured report, a minimum-time result, a torque-vectoring/FM U result, or a qualified virtual-driver result.

## Result

A planar 7DoF, physical-no-TV yaw-aware run completed one MIS lap and was exported as an MP4.

| Item | Value |
|---|---:|
| Track | `models/tracks/mis_ev_endurance_2024_width.xml` |
| Map | `physical_no_tv_v1` |
| Yaw-map grid | 7 speed × 7 longitudinal-acceleration × 13 lateral-acceleration points |
| Map speed range | 0–8 m/s |
| Planner controls | 2,000 m control spacing; 200 m path grid; one Powell iteration; one outer pass |
| Planned profile time | 125.69535597 s |
| Forward simulated time | 128.40820348 s |
| Difference (forward − plan) | +2.71284751 s |
| Forward completion | true |
| Sampled centre-point corridor violation | 0.0 m |
| Animation | 960 × 540 H.264, 30 fps, 128.433 s, 2.6 MiB |

The intentionally coarse line/planning grid makes this a **pipeline smoke test**, not an engineering-quality racing line. The saved plan reports `status: single_pass`; its retained candidate diagnostics report `optimizer_success: false`, so it must not be interpreted as an optimizer-converged line.

## Reproduction command

```sh
PYTHONPATH=src:. python3 tools/run/run_yaw_aware_forward_driver.py \
  --track models/tracks/mis_ev_endurance_2024_width.xml \
  --output-dir runs/yaw_aware_driver/mis-no-tv-animation-20261025 \
  --yaw-speed-points 7 --yaw-ax-points 7 --yaw-ay-points 13 \
  --maximum-speed 8 --lateral-margin 0.15 \
  --control-spacing 2000 --planning-grid-spacing 200 \
  --planner-iterations 1 --planner-passes 1 \
  --duration 130 --wall-seconds 180 --maximum-step 0.1 --method BDF \
  --record --record-precision float32
```

The generated run directory is gitignored; it contains `plan.json`, `summary.json`, `line.csv`, `trace.csv`, and `run_record.npz`.

Animation was exported with:

```sh
PYTHONPATH=src:. python3 tools/anim/export_animation.py \
  --record runs/yaw_aware_driver/mis-no-tv-animation-20261025/run_record.npz \
  --output runs/yaw_aware_driver/mis-no-tv-animation-20261025/mis-yaw-aware-no-tv-diagnostic.mp4 \
  --frame-rate 30 --width 960 --height 540 --progress
```

`ffmpeg` was available at `/opt/homebrew/bin/ffmpeg`. The exporter completed successfully. Matplotlib warned that its ordinary user cache was not writable and created a temporary cache; this did not affect output.

## Earlier retained failures

The following output directories are intentionally retained and empty because planning failed before an artifact could be written:

| Directory | Failure |
|---|---|
| `runs/yaw_aware_driver/mis-no-tv-preflight-20261025/` | all yaw-aware line candidates rejected |
| `runs/yaw_aware_driver/mis-no-tv-diagnostic-20261025/` | all yaw-aware line candidates rejected |
| `runs/yaw_aware_driver/mis-no-tv-diagnostic-20261025b/` | command aborted before artifact output |

The first two failures exposed the initial yaw-map acceleration-grid mismatch. A direct centreline probe found that the physical no-TV map admits the 0–8 m/s profile with an acceleration axis of −6…3 m/s²; the runner now uses that domain.

## Non-claims and remaining gates

This output does **not** establish:

- global or local minimum lap time;
- a reproducible, refined yaw-aware line or profile;
- the expected more-V-shaped TV line versus no-TV;
- FMU controller, TV, or regen-vectoring behavior;
- vehicle-footprint or between-step containment;
- a periodic-state gate, refinement tube, or full U3/virtual-driver qualification;
- comparison or validation against measured MIS telemetry.

The forward record uses the optimized line as its world geometry, while retaining the MIS XML as the named physical track for the animation scene. The FSC branch is separately blocked by [the FSC admission report](yaw-aware-fsc-admission-2026-10-25.md).