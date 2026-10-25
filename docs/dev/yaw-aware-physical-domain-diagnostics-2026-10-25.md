# No-explicit-speed-cap yaw-aware diagnostics — 2026-10-25

## Change

The yaw-aware runner no longer defaults to `--maximum-speed 7` or `8`. With that argument omitted it derives its finite map upper domain from the assembled vehicle’s physical zero-curvature, positive-drive straight-line envelope, bounded by the drivetrain motor-speed limit. An explicit `--maximum-speed` remains an opt-in limiter for bounded diagnostic studies.

For the compiled IEM24 planar vehicle, the no-explicit-cap calculation selected:

| Quantity | Value |
|---|---:|
| No-load motor-speed ceiling at initial SOC | 35.067453 m/s |
| Positive-drive straight-line envelope endpoint | **33.316187 m/s** |
| Runner map-domain source | `physical_straight_line_envelope` |

This is not an arbitrary user-selected velocity ceiling. It remains a finite numerical map domain and changes with vehicle/powertrain/SOC modelling; it is not a proof of an achieved on-track speed.

## Commands attempted

Both diagnostics intentionally omit `--maximum-speed`.

```sh
PYTHONPATH=src:. python3 tools/run/run_yaw_aware_forward_driver.py \
  --track models/tracks/2024_michigan_endurance_closed_edged.xml \
  --output-dir runs/yaw_aware_driver/mis-physical-domain-no-tv-20261025 \
  --yaw-speed-points 12 --yaw-ax-points 9 --yaw-ay-points 17 \
  --lateral-margin 0.15 --control-spacing 2000 --planning-grid-spacing 0.05 \
  --planner-iterations 1 --planner-passes 1 --duration 300 --wall-seconds 290 \
  --maximum-step 0.05 --method BDF --record --record-precision float32
```

```sh
PYTHONPATH=src:. python3 tools/run/run_yaw_aware_forward_driver.py \
  --track models/tracks/fsc_may_2026_synthetic_3m_edged.xml \
  --output-dir runs/yaw_aware_driver/fsc-synthetic-3m-physical-domain-no-tv-20261025 \
  --yaw-speed-points 12 --yaw-ax-points 9 --yaw-ay-points 17 \
  --lateral-margin 0.15 --control-spacing 2000 --planning-grid-spacing 0.05 \
  --planner-iterations 1 --planner-passes 1 --duration 300 --wall-seconds 290 \
  --maximum-step 0.05 --method BDF --record --record-precision float32
```

## Outcome

Neither track reached a forward simulation. Both failed during yaw-aware line/profile candidate admission with:

```text
error: all yaw-aware line candidates rejected
```

The fresh ignored output directories are retained but empty because the runner does not write plan or record artifacts until it has an admissible `YawAwareLinePlan`.

| Track | No explicit CLI speed cap | Result |
|---|---|---|
| MIS edged corridor | yes | planning/profile rejected all candidates |
| FSC synthetic 3 m corridor | yes | planning/profile rejected all candidates |

A direct MIS centreline profile probe at 0.05 m spacing identifies the immediate mechanism: near a curvature magnitude of about 0.119 1/m, the map’s station cap is about 9.1 m/s. On the fine grid, the current independent forward/backward sampled transition logic fails to create a mutually feasible preceding deceleration transition. This is a profile-algorithm limitation, not a restoration of the old 7/8 m/s runner cap.

## Interpretation

The previous 7 m/s FSC and 8 m/s MIS results are still useful **bounded smoke tests**, but they are no longer described as a default runner limit. The no-explicit-cap test demonstrates that the physical map domain is now derived correctly, and also exposes the next real solver requirement: a robust coupled high-resolution forward/backward transition algorithm across rapidly changing curvature, including at the high-speed corners admitted by the physical map.

No MP4 was generated for these no-explicit-cap attempts because no run record exists. The earlier animation artifacts remain available only for the explicitly bounded diagnostics:

- [MIS bounded diagnostic](yaw-aware-mis-diagnostic-2026-10-25.md)
- [FSC synthetic bounded diagnostic](yaw-aware-fsc-synthetic-diagnostic-2026-10-25.md)

The FSC geometry remains the synthetic assumed 3 m total-width corridor documented in [the FSC corridor provenance note](fsc-synthetic-corridor-2026-10-25.md), not surveyed edges.
