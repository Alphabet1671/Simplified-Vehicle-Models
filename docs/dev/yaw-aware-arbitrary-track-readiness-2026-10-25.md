# Yaw-aware virtual driver: arbitrary-track readiness results

**Date:** 2026-10-25/26. **Branch:** `feature/yaw-aware-virtual-driver`. Developer diagnostic, not an MIS 2024 measured-comparison report.

Status: these are diagnostic results. They do not claim minimum time, continuous feasibility, FMU-TV validation, or U3 qualification. The legacy QSS OCP code and its tests are unchanged.

## What changed

| Step | Commits | Result |
|---|---|---|
| Coupled solver refinement (Task 5) | `d817d83`, `ff47458` | Bounded directional sweeps start from the actual graph cut. The local window doubles in radius when needed. Wall-clock deadline is enforced, and candidate limits are checked at each insertion. All 9 frozen MIS/FSC windows where the legacy sweep failed now solve and pass validation. |
| Public API (Task 6) | `721494c` | `yaw_aware_profile(solver="coupled")`. Never falls back to the legacy solver. Every segment is re-validated in original station order. Coupled is the default in `YawLineSettings`. |
| Speed of the strict edge check | `ac56d59`, `ff47458` | The edge check now runs vectorised: 0 mismatches against the scalar check on 40,600 physical pairs. Exact-knot cases are pinned by tests (`179a22b`). |
| Yaw map braking bug | `baa0143` | Hydraulic brakes were forced to zero on undriven wheels while a 0.65 front brake bias was still enforced. On the rear-driven IEM24 car this left no braking cells in the map, and the coupled solver returned a constant 6.06 m/s lap. Fixed; the regression test fails without the fix. |
| Solver/validator arithmetic | `60ad6c7` | Both now compute speed and acceleration from `q = v*v` the same way, and snapping uses `s*s`. 0 disagreements on 6,000 edges lying on map knot faces. |
| Preflight | `fcca3e6` | Blocks on map holes (with their locations). Warns on near-holes, lateral-axis saturation, and curvature-rate noise. Reports only; never smooths. |
| Runner failures and provenance (Task 7) | `192900c` | `inputs.json` and `yaw_map.npz` are written before planning. A planning failure writes `planning_failure.json` with typed causes. JSON output is strict. Provenance is stored under `model['yaw_profile']` and survives a real record save/load round trip. Exit codes: 0 completed or planned, 3 incomplete lap, 4 failure summary written. |

## Full-lap fixed-geometry results (0.05 m, no explicit cap)

These solves use the zero-offset path, the anchored 12/9/17 physical no-TV map, the coupled solver, and the default `CoupledProfileSettings`.

| Track | Stations | Lap time | Speed range | Wall | Edges | `resolution_converged` |
|---|---|---|---|---|---|---|
| MIS | 20,335 | 80.05 s | 6.9–22.8 m/s | 673 s | — | False (8 rounds) |
| FSC synthetic 3 m | 15,353 | 65.79 s | 7.2–16.9 m/s | 621 s | 2.48 M | False (8 rounds) |

Each returned profile, and every graph candidate in between, passed independent validation. The best lap time never increased from one round to the next. Before the map fix, the MIS result was 167.84 s at a constant 6.06 m/s.

**Performance gate:** the plan's target is 20,000 stations in 120 s. Neither lap meets it. Most of the remaining time is in the graph solve itself (pure-Python Bellman-Ford over about 1M edges) and in repeating the solve for each closed-lap seam anchor.

## Preflight (MIS / FSC, 0.05 m)

- No map holes or near-holes on either track.
- **Lateral-axis saturation:** 79% of MIS stations and 88% of FSC stations are speed-limited by the map's ±1 g lateral-acceleration range, not by the vehicle. Cornering speeds are set by the map grid, so the map's lateral range should be widened before trusting corner speeds.
- Curvature-rate high-frequency noise is 4.3% on MIS and 3.5% on FSC, under the 30% warning threshold.

## Uncapped MIS end-to-end runner check

Planning grid 0.5 m, 2,000 m lateral controls. This is a profile-integration smoke test, not a converged racing line.

- Plan only: 76.4 s predicted, profile converged, 126 candidate profile solves, 45 min.
- Replay (BDF, 0.05 s maximum step, recorded): **lap completed**, driven 82.35 s against 76.38 s predicted. Driven speed 6.2–20.0 m/s.
- **Throttle is now used:** maximum 0.88 (versus about 0.12 under the old 8 m/s cap). Throttle ≥ 0.5 for 35% of the lap and for 75% of straight-line time (|κ| < 0.002 1/m). It never reaches ≥ 0.95, because the speed tracker's feedforward scaling and gain policy are unchanged.
- **Corridor containment failed:** the sampled centre point leaves the corridor by up to 0.198 m near station 1001.8 m. This is a separate tracking-controller result and should be investigated on its own, not fixed by weakening the profile constraints.

## Remaining work for arbitrary tracks

1. **Graph-solve performance.** Vectorise the open/closed DP and reduce seam-anchor repetition. Full laps are currently 10–11 min.
2. **Line-optimisation cost.** Each line candidate is a full profile solve. Lateral-control spacing below the 2,000 m smoke setting will need warm-started or windowed re-solves.
3. **Map lateral range.** Widen the ay axis, or derive it from tyre capacity, before quoting corner speeds.
4. **Tracking.** Containment margin and full-throttle use belong to the controller, not the planner.
5. `resolution_converged=False` on full laps at 8 rounds: refinement convergence is not demonstrated for these tracks.

## Reproduction

```sh
PYTHONPATH=src:. python3 tools/run/probe_coupled_yaw_profile.py \
  --track models/tracks/2024_michigan_endurance_closed_edged.xml \
  --output runs/yaw_profile_bench/mis-zero-0p05-coupled.json \
  --geometry zero-offset --spacing .05 --anchored --solver coupled --wall-seconds 1500

PYTHONPATH=src:. python3 tools/run/preflight_yaw_track.py \
  --track models/tracks/fsc_may_2026_synthetic_3m_edged.xml --spacing .05

PYTHONPATH=src:. python3 tools/run/run_yaw_aware_forward_driver.py \
  --track models/tracks/2024_michigan_endurance_closed_edged.xml \
  --output-dir runs/yaw_aware_driver/mis-coupled-replay-smoke \
  --yaw-speed-points 12 --yaw-ax-points 9 --yaw-ay-points 17 \
  --planning-grid-spacing .5 --control-spacing 2000 \
  --planner-iterations 1 --planner-passes 1 \
  --profile-solver coupled --profile-wall-seconds 300 \
  --duration 300 --wall-seconds 290 --maximum-step .05 --method BDF --record
```
