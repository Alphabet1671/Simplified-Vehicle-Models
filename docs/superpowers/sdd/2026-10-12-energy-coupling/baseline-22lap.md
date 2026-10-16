# 22-Lap Endurance Baseline (truncated at 11 laps by user instruction)

Status: **baseline for the energy-coupling redesign comparison.** The run was
stopped after 11 completed laps on user instruction ("the few laps it already
completed is enough for comparison"). The remaining 11 laps were NOT measured.

## Configuration

| Field | Value |
|---|---|
| Revision | `a8d7bc2` (branch `feature/powertrain`, pushed as `1317110` + scope doc) |
| Track | `models/tracks/2024_michigan_endurance_v11.xml` (1,014.33 m, 1,407 nodes, 1,406 segments) |
| Vehicle | `models/formulations/murphy_point_mass.xml` |
| Motor mode | `lut` (default) |
| Initial SOC | 0.9617234998917527 (600.0 V OCV regulation start) |
| Regen | disabled |
| `reuse_profile` | False (every lap fully state-coupled) |
| Cache | exact keys (`DriveCachePolicy()`) |
| Tolerances | energy 1e-5 (max 50 iter), lap 1e-6 (max 200), root 1e-8 |
| Timing script | `/tmp/time_endurance_22_a8d7bc2.py` (no longer present in repo; recreate from this table) |
| Machine | 8 cores; the run was effectively uncontended (only light grep/read activity alongside) |

## Measured wall time

Every completed lap reported `energy_iterations=13`, `profile_iterations=2`
(15 full `solve_lap` calls per lap).

| Lap | Lap wall (s) | Cumulative (s) |
|---:|---:|---:|
| 1 | 160.774 | 160.774 |
| 2 | 233.327 | 394.101 |
| 3 | 187.117 | 581.218 |
| 4 | 193.349 | 774.567 |
| 5 | 199.032 | 973.599 |
| 6 | 169.506 | 1143.105 |
| 7 | 248.309 | 1391.414 |
| 8 | 381.251 | 1772.665 |
| 9 | 381.249 | 2153.914 |
| 10 | 407.669 | 2561.583 |
| 11 | 411.129 | 2972.712 |

Cumulative through lap 11: **2,972.712 s = 49.5 min.**

## The finding that motivates the redesign

Per-lap wall time more than doubles across the stint at **identical iteration
counts** (13 + 2 every lap): lap 6 took 169.5 s while laps 8-11 took
381-411 s. Iteration count cannot explain this; per-pass cost rising as SOC
falls can. The leading hypothesis (scope D3) is that terminal voltage falls
below the LUT's 530 V axis floor, forcing the exact analytical fallback in the
speed solve, and that `_energy_trace` runs on the analytical oracle rather than
the LUT regardless of SOC.

## Extrapolation (NOT a measurement)

Laps 8-11 average 395.3 s. If the remaining 11 laps continued in the 400-500 s
range, the full 22-lap stint would have taken roughly 7,400-8,500 s
(2.0-2.4 h). This figure is an extrapolation for sizing only; it must not be
quoted as a measured result.

## Data lost on truncation (disclosure)

The script accumulated per-lap `lap_time_s`, final SOC, minimum/maximum bus
voltage, consumed energy and pack-discharge checks in memory and wrote the JSON
report only on completion, so those per-lap details were lost when the process
was killed. The wall times and iteration counts above come from the preserved
stdout progress lines. Nearest measured reference for simulated lap time: the
uncoupled flying lap is 69.1917773155225 s (LUT default, same revision).

**Lesson applied to W0:** diagnostic runs must flush their per-iteration and
per-lap records incrementally, not only at completion.
