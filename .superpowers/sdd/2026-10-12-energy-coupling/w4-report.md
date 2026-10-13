# W4 Report — LUT Voltage-Axis Extension

Date: 2026-10-12/13 · Branch `feature/energy-coupling` · Worktree `.worktrees/point-mass`

**Provenance note:** A Ling 3.0 Flash Fin agent landed commits `d3c2f04`
(regeneration) and `0e1abca` (XML/provenance/digest pin), then died mid-way
through test updates with a garbled return (its second failure on this
workstream). The controller completed the remaining work directly: test
recalibration (`72df863`), the flying-lap re-pin, this report, and all
validation measurements below.

## Measured floor

Coupled (loaded) minimum bus voltage seen by the speed solve, swept over
0-32 m/s through `maximum_drive_force` per SOC:

| SOC | Min coupled bus voltage |
|---|---|
| 0.5 | 488.0 V |
| 0.3 | 474.9 V |
| 0.2 | 469.8 V |
| 0.1 | 452.4 V |
| 0.05 | 392.9 V |
| 0.02 | 333.6 V |

**Chosen floor: 400 V.** A 22-lap endurance stint starting at SOC 0.9617 ends
near SOC 0.28 (usable pack energy ≈ 26.3 MJ vs ≈ 18 MJ consumed), where the
minimum coupled voltage is ≈ 470 V; 400 V covers that with ≈ 15 % margin.
SOC below ≈ 0.1 measures below the floor but is unreachable in a 22-lap stint
(would need ≈ 35 laps), and 400 V stays above the pack OCV at SOC 0
(448.8 V). The floor does NOT cover sub-0.1 SOC operation; the exact
analytical fallback handles those queries, which is the designed behavior.

## Artifact

| Field | Value |
|---|---|
| Axis | 400–600 V × 0–18,600 rpm × 65 loads (25×129×65, unchanged grid) |
| Generator | `motor-lut-v5` (unchanged; the axis is a CLI parameter) |
| SHA-256 | `79a615bb836054818aecc45cd6c7a7d5bdb02967c9dea8d16a0eb26a93559448` |
| File size | 2,606,883 bytes |
| Reproducibility | Two regenerations byte-identical (controller-verified with `cmp`) |
| Invariants | 0 drive monotonic violations; 0 envelope reductions; 55 cliff cells routed to the exact oracle; 0 near-zero drive nodes with nonzero cap |

The per-volt interpolation step coarsens from 2.92 V/pt (530-600) to
8.33 V/pt (400-600) at 25 points. Measured consequences: low-speed envelope
error 0.176 Nm (400 V / 62.5 rpm, was 0.1359 Nm), high-sweep regen error
0.050 Nm (540 V / 11,790 rpm). Raising `--voltage-points` was considered and
rejected: all gates below pass at 25 points, and the earlier provenance
history shows finer grids buy < 0.04 ms.

## LUT vs exact in the new 400-530 V band

Forward `evaluate_motor` vs `evaluate_shaft_power_exact` over 25 voltages ×
60 speeds × 5 signed torques (cap-respecting): **p95 error 3.0 W, max
135 W** — consistent with the old band's accepted fidelity (existing test
asserts p95 < 120 W over in-domain samples).

## Gate (d): flying-lap compare (`--mode compare --runs 3`, alone)

| Gate | Measurement | Result |
|---|---|---|
| Lap delta ≤ 1 ms | +0.785 ms | PASS |
| Max speed displacement ≤ 0.05 m/s | 0.0222 m/s | PASS |
| Both converged | yes / yes | PASS |
| LUT faster | analytical 16.00 s vs LUT 5.33 s median | PASS (3.00×) |

## Gate (e): full-track acceptance vs W0 references

One-lap energy solves, `regen_enabled=False`, default tolerances:

| SOC | Wall (W0) | Wall (W4) | Iterations | lap Δ | SOC Δ | energy Δ | Verdict |
|---|---|---|---|---|---|---|---|
| 0.9617 | 182.5 s | **39.7 s** | 13 → 3 | 0.0021 ms | 2.6e-8 | 0.0001 % | PASS (4.6×) |
| 0.5 | 894.8 s | **25.4 s** | 13 → 2 | 0.0107 ms | 2.1e-7 | 0.0006 % | PASS (**35×**) |

No acceptance re-pinning was needed; the W1 acceptance tests pass as
pinned. The one legitimate pin change is the LUT flying lap:
69.1917773155225 → 69.19177483621608 s (**2.5 µs**), because speed-solve
coupled voltages below the old 530 V floor are now answered by in-domain
interpolation instead of exact fallback. The analytical reference pin is
unchanged.

## Test-suite changes (`72df863`)

- Out-of-domain probes 400-450 V → 380 V (below the new floor); the caching
  test uses 370 V to avoid a module-level `_EXACT_LIMIT_CACHE` key collision
  with sibling tests (a reused key would make its first call a cache hit with
  zero evaluations).
- Near-zero-drive-node test rewritten per voltage column: the envelope corner
  moves per column (13,333 rpm at 400 V vs 20,000 rpm at 600 V), so a fixed
  17,000 rpm cut would flag legitimate zero-torque nodes.
- Cliff-cell test moved to (0, 91), the new map's first straddling cell.
- Two-sided envelope slacks re-measured: 0.20 Nm below 300 rpm in the
  400-530 V band, 0.06 Nm regen in the high sweep.

## Full suite

**293 tests OK** (controller-verified twice: 152 s and 168 s runs).

## Commands

```text
PYTHONPATH=src python3 tools/generate_motor_lut.py models/power-plants/amk_dd5_100c.xml models/data/amk_dd5_100c_operating_map.npz --voltage-points 25 --speed-points 129 --load-points 65 --voltage-min 400 --voltage-max 600
PYTHONPATH=src python3 examples/benchmark_soc_solver.py --mode compare --runs 3
PYTHONPATH=src python3 -m unittest discover -s tests -q
```

## Deviations

1. The floor justification in the committed provenance row cites "the
   observed minimum at SOC 0.2"; the honest framing is that the floor covers
   the *stint-reachable* range (ends ≈ SOC 0.28) and that SOC < 0.1 measures
   below the floor by design (exact fallback covers it). The controller
   verified the 400 V choice against the stint endpoint, not just SOC 0.2.
2. The controller, not a subagent, completed the test recalibration and
   validation after two Ling failures on this workstream.