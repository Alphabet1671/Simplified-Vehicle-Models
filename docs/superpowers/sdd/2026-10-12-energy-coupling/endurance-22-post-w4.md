# 22-Lap Endurance Stint — Post-Redesign Measurement (W6)

Date: 2026-10-13 · Revision `1f987e3` (branch `feature/energy-coupling`) ·
Machine: 8-core, uncontended (stint was the only compute during measurement).

## Configuration

| Field | Value |
|---|---|
| Track | Michigan endurance v11 (1,014.33 m, 1,407 nodes) |
| Laps | 22, `LapMode.FLYING`, no profile reuse (every lap fully state-coupled) |
| Motor mode | `lut` (default), canonical map 400-600 V (`motor-lut-v5`) |
| Initial SOC | 0.961723499892 (600.0 V OCV regulation start) |
| Regen | disabled · cache keys exact (`DriveCachePolicy()`) |
| Tolerances | energy 1e-5 (max 50), lap 1e-6 (max 200), root 1e-8 |
| Script | `/tmp/time_endurance_22_post_w4.py` (JSON-lines flushed per lap) |

## Result

| Metric | Value |
|---|---|
| **Wall time (22 laps)** | **960.0 s = 16.0 minutes** |
| Simulated duration | 1,520.2 s (25.3 min of racing) |
| Lap times | 69.023 - 69.187 s |
| Energy iterations per lap | 3 (SOC > ~0.5) → 2 (late stint) |
| Laps converged | 22 / 22 |
| Energy loops converged | 22 / 22 |
| Max state-profile residual | 8.3e-10 |
| Final SOC | 0.2842 |
| Consumed energy | 18.07 MJ |
| Bus voltage range | 516.95 - 600.0 V (all in the 400-600 V LUT domain) |
| Pack-discharge violations | **0** (max excess 0.0 W) |
| Per-lap wall | 39.9 - 45.1 s, **flat across the stint** |

## Comparison against the pre-redesign baseline

Baseline (recorded in `baseline-22lap.md`, revision `a8d7bc2`, same
configuration): every lap ran **13 energy + 2 reconciliation iterations**
(15 full `solve_lap` passes); per-lap wall **160.8 - 411.1 s**, rising
through the stint as the speed solve fell out of the 530-600 V LUT domain.

| Comparison | Baseline | Post-redesign | Speedup |
|---|---|---|---|
| Identical 11-lap prefix (measured) | 2,972.7 s | 525.1 s | **5.66×** |
| Full 22 laps | ~7,400-8,500 s (extrapolated, not measured) | 960.0 s | ~5.2-5.7× (est.) |
| Worst single lap | 411.1 s (lap 10) | 45.6 s | 9.0× |
| Iterations per lap | 13 + 2 | 3 + reconciliation (2 early, fewer passes late) | ~4-5× |

The 5× redesign target is **met on the directly measured 11-lap prefix
(5.66×)**. The per-lap wall is flat (39.9-45.1 s) because the extended
400-600 V axis keeps the speed solve in-domain as SOC declines — the
baseline's late-stint 2.3× slowdown is gone.

## Attribution

- **W1** (undamped SOC coupling update with divergence guard): 13 → 2-3
  coupling iterations; the pre-redesign residual sequence halved at exactly
  the hard-coded 0.5 relaxation, so the iteration count was pure damping, not
  physics.
- **W4** (LUT voltage-axis extension 530 → 400 V floor): keeps every speed
  solve in-domain across the stint; per-pass cost at low SOC fell from ~60 s
  to ~12-14 s.
- W3 (joint fixed point) and W5 (stint-level SOC seeding) were measured
  unnecessary for the target and skipped/deferred — see the ledger.

Converged-answer fidelity was gated throughout: one-lap answers match the
pre-redesign references to ≤ 0.011 ms lap time, ≤ 2.1e-7 SOC, ≤ 0.0006 %
energy; the flying-lap pin moved 2.5 µs (in-domain interpolation replacing
exact fallback below 530 V), documented in commit `21cff00`. Full suite:
**293 tests OK**.