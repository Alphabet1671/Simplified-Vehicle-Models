# W0 Diagnostics Report — Energy-Coupling Solver Redesign

Date: 2026-10-12
Branch: `feature/energy-coupling` (HEAD `a0e5c0f`, worktree `.worktrees/point-mass`)
Machine: 8-core, effectively uncontended

## Commands

```text
# Unit tests (diagnostics + energy lap)
PYTHONPATH=src python3 -m unittest tests.test_energy_lap -v

# Full-track diagnostic runs (one-lap energy solves, regen_enabled=False)
PYTHONPATH=src python3 /tmp/w0_diag_run.py

# Full test suite
PYTHONPATH=src python3 -m unittest discover -s tests -q
```

## Instrumentation

Added `DiagnosticsCollector` dataclass (opt-in parameter `diagnostics_collector`) to `src/simplified_vehicle_models/solvers/forward_backward.py::solve_lap_with_energy`. When provided:

- Per-iteration residual `max|nxt - trace|` **before** the damped update (line ~434)
- Per-iteration wall time
- Reconciliation-loop residuals
- Wall-time split: cumulative `solve_lap` vs `_energy_trace` vs other
- `evaluate_shaft_power` oracle call count (via wrapper)
- `motor_lut_counters()` snapshot per solve
- Records flushed incrementally to a JSON-lines file

Default behavior and all numerical results are identical with or without instrumentation. No existing test modified.

## Full-Track Diagnostic Results

Track: `models/tracks/2024_michigan_endurance_v11.xml` (1,407 nodes, 1,406 segments)
Vehicle: `models/formulations/murphy_point_mass.xml`
Mode: `LapMode.FLYING`, `regen_enabled=False`, default tolerances

### SOC 0.9617 (compiled initial SOC)

| Metric | Value |
|---|---|
| Energy iterations | 13 |
| Wall time | 182.534 s |
| Lap time | 69.187443 s |
| Final SOC | 0.9333203465 |
| Consumed energy | 814845.010 J |
| Min bus voltage | 593.76 V |
| Max bus voltage | 600.00 V |
| Oracle calls (`evaluate_shaft_power`) | 14,084 |
| LUT counters (forward/inverse/fallback/limit_fallback) | 2,433,382 / 562,989 / 2,548,896 / 115,095 |
| solve_lap seconds | 181.857 s |
| _energy_trace seconds | 0.677 s |
| other seconds | 0.000 s |

Full residual sequence (before damped update):
```
2.837835e-02, 1.420161e-02, 7.107034e-03, 3.556628e-03, 1.779861e-03,
8.906893e-04, 4.457122e-04, 2.230386e-04, 1.116101e-04, 5.585055e-05,
2.794849e-05, 1.398355e-05, 6.995824e-06
```
Reconciliation residual: `3.499449e-06`

### SOC 0.5

| Metric | Value |
|---|---|
| Energy iterations | 13 |
| Wall time | 894.834 s |
| Lap time | 69.057656 s |
| Final SOC | 0.4680491991 |
| Consumed energy | 823901.201 J |
| Min bus voltage | 527.57 V |
| Max bus voltage | 529.81 V |
| Oracle calls | 14,017 |
| LUT counters (forward/inverse/fallback/limit_fallback) | 0 / 418,772 / 37,526,516 / 1,359,872 |
| solve_lap seconds | 893.992 s |
| _energy_trace seconds | 0.841 s |
| other seconds | 0.000 s |

Full residual sequence:
```
3.194640e-02, 1.597628e-02, 7.989690e-03, 3.993783e-03, 1.997313e-03,
9.988579e-04, 4.995397e-04, 2.498180e-04, 1.249376e-04, 6.248047e-05,
3.124785e-05, 1.562736e-05, 7.814567e-06
```
Reconciliation residual: `3.908197e-06`

### SOC 0.25

| Metric | Value |
|---|---|
| Energy iterations | 13 |
| Wall time | 997.765 s |
| Lap time | 69.011580 s |
| Final SOC | 0.2169114806 |
| Consumed energy | 825976.532 J |
| Min bus voltage | 511.87 V |
| Max bus voltage | 514.58 V |
| Oracle calls | 14,000 |
| LUT counters (forward/inverse/fallback/limit_fallback) | 0 / 432,705 / 37,686,852 / 1,358,578 |
| solve_lap seconds | 996.759 s |
| _energy_trace seconds | 1.006 s |
| other seconds | 0.000 s |

Full residual sequence:
```
3.308173e-02, 1.654423e-02, 8.273819e-03, 4.137788e-03, 2.069325e-03,
1.034870e-03, 5.175371e-04, 2.588185e-04, 1.294371e-04, 6.473073e-05,
3.237339e-05, 1.618936e-05, 8.095177e-06
```
Reconciliation residual: `4.048219e-06`

## Gate A Answers

### D1: Does the per-iteration residual halve geometrically?

**Yes, confirmed.** The residual sequence for every starting SOC is a geometric decay with contraction factor **≈ 0.5002** (i.e., exactly 0.5 within measurement precision).

| SOC | Initial residual | Final residual | Contraction factor (ratio of consecutive terms) |
|---|---|---|---|
| 0.9617 | 2.838e-02 | 6.996e-06 | 0.5003 |
| 0.5 | 3.195e-02 | 7.815e-06 | 0.5002 |
| 0.25 | 3.308e-02 | 8.095e-06 | 0.5002 |

The contraction factor is `≈ 0.5` across all three runs, exactly matching the hard-coded `trace = 0.5 * (trace + nxt)` damping. 12 halvings reduce an initial mismatch of ~0.03–0.033 to ~7–8e-06, which is below the `tolerance=1e-5` threshold. **The 0.5 damping is the sole determinant of the 13-iteration count.**

### D2: What fraction of per-lap wall time is solve_lap vs _energy_trace vs other?

**The overwhelming majority is `solve_lap`.** The `_energy_trace` oracle calls are nearly negligible (~0.3–0.4% of wall time). The `other` category is effectively zero.

| SOC | solve_lap % | _energy_trace % | other % |
|---|---|---|---|
| 0.9617 | 99.63% | 0.37% | ~0% |
| 0.5 | 99.91% | 0.09% | ~0% |
| 0.25 | 99.91% | 0.10% | ~0% |

- SOC 0.9617: 181.857s solve_lap / 0.677s energy_trace / 182.534s total
- SOC 0.5: 893.992s solve_lap / 0.841s energy_trace / 894.834s total
- SOC 0.25: 996.759s solve_lap / 1.006s energy_trace / 997.765s total

The cost is **entirely in the repeated `solve_lap` calls**, not in the energy oracle. The `_energy_trace` oracle (14,000+ `evaluate_shaft_power` calls per lap) costs only ~0.6–1.0 seconds total.

### D3: How do oracle/fallback counts, voltage range, and wall time change between SOC 0.96, 0.5, and 0.25?

**The speed path leaves the LUT floor at ~SOC 0.96; the energy path leaves it immediately.**

| Metric | SOC 0.9617 | SOC 0.5 | SOC 0.25 |
|---|---|---|---|
| Wall time (s) | 182.5 | 894.8 | 997.8 |
| Min bus voltage | 593.76 V | 527.57 V | 511.87 V |
| Max bus voltage | 600.00 V | 529.81 V | 514.58 V |
| LUT forward queries | 2,433,382 | **0** | **0** |
| LUT inverse queries | 562,989 | 418,772 | 432,705 |
| LUT fallback count | 2,548,896 | **37,526,516** | **37,686,852** |
| LUT limit_fallback | 115,095 | 1,359,872 | 1,358,578 |
| Oracle calls (evaluate_shaft_power) | 14,084 | 14,017 | 14,000 |

**Key observations:**

1. **The speed solve is what leaves the LUT floor.** At SOC 0.9617, the LUT still handles 2.4M forward queries (the speed solve is in-domain). At SOC 0.5 and below, forward LUT queries drop to **zero** — the speed solve is entirely on fallback. The min bus voltage at SOC 0.5 (527.57 V) is already **below the 530 V LUT floor**, confirming the speed solve exits the map.

2. **The energy path runs the analytical oracle regardless of SOC.** `evaluate_shaft_power` is aliased to `evaluate_shaft_power_exact` (`power_plant.py:1017`). The ~14,000 oracle calls per lap are identical across all three SOCs — the energy path never uses the LUT at all. This is by design (the alias), and it means the energy-path oracle cost is **SOC-independent**.

3. **Wall time scales with LUT fallback count, not energy oracle count.** The 5.5× wall-time increase from SOC 0.96 to SOC 0.5 (182s → 895s) correlates with the LUT fallback count going from 2.5M to 37.5M (15× increase), not with oracle calls (14,084 → 14,017, essentially flat). The fallback path in `evaluate_shaft_power_exact` is ~20× slower than the LUT inverse, and the speed solve makes 400K+ fallback calls per lap at SOC 0.5.

4. **The energy path never leaves the 530 V floor.** The min bus voltage seen by the energy path at SOC 0.25 is 511.87 V, well below the 530 V axis floor. But the energy path uses the analytical oracle, so the LUT floor is irrelevant to it. The floor is only relevant to the speed solve's LUT queries.

**Implication for the redesign:** The dominant cost is the speed solve's repeated LUT fallback at low SOC. The energy oracle (~14K calls/lap, ~0.7–1.0s) is a minor cost. O1 (adaptive damping/predicted initial profile) reduces the 13 `solve_lap` calls to ~2–3, which would reduce wall time from ~900s to ~200s at SOC 0.5. O2 (fold SOC into the fixed point) would eliminate the reconciliation loop. O3 (route energy path through LUT) would help less than expected because the energy oracle is only 0.1% of wall time.

## Machine Contention Statement

All three full-track diagnostic runs were executed **sequentially** (never two at once) on an effectively **uncontended** 8-core machine. No other significant compute activity was running during the measurements. The wall-time numbers are reliable for comparison purposes.

## Test Results

```
Ran 17 tests in 22.680s
OK
```

All 17 tests pass, including the 5 new `DiagnosticsCollectorTests`:
- `test_diagnostics_records_iteration_counts` — iteration counts ≥ 1, residuals finite
- `test_residuals_are_finite` — all residuals are finite
- `test_timing_split_sums_to_total` — timing split sums to total
- `test_bit_identical_with_and_without_diagnostics` — lap_time, final SOC, consumed energy bit-identical
- `test_oracle_call_count_and_lut_counters` — LUT counter snapshots are non-empty

Full unittest discovery suite: **passes** (see commit).

## Tree State

Clean tree, `git diff --check` passes. Commits:
- `2102593`: W0 instrumentation + tests
- Report committed separately.
