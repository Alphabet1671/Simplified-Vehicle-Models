# Task 4 Report

## Status
Implemented LUT-first drive and regeneration assembly integration, explicit analytical benchmark mode, and query diagnostics. No merge or push performed.

## Implementation
- `PowertrainParameters.motor_mode` selects canonical `lut` (default) or explicit `analytical` baseline.
- Drive voltage-coupling passes use `motor_torque_limits` and `torque_for_bus_power`; analytical mode retains the old bounded bisection baseline.
- Regen uses the signed inverse charge-power target and preserves tire/friction allocation, speed ceiling, and storage constraints. Inverter loss is not applied a second time after bus-power lookup.
- `LapResult` carries forward/inverse/fallback query counts.
- Maintained benchmark supports `--mode analytical|lut|compare`, untimed warm-up, repeated same-process laps, median/min wall times, residuals, constraint status, counters, and map-load cost.

## Verification
- Focused powertrain suite: **32 tests, OK**.
- `git diff --check`: passed.
- Focused Task 4 command (`test_powertrain test_mis_regression test_energy_lap`): 53 passed, 1 failed. Existing assembled MIS reference failed: LUT default lap **69.2110489030 s** versus analytical reported reference **69.2012862436 s**, delta **+0.0097626594 s** (the 1 ms gate is not met). Other MIS and energy tests passed.
- Full `unittest discover -s tests -v` was started with the bundled interpreter but remained running beyond the practical session window; no passing result is claimed.
- Maintained `benchmark_soc_solver.py --mode compare --runs 3` was started with the bundled interpreter and exact cache default. It exceeded the practical execution window before emitting final JSON and was stopped; therefore no timing win is claimed.

## Regen energy
Not run: each full-track mode did not complete within the required 15-minute per-mode budget. No dimensionally defective legacy energy field was used or expanded.

## Concerns
The coarse persisted map introduces an assembled flying-lap displacement of approximately 9.8 ms, and the repeated full-track LUT benchmark is currently too slow to complete in the available window. The implementation is retained as an isolated feature change, but the LUT performance/numerical acceptance gates remain open and should not be promoted as a performance win.

## Files
See `docs/validation/powertrain.md` for the maintained command, diagnostics, fallback behavior, map provenance, and gate interpretation.

---

# Fix round 2

## Reconciliation

The worktree is clean at `9014a71` with no half-finished edits. HEAD history for
this round:

| Commit | Contents |
|---|---|
| `156f69b` | Regenerated canonical map (25x129x65, digest `0166bd0b...`) with the exact drive envelope, and the canonical shape/digest/regen-node test updates |
| `9de3832` | NEW-1 (LUT drive envelope must respect the storage discharge limit) and NEW-2 (row tolerance must not shadow the coupling tolerance) |
| `75df191` | Bounded out-of-map LUT limit and inverse cost; `_exact_limit` memoized on the map content identity |
| `9014a71` | Halved the bounded exact-limit (16 bisections) and regen-scan (9 samples) fallback cost |

Earlier in the round the working tree was committed as `29be5af` (vectorized
`_interpolated_row`, row-based drive acceptance, vectorized regen scan, reused
inverse operating point, bounded fallback) and `0eaf181` (exact drive envelope
generation). No merge and no push were performed. `git diff --check` is clean.

## Defect fixes and their regressions

| Defect | Fix | Covering tests |
|---|---|---|
| 1. Fallback storm / driven acceptance | `torque_for_bus_power` solves the monotone drive row directly and accepts on that row plus a discretization tolerance (`_row_power_tolerance` = 1.0e-6 W + 1e-12 relative) instead of demanding sub-1e-7 W agreement with the differently ordered trilinear forward | `test_in_map_drive_inverse_does_not_fall_back`, `test_in_map_drive_inverse_is_cheap` |
| 2. Expensive fallback | `_exact_limit` 16 bisections (was 64); the drive fallback is a single monotone bisection; the regen fallback does not use the multi-thousand-knot loss map as a sample set | `test_exact_fallback_is_bounded`, `test_out_of_domain_limit_query_is_cached` |
| 3. Scalar row construction | `_interpolated_row` vectorizes the four-corner load-axis interpolation; the regen mask and the regen segment scan are vectorized | `test_vectorized_row_matches_scalar_reference`, `test_regen_inverse_segment_scan_selects_deepest_segment`, `test_regen_scan_matches_independent_row_reference` |
| 4. Double operating-point evaluation | `maximum_drive_force` reuses `solved.operating_point` instead of recomputing `evaluate_motor` | `test_drive_force_operating_point_has_positive_evaluated_bus_power` |
| NEW-1 | The envelope shortcut now requires `envelope_result.feasible and envelope_result.bus_power <= target` | `test_lut_drive_limit_never_exceeds_storage_discharge_power`, `test_lut_active_constraint_matches_storage_limit` |
| NEW-2 | The local row tolerance is named `row_tolerance`, so the coupling `tolerance = 1.0e-6` is no longer shadowed | `test_lut_drive_coupling_tolerance_is_not_widened` |

## Evidence (a): inverse microbenchmark, before and after

Command (bundled interpreter, `PYTHONPATH=src`, worktree root):

```text
python3 /tmp/lutfix/inverse_bench.py  /Users/.../.worktrees/point-mass   # before
python3 /tmp/lutfix/inverse_bench2.py /Users/.../.worktrees/point-mass   # after
```

The "before" run measured the committed baseline `33971fd` (stash A/B, stash
popped afterwards).

```text
BEFORE (committed 33971fd), in-map-scale targets 20,000..200,000 W:
in-map : 2000 queries, 70.2 us/call, fallback evaluations=0 (0.0/call)
beyond-map: 2000 queries, 52237.8 us/call, fallback evaluations=6220000 (3110.0/call)

AFTER (9014a71), three arms:
in-map     : 2000 queries,     25.2 us/call, fallback evaluations=       0 (   0.0/call)
above-row  : 2000 queries,     22.0 us/call, fallback evaluations=       0 (   0.0/call)
out-of-map : 2000 queries,    734.0 us/call, fallback evaluations=   81200 (  40.6/call)
```

Out-of-domain worst case: **52,238 us/call and 3,110 fallback evaluations per
call before, 734 us/call and 40.6 per call after (71x faster, 77x fewer
evaluations)**; the in-map hot path is unchanged at 0 fallbacks and is 2.8x
faster per call (70.2 -> 25.2 us) because the row is now vectorized.

Assembly-level check: a 24-call `maximum_drive_force`/`maximum_regen_force`
sweep over 1..100 m/s in LUT mode completes in 0.9 ms with 4 inverse queries and
**0 fallback evaluations**.

## Evidence (b): maintained benchmark gates

```text
PYTHONPATH=src <bundled python3> examples/benchmark_soc_solver.py --mode compare --runs 3
```

Raw summary (JSON in `/tmp/lutfix/bench_gate_r3.json`):

```json
{
  "analytical": {"median_wall_seconds": 8.036, "min_wall_seconds": 8.033,
                 "lap_time_s": 69.201286244, "converged": true,
                 "motor_forward_queries": 0, "motor_inverse_queries": 0,
                 "motor_fallback_queries": 0,
                 "force_residual_max": 6.821e-13},
  "lut":        {"median_wall_seconds": 9.151, "min_wall_seconds": 9.146,
                 "lap_time_s": 69.211083202, "converged": true,
                 "motor_forward_queries": 39339,
                 "motor_inverse_queries": 24854,
                 "motor_fallback_queries": 360611,
                 "force_residual_max": 6.821e-13},
  "comparison": {"lap_time_delta_s": 0.009796958693911506,
                 "maximum_speed_displacement_mps": 0.13464732178945127,
                 "median_wall_delta_s": 1.1142574169207364,
                 "both_converged": true}
}
```

| Gate | Threshold | Measured | Pass |
|---|---|---|---|
| Exact assembled MIS lap converges | both converged | both true | yes |
| Lap-time displacement | <= 1 ms | 9.797 ms | **no** |
| Maximum pointwise speed displacement | <= 0.05 m/s | 0.134647 m/s | **no** |
| LUT median wall < analytical median wall | strictly lower | 9.151 s vs 8.036 s | **no** |

Only the convergence gate passes. The `--runs 1` run reproduces the same
numbers (lap delta 9.797 ms, displacement 0.134647 m/s, LUT 9.149 s vs
analytical 8.097 s).

## Evidence (c): grid table

One grid candidate per isolated repo copy and process, so no compiled-map or
memoized-limit state leaks between candidates; the analytical baseline is
map-independent and measured separately (`69.201286244 s`, `8.089 s`).
Generation time is wall time of `tools/generate_motor_lut.py` for that grid.

| Grid | NPZ MB | Generation s | Lap delta ms | Max speed displacement m/s | LUT 1-run wall s | Fallbacks |
|---|---|---|---|---|---|---|
| 9x129x65 | 1.07 | 4.0 | 9.7627 | 0.133847 | 15.59 | 722,601 |
| 13x129x65 | 1.54 | 5.7 | 9.7964 | 0.134635 | 15.55 | 722,493 |
| 17x129x65 | 2.01 | 7.5 | 9.7903 | 0.134488 | 15.48 | 722,493 |
| 17x257x65 | 3.99 | 14.2 | 9.7933 | 0.134558 | 15.61 | 722,574 |
| **25x129x65 (canonical)** | **2.95** | **10.6** | **9.7970** | **0.134647** | **15.52** | **722,736** |
| 25x385x65 | 8.80 | 30.7 | 9.8006 | 0.134734 | 15.53 | 723,411 |
| 33x257x65 | 7.74 | 27.0 | 9.7985 | 0.134680 | 16.17 | 722,655 |

The wall times in this table come from a per-candidate process that also
regenerates the map, so they are not the maintained comparison timings (those
are the 8.036 s / 9.151 s medians above). The important result is that
**refining the grid changes the lap displacement by at most 0.038 ms and the
speed displacement by at most 0.0009 m/s** - neither failing gate is a grid
resolution problem. 25x129x65 is retained as canonical because it is the
smallest grid that is not worse than the alternatives.

Reproducibility: `tools/generate_motor_lut.py` exposes `--voltage-points`,
`--speed-points` and `--load-points`; regenerating the canonical map twice is
byte-identical (`0166bd0b42972e2d356d03ac462786ef5d0f79d6e131e78ef850f83471728608`).
The chosen grid and digest are recorded in `models/data/provenance.md`.

## Evidence (d): suites

```text
python3 -m unittest tests.test_powertrain -q          Ran 50 tests  OK
python3 -m unittest tests.test_mis_regression -q      Ran 10 tests  OK
python3 -m unittest tests.test_energy_lap -q          Ran 12 tests  OK
python3 -m unittest discover -s tests -q              Ran 254 tests OK
```

`git diff --check` is clean and the worktree is clean.

## Regen full-track comparison

Not run. Each full-track mode exceeds the 15-minute-per-mode budget, so no
regen energy comparison is claimed.

## Honest conclusion

The four originally measured defects are fixed and covered by regressions, and
the earlier storage-safety defect introduced while fixing them (NEW-1) is fixed
too. The performance picture is nevertheless negative:

* the LUT assembled lap is **slower** than the analytical baseline (9.151 s vs
  8.036 s median, +1.115 s) once the LUT correctly respects the storage
  discharge limit;
* the lap-time displacement is 9.797 ms, 9.8x the 1 ms gate;
* the maximum pointwise speed displacement is 0.1346 m/s, 2.7x the 0.05 m/s
  gate.

An intermediate state during this round did measure the LUT at 4.83 s vs the
analytical 8.06 s (1.67x faster), but that state used the interpolated motor
envelope even when it exceeded the storage discharge limit, i.e. it was faster
because it was wrong. That number is recorded here only to make clear it is
**not** a valid speedup claim. `PowertrainParameters.motor_mode` therefore
remains `"analytical"` by default and the LUT stays an explicit opt-in
experiment. The recommendation is not to promote the LUT until the numerical
gates pass.

---

# Task 7: attribution and documentation

## Status
Documentation corrected, residual displacement attributed, all test suites pass.
No production code changed. No map artifact changed. No digest changed.

## Deliverable 1: Stale documentation correction

`docs/validation/powertrain.md` previously recorded:
- Digest `0166bd0b...` (now `31cfadc8...`)
- A table saying the LUT was slower (9.151 s vs 8.036 s)

These are now false. Updated to:
- Digest `31cfadc88b66d1425d210ad71cbc85ba0dea75564903014a1075efb7669d5fe9`
- Reproduction command includes `--voltage-points 25 --speed-points 129 --load-points 65 --voltage-min 530 --voltage-max 620`
- Measured results table showing analytical 8.253 s vs LUT 4.394 s (1.878x), zero fallback queries, both converged, and the two numerical gates still failing (lap delta +9.797 ms, max speed displacement 0.134642 m/s)
- Plain statement that the LUT is faster but not numerically equivalent enough to promote, so analytical stays the default

`models/data/provenance.md` already had the correct digest, 530-620 V axis, point counts, and command. Added the measurement that justified the voltage axis change (the 1.878x speedup with zero fallback after extending the axis).

## Deliverable 2: Residual displacement attribution

### Reference-solve method

A high-fidelity reference was built in `/tmp/reference_solve.py` (not production code). It computes the drive force limit with a converged analytical solve at 48 bisection steps (vs the production default's 8 steps) plus a properly converged voltage coupling loop (10 iterations, tolerance 1e-3 V). The reference was evaluated at a representative sweep of 7 speeds (3000-18000 rpm) x 5 SOC values (0.05-0.95) = 35 points.

For each point, three modes were computed:
- **Reference**: 48 bisection steps + converged voltage coupling
- **Analytical default**: 8 bisection steps (exactly replicating assembly.py lines 313-319)
- **LUT mode**: `motor_torque_limits` + `torque_for_bus_power` with voltage coupling (exactly replicating assembly.py lines 304-337)

Errors were measured as force (N), bus-power (W), torque (Nm), and terminal voltage (V).

### Error table

**Force errors (N, wheel force)**:

| Metric | Analytical default | LUT |
|---|---|---|
| Max error | 2.635 N | 443.98 N |
| 95th percentile | 2.073 N | 424.91 N |
| Mean | 0.507 N | 104.35 N |

**Bus-power errors (W)**:

| Metric | Analytical default | LUT |
|---|---|---|
| Max error | 284.4 W | 22,126.5 W |
| 95th percentile | 264.2 W | 20,997.9 W |
| Mean | 75.2 W | 5,036.5 W |

**Torque errors (Nm)**:

| Metric | Analytical default | LUT |
|---|---|---|
| Max error | 0.064 Nm | 10.72 Nm |
| 95th percentile | 0.050 Nm | 10.26 Nm |

**Voltage errors (V)**:

| Metric | Analytical default | LUT |
|---|---|---|
| Max error | 1.26 V | 115.1 V |
| 95th percentile | 0.94 V | 92.6 V |

### Which mode is closer to the reference

**The analytical default is closer to the converged reference.** At low and
moderate speeds (3000-12000 rpm) both modes closely match the reference
(torque error < 0.06 Nm). At high speeds where the terminal voltage drops
below the 530 V map floor (e.g., 14540-18000 rpm at SOC 0.05-0.95), the
reference correctly returns zero torque (infeasible), the analytical default
also returns zero (matching the reference), but the LUT clamps to the 530 V
map floor and reports feasible torque (up to 10.7 Nm). The LUT's voltage
error of 115 V max confirms this clamping effect.

**Interpretation of the 9.8 ms lap displacement**: The displacement is
therefore NOT "LUT error" but rather "difference between an exact inverse
and an 8-step approximation." The analytical default's 8-step bisection
from an infeasible envelope (`maximum_motor_torque` at 14540 rpm / 516.6 V
returns 19.622 Nm while the exact feasible limit is 12.710 Nm) is itself an
approximation. The LUT is not proven to be closer to the truth than the
analytical default, and neither mode passes the numerical gates.

### Flying lap with converged reference

Not practical within the budget. Running a full flying lap with a monkeypatched
converged reference drive path would require replacing `assembly.py`'s drive
limit solver at runtime for the entire track, which is not feasible without
modifying production code. The pointwise sweep above is the best available
attribution.

### Raw numbers

The full sweep data is in `/tmp/reference_summary.json`. Representative
points:
- SOC=0.05, speed=14540 rpm: ref_torque=0.0000 Nm, an_torque=0.0000 Nm,
  lut_torque=9.0944 Nm (LUT reports feasible where reference says infeasible)
- SOC=0.50, speed=9000 rpm: ref_torque=18.2505 Nm, an_torque=18.2245 Nm,
  lut_torque=18.2513 Nm (all modes agree to within 0.03 Nm)
- SOC=0.80, speed=9000 rpm: ref_torque=20.5508 Nm, an_torque=20.5508 Nm,
  lut_torque=19.1828 Nm (LUT under-delivers by 1.37 Nm due to map discretization)

## Deliverable 3: Test suites and tree cleanliness

### Test counts

```text
PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest tests.test_powertrain -q
  Ran 50 tests OK

PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest tests.test_mis_regression -q
  Ran 10 tests OK

PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest tests.test_energy_lap -q
  Ran 12 tests OK

PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest discover -s tests -q
  Ran 254 tests OK
```

### Verification

- `git diff --check`: passed (no whitespace errors)
- `git status`: clean (working tree has no uncommitted changes after commit)
- HEAD: `de033b9` (commit on branch `feature/powertrain`, no merge, no push)

## Commands used

```bash
# Reference attribution solve
PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 /tmp/reference_solve.py

# Test suites
PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest tests.test_powertrain -q
PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest tests.test_mis_regression -q
PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest tests.test_energy_lap -q
PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest discover -s tests -q

# Verification
git diff --check
git status
```

## Concerns

1. The LUT mode's clamping to the 530 V map floor when the terminal voltage
   drops below that range is a correctness concern. The LUT can report
   feasible torque where the true physics says infeasible. This is not a
   storage-discharge-limit violation (the LUT's 1,010-point validation showed
   zero violations), but it means the LUT drive limit is not numerically
   equivalent to the analytical default at high speeds.
2. The 8-step analytical bisection from an infeasible envelope is itself an
   approximation (e.g., 19.622 Nm where the true limit is 12.710 Nm at
   14540 rpm / 516.6 V). The analytical default is not proven to be closer to
   the truth either.
3. The lap-time displacement (+9.797 ms) and speed displacement (0.134642 m/s)
   remain above their respective gates (1 ms, 0.05 m/s). Neither mode passes
   the numerical gates.
4. The reference solve used `maximum_motor_torque` for the envelope, which is
   the same function used by the analytical default. The reference 48-step
   bisection finds the true feasible limit within that envelope's validity.
   The LUT's errors are partly due to using a different envelope (the map's
   interpolated envelope at a clamped voltage) rather than the true terminal
   voltage envelope.

## Report path

`/Users/alphabet1671/Dev/Simplified-Vehicle-Models/.worktrees/point-mass/.superpowers/sdd/2026-10-12-motor-lut/task-4-report.md`

---

## Honest conclusion (Task 7)

The attribution is unambiguous: **the analytical default is closer to the
converged reference than the LUT mode**. The analytical default's 8-step
bisection from the infeasible envelope returns 0 Nm at high speeds where the
reference also returns 0 Nm, while the LUT clamps to the 530 V map floor and
reports non-zero torque. The LUT's real 1.878x wall-time speedup with zero
fallback queries is genuine, but it is achieved by using a map that does not
cover the full loaded terminal voltage range, and it is not numerically
equivalent enough to promote. The displacement is dominated by LUT error rather
than by replacing the analytical 8-step bisection with an exact inverse: on the
attribution sweep the analytical default's force error against the converged
reference was 2.63 Nm maximum and 2.07 Nm at the 95th percentile, against
443.98 Nm and 424.91 Nm for the LUT. A controller-run side-by-side at matched
speed and state of charge confirms the LUT reports force where the truth is
zero: at SOC 0.30 and 34 m/s the LUT reported 2070.4 N while the analytical
default reported 0.0 N, with the same pattern at SOC 0.15 (30 and 34 m/s),
SOC 0.08 (30 and 34 m/s), SOC 0.04 (30 m/s) and SOC 0.02 (25, 30, 34 m/s).
Both modes remain below their numerical gates, and analytical stays the default.

---

## Task 8 (defects A, B, C)

Commits: A `37ed923`, B `f45e01e`, docs commit follows. Python is the bundled runtime, `PYTHONPATH=src`, run from the worktree root.

### A. Per-motor vs pack limit (analytical default)
Verified: at 17f8d69 the analytical branch compared per-motor `bus_power` with the pack `discharge_power_limit`. Fix: `/ motor.unit_count` in both comparisons. The pre-fix regression test fails on the old code (verified by reverting the diff) and passes after.
Golden analytical lap: 69.20128624364081 -> **69.20462625809404 s** (pinned in tests/test_mis_regression.py and docs). The lumped path does `bus = min(bus, pack_limit)` on a total power, so it has no such bug.

### B. Voltage-scaled envelope
- New `voltage_scaled_envelope(parameters, speed, V)` = `maximum_motor_torque(parameters, speed, V/sqrt(3))`, used by the generator (drive and regen) and `_exact_limit_uncached` (fallback).
- Regen: the analytical regen path takes `maximum_motor_torque(V/sqrt(3))` as its envelope too, so it is the reference; the same cap is applied.
- Generator `motor-lut-v3`; SHA-256 `0fe463ea9f8c74a7b29aa2f7ebdb17aa8b376d9a38dfcd9dd4dee4723ee1eeb9` (was `31cfadc8...`). Regenerated twice: `cmp` byte-identical.
- Cliff interpolation measured (17,000-18,600 rpm x 530-560 V, 4,991 probes, before mitigation): LUT above exact at 3,614 probes, max over 10.45 Nm, max under 9.51 Nm: significant. Mitigation: `_envelope_cliff_cell` (14 of 3,072 cells straddle zero/non-zero) sends those cells to the exact oracle in `_lut_forward`, `motor_torque_limits`, `torque_for_bus_power`. Residual: LUT limit > cap only by <= 8.7e-7 Nm above 1000 rpm, 2.8e-6 above 300 rpm, 0.108 Nm below 300 rpm (interpolating the source envelope breakpoints); 0 wherever the cap is 0.
- Tests added: `VoltageScaledEnvelopeTests` (corners 18273 rpm at 540/517.95/529.999 V all 0; 36 voltages incl. below the 530 V floor x 187 speeds never exceed the cap). Existing fixtures adapted (digest, generator mock, `_in_map_targets` skips cliff cells).

### Final measurement (`examples/benchmark_soc_solver.py --mode compare --runs 3`, run alone)
| Gate | Analytical | LUT | Result |
|---|---|---|---|
| median wall | 8.831 s | 5.186 s | LUT faster (1.70x) pass |
| lap time | 69.20462625809404 | 69.20391185187009 | delta -0.714 ms (<=1 ms) pass |
| max pointwise speed displacement | | 0.004999 m/s (<=0.05) | pass |
| converged | True | True | pass |
Counters (LUT): forward 104,641 / inverse 24,616 / fallback 1,725 (cliff cells + out-of-map). Exact cache: 189,747 hits/26,739 entries (an), 189,764/26,722 (LUT). Peak speed 29.7746 vs 29.7785 m/s.

### Sweep (SOC 1,.9,.75,.5,.3,.15,.08,.04,.02 x 0-38 m/s step 0.5, 693 points/mode)
- pack-discharge-limit violations: analytical 0, LUT 0.
- Instrumented every `motor_torque_limits` call in the LUT sweep (1,260 queries): 0 above the voltage-scaled cap.
- SOC 0.30/0.15/0.08/0.04/0.02 at 25/30/34/36/38 m/s, LUT vs analytical force (N): 0.30: 2763/2753, 2330/2323, 0/0, 0, 0; 0.15: 2692/2686, then 0; 0.08: 2618/2610, then 0; 0.04: 2169/2162, then 0; 0.02: all 0. The old 2,070 N at SOC 0.30 / 34 m/s is now 0.

### C. Attribution of the remainder (LUT - analytical = -0.714 ms, 0.005 m/s)
Sweep of 312 points (SOC 1-0.15 x 10-35.5 m/s), force differences, N:
| Candidate | Number |
|---|---|
| LUT vs analytical 8-step | max 15.97 (SOC 0.3, 18 m/s), mean 4.17, signed mean LUT-an +3.48 |
| analytical 24-step vs 8-step bisection | max 16.01, signed mean +4.40 (8-step is a lower bound) |
| LUT vs analytical 24-step | max 15.75, mean 0.94 |
| LUT vs analytical, both 24-step & 6-pass coupling | max 6.60, mean 0.19 |
| 2 vs 6 voltage-coupling passes | analytical max 2386 N, mean 158; LUT max 2410 N, mean 166 (common to both; neither converges in any of 312 points at 6 passes) |
| coupled V < 530 V floor | 271 of 312 points in both modes (the map floor is clamped, but the cap/fallback/cliff handling now agrees with analytical) |
Lap with a 24-step analytical bisection (temporary experiment, reverted): analytical 69.20288182 s, LUT 69.20391185 s, delta +1.030 ms, max displacement 0.0247 m/s. So the -0.714 ms residual is the analytical 8-step lower bound (about -1.7 ms of lap) being larger than the LUT's difference from the exact limit; i.e. LUT is closer to the exact-bisection reference than the shipped analytical default. The coupling non-convergence affects both modes equally (same 2-pass scheme) and cancels in the comparison. Verdict: accepted approximation difference, not a defect; no change.

### Tests
tests.test_powertrain + test_mis_regression + test_energy_lap: 75 tests OK after B. Full suite `python -m unittest discover -s tests -q`: 257 tests OK.

### Default flip
Not done. Experiment (reverted): flipping `motor_mode` to "lut" makes the full suite fail 2 tests that pin the analytical golden (lap 69.2046 vs 69.2039, and a bus-power value off by 6.8 mW), so the stated precondition (suite passing) is not met without re-baselining; left as a controller decision.

## Task 9 (regulatory bus-voltage clamp)

Commands (from worktree root, `PYTHONPATH=src`, bundled python):
- `tools/generate_motor_lut.py models/power-plants/amk_dd5_100c.xml /tmp/m1.npz --voltage-points 25 --speed-points 129 --load-points 65 --voltage-min 530 --voltage-max 600` twice -> `cmp` IDENTICAL; SHA-256 `166bf1346d263267224f77fe7396ab15180e0c2f54391652fee28f8575fcfc5b` (~10 s each). Committed XML sha256 `e36e52b296e505fef82dc79ae63716d984955964803911c1bee8e4b6f06f4499`.
- Map: min drive torque below 17,000 rpm 11.36 Nm; zero nodes only at 17,728-18,600 rpm (the cap corner), 0 near-zero nodes at low speed (was 26). `motor_torque_limits(611.09 V, 8.4 rpm)` now 21.81 (clamped to 600 V).
- Measured: low-speed drive overshoot vs cap 0.13589 Nm (530 V / 88.35 rpm; 0.05 rpm grid); >=300 rpm 1.06e-6 Nm. LUT vs exact limit: 0.1152 Nm max <700 rpm (0.00033 Nm for 300-700 rpm), 0.0609 Nm above 700 rpm (556 V / 9,381 rpm).
- Tests: test_powertrain + test_mis_regression + test_energy_lap 83 OK.
- Golden analytical lap 69.20462625809404 -> 69.20459210208878 s.
- Benchmark compare, runs 3: analytical median 9.00733 s (min 8.95954), lap 69.20459210208878; LUT median 5.19253 s (min 5.14100), lap 69.20391204362008; delta -0.680058 ms; max speed displacement 0.003946162 m/s; both converged; analytical counters 0/0/0; LUT forward 104,716 inverse 24,619 fallback 1,725; speedup 1.735x. Gates: lap delta <=1 ms PASS, speed <=0.05 PASS, converged PASS, LUT faster PASS.
- Sweep (/tmp/sweep9.py; SOC 1,.9,.75,.5,.3,.15,.08,.04,.02 x 0-38 m/s step 0.5 = 693 pts): pack-limit violations 0 / 0; LUT limits above clamped cap 0 of 1,890 queries (>1e-5 Nm); max |dF| 16.5515 N, mean 2.8175 N; zero-vs-nonzero mismatches 0.

## Task 9 fix round

Docs/tests/comments only; no behaviour, map, digest, golden lap or default motor_mode change.

- Doc reproduce command now `--voltage-max 600` (matches provenance.md line 12); regenerated into /tmp, SHA-256 `166bf134...fcfc5b` equals the committed artifact.
- Documented terminal vs clamped voltage in the Task 9 choke-point paragraph and the `DriveLimit` docstring; `_exact_limit` docstring states clamp-before-cache-key; idempotent-clamp and per-phase no-op comments added.
- Tests: two-sided regen check (measured max error 0.1362 Nm below 300 rpm -> delta 0.14; 0.00033 Nm above -> 0.001; 0.0235 Nm at 700-18000 rpm -> 0.03). Fallback-skip bounds: measured 0 (low-speed sweep) -> <=2; 1 (high-speed sweep) -> <=3. New guard `test_raising_maximum_bus_voltage_above_600_exposes_known_defect` (800 V XML, 650 V bus: measured 129,629 W at 0 rpm, 124,919 W at 145 rpm; asserts > 50 kW). Doc line added.
- Counts: test_powertrain 62, test_energy_lap 12, test_mis_regression 10, full suite 266, all OK.
