# P10 powertrain validation

## Scope and defaults

This validation covers the assembled point-mass solver and the P9 SOC/energy
interface on the 1,014.32575 m, 1,407-node Michigan endurance track. The
normal assembled default is **exact** drive evaluation (`DriveCachePolicy()`);
the quantized cache is an explicit opt-in only. Transition/profile
initial-speed acceleration is unsupported and ignored. Bracket-preserving
corner-root hints are supported and exact (not approximate); no approximate
profile reuse or default change is implied. No approximate setting is
promoted to a default without the numerical gates below.

## Assembled MIS flying-lap benchmark

Maintained reproducer command (Python and `/usr/bin/time`; historical cProfile figures are reported below):

```text
PYTHONPATH=src /usr/bin/time -p \
  /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 examples/benchmark_soc_solver.py --mode exact
```

The exact run converged in 2 iterations. Wall time was 23.77 s (`real`),
with 22.91 s measured around `solve_lap`; lap time was 69.20128624364081 s (pre-Task-8 value; see the Task 8 section).
The solve made 189,763 exact-cache hits and 26,723 entries, with zero cache
speed/SOC displacement. The largest force-balance residual was
6.82e-13 N.

The cProfile accounting (cumulative seconds) was:

| component | seconds |
|---|---:|
| equilibrium evaluation | 17.060 |
| corner-speed evaluation | 17.031 |
| drive-limit/available-drive evaluation | 16.603 |
| shaft-power evaluation | 11.169 |
| forward sweep | 3.586 |
| backward sweep | included in solve total |

The profile confirms that repeated assembled equilibrium and motor-map work,
not lap integration, dominates this benchmark.

The explicit quantized candidate used:

```python
DriveCachePolicy(speed_resolution=1e-3, soc_resolution=1e-5)
```

It converged in 2 iterations, took 6.52 s wall time (`real`), and returned
69.20126421237728 s. This is a 3.64x wall-time speedup and a lap-time delta of
-2.2031e-05 s. It had 209,479 cache hits, 7,007 entries, and maximum speed
bucket displacement 9.9017e-4 m/s. It therefore remains opt-in: the candidate
was not validated against the full regen/SOC numerical gates.

The promotion gates are assembled lap-time delta < 0.01 s, final-SOC delta
< 1e-4, recovered-energy delta < 0.5%, and matching convergence/constraints.
The lap-time gate passes; the full coupled energy evidence was not completed,
so the default is unchanged.

## Regen one-lap cold/warm evidence

The full-track benchmark used the exact default cache for both runs and the
same explicit controls: `tolerance=1e-5`, `maximum_iterations=50`,
`lap_tolerance=1e-6`, `lap_maximum_iterations=200`, `root_tolerance=1e-8`,
`regen_enabled=True`, and `drive_cache_policy=DriveCachePolicy()`. Command:

```text
PYTHONPATH=src /usr/bin/time -p \
  /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 examples/benchmark_soc_solver.py --mode regen-one --progress
```

Both cold and warm runs completed within the allowed 900 s per run. The warm
run supplied both `initial_speed` and `corner_speed_hint` from cold, but
transition/profile `initial_speed` acceleration is unsupported and ignored.
Only the bracket-preserving, exact `corner_speed_hint` can affect work; it is
not approximate profile reuse. Results:

| metric | cold | warm | warm-cold |
|---|---:|---:|---:|
| wall time (s) | 611.132856 | 460.809639 | -150.323218 |
| lap time (s) | 69.1982147152843 | 69.1982147152843 | 0 |
| energy iterations | 13 | 13 | 0 |
| profile residual | 2.5795463698408128e-06 | 2.579546369951835e-06 | 1.11e-16 |
| final SOC | 0.9790593110040156 | 0.9790593110040156 | 0 |
| consumed energy (J) | 837888.6096674838 | 837888.6096674838 | 0 |
| recovered energy (J) | 403.4186193658725 | 403.4186193658725 | 0 |

Both runs converged (`lap_converged=True`, `energy_converged=True`), used two
inner lap iterations, and reported zero cache approximation displacement. The
warm run was 24.6% faster; all numerical deltas were zero to displayed
precision. This completes the exact default cold/warm evidence; no accelerated
mode is used for this benchmark.

## Fully coupled 22-lap regen stint

The requested fully coupled (`reuse_profile=False`) attempt used a managed
background job and explicit progress callback:

```text
PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 examples/benchmark_soc_solver.py --mode regen-22 --progress
```

It was stopped at the practical 120 s limit. Progress evidence: lap 1
`lap_start` at 0 s; energy iterations 1, 2, and 3 at 22.08, 59.92, and
98.17 s. No lap completed, so completed laps = 0 and no stint totals are
reported. This lower bound demonstrates the remaining runtime limitation;
it is not evidence of convergence or failure.

The existing fast endurance study with `reuse_profile=True` is a fixed-profile
approximation and must not be read as fully state-coupled regen validation.

## Task 3 motor LUT query validation

The focused numerical suite is `tests.test_powertrain` (27 tests). It checks exact
node identity, 100 seeded random forward samples, inverse drive/regen residuals,
zero-load behavior, and analytical fallback observability. The random forward
absolute bus-power error 95th percentile was below 120 W; out-of-domain voltage
fallback returned the exact oracle result and incremented the fallback counter.
Regen inversion scans every negative-load segment and selects the most-negative
feasible torque, including non-monotone rows.

A 100,000-query microbenchmark (Python 3, canonical AMK map) measured:

| operation | microseconds/query |
|---|---:|
| exact analytical evaluator | 16.27 |
| LUT forward evaluator | 10.61 |
| in-domain drive inverse | 70.53 |

Inverse targets were generated from an in-domain half-drive-load operating point;
benchmark timing is informational and is not a test gate. Voltage, speed, and
normalized-load queries outside the persisted domain use the exact analytical
evaluator without clamping or extrapolation.

## Task 3 round 1/5 remediation evidence

Focused regression coverage now includes out-of-domain exact torque limits, exact
regen nodes, all-endpoints-satisfy selection, mixed feasibility masks, and
non-monotone branch crossings. Exact fallback accounting is centralized, including
`operating_map=None`; nonzero torque never silently evaluates LUT load zero when
the directional envelope is nonpositive.

The inverse microbenchmark compares the prior repeated analytical evaluator plus
40-step bisection against the LUT inverse, in one process with 100,000 queries:

| operation | calls/query | microseconds/query |
|---|---:|---:|
| old exact evaluator + bisection | 41 | 627.18 |
| LUT inverse | 1 inverse + 1 forward | 68.06 |

The LUT inverse was 9.22x faster in this run. This is a motor-query baseline;
assembled comparison remains Task 4.

## Task 4 LUT assembly integration

The assembled path keeps analytical motor evaluation as the default and the
`motor_mode="lut"` path as an explicit opt-in. In LUT mode the drive limit solves
the monotone mapped bus-power row directly through `torque_for_bus_power`, the
regen inverse scans every productive segment of the non-monotone row
vectorized, and the inverse's own operating point is reused instead of being
recomputed. Any genuinely out-of-domain query (no map, voltage or speed outside
the map, a nonpositive directional envelope, a target below the mapped row's
zero-load power) uses the exact analytical fallback, which is observable through
the LUT counters.

The canonical map is the regenerated 25x129x65 NPZ with SHA-256
`31cfadc88b66d1425d210ad71cbc85ba0dea75564903014a1075efb7669d5fe9` and
generator `motor-lut-v2`. The voltage axis was extended from 448.848-608.688 V to
530-620 V to cover the observed loaded terminal voltage range (553.20-608.69 V)
with margin. It is reproducible byte-for-byte from:

```text
PYTHONPATH=src <bundled python3> tools/generate_motor_lut.py \
  models/power-plants/amk_dd5_100c.xml \
  models/data/amk_dd5_100c_operating_map.npz \
  --voltage-points 25 --speed-points 129 --load-points 65 --voltage-min 530 --voltage-max 620
```

`models/data/provenance.md` records the chosen grid and digest. Refining the
grid to 33x257x65 or 41x257x65 changes the assembled lap-time displacement by at
most 0.04 ms and the maximum pointwise speed displacement by at most
0.001 m/s, so the finer grids buy no gate margin at 3x-10x the file size.

The maintained comparison command is:

```text
PYTHONPATH=src <bundled python3> examples/benchmark_soc_solver.py --mode compare --runs 3
```

It performs one untimed warm-up and three same-process timed flying laps per
mode, resetting LUT counters before every lap so warm-up work is excluded. JSON
retains the full speed arrays and computes `max(abs(lut.speed -
analytical.speed))`, along with median/min wall time, lap delta, force residual,
query/fallback counters, and map-load cost.

### Measured result (bd861e4)

| Quantity | Analytical | LUT | Gate | Result |
|---|---|---|---|---|
| Lap time | 69.201286244 s | 69.21108300 s | delta <= 1 ms | +9.797 ms, **fail** |
| Max pointwise speed displacement | - | 0.134642 m/s | <= 0.05 m/s | **fail** |
| Converged | yes | yes | both converged | pass |
| Median wall time (runs=3) | 8.253 s | 4.394 s | LUT < analytical | **pass** (1.878x) |
| Force residual max | 6.821e-13 | 6.821e-13 | existing tolerances | pass |
| LUT forward / inverse / fallback queries | - | 105,166 / 24,856 / 0 | observable | pass (zero fallback) |

**The LUT is a genuine 1.878x wall-time win with zero fallback queries**, but
the 1 ms lap-time and 0.05 m/s speed-displacement gates do not pass. The
lap-time displacement is +9.797 ms and the maximum pointwise speed displacement
is 0.134642 m/s. Two numerical gates are still failing.

The LUT is faster but not numerically equivalent enough to promote, so
`PowertrainParameters.motor_mode` remains `"analytical"` and the LUT stays an
explicit opt-in. The speedup is real and the numerical displacement is real;
neither claim may be dropped.

An intermediate state in fix round 2 measured the LUT at 4.83 s against an
analytical 8.06 s (1.67x faster), but that state used the interpolated motor
envelope even when it exceeded the storage discharge limit. The speedup came
from a correctness defect (NEW-1) and is not a valid claim.

The microbenchmark for the fixed inverse, with the bundled interpreter and
`PYTHONPATH=src`:

| Arm | Before (33971fd) | After (9014a71) |
|---|---|---|
| In-map drive targets | 70.2 us/call, 0 fallback evals | 25.2 us/call, 0 fallback evals |
| Targets above the row maximum | 52,238 us/call, 3,110 evals/call | 22.0 us/call, 0 evals/call |
| Voltage below the map floor (437 V) | 52,591 us/call, 3,098 evals/call | 734 us/call, 40.6 evals/call |

The regen full-track comparison was not run: each mode exceeds the
900-second-per-mode budget.

### Numerical attribution of the residual displacement

The +9.797 ms lap displacement is systematic, not a grid-resolution problem
(grid refinement moves it by at most 0.038 ms). A converged analytical reference
(48 bisection steps with properly converged voltage coupling) was computed at a
representative sweep of 7 speeds x 5 SOC values (35 points). The error of each
mode against this reference was reported in
`.superpowers/sdd/2026-10-12-motor-lut/task-4-report.md` under the Task 7
attribution section. Key findings:

- The analytical default's drive limit bisects only 8 steps from
  `maximum_motor_torque`, which is **not always feasible** — at 14540 rpm and
  516.6 V it returns 19.622 Nm while the exact feasible limit is 12.710 Nm.
  The analytical drive limit is itself an approximation.
- The LUT mode is **not** numerically closer to the converged reference than
  the analytical default. At high speeds where the terminal voltage drops below
  the 530 V map floor, the LUT clamps to the map floor and reports feasible
  torque where the converged reference correctly returns zero. The analytical
  default returns 0 Nm (matching the reference) because its envelope is also
  infeasible at those voltages.
- On the attribution sweep the analytical default tracked the converged
  reference much more closely than the LUT did (force maximum error 2.63 Nm and
  95th percentile 2.07 Nm, against 443.98 Nm and 424.91 Nm for the LUT), because
  the LUT over-reports available force once the terminal voltage leaves the map.
- A controller-run side-by-side at matched speed and state of charge confirms
  that this over-reporting is a LUT defect rather than a coupling artifact of
  both modes. At SOC 0.30 and 34 m/s the LUT reported 2070.4 N of available
  drive force while the analytical default reported 0.0 N; the same pattern
  appeared at SOC 0.15 (30 and 34 m/s), SOC 0.08 (30 and 34 m/s), SOC 0.04
  (30 m/s) and SOC 0.02 (25, 30 and 34 m/s).
- The +9.797 ms lap displacement is therefore dominated by LUT error, not by
  the analytical default's 8-step bisection being replaced by an exact inverse.
  The LUT must not be promoted, and its 1.878x wall-time result holds only for
  the in-domain part of the operating envelope that the assembled MIS flying lap
  visits (roughly 555-609 V). Making the LUT usable for the SOC-coupled
  endurance case requires covering the loaded terminal voltage range and
  re-passing the numerical gates.

## Reference checks and assumptions

Legacy MIS flying and standing-start golden results remain 69.22653728547469 s
and 70.38955050759907 s. The assembled flying reference is 69.20462625809404 s (was 69.20128624364081 s before the Task 8 per-motor discharge-limit fix)
s. The motor-speed ceiling is 34.60897 m/s, while the MIS peak is below that
ceiling; MIS is therefore not a voltage-field-weakening regression track.

The model assumes quasi-static battery terminal voltage, fixed ratio and motor
count, reference iron/mechanical losses, planar unsigned-radius track geometry,
and a perfect-limit point-mass driver. Full state-coupled 22-lap regen remains
an open performance/validation item.
