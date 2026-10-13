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
`166bf1346d263267224f77fe7396ab15180e0c2f54391652fee28f8575fcfc5b` and
generator `motor-lut-v4` (drive/regen envelopes capped at the voltage-scaled map envelope, Task 8; voltage axis 530-600 V because bus voltage is clamped to `maximum_bus_voltage` = 600 V, Task 9; the Task 8 map was `0fe463ea...1eeb9` on 530-620 V). It is reproducible byte-for-byte from:

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

### Measured result (bd861e4) - SUPERSEDED by Tasks 8 and 9 (see the Task 9 section for current figures)

| Quantity | Analytical | LUT | Gate | Result |
|---|---|---|---|---|
| Lap time | 69.201286244 s | 69.21108300 s | delta <= 1 ms | +9.797 ms, **fail** (superseded) |
| Max pointwise speed displacement | - | 0.134642 m/s | <= 0.05 m/s | **fail** |
| Converged | yes | yes | both converged | pass |
| Median wall time (runs=3) | 8.253 s | 4.394 s | LUT < analytical | **pass** (1.878x, superseded) |
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
and 70.38955050759907 s. The assembled flying reference is 69.20459210208878 s (Task 9; was 69.20462625809404 s after the Task 8 fix and 69.20128624364081 s before it)
s. The motor-speed ceiling is 34.60897 m/s, while the MIS peak is below that
ceiling; MIS is therefore not a voltage-field-weakening regression track.

The model assumes quasi-static battery terminal voltage, fixed ratio and motor
count, reference iron/mechanical losses, planar unsigned-radius track geometry,
and a perfect-limit point-mass driver. Full state-coupled 22-lap regen remains
an open performance/validation item.


## Task 8: unit-bug fix, voltage-scaled envelope, residual attribution

Three defects were diagnosed on 17f8d69 and fixed in separate commits.

**A. Per-motor power compared with the pack limit (analytical default).**
`maximum_drive_force` compared per-motor `bus_power` with the PACK
`discharge_power_limit`, so each motor could draw the whole pack limit (17 of 378
points over the limit; worst SOC 1.0 / 17.5 m/s: 102,775 W vs 100,495 W). The
bug pre-dates the LUT (present at 9aedaf6). The analytical branch now compares
with `discharge_power_limit / unit_count`, as the LUT branch always did. The
legacy lumped path clamps total bus power to the pack limit and does not have
this bug. **The canonical assembled flying lap changes from 69.20128624364081 s
to 69.20462625809404 s (+3.34 ms); this is intended.** Regression test:
`test_drive_limit_never_exceeds_pack_discharge_limit_in_both_modes`.

**B. Voltage-scaled envelope.** The analytical path bisects inside
`maximum_motor_torque(V/sqrt(3))`, which scales the 600 V map speed axis and is a
hard zero once the scaled speed passes 20,000 rpm. The generator and the
out-of-map `_exact_limit` used only electrical feasibility. Both now use
`voltage_scaled_envelope` (= `maximum_motor_torque(V/sqrt(3))`), so
drive = min(voltage-scaled envelope, electrical feasibility). The regen envelope
gets the same cap: the analytical regen path uses the same
`maximum_motor_torque(V/sqrt(3))` as its torque limit, so it is the reference.
The map was regenerated (generator `motor-lut-v3`, SHA-256 `0fe463ea...1eeb9`;
two regenerations are byte-identical).

*Cliff interpolation.* The cap drops from ~10-20 Nm to 0 along a diagonal in
(V, speed). Bilinear interpolation across such a cell smeared the step by up to
10.4 Nm over / 9.5 Nm under the exact value (3,614 of 4,991 probe points in the
17,000-18,600 rpm, 530-560 V region differed by more than 1e-6 Nm). That is
significant, so the 14 of 3,072 cells that straddle a zero/non-zero envelope
corner are never interpolated: `_envelope_cliff_cell` routes them to the exact
oracle (forward, limits, inverse). After that, no LUT limit exceeds the cap by
more than 1e-5 Nm above 300 rpm (measured 0.1359 Nm at most, at 530 V / 88.35 rpm, below 300 rpm - an earlier 0.108 Nm figure was wrong - from interpolation of
the source envelope's own breakpoints); everywhere the cap is 0 the LUT returns 0.

**C. Residual attribution** (after A and B; see task-4-report.md for numbers).
The +9.797 ms / 0.1346 m/s displacement (and the 1.878x speedup quoted for bd861e4, both superseded) is gone: LUT 69.20391185 s vs analytical
69.20462626 s (-0.714 ms), max displacement 0.004999 m/s. The remaining
difference is an accepted approximation difference, dominated by the analytical
8-step bisection (a lower bound on the torque), not a defect.


## Task 9: regulatory bus-voltage clamp (`maximum_bus_voltage`)

**Root cause.** `evaluate_shaft_power_exact` adds a field-weakening correction
`R*1.5*(id(V)^2 - id(600 V)^2)`, clamped at zero below the 600 V motor
identification voltage but large and positive above it (the smallest-magnitude
d-axis root grows with available voltage at low speed). At zero torque and
145 rpm the per-motor bus draw was 3.58 W at 560-600 V but 3,004 W at 601.25 V
and 48,762 W at 620 V; at the SOC 1.0 OCV of 608.688 V it was 21,780 W at 0 rpm.
Drive bus power therefore fell with torque at low speed and the generator's
monotone-drive bisection collapsed the envelope to ~1e-10 Nm at 26 map nodes
(601-620 V, 0-581 rpm) where the exact limit is 21.4-21.8 Nm. The analytical
default was affected too whenever terminal voltage exceeded 600 V (SOC > ~0.96).

**Decision (user).** 600 V is the regulatory DC bus voltage limit; exceeding it
is a disqualification. The power-plant XML parameter `maximum_bus_voltage`
(unit V, 600 for the AMK DD5-100C) is compiled into `PowerPlantParameters`
(validated positive and finite). It is the regulatory limit and is deliberately
separate from `reference_bus_voltage` (the voltage at which the motor was
identified); both are 600 V today but the former can become 800 V.

**Choke point.** `limit_bus_voltage(parameters, v) = min(v, maximum_bus_voltage)`.
It is applied inside `calculate_available_voltage` and `maximum_motor_torque`
(which every analytical path, the voltage-scaled envelope and the exact oracle
call), at the top of `evaluate_shaft_power_exact`, `_lut_forward`,
`motor_torque_limits`, `torque_for_bus_power`, `lut_bus_power_range` and
`_exact_limit`. Because the clamp lives in the primitives, out-of-map fallbacks,
the generator and the assembly (which now calls `voltage_scaled_envelope`
instead of re-deriving `V/sqrt(3)`) cannot bypass it. Battery OCV, resistance,
current and SOC are not clamped; only what the motor and inverter see is.

**Disclosures.** The canonical pack's 608.7 V OCV at SOC 1.0 itself exceeds the
600 V limit; battery data were not changed. Raising `maximum_bus_voltage` above
the 600 V motor reference (e.g. 800 V) would re-expose the above-reference
field-weakening correction defect, which then needs a source-model fix (d-axis
root choice) before use.

**Map.** 530-600 V x 0-18,600 rpm x 65 loads, `motor-lut-v4`, SHA-256
`166bf1346d263267224f77fe7396ab15180e0c2f54391652fee28f8575fcfc5b`; two
regenerations are byte-identical. No near-zero drive node remains below the
17,700 rpm cap corner.

**Golden lap.** The assembled analytical flying lap changes from
69.20462625809404 s to 69.20459210208878 s (-0.034 ms) because the flying lap
starts at SOC 1.0 (608.7 V terminal), which is now clamped to 600 V for the
motor. Pinned in `tests/test_mis_regression.py`.

**Review findings closed.** Cliff detector now uses a 1e-6 Nm tolerance instead
of exact 0.0; the envelope test is two-sided against the exact limit; low-speed
overshoot is 0.1359 Nm (test slack 0.14); `motor_torque_limits` fallbacks are
counted in `limit_fallback`; stale 1.878x / +9.797 ms figures are marked
superseded.

**Task 9 measurements** (`examples/benchmark_soc_solver.py --mode compare --runs 3`, run alone):
analytical median wall 9.007 s, lap 69.20459210 s; LUT median wall 5.193 s
(1.735x), lap 69.20391204 s; lap delta -0.680 ms; max pointwise speed
displacement 0.003946 m/s; both converged; LUT counters forward / inverse /
fallback 104,716 / 24,619 / 1,725. Correctness sweep (SOC 1.0-0.02, 0-38 m/s,
693 points per mode): 0 pack-limit violations in both modes, 0 LUT limits above
the clamped voltage-scaled cap (1,890 limit queries), max |dF| 16.55 N, mean
2.82 N, 0 zero-versus-nonzero force mismatches.
