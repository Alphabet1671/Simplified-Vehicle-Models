# Task 6 report: benchmark, tune, and document

## Status

Implemented and documented Task 6. Exact assembled mode remains the default;
quantized drive caching remains explicit opt-in because its SOC/regen promotion
gates remain incomplete. The exact full-track cold/warm regen benchmark
completed within 900 s per run. The quantized SOC/regen gates and the fully
coupled 22-lap stint remain incomplete. No merge or push was performed.

## Step 1: exact assembled MIS flying lap

Command:

```text
PYTHONPATH=src /usr/bin/time -p \
  /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 \
  -m cProfile -o /tmp/task6_exact.prof /tmp/task6_exact.py
```

The script loaded `murphy_point_mass.xml` and the Michigan track, then called
`solve_lap(..., LapMode.FLYING, tolerance=1e-6)` with the default exact cache
policy. Results:

| metric | result |
|---|---:|
| `/usr/bin/time` real | 23.77 s |
| measured solve wall | 22.91 s |
| user / sys | 20.97 / 0.62 s |
| lap time | 69.20128624364081 s |
| converged | true |
| iterations | 2 |
| nodes | 1,407 |
| exact-cache hits / entries | 189,763 / 26,723 |
| max speed/SOC bucket displacement | 0 / 0 |
| max force-balance residual | 6.82e-13 N |

The cProfile output (`/tmp/task6_exact.prof`) reported cumulative time of
17.060 s in equilibrium evaluation, 17.031 s in corner-speed evaluation,
16.603 s in available-drive/drive-limit evaluation, 11.169 s in shaft-power
evaluation, and 3.586 s in the forward sweep. These overlap as cumulative
call-tree times; they are not additive wall-clock components.

## Step 2: regen one-lap cold/warm benchmark

Command (same explicit controls for both runs):

```text
PYTHONPATH=src /usr/bin/time -p \
  /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 \
  /tmp/task6_energy_round1.py > /tmp/task6_energy_round1_fixed.log 2>&1
```

The controls were `tolerance=1e-5`, `maximum_iterations=50`,
`lap_tolerance=1e-6`, `lap_maximum_iterations=200`, `root_tolerance=1e-8`,
`regen_enabled=True`, and `drive_cache_policy=DriveCachePolicy()` (exact
cache). The script enabled progress callbacks and ran cold first, then warm
with cold's `initial_speed` and `corner_speed_hint`. Transition/profile
`initial_speed` acceleration is unsupported and ignored; only the
bracket-preserving `corner_speed_hint` is supported. The corner hint is exact,
not approximate profile reuse, and is the source of any warm-run work
reduction.

Exact command output:

```text
cold PROGRESS SolverProgress(stage='lap_start', lap=1, lap_count=1, energy_iteration=0, profile_iteration=0, elapsed_seconds=1.6669509932398796e-06)
cold PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=1, profile_iteration=2, elapsed_seconds=27.310426124953665)
cold PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=2, profile_iteration=2, elapsed_seconds=74.05481541692279)
cold PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=3, profile_iteration=2, elapsed_seconds=120.24454762495589)
cold PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=4, profile_iteration=2, elapsed_seconds=166.91425274999347)
cold PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=5, profile_iteration=2, elapsed_seconds=209.47308087488636)
cold PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=6, profile_iteration=2, elapsed_seconds=251.9354197079083)
cold PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=7, profile_iteration=2, elapsed_seconds=298.1986129999859)
cold PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=8, profile_iteration=2, elapsed_seconds=345.0519679579884)
cold PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=9, profile_iteration=2, elapsed_seconds=388.7729828329757)
cold PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=10, profile_iteration=2, elapsed_seconds=431.52968474989757)
cold PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=11, profile_iteration=2, elapsed_seconds=473.82718708296306)
cold PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=12, profile_iteration=2, elapsed_seconds=517.7851929169847)
cold PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=13, profile_iteration=2, elapsed_seconds=563.120865332894)
cold PROGRESS SolverProgress(stage='lap_complete', lap=1, lap_count=1, energy_iteration=13, profile_iteration=2, elapsed_seconds=611.132667082944)
cold RESULT wall_s= 611.1328564169817 lap_time_s= 69.1982147152843 lap_converged= True energy_converged= True energy_iterations= 13 profile_residual= 2.5795463698408128e-06 final_soc= 0.9790593110040156 consumed_j= 837888.6096674838 recovered_j= 403.4186193658725 net_j= 837485.1910481179 minimum_voltage_v= 603.907809990235 lap_iterations= 2 cache_hits= 165954 cache_entries= 51104 max_speed_displacement= 0.0 max_soc_displacement= 0.0
warm PROGRESS SolverProgress(stage='lap_start', lap=1, lap_count=1, energy_iteration=0, profile_iteration=0, elapsed_seconds=1.500011421740055e-06)
warm PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=1, profile_iteration=2, elapsed_seconds=46.085524082998745)
warm PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=2, profile_iteration=2, elapsed_seconds=93.13673479098361)
warm PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=3, profile_iteration=2, elapsed_seconds=140.9260882079834)
warm PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=4, profile_iteration=2, elapsed_seconds=183.38888391596265)
warm PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=5, profile_iteration=2, elapsed_seconds=217.02430420799647)
warm PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=6, profile_iteration=2, elapsed_seconds=244.57882520800922)
warm PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=7, profile_iteration=2, elapsed_seconds=271.3651812499159)
warm PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=8, profile_iteration=2, elapsed_seconds=297.85469762491994)
warm PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=9, profile_iteration=2, elapsed_seconds=324.80754308297765)
warm PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=10, profile_iteration=2, elapsed_seconds=352.9058809999842)
warm PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=11, profile_iteration=2, elapsed_seconds=380.24149916600436)
warm PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=12, profile_iteration=2, elapsed_seconds=407.43720658298116)
warm PROGRESS SolverProgress(stage='energy_iteration', lap=1, lap_count=1, energy_iteration=13, profile_iteration=2, elapsed_seconds=434.072840290959)
warm PROGRESS SolverProgress(stage='lap_complete', lap=1, lap_count=1, energy_iteration=13, profile_iteration=2, elapsed_seconds=460.80955200002063)
warm RESULT wall_s= 460.80963879206683 lap_time_s= 69.1982147152843 lap_converged= True energy_converged= True energy_iterations= 13 profile_residual= 2.579546369951835e-06 final_soc= 0.9790593110040156 consumed_j= 837888.6096674838 recovered_j= 403.4186193658725 net_j= 837485.1910481179 minimum_voltage_v= 603.907809990235 lap_iterations= 2 cache_hits= 165954 cache_entries= 51104 max_speed_displacement= 0.0 max_soc_displacement= 0.0
DELTA cold_warm_wall_s= -150.32321762491483 cold_warm_lap_time_s= 0.0 cold_warm_final_soc= 0.0 cold_warm_consumed_j= 0.0 cold_warm_recovered_j= 0.0 cold_warm_profile_residual= 1.1102230246251565e-16
real 1072.91
user 995.89
sys 28.11
```

Summary: both runs converged in 13 energy iterations and two inner profile
iterations. Warm was 150.323218 s (24.6%) faster; lap time, final SOC,
consumed energy, recovered energy, and profile residual were numerically
unchanged to displayed precision. Each run was below the allowed 900 s.

## Step 3: candidate selection

The explicit candidate was:

```python
DriveCachePolicy(speed_resolution=1e-3, soc_resolution=1e-5)
```

Command:

```text
PYTHONPATH=src /usr/bin/time -p \
  /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 \
  /tmp/task6_quant.py
```

Results: 6.52 s real, 69.20126421237728 s lap time, converged in 2
iterations, 209,479 cache hits, 7,007 entries, and maximum speed displacement
9.9017e-4 m/s. Compared with exact, the speedup was 23.77/6.52 = 3.64x and
lap-time delta was -2.2031e-05 s, passing the <0.01 s lap gate. However, SOC
and recovered-energy deltas were not measured because the quantized candidate's
full SOC/regen gates were not completed. The exact full-track cold/warm
benchmark above did complete. Therefore the candidate is not promoted and
exact remains the default. The test suite now also asserts that the exact
default reports zero speed/SOC approximation displacement.

Promotion gates used: assembled lap-time delta <0.01 s, final-SOC delta
<1e-4, recovered-energy delta <0.5%, and matching convergence/constraints.

## Step 4: fully coupled 22-lap regen stint

Managed background command:

```text
PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 /tmp/task6_stint.py
```

The call used `solve_stint(..., laps=22, regen_enabled=True,
reuse_profile=False, progress_callback=...)` and was stopped at a practical
120 s limit. Progress was:

```text
lap 1 lap_start: 0.000 s
lap 1 energy_iteration 1: 22.0782 s
lap 1 energy_iteration 2: 59.9162 s
lap 1 energy_iteration 3: 98.1738 s
```

No `lap_complete` event occurred: completed laps = 0, and no final stint
result exists. The report records this lower bound and latest event rather
than claiming a timeout result. The existing `reuse_profile=True` endurance
path is a fixed-profile approximation and is not substituted for this
fully coupled evidence.

## Step 5: full suite

Command:

```text
PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest discover -s tests -v
```

Output summary:

```text
Ran 213 tests in 178.087s
OK
```

## Step 6: files changed

* `tests/test_mis_regression.py`: added an exact-default assertion that no
  speed/SOC approximation displacement is reported.
* `docs/validation/powertrain.md`: replaced stale runtime claims with exact
  and quantized measurements, cProfile component accounting, promotion gates,
  regen progress evidence, and remaining runtime limitation.
* `.superpowers/sdd/2026-10-12-soc-solver-performance/task-6-report.md`:
  this full benchmark report.

## Self-review and concerns

* `git diff --check` passed before commit.
* Exact and quantized lap results converged and preserved the force residual;
  only the quantized candidate's lap-time gate is validated.
* Exact one-lap cold/warm regen completed, but the quantized candidate was not
  rerun through the full SOC/regen gates; therefore no accelerated default was
  promoted. The 22-lap fully coupled regen attempt remains incomplete and is
  the principal open performance concern.
* Profile component numbers are cumulative cProfile values and intentionally
  are not added together.
* No generated benchmark files were added to the repository; temporary
  scripts/profile output were under `/tmp`.
