# Task 6 report: benchmark, tune, and document

## Status

Implemented and documented Task 6. Exact assembled mode remains the default;
quantized drive caching remains explicit opt-in because the full SOC/regen
numerical gates could not be completed within practical limits. No merge or
push was performed.

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

Command:

```text
PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 /tmp/task6_energy.py
```

The script enabled regen and progress callbacks, then attempted cold and warm
(`initial_speed` and `corner_speed_hint` from cold) full-track one-lap runs.
The cold run was stopped at the practical 300 s limit before a lap completed.
Progress evidence was:

```text
lap_start: 0.000 s
energy_iteration 1: 14.0406 s
energy_iteration 2: 42.7343 s
energy_iteration 3: 85.4262 s
energy_iteration 4: 132.0146 s
energy_iteration 5: 172.5833 s
energy_iteration 6: 208.6798 s
energy_iteration 7: 245.5776 s
energy_iteration 8: 283.0033 s
```

There was no `lap_complete`, so no converged energy iteration count, profile
residual, final SOC, consumed/recovered energy, or lap time is reported. The
warm run did not start after cancellation. This is an observed lower bound,
not an opaque timeout and not evidence of numerical failure.

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
and recovered-energy deltas were not measured because the full regen benchmark
did not finish. Therefore the candidate is not promoted and exact remains the
default. The test suite now also asserts that the exact default reports zero
speed/SOC approximation displacement.

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
* One-lap regen and 22-lap fully coupled regen did not complete under managed
  limits, so no accelerated default was promoted. This is the principal open
  performance concern.
* Profile component numbers are cumulative cProfile values and intentionally
  are not added together.
* No generated benchmark files were added to the repository; temporary
  scripts/profile output were under `/tmp`.
