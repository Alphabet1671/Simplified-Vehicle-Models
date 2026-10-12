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
  python examples/benchmark_soc_solver.py --mode exact
```

The exact run converged in 2 iterations. Wall time was 23.77 s (`real`),
with 22.91 s measured around `solve_lap`; lap time was 69.20128624364081 s.
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
  python examples/benchmark_soc_solver.py --mode regen-one --progress
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
PYTHONPATH=src python examples/benchmark_soc_solver.py --mode regen-22 --progress
```

It was stopped at the practical 120 s limit. Progress evidence: lap 1
`lap_start` at 0 s; energy iterations 1, 2, and 3 at 22.08, 59.92, and
98.17 s. No lap completed, so completed laps = 0 and no stint totals are
reported. This lower bound demonstrates the remaining runtime limitation;
it is not evidence of convergence or failure.

The existing fast endurance study with `reuse_profile=True` is a fixed-profile
approximation and must not be read as fully state-coupled regen validation.

## Reference checks and assumptions

Legacy MIS flying and standing-start golden results remain 69.22653728547469 s
and 70.38955050759907 s. The assembled flying reference remains 69.20128624364081
s. The motor-speed ceiling is 34.60897 m/s, while the MIS peak is below that
ceiling; MIS is therefore not a voltage-field-weakening regression track.

The model assumes quasi-static battery terminal voltage, fixed ratio and motor
count, reference iron/mechanical losses, planar unsigned-radius track geometry,
and a perfect-limit point-mass driver. Full state-coupled 22-lap regen remains
an open performance/validation item.
