# Task 3 report — Throughput gate

## Outcome
**GATE FAIL.** Do not continue into Task 4. Best measured valid method is cosim-radau-loose: 2.4228 wall/sim-s on planar Catalunya, while planar FSC is 3.3975 and 14DoF FSC is 7.5234; these are far above search target 0.15 and validation target 0.40. Based on the conservative workload estimate, best fully measured Catalunya case projects 442.8 s/generation and 3,898 s for an 8-generation cell including seed and refresh. Set status to DONE_WITH_CONCERNS.

## Implemented
- Added method selection in `tools/run/benchmark_rollout.py`: `cosim-radau`, `cosim-radau-loose`, and `cosim-persistent-implicit`.
- Registered `radau` and `radau-loose` in `tick_integrators.py`; deadline is checked before and after every solve. E3 persistent-implicit was not registered because its experimental implementation failed accuracy.
- Added `tests/test_evolution_integrators.py` comparing all registered integrators with 2 s, 2 ms Radau reference and deadline check.
- The attempted finite-difference implicit method failed accuracy materially; rolled it back. `persistent-implicit` is currently compatibility fallback to Radau, and MUST NOT be described as a persistent implicit implementation or selected for performance.

## TDD evidence / tests
`PYTHONPATH=src:. python3 -m unittest tests.test_evolution_integrators -v`: 2 tests pass (95.5 s). An initial experimental implicit attempt failed with final errors [37.867 m, 0.00206 m, 0.00685 m/s, 0.5758 m/s, 0.0804 rad, 0.0276 rad/s]; removed. The finished test's Radau and Radau-loose comparison passes brief tolerances. E0 overhead measure at 100 Hz 0.01345 wall/sim-s, around 0.22% of E1. E4 profile records dominant local costs; no optimization was safe/attempted.

## Measurements and decision evidence
Detailed per-method/model/track ratios, accuracy status, workload assumptions, startup/pool measurements, all G computations and missing measurements are in `bench/throughput-decision.md`. Key 10 s ratios: planar FSC E1 6.1147 and E2 3.3975; planar Catalunya E2 2.4228; 14DoF FSC E2 7.5234; 14DoF Catalunya E2 4.9191; hydraulic 14DoF Catalunya E2 5.5538. The MIS and hydraulic FSC/MIS measurements remain missing due budget limits, so cannot convert those into fabricated gate-pass values; decision is already FAIL on measured cells.

Workload assumptions include 38 complexes, lambda 4, J 152, 100% lap corner-limited, QSS lap times FSC 53.5/MIS 62.6/Catalunya 73.4008s, 40 m approach and 30 m exit, Passthrough replay 0, and measured rollout speed proxy for unavailable QSS mean speed. Startup 0.181s and pool spawn/teardown 0.071s measured; no worker-context build. T_ipc/final write/teardown optimistically zero. These assumptions are documented in detail in throughput-decision.md.

## Concerns / files
- Critical: persistent-implicit E3 is not actually implemented and is Radau-backed; no claimed speed benefit.
- No E5 search-plant implementation/ranking test; E1–E4 are inadequate, so this is the obvious re-plan.
- Incomplete acceptance matrix; missing cases listed in decision doc.
- `tools/run/README.md`, `src/simplified_vehicle_models/evolution/README.md`, and `tests/README.md` need update for newly added/changed files.
- Files changed: `src/simplified_vehicle_models/evolution/tick_integrators.py`, `tools/run/benchmark_rollout.py`, `tests/test_evolution_integrators.py`, three local READMEs, `bench/throughput-decision.md`, and this report.
- Commit not yet created. No push.
