# SDD ledger — plan: docs/superpowers/plans/2026-10-12-soc-solver-performance.md

Base commit before implementation: 24d1551

## Preflight consistency scan

| Scope | Producer / consumer relationship | Finding |
|---|---|---|
| Task 1 | Direction-specific scalar limits consumed by forward/backward sweeps | Consistent; full `acceleration_limits` remains for compatibility. |
| Task 2 | Root/corner/profile hints consumed later by Task 4 | Consistent; all hints must retain a valid bracket and be optional. |
| Task 3 | Solver-local cache policy consumed by Tasks 4 and 6 | Consistent; exact zero-resolution default preserves behavior, quantization stays opt-in unless benchmark gates pass. |
| Task 4 | Inner controls and hint reuse consume Tasks 2–3 interfaces | Consistent; energy and lap iteration settings remain separate. |
| Task 5 | Progress callback consumed by Task 6 benchmark | Consistent; callback is observational and cannot alter numerics. |
| Task 6 | Benchmark consumes all earlier interfaces | Consistent; numerical gates determine whether approximate settings remain opt-in. |
| Tasks 1 ↔ 2 (`point_mass.py`, `forward_backward.py`) | Directional kernels are established before root/hint changes | No conflict; Task 2 must preserve Task 1 call separation. |
| Tasks 1 ↔ 3 (`point_mass.py`, `forward_backward.py`) | Directional kernels must accept the solver-local cache context | Interface detail not explicit in Task 1. Ruling recorded below. |
| Tasks 1 ↔ 4 (`forward_backward.py`) | Energy layer later supplies hints/controls to Task 1 sweeps | No conflict. |
| Tasks 2 ↔ 3 (`point_mass.py`, `forward_backward.py`) | Hinted roots must use the same per-solve cache | No conflict; cache context threads through hinted calls. |
| Tasks 2 ↔ 4 (`forward_backward.py`) | Warm-start interface is required by coupled-lap reuse | No conflict. |
| Tasks 2 ↔ 5 (`forward_backward.py`) | Progress reporting wraps profile work without changing hints | No conflict. |
| Tasks 3 ↔ 4 (`forward_backward.py`) | Cache policy is forwarded into all coupled passes | No conflict. |
| Tasks 3 ↔ 6 (`test_mis_regression.py`) | Exact/quantized policies are benchmarked before any default decision | No conflict. |
| Tasks 4 ↔ 5 (`forward_backward.py`, `test_energy_lap.py`) | Progress events report coupled iterations after controls are explicit | No conflict. |
| Tasks 4 ↔ 6 (`test_mis_regression.py`) | Warm coupled mode receives performance and numerical gates | No conflict. |
| Tasks 5 ↔ 6 (`powertrain.md`) | Progress enables non-opaque long benchmark reporting | No conflict. |
| Task 1 internal | Tests compare directional and combined limits; implementation preserves diagnostic API | Consistent. |
| Task 2 internal | Tests cover hints above/below roots and warm/cold equality | Consistent. |
| Task 3 internal | Default exact cache and opt-in quantization are both tested | Consistent. |
| Task 4 internal | Mock propagation plus regen numerical equivalence cover interface and behavior | Consistent. |
| Task 5 internal | Event ordering and no-callback behavior cover observational contract | Consistent. |
| Task 6 internal | Exact baseline, regen benchmark, full suite, and documentation gates align | Consistent. |

Ruling: Thread an optional private cache context through Task 1’s scalar kernels even though the public directional signatures stay as specified — this enables Task 3 without another duplicate kernel; if wrong, Task 3 may require a narrow signature adjustment.

Task 1: complete (commits 24d1551..7acfe58, review clean)
Task 2: recovery dispatch — first GPT-5.6 Luna implementer was interrupted after >25 minutes silent with uncommitted changes and no report; fresh implementer must inspect, test, repair, and complete them.
Task 2: review failed — assembled MIS unverified/>120 s; non-monotone upward bisection unsafe; descent silently accepts nonconvergence; corner root may alter legacy lower bound.
Task 2: Ruling: correctness and sub-30-second assembled runtime override transition/profile warm-start acceleration; retain only bracket-safe positive-root/corner hints and restore the original conservative transition iteration — cost if wrong: one of the eight proposed optimizations remains unsupported until a safe non-monotone algorithm is designed.
Task 2: fix round 1/5 (5 addressed, 0 open — commits a446a25..00ca24b)
Task 2: complete (commits 7acfe58..00ca24b, review clean under ruling)
Task 3: review failed — cache missing from final-result equilibrium; MIS SOC comparison incomplete; reviewer interpreted 37.219 s two-policy test as one-lap runtime.
Task 3: fix round 1/5 (3 addressed, 0 open — commits ae5bd9d..caf3761)
Task 3: complete (commits 00ca24b..caf3761, review clean)
Task 4: review failed — same-shaped hints lack vehicle/track/mode context compatibility guard.
Task 4: fix round 1/5 (1 addressed, 0 open — commits 5ced6b7..230a0fa)
Task 4: complete (commits caf3761..230a0fa, review clean)
Task 5: complete (commits 230a0fa..e4861a4, review clean)
Task 6: review failed — full-track cold/warm regen benchmark did not complete; validation commands contain placeholder/unpinned interpreter paths.
Task 6: fix round 1/5 (2 addressed, 1 new open — docs conflate unsupported transition/profile warm start with supported corner-root hints; commits 2caaedd..4a164f2)
Task 6: fix round 2/5 (1 addressed, 0 open — commits 4a164f2..d754139)
Task 6: complete (commits e4861a4..d754139, review clean)
Final review: CHANGES_REQUIRED — root_tolerance not threaded into directional scalar roots; plan checklist stale; temporary benchmark scripts not reproducible. Reviewer accepts transition-profile warm-start ruling and treats quantized SOC/regen plus 22-lap completion as documented limitations.

Final fix wave: complete — directional acceleration/deceleration roots now accept and receive `root_tolerance` (default remains `1e-8`), with sweep propagation tests; plan checklist now labels historical status and marks corner hints complete while marking transition/profile initial-speed acceleration ruled out/unsupported; maintained `examples/benchmark_soc_solver.py` replaces temporary benchmark commands and keeps 22-lap mode explicit. Quantized caching remains opt-in and 22-lap evidence remains incomplete.
