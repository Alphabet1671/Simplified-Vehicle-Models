# Task 3 throughput decision

**Machine:** 8 CPUs, macOS arm64 (Python 3.13). Benchmarks were run sequentially; each subprocess was bounded to 110 s (hard maximum 120 s). These are single-run wall/simulated-second observations, not confidence intervals. Controller is PassthroughTV; cosim benchmark uses 10 ms controller ticks. Search and validation are both full plant because E5 was not completed.

## Experiments

| Experiment/method | Planar FSC ratio (wall/sim-s) | Result |
|---|---:|---|
| E0: trivial linear 10 ms Radau construction+solve | 0.01345 projected at 100 Hz | 0.22% of E1 cosim-Radau wall; does not disqualify |
| E1 cosim-radau (controller outside RHS) | 6.1147 | 10 s completed; 61.147 s wall |
| E2 cosim-radau-loose (rtol=1e-4, atol=1e-6) | 3.3975 | 10 s completed; 33.975 s wall |
| E3 persistent-implicit | 6.1817 experimental placeholder reading only | Experimental attempt failed accuracy and was removed; not registered/available for selection |
| E4 profile, plant.evaluate ×2,000 | 1.500 ms/call | Top cumulative component was planar dynamics (1.116 ms/call); Unitire was 0.657 ms/call, powertrain 0.334 ms/call including brentq 0.146 ms/call. No source optimization attempted; outputs unchanged by no-op. |
| E5 reduced search plant | — | Not attempted; E1–E4 clearly miss target and E5 ranking validation is not available in candidate code. |

The accuracy test compared terminal [s,n,v_y,v_x,steering,yaw_rate] against 2 s dt=2 ms Radau reference using brief tolerances. At dt=10 ms, radau and radau-loose passed. `persistent-implicit` is not registered because the initial experiment did not meet tolerances. The initial experimental finite-difference linearly-implicit implementation failed: errors [37.867 m, 0.00206 m, 0.00685 m/s, 0.5758 m/s, 0.0804 rad, 0.0276 rad/s]; it was removed rather than claimed as valid.

## Acceptance measurements

Ratios below measured using the selected valid method cosim-radau-loose. `r_search = r_val` because no E5 search plant was implemented. Comparison ratio against the Task 1 baseline is shown in parentheses.

| Model class | Track | r_search = r_val | Baseline ratio |
|---|---|---:|---:|
| Planar 7DoF | FSC | 3.3975 | Task 1 baseline 3.6996; 0.918x |
| Planar 7DoF | MIS | not measured | Task 1 baseline 3.2435; conservative proxy only |
| Planar 7DoF | Catalunya | 2.4228 | not previously measured in baseline matrix |
| 14DoF | FSC | 7.5234 | Task 1 baseline 5.1668; 1.456x |
| 14DoF | MIS | not measured | no Task 1 baseline |
| 14DoF | Catalunya | 4.9191 | not previously measured in baseline matrix |
| 14DoF + TTC tire + hydraulic suspension | FSC | not measured | — |
| 14DoF + TTC tire + hydraulic suspension | MIS | not measured | — |
| 14DoF + TTC tire + hydraulic suspension | Catalunya | 5.5538 | — |

Accuracy errors: radau and radau-loose final state differences were within [0.5 m, 0.05 m, inf, 0.1 m/s, inf, 0.02 rad/s] tolerance against reference (test pass). Explicit per-axis errors were not emitted by the test. Tests report numerical pass/fail only.

## Startup and pool

Interpreter+NumPy/SciPy imports+planar formulation compile: 0.181 s in a subprocess (track preparation is done in benchmark process; not separately isolated). Four-worker multiprocessing spawn+pool teardown without worker context build: 0.071 s. No worker context build measurement is available; treating measured pool cost as a lower bound understates startup cost.

## Workload model

Using brief formula with complex count 38 (MIS, longest measured track), λ=4, J=152, `ceil(J/4)=38`. QSS lap times: FSC 53.5 s, MIS 62.6 s, Catalunya 73.4008 s. Corner-limited fraction and candidate window are not yet measured; conservatively assume 100% lap as corner-limited for workload, so W_job = QSS lap time / 38 + (40 m approach + 30 m exit)/mean QSS speed. Mean QSS speed is unavailable, so use the measured 10 s rollout distance divided by 10 s as proxy (FSC planar 15.17 m/s; Catalunya planar 22.10 m/s; FSC 14DoF 15.16 m/s; Catalunya 14DoF 22.11 m/s; hydraulic Catalunya 22.66 m/s). Approach replay = 0 s (PassthroughTV has no state). Thus W_job proxies: FSC planar 5.47 s, Catalunya planar 4.79 s, FSC 14DoF 5.47 s, Catalunya 14DoF 4.78 s, hydraulic Catalunya 4.69 s. MIS uses proxy speed 15 m/s => 6.34 s. This deliberately overstates lap-window fraction but the speed proxy is not a QSS lap mean.

Assume T_startup=0.252 s measured (startup+pool lower bound), T_ipc=0 (unmeasured optimistic), T_final_write+teardown=0 (optimistic), T_seed_lap = r_val*L, K=1 periodic refresh validated lap, and reserve this final lap within the 300 s budget. Formula solved for max G as floor((300 - startup - (1+K)*r_val*L) / (ceil(J/4)*r_search*W_job + T_ipc)). The brief's required G≥8 is a per-cell feasibility requirement; the computed value is available generations after one seed and one refresh. Missing acceptance ratios are not invented: MIS uses Task 1 planar baseline only as proxy and acceptance hydraulic FSC/MIS cannot be computed with absent ratios.

| Model class × track | Inputs (L, r_search/r_val, W_job) | Projected generation time (s) | Max G in 300 s | Gate |
|---|---|---:|---:|---|
| Planar FSC | 53.5, 3.3975, 5.47 | 707.5 | 0 | FAIL |
| Planar MIS (proxy) | 62.6, 3.2435, 6.34 | 788.0 | 0 | FAIL |
| Planar Catalunya | 73.4008, 2.4228, 4.79 | 442.8 | 0 | FAIL |
| 14DoF FSC | 53.5, 7.5234, 5.47 | 1,566 | 0 | FAIL |
| 14DoF MIS | 62.6, unavailable | — | — | FAIL / unmeasured |
| 14DoF Catalunya | 73.4008, 4.9191, 4.78 | 893.9 | 0 | FAIL |
| 14DoF+hydraulic FSC | 53.5, unavailable | — | — | FAIL / unmeasured |
| 14DoF+hydraulic MIS | 62.6, unavailable | — | — | FAIL / unmeasured |
| 14DoF+hydraulic Catalunya | 73.4008, 5.5538, 4.69 | 992.7 | 0 | FAIL |

## Decision

**GATE FAIL.** Best observed valid full-plant ratio was 2.4228 wall/sim-s (planar Catalunya), but best planar FSC search ratio was 3.3975, 22.7× above the 0.15 starting target; full validation ratios also exceed 0.40. Best projected cell among fully measured cases still cannot complete one generation: planar Catalunya generation alone is 442.8 s, with 355.0 s seed+refresh and startup (projected total for 8 generations: 3,898 s). Stop here per brief; Task 4 must not proceed without re-plan. Recommended next: correctly implement and validate E5 reduced search plant or revise allowable budget/window; current full-plant integrator choice is radau-loose for measured cases, strict radau for validation when fidelity is preferred.
