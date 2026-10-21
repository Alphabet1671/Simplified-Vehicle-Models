# Minimum-Time Racing Line

This page describes the stage 1 quasi-steady minimum-time line pipeline. It uses Simplified Technical English. See [forward-driver.md](forward-driver.md) for the separate forward simulation.

## 1. Purpose

The solver changes the lateral path inside a track corridor. It solves a quasi-steady minimum-time problem. It then rebuilds the path and checks it with the existing point-mass lap solver. It supports a closed lap and an open course with a start and an end.

The corridor is `assumed` unless the track declares `surveyed` width provenance. A result on an assumed corridor is conditional. It is not a certified result. A track without width data can use a constant reference-point half-width. That width is also tagged `assumed`.

## 2. Files

| Item | File |
|---|---|
| Corridor bounds | [corridor.py](../../src/simplified_vehicle_models/tracks/corridor.py) |
| Quasi-steady line OCP | [qss_line_ocp.py](../../src/simplified_vehicle_models/solvers/qss_line_ocp.py) |
| Path reconstruction | [racing_line.py](../../src/simplified_vehicle_models/tracks/racing_line.py) |
| Five acceptance checks | [check_minimum_time_line.py](../../tools/gates/check_minimum_time_line.py) |
| Pipeline | [solve_minimum_time_line.py](../../tools/run/solve_minimum_time_line.py) |
| Acceptance-check tests | [test_minimum_time_line_gate.py](../../tests/test_minimum_time_line_gate.py) |

## 3. Terms

| Term | Meaning |
|---|---|
| Corridor | Allowed lateral offset from the track centreline. Positive offset is to the right. |
| Capability surface | Sampled acceleration and speed limits for the point-mass vehicle. |
| Mesh | Distance points where the solver calculates the line and speed. |
| Replay | A new lap solve on the rebuilt racing-line path. |
| Conditional result | A result that depends on width data that is not surveyed. |

## 4. How the solver works

1. Load the track and build its corridor using the requested margin. If the track has width data, use it. If it has no width data, require a constant `--half-width` and label the corridor `assumed`.
2. Compile the formulation and use its planar point-mass view. Load the capability surface. Stop if its provenance hash does not match the vehicle.
3. Solve the centreline envelope. Use a flying lap for a closed track and a standing start for an open course.
4. Build an exactly collapsed corridor with `minimum = maximum = 0`. Solve the zero-corridor control. For an open course, seed it from the envelope speed profile.
5. Call `solve_ladder` at 5 m, 2 m, and 1 m spacing by default. Warm-start each finer mesh from the previous solution. For an open course, seed the first stage from the envelope and use a small stage-dependent lateral regularisation. The stage callback records iteration counts and wall time around each `solve_line` call; mesh construction and seed projection are not included in those durations.
6. Rebuild the finest line as a path. Write the rebuilt XML and CSV into the run directory. Widths are carried by source node and projected onto the rebuilt path's local normals; a tangent change above 0.25 rad is rejected because the projection is not a continuous boundary reconstruction. Load the rebuilt path and calculate its replay lap time.
7. Sample corridor minimum and maximum values at the solution nodes. Exclude the repeated terminal node on a closed track. This tests containment on the solution mesh. It is not a proof of containment between mesh nodes.

For an open course, the OCP fixes heading at both ends and fixes the start speed. Lateral offset remains a free corridor decision at both ends. The default start speed is the solver minimum speed, because the distance-domain dynamics divide by speed. Set `QssSettings.start_speed` to change it. The OCP uses a small lateral regularisation on open courses to remove a null direction on straight sections.

The OCP uses a quasi-steady model. It does not model transient dynamics, sideslip, suspension, or 14DoF states.

## 5. Acceptance checks and summary

The runner writes `summary.json`, `line.csv`, `racing_line.csv`, `racing_line_track.xml`, and `racing_line_track.csv` in the output directory. The five checks are:

1. `zero_corridor_matches_envelope`: zero-corridor OCP and the centreline reference differ by no more than 1%. For a closed track the reference is the flying-lap envelope. For an open course it is the one-dimensional capability-surface integration, because the point-mass standing-start envelope is not the same model.
2. `free_line_not_slower`: the finest free-line OCP time is no greater than the centreline envelope time.
3. `corridor_containment`: every solution-node offset lies between the corridor bounds, with a 1e-6 m numerical allowance. Sampling is periodic only for a closed track.
4. `mesh_convergence`: the two finest lap times differ by no more than 0.5%.
5. `replay_agreement`: rebuilt-path replay and finest OCP times differ by no more than 1%.

`summary.json` contains measured lap times, gain, solver iterations, stage wall times, maximum constraint violation, track/corridor provenance, and a boolean for each check. A non-surveyed width source sets `conditional_result` to true. The command exits with status 1 if a completed run has any failed check; a setup or solver error exits with status 2.

## 6. How to run it

Build the capability surface when the requested cache is absent:

```bash
PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m tools.model_data.build_capability_surface \
  --formulation models/formulations/iem24_murphy_real_planar_7dof.xml \
  --track models/tracks/2024_michigan_endurance_closed_edged.xml \
  --output runs/capability/iem24_murphy_7dof_mis.npz
```

Run the acceptance pipeline:

```bash
PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m tools.run.solve_minimum_time_line \
  --track models/tracks/2024_michigan_endurance_closed_edged.xml \
  --formulation models/formulations/iem24_murphy_real_planar_7dof.xml \
  --surface runs/capability/iem24_murphy_7dof_mis.npz \
  --output runs/minimum_time_line/stage1 --margin 0.10
```

Run an open course that has no width data. The constant width is assumed, so the result is conditional:

```bash
PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m tools.run.solve_minimum_time_line \
  --track models/tracks/fsae_2026_acceleration.xml \
  --formulation models/formulations/iem24_murphy_real_planar_7dof.xml \
  --surface runs/capability/iem24_murphy_7dof_mis.npz \
  --output runs/minimum_time_line/acceleration --margin 0.10 --half-width 2.45
```

Use `--spacings 10,5` when a finer strict-tolerance stage does not converge. The mesh-convergence check then compares those two stages.

Run the focused gate tests:

```bash
PYTHONPATH=src python3 -m unittest tests.test_minimum_time_line_gate -v
```

## 7. Measured stage 1 results

The full integration run for the specified Michigan track has not completed successfully. These are solver diagnostics, not acceptance results. The acceptance runner uses the default `QssSettings` tolerance (1e-6); after midpoint-control projection was fixed, its default-tolerance Michigan zero-corridor 5/2/1 m ladder still failed with `Maximum_Iterations_Exceeded` after about 250 seconds and reported no completed stage. Thus the successful 1e-5 zero-corridor ladder is not sufficient for the runner configuration.

Before decision-variable normalization, the 5 m zero-corridor solve at tolerance 1e-5 succeeded in 100 iterations: 57.4224 s, maximum constraint violation 4.56e-14. At tolerance 1e-6, the 5 m solve exceeded 500 iterations. After fixed decision scaling, an independent 5 m `1e-5` solve was reevaluated against the exact same NLP rebuilt with requested tolerance `1e-6`: status `Solve_Succeeded`, objective 57.423162982 s, constraint violation 1.1883e-11, and variable-bound violation 0.0. The model equations and physical bounds were identical, and this point passes the existing `1e-6` physical primal-feasibility validator. This is strong numerical evidence of primal feasibility, not a KKT certificate or accepted strict-tolerance solve; IPOPT strict runs still fail to converge. A strict 5 m retry from this feasible point with fresh duals also failed after 500 iterations (objective 57.421293 s, violation 3.95e-3); changing only to a limited-memory Hessian from the same primal seed failed after 500 iterations with 4.13e-3 violation. Thus stale duals and exact Hessian alone do not explain the strict-stage failure. Capability limits use CasADi piecewise-linear interpolants; their gradients are valid within each interpolation cell but can jump at cell boundaries. On the accepted 5 m point, 20 of 612 node/midpoint capability queries were within 1e-4 1/m of a curvature knot (two within 1e-5); the nearest upper-acceleration query was 2.73e-7 1/m from a knot with only 0.00117 m/s² slack, and another was 2.21e-5 1/m away with 0.00031 m/s² slack. Forty-three of 612 queries were within 0.01 m/s of a speed knot. This makes an active kinked capability row plausible near the current point, but does not show that the strict IPOPT iterates stall at those knots. The interpolation nonsmoothness is a testable hypothesis, not a demonstrated root cause or derivative defect. As a diagnostic, replacing all capability interpolants with CasADi B-splines made the 5 m zero-corridor stages converge at 1e-4/1e-5/1e-6 in 26/0/2 iterations (57.4520 s, smooth-NLP residual 4.89e-7). However, evaluating that point against the production piecewise-linear NLP produced 248 violated rows and a maximum physical constraint violation of 0.473 m/s² (variable bounds remained satisfied). The smoother solution is therefore a relaxation, not an acceptable result or directly usable production seed; no interpolation change was made. A separate restoration NLP minimized scaled distance to this spline target under the production linear constraints, starting from the feasible linear seed; it exhausted 500 iterations at distance objective 1.67e-5 with residual 3.21e-5. A subsequent 500-iteration strict solve from that point failed with 9.19e-3 violation. A strict 5 m run using the monotone barrier from the standard feasible initial guess also failed at 500 iterations (violation 1.25e-3). These diagnostics do not justify a production solver-option change. An adaptive-barrier 1 m diagnostic ended `Infeasible_Problem_Detected` after 169 iterations (objective 59.393 s; max sampled violation 3.84e-6).

The current implementation applies a reversible, fixed characteristic scaling to the grouped QSS decision variables and uses IPOPT's adaptive barrier strategy. The scales are heuristic (offset 1 m, heading 0.15 rad, speed centered at 20 m/s with 10 m/s scale, acceleration 5 m/s², curvature rate 0.1 1/m); they change conditioning, not the physical objective, constraints, bounds, or returned values. With this change, a fresh 1 m zero-corridor solve at tolerance 1e-5 succeeded in 71 iterations (57.6092 s; maximum violation 9.01e-9). The 5 m and 2 m zero-corridor ladder stages also succeeded (185 and 49 iterations). Before midpoint-control preservation, projected 1 m failed; with midpoint controls retained, the full 5/2/1 m zero-corridor ladder at tolerance 1e-5 succeeded in 185/23/181 iterations, with the 1 m stage at 57.60950 s and maximum violation 1.84e-9. On the 1 m zero corridor, a tolerance homotopy from 1e-4 to 1e-5 also succeeded (71 then 390 iterations; second stage 57.609186 s, max violation 8.35e-9). Tightening to runner-default 1e-6 from that accepted solution still exceeded 1500 iterations after about 485 seconds. Thus tolerance homotopy improves the 1e-5 refinement but does not resolve strict-default acceptance. An opt-in same-mesh multiplier-warm-start continuation is available through `QssSettings(tolerance_homotopy=True)` for requested tolerances below 1e-5; it solves at 1e-4 then 1e-5 and the requested tolerance, carrying `lam_x`/`lam_g` only within the unchanged NLP. It is disabled by default. With homotopy enabled and a 500-iteration aggregate cap, the 1 m zero-corridor sequence converged at 1e-6 after the 1e-4/1e-5 stages (71/0/68 iterations, final time 57.609930 s and max violation 9.89e-9). On 5 m with homotopy enabled and the same 500 aggregate cap, 1e-4 and 1e-5 stages succeeded (185/0 iterations), but 1e-6 exceeded the remaining budget (max violation 5.76e-3). A full-cap (`maximum_iterations=3000`) default Michigan 5/2/1 m zero-corridor ladder with homotopy disabled failed its first 5 m stage after about 211 seconds; with homotopy enabled it still failed that stage after about 233 seconds. Therefore the full strict ladder remains unaccepted. Diagnostic variants on 5 m also failed: disabling IPOPT automatic NLP scaling reached the 1e-6 stage but exhausted a 1,500-iteration aggregate budget with 8.70e-3 maximum violation; a monotone-barrier continuation accepted 1e-4 in 49 iterations but consumed the remaining 2,951 iterations at 1e-5 (primal violation 1.26e-13) without advancing to 1e-6. These are not production settings. Iteration counts returned by `solve_line` sum same-mesh tolerance stages, and the aggregate cap is shared across them. At runner-default tolerance 1e-6, however, the isolated 5 m zero-corridor stage exceeded 3000 iterations after about 257 seconds. A fresh 1 m free-corridor solve exceeded 500 iterations, and a zero-corridor speed/control seed transferred to the free corridor exceeded 300 iterations. The free-line 5/2/1 m ladder at tolerance 1e-5 failed in its initial 5 m stage; seeding that stage from a centerline forward/backward speed envelope (61.1423 s) or from the converged 1 m zero-corridor solution also exceeded the iteration cap. The default constant-speed 5 m free-line initial guess is feasible at all sampled constraints and variable bounds, including midpoint capability limits (constant speed about 6.78 m/s, zero midpoint acceleration). The 5 m free-corridor solve also exceeds 1000 iterations at tolerance 1e-5. A diagnostic explicit physical-scale normalization of constraint rows (with IPOPT automatic NLP scaling disabled) still exceeded 250 iterations and left a maximum midpoint upper-acceleration violation of 1.68 m/s²; this is not a production change. The default 6.78 m/s seed is fully feasible; constant 8 and 10 m/s alternatives are also feasible but both failed within 150 iterations. A projected accepted 1 m zero-corridor solution (57.60918 s, 9.01e-9 violation) used to warm-start free 5 m still exceeded 300 iterations. Limited-memory Hessian reached a 0.688 m/s² midpoint upper-acceleration violation after 250 iterations but no convergence. The exact CasADi gradient of the midpoint upper-acceleration row agreed with centered finite differences at both the feasible start and a failed iterate (worst-row derivative discrepancies below 1e-9); midpoint curvature-rate sensitivity on the failed iterate was about 247. A diagnostic with nonnegative slack on midpoint upper-acceleration rows still exceeded 300 iterations and left a 2.27 hard-constraint violation, so this family alone does not explain convergence failure. Independent review found no demonstrated transcription defect; the cause remains unproven. Increasing curvature-rate regularization by 10,000x did not produce an accepted 5 m free-corridor solution within 600 iterations. Progressive opening to 10% corridor width failed within 500 iterations; fine opening from a converged zero-corridor 5 m solution failed at only 0.5% width within 300 iterations. Corridor bounds are symmetric signed bounds around zero and validated against the fold cap, so these failures are not explained by a simple width-scale or sign mistake. A 60-iteration IPOPT trace then reduced the objective from about 150 s to 42 s while primal constraint violation grew above 2 and dual infeasibility remained around 0.37. Same-mesh zero-corridor warm-start and corridor continuation were also tested: the 5 m zero-corridor stage succeeds (57.4232 s), but expanding corridor width to ±0.02 m fails within 250 iterations. This abrupt transition is observed, but its cause is not established. On the 100-iteration failed 5 m iterate, the largest violations are midpoint upper-acceleration capability rows (`a_max(v_mid, κ_vehicle) - a_mid ≥ 0`), with 47 rows above 1e-4 and maximum violation 2.009 m/s². The midpoint vehicle-curvature-domain, lower-acceleration, speed-ceiling and fold rows are satisfied; HS defect max is 0.00124, and interior corridor and periodic rows are clean. An exact node/midpoint query-range check found no capability-table extrapolation at this iterate: sampled speed ranged about 13.9–29.0 m/s and vehicle curvature about −0.089–0.048 1/m, inside the table domain (0.5–35.1 m/s, −0.243–0.243 1/m). This does not rule out out-of-domain transient line-search points. A direct numerical cross-check of the worst midpoint rows agrees exactly with `a_max(v_mid, κ_vehicle) - a_mid`; for example, one row has `a_mid=3.469` m/s² versus `a_max=1.460` m/s². The converged 5 m zero-corridor control has node accelerations within its sampled positive-acceleration envelope. A diagnostic-only NLP adding capability limits at Hermite quarter states (with quadratic control interpolation through endpoint and midpoint controls) still exceeded 180 iterations; its maximum original-constraint violation was 1.982 and added-row violation 1.330. This stricter collocation did not establish a fix and is not in production. A separate ladder robustness fix now retains midpoint controls in `QssSolution` and projects them periodically onto subsequent meshes rather than resetting them to zero; this preserves accepted collocation controls. After the change, the Michigan zero-corridor 5/2/1 m ladder at tolerance 1e-5 converged at all stages (185/23/181 iterations; 1 m time 57.60950 s, maximum violation 1.84e-9). This demonstrates a better zero-corridor ladder transfer, but the free-corridor ladder remains unverified/unsuccessful. An independent equation/sign review found no proven defect in the QSS equations, Hermite–Simpson defects, or right-positive corridor bounds. Therefore the free-line solve, complete mesh ladder, replay, and acceptance checks remain unverified.

Separate diagnostic row-scaling trials on the three Hermite–Simpson defect equalities did not produce an accepted solve. Multiplying by element length failed at 200 iterations (violation 1.21e-3); dividing by element length with automatic NLP scaling failed at 200 (1.48e-6); adding adaptive barrier and disabling automatic NLP scaling reached 57.62 s and 4.06e-7 violation at 200, but at 500 iterations ended `Infeasible_Problem_Detected` (58.10 s, violation 1.80e-6). These show that low primal residual alone is insufficient for acceptance. An earlier full runner attempt produced no further output after starting its zero-corridor control and was stopped after an extended silent run. No free-line lap time or acceptance-check outcome is claimed. The corridor provenance is `assumed`, so any completed result for this track is conditional. Record measured values from `summary.json` only after a successful full run.

The focused gate and OCP suites pass. The full GUI-inclusive repository suite stalled after extended GUI/font-cache output and was cancelled without a final summary. A separate run excluding eight Qt GUI modules completed 1,810 tests with 20 failures, 6 errors, and 25 skipped; reported unrelated failure groups include repository-layout checks for five tracked `.superpowers` reports, vertical between-step/evidence tests, recorded source-hash mismatches, and one animation-panel tolerance assertion. Neither run is a full-suite pass. Integration convergence and full-suite verification remain open.

On 2026-10-21, two bounded 5 m zero-corridor diagnostics were run on the exact production capability interpolants. A regularisation continuation changed only the curvature-rate penalty through `1e-4`, `1e-5`, and production `1e-6`; each stage had a 120-iteration cap and only primal variables were transferred. The first stage reached `Maximum_Iterations_Exceeded` at `1e-4` with constraint violation `3.01e-4`; `1e-5` then returned `Solve_Succeeded` in 55 iterations with `7.91e-12` violation; the final production-objective `1e-6` stage reached its cap with `1.34e-3` violation. This did not produce a strict accepted solve. A separate adaptive-barrier `mu_init` sweep (`0.1`, `0.01`, `0.001`) used the same 5 m production NLP, strict `1e-6` tolerance, fresh default seed, and 120 iterations per run. All three returned `Maximum_Iterations_Exceeded` at 120 iterations with the same reported objective (`57.421813236 s`) and constraint violation (`1.055e-3`). These experiments did not change production defaults or acceptance criteria; they provide no accepted Michigan result or five-gate outcome. A further bounded multistart diagnostic used two deterministic small offset/heading perturbations on the 5 m strict production NLP, with fresh duals and 100 iterations per attempt. Its preceding `1e-5` seed-generation stage itself reached the same 100-iteration cap (`4.93e-3` constraint violation); the two strict runs also reached their caps with violations `1.10e-3` and `7.60e-3`. No multistart result was accepted.

On 2026-10-21, a bounded derivative diagnostic compared CasADi's production linear interpolant for the Michigan `acceleration_max` capability grid against centered finite differences at 100 deterministic points strictly inside interpolation cells. The maximum scaled gradient discrepancy was `7.74e-10` (median `0`). A separate deterministic sample measured one-sided curvature-gradient differences at 500 interior curvature knots while holding speed inside grid cells: 8% had a scaled jump greater than `1e-6`, the median jump was `0`, and the maximum was `0.996`. This confirms that some sampled knots are genuinely nonsmooth while ruling out an obvious within-cell derivative inconsistency. It does not establish that iterates stall at those knots, identify the cause of IPOPT's convergence failures, or count as solver acceptance.

## 8. Multi-track pipeline compatibility checks

On 2026-10-21, the runner was extended to support open tracks and tracks without width data. The untouched FSC source still has no lateral corridor. The untouched skidpad and acceleration sources are open. With an explicit `--half-width`, all three tracks load and reach the OCP; the width is tagged `assumed`.

Measured outcomes with the only available capability surface, whose provenance hash matches the IEM24 Murphy planar formulation:

- Acceleration (`--half-width 2.45`): the zero-corridor control converged at 6.408411 s. The 5/2/1 m free-line ladder converged at 6.562297/6.408411/6.384903 s with maximum violation 9.88e-9. Four checks pass: zero corridor against the 6.378147 s capability integration, free line not slower, containment, and mesh convergence (0.37%). Replay agreement fails: the rebuilt straight path replayed with the point-mass solver at 7.717125 s, while the OCP reports 6.384903 s. This is a 21% model disagreement on open straight-line acceleration, not a path-reconstruction error. The result is conditional because the width is assumed and the capability surface is not an FSAE calibration.
- Skidpad (`--half-width 1.50`): the 2 m zero-corridor control converged at 22.221716 s. The strict free-line ladder failed in its first 5 m stage with `Maximum_Iterations_Exceeded` at every tested lateral regularisation from 1e-6 to 1e-2, including seeds from the envelope and from the zero-corridor solution.
- FSC (`--half-width 3.0`): the zero-corridor control converged at 48.866054 s. The strict free-line ladder still failed with `Maximum_Iterations_Exceeded`; no summary was produced.
- MIS: the closed edged fixture remains in the same state as the diagnostics above; its strict free-line convergence is still unresolved.

The acceleration result shows that the open formulation and runner work end to end, but the five-gate acceptance is not closed. The replay disagreement must be resolved or explained before the open-track acceptance claim is complete. Skidpad and FSC free-line convergence remain open. Event-valid verification also requires real corridor data and vehicle-matched capability surfaces; the FSAE widths used here are rule-course widths reduced by the requested margin, not measured reference-point clearances.

Source XML/CSV hashes for the FSC, skidpad, and acceleration tracks matched before and after these attempts.

## 9. Limits and roadmap

This is stage 1 of three. Stage 2 (transient OCP) and stage 3 (certification) are separate plans. Stage 1 is quasi-steady and samples corridor containment only at its solution nodes. It does not certify continuous containment between nodes or provide a driven-vehicle certification replay.
