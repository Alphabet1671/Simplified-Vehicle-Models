# Minimum-Time Racing Line

This page describes the stage 1 quasi-steady minimum-time line pipeline. It uses Simplified Technical English. See [forward-driver.md](forward-driver.md) for the separate forward simulation.

## 1. Purpose

The solver changes the lateral path inside a track corridor. It solves a quasi-steady minimum-time problem. It then rebuilds the path and checks it with the existing point-mass lap solver.

The corridor is `assumed` unless the track declares `surveyed` width provenance. A result on an assumed corridor is conditional. It is not a certified result.

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

1. Load the edged track and build its corridor using the requested margin.
2. Compile the formulation and use its planar point-mass view. Load the capability surface. Stop if its provenance hash does not match the vehicle.
3. Solve a flying lap on the edged centreline. This is the centreline envelope.
4. Build an exactly collapsed corridor with `minimum = maximum = 0`. Solve the zero-corridor control. Do not use `half_width`; the track may have asymmetric bounds.
5. Call `solve_ladder` at 5 m, 2 m, and 1 m spacing. Warm-start each finer mesh from the previous solution. The stage callback records iteration counts and wall time around each `solve_line` call; mesh construction and seed projection are not included in those durations.
6. Rebuild the finest line as a path. Write the rebuilt XML and CSV into the run directory. Widths are carried by source node and projected onto the rebuilt path's local normals; a tangent change above 0.25 rad is rejected because the projection is not a continuous boundary reconstruction. Load the rebuilt path and calculate its replay lap time.
7. Sample corridor minimum and maximum values periodically at the solution nodes. Exclude the repeated terminal node for periodic sampling. This tests containment on the solution mesh. It is not a proof of containment between mesh nodes.

The OCP uses a quasi-steady model. It does not model transient dynamics, sideslip, suspension, or 14DoF states.

## 5. Acceptance checks and summary

The runner writes `summary.json`, `line.csv`, `racing_line.csv`, `racing_line_track.xml`, and `racing_line_track.csv` in the output directory. The five checks are:

1. `zero_corridor_matches_envelope`: zero-corridor OCP and centreline envelope differ by no more than 1%.
2. `free_line_not_slower`: the 1 m free-line OCP time is no greater than the centreline time.
3. `corridor_containment`: every solution-node offset lies between the periodic corridor bounds, with a 1e-6 m numerical allowance.
4. `mesh_convergence`: the 2 m and 1 m lap times differ by no more than 0.5%.
5. `replay_agreement`: rebuilt-path replay and 1 m OCP times differ by no more than 1%.

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

Run the focused gate tests:

```bash
PYTHONPATH=src python3 -m unittest tests.test_minimum_time_line_gate -v
```

## 7. Measured stage 1 results

The full integration run for the specified Michigan track has not completed successfully. These are solver diagnostics, not acceptance results. The acceptance runner uses the default `QssSettings` tolerance (1e-6); after midpoint-control projection was fixed, its default-tolerance Michigan zero-corridor 5/2/1 m ladder still failed with `Maximum_Iterations_Exceeded` after about 250 seconds and reported no completed stage. Thus the successful 1e-5 zero-corridor ladder is not sufficient for the runner configuration.

Before decision-variable normalization, the 5 m zero-corridor solve at tolerance 1e-5 succeeded in 100 iterations: 57.4224 s, maximum constraint violation 4.56e-14. At tolerance 1e-6, the 5 m solve exceeded 500 iterations. Limited-memory Hessian approximation did not resolve prior 1 m failures. An adaptive-barrier 1 m diagnostic ended `Infeasible_Problem_Detected` after 169 iterations (objective 59.393 s; max sampled violation 3.84e-6).

The current implementation applies a reversible, fixed characteristic scaling to the grouped QSS decision variables and uses IPOPT's adaptive barrier strategy. The scales are heuristic (offset 1 m, heading 0.15 rad, speed centered at 20 m/s with 10 m/s scale, acceleration 5 m/s², curvature rate 0.1 1/m); they change conditioning, not the physical objective, constraints, bounds, or returned values. With this change, a fresh 1 m zero-corridor solve at tolerance 1e-5 succeeded in 71 iterations (57.6092 s; maximum violation 9.01e-9). The 5 m and 2 m zero-corridor ladder stages also succeeded (185 and 49 iterations). Before midpoint-control preservation, projected 1 m failed; with midpoint controls retained, the full 5/2/1 m zero-corridor ladder at tolerance 1e-5 succeeded in 185/23/181 iterations, with the 1 m stage at 57.60950 s and maximum violation 1.84e-9. At runner-default tolerance 1e-6, however, the isolated 5 m zero-corridor stage exceeded 3000 iterations after about 257 seconds. A fresh 1 m free-corridor solve exceeded 500 iterations, and a zero-corridor speed/control seed transferred to the free corridor exceeded 300 iterations. The free-line 5/2/1 m ladder at tolerance 1e-5 failed in its initial 5 m stage; seeding that stage from a centerline forward/backward speed envelope (61.1423 s) or from the converged 1 m zero-corridor solution also exceeded the iteration cap. The default constant-speed 5 m free-line initial guess is feasible at all sampled constraints and variable bounds, including midpoint capability limits (constant speed about 6.78 m/s, zero midpoint acceleration). The 5 m free-corridor solve also exceeds 1000 iterations at tolerance 1e-5. A diagnostic explicit physical-scale normalization of constraint rows (with IPOPT automatic NLP scaling disabled) still exceeded 250 iterations and left a maximum midpoint upper-acceleration violation of 1.68 m/s²; this is not a production change. A 60-iteration IPOPT trace then reduced the objective from about 150 s to 42 s while primal constraint violation grew above 2 and dual infeasibility remained around 0.37. Same-mesh zero-corridor warm-start and corridor continuation were also tested: the 5 m zero-corridor stage succeeds (57.4232 s), but expanding corridor width to ±0.02 m fails within 250 iterations. This abrupt transition is observed, but its cause is not established. On the 100-iteration failed 5 m iterate, the largest violations are midpoint upper-acceleration capability rows (`a_max(v_mid, κ_vehicle) - a_mid ≥ 0`), with 47 rows above 1e-4 and maximum violation 2.009 m/s². The midpoint vehicle-curvature-domain, lower-acceleration, speed-ceiling and fold rows are satisfied; HS defect max is 0.00124, and interior corridor and periodic rows are clean. An exact node/midpoint query-range check found no capability-table extrapolation at this iterate: sampled speed ranged about 13.9–29.0 m/s and vehicle curvature about −0.089–0.048 1/m, inside the table domain (0.5–35.1 m/s, −0.243–0.243 1/m). This does not rule out out-of-domain transient line-search points. A direct numerical cross-check of the worst midpoint rows agrees exactly with `a_max(v_mid, κ_vehicle) - a_mid`; for example, one row has `a_mid=3.469` m/s² versus `a_max=1.460` m/s². The converged 5 m zero-corridor control has node accelerations within its sampled positive-acceleration envelope. A diagnostic-only NLP adding capability limits at Hermite quarter states (with quadratic control interpolation through endpoint and midpoint controls) still exceeded 180 iterations; its maximum original-constraint violation was 1.982 and added-row violation 1.330. This stricter collocation did not establish a fix and is not in production. A separate ladder robustness fix now retains midpoint controls in `QssSolution` and projects them periodically onto subsequent meshes rather than resetting them to zero; this preserves accepted collocation controls. After the change, the Michigan zero-corridor 5/2/1 m ladder at tolerance 1e-5 converged at all stages (185/23/181 iterations; 1 m time 57.60950 s, maximum violation 1.84e-9). This demonstrates a better zero-corridor ladder transfer, but the free-corridor ladder remains unverified/unsuccessful. An independent equation/sign review found no proven defect in the QSS equations, Hermite–Simpson defects, or right-positive corridor bounds. Therefore the free-line solve, complete mesh ladder, replay, and acceptance checks remain unverified.

Separate diagnostic row-scaling trials on the three Hermite–Simpson defect equalities did not produce an accepted solve. Multiplying by element length failed at 200 iterations (violation 1.21e-3); dividing by element length with automatic NLP scaling failed at 200 (1.48e-6); adding adaptive barrier and disabling automatic NLP scaling reached 57.62 s and 4.06e-7 violation at 200, but at 500 iterations ended `Infeasible_Problem_Detected` (58.10 s, violation 1.80e-6). These show that low primal residual alone is insufficient for acceptance. An earlier full runner attempt produced no further output after starting its zero-corridor control and was stopped after an extended silent run. No free-line lap time or acceptance-check outcome is claimed. The corridor provenance is `assumed`, so any completed result for this track is conditional. Record measured values from `summary.json` only after a successful full run.

The focused gate and OCP suites pass. The full GUI-inclusive repository suite stalled after extended GUI/font-cache output and was cancelled without a final summary. A separate run excluding eight Qt GUI modules completed 1,810 tests with 20 failures, 6 errors, and 25 skipped; reported unrelated failure groups include repository-layout checks for five tracked `.superpowers` reports, vertical between-step/evidence tests, recorded source-hash mismatches, and one animation-panel tolerance assertion. Neither run is a full-suite pass. Integration convergence and full-suite verification remain open.

## 8. Limits and roadmap

This is stage 1 of three. Stage 2 (transient OCP) and stage 3 (certification) are separate plans. Stage 1 is quasi-steady and samples corridor containment only at its solution nodes. It does not certify continuous containment between nodes or provide a driven-vehicle certification replay.
