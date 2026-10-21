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

The full integration run for the specified Michigan track has not completed successfully. Under the verified CasADi 3.8.1/IPOPT runtime, an earlier 1 m zero-corridor solve exited with `Maximum_Iterations_Exceeded`; a bounded 1 m solve at tolerance 1e-5 also exceeded 500 iterations. At 5 m, tolerance 1e-6 exceeded 500 iterations, while a standalone solve at 1e-5 succeeded in 100 iterations: 57.4224 s, maximum constraint violation 4.56e-14. Limited-memory Hessian approximation did not resolve the bounded 1 m case. A 1 m adaptive-barrier diagnostic stopped with `Infeasible_Problem_Detected` after 169 iterations (objective 59.393 s; max sampled violation 3.84e-6); despite the small residual, this is not a successful solution. These are solver diagnostics, not acceptance results. A subsequent full runner attempt produced no further output after starting its zero-corridor control and was stopped after an extended silent run. The full 1 m free-line solve, mesh ladder, replay, and acceptance checks remain unverified. No free-line lap time or acceptance-check outcome is claimed. The corridor provenance is `assumed`, so any completed result for this track is conditional. Record measured values from `summary.json` only after a successful full run.

The focused gate and OCP suites pass, but the full repository suite was stopped after an extended run without a final result. Do not interpret that as a pass. Integration convergence and full-suite verification remain open.

## 8. Limits and roadmap

This is stage 1 of three. Stage 2 (transient OCP) and stage 3 (certification) are separate plans. Stage 1 is quasi-steady and samples corridor containment only at its solution nodes. It does not certify continuous containment between nodes or provide a driven-vehicle certification replay.
