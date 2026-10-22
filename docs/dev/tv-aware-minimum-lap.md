# TV-Aware Minimum-Time Lap: Acceptance and Decision Record

This page is a decision record for the controller-aware (torque-vectoring, TV) minimum-time line work. It is not a new acceptance standard. It records what the code and tests establish, and it separates that from what has not been run.

**Bottom line.** The architecture for `ideal`, `allocation`, and `closed_loop` modes exists and its seams are tested. No production TV control policy is registered and no TV replay adapter is registered. Therefore no real-track TV or allocation run has been made, and no TV lap-time result is claimed. The only registered replay adapter is a point-mass parity replay, not a transient-vehicle controller.

See [minimum-time-line.md](minimum-time-line.md) for the stage 1 point-mass pipeline and its measured legacy results.

## 1. Status by mode

| `--control-mode` | Capability source | Default solver | Real-track TV run | What is established |
|---|---|---|---|---|
| `ideal` | provenance-checked capability cache through the ideal backend | `legacy` | No TV claim (not a TV mode) | Existing stage 1 point-mass pipeline; unchanged default path |
| `allocation` | `AllocationCapabilityBackend` samples a memoryless policy over `(v, κ)` | `legacy` (`--solver spline` opt-in) | **NOT RUN** | Backend contract, controller hash, stateful-policy rejection, infeasible-cell masking, `amax <= ideal` on the feasible mask |
| `closed_loop` | ideal, or allocation when a non-stateful policy is supplied | requires explicit `--solver spline` | **NOT RUN** as TV | Correction loop contract with test doubles; open course rejected; mesh convergence unavailable; point-mass adapter is a parity seam |

The runner exposes `--control-mode`, `--control-policy`, `--replay-adapter`, `--solver`, and `--compare-control-modes`. `CONTROL_POLICIES` is empty, so `allocation` and policy-aware `closed_loop` cannot be configured on any track in this repository state. `REPLAY_ADAPTERS` contains only `point-mass`.

## 2. What the tests establish

### 2.1 Ideal and allocation capability

`tests/test_capability_backend.py` covers the capability backend:

- `ideal` mode returns a surface with the vehicle provenance hash and an empty controller hash.
- `allocation` mode treats the policy as an oracle over `(speed, curvature)`, returns a non-empty controller hash, and records `curvature_limit`, `scan_points`, and `policy_name` in metadata.
- A stateful policy is rejected in allocation mode with a `ValueError`; only a memoryless policy may be sampled.
- An always-infeasible policy marks cells infeasible (`feasible_mask=False`) and zeroes their accelerations and ceiling. The zero placeholder is a mask, not a legal hold-speed capability. Downstream code must honour the mask; the runner fails on any infeasible sampled cell.
- Saturation tags are collected at both interval endpoints, including braking-side tags.
- A non-monotone feasible set is flagged and falls back to the widest feasible scan run.
- `test_allocation_never_exceeds_ideal` asserts that, on every feasible mask cell, allocation `acceleration_max` is at most the ideal value and `acceleration_min` is at least the ideal value, with at least one strictly narrower cell so the bound is not vacuous.

The allocation backend is a capability sampler. It does not model a controller as a plant and it cannot accept a stateful policy.

### 2.2 Spline transcription and corridor containment

The cubic-spline offset transcription is opt-in (`--solver spline`). The default remains `legacy` for `ideal` and `allocation`; `closed_loop` has no legacy path. The spline path replaces per-node offsets with control points along the track, periodic for a closed lap.

`tests/test_qss_spline_ocp.py` covers the transcription, including the new dense containment gate:

- `test_spline_solution_respects_dense_corridor` solves a closed 40 m circle with 3 m edges, then evaluates the spline at 25 samples per mesh element and checks every offset against corridor bounds sampled with `track_sample`. A 1e-6 m numerical allowance is used. This checks the represented line between nodes, not only at nodes.
- Existing tests cover periodic and clamped bases, zero-corridor parity with the node solver, heading and curvature-rate bounds, decision layout and scaling, and invalid speed-cap/spline rejection.

The lap-time objective integrates the per-track-distance time `den / (v cos ψ)`, where `den = 1 - κ₀ n` is the offset-frame metric factor, not `1 / (v cos ψ)`. The `den` factor enters because a lateral offset changes the vehicle-path length relative to track distance. The implementation and its comment are in [qss_spline_ocp.py](../../src/simplified_vehicle_models/solvers/qss_spline_ocp.py).

### 2.3 Closed-loop replay correction

`tests/test_closed_loop_line.py` covers the correction loop with test-double replays:

- Each round solves the spline OCP, rebuilds the path, calls the replay adapter, and accepts only when the sample is feasible and the replay time is within a one-sided 1% tolerance of the OCP time. A faster replay is accepted without tightening caps.
- An infeasible replay lowers the speed cap by a fixed 0.1 m/s margin and re-solves.
- A solver failure under a cap returns `accepted=False` with the attempts and caps recorded.
- `test_closed_loop_rejects_a_replay_that_never_becomes_feasible` uses a replay that is always infeasible and asserts `accepted=False` with `rounds == maximum_rounds` (3), three recorded caps, and a finite replay time. This proves the budget is exhausted rather than the loop stopping early.

Runner-level constraints (from `tests/test_tv_runner.py` and the Task 8 artifacts):

- `closed_loop` on an open course is rejected before any output is created, because the OCP time excludes a launch while the adapter replay includes its own standing start.
- `closed_loop` mesh convergence is recorded as `None` with a `check_unavailable` reason. The written line is normally the correction loop's finest re-solve, which is not the pre-correction ladder's fine stage. If the first correction solve fails, the pre-correction finest line is retained with `accepted=False`; the gate still stays unavailable because the normal path writes a different line. The comparison treats an explicit `None` as unavailable, not as a pass.
- `closed_loop` rejects `--path-smoothing > 0` because the unsmoothed path is what the adapter replayed.

## 3. Controller and replay provenance

No production TV controller name or hash exists in this repository state. The policies used in tests are memoryless algebraic test doubles: `cap-at-two`, `always-feasible`, `brake-only-saturation`, `never`, and `two-island`. Their hashes identify test fixtures only and must not be reported as production controller provenance.

No stateful controller is registered. The allocation backend would reject one; a stateful policy could only be used through `closed_loop` with a replay adapter that owns its own state.

The only registered replay adapter is `point-mass` (`PointMassReplay`). It rebuilds a track from the path and runs the existing point-mass lap solver. It is a parity seam: it proves the correction loop can call an adapter and compare times. It is not a TV controller, it does not replay a transient-vehicle model, and a `closed_loop` run driven by it is not a TV lap.

## 4. Real-track run matrix

**All real-track allocation and TV closed-loop cells are NOT RUN.** No real-track OCP or solver command was executed for this record.

| Track | `ideal` (legacy point-mass) | `allocation` | `closed_loop` (TV) |
|---|---|---|---|
| MIS | Zero-corridor centreline diagnostics only; strict free-line convergence is unresolved, so there is no accepted free-line result | **NOT RUN** | **NOT RUN** |
| Skidpad | Conditional stage 1 free-line result with the reduced offset basis (5/4.75 m); not repeated here and not a TV result | **NOT RUN** | **NOT RUN** |
| Acceleration | Conditional stage 1 free-line result (all five checks); not repeated here and not a TV result | **NOT RUN** | **NOT RUN** |
| FSC | Zero-corridor centreline only; strict free-line convergence is unresolved, so there is no accepted free-line result | **NOT RUN** | **NOT RUN** |

Consequences for the fields the brief requests:

- **Per-track difference between ideal and controller-aware lines:** NOT MEASURED. No allocation run exists on any track.
- **Whether controller-aware replay improved, matched, or worsened lap time:** NOT MEASURED. No TV replay exists.
- **Controller name/hash and algebraic/stateful:** no production controller; test doubles are algebraic and are not production provenance.
- **Assumed widths and surfaces:** any real-track run would inherit the stage 1 corridor conventions, which are `assumed` unless the track declares surveyed width, and the only available capability surface matches the IEM24 Murphy planar 7-DOF formulation, not an FSAE calibration. No width or surface was measured or changed for this record.
- **Unresolved convergence failures:** the strict free-line stage 1 ladders for FSC and MIS remain unresolved, so those tracks have only zero-corridor centreline results; skidpad needs the reduced offset basis; `closed_loop` mesh convergence is unavailable by construction. See [minimum-time-line.md](minimum-time-line.md) for the measured legacy diagnostics.

## 5. Synthetic smoke checks (not acceptance, not TV, not reproducible evidence)

Earlier Task 8 work ran manual real-solver smoke checks on a synthetic circle with a stubbed vehicle and a fake replay, using a synthetic capability surface. They exercised `solve_spline_line`, the offset spline, the rebuilt track, the allocation cache, and `correct_to_replay`. The numeric outputs were recorded only in a session artifact; no committed test reproduces them. They are therefore not repeated here as evidence.

These checks are not real-track results, not acceptance results, and not evidence of a TV lap-time gain. They only showed that the seams wire together.

## 6. Decision

- The `ideal` path remains the default and is unchanged as a point-mass stage 1 pipeline.
- The spline transcription, the allocation backend, and the closed-loop correction loop are implemented and tested at their seams. The spline solver stays opt-in.
- A controller-aware lap-time gain is conditional on the controller and the replay adapter. It is not a universal TV benefit, and this record claims none.
- `allocation` and `closed_loop` must not be reported as real-track results until a production TV policy and a TV replay adapter are registered and a real-track matrix is run with saved outputs.
- A point-mass parity replay must not be labelled a TV lap.
- Infeasible capability cells must be rejected or masked by every consumer. The zero placeholder is never a hold-speed capability.

## 7. Open work

1. Register a production memoryless TV policy with a stable provenance hash, or document that allocation remains unavailable.
2. Register a TV replay adapter that owns its state and can report replay time and saturation honestly. Extend it to support launch-inclusive comparison before enabling open-course `closed_loop`.
3. Run the MIS, skidpad, acceleration, and FSC matrix for `ideal` and `allocation` with the same corridor and mesh, and for `closed_loop` once a TV adapter exists. Save to new paths; the runner refuses overwrite.
4. Add a post-correction mesh ladder so `closed_loop` can report a real mesh-convergence gate instead of `None`.
5. Keep track XML/CSV, tolerances, and acceptance standards unchanged.

## 8. Related files

| Item | File |
|---|---|
| Allocation capability backend | [controller_capability.py](../../src/simplified_vehicle_models/formulations/controller_capability.py) |
| Spline transcription | [qss_spline_ocp.py](../../src/simplified_vehicle_models/solvers/qss_spline_ocp.py) |
| Closed-loop correction | [closed_loop_line.py](../../src/simplified_vehicle_models/solvers/closed_loop_line.py) |
| Replay adapters | [replay.py](../../src/simplified_vehicle_models/controls/replay.py) |
| Runner modes | [solve_minimum_time_line.py](../../tools/run/solve_minimum_time_line.py) |
| Stage 1 pipeline | [minimum-time-line.md](minimum-time-line.md) |
