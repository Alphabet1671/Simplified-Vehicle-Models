# Energy-Coupling Redesign — Scope

Status: **scope for approval.** No code changed. Baseline stint still running.

## 1. Measured problem

| Run | Configuration | Wall time |
|---|---|---|
| 1 flying lap, LUT default | `--mode compare --runs 3` | 5.099 s (median) |
| 22-lap stint, lap 1 | 13 energy + 2 profile iterations | 160.8 s |
| 22-lap stint, laps 8 and 9 | same iteration counts | 381.2 s, 381.3 s |

Per-lap wall time for the completed laps (seconds):

```text
160.8  233.3  187.1  193.3  199.0  169.5  248.3  381.3  381.2
```

Laps 8–9 cost **2.3× lap 6** with identical iteration counts. Something per-pass is
getting more expensive as the stint proceeds, not just iteration count.

## 2. Three independent cost drivers

### D1 — Constant initial SOC profile plus fixed 0.5 damping

`trace` starts as a constant SOC vector and is updated with a hard-coded 50 % relaxation:

```python
trace = 0.5 * (trace + nxt)          # forward_backward.py:434
```

Halving an initial SOC mismatch of ~0.04 until it is under `1e-5` needs 12 halvings,
so 13 iterations is arithmetically consistent with the observed count. **The mechanism
is inferred from the code, not yet logged** — the per-iteration residual sequence has
not been recorded. Confirming it is a cheap diagnostic (§4 W0).

### D2 — SOC coupling solved outside the speed fixed point

`profile_iterations=2` shows the inner speed solve converges in two passes, yet each
lap runs **15 full `solve_lap` calls**: 13 coupling iterations plus 2 reconciliation
solves. The SOC loop is wrapped around the speed loop rather than solved with it.

### D3 — Per-pass work is on the wrong motor path, and loses coverage mid-stint

- `_energy_trace` calls `evaluate_shaft_power`, which is aliased to the **exact analytical
  oracle** (`power_plant.py:1017`), not the compiled LUT. That is roughly 1,406 oracle
  calls per pass and about 21,000 per lap, bypassing the artifact entirely
  ([forward_backward.py:503](/Users/alphabet1671/Dev/Simplified-Vehicle-Models/.worktrees/point-mass/src/simplified_vehicle_models/solvers/forward_backward.py#L503)).
- The LUT voltage axis floor is **530 V**, which is 0.503 SOC *unloaded*; loaded terminal
  voltage is lower, so the speed solve leaves the map well above 50 % SOC. With regen
  disabled SOC only falls, so late-stint laps should pay exact fallbacks. The 381 s laps
  are consistent with this, but the correlation is **not yet proven** — the running stint
  records per-lap final SOC and minimum bus voltage, which will settle it.

**Unmeasured:** the split between (a) repeated `solve_lap`, (b) `_energy_trace` oracle
calls, and (c) out-of-domain fallback. The redesign's payoff depends on this split, so
W0 gates the design choice.

## 3. Design options

| | Option | Effect | Size | Risk |
|---|---|---|---|---|
| O1 | Keep the outer loop; adaptive update + predicted initial profile | Removes most of the 13 iterations | S–M | Low |
| O2 | Fold SOC into the existing sweep fixed point | Removes the reconciliation loop and duplicate solves | M–L | Medium |
| O3 | Route the energy path through the LUT; extend the voltage axis | Cuts per-pass cost and late-stint fallbacks | M | Medium |

**Recommended sequence: O1 → measure → O2 → O3.** O1 is independently valuable and a
prerequisite for judging O2. O2 changes convergence structure, so it must be validated
against the current converged answer, not just iteration counts.

## 4. Workstreams

### W0 — Diagnostics (must precede the design commitment)
Log the per-iteration residual sequence, the SOC-error profile, per-pass oracle call
counts, and a wall-time split across `solve_lap` vs `_energy_trace` vs fallback.
**Deliverable:** a profile table. **Gate A:** choose O1-only or O1+O2.

### W1 — Adaptive coupling update
`forward_backward.py::solve_lap_with_energy`. Replace the fixed 0.5 relaxation with an
undamped update plus divergence-triggered damping. Preserve `tolerance`,
`maximum_iterations`, `energy_converged`, and `state_profile_residual` semantics.
**Tests:** the existing `energy_iterations == 1` and iteration-sequence progress tests
still bind; add monotone-residual and divergence-fallback tests.

### W2 — Predicted initial SOC profile
Seed `trace` from a cheap constant-SOC forward energy integration rather than a constant.
Must stay clearly distinct from `reuse_profile`, which reuses *speed*.

### W3 — Joint `(speed, SOC)` fixed point
Integrate SOC inside `_forward_sweep` (and backward only where regen is enabled) so the
speed iteration absorbs the coupling; drop the reconciliation loop. `_storage_trace`
becomes an in/out iterate. Keep `solve_lap_with_energy` as the public entry and leave
`solve_lap` SOC-free for compatibility.

### W4 — Energy-path LUT and endurance voltage coverage
Route `_energy_trace` through `evaluate_motor` with exact fallback, and extend the
canonical LUT axis downward to cover loaded endurance voltage. Pick the floor from the
stint's measured minimum bus voltage with margin, not by guessing. Regenerate, re-validate
monotone drive rows, the regen productive mask and the digest, and re-pin goldens.

### W5 — Stint-level coupling
`solve_stint` currently re-solves each lap from a constant SOC guess. Seed each lap from
the previous lap's converged SOC shape.

### W6 — Validation and documentation
Gates, goldens, `docs/validation/powertrain.md`, `models/data/provenance.md`.

## 5. Acceptance criteria

- Numerical equivalence to today's converged solver on the one-lap energy cases:
  lap-time delta ≤ 1 ms, final-SOC delta ≤ 1e-5, consumed-energy delta ≤ 0.1 %,
  per-segment SOC profile delta ≤ 1e-5.
- `energy_converged` keeps its meaning: the profile input and reported state agree to
  `tolerance`. No claim of convergence without that residual.
- **≥ 5× on the 22-lap stint** versus the baseline recorded in §1, with both numbers and
  the configuration reported.
- No regression in the flying-lap LUT gates; legacy lumped MIS pins unchanged.
- Full suite passes.

## 6. Risks and unknowns

- D1's mechanism is inferred; W0 must confirm it before W1 is designed.
- The dominant cost is unmeasured.
- Changing damping can converge to a **different fixed point**. `_fixed_point_transition`
  already warns that assembled capability is non-monotone and residual bisection can select
  another crossing. Every change must be checked against the current converged answer.
- Extending the LUT axis changes the artifact, the XML digest and pinned goldens.
- The regen backward sweep needs a SOC integration direction that does not exist yet;
  regen is off by default, so O2 can land for the no-regen path first.
- Pre-existing `recovered_energy` unit defect is out of scope unless it blocks validation.

## 7. Non-goals

- No new tire, aero, track or battery physics; no motor-model change beyond routing the
  energy path to the existing validated LUT.
- Not rewriting the forward/backward minimum-time formulation.
- No merge and no push.

## 8. Sequencing

```text
W0 ──Gate A──▶ W1 ──Gate B──▶ (stop: O1 sufficient)  or  W3
                 │
                 └──▶ W2, W4 (parallel-safe), W5
                                       │
                              Gate C ──▶ W6
```

Sizes: W0 S · W1 S–M · W2 S · W3 M–L · W4 M · W5 S · W6 S.
