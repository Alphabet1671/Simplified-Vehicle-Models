# SOC-Coupled Solver Performance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Apply the eight identified solver optimizations while preserving default numerical behavior, exposing approximations explicitly, and reducing assembled MIS SOC-coupled runtime.

**Architecture:** Keep the existing forward/backward minimum-time formulation, but remove redundant work at each nesting level. Add direction-specific acceleration/braking limit kernels, tolerance-based scalar roots, bracket-preserving corner-root hints, solver-local drive-limit caching with an explicit optional quantization policy, propagated inner-solver controls, and structured progress callbacks. Transition/profile `initial_speed` acceleration remains unsupported and is ignored after API validation; do not imply approximate profile reuse. Validate each layer against the current solver before changing benchmark defaults.

**Tech Stack:** Python 3.12, NumPy, `unittest`, `cProfile`, existing XML model compiler and MIS regression fixtures.

**Spec:** `docs/validation/powertrain.md` and the eight optimization items recorded in the preceding solver analysis.

> **Historical checklist status (final review):** Tasks 1–6 were implemented in the commits recorded by the SDD ledger. The checklist below is retained as implementation history; the transition/profile `initial_speed` acceleration item is explicitly ruled out/unsupported, while bracket-preserving corner hints are complete. Final-review fixes are recorded in the ledger and Task 6 report.

## Global Constraints

- Preserve the existing public defaults and legacy lumped drivetrain results unless a benchmark-backed default change is separately justified.
- Preserve D21 maximum motor speed of 18,600 rpm.
- Regeneration must remain charge-power-, motor-, and tire-limited; friction braking supplies any remainder.
- Any speed/SOC cache quantization is an approximation and must be opt-in, reported, bounded, and regression-tested against exact mode.
- A normal assembled MIS lap should remain below 30 seconds.
- Do not merge or push without user approval.

## File Structure

- Modify `src/simplified_vehicle_models/formulations/point_mass.py`: direction-specific limit kernels, tolerance-based roots, scalar/root fast path, solver-local drive-force cache policy.
- Modify `src/simplified_vehicle_models/solvers/forward_backward.py`: corner/profile warm starts, inner-control propagation, progress events, cache lifecycle.
- Modify `src/simplified_vehicle_models/results/traces.py`: immutable progress/config/result metadata only if required by the public interface.
- Modify `tests/test_point_mass.py`: directional/root/cache equivalence tests.
- Modify `tests/test_lap_solver.py`: warm-start and tolerance equivalence tests.
- Modify `tests/test_energy_lap.py`: coupled-control propagation, regen, and progress tests.
- Modify `tests/test_mis_regression.py`: exact and accelerated assembled MIS gates.
- Modify `docs/validation/powertrain.md`: measured before/after timings, numerical deltas, and approximation status.

---

### Task 1: Direction-Specific Limit Solves and Scalar Fast Path

**Files:**
- Modify: `src/simplified_vehicle_models/formulations/point_mass.py:246-461`
- Modify: `src/simplified_vehicle_models/solvers/forward_backward.py:92-164`
- Test: `tests/test_point_mass.py`
- Test: `tests/test_lap_solver.py`

**Interfaces:**
- Produces: `maximum_acceleration_limit(vehicle, speed, curvature, storage_state=None) -> float`
- Produces: `maximum_deceleration_limit(vehicle, speed, curvature, storage_state=None) -> float`
- Preserves: `acceleration_limits(...) -> AccelerationLimits` for callers needing both full diagnostic states.

- [ ] **Step 1: Add failing directional-equivalence tests**

```python
limits = acceleration_limits(vehicle, 15.0, 0.01, StorageState(0.8))
self.assertAlmostEqual(
    maximum_acceleration_limit(vehicle, 15.0, 0.01, StorageState(0.8)),
    limits.maximum_acceleration,
    places=8,
)
self.assertAlmostEqual(
    maximum_deceleration_limit(vehicle, 15.0, 0.01, StorageState(0.8)),
    limits.maximum_deceleration,
    places=8,
)
```

- [ ] **Step 2: Run the focused tests and verify the new imports/functions fail**

Run: `PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest tests.test_point_mass tests.test_lap_solver -v`

- [ ] **Step 3: Extract a private scalar feasibility kernel**

Implement `_equilibrium_limits(vehicle, speed, curvature, longitudinal_acceleration, storage_state)` returning only the booleans, drag, drive limit, and brake limit needed by root residuals. Use scalar four-wheel arithmetic or reusable local tuples; do not allocate/freeze diagnostic arrays. Keep `evaluate_equilibrium` as the authoritative full diagnostic constructor and compare the private kernel against it in tests across acceleration, braking, curvature, and SOC cases.

- [ ] **Step 4: Implement direction-specific public functions**

`maximum_acceleration_limit` evaluates coasting lateral feasibility, solves only the drive residual, and returns one scalar. `maximum_deceleration_limit` does the same for braking. Retain `acceleration_limits` by calling both functions and then constructing its three full diagnostic states.

- [ ] **Step 5: Change the forward sweep to call only `maximum_acceleration_limit` and the backward sweep to call only `maximum_deceleration_limit`**

Do not change transition equations or SOC node selection.

- [ ] **Step 6: Run focused and MIS regression tests**

Run: `PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest tests.test_point_mass tests.test_lap_solver tests.test_mis_regression -v`
Expected: all pass with existing exact-mode lap-time tolerances.

- [ ] **Step 7: Commit**

```bash
git add src/simplified_vehicle_models/formulations/point_mass.py src/simplified_vehicle_models/solvers/forward_backward.py tests/test_point_mass.py tests/test_lap_solver.py
git commit -m "perf: split directional point-mass limit solves"
```

---

### Task 2: Tolerance-Based Roots and Correct Warm Starts

**Files:**
- Modify: `src/simplified_vehicle_models/formulations/point_mass.py:381-402,588-600`
- Modify: `src/simplified_vehicle_models/solvers/forward_backward.py:21-83,92-164`
- Test: `tests/test_point_mass.py`
- Test: `tests/test_lap_solver.py`

**Interfaces:**
- Produces: `_positive_root(function, upper, tolerance=1e-8, hint=None) -> float`
- Produces: `maximum_corner_speed(..., tolerance=1e-8, hint=None) -> float`
- Extends: `solve_lap(..., initial_speed=None, corner_speed_hint=None, root_tolerance=1e-8)`.

- [ ] **Step 1: Add failing root and corner-hint equivalence tests**

Test hints on both sides of the root. Require hinted and unhinted results to differ by no more than `2 * tolerance`, and instrument the residual callable to prove a close hint reduces evaluations.

- [ ] **Step 2: Implement bracket-preserving hinted bisection**

Evaluate `hint` once. If feasible/positive, move only the lower bracket to the hint; otherwise move only the upper bracket. Stop when `upper - lower <= tolerance`, retaining the existing conservative lower-bound return value.

- [x] **Step 3: Use previous node corner speeds as hints**

`solve_lap` accepts `corner_speed_hint` and uses each finite hint as a bracket-preserving root candidate for the current SOC. It may reduce root work but cannot change the exact corner-root result; validate shape and finiteness.

- [x] **Step 4: Keep transition/profile initial-speed warm starts ruled out/unsupported**

Validate `initial_speed` for shape, finiteness, and nonnegative values for API compatibility, then ignore it and initialize the profile from conservative corner caps. Do not add approximate profile reuse or imply a default change. Only `corner_speed_hint` uses bracket-preserving exact root acceleration.

- [ ] **Step 5: Add cold-versus-warm lap tests**

Solve a synthetic lap cold, then solve with its corner limits as exact bracket-preserving hints. Supply `initial_speed` only to verify it is accepted and ignored. Require matching speed and lap time within the configured root tolerance and fewer counted limit evaluations from the corner hints.

- [ ] **Step 6: Run focused and MIS tests**

Run: `PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest tests.test_point_mass tests.test_lap_solver tests.test_mis_regression -v`

- [ ] **Step 7: Commit**

```bash
git add src/simplified_vehicle_models/formulations/point_mass.py src/simplified_vehicle_models/solvers/forward_backward.py tests/test_point_mass.py tests/test_lap_solver.py
git commit -m "perf: add bracketed solver warm starts"
```

---

### Task 3: Solver-Local Drive Cache With Explicit Approximation Policy

**Files:**
- Modify: `src/simplified_vehicle_models/formulations/point_mass.py:44,512-548`
- Modify: `src/simplified_vehicle_models/solvers/forward_backward.py:21-83,270-311`
- Test: `tests/test_point_mass.py`
- Test: `tests/test_mis_regression.py`

**Interfaces:**
- Produces: immutable `DriveCachePolicy(speed_resolution: float = 0.0, soc_resolution: float = 0.0)`.
- Produces: per-solve cache passed through point-mass evaluations; resolution `0.0` means exact keys.
- Extends: `solve_lap(..., drive_cache_policy=DriveCachePolicy())`.

- [ ] **Step 1: Add failing exact-cache lifecycle tests**

Prove two identical operating points hit the same entry, different solves do not retain unbounded global entries, and exact mode returns identical force and active constraint.

- [ ] **Step 2: Replace the module-global unbounded cache with a solver-local cache**

Create it in `solve_lap` and pass it through corner, sweep, root, and final-result evaluations. Keep exact float keys in default mode.

- [ ] **Step 3: Add opt-in bounded key quantization**

For positive resolutions, key with `round(speed / speed_resolution)` and `round(soc / soc_resolution)`. Store the actual evaluation coordinates with the cached value and expose maximum bucket displacement in diagnostics so approximation magnitude is measurable.

- [ ] **Step 4: Add exact-versus-quantized MIS validation**

Benchmark candidate policies beginning with `speed_resolution=1e-3 m/s` and `soc_resolution=1e-5`. Require lap-time delta below 0.01 s, final-SOC delta below `1e-4`, identical convergence status, and no motor-speed-limit violation before recommending any nonzero policy.

- [ ] **Step 5: Run focused tests and commit**

```bash
PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest tests.test_point_mass tests.test_mis_regression -v
git add src/simplified_vehicle_models/formulations/point_mass.py src/simplified_vehicle_models/solvers/forward_backward.py tests/test_point_mass.py tests/test_mis_regression.py
git commit -m "perf: scope and bound drive-force caching"
```

---

### Task 4: Propagate Inner Controls and Reuse Coupled-Lap Hints

**Files:**
- Modify: `src/simplified_vehicle_models/solvers/forward_backward.py:270-311,421-449`
- Test: `tests/test_energy_lap.py`

**Interfaces:**
- Extends: `solve_lap_with_energy(..., lap_tolerance=1e-6, lap_maximum_iterations=200, root_tolerance=1e-8, drive_cache_policy=DriveCachePolicy())`.
- Extends: `solve_stint` by forwarding those named controls without changing existing defaults.

- [ ] **Step 1: Add failing mock-based propagation tests**

Patch `solve_lap` and assert every SOC and reconciliation call receives `lap_tolerance`, `lap_maximum_iterations`, `root_tolerance`, and the cache policy.

- [ ] **Step 2: Forward explicit controls to every inner `solve_lap` call**

Keep energy `tolerance` and `maximum_iterations` semantically separate from lap profile controls.

- [ ] **Step 3: Carry corner hints between SOC passes and between stint laps**

Pass previous coupled corner-speed results through Task 2’s bracket-preserving interface. `initial_speed` remains unsupported and ignored for transition/profile acceleration; do not reuse a prior profile approximately. Reset corner hints if mode, track, vehicle, or array shape differs.

- [ ] **Step 4: Add regen-enabled cold-versus-warm equivalence test**

Use the synthetic energy track with `regen_enabled=True`. Require equal convergence flags and numerical equality within configured tolerances for speed, lap time, final SOC, consumed energy, and recovered energy.

- [ ] **Step 5: Run energy and MIS tests and commit**

```bash
PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest tests.test_energy_lap tests.test_lap_solver tests.test_mis_regression -v
git add src/simplified_vehicle_models/solvers/forward_backward.py tests/test_energy_lap.py
git commit -m "perf: reuse coupled lap solve hints"
```

---

### Task 5: Structured Progress Reporting

**Files:**
- Modify: `src/simplified_vehicle_models/results/traces.py`
- Modify: `src/simplified_vehicle_models/solvers/forward_backward.py`
- Test: `tests/test_energy_lap.py`
- Modify: `examples/run_mis_point_mass.py`

**Interfaces:**
- Produces: frozen `SolverProgress(stage: str, lap: int, lap_count: int, energy_iteration: int, profile_iteration: int, elapsed_seconds: float)`.
- Extends: `solve_lap_with_energy(..., progress_callback: Callable[[SolverProgress], None] | None = None)`.
- Extends: `solve_stint(..., progress_callback=None)`.

- [ ] **Step 1: Add failing callback-order tests**

Require `lap_start`, `energy_iteration`, `lap_complete`, and `stint_complete` events in monotone lap/iteration order. Verify no events are emitted when callback is `None`.

- [ ] **Step 2: Emit progress outside numerical kernels**

Use `time.perf_counter()` only for reporting. Callback return values must be ignored and must not alter solver behavior.

- [ ] **Step 3: Add example CLI progress output**

Add `--progress` to print one concise line per energy iteration and lap completion, including elapsed wall time.

- [ ] **Step 4: Run tests and commit**

```bash
PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest tests.test_energy_lap -v
git add src/simplified_vehicle_models/results/traces.py src/simplified_vehicle_models/solvers/forward_backward.py tests/test_energy_lap.py examples/run_mis_point_mass.py
git commit -m "feat: report coupled solver progress"
```

---

### Task 6: Benchmark, Tune, and Document

**Files:**
- Modify: `tests/test_mis_regression.py`
- Modify: `docs/validation/powertrain.md`

**Interfaces:**
- Consumes all prior exact and accelerated solver controls.
- Produces measured exact-mode and optional accelerated-mode timing and numerical-delta tables.

- [ ] **Step 1: Run a fresh exact assembled MIS flying lap under `/usr/bin/time` and `cProfile`**

Record wall time, equilibrium calls, corner-speed time, drive-limit time, shaft-power time, lap time, convergence, and iterations.

- [ ] **Step 2: Run a regen-enabled one-lap SOC-coupled benchmark**

Record wall time, energy iterations, profile residual, final SOC, consumed/recovered energy, and lap time. Compare cold and warm/hinted modes.

- [ ] **Step 3: Select accelerated defaults only from validated candidates**

A candidate can become default only if assembled MIS lap-time delta is below 0.01 s, final-SOC delta is below `1e-4`, recovered-energy delta is below 0.5%, and all convergence/constraint checks match. Otherwise retain it as explicit opt-in.

- [ ] **Step 4: Attempt the fully coupled 22-lap regen-enabled stint with progress enabled**

Use a managed background job. Record completed runtime and result if it finishes; otherwise record the observed lower bound and completed-lap/iteration progress rather than reporting an opaque timeout.

- [ ] **Step 5: Run the full suite**

Run: `PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest discover -s tests -v`
Expected: all tests pass.

- [ ] **Step 6: Update validation documentation**

Document exact versus accelerated settings, measured speedup, numerical deltas, cache approximation status, regen configuration, and any remaining runtime limitation.

- [ ] **Step 7: Commit**

```bash
git add tests/test_mis_regression.py docs/validation/powertrain.md
git commit -m "docs: validate coupled solver performance"
```

- [ ] **Step 8: Final review**

Request independent specification and quality reviews. Verify `git status --short --branch` is clean and do not merge or push.
