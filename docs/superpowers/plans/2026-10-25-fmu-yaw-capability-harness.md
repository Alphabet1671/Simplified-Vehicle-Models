# FMU Yaw-Capability Harness Implementation Plan

> **STATUS: SHELVED (2026-10-25).** Do not execute until every item in *Unshelve preconditions* is met and the user explicitly re-authorizes it. Until then it is a design record. It depends on the active [virtual-driver proposal v2](../../dev/virtual-driver-proposal-v2-2026-10-25.md). Run Phase 1 of that proposal first; it defines the `YawCapabilityMap` type this harness produces.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn an imported torque-vectoring (TV) controller FMU into a yaw-capability map. The map is measured on this repository's forward plants, with the same driver that later runs the lap, and the yaw-aware line planner consumes it like any other capability source.

**Architecture:** Measure the *closed-loop capability* of plant + FMU + `PreviewDriver`. Do not tabulate the FMU's input/output law: the FMU may be stateful and carry its own reference model or MPC. A per-wheel torque-allocation hook in the shared powertrain closure lets an external controller redistribute motor torque. The motor, battery and power limits stay authoritative. A co-simulation layer holds the FMU outputs constant over each communication step (zero-order hold). A deterministic manoeuvre library probes:
- the steady-state envelope (map A),
- curvature build-up and wind-off limits (map B),
- response descriptors (map C).

The same probes run against an equal-split baseline, so the difference between the two maps can be attributed to the FMU.

**Tech Stack:** Python 3.13, NumPy, SciPy (the existing `forward` extra), `fmpy` (new optional extra `fmu`), unittest.

**Spec:** [virtual-driver proposal v2](../../dev/virtual-driver-proposal-v2-2026-10-25.md), §4 (capability sources) and §8 (FMU source, shelved). Design discussion: the 2026-10-25 session leading to this plan.

## Global Constraints

- SI units. Wheel order `(FL, FR, RL, RR)`. Repository sign convention: positive lateral offset and positive curvature are to the **right** of travel. FMU signals are converted explicitly through the signal map, never implicitly.
- Default behaviour of every existing plant and powertrain path stays bitwise identical when no allocator is supplied.
- The motor envelope, inverter speed regulation, battery current and power closure in `forward_powertrain.py` are applied **after** the allocator, so an FMU cannot exceed electrical limits.
- An FMU instance is never shared between map cells. Each cell instantiates a fresh FMU, starts from a settled trim and discards a settling window.
- Every map is paired with an equal-split baseline built on the same plant, driver, grid and manoeuvres.
- A map value that exceeds the physical LP bound from proposal v2 Phase 1 is a harness or plant fault, not a capability.
- `fmpy` is optional. Tests use an in-repository fake controller and must not require a real FMU binary.
- Unshelving does not authorize changes to source tracks, capability caches or existing acceptance gates.

## Unshelve preconditions

- [ ] Proposal v2 Phases 1–3 are complete: the `YawCapabilityMap` type, physics-based maps, the yaw-aware speed profile and the line optimizer exist and have been reviewed.
- [ ] An FMU (or a representative one) is available, with: FMI version (2.0 or 3.0, Co-Simulation); platform binaries for the execution host (a macOS arm64 binary, or source-code FMU, or a Linux container decision); a signal list with units and axes; communication rate; and, for MPC-type controllers, whether a reference path or preview is needed.
- [ ] The user re-authorizes execution.

## File structure

| File | Responsibility |
|---|---|
| `src/simplified_vehicle_models/solvers/torque_allocation.py` (new) | `AllocationRequest`, `TorqueAllocator` protocol, `EqualSplitAllocator` |
| `src/simplified_vehicle_models/solvers/forward_powertrain.py` (modify) | Optional `allocator` argument applied before the envelope and power closure |
| `src/simplified_vehicle_models/solvers/planar_forward.py`, `vertical_forward.py` (modify) | Pass an optional allocator through to the powertrain |
| `src/simplified_vehicle_models/cosim/signal_map.py` (new) | Versioned, JSON-loaded mapping between plant signals and FMU variables |
| `src/simplified_vehicle_models/cosim/controller.py` (new) | `CoSimController` protocol, `FakeYawPIController` (test double), `FmuController` (`fmpy`) |
| `src/simplified_vehicle_models/cosim/hold.py` (new) | `HeldControllerAllocator`: zero-order hold driven by accepted steps |
| `src/simplified_vehicle_models/cosim/manoeuvres.py` (new) | Deterministic manoeuvre definitions and the runner |
| `src/simplified_vehicle_models/cosim/yaw_map_harness.py` (new) | Map A/B/C extraction, baseline pairing, validation against the physical bound |
| `tools/run/build_fmu_yaw_map.py` (new) | CLI: FMU + signal map + vehicle + grid → map cache and report JSON |
| `tests/test_torque_allocation.py`, `tests/test_cosim_signal_map.py`, `tests/test_cosim_hold.py`, `tests/test_cosim_manoeuvres.py`, `tests/test_fmu_yaw_map_harness.py` (new) | Unit and integration tests using the fake controller |

---

### Task 1: Per-wheel torque allocation hook

**Files:**
- Create: `src/simplified_vehicle_models/solvers/torque_allocation.py`
- Modify: `src/simplified_vehicle_models/solvers/forward_powertrain.py:37-106`, `planar_forward.py` (`powertrain`), `vertical_forward.py` (its powertrain call)
- Test: `tests/test_torque_allocation.py`

**Interfaces:**
- Produces:
  - `AllocationRequest(time: float, state: np.ndarray, command: DriverCommand, requested_motor_torque: np.ndarray)`. The last field is the per-driven-motor equal-split torque in N·m, after the throttle mapping and before the envelope.
  - `TorqueAllocator` protocol: `motor_torque(request) -> np.ndarray` (one value per driven motor, N·m).
  - `EqualSplitAllocator` returns `requested_motor_torque` unchanged.
  - `evaluate_forward_powertrain(..., command, *, allocator=None, time=0., state=None)`.

- [ ] **Step 1: Write the failing tests**

```python
import unittest
import numpy as np
from simplified_vehicle_models.solvers.torque_allocation import (
    AllocationRequest, EqualSplitAllocator)

class TorqueAllocationTest(unittest.TestCase):
    def test_equal_split_is_identity(self):
        request = AllocationRequest(0., np.zeros(12), None, np.array([40., 40.]))
        np.testing.assert_array_equal(EqualSplitAllocator().motor_torque(request), [40., 40.])

    def test_default_powertrain_is_bitwise_unchanged(self):
        # Fixture: the existing planar forward plant from tests/test_planar_forward.py.
        plant, state, command = self.planar_fixture()
        before = plant.powertrain(state, command)
        after = plant.powertrain(state, command, allocator=EqualSplitAllocator())
        np.testing.assert_array_equal(before.wheel_torques, after.wheel_torques)
        self.assertEqual(before.electrical_power, after.electrical_power)

    def test_allocator_cannot_exceed_motor_envelope(self):
        class Greedy:
            def motor_torque(self, request):
                return np.array([1.e6, -1.e6])
        plant, state, command = self.planar_fixture()
        result = plant.powertrain(state, command, allocator=Greedy())
        peak = plant.vehicle.motor.peak_motor_torque * plant.gearing
        self.assertTrue(np.all(np.abs(result.motor_wheel_torques) <= peak + 1e-9))
```

- [ ] **Step 2: Run the tests and confirm they fail**

Run `PYTHONPATH=src python3 -m unittest tests.test_torque_allocation -v`. Expected: `ModuleNotFoundError`.

- [ ] **Step 3: Implement**

In `operating_point(voltage)`:
1. Compute the equal-split `requested` vector per driven motor.
2. If an allocator is given, replace it with `allocator.motor_torque(...)`.
3. Validate the result: finite, and shaped `(len(driven),)`.
4. Clip each motor to `[-regen_available, available]` (the per-motor `speed_regulated_torque_limit`).
5. Continue with the existing power scaling and battery closure, unchanged.

Negative allocator torque is regeneration and is bounded by `regeneration_torque_fraction`. Thread `allocator` through `planar_forward.PlanarForwardPlant.powertrain` and the vertical plant as a keyword-only argument (default `None`).

- [ ] **Step 4: Run the new tests plus `tests.test_planar_forward`, `tests.test_powertrain_closure_audit` and the vertical forward tests; all pass**
- [ ] **Step 5: Commit** `git add` the explicit files, then `git commit -m "Add optional per-wheel torque allocation hook"`

### Task 2: Signal map and controller protocol with a fake controller

**Files:**
- Create: `src/simplified_vehicle_models/cosim/__init__.py`, `signal_map.py`, `controller.py`
- Test: `tests/test_cosim_signal_map.py`

**Interfaces:**
- Produces:
  - `SignalMap.from_json(path) -> SignalMap`, with schema `"fmu_signal_map_v1"` and fields:
    - `inputs: list[{fmu, source, scale, offset}]`. `source` is drawn from a closed vocabulary: `vx, vy, yaw_rate, ax, ay, steering, throttle, brake_force, wheel_speed_fl..rr, soc, lateral_error, heading_error, path_curvature, preview_curvature`.
    - `outputs: {kind: "motor_torque" | "delta_torque", fmu: [...], scale}`.
    - `communication_step_s`.
    - `parameters: {name: value}`.
  - `SignalMap.inputs_from(observation, plant_signals) -> dict[str, float]`.
  - `CoSimController` protocol: `reset(start_time, parameters)`, `step(time, inputs: dict, dt) -> dict`, `identity() -> dict`.
  - `FakeYawPIController(gain, integral_gain, understeer_gradient, wheelbase, max_delta_torque)`: a deterministic 2DoF-reference yaw-rate PI that outputs `delta_torque`.

- [ ] **Step 1: Write the failing tests**

```python
class SignalMapTest(unittest.TestCase):
    def test_rejects_unknown_source_and_missing_step(self):
        with self.assertRaises(ValueError):
            SignalMap.from_dict({"schema": "fmu_signal_map_v1", "inputs":
                [{"fmu": "r", "source": "yaw_accel", "scale": 1, "offset": 0}],
                "outputs": {"kind": "delta_torque", "fmu": ["dT"], "scale": 1}})

    def test_iso_sign_conversion_is_explicit(self):
        # ISO 8855 yaw rate is left-positive; the repository is right-positive.
        m = SignalMap.from_dict({"schema": "fmu_signal_map_v1", "communication_step_s": 0.005,
            "inputs": [{"fmu": "r", "source": "yaw_rate", "scale": -1.0, "offset": 0.0}],
            "outputs": {"kind": "delta_torque", "fmu": ["dT"], "scale": 1.0}, "parameters": {}})
        self.assertEqual(m.inputs_from(obs(yaw_rate=0.5), {})["r"], -0.5)

    def test_fake_controller_is_deterministic(self):
        a, b = FakeYawPIController(...), FakeYawPIController(...)
        for c in (a, b):
            c.reset(0., {})
        seq = [{"vx": 15., "steering": 0.05, "yaw_rate": 0.1}] * 50
        out_a = [a.step(i * .005, s, .005) for i, s in enumerate(seq)]
        out_b = [b.step(i * .005, s, .005) for i, s in enumerate(seq)]
        self.assertEqual(out_a, out_b)
```

- [ ] **Step 2: Run and confirm failure.** Run `PYTHONPATH=src python3 -m unittest tests.test_cosim_signal_map -v`.
- [ ] **Step 3: Implement** `SignalMap` (strict keys, finite numbers, positive step) and `FakeYawPIController`:
  - Reference yaw rate: `r_ref = vx*δ/(L + K_us*vx²)`.
  - Output: `dT = clip(kp*e + ki*∫e, ±max)`.

  Implement `FmuController` as a thin `fmpy` wrapper:
  - Use `fmpy.read_model_description`, `extract`, then `FMU2Slave`/`FMU3Slave`.
  - In `reset`, instantiate, set parameters, call `enterInitializationMode`/`exit...`.
  - In `step`, call `setReal` + `doStep` + `getReal`.

  Import `fmpy` lazily, and raise a clear error when the extra is not installed.
- [ ] **Step 4: Tests pass.** Also add `fmu = ["fmpy>=0.3"]` to `pyproject.toml` optional dependencies.
- [ ] **Step 5: Commit**

### Task 3: Zero-order-hold co-simulation allocator

**Files:**
- Create: `src/simplified_vehicle_models/cosim/hold.py`
- Test: `tests/test_cosim_hold.py`

**Interfaces:**
- Consumes: `TorqueAllocator` (Task 1), `CoSimController` and `SignalMap` (Task 2), `solve_forward(..., maximum_step, step_sink)` (existing).
- Produces:
  - `HeldControllerAllocator(controller, signal_map, plant, driven)`, which implements `TorqueAllocator`. It returns the held output inside the RHS and calls `on_accepted_step(sample)` from `step_sink`.
  - `run_cosim(plant, driver, controller, signal_map, initial_state, duration, *, wall_seconds) -> CoSimResult` (forward result plus a log of controller inputs and outputs per communication step).

Design: the RHS must be a pure function of `(t, x)` between communication points. The allocator therefore returns the **held** output only. On each accepted step that crosses the next communication boundary, it advances the controller with inputs built from the accepted state. `solve_forward` is called with `maximum_step <= communication_step_s`. Held outputs change only at accepted steps, so a rejected Newton trial can never advance controller state.

- [ ] **Step 1: Write the failing tests**

```python
class HoldTest(unittest.TestCase):
    def test_rhs_trials_do_not_advance_controller(self):
        counter = CountingController()
        alloc = HeldControllerAllocator(counter, signal_map(step=0.01), plant, driven)
        for _ in range(20):
            alloc.motor_torque(request_at(t=0.003))
        self.assertEqual(counter.steps, 0)

    def test_controller_advances_once_per_boundary(self):
        result = run_cosim(plant, driver, CountingController(), signal_map(step=0.01),
                           x0, duration=0.1, wall_seconds=30.)
        self.assertEqual(len(result.controller_log), 10)

    def test_repeat_run_is_identical(self):
        a = run_cosim(...); b = run_cosim(...)
        np.testing.assert_array_equal(a.forward.state, b.forward.state)
```

- [ ] **Step 2: Confirm failure. Step 3: Implement.** Boundary crossing uses integer step indices (`floor(t/dt + 1e-9)`), never accumulated floats. Convert `delta_torque` to per-motor torque as `requested ± dT/2`, with the right-positive sign applied explicitly.
- [ ] **Step 4: Tests pass**, with a short (2 s) real planar run using `FakeYawPIController`. **Step 5: Commit.**

### Task 4: Manoeuvre library and self-tests

**Files:**
- Create: `src/simplified_vehicle_models/cosim/manoeuvres.py`
- Test: `tests/test_cosim_manoeuvres.py`

**Interfaces:**
- Produces:
  - `Manoeuvre` (frozen) with fields `name`, `path_builder() -> (distance, curvature)`, `speed_profile`, `settle_s` and `measure_window`.
  - Builders: `steady_radius(radius, speed_ramp)`, `clothoid_corner(speed, a_x, curvature_peak, entry_length, hold_length, exit_length)`, `step_steer(speed, amplitude)`, `sine_sweep(speed, f0, f1)`.
  - `run_manoeuvre(manoeuvre, plant_factory, driver_factory, controller_factory, signal_map) -> ManoeuvreOutcome`. Fields: `completed`, `max_lateral_error`, `max_combined_utilization`, `peak_yaw_rate_error`, `stable`, `trace`.
  - Self-tests: `sign_check(...)` and `repeatability_check(...)`.

- [ ] **Step 1: Write the failing tests:**
  - A clothoid corner's `path_builder` curvature integrates to the requested heading change, within 1e-9.
  - A positive-curvature (right) step steer gives positive yaw rate on the plant.
  - `sign_check` with `FakeYawPIController` reports that a positive yaw-rate error produces the right-turning torque difference.
  - Two runs of `repeatability_check` match exactly.
  - A controller whose outputs are reversed in sign fails `sign_check`.
- [ ] **Step 2: Confirm failure. Step 3: Implement.** The plant starts from a settled trim: the existing forward-runner seed, followed by a `settle_s` hold. A manoeuvre fails if any of these hold: lateral error > 0.25 m (the existing forward gate), the plant rejects a trial state, or the yaw-rate error oscillates without decaying within the measurement window.
- [ ] **Step 4: Tests pass. Step 5: Commit.**

### Task 5: Map extraction (A, B, C) and baseline pairing

**Files:**
- Create: `src/simplified_vehicle_models/cosim/yaw_map_harness.py`
- Test: `tests/test_fmu_yaw_map_harness.py`

**Interfaces:**
- Consumes:
  - `YawCapabilityMap` from proposal v2 Phase 1 (`src/simplified_vehicle_models/formulations/yaw_capability.py`). Fields:
    - grid: `speed`, `longitudinal_acceleration`, `lateral_acceleration`;
    - authorities: `yaw_moment_into_turn`, `yaw_moment_out_of_turn` (N·m, both ≥ 0), with `yaw_inertia` stored alongside;
    - envelope: `acceleration_max`, `acceleration_min`, `feasible_mask`;
    - provenance: `source_kind` and `source_identity`.
  - The physical LP builder `build_physical_yaw_map(vehicle, grid, tv_limits)`.
- Produces:
  - `build_fmu_yaw_map(vehicle, plant_factory, driver_factory, controller_factory, signal_map, grid, *, workers, wall_seconds) -> FmuMapResult`. It holds the `fmu` map, the `baseline` map (equal-split controller), `validation` and `cells` (one outcome record per cell).
  - Effective yaw authority from a clothoid-entry bisection: `M_into = I_z * (v² κ'_max + a_x κ)`, where `κ'_max` is the sharpest entry the closed loop follows (likewise for exit).
  - Map C is stored in `cells` only; it is diagnostic.

- [ ] **Step 1: Write the failing tests** (fake controller, coarse 2×2×2 grid):
  - The baseline map is built with the equal-split controller.
  - On every feasible cell, the FMU map's `yaw_moment_into_turn` is at most the physical LP's ideal-TV bound times (1 + bisection tolerance).
  - A fake controller with zero gain gives a map equal to the baseline, within bisection tolerance.
  - A cell whose manoeuvre fails at every probe is marked infeasible, not zeroed as if it had capability.
  - Results with `workers=1` and `workers=2` are identical.
- [ ] **Step 2: Confirm failure. Step 3: Implement.**
  - Bisect on clothoid entry length from a geometric starting bracket, with a fixed iteration count (8 by default). Report the bracket width as uncertainty.
  - Run cells in a `ProcessPoolExecutor`. Each process gets its own controller instance, and results are ordered by cell index.
  - `validation` records cells where FMU > physical bound (a fault), cells where FMU < baseline (a finding to report, not an error), and coverage.
- [ ] **Step 4: Tests pass. Step 5: Commit.**

### Task 6: CLI, cache provenance and report

**Files:**
- Create: `tools/run/build_fmu_yaw_map.py`
- Modify: `formulations/yaw_capability.py` save/load to accept `source_kind="fmu_closed_loop_v1"`
- Test: extend `tests/test_fmu_yaw_map_harness.py`

**Interfaces:**
- Produces: CLI `--fmu PATH --signal-map JSON --vehicle XML --grid JSON --output DIR --workers N --wall-seconds S`. Outputs: `fmu_map.npz`, `baseline_map.npz`, `report.json`.
- Provenance (`source_identity`):
  - FMU SHA-256, FMI version, model identifier;
  - the parameter values from the signal map, the signal-map SHA-256, the communication step;
  - vehicle and plant source hashes, driver gains;
  - manoeuvre and grid definitions, bisection settings.

- [ ] **Step 1: Write the failing tests.** Round-trip save and load preserves `source_identity`. Loading a map whose stored FMU hash differs from the given `--fmu` raises an error. The CLI refuses to overwrite existing output.
- [ ] **Step 2–4: Implement and pass.** Run one real smoke build with the fake controller packaged as a Python controller (no FMU binary) on a 2×2×2 grid.
- [ ] **Step 5: Commit.**

### Task 7: Planner integration check

**Files:** Test only: `tests/test_fmu_yaw_map_harness.py` (integration).

- [ ] **Step 1:** Using the fake-controller map and its baseline, run the proposal-v2 line optimizer on the synthetic 90° corner fixture. Assert that the corner time with the FMU map is ≤ the baseline time, and that the FMU line's rotation length `L50` is ≤ the baseline's.
- [ ] **Step 2:** Run a closed-loop lap on the planar plant with `HeldControllerAllocator`, driving the FMU-map line. Assert that the lap completes, the existing forward gates pass, and the driven trajectory stays inside the corridor.
- [ ] **Step 3:** Commit, then request an independent review of sign conventions and baseline pairing.

## Known risks (record at unshelve time)

- **Platform:** Simulink FMUs are often win64/linux64 only. Decide on a source-code FMU or a Linux container before starting Task 2's `FmuController`.
- **MPC FMUs:**
  - An MPC with path-error traction control may fight `PreviewDriver`. Record its path-error and preview inputs explicitly in the signal map, and have `sign_check` flag opposing steering-equivalent action.
  - Configure fixed iteration counts and single-threaded solves for determinism.
  - Expect 10–100× cost per step.
- **Plant mismatch:** a controller designed on a different plant is being characterized on this one. The map describes *this plant with that controller*. State this in `report.json`.
