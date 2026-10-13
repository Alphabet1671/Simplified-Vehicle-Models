# Voltage-Inequality Motor Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Implement approved Fix 1, start the canonical pack at regulation voltage, and validate regenerated LUTs against the corrected analytical source.

**Architecture:** Preserve runtime query signatures and LUT default. Correct the shared dq oracle and conservative reference-loss adjustment. Let battery XML specify either initial SOC or initial open-circuit bus voltage, compiled to the existing initial SOC field. Version/fingerprint persisted maps so stale tables cannot coexist with a changed fallback.

**Tech Stack:** Python, NumPy, unittest, XML, compressed NPZ.

**Spec:** `docs/superpowers/specs/2026-10-12-motor-800v-options.md`, Option 1; user approval and regulation-voltage start clarification.

## Global Constraints
- No merge or push; keep production voltage cap at 600 V and LUT default.
- Preserve physical battery voltages and the exact out-of-domain fallback.
- Higher-voltage tests establish model behaviour, not hardware certification.

### Task 1: Correct analytical source
**Files:** `powertrain/power_plant.py` and `tests/test_power_plant.py`.
**Interfaces:** Preserve `solve_direct_current(...) -> DirectCurrentResult` and `evaluate_shaft_power_exact(...) -> ShaftPowerResult`.
- [x] Add failing headroom/negative-voltage/signed-torque/boundary/high-voltage-loss/reference-calibration tests using `compile_analytical_motor_source`.
- [x] Reject negative available voltage; return `id=0` for `c<=0`; use stable roots and scale-aware tolerance otherwise.
- [x] Add a private unclamped calibration-voltage helper. At/above reference use map losses; below reference add only nonnegative extra d-axis copper loss. Actual feasibility is authoritative.
- [x] Run source tests and commit the corrected source together with regenerated artifacts before declaring the branch runnable.

### Task 2: Regulation-voltage initialization and map regeneration
**Files:** `powertrain/energy_storage.py`, battery XML, `tests/test_energy_storage.py`, generator, loader, canonical motor XML/map and provenance.
**Interfaces:** XML accepts exactly one of `initial_state_of_charge` (1) or `initial_bus_voltage` (V); compiled consumers continue to use `initial_state_of_charge`.
- [x] Test voltage-to-SOC roundtrip, invalid/out-of-range inputs and ambiguous declarations.
- [x] Invert the monotone source OCV curve for `initial_bus_voltage=600`; do not cap the battery curve or change capacity.
- [x] Add shared analytical model version/fingerprint metadata to generator and loader; reject old-version or mismatched-source tables even with correct SHA-256.
- [x] Regenerate 530–600 V canonical map twice, compare digests, update declaration/provenance and assert no near-zero low-speed drive nodes.
- [x] Generate an 800 V test artifact against an explicitly configured test motor; verify analytical/forward/inverse agreement and productive regen masks.

### Task 3: Validation and reporting
**Files:** affected regression tests, validation documentation, selected design status.
- [x] Run safety sweeps for voltage feasibility, torque caps and pack power, including signed torque and 800 V tests.
- [x] Measure explicit analytical/LUT comparison for three isolated runs; re-pin changed golden laps only after attribution.
- [x] Require <=1 ms lap delta, <=0.05 m/s speed delta, both converged, and LUT runtime win; if a gate fails, report and investigate rather than claim promotion-safe behaviour.
- [x] Run full unittest discovery, review diff, record initial SOC/voltage, all gate results, source-model limitations and exact fallback behaviour.
- [x] Commit focused changes and leave the worktree clean.
