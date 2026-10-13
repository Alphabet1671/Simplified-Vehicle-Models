# Motor LUT Promotion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use subagent-driven-development or executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Promote the validated LUT to the assembled default and propose, but do not yet implement, an architecture-compatible 800 V source-model correction.

**Architecture:** Change only the assembled mode default; keep the analytical mode selectable and preserve exact out-of-domain fallback. Fix numerical expectations by testing each operating point using its selected mode, not by loosening tolerances. Investigate the d-axis voltage inequality and reference-map loss correction separately.

**Tech Stack:** Python, NumPy, unittest, XML model compilation.

**Spec:** User request: “switch. Also come up with a fix for 800V, propose solutions again to be compatible with the current architecture.”

## Global Constraints
- Do not merge or push.
- Retain the configurable 600 V regulatory clamp and keep battery terminal voltage physical.
- Do not implement an unapproved 800 V source-model redesign.
- Preserve the analytical oracle and exact out-of-domain fallback.

### Task 1: Promote and validate
**Files:** `src/simplified_vehicle_models/powertrain/assembly.py`, `tests/test_powertrain.py`, `tests/test_mis_regression.py`, `docs/validation/powertrain.md`, `docs/superpowers/specs/2026-10-12-motor-lut.md`.
**Interfaces:** `PowertrainParameters.motor_mode` becomes `"lut"`; `"analytical"` remains supported.
- [x] Assert compiled `powertrain.motor_mode == "lut"`; run that test and observe failure.
- [x] Replace the dataclass default with `motor_mode: str = "lut"`.
- [x] Re-pin assembled lap at `69.20391204362008` seconds, keeping explicit analytical coverage at `69.20459210208878` seconds.
- [x] Use `evaluate_motor` for a LUT assembly operating-point consistency assertion; keep explicit analytical mode in cross-mode coupling regression tests.
- [x] Run `PYTHONPATH=src python -m unittest tests.test_powertrain tests.test_mis_regression tests.test_energy_lap -q`, then full discovery with `-s tests -q`; investigate every failure.
- [x] Update current default documentation and commit the focused switch.

### Task 2: Propose 800 V solutions
**Files:** Create `docs/superpowers/specs/2026-10-12-motor-800v-options.md`.
**Interfaces:** Preserve `solve_direct_current`, `evaluate_shaft_power_exact`, LUT forward/limit/inverse queries and the generator's shared analytical source.
- [x] Probe a corrected voltage-inequality solve in memory without altering production code: `if available_voltage < 0: infeasible; elif c <= 0: id = 0; else: choose minimum-magnitude feasible quadratic-boundary root`.
- [x] Compare existing and candidate solver over voltage/speed/torque grids, including signed torque, standstill and points infeasible at reference voltage but feasible at higher voltage.
- [x] Document a recommended conservative correction, a calibrated signed-loss correction alternative, and a full current-constrained optimization alternative, including changes to reference feasibility, regen, LUT metadata/rebuilding, and validation gates.
- [x] Clearly distinguish regulatory limit, source reference voltage, and actual hardware rating; recommend an option without treating a software cap as regulatory compliance.
