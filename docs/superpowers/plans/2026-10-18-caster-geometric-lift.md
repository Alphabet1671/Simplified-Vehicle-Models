# Geometric Steering Lift Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Provide an opt-in front-wheel steering-induced hub-height change to the simplified 14DoF model, preserving existing behavior with no geometry measurement.

**Architecture:** Keep optional per-front-wheel sine and cosine lift coefficients in compiled suspension kinematics, defaulting to zero, and evaluate `h(δ)=b sinδ+a(1-cosδ)` in a dedicated helper. A mirrored 3-D steering axis can yield an odd lift/drop term, so an even-only approximation is insufficient. Apply height to hub-down/tire compliance in both dynamic evaluator and equilibrium trial calculations; include steering-rate-induced hub velocity when supplied. Do not add a second force-pair or pretend the existing caster angle determines the unknown coefficient. Forward steering trajectories lack acceleration, so nonzero-rate commands with enabled lift are rejected until a smooth steering acceleration is available.

**Tech Stack:** Python, NumPy, XML model schema, unittest.

**Spec:** `docs/dev/caster-jacking-14dof-investigation.md`

## Global Constraints

- Never disclose private-source locations or filenames in committed documentation; call their provenance “from real car data.”
- The existing vehicle configuration must have zero lift until a measured signed effective lift coefficient is supplied.
- Steering lift is a position-level geometric coupling, not an extra tire-load force or roll-centre jacking term.
- Preserve nonzero rear-wheel steering compatibility by defining zero rear lift.

---

### Task 1: Opt-in validated geometry and geometry tests

**Files:** Modify `src/simplified_vehicle_models/suspension/kinematics.py`, `src/simplified_vehicle_models/suspension/components.py`; test `tests/test_suspension_layouts.py`.

**Interfaces:** `SuspensionKinematics.steering_lift(steering) -> (height[4], slope[4], curvature[4])`; `AxleGeometry.steering_lift_sine_left/right` and `steering_lift_cosine_left/right` default to zero; XML optional four front coefficients in meters. Provide a smooth lift/drop curve with zero height at zero steer.

- [x] Add tests for zero default, signed sine/cosine front response, analytic derivatives, zero rear, invalid nonfinite coefficient, and XML opt-in.
- [x] Run focused tests.
- [x] Add the optional validated coefficient and geometry helper; run tests again.

### Task 2: Vertical dynamics and steady-trial consistency

**Files:** Modify `src/simplified_vehicle_models/formulations/vertical_14dof.py`, `src/simplified_vehicle_models/solvers/vertical_equilibrium.py`, `src/simplified_vehicle_models/solvers/vertical_forward.py`; test `tests/test_vertical_14dof.py`, `tests/test_vertical_equilibrium.py`.

**Interfaces:** `evaluate_vertical_dynamics(..., network_state=None, *, steering_rate=0.0, steering_acceleration=0.0)`; front hub-down `z_body − jounce − h(δ)`; hub-rate `z_dot − jounce_rate − h'(δ)*δdot`; mass block unchanged and forcing receives the known unsprung inertial term `m_u*(h'' δdot²+h' δddot)*c_i`. Steady trial applies same height with zero steering rate. Forward plant obtains steering acceleration from its command trajectory if available; until then explicitly reject nonzero lift when steering rate changes discontinuously.

- [x] Add evaluator tests for nonzero-lift normal loads at fixed state, signed steer, rear unaffected, steering-rate damping and acceleration; test equilibrium trial load consistency and forward rate rejection.
- [x] Run focused tests and fix a mistaken signed-steer expectation.
- [x] Apply geometry to both vertical paths and forward steering rate; focused suites pass.

### Task 3: Documentation and final verification

**Files:** Modify `docs/dev/caster-jacking-14dof-investigation.md`; test focused and full suites.

- [x] Document XML opt-in, signed lift/drop convention, missing geometry data, why forward steering ramps are rejected until acceleration is known, and validation measurements needed.
- [x] Run 118 focused vertical/equilibrium/suspension/forward tests and `git diff --check`; no unrelated files staged. Full 1206-test suite has five expected source-hash provenance failures because a historical acceptance artifact pins the previous source bytes; do not rewrite historical evidence or claim it validates this change.
