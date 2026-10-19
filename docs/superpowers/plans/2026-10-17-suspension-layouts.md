# Suspension Layout Generalisation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

> **Status: PLAN ONLY. Nothing in this document has been implemented.** No model, source, test or tool file has been changed. Open questions are listed under "Open decisions" and must be answered before the tasks they gate begin.

**Goal:** Let the `14dof-simple` vehicle accept suspension layouts other than four independent corners: roll/heave-decoupled axles (a heave spring and damper plus a roll spring and damper per axle), a four-mode decoupled layout (heave, roll, pitch, warp), and a hydraulically interconnected layout. The IEM24 car must keep producing the same numbers on the current layout.

**Architecture:** Every layout is a function from the four jounces, the four jounce rates and any internal suspension states to four wheel-referred corner forces. The 14DoF evaluator already turns corner forces into generalised forces by virtual work, so it does not need to know what produced them. The change is therefore a seam, not a new vehicle model. A compiled suspension is split into **kinematics** (geometry that is per wheel and independent of the force law) and a **force network** (the layout-specific part). The vehicle assembly names one `suspension` role. The suspension assembly nests its own children, in the same way the powertrain and drivetrain assemblies already do.

**Tech Stack:** Python 3.12, NumPy (library), SciPy BDF/Radau (forward solver), unittest. No new dependencies.

**Spec:** [project-master-plan.md](../../project-master-plan.md) §3.4 and §5; builds on [2026-10-16-vertical-14dof-model.md](2026-10-16-vertical-14dof-model.md).

> Plan style: like the 14DoF plan, this gives interfaces, test assertions and commands instead of full code bodies.

## Current state (read 2026-10-17)

| # | Finding | Evidence |
|---|---|---|
| C1 | The vehicle assembly names three suspension roles: `suspension`, `damper-front`, `damper-rear`. The formulation requires `suspension-corner`, `front-damper-force`, `rear-damper-force`. | `models/vehicles/iem24_murphy_real_planar.xml`, `models/formulations/iem24_murphy_real_14dof_simple.xml` |
| C2 | `SuspensionModel` is a `(front, rear)` pair of `AxleSuspension`. One axle mixes geometry (motion ratio, roll centre, anti-brake/anti-drive, camber gain, jounce steer, limits, static toe) with force elements (spring curve, ARB rate, inertance). | `suspension/corner.py:61-114` |
| C3 | `corner_forces()` is a fixed law: per-wheel spring, per-wheel damper through the motion ratio, and an ARB of `0.5*k*(zL - zR)`. It takes `dampers=(front, rear)`. | `suspension/corner.py:138-176` |
| C4 | The evaluator reads `.front` / `.rear` directly for roll-centre height, anti-brake/anti-drive, inertance and motion ratio. Other consumers do the same. | `formulations/vertical_14dof.py:370,425-426,457-461,535-546`; `solvers/vertical_forward.py:171,265`; `solvers/vertical_equilibrium.py:451,650`; `tools/run/run_iem24_forward_driver.py:306`; `tools/run/sweep_vertical_setup.py` (many); many tests |
| C5 | `VerticalState` is a fixed 21-vector. The plant has 38 states (21 mechanical, 12 tire, plus the driveline and powertrain states). The inerter adds `b/MR^2` to the jounce diagonal of the mass block; there is no other suspension state. | `formulations/vertical_14dof.py:216-255,346-374` |
| C6 | **Nesting is hard-coded.** `_assembly_candidates` skips only `powertrain-assembly` and `drivetrain-assembly` and builds a synthetic provider for them. Every other component goes through `require_no_references`, which forbids a reference inside a component. | `compiler/compile.py:186-187,205-230`; `xml/schema.py:89` |
| C7 | The formulation hook is a hard-coded `if key == (...)` chain. Capabilities are string constants. | `compiler/compile.py:306-352`; `compiler/capabilities.py:33-35` |
| C8 | The XML loader accepts only scalar `<parameter>` elements and csv/npz `<data>` (14DoF plan finding F1). A hydraulic circuit graph therefore has to live in an NPZ or in a flat parameter list. | 2026-10-16 plan, F1 |
| C9 | The damper NPZ is already shared by the front and rear damper XML. The corner NPZ holds both axles. | `models/dampers/*.xml`, `models/data/iem24_murphy_real_corners.npz` |

## What the layouts need

| Layout | Force network | Internal states | Couples wheels by |
|---|---|---|---|
| Independent corner (today) | spring + damper per wheel, one ARB per axle | none | ARB only |
| Roll/heave per axle | per axle: heave spring+damper on `(zL+zR)/2`, roll spring+damper on `(zL-zR)` | none | the axle mode transform |
| Four-mode decoupled | heave, roll, pitch, warp spring+damper acting on the four mode coordinates | none | the full 4x4 mode transform |
| Hydraulic interconnected | cylinders, lines, orifices/valves, accumulators | accumulator volumes (first), optionally line flow | the fluid circuit |

The first two stay stateless. They are exactly the corner-force map `F = T^T * F_mode(T z, T zdot)` with an orthogonal mode matrix `T`. Only the hydraulic layout needs states.

## Open decisions (to be answered by the user)

These are deliberately **not** resolved here.

| ID | Question | Why it matters | Default if you have no preference |
|---|---|---|---|
| Q1 | Do the `.front` / `.rear` / `.dampers` compatibility shims stay permanently, or only until Task 3 ends? | Permanent shims keep old tools working but keep the old shape in the API. | Remove after Task 3 |
| Q2 | Keep the formulation name `14dof-simple` for all layouts (the suspension assembly selects the layout), or add one formulation subtype per layout? | A per-layout subtype changes `compile.py`, the registry and the runner model names. | Keep the name |
| Q3 | Mode units: wheel-referred rates (N/m of wheel travel, no geometry needed) or physical units (N·m/rad, N/m, which need track and wheelbase at compile time)? | Wheel-referred keeps the suspension component independent of the vehicle assembly. | Wheel-referred |
| Q4 | How far should the hydraulic layout go? (a) accumulator and orifice states only; (b) plus line compressibility; (c) plus fluid inertance in the lines. | Each step adds states and stiffness. (a) is a quasi-static circuit with accumulator dynamics. | (a), then extend if needed |
| Q5 | Is it acceptable that Tasks 3-4 invalidate the stored `14dof-simple` acceptance evidence, so one fresh cold lap, one warm baseline and one refinement run must be repeated? | The gate tool pins source hashes and trace hashes. | Yes, regenerate once at the end of Task 3 |
| Q6 | Should roll centre and anti-geometry stay in the kinematics component, or become properties of the layout assembly? | A hydraulic or active layout could change them; today they are pure geometry. | Kinematics |

Tasks 1-2 do not depend on any of these. Task 3 depends on Q1 and Q5. Task 4 depends on Q3. Task 6 depends on Q4. Q2 gates Task 2.

## Target file structure

```
models/
  vehicles/<car>.xml                     roles: suspension (damper-front and damper-rear leave the vehicle)
  suspensions/                           type suspension-assembly, one subtype per layout
    iem24_murphy_real_suspension.xml     subtype independent-corner   (today's behaviour, regenerated)
    <car>_roll_heave_axle.xml            subtype axle-roll-heave
    <car>_four_mode.xml                  subtype four-mode
    <car>_hydraulic.xml                  subtype hydraulic-network
  suspension-kinematics/                 type suspension-kinematics: geometry only (reads the corner NPZ)
  springs/                               spring-model: rate curves with preload
  anti-roll-bars/                        arb-model
  dampers/                               existing, plus generic modal dampers
  hydraulics/                            hydraulic-circuit: graph, cylinders, accumulators, valves
  data/                                  existing corner NPZ unchanged; new circuit NPZs
```

Each `suspension-assembly` subtype declares its own required child roles, in the same way a formulation lists `requires`.

| Subtype | Child roles |
|---|---|
| `independent-corner` | `kinematics`, `spring`, `arb`, `damper-front`, `damper-rear` |
| `axle-roll-heave` | `kinematics`, and per axle `heave-spring`, `heave-damper`, `roll-spring`, `roll-damper` |
| `four-mode` | `kinematics`, `heave-spring`, `roll-spring`, `pitch-spring`, `warp-spring` and the four matching dampers |
| `hydraulic-network` | `kinematics`, `circuit` |

Example resolved trees:

```
suspension -> <car>_four_mode.xml [suspension-assembly / four-mode]
   |- kinematics   -> suspension-kinematics/<car>.xml
   |- heave-spring, roll-spring, pitch-spring, warp-spring -> springs/*.xml
   `- heave-damper, roll-damper, pitch-damper, warp-damper -> dampers/*.xml

suspension -> <car>_hydraulic.xml [suspension-assembly / hydraulic-network]
   |- kinematics   -> suspension-kinematics/<car>.xml
   `- circuit      -> hydraulics/<car>_circuit.xml [cylinders FL..RR, nodes, accumulators, valves]
```

## The interface between suspension and evaluator

The compiled suspension exposes two parts.

**Kinematics** (per wheel, no force law):

- `motion_ratio(i, z)`, `roll_centre_height(axle, roll)`, `roll_centre_lateral(axle, roll)`
- `anti_brake(axle)`, `anti_drive(axle)`, `camber_gain(axle)`
- `toe(z)` returning the four offsets that `corner_toe_angles` returns today
- `jounce_limits` per wheel (replaces `[front.jounce_limits]*2 + [rear.jounce_limits]*2`)

**Force network:**

- `n_states`, `initial_state(z_static)`
- `forces(z, zdot, s) -> NetworkForces` with `total` (4,), a named per-element breakdown, `out_of_range` (4,) and `ds_dt`
- `inertia_block(z) -> (4,4)` and `inertial_forcing(z, zdot) -> (4,)`. These replace the inertance special case at `vertical_14dof.py:346-374,535-550`. The default implementation returns the same `b/MR^2` diagonal.
- `static_check(z=0) -> (4,)` returning the corner loads at zero jounce. Compile fails if this does not equal the sprung static corner loads, which is the existing "zero jounce is static equilibrium" convention.

The existing `CornerForces` breakdown (`spring`, `arb`, `damper`) stays for the independent-corner layout. Other layouts report their own named elements, so the trace schema must allow a variable element list (Task 4).

## Global Constraints

- **Golden numbers must not move in Tasks 1-3.** Task 1 records them. The independent-corner layout is a pure refactor and must reproduce them to the recorded tolerance. Also keep the existing golden flying-lap values listed in the 14DoF plan's Global Constraints.
- Do not change `planar_7dof.py`, `planar_forward.py` or the steady UniTire code. The planar 7DoF formulation must keep compiling against the same shared assembly (`tests/test_planar_7dof.py` requires `unused_components == ()` and `unconsumed_parameters == ()`; the allow-list of 14DoF-only names changes with the new roles).
- Library code is NumPy only.
- Every table is an external NPZ with a `sha256` in its `<data>` element. `iem24_murphy_real_corners.npz` and `tools/model_data/generate_suspension_lut.py` stay unchanged unless Q6 moves a quantity.
- No silent extrapolation. Out-of-table queries stay counted, and the existing gates on the counters stay.
- Conventions: SAE J670 axes, wheel order FL, FR, RL, RR, jounce positive in compression, force positive in compression (pushes the wheel down and the body up).
- Each PROPOSED value gets an XML comment with the word `PROPOSED` and its source.
- Full suite `PYTHONPATH=src python3 -m unittest discover -s tests -q` passes at the end of every task.
- Do not delete committed evidence. Mark superseded evidence as superseded in its report.

## File map

| File | Change |
|---|---|
| `src/.../suspension/kinematics.py` | NEW. Kinematics component; wheel-indexed geometry accessors |
| `src/.../suspension/network.py` | NEW. `ForceNetwork` Protocol, `NetworkForces`, mode transform helpers |
| `src/.../suspension/layouts/independent_corner.py` | NEW. Today's spring/ARB/damper law behind the Protocol |
| `src/.../suspension/layouts/axle_roll_heave.py`, `four_mode.py` | NEW. Stateless modal layouts (Tasks 4-5) |
| `src/.../suspension/layouts/hydraulic.py`, `src/.../hydraulics/` | NEW. Circuit graph and element models (Task 6) |
| `src/.../suspension/corner.py` | Reduced to the NPZ loader and spline-based geometry; `corner_forces` moves into the independent-corner layout |
| `src/.../suspension/components.py` | Registers the new component types |
| `src/.../compiler/compile.py`, `compiler/capabilities.py`, `compiler/registry.py` | Generalise nested assemblies; add `suspension-forces` and `suspension-kinematics` capabilities |
| `src/.../formulations/vertical_14dof.py` | Consume the two parts; remove `.front` / `.rear` reaches; extend `VerticalState` handling for suspension states |
| `src/.../solvers/vertical_forward.py`, `vertical_equilibrium.py` | Use kinematics limits and network state; solve initial network state |
| `tools/run/run_iem24_forward_driver.py`, `tools/run/sweep_vertical_setup.py` | Use the new accessors |
| `tools/model_data/generate_suspension_lut.py` | Emits the new assembly + kinematics + spring + ARB XML for the existing car |
| `models/vehicles/iem24_murphy_real_planar.xml`, `models/formulations/iem24_murphy_real_14dof_simple.xml` | Drop the two damper roles and capabilities (Task 3) |
| `tests/test_repository_layout.py` | Read before Task 2 to see whether new top-level `models/` directories need registering (only line 59 matched a `models/` pattern, so this may be unnecessary) |
| `docs/validation/14dof-simple-parameters.md`, `docs/project-master-plan.md` §3.4/§5 | Document the new structure and the regenerated evidence |

## Feasibility audit

| # | Area | Finding | Result |
|---|---|---|---|
| A1 | Nested components | C6: nesting exists only for powertrain/drivetrain. The loader does resolve `ref=` inside any model file, since `powertrain-assembly` does it. The restriction sits in `_assembly_candidates` and `require_no_references`. | Task 2 generalises it with a registry flag (`nests_components=True`) instead of another hard-coded type list. Risk: manifest and hash recording for nested children must still include every child file, since the runner records component-file hashes. |
| A2 | Capability model | The formulation requests capability strings; each provider declares one. Two roles with the same capability (front/rear damper) needed two distinct constants. | Do not repeat that. The suspension assembly is the single provider of `suspension-forces`; per-axle choices live inside it. |
| A3 | Evaluator seam | Corner forces enter through `total_force` (one line) and the mass block. Everything else is geometry. | Feasible. The only coupling is the roll-centre/anti/inertance reaches listed in C4. |
| A4 | Stateless layouts | A linear mode transform has no state. | Feasible without touching the solver. |
| A5 | State vector | `VerticalState` is a frozen 21-field dataclass, `from_vector` checks shape `(21,)`, and `VERTICAL_ACCELERATION_INDEX` is fixed. The plant is 38 states. | Task 6 appends suspension states after the existing ones, so existing indices do not move. `RateSystem` already supports a mass matrix, but the accumulator states are first-order. Needs the runner's state-name and tolerance metadata (14DoF plan F10) to extend. |
| A6 | Solver | Wheel hop is 16-18 Hz and BDF is used. An accumulator adds a time constant set by orifice resistance and gas stiffness, which can be fast. | Feasible; measure on a short run before the lap. If stiff, BDF handles it; a block Jacobian stays optional. |
| A7 | Equilibrium | `solve_static` / steady-corner solves assume zero-state suspension. A hydraulic circuit has a static pressure that depends on the preload. | Task 6 solves the network's own static equilibrium (`initial_state`) before the vehicle solve, then checks `static_check`. |
| A8 | Nonlinearity | Modal decoupling is exact only for linear elements. A nonlinear (gas) spring on a mode is a modelling choice, not a decoupling theorem. | State this in the layout docs and in the parameters document. Hydraulic Task 6 is the physical version. |
| A9 | Tests that reach into internals | Many tests use `vehicle.suspension.front`, `.rear` and `vehicle.dampers`, and build modified vehicles with `dataclasses.replace(vehicle.suspension, front=...)`. | These need either the shims (Q1) or rewriting. Count and classify them in Task 1 before choosing. |

## Tasks

### Task 1: Baseline and inventory (no behaviour change)

**Files:** NEW `docs/superpowers/sdd/2026-10-17-suspension-layouts/baseline.md`; NEW `tests/test_suspension_baseline.py` (kept as a permanent regression gate).

- [ ] **Step 1:** Record the golden numbers for the current IEM24 car: `corner_forces` at a set of jounce/jounce-rate samples (including both ARB directions and the out-of-range flag), the static equilibrium jounces, one steady-corner solution, and a short (about 100 m) forward run's state at fixed times. Pin them with explicit tolerances.
- [ ] **Step 2:** Write the pinning test so it passes on the current code. Run it. Expected: PASS before any other change.
- [ ] **Step 3:** Grep-inventory every `.suspension.front`, `.suspension.rear`, `.dampers` and `dataclasses.replace(...suspension...)` site. Classify each as (a) geometry read, (b) force-law read, (c) test fixture mutation. Put the table in `baseline.md`. This answers Q1.
- [ ] **Step 4:** Read `tests/test_repository_layout.py` and `tests/test_vertical_14dof_evidence.py` and record exactly which files and hashes they pin.
- [ ] **Step 5:** Commit: `git commit -m "test: pin the 14DoF suspension baseline before layout refactor"`.

### Task 2: Generalise nested assemblies in the compiler

Depends on Q2.

**Files:** `compiler/compile.py`, `compiler/registry.py`, `compiler/capabilities.py`, `xml/schema.py` (only if needed); tests `tests/test_compiler_nested_assembly.py` (NEW).

**Interfaces:** `register_component(..., nests_components=True, child_roles=(...))`. The powertrain and drivetrain assemblies are re-registered through this flag with unchanged behaviour. Each nested child appears in the manifest with its role path (for example `suspension/kinematics`).

- [ ] **Step 1:** Write failing tests: a synthetic assembly with a nested child compiles; a duplicate child role fails; a missing required child role fails with a message naming the role; manifest lists every child file and hash.
- [ ] **Step 2:** Replace the hard-coded `{"powertrain-assembly", "drivetrain-assembly"}` sets in `_assembly_candidates` with the registry flag. Keep the powertrain/drivetrain synthetic provider path as is.
- [ ] **Step 3:** Run the full suite. Expected: PASS with identical powertrain results and identical existing manifests (compare the manifest of the current 14DoF vehicle before and after).
- [ ] **Step 4:** Commit: `git commit -m "refactor: register nested assemblies instead of hard-coding powertrain types"`.

### Task 3: Suspension seam with the independent-corner layout (pure refactor)

Depends on Q1, Q5, and Task 2.

**Files:** NEW `suspension/kinematics.py`, `suspension/network.py`, `suspension/layouts/independent_corner.py`; modify `suspension/corner.py`, `suspension/components.py`, `formulations/vertical_14dof.py`, `solvers/vertical_forward.py`, `solvers/vertical_equilibrium.py`, `tools/run/run_iem24_forward_driver.py`, `tools/run/sweep_vertical_setup.py`, the suspension and damper XML files, `models/vehicles/iem24_murphy_real_planar.xml`, `models/formulations/iem24_murphy_real_14dof_simple.xml`, `tools/model_data/generate_suspension_lut.py`, affected tests.

**Interfaces:** As in "The interface between suspension and evaluator". `VerticalVehicleParameters.suspension` becomes the assembly's compiled value exposing `.kinematics` and `.network`. `VerticalVehicleParameters.dampers` is removed (or kept as a shim, Q1).

- [ ] **Step 1:** Write the new unit tests first: kinematics accessors return the values the old `AxleSuspension` fields returned; the independent-corner network returns bit-identical `total`, `spring`, `arb`, `damper`, `damper_velocity`, `out_of_range` to the old `corner_forces`; `static_check` equals the sprung static loads; `inertia_block` equals the old inertance diagonal.
- [ ] **Step 2:** Implement kinematics, the network Protocol and the independent-corner layout by moving, not rewriting, the existing code.
- [ ] **Step 3:** Change the evaluator, forward plant, equilibrium solver and tools to call the two parts. Remove the `front if i < 2 else rear` branches in favour of per-wheel kinematics accessors.
- [ ] **Step 4:** Regenerate the suspension XML (assembly, kinematics, spring, ARB) with the generator. The corner NPZ and its sha256 must be unchanged.
- [ ] **Step 5:** Drop the `damper-front` / `damper-rear` roles and the two damper capabilities from the vehicle and formulation XML. Update the allow-list in `tests/test_planar_7dof.py` for the 14DoF-only names.
- [ ] **Step 6:** Rewrite or shim the tests classified in Task 1 step 3.
- [ ] **Step 7:** Run the baseline test, then the full suite. Expected: baseline numbers unchanged within the recorded tolerance, everything else PASS. A baseline difference is a bug in the refactor, not a tolerance to widen.
- [ ] **Step 8 (if Q5 = yes):** Re-run the cold lap, warm periodicity and refinement attempts and the gate tool; mark the previous acceptance attempt superseded in `docs/validation/14dof-simple-parameters.md`. Do not delete it.
- [ ] **Step 9:** Commit in two steps: the structural change, then the regenerated evidence.

### Task 4: Stateless modal layouts (roll/heave per axle, then four-mode)

Depends on Q3 and Task 3.

**Files:** NEW `suspension/layouts/axle_roll_heave.py`, `suspension/layouts/four_mode.py`, `models/springs/`, `models/anti-roll-bars/` and example assembly XML for each layout; tests `tests/test_suspension_layouts.py` (NEW). Trace schema: allow a named variable element list in the run result.

**Interfaces:** The mode matrix for four wheels in FL, FR, RL, RR order, normalised so that `T T^T = I`:

```
heave = (zFL + zFR + zRL + zRR) / 2
roll  = (zFL - zFR + zRL - zRR) / 2
pitch = (zFL + zFR - zRL - zRR) / 2
warp  = (zFL - zFR - zRL + zRR) / 2
```

and `F_corner = T^T F_mode(T z, T zdot)`. The per-axle layout applies the 2x2 version to each axle separately. Mode rates are defined as wheel-referred (Q3 default), so no geometry enters.

- [ ] **Step 1:** Write the failing tests: (a) with equal mode rates `k` and no damping, the layout reproduces four independent springs of rate `k`; (b) a pure heave, roll, pitch or warp displacement produces force only in that mode (check `T F` is zero in the other three); (c) a single-wheel displacement gives the analytic stiffness matrix `T^T K_mode T`; (d) force is zero at zero jounce after the static preload is accounted for; (e) a per-axle roll-only rate reproduces the existing ARB law when the roll rate equals the ARB rate and the other modes are set to the corner rates; (f) a non-orthogonal `T` or a mode with a negative rate fails at compile time.
- [ ] **Step 2:** Implement the transform helpers once in `network.py`, then the two layouts on top.
- [ ] **Step 3:** Decide how the motion ratio enters. Mode forces are wheel-referred, so dampers on a mode need a velocity-referral choice. Default: apply the damper to the wheel-referred mode velocity with no extra motion ratio, and document it. Ask the user if this is not acceptable, because it changes how damper tables are read.
- [ ] **Step 4:** Add example assemblies using the existing IEM24 spring and damper numbers where an exact equivalent exists (per-axle roll/heave with the current corner rates and the rear ARB must match the independent-corner result; this is test (e)).
- [ ] **Step 5:** Run an equilibrium solve and a short forward run on each layout. Expected: contact valid, zero out-of-range counters, and no change to the baseline when the equivalent parameters are used.
- [ ] **Step 6:** Commit.

### Task 5: Layout-aware setup sweeps and reporting

**Files:** `tools/run/sweep_vertical_setup.py`, `tools/reports/compare_iem24_four_way.py` (only the lines that read dampers), `docs/validation/14dof-simple-parameters.md`.

- [ ] **Step 1:** Make the sweep tool take mode rates (heave, roll, pitch, warp) for the modal layouts, with the existing click and ARB sweeps kept for the independent-corner layout.
- [ ] **Step 2:** Make the four-way report read damper settings through the layout, not `vertical.dampers[0].setting.clicks`.
- [ ] **Step 3:** Run `tests.test_mis_report_standard` and the repository layout tests.
- [ ] **Step 4:** Commit.

### Task 6: Hydraulically interconnected layout

Depends on Q4, Q3 and Tasks 3-4.

**Files:** NEW `src/.../hydraulics/` (circuit graph, cylinder, orifice/valve, accumulator), `suspension/layouts/hydraulic.py`, `models/hydraulics/<car>_circuit.xml`, `models/data/<car>_circuit.npz`, `tools/model_data/generate_hydraulic_circuit.py`; changes to `VerticalState` handling, `solvers/vertical_forward.py` and `solvers/vertical_equilibrium.py`; tests `tests/test_hydraulics.py`, `tests/test_suspension_hydraulic.py` (NEW).

**Interfaces:** The circuit is stored as an NPZ because the loader accepts only scalars and csv/npz (C8): node list, cylinder-to-node incidence with piston areas (bore and rod side), line orifice tables (pressure drop versus flow), accumulator gas preload, volume and polytropic exponent, valve tables. The XML keeps only the scalar parameters and the pinned `sha256`.

**State layout.** Suspension states are appended after the existing plant states so no existing index moves. Their number and names come from the network (`n_states`, `state_names`), and the runner reads them from plant metadata. Option (a) in Q4 gives one state per accumulator (gas volume or pressure); (b) adds line volumes; (c) adds line flow rates.

- [ ] **Step 1:** Write the failing element tests: a cylinder converts jounce and rate to flow with the right sign and area; an orifice table is monotone and has a counted out-of-range flag; an accumulator's pressure follows the polytropic law and is bounded below; mass (volume) is conserved around the circuit.
- [ ] **Step 2:** Write the **reduction oracle**: a cross-plumbed circuit whose line resistances are made very small must reproduce the four-mode layout's forces (stiff-circuit limit), and one with the lines closed must reproduce four independent gas springs. These use the Task 4 code as an independent check.
- [ ] **Step 3:** Implement the elements and the circuit assembly. Solve the algebraic node pressures at each evaluation (Q4 option (a)) or carry them as states (b/c).
- [ ] **Step 4:** Implement `initial_state` as the circuit's static equilibrium. Compile fails if the static corner loads do not match the sprung loads within a stated tolerance.
- [ ] **Step 5:** Extend the runner's state names and periodic tolerances for the suspension states; extend the evidence gates so a state outside its physical range (negative pressure, gas volume outside bounds) fails a named gate.
- [ ] **Step 6:** Run equilibrium and a 100 m forward run. Report wall time against the independent-corner baseline before any full lap.
- [ ] **Step 7:** Commit.

### Task 7: Documentation and project record

**Files:** `docs/project-master-plan.md` (§3.4, §5, §15), `docs/dev/` (a short "adding a suspension layout" note), `docs/validation/14dof-simple-parameters.md`, a task report under `docs/superpowers/sdd/2026-10-17-suspension-layouts/`.

- [ ] **Step 1:** Document the assembly subtypes, their child roles, the sign and unit conventions, and the modal-decoupling limitation (A8).
- [ ] **Step 2:** Record which parameters in the example layouts are PROPOSED.
- [ ] **Step 3:** Run the full suite and `git diff --check`.

## Risks

| # | Risk | Mitigation |
|---|---|---|
| R1 | The refactor changes numbers by accident. | Task 1 pins them first; Task 3 treats any difference as a bug. |
| R2 | The stored acceptance evidence goes stale (the gate tool pins sha256 of source files and traces). | Q5. Regenerate once, supersede, never delete. |
| R3 | Generalising nesting breaks the manifest or the recorded component hashes (`test_forward_driver_runner` checks component-file hashes). | Task 2 compares manifests before and after on the current car. |
| R4 | Test churn from `.front` / `.rear` reaches is large. | Task 1 inventories it first; Q1 chooses shims or rewrites. |
| R5 | Hydraulic states make the plant stiff or slow. | Short-run timing before any lap; BDF already in use; block Jacobian optional. |
| R6 | A modal layout with nonlinear elements is mistaken for exact decoupling. | A8 disclosure in the docs and the parameters document. |
| R7 | The loader cannot hold a circuit graph in XML. | Store the graph in a pinned NPZ (C8). |

## Out of scope

- Camber, jounce-steer and compliance models beyond what the kinematics component already carries.
- Active or semi-active control of valves and dampers.
- Fitting any of the new elements to measured data. All new parameters are PROPOSED until a fitting tool exists.
- Changing the planar 7DoF or the point-mass formulations.
- Changing the `14dof-complete` plan (camber-sensitive full UniTire and fitted parameters).
