# Suspension layouts: Task 1 baseline

Recorded on branch `feature/docs-tools-layout` at `948518d`, before any source change.
Plan: [2026-10-17-suspension-layouts.md](../../plans/2026-10-17-suspension-layouts.md).

## Suite status on the untouched tree

Command: `PYTHONPATH=src python3 -m unittest discover -s tests`

Result: `Ran 1102 tests in 310.741s`, `FAILED (errors=7)`. Wall time is about 5 min.

All seven errors are `FileNotFoundError` in
`tests/test_vertical_14dof_evidence.py::PinnedTraceEvidenceTests`. Each one opens a `trace.csv`
under `docs/validation/iem24-murphy/14dof_simple_trials/**/attempt_*/`. `.gitignore` line 30
(`docs/validation/**/attempt_*/trace.csv`) ignores those files, so they do not exist in this
checkout. The failures do not depend on the suspension code.

**Gate used for Tasks 1-7:** the suite must show exactly these seven errors and nothing else.
Any other failure is mine. The seven can be cleared only by restoring the traces, which is outside
this plan. The Task 3 evidence regeneration also writes new traces, so check the count again then.

## Golden-number pin

`tests/test_suspension_baseline.py` with the fixture `tests/fixtures/suspension_baseline.json`.
It records, from the current code:

- `corner_forces` breakdown (spring, ARB, damper, total, damper velocity, out-of-range) for four
  jounce/rate samples: rest, heave bump, roll, warp with rebound.
- `evaluate_vertical_dynamics` rates, generalized forces, normal loads and corner totals for three
  mechanical states.
- the static equilibrium and a steady corner (15 m/s, 6 m/s^2).
- the plant initial state, initial rates, jounce trial bounds, and the state after a 0.6 s
  forward run.

Single-evaluation checks use rtol = atol = 1e-12. The forward run uses 1e-9 because the adaptive
stepper amplifies ulp differences. To re-record, run
`PYTHONPATH=src python3 tests/test_suspension_baseline.py --record`. Do this only for a deliberate
physics change, never to make a refactor pass.

Only `test_corner_force_breakdown_is_unchanged` reaches into `vehicle.suspension` and
`vehicle.dampers`. Task 3 must change that one call to the new network API.

## Inventory of suspension reach sites

Counts are matching lines from `grep` for `suspension.front|rear`, `.dampers`, `corner_forces(`,
`corner_toe_angles`, and the type names.

### Library

| File | What it reaches |
|---|---|
| `formulations/vertical_14dof.py` | `corner_forces(vehicle.suspension, ..., vehicle.dampers)` (l. 417); `suspension.front/rear.roll_centre_height` (425-426); anti geometry per axle (457-461); inertance and `motion_ratio_damper` (535-550); dataclass fields `suspension`, `dampers` and the compile at l. 206 |
| `solvers/vertical_forward.py` | `suspension.front/rear.jounce_limits` (171-172); `corner_toe_angles` (265); `corner_forces.damper_velocity` (446) |
| `solvers/vertical_equilibrium.py` | `corner_toe_angles` (451); spring forces per axle (650-651) |
| `suspension/corner.py` | the model itself: `AxleSuspension`, `SuspensionModel`, `corner_forces`, `corner_toe_angles` |
| `suspension/components.py` | `compile_damper`, `compile_suspension_corner`, `register_component` |

### Tools

| File | What it reaches |
|---|---|
| `tools/run/run_iem24_forward_driver.py` | `jounce_limits` (l. 306); roles `suspension, damper-front, damper-rear` (l. 110); `suspension/corner.py` in `_VERTICAL_SOURCES` (l. 60) |
| `tools/run/sweep_vertical_setup.py` | builds `SuspensionModel` and `dampers=` directly (l. 372); 7 `suspension.front/rear` and 5 `.dampers` sites |
| `tools/reports/compare_iem24_four_way.py` | `vertical.dampers[0].setting.clicks` (l. 624) |

### Tests that build or replace suspension objects

`test_vertical_14dof.py` (15 + 9 `replace`), `test_vertical_acceptance.py` (16 + 6, plus a
`ZeroDamper` pair at l. 127), `test_vertical_forward.py` (8 + 3; the closed-form mass, stiffness
and damping helpers at l. 125-210 read `.front/.rear` and `.dampers`), `test_vertical_equilibrium.py`
(6 + 4 + 2), `test_vertical_setup_sweep.py` (4 + 10), `test_suspension_corner.py`,
`test_vertical_14dof_compile.py`, `test_forward_driver_runner.py` (roles and capability ids),
`test_planar_7dof.py` (the allow-list of 14DoF-only roles).

## Facts that later tasks depend on

- Loader: `ref=` is resolved on **every** element of **any** model file
  (`for element in root.iter()` in `xml/loader.py`), so a suspension XML can already carry
  child references. The plan's claim A-loader holds. What blocks nesting is
  `require_no_references` plus the two hard-coded nested types in `_assembly_candidates`.
- `compile_formulation` records a nested provider as loaded only through a hard-coded
  `powertrain_drivetrain` branch (compile.py l. 111-115). Task 2 replaces it with a registry flag.
- The IEM24 corner NPZ (`models/data/iem24_murphy_real_corners.npz`) has static loads
  505.78 N (front) and 646.50 N (rear) per corner and jounce limits +/-0.0318 m. Its sha256 must
  not change in Task 3.
- `tools/run/run_iem24_forward_driver.py` hashes `suspension/corner.py` and the vertical sources.
  Touching any of them changes the source hash that the stored acceptance attempt pins; Task 3
  regenerates that evidence once.
