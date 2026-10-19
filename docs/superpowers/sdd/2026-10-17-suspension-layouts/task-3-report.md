# Task 3 report: suspension seam with the independent-corner layout

Plan: [2026-10-17-suspension-layouts.md](../../plans/2026-10-17-suspension-layouts.md), Task 3.
Type: pure refactor. No result may move.

## What changed

* The suspension compiles to `CompiledSuspension(kinematics, network, layout)`.
  * `suspension/kinematics.py`: per-wheel geometry (`motion_ratio`, roll centre, anti geometry, camber gain, `toe`, `jounce_limits`, `static_sprung_load`).
  * `suspension/network.py`: the `ForceNetwork` Protocol and `NetworkForces`.
  * `suspension/layouts/independent_corner.py`: the old spring, ARB, damper and inerter law, moved without arithmetic changes. It also holds the compile-time static-load check.
  * `suspension/elements.py`: value holders for the spring curves and ARB rates.
  * `suspension/corner.py` is reduced to the NPZ loader.
* One new capability, `suspension-forces`, replaces `suspension-corner`, `front-damper-force` and `rear-damper-force` in the 14DoF formulation.
* The suspension XML is an assembly (`suspension-assembly/independent-corner`) that nests five children: kinematics, spring, ARB and two dampers. The manifest records them as `suspension/<role>`.
* `VerticalVehicleParameters.suspension` replaces `.front`, `.rear` and `.dampers`. No shim is left.
* Tools ported: `generate_suspension_lut.py` (writes the assembly and three part files), `run_iem24_forward_driver.py`, `sweep_vertical_setup.py` (`source_hashes` follows nested children), `check_14dof_simple_gates.py`, `compare_iem24_four_way.py`.
* The corner NPZ is byte-identical (`04bb06c9…f0f0`).

## Evidence that nothing moved

| Check | Result |
|---|---|
| `tests/test_suspension_baseline.py` golden numbers (rtol = atol = 1e-12) | 5 of 5 pass, fixture unchanged |
| 14DoF cold lap | 66.56339750 s, 20 075 steps, 74 486 evaluations, minimum contact load 97.887 N: identical to the pre-refactor attempt |
| Refinement against the periodicity baseline | lap-time difference 7.65e-7 s, max speed difference 9.36e-05 m/s: identical |
| Planar 7DoF reference lap, regenerated | 66.53234595 s: identical (the planar code was not touched) |
| Seven plan gates | all pass |
| Manifest diff of all five formulations | only the intended changes: the 14DoF capability set and nested children, and the planar 7DoF unused list |

## Evidence regeneration

`source_hashes` and the gate tool's sha256 pin the source files, so the acceptance evidence was regenerated once from the refactored sources (plan Q5 default).

| Role | New attempt |
|---|---|
| Cold | `14dof_simple_trials/attempt_1792288406878879000` |
| Periodicity | `14dof_simple_trials/periodicity/attempt_1792288811970287000` |
| Refinement | `14dof_simple_trials/refinement/attempt_1792288547293860000` |
| Planar reference | `seven_dof_track_pair/original/attempt_1792288979150821000` |

The old attempts stay in the repository and are marked superseded in `docs/validation/14dof-simple-parameters.md` and `docs/model-data/provenance.md`. The trace files are gitignored, so they exist only on this machine. The planar reference trace had to be regenerated for the gate tool to run.

## Errors met on the way

* A baseline-suite exit code hidden by `| tail`. Fixed by logging to a file.
* `sweep_vertical_setup` `source_hashes` did not reach the nested NPZ. Fixed with a recursive collector.
* The planar 7DoF test allow-list of 14DoF-only roles changed to `("suspension", "tire-transient")`.

## Not verified

* The regenerated traces are not committed, so the evidence test's pinned-trace checks pass only where the traces were regenerated.
* No wall-time claim is made beyond the one lap: 140.1 s child wall against 138.0 s before, on a machine under other load.
