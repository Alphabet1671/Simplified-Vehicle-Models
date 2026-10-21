# Task 4 implementation report — sampled quasi-steady capability surface

## Scope and interface
Implemented the Task 4 sampled capability surface consumed by Task 5. `CapabilitySurface` stores speed and signed-curvature axes, speed-by-curvature maximum/minimum acceleration arrays, per-curvature speed ceilings, and a provenance hash. It validates finite arrays and shape contracts, copies arrays, and makes them read-only. Save/load use compressed NumPy NPZ with pickle disabled on load.

## Implementation
- Added `src/simplified_vehicle_models/formulations/point_mass_capability.py` with surface construction, persistence, and provenance hashing. Every grid node delegates to `point_mass.acceleration_limits`; ceilings use existing NumPy equilibrium/limit APIs.
- Ceiling detection checks contact/lateral equilibrium feasibility via `_equilibrium_limits` before interpreting `maximum_acceleration_limit`. The latter's documented sentinel of zero for a laterally infeasible zero-longitudinal state is not considered a feasibility proof. The feasible speed boundary is bracketed and treated as a cap when the acceleration-limit sentinel cannot form a valid root bracket. The boundary is biased just within the feasible side of the numerical bracket.
- `capability_hash` canonicalizes dataclasses, nested mapping/sequence/NumPy values, public object attributes and slots, and stable enum values before SHA-256 hashing. Unsupported object types raise rather than silently hashing unstable repr output. This includes nested tire/drivetrain runtime physics rather than only top-level numeric attributes.
- Added regression coverage for axes and array ordering, NumPy grid-node reproduction, feasible high-curvature ceiling behavior, deterministic/nested-physics-sensitive hashing, archive round-trip and invalid curvature input.
- Added the helper script `tools/model_data/build_capability_surface.py`, including its documented module and direct-script entrypoints and argument interface.

## Tests and commands
Focused suite command:

```text
PYTHONPATH=src python3 -m unittest tests.test_point_mass_capability -v
```

Exact final output:

```text
test_axes_are_sorted_and_symmetric (tests.test_point_mass_capability.CapabilitySurfaceTest.test_axes_are_sorted_and_symmetric) ... ok
test_ceiling_rejects_laterally_infeasible_high_curvature_top_speed (tests.test_point_mass_capability.CapabilitySurfaceTest.test_ceiling_rejects_laterally_infeasible_high_curvature_top_speed) ... ok
test_hash_stable_hex_and_nested_physics_sensitive (tests.test_point_mass_capability.CapabilitySurfaceTest.test_hash_stable_hex_and_nested_physics_sensitive) ... ok
test_limits_reproduce_numpy_physics_at_grid_nodes (tests.test_point_mass_capability.CapabilitySurfaceTest.test_limits_reproduce_numpy_physics_at_grid_nodes) ... ok
test_rejects_bad_curvature_limit (tests.test_point_mass_capability.CapabilitySurfaceTest.test_rejects_bad_curvature_limit) ... ok
test_round_trip_preserves_every_array (tests.test_point_mass_capability.CapabilitySurfaceTest.test_round_trip_preserves_every_array) ... ok
test_speed_ceiling_is_feasible_and_acceleration_boundary (tests.test_point_mass_capability.CapabilitySurfaceTest.test_speed_ceiling_is_feasible_and_acceleration_boundary) ... ok

----------------------------------------------------------------------
Ran 7 tests in 2.294s

OK
```

Helper entrypoint check:

```text
PYTHONPATH=src python3 -m tools.model_data.build_capability_surface --help
```

Passed; usage displayed required `--formulation`, `--track`, `--output` and optional `--curvature-margin`.

`git diff --check` passed. Full suite not run; prior plan ledger records full-suite runs exceeding the 120-second limit. The Task 4 integration build that writes the MIS surface was not run; this was deliberately not generated as a committed `runs/` artifact.

## Self-review
- Verified the array indexing convention is `(speed, curvature)` and ceiling is indexed by curvature.
- Verified each sampled acceleration entry reproduces the authoritative NumPy point-mass limits within the test's tight tolerance.
- Reproduced and fixed the high-curvature false-top-speed hazard: the feasibility-aware ceiling does not trust a zero acceleration-limit result from infeasible equilibrium and uses contact/lateral feasibility to establish the relevant boundary.
- Verified nested tire physics mutation changes the provenance hash while repeated hashing is stable and hexadecimal.
- Verified save/load retains every array and provenance value.
- Checked CLI helper `--help` as required by the brief; did not run the full 121×81 production surface due the stated multi-minute budget.

## Fix round 1 — review findings and validation

Addressed the scoped review findings after the initial implementation commit:
- Hash serialization now includes qualified type tags for structured objects/dataclasses, plus typed sorted mapping key/value pairs. Added collision regressions for integer-vs-string mapping keys, same-field dataclass types, and same-public-attribute classes.
- Ceiling logic now explicitly distinguishes an acceleration-zero root from the contact/lateral feasibility cap. It checks signs at the low and backed-off boundary endpoints before calling `brentq`; nonpositive low residual returns the low endpoint, positive boundary residual returns the feasibility cap, and near-zero boundary residual returns the boundary.
- Updated the helper docstring to describe a highest contact/lateral feasible cap or acceleration-zero root, whichever limits first, with a backed-off feasibility boundary.
- Added direct tests for root-before-feasibility and a feasibility cap that stays inside the feasible range.

Focused command:

```text
PYTHONPATH=src python3 -m unittest tests.test_point_mass_capability -v
```

Exact fix-round output:

```text
test_acceleration_zero_root_precedes_lateral_feasibility_cap (tests.test_point_mass_capability.CapabilitySurfaceTest.test_acceleration_zero_root_precedes_lateral_feasibility_cap) ... ok
test_axes_are_sorted_and_symmetric (tests.test_point_mass_capability.CapabilitySurfaceTest.test_axes_are_sorted_and_symmetric) ... ok
test_ceiling_rejects_laterally_infeasible_high_curvature_top_speed (tests.test_point_mass_capability.CapabilitySurfaceTest.test_ceiling_rejects_laterally_infeasible_high_curvature_top_speed) ... ok
test_hash_distinguishes_mapping_key_types (tests.test_point_mass_capability.CapabilitySurfaceTest.test_hash_distinguishes_mapping_key_types) ... ok
test_hash_distinguishes_public_attribute_classes (tests.test_point_mass_capability.CapabilitySurfaceTest.test_hash_distinguishes_public_attribute_classes) ... ok
test_hash_distinguishes_same_fields_different_types (tests.test_point_mass_capability.CapabilitySurfaceTest.test_hash_distinguishes_same_fields_different_types) ... ok
test_hash_stable_hex_and_nested_physics_sensitive (tests.test_point_mass_capability.CapabilitySurfaceTest.test_hash_stable_hex_and_nested_physics_sensitive) ... ok
test_lateral_feasibility_cap_is_backed_into_feasible_interval (tests.test_point_mass_capability.CapabilitySurfaceTest.test_lateral_feasibility_cap_is_backed_into_feasible_interval) ... ok
test_limits_reproduce_numpy_physics_at_grid_nodes (tests.test_point_mass_capability.CapabilitySurfaceTest.test_limits_reproduce_numpy_physics_at_grid_nodes) ... ok
test_rejects_bad_curvature_limit (tests.test_point_mass_capability.CapabilitySurfaceTest.test_rejects_bad_curvature_limit) ... ok
test_round_trip_preserves_every_array (tests.test_point_mass_capability.CapabilitySurfaceTest.test_round_trip_preserves_every_array) ... ok
test_speed_ceiling_is_feasible_and_acceleration_boundary (tests.test_point_mass_capability.CapabilitySurfaceTest.test_speed_ceiling_is_feasible_and_acceleration_boundary) ... ok

----------------------------------------------------------------------
Ran 12 tests in 2.467s

OK
```

`git diff --check` and compile checks passed for the fix round. Review fixes committed separately as `8149a71` (`fix: harden capability provenance and ceiling roots`).

## Concerns / limitations
- Production MIS surface generation and its runtime/file size remain unmeasured. The brief estimates 5–20 minutes; this report does not claim that estimate as measured.
- The feasibility bisection assumes a contiguous feasible speed interval from the low-speed probe, consistent with the physical cornering envelope used here.
