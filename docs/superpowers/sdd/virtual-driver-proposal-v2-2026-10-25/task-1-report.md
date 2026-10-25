# Phase 1 task report: yaw capability map

## Delivered

- Added `formulations/yaw_capability.py` with immutable `YawCapabilityGrid` and `YawCapabilityMap` data structures, read-only defensive array copies, finite/order/shape/bounds checks, feasible-cell mask, positive explicit yaw inertia, and source provenance.
- Added physical no-TV and bounded physical-TV builder variants, plus an ideal-TV upper-bound builder. The builders solve deterministic per-cell linear programs with SciPy `linprog(method="highs")`; SciPy remains optional and a clear runtime error is raised when it is unavailable.
- Added focused unit tests for grid ordering and speed bounds, defensive read-only fields, provenance and shape/bound validation, TV limit validation, and building all three source variants with ideal-vs-physical bound checks.
- No existing `qss_*` OCP file was modified.

## Round-1 review repair

The first review correctly identified missing assembled drivetrain handling and weak map validation. The builder now reads driven wheels from `FullDrivetrainRuntime.drivetrain.transmission` and constrains undriven wheel forces to zero. It caps aggregate tractive force with the existing assembled `maximum_drive_force` envelope (or legacy torque/power envelope). It enforces the standard front/rear brake-bias split unless brake or regen vectoring is explicitly authorized; left/right torque vectoring bounds are omitted for an ideal-TV upper bound and controlled by `TvLimits` for physical TV. `regen_vectoring_allowed`/`brake_vectoring_allowed` now affect braking-cell constraints. Map construction now checks nonnegative speed and nonempty string provenance, and the steady view rejects grids without an exact zero-acceleration sample.

Repair tests cover source variants and bounds, validated axis/provenance, exact steady-axis requirement, and braking-flag feasible-set monotonicity. The implementation remains an aggregate-envelope approximation: motor limits are applied to total tractive force, not a full per-wheel motor torque/power dispatch LP; regenerative power and per-wheel brake hardware limits are not independently modeled. It should be treated as conservative prototype capability rather than full powertrain validation.

## Test results

`PYTHONPATH=src python3 -m unittest tests.test_yaw_capability -v` — 4 tests passed.

## Constraints and known limitations

The current LP is a first implementation and does **not** yet satisfy every physical detail enumerated in proposal §4. It models per-tire friction polygons, force balance, load-dependent axle lateral-force sharing, no-TV left/right equalization for longitudinal wheel forces, and a physical TV left/right delta-force bound. It does not yet enforce the assembled powertrain's motor torque/power envelope or brake-bias distribution. `regen_vectoring_allowed` and `brake_vectoring_allowed` are retained in validated TV parameters/provenance but do not yet specialize the TV constraints by propulsion/braking mode. These omissions can make some reported cells optimistic and should be addressed before treating these maps as calibrated vehicle capability. The point-mass vehicle type does not carry yaw inertia, so builders deliberately require the caller to supply an explicit finite positive `yaw_inertia`; it participates in provenance.

The tests cover structural validation and variant/provenance behavior, not exhaustive force-envelope comparisons, full mirror-symmetry numerics, or actual powertrain-limit equivalence. No FMU work was done.
