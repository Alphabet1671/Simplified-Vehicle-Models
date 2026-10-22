# Task 5 — optional energy-NLP profile line seed

## Delivered interface

`build_profile_line_seed(track, corridor, surface, mesh, spline, settings, *, profile_step=2., maximum_profile_iterations=200, maximum_profile_seconds=30.) -> (seed | None, diagnostics)` is implemented in `src/simplified_vehicle_models/solvers/profile_line_seed.py`.

The factory constructs `_reference_curve(track)`, an exact zero-coefficient `OffsetTrajectory`, and an analytic path. It calls the independent `fixed_path_energy_profile` with `flying` for closed tracks and `rolling` for open tracks at `max(settings.start_speed, settings.minimum_speed)`. The profile gets explicit step, iteration, and IPOPT soft wall-budget arguments.

The profile is projected in physical path arclength using `trajectory.station_map.to_path(mesh)`. It interpolates `W = v²`, derives node acceleration as `W_ell / 2` using deterministic one-sided endpoint differences, and computes the midpoint acceleration that makes the energy Hermite--Simpson defect exactly zero:

`am = [6(W1-W0)/h - 2G0a0 - 2G1a1] / (8Gm)`.

It neither clips speed/acceleration/domain queries nor claims OCP feasibility. The factory performs only a cheap global variable-bound check to reject nonfinite/layout-invalid or variable-bound-violating projected seeds; it does not construct a duplicate OCP. A returned candidate is explicitly labelled `projection_only` and `ocp_feasible: false`; full constraint-row feasibility remains the OCP's responsibility.

## Harness option and failure behavior

The no-TV matrix harness has `--seed-profile {none,energy-nlp}` (default `none`) and `--strict-profile-seed`.

* `none` preserves the existing initializer exactly.
* `energy-nlp` replaces only a centerline seed when the profile factory returns one.
* Profile nonconvergence, unsupported legacy inputs, geometry/domain errors, or projected-bound failures leave the established seed intact and record `profile_seed.fallback = "explicit-old-seed"` plus diagnostics.
* `--strict-profile-seed` fails instead of using that fallback.
* The pre-existing `low_amplitude_offset` perturbation is preserved rather than overwritten with a centreline profile; strict mode rejects this incompatible request explicitly.

No solver default, physics, tolerance, TV behavior, source data, or profile/reference backend selection was changed.

## Tests run

```text
PYTHONPATH=src:/tmp/no-tv-casadi-runtime python3 -m unittest tests.test_profile_line_seed tests.test_no_tv_minimum_lap -q
Ran 34 tests in 0.060s
OK
```

Focused coverage verifies physical-ell energy interpolation, nonunit-metric exact HS repair, rolling/flying boundary dispatch, unavailable/legacy-style profile handling, CLI/request serialization, and explicit worker fallback recording.

## Limits

This is a cheap optional initializer, not an assertion of fixed-path or free-line optimality. The fixed-path NLP's wall limit remains its documented IPOPT soft internal budget; the harness worker remains the hard wall-limit owner. A bounded actual FSC closed construction diagnostic used the signed cache, 5 m mesh, 35 m basis, 32 off-collocation sites, limited-memory setting, `maximum_profile_iterations=100`, and a 30 s profile budget. Its profile converged, but the unmodified projection exceeded the global OCP acceleration variable bound by `0.26228`, so the factory returned `None` with `reason: variable-bound-violation:0.26228`. Even an arbitrarily small positive variable-bound excess is rejected (rather than relying on the solver's later clipping). This is the intended explicit limitation: it did not clip the candidate and did not silently supply a profile seed. No free-line OCP was run here.
