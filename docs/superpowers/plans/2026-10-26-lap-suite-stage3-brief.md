# Stage3 brief: convergence and lap-suite driver

Implement only after bounded Stage2 interfaces stable. Public module planning/yaw_convergence.py separates profile refinement, optimizer stationarity, physical geometry-grid convergence, and control-basis convergence. Do not derive one boolean from another.

Data record fields: label, geometry_spacing_m, control_spacing_m, lap_time_s, reference_distance ndarray, speed ndarray, offset ndarray, profile_valid bool, profile_resolution_converged bool, optimizer_converged bool, budget_reason optional. Comparison resamples onto common reference station overlap and records relative lap-time difference, max speed difference, max offset difference. At least two distinct geometry spacings needed grid verdict; two distinct control spacings needed basis verdict. All compared records independently valid/no budget, tolerance passing, and fixed other resolution knobs. Unmatched baseline => inconclusive. Multiple same-resolution repeats => no mesh/basis convergence. Failed typed solves remain explicit rows with causes.

Create tests synthetic records differing only relevant dimension, same-grid repeats false, budget/invalid inconclusive, nonfinite rejected, reference station interpolation correct for unequal point counts. Use frozen dataclasses readonly arrays.

Lap suite tools/run/run_yaw_lap_suite.py accepts cached map path, track, output fresh directory, mode fixed|line|replay|convergence; all underlying executions process guarded1800s via existing run_with_deadline outer entrypoint. Settings wall budget<=1500 reserve for persistence. No automatic map rebuild if artifact stale/incomplete. Store inputs, exact settings/map identity, selected line/speed plan NPZ, convergence JSON, replay summary/record when requested. A replay of persisted plan must not redo line optimization. Separate plan generation from replay to avoid 45minute repeated planning old runner issue.

Persist enough path source station, x/y,heading,curvature,rate,profile speed/ax/time,coefficients/control spacing, track/map hash for validated load/replay. Loader verifies source identities and strict all-edge profile validation. New speed plan must follow selected optimized geometry, not original track centreline. Closed seam consistency checked.

Avoid claiming overall goal complete until MIS/FSC concrete bounded probes and tracking evidence collected. No OCP edits, no map physical model edits. Tests/commands process watchdog<=1800 mandatory. Report docs/superpowers/plans/2026-10-26-lap-suite-stage3-report.md.
