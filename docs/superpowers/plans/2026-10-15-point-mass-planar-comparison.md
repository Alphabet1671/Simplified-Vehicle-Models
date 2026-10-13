# Point-Mass and Planar Lap Comparison Plan

> **For agentic workers:** implement this plan task-by-task, checking each result before continuing.

**Goal:** Compare the point-mass and planar solvers on the same placeholder vehicle and closed Michigan track, derive point-mass tire grip/load sensitivity from simplified UniTire instead of the legacy friction ellipse, benchmark elapsed time, and test point-mass-derived planar warm starts.

**Architecture:** Add a small UniTire-to-point-mass pure-force-envelope adapter that samples pure longitudinal/lateral peak force versus normal load from the already compiled simplified UniTire model. Build the point-mass parameter object from the same compiled placeholder assembly (mass, geometry, aero, drivetrain), then run both solvers on the same signed track profile. For warm starts, resample point-mass speed to the planar OCP mesh, construct kinematic states and approximate pedals, locally project loads/wheel speeds/current, and compare against the equilibrium-seed baseline with the same OCP settings.

**Tech Stack:** Python, NumPy, existing point-mass forward/backward solver, simplified UniTire evaluator, CasADi/IPOPT planar OCP.

**Spec:** Direct user request in this session: use the same models and track, replace legacy 2025 Matlab lap-sim tire load sensitivity/grip peaks with simplified UniTire values, compare lap time then compute time, and investigate point-mass/LUT warm starts.

## Global Constraints

- Use `models/formulations/placeholder_planar_7dof.xml` and `models/tracks/2024_michigan_endurance_closed.xml` for both fidelity levels.
- Derive tire force peaks and load response from `models/tires/hoosier-43075-r20-simple.xml` through `evaluate_unitire`; do not use the legacy 2025 Matlab lap-sim tire parameters.
- Keep point-mass simplification assumptions explicit; it does not model planar yaw/steering or the full wheel-level dynamics.
- Benchmark wall time and iteration counts only with recorded options, cold process/cache conditions, and successful feasibility gates.
- Use the planar 1 m mesh and rate-limited configuration as the primary comparison; preserve a 2 m quick-run option for debugging.

---

## File Structure

- Create `src/simplified_vehicle_models/tires/unitire_envelope.py`: pure-slip peak-force sampling over vertical load and lookup interpolation data.
- Modify `src/simplified_vehicle_models/formulations/point_mass.py`: allow point-mass tire capacities to consume the UniTire-derived peak lookup while preserving legacy `TireEnvelopeParameters` behavior.
- Modify `src/simplified_vehicle_models/parameters/tire.py` and `parameters/vehicle.py`: define/type the load-indexed envelope data explicitly.
- Create `tools/compare_planar_point_mass.py`: same-source vehicle/track setup, runtime measurement, trajectory bridge, planar warm-start runs, and JSON/Markdown report output.
- Create `tests/test_unitire_envelope.py`: peak monotonicity, fit/interpolation, endpoints, and validation tests.
- Modify `tests/test_point_mass.py`: prove point-mass force capacities use the Unitire-derived lookup and preserve legacy-envelope behavior.
- Update `docs/validation/planar-ocp.md` or create `docs/validation/point-mass-planar.md`: method, derived tire curves, lap/compute-time comparison, warm-start results, and limitations.

## Task 1: Build and test a UniTire peak-force envelope

**Interfaces:** `build_unitire_peak_envelope(parameters, loads, reference_speed=15.0) -> UnitirePeakEnvelope`; envelope exposes vertical-load grid, longitudinal peak force, lateral peak force, and effective radius at reference load. Use slip ratio and slip angle sweeps with the existing `evaluate_unitire` evaluator and maximize positive force in each pure-slip direction.

- [x] Add tests that a positive increasing load grid yields finite, nonnegative force capacities, zero-load capacity is zero, lateral grip at rated load agrees with the fitted UniTire peak, and interpolation clamps/validates out-of-range inputs.
- [x] Run the new tests; confirm the adapter/type and lookup dispatch work.
- [x] Implement the sampler with a documented 15 m/s reference speed, 0–0.6 rad slip-angle and 0–1.5 slip-ratio ranges, local parabolic peak refinement, and a 71-node load lookup.
- [x] Re-run focused tests and compare fitted peaks at 250–2500 N against direct `evaluate_unitire` sweeps.

## Task 2: Use the lookup in point-mass force capacity

**Interfaces:** `PointMassVehicleParameters.tire` accepts either the existing linear envelope or `UnitirePeakEnvelope`. `tire_force_capacities(vehicle, loads)` dispatches to the appropriate model while the legacy behavior remains unchanged.

- [x] Add a focused test with a synthetic load lookup and known capacities.
- [x] Implement lookup-based force capacity and preserve the linear-envelope branch unchanged.
- [x] Run point-mass tire/capacity tests and `tests/test_point_mass.py`.

## Task 3: Run same-vehicle, same-track benchmark

**Files:** Create `tools/compare_planar_point_mass.py`; add focused tests for the vehicle bridge and tire source metadata.

- [x] Compile the placeholder formulation and reuse its mass, geometry, aero, drivetrain, and UniTire parameters to construct the point-mass object; no Murphy or legacy Matlab model XML enters the run.
- [x] Load the same closed XY track. Use sampled curvature magnitudes for point mass and signed path coordinates for planar OCP; record the 1,016.701 m lap and 1,018-node comparison mesh.
- [x] Run point mass in flying-lap mode with energy coupling and the rate-limited 1 m planar OCP. Record lap time, timing, iterations, status, tire source, and feasibility diagnostics; report lap times before compute time.
- [x] Inspect speed minima/maxima, final SOC, tire peak/load curves, and mesh/corridor settings; note the 10.9-vs-11.589 ratio fields and align point mass with the ratio used in planar equations.
- [x] Repeat the same-mesh timing once in an isolated process; report it as a single sample, not a median or stable performance guarantee.

## Task 4: Test point-mass-profile planar warm starts

**Interfaces:** `point_mass_planar_primal_seed` returns `state (7,N)` and `control (3,N)` arrays plus distance-resampled speed/time and pedal/kinematic-steering guesses. The benchmark combines the speed/time profile with the planar equilibrium seed's sideslip/steering, uses speed-slope pedal guesses, retains only planar algebraics, then invokes `_project_start`.

- [x] Add tests for dimensions, distance-resampling, initial charge, nonnegative mapped speed, and monotone elapsed time.
- [x] Implement the point-mass-to-planar trajectory adapter without copying point-mass normal loads, because load and wheel equations differ by formulation.
- [x] Compare against the verified equilibrium-seed run on the identical 1 m mesh with exact Hessian, rate limit, path bounds, and iteration cap; record iterations, wall time, diagnostics, and validity.
- [x] Report the nonconverged 500-iteration warm-start trial honestly; no iteration savings or valid warm-start lap time were obtained. The point-mass drive LUT is consumed within its solver, not itself a spatial OCP state guess.

## Task 5: Validate, document, and report

- [x] Run focused tests, then complete CasADi-enabled suite (514 tests passed).
- [x] Save JSON benchmark output with models, command/options, tire peaks, solver diagnostics, and runtime sample.
- [x] Document the UniTire peak-extraction method and approximation limits, lap-time results first, compute times second, then warm-start results.
- [x] Verify `git diff --check`, review generated report and tests, and commit the completed comparison.
