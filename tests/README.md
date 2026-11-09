# tests

The unit and regression suite, run with `unittest`.

## Files

| File | Contents |
|---|---|
| `test_catalunya_ttc_formulation.py` | Exact TTC artifact preservation, steady/transient parameter partition, and planar 7-DoF assembly selection. |
| `test_track_gui_lap_closure.py` | Synthetic stadium coverage for bounded periodic seam smoothing and skip/refusal conditions. |
| `test_damper_table.py` | Zero-origin normalization, Akima damper interpolation, click blending, tails, validation, and NPZ loading. |
| `test_suspension_physical.py` | Physical shaft strut force, disabled-damper behavior, array shapes, and parameter validation. |
| `test_forward_acceptance_chain.py` | Cold-then-warm runner command assembly, seed selection, and removed refinement-stage rejection. |
| `test_yaw_transition.py` | Sampled axle force/moment yaw-transition speed screen, numerical assumptions and closed seams. |
| `test_steering_actuator_inverse.py` | Optional first-order steering-actuator inversion and physical angle/slew enforcement. |
| `test_transient_corner_runner.py` | Opt-in transient corner planning integration, artifacts and immutable warm-start identity. |
| `test_steering_capability.py` | Kinematic finite-rate and first-order steering-transition speed screening. |
| `test_vertical_capability_runner.py` | Opt-in model-aware runner restrictions, query budgets, source closure and strict warm-start identity. |
| `test_vertical_capability.py` | Bounded signed-curvature vertical steady-corner samples and cap evidence. |
| `test_vertical_solve_fixes.py` | 14DoF solve-chain regressions: rolling-resistance sign, retryable trial rejections, kinematic-slip brake governor, smooth anti geometry and roll-centre query. |
| `test_suspension_unlimited.py` | Explicit spline extrapolation policies, tangent spring tails, motion-ratio holds, and finite-query behavior. |
| `legacy_suspension.py` | Explicit fixture builder for tests whose contract is the historical independent-corner suspension law. |
| `test_vertical_setup_sweep_physical.py` | Active physical shaft sweep consumer names, overrides, and unsupported legacy-metric guard. |
| `test_suspension_unlimited_task2.py` | Unlimited-travel dense-certificate policy, malformed metadata, and nonfinite enclosure regression tests. |
| `test_table_extension_policy.py` | Suspension-table extensions are descriptive under the unlimited policy; invalid diagnostics and network states still reject. |
| `test_unlimited_travel_gate.py` | Unlimited-travel gate finite, ordered jounce diagnostic requirements and historical fallback. |
| `test_suspension_parallel.py` | Additive corner/heave/roll force mapping, static support, inertia, and model integration. |
| `test_forward_validation_cost.py` | Accepted-state and dense-midpoint validation coverage and timing-accounting regressions for the shared forward solver. |
| `test_forward_driver.py` | Continuous forward-driver dynamics, directional pedal slew, and speed planning. |
| `test_historical_preview_restoration.py` | Historical command, steering, smoothing, directional pedals, defaults and diagnostics. |
| `test_forward_stop_event.py` | Optional operational body-speed terminal event, event ordering, full-state root rows, and observer parity. |
| `test_brake_inverse.py` | Actual forward-powertrain hydraulic effort inversion, force-grid roundtrips, feasibility, regen rejection, and monotonicity certification. |
| `test_brake_authority.py` | Hydraulic-limited quasi-steady longitudinal deceleration prediction, point-mass oracle separation, context/domain, tire capacity and shortfall. |
| `test_hybrid_actuator_map.py` | Shared immutable physical signed-pedal map saturation, interpolation, inverse/forward roundtrip, and fail-closed missing-map checks. |
| `test_hybrid_capability.py` | Hybrid capability/allocator contracts, synthetic TV reduction, asymmetric-radius effective-mass oracle, and Frenet quantity-conversion identities. |
| `test_hybrid_driver.py` | Hybrid signed policy, complete schedule forecasts, verified continuation relay, bounded probe-work telemetry, unknown/inconsistent mixed-probe rejection, retained safe endpoint at budget exhaustion, and physical Frenet conversion regressions. |
| `test_hybrid_setup.py` | Real compiled-model hybrid setup, pointwise physical and valid-cell caps, single-builder/no-legacy-planner spy, fail-closed audit, immutable cap buffers, setup identity sensitivity, source-byte binding, and SOC checks. |
| `test_hybrid_maneuvers.py` | Portable time-weighted hybrid maneuver acceptance metrics and empty/outlier/contiguous-window tests. |
| `test_hybrid_transitions.py` | Transition runner case selection, finite config/trace gates, required rollout/source identity, terminal-stop fixture cap/reference, real prospective event oracle/query evidence and convergence, arrival gates, accepted-sample seam bracket command/pedal continuity and missing/discontinuity rejection; explicit physical oracle query scope excludes internal policy queries, observer availability, allocator rejection, CLI failure artifact preservation and bounded CLI contract. |
| `test_hybrid_speed_plan.py` | Hybrid drive/brake trajectory bounds, explicit initial speed, closed seams and immutable plan samples. |
| `test_brake_stop_validation.py` | Portable runner metrics parsing, positive-speed stop-event contract, and positive finite stop-distance error gates. |
| `test_state_aware_brake_context.py` | Pure current-state physical brake inverse parity, state sensitivity/order independence, steering/yaw projection, capacity-prefix replay, explicit nonlinear-map refusal, and physical-policy live-context selection. |
| `test_braking_envelope.py` | Analytic sampled braking-cap propagation, closed seams, sampling and oracle-query boundaries. |
| `test_envelope_driver.py` | Pure actual-speed/pedal-aware policy arithmetic and forward-solver diagnostics hook. |
| `test_brake_transition_planning.py` | Opt-in physical-reachable longitudinal policy, immutable forecast table, bounded candidates and stationary pedal-invariant reference. |
| `test_braking_maneuvers.py` | Full-plant transition and original braking-event evidence regressions (fixture-based; not a new maneuver run). |
| `test_forward_driver_runner.py` | Runner CLI, warm-start, provenance, and synthetic integration regression coverage, including physical forecast exact-identity rejection. |
| `test_virtual_driver_comparison.py` | Controlled-comparison shared identity, historical forward/backward planner and longitudinal law/smoothing, signed Frenet/current-inverse adapter path, mocked solve seam/full-state setup, geometry-first event, and physical-unit accepted-trace KPI contracts. |
| `test_hybrid_runner_integration.py` | Hybrid runner opt-in/default and rolling-start CLI contracts, source-closure opt-in, actual temporary-source mutation rejection before compile, and initial-speed warm-start identity. |
| `test_hybrid_final_contracts.py` | Consolidated hybrid fix-wave contracts for canonical source paths, progress/body coordinate assumptions, derived version/config metadata, and canonical steering-plan reuse. |
| `test_default_track.py` | Canonical default-track coverage for the runner, comparison tools, and plots. |
| `test_driver_maps.py` | Immutable pedal map interpolation, validation, shaping, strict JSON configuration, and stable digest.
| `test_pedal_actuator_calibration.py` | Runtime physical pedal calibration binding, forward evaluator parity, and bounds.
| `test_sim_gui_forward.py` | Current GUI forward-driver settings, strict opt-in controls, immutable identity, child dispatch, RunRecord-backed completion gate, and workflow-specific Qt visibility. |
| `test_sim_gui_case.py` | GUI case adapter dispatch, point-mass boundaries, and retired workflow rejection. |
| `test_sim_gui_run.py` | GUI child attempt freshness/symlink rejection and actual orchestrated tiny-child regression coverage. |
| `test_run_record_runner.py` | Runner record opt-in, policy telemetry versus optional step sink, and trace parity. |

Test modules named `test_<topic>.py` each cover one topic and run under `unittest`. `test_gui_launchers.py` exercises direct venv launch from unrelated working directories. Maneuver result tests verify canonical RunRecord handoff to animator readers; video encoding is not a maneuver responsibility.

## Subdirectories

| Directory | Purpose |
|---|---|
| `fixtures/` | Data files that tests read. |
