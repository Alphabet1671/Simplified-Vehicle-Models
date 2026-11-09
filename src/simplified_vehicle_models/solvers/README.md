# solvers

Lap, line, forward-driver and maneuver solvers, with their certificates and evidence helpers.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Package marker. |
| `adaptive_line.py` | Bounded, fail-closed adaptive mesh wrapper for the spline line OCP. |
| `artifact_chart_audit.py` | Bounded whole-artifact aggregation for the cached-polynomial chart check. |
| `artifact_chart_check.py` | Bind a loaded energy-Hermite artifact to the explicit 3h chart checker. |
| `bounded_attempts.py` | Pure parent-deadline controller for bounded synchronous attempts. |
| `braking_envelope.py` | Offline sampled quasi-static corner/braking envelope, pure backward-cap propagation, and lowering existing caps without adding braking authority. |
| `brake_authority.py` | Bounded quasi-steady hydraulic-limited longitudinal deceleration prediction with explicit context/domain, contact capacity and shortfall; includes a pure current-state linear-hydraulic inverse; not a transient tire solver. |
| `capability_exchange.py` | Bounded empirical physical-residual localization and fixed-mesh exchange. |
| `capability_profile.py` | Independent fixed-path capability-profile reference (no torque vectoring). |
| `chart_element_certificate.py` | Bound explicit rational energy/control polynomials against a stored signed chart. |
| `closed_loop_line.py` | Correct a spline minimum-time line against a stateful controller replay. |
| `consumer_approval_record.py` | Fail-closed, JSON-only consumer approval-record traceability verifier. |
| `convergence_policy.py` | Proposed, versioned numerical-reference accuracy policy and comparator. |
| `curvature_guard_proposals.py` | Bounded computational curvature-guard proposals (not an extrema certificate). |
| `dense_domain.py` | Conservative component ranges of SciPy BDF dense-output polynomials. |
| `energy_polynomial_bounds.py` | Exact bounds for explicit rational energy and control polynomials. |
| `energy_seed_projection.py` | Bounded mechanical projection for fixed-energy Hermite--Simpson seeds. |
| `fixed_path_energy_nlp.py` | Independent fixed-path energy NLP reference using a signed capability chart. |
| `forward_backward.py` | Python module `forward_backward`. |
| `forward_driver.py` | Historical preview speed/steering driver with optional signed-pedal dynamics, newer opt-in policies, and complete-state forward integration. |
| `envelope_driver.py` | Pure actual-speed longitudinal policy; legacy behavior plus opt-in stationary-reference physical-reachable planning with immutable forecast tables, separately declared forecast horizon and foot-transition timing, and a separate current-state physical inverse path. |
| `driver_maps.py` | Immutable physical pedal calibration and strict versioned JSON configuration.
| `forward_powertrain.py` | Shared forward-driver powertrain closure. |
| `input_preflight.py` | Bounded filesystem admission checks for worker-owned model inputs. |
| `joint_event_contract.py` | Analytic helper for constant-acceleration motion from rest. |
| `line_refinement.py` | Mesh refinement and warm-seed projection for the energy-HS line solver. |
| `line_validation.py` | Independent, fail-closed dense validation for minimum-time line results. |
| `live_standing_profile.py` | Bounded same-original-physics standing profile on a frozen open path. |
| `longitudinal_capability.py` | Immutable sampled current-vehicle longitudinal capabilities with physical signed pedal-response maps, canonical regular-file source identities, allocator identity guard, and pure Frenet progress/body acceleration conversion. |
| `hybrid_speed_plan.py` | Immutable converged forward-drive/backward-brake sampled speed trajectory with explicit SOC, signed endpoint/midpoint capability queries, braking event/onset metadata and deterministic versioned identity. |
| `hybrid_driver.py` | Opt-in stateless signed longitudinal policy for HybridSpeedPlan, with explicit progress/body acceleration telemetry, complete physical schedule forecasts, and an eight-extra-forecast verified mixed-schedule relay search with bounded work diagnostics. |
| `hybrid_setup.py` | Shared bounded model-derived provider, immutable physical/domain/final corner caps, deterministic setup identity, HybridSpeedPlan and policy setup for runner/GUI opt-in integration; actual builder-query audit, distinct provider/execution-allocator identity, and invalid capability interpolation fail closed. |
| `load_interval.py` | Interval enclosure of the planar load map and its Jacobian. |
| `loaded_geometry_admission.py` | Pure admission evidence for the exact stored fitted geometry. |
| `loaded_track_problem.py` | Loaded-data binding for the versioned track problem contract. |
| `longitudinal_actuator.py` | Invert the existing fixed-split forward powertrain for total wheel torque. |
| `maneuver_planar.py` | Cartesian open-loop maneuver adapter for the transient planar vehicle. |
| `maneuver_simulation.py` | Sampled-control maneuver simulation using segmented stiff integration. |
| `maneuver_vertical.py` | Cartesian open-loop maneuver adapter for the vertical 14-DoF plant. |
| `minimum_time_run.py` | Spawned, deadline-owning supervisor for minimum-time harness attempts. |
| `offset_geometry_bounds.py` | Exact-rational enclosures for the stored analytic offset path geometry. |
| `planar_dense_certificate.py` | Certify the planar plant's explicit state domain over BDF dense intervals. |
| `planar_forward.py` | Adapt the transient planar vehicle to the shared forward driver, including immutable current-state brake authority contexts for opt-in operational forecasting. |
| `profile_line_seed.py` | Optional exact-centreline fixed-path energy-profile seed for spline OCPs. |
| `qss_line_ocp.py` | Minimum-time quasi-steady racing line in the distance domain. |
| `qss_spline_ocp.py` | Energy-consistent cubic-spline minimum-time line OCP in distance. |
| `reference_geometry_bounds.py` | Exact-rational interval bounds for reference metric and curvature. |
| `reference_spline_bounds.py` | Exact-rational interval bounds for stored reference B-spline derivatives. |
| `refinement_comparison.py` | Pure evidence ledger for comparing bounded refinement attempts. |
| `run_manifest.py` | Canonical, fail-closed run manifest sidecar primitives. |
| `signed_chart_bounds.py` | Outward-rounded bounds for a signed chart's bilinear table rectangle. |
| `spline_seed_diagnostics.py` | Exact-coordinate diagnostics for a built spline NLP seed (never acceptance). |
| `standing_event.py` | Frozen-path standing launch plus a rolling fixed-path remainder profile. |
| `standing_launch.py` | Original-model time-domain launch on a frozen analytic trajectory. |
| `standing_straight_ocp.py` | Actual fixed-straight, nonnegative-acceleration standing-start OCP. |
| `track_problem_contract.py` | Pure structural preflight for the versioned track-problem request contract. |
| `trajectory_artifact.py` | Strict reader and exact reference bridge for analytic trajectory artifacts. |
| `yaw_transition.py` | Sampled pure-lateral axle force/moment screening for curvature-transition yaw demand; operational limits, not a feasibility guarantee. |
| `steering_capability.py` | Kinematic steering-rate and first-order actuator speed screening; not a closed-loop tracking guarantee. |
| `vertical_capability.py` | Bounded offline sampled vertical-model tested-speed restrictions (not physical maxima or continuous certificates). |
| `vertical_dense_certificate.py` | Conservative state-domain certificate for vertical 14-DoF dense output. |
| `vertical_equilibrium.py` | Static and steady-corner vertical equilibrium for the 14-DoF vehicle model. |
| `vertical_forward.py` | Adapt the vertical 14-DoF vehicle to the shared forward driver; derive immutable brake authority contexts from explicit current states. |
