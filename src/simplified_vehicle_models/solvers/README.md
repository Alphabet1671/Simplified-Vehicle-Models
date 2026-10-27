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
| `forward_driver.py` | Drive a complete vehicle state forward along a prescribed path. |
| `forward_powertrain.py` | Shared forward-driver powertrain closure. |
| `input_preflight.py` | Bounded filesystem admission checks for worker-owned model inputs. |
| `joint_event_contract.py` | Analytic helper for constant-acceleration motion from rest. |
| `line_refinement.py` | Mesh refinement and warm-seed projection for the energy-HS line solver. |
| `line_validation.py` | Independent, fail-closed dense validation for minimum-time line results. |
| `live_standing_profile.py` | Bounded same-original-physics standing profile on a frozen open path. |
| `load_interval.py` | Interval enclosure of the planar load map and its Jacobian. |
| `loaded_geometry_admission.py` | Pure admission evidence for the exact stored fitted geometry. |
| `loaded_track_problem.py` | Loaded-data binding for the versioned track problem contract. |
| `longitudinal_actuator.py` | Invert the existing fixed-split forward powertrain for total wheel torque. |
| `maneuver_planar.py` | Cartesian open-loop maneuver adapter for the transient planar vehicle. |
| `maneuver_simulation.py` | Sampled-control maneuver simulation using segmented stiff integration. |
| `maneuver_vertical.py` | Cartesian open-loop maneuver adapter for the vertical 14-DoF plant. |
| `minimum_time_run.py` | Spawned, deadline-owning supervisor for minimum-time harness attempts. |
| `offset_geometry_bounds.py` | Exact-rational enclosures for the stored analytic offset path geometry. |
| `physical_longitudinal_mapper.py` | Opt-in quasi-steady force-to-actuator command mapping. |
| `planar_dense_certificate.py` | Certify the planar plant's explicit state domain over BDF dense intervals. |
| `planar_forward.py` | Adapt the transient planar vehicle to the shared forward driver. |
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
| `trajectory_forward_replay.py` | Diagnostic exact-reference trials for serialized trajectories. |
| `vertical_between_step.py` | Between-step evaluation-domain certificate for the vertical 14-DoF plant. |
| `vertical_dense_certificate.py` | Conservative state-domain certificate for vertical 14-DoF dense output. |
| `vertical_equilibrium.py` | Static and steady-corner vertical equilibrium for the 14-DoF vehicle model. |
| `vertical_forward.py` | Adapt the vertical 14-DoF vehicle to the shared forward driver. |
