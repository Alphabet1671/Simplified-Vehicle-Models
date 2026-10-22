# Minimum-lap-time readiness audit — 2026-10-22

## Scope and verdict

Audited branch `feature/minimum-time-line-2d-track`, HEAD `0022fbcd52e5d58761858733a729a9fc1caeac91`. The tree was clean at audit start. This audit changes no solver, test, or source track data; this report is the only new deliverable. Two read-only audits covered geometry/numerics and controller/acceptance semantics.

**Verdict:** a useful research/prototyping foundation, not yet a generally reliable minimum-lap-time simulator. Completing the implementation plan did not establish real-track convergence, validated TV behavior, or physical predictive accuracy. Keep the legacy default; do not promote the spline/controller modes on test counts alone.

## Fresh evidence

The current seven-module solver suite passed **132 tests in 73.989 s**. Modules: point_mass_capability, capability_backend, qss_spline_ocp, closed_loop_line, tv_runner, minimum_time_line_gate, replay_adapter. Printed gate tables belong to tests (including mocked runs), not new event simulations. The prior full GUI-inclusive suite did not finish; no full-suite pass is claimed.

A real-model smoke check compiled `iem24_murphy_real_planar_7dof`, built its point-mass view, verified the capability cache's vehicle hash, and evaluated acceleration limits at 10 m/s and 0.01 1/m successfully. The current control-policy registry is empty; the only replay adapter is `point-mass`.

### Fresh bounded spline smoke matrix

Same production IEM24 capability cache, 5 m mesh, 35 m spline control spacing, `maximum_iterations=200`, default strict tolerance, 0.1 m corridor margin. Source MIS widths retained; FSC half-width 3 m, skidpad 1.5 m, acceleration 2.45 m assumed explicitly. These are individual NLP solves, not pipeline/mesh/replay acceptance.

| Track | Current spline result | Evidence |
|---|---|---|
| MIS | Maximum_Iterations_Exceeded | 51.56 s wall time; no accepted solution |
| FSC | Maximum_Iterations_Exceeded | 30.26 s wall time; no accepted solution |
| Skidpad | Maximum_Iterations_Exceeded | 3.66 s wall time; no accepted solution |
| Acceleration | Solve_Succeeded, 10 iterations | OCP time 6.562297120 s; max violation 1.225e-8; 0.12 s wall time |

The acceleration time excludes any external launch correction. A 200-iteration failure is neither an infeasibility proof nor evidence that no initialization/settings could succeed. These runs do show that the current spline default settings are not yet robust across the four fixtures. Earlier legacy acceleration/skidpad conditional five-gate results remain historical evidence, not validation of the new spline or TV modes.

## Priority 0 — correctness and truthful acceptance

### 1. Preserve the optimized trajectory through replay/export

The spline OCP derives offset, heading and curvature from spline coefficients. But [path_from_solution](../../src/simplified_vehicle_models/tracks/racing_line.py#L93-L155) ignores those coefficients: the closed branch linearly samples node offsets and rebuilds finite-difference geometry; the open branch fits new XY cubic splines through displaced nodes. Optional smoothing changes the line again.

**Remaining work:** return an evaluable trajectory with its reference geometry and coefficient/basis metadata; export/replay that trajectory directly. Independently compare OCP and exported position, tangent, curvature, length and time. Any smoothing must be a new candidate requiring revalidation, not a way to make replay agree after the fact.

### 2. Map replay path arclength back to reference distance

[The cap update](../../src/simplified_vehicle_models/solvers/closed_loop_line.py#L110-L163) directly interpolates `replay.distance` onto the OCP mesh. Replay distance is rebuilt-path arclength; the OCP mesh is reference-track distance. Offsets change path length, so the two axes are not interchangeable. Endpoint clamping does not correct the mismatch.

**Remaining work:** carry `s_reference -> ell_path` and its inverse through reconstruction and replay. Test constant inward/outward offsets (known scale factor), nonconstant offsets and the closed seam. Also invalidate path-dependent corrections when the line changes beyond the region in which the replay was evaluated.

### 3. Separate candidate feasibility from validated acceptance

[Runner acceptance](../../tools/run/solve_minimum_time_line.py#L773-L895) permits `None` gates. A closed-loop run may be accepted with zero-control, no-slower and mesh checks unavailable; allocation has no controller replay agreement check. This is documented, but `accepted=True`/exit zero is not a five-gate certification.

**Remaining work:** separate numerical solution, replay feasibility, numerical validation and physical validation statuses. Missing mandatory checks must prevent the corresponding validation tier. Run a post-correction mesh ladder and same-condition baseline. A no-controller point-mass replay must remain explicitly distinct from a controller-in-loop result.

### 4. Make reference geometry and derivatives consistent

[The spline geometry helper](../../src/simplified_vehicle_models/solvers/qss_spline_ocp.py#L50-L92) linearly interpolates reference curvature but interpolates a separately calculated `np.gradient` derivative. This derivative is not generally the derivative of the interpolant; its closed endpoints are one-sided. Circle tests cannot expose this.

**Remaining work:** use a common differentiable reference geometry/interpolant with consistent tangent, curvature and curvature derivative and periodic seam handling; validate against independently differentiated XY geometry and on nonconstant-curvature tracks. Preserve original data; record any derived smoothing/fit and its error.

## Priority 1 — numerical trust in a minimum-time result

### Constraint resolution and geometric containment

Constraints are sampled at selected nodes/collocation points; the runner's containment gate checks only solution-node offsets. The dense regression uses a synthetic fixture, not an independent validator on every output. A spline can violate width, fold, heading, curvature or capability between samples.

Add an independent dense/adaptive validator for the final exported trajectory and state/control interpolation. Refine near constraint extrema/violations. Explicitly distinguish reference-point width from physical road edges and vehicle swept-footprint clearance; do not subtract vehicle half-width twice.

### Mesh, basis and initialization convergence

Refine both the speed/collocation mesh and the spline control spacing independently. Compare not just elapsed time but path, speed, curvature, active constraints and dense violations. A nearby mesh pair selected after failures is not convincing asymptotic convergence. Test several feasible seeds and equivalent seam placements. Label the answer a best-found/local minimum unless stronger optimality evidence exists; IPOPT success is not a global-minimum certificate.

Retain production-capability paired closed/cut-open MIS/FSC tests. The earlier controls showed that both topologies can solve with a simple nonphysical capability surface; they did not isolate nonsmooth interpolation as the root cause. Diagnose active capability cells, infeasible samples and scaling before replacing the surface. Any smoother surrogate must be independently checked against original physical limits; unconstrained spline interpolation can overshoot.

### Baselines and replay

Compare a free line against a centerline under the same model, controller, start/end state and numerical accuracy. Resolve launch consistently: the OCP uses a positive speed floor, whereas standing-start replay includes rest. Recheck legacy results after shared-transcription changes such as midpoint-speed bounds. Archive reproducible commands and artifacts for acceleration, skidpad, MIS, FSC and both long-track cut-open views.

## Priority 2 — meaningful TV/controller conclusions

[Allocation sampling](../../src/simplified_vehicle_models/formulations/controller_capability.py#L54-L160) uses a policy's Boolean feasibility answer to restrict a pre-existing envelope. It does not independently establish tire force balance, yaw equilibrium, wheel torques, slip, power or controller stability. It also cannot expand beyond its input envelope, so that envelope must be demonstrated to cover the intended vehicle/control authority; calling the present baseline “ideal” does not prove it is an upper bound on all TV strategies.

Required distinctions:

1. Fixed-split/TV-disabled baseline with its actual allocation policy.
2. A specified TV algorithm, fixed calibration, included in planning and replay.
3. An ideal actuator-allocation optimum as a separate benchmark, only if physically modelled with the same limits.

A real memoryless policy needs a vehicle/controller equilibrium solve, not just a fabricated acceleration cap. Stateful TV requires yaw/sideslip/wheel/controller states, sample timing, actuator limits, estimator/reset behavior and path-dependent behavior in replay or augmented optimal control. A static `(v, kappa)` surface cannot establish transient feasibility.

The current replay-cap loop is a heuristic feasibility correction: it does not prove a controller-aware minimum-time optimum. Slowing down is not guaranteed to fix every TV stability/tracking problem, and previous caps can bias later lines. A rigorous controller-aware optimization should include plant/controller dynamics or a validated simulation-based trajectory optimization with appropriate sensitivities/trust regions.

For algorithm comparisons, run a cross-replay matrix: optimize each line with its own policy, replay both lines under both policies with identical starts and vehicle parameters. This separates controller improvement, line adaptation and model optimism. Do not merely replay a TV-unaware line and call it TV-optimal.

## Priority 3 — physical predictive accuracy and reproducibility

Calibrate the vehicle/tire/powertrain model for the event and operating conditions; verify load/combined-slip behavior, torque/regen limits, steering/actuator rates, battery initial state and power. Use surveyed corridor data or explicitly conditional width assumptions. Define flying-lap periodicity for fast vehicle/controller states separately from nonperiodic energy states; specify open start/finish requirements.

Validate against independent runs/data, with uncertainty in tire grip, width, surface condition, mass and controller calibration. Report a sensitivity range rather than relying on optimizer tolerance as a physical error bar.

Cache hashes now carry controller mode/provenance for new surfaces, which is useful. Still add replay implementation/version, model dependencies, controller sample timing/initialization and run settings to reproducibility records. Legacy caches without controller metadata need an explicit trust/migration policy. Complete the full test suite in a controlled environment, separating long numerical and GUI tests, with reproducible timeouts and no inferred pass from an incomplete run.

## Recommended sequence and completion gates

1. **Geometry-to-replay consistency and station mapping.** Independent nonconstant-offset analytic/XY checks pass; exported path equals optimized path within declared geometry tolerances.
2. **Truthful statuses and independent dense feasibility.** Every result states which gates were executed; no validation tier passes with missing required evidence.
3. **Robust no-TV quasi-steady baseline.** Four-track plus cut-open matrix converges with repeatable settings; mesh and control-spacing refinement plateau; multiple seeds and replay agree. Existing 0.5% mesh and 1% replay thresholds are starting numerical gates, not proof of physical accuracy.
4. **Real TV integration.** Validated controller/plant adapter, explicit state/reset/timing semantics, same-condition baselines and cross-replay experiments. Until then, TV performance remains not measured.
5. **Predictive validation.** Match independent measurements and quantify uncertainty; use event-matched capability and corridor data before event-valid claims.

The immediate next work should be steps 1–3, not more mode switches or tuning until selected gates turn green. These correctness and validation improvements benefit both open and closed tracks and avoid optimizing against a representation/replay mismatch.
