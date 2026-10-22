# QSS closed/cut-open investigation — 2026-10-21

## Question and conclusion

Can one free-line solver handle both open and closed tracks? Are the MIS/FSC failures caused by periodic endpoints or by long tracks?

**The paired evidence does not support splitting the solver by topology.** Cutting either full-length MIS or FSC open did not resolve convergence with the production capability surface. Both approximately 200 m open prefixes also failed at the shared 1,000-iteration cap without a speed cap. Conversely, the same NLP solved both full real geometries and a 1 km circle in both topologies with a deliberately simple capability control. These controls show that neither periodicity nor length alone is sufficient to explain the failures. They do not prove a unique root cause.

This corrects earlier developer-note/narrative statements attributing failures specifically to closed tracks, lateral-state freedom, or wrap kinks. Those were hypotheses, not isolated causes. Failure of a bounded nonlinear optimizer is not evidence of mathematical infeasibility.

## Reproducible implementation

- Diagnostic runner: [compare_qss_track_boundaries.py](../../tools/diagnostics/compare_qss_track_boundaries.py).
- Regression tests: [test_qss_track_boundaries.py](../../tests/test_qss_track_boundaries.py).
- The source XML/CSV files are never edited. Views are created in memory with `dataclasses.replace`.
- The same-length cut retains every cross-section, including the terminal position that coincides with the first. It represents an unwrapped traversal from a start to a finish at the same XY location; no cyclic constraints or wrap sampling are imposed. It is not exported as a newly measured open XML track.
- Cropped prefixes retain original samples through the last node at or before 200 m. Actual lengths are 199.539 m (MIS) and 198.270 m (FSC).

### Shared experimental controls

- Same IEM24 Murphy planar vehicle and matching capability cache.
- Same 5 m mesh construction, requested tolerance `1e-6`, exact Hessian, fixed characteristic variable scaling, gradient NLP scaling, adaptive IPOPT barrier.
- Same **per-node** offset layout. No reduced basis, so open/closed basis differences cannot explain the primary comparison.
- Same constant 10 m/s primal seed (SHA256 equality checked within full pairs), same zero offset/heading/control seed, fresh duals.
- Same objective: open-only offset/heading regularization explicitly set to zero; curvature-rate regularization remains `1e-6` in both cases.
- Closed endpoints: periodic offset/heading/speed. Open endpoints: zero heading error at start/finish, 10 m/s start speed, free terminal speed and lateral endpoints. These boundary conditions inevitably differ; this is boundary/topology sensitivity, not a pure topological invariant test.
- Rolling-start NLP diagnostics only. No standing-start launch or replay gate claims.
- MIS uses its existing assumed widths; FSC uses an explicitly assumed 3 m reference-point half-width. A 0.1 m margin is applied.
- All input hashes checked unchanged at completion. Existing output files are rejected.

Tests verify identical full-pair interior objectives, constraints, variable/constraint bounds, dimensions and seeds. Only the final three endpoint rows differ in value/expression.

## Production-capability results

All six cases hit `Maximum_Iterations_Exceeded` at both 200 and 1,000 iterations. Values below are **failed-iterate constraint violations**, not feasible lap times.

| Track/view | Length (m) | Mesh nodes | Status at 1,000 | Max constraint violation | Max HS defect |
|---|---:|---:|---|---:|---:|
| MIS closed full | 1016.701 | 205 | cap | 2.3844 | 2.478e-2 |
| MIS cut-open full | 1016.701 | 205 | cap | 2.1609 | 1.766e-3 |
| MIS open prefix | 199.539 | 41 | cap | 1.8028 | 2.323e-4 |
| FSC closed full | 768.296 | 155 | cap | 5.056e-1 | 2.765e-3 |
| FSC cut-open full | 768.296 | 155 | cap | 7.992e-1 | 2.422e-3 |
| FSC open prefix | 198.270 | 41 | cap | 8.311e-4 | 8.311e-4 |

Opening the full track is not a cure. Smaller cases have lower cost and sometimes lower residuals, but a short prefix also changes curvature content and endpoints; this does not establish length alone as causal. No returned objective from these failed runs is presented as performance.

### Low-speed production control

A second six-case matrix retains the full production capability constraints but adds a diagnostic node-speed cap of 10 m/s (matching the seed and rolling start). Five cases still hit the cap. The **FSC open prefix succeeds in 82 iterations**, maximum constraint violation `7.65e-9`. MIS cut-open full becomes much closer to primal feasibility (`7.38e-5`) but does not satisfy strict status/tolerance acceptance. Thus speed/active capability constraints matter, but low speed is not a universal fix.

## Nonphysical smooth-capability controls

For these controls only, replace the capability with constant maximum acceleration 3 m/s², minimum acceleration -8 m/s², speed ceiling 10 m/s and curvature domain [-1, 1] 1/m. This is **not a vehicle calibration** and the results must not be used as event times or acceptance.

| Geometry/view | Accepted NLP? | Iterations |
|---|---|---:|
| 1005.310 m synthetic circle, closed | yes | 10 |
| Same circle cut open | yes | 10 |
| Full MIS closed | yes | 16 |
| Full MIS cut open | yes | 16 |
| MIS open prefix | yes | 13 |
| Full FSC closed | yes | 23 |
| Full FSC cut open | yes | 21 |
| FSC open prefix | yes | 20 |

All passed the production status/finite/primal-feasibility validator at the requested tolerance. This shows that the long real geometry and periodic endpoints can be handled by the same transcription and optimizer. The constant control changes both smoothness **and physical limits/active operating region**; it does not isolate gradient discontinuity as the cause. Earlier within-cell derivative tests were good; active interpolation kinks remain a plausible but unproven contributor.

## Unified alternatives, ranked

### 1. Consistent smooth offset geometry, one boundary operator

Use one C1/C2 offset spline and evaluate `n`, `n'`, and `n''` from the same basis at every collocation point. Derive heading error from `atan2(n', 1-k0*n)` and vehicle curvature consistently, or collocate a derivative relation explicitly. Collocate only independent states (e.g. speed or kinetic energy). Apply the same corridor/capability rows for both topologies; periodic spline constraints versus clamped/free open endpoints are a boundary operator, not separate solvers.

**Why:** the current reduced basis constrains only node offset values while the HS midpoint/dense offset is reconstructed independently from node states and endpoint rates. It is a mixed representation, not a globally enforced spline trajectory. Rebuilding a different cubic path later compounds replay mismatch. A consistent trajectory would make OCP geometry and reconstruction the same object.

**Risks/tests:** reference-track curvature derivative quality; endpoint regularity; standing-start singularity; verify analytic derivatives against finite differences, recover steady circle/straight times, preserve corridor at all sampled points, test both real full/cut-open tracks under the same configuration.

### 2. Keep full-state HS; improve capability model/continuation in a controlled way

Retain exactly the current state/control equations for both topologies. Investigate a differentiable conservative capability approximation or cell-aware continuation, but validate its values and physical feasibility against the original sampled surface. Compare same limits with only interpolation treatment changed before claiming smoothness causality. Do not silently replace linear interpolation with an unconstrained B-spline: overshoot can relax acceleration limits.

**Why:** production vs constant-capability controls point to the capability/operating-region interaction, not closure alone. This alternative can be isolated with the new diagnostic runner.

### 3. Shared multiple shooting or different collocation coordinates

Use one distance-domain multiple-shooting or collocation core with a kinetic-energy state to reduce low-speed conditioning; use periodic or initial/terminal constraints as boundary data. Validate integrator sensitivities and capability interpolation, because merely changing optimizer (the previous SQP attempt) did not establish a solution.

**Recommendation:** keep one solver. First prototype alternative 1 behind an opt-in setting and use the paired diagnostic matrix as its acceptance guard; investigate capability regularity separately, not by topology-specific code forks. Do not relax existing tolerances or declare event acceptance from these diagnostics.

## Commands and raw evidence

```bash
PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m tools.diagnostics.compare_qss_track_boundaries \
  --track models/tracks/2024_michigan_endurance_closed_edged.xml \
  --track tracks_out/fsc_may_2026.xml \
  --formulation models/formulations/iem24_murphy_real_planar_7dof.xml \
  --surface runs/capability/iem24_murphy_7dof_mis.npz \
  --output runs/minimum_time_line/new_boundary_comparison.json \
  --spacing 5 --maximum-iterations 1000 --start-speed 10 --prefix-length 200
```

Use a **new output path** each run. Add `--synthetic-capability-control` for the explicitly nonphysical control, or `--speed-cap 10` for the production low-speed diagnostic. Inputs are not rewritten. CLI exit zero means the matrix ran, not that all NLPs converged; inspect `accepted` per case.

Raw local evidence (ignored run artifacts):
- [200-iteration matrix](../../runs/minimum_time_line/boundary_comparison_20261021.json)
- [1,000-iteration matrix](../../runs/minimum_time_line/boundary_comparison_1000_20261021.json)
- [Production capability with 10 m/s cap](../../runs/minimum_time_line/boundary_production_speed10_20261021.json)
- [Constant-capability matrix](../../runs/minimum_time_line/boundary_constant_capability_20261021.json)
- [Long-circle and constant-capability controls](../../runs/minimum_time_line/boundary_synthetic_controls_20261021.json)

These files contain statuses, residuals, dimensions, seed hashes and provenance. Failed objective values are diagnostics only. The matrix does not perform mesh/replay/event acceptance.
