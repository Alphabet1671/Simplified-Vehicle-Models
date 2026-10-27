# Yaw lap-suite study protocol

## Purpose and frozen interpretation

This protocol defines the comparison identity and evidence required for yaw-aware planning/replay studies. It does not change legacy OCPs, rebuild planning/replay/convergence systems, relax map semantics, or certify continuous feasibility. Fresh runs must record the executed settings; historical records with absent source history remain explicitly `unknown`.

## Paper-to-code decisions

| Source section | Decision | Required check/evidence |
|---|---|---|
| Massaro & Limebeer, *Minimum-lap-time optimisation and simulation*, §§4.4–4.7, 7–9; Eq. (42), (48)–(53) | Separate QSS/envelope feasibility from transient equations-of-motion feasibility. Audit reference/physical station conversion, actuator rates, and yawed footprint. Refine the actual model rather than borrowing another model's mesh. | Keep strict map validation; record station mapping, actuator-rate checks, yawed footprint status and discretization convergence separately from envelope feasibility. |
| Thommyppillai, Evangelou & Sharp, *Advances in the development of a virtual car driver*, §§1, 3–5; Eqs. (10)–(15) | Current controller is nonlinear feedforward plus feedback, not the paper's optimal LQ preview controller. Measure preview horizon, actuator bandwidth, and coupled longitudinal/lateral tracking. Adaptation is deferred; do not claim gain scheduling or near-limit robustness. | Label controller accurately and record resolved gains/horizon, actuator response, lateral and speed tracking; no adaptation claim. |
| Rijns et al., *Optimising vehicle performance with advanced active aerodynamic systems*, §§5.1–5.3, Fig. 6 and Table 5 | Report wall time alongside discretization error and retain actuator-rate limits. Its 10 m / approximately 0.01% result is a transient collocation result on Barcelona, not this graph solver. Do not import IPOPT tolerances, steering limits, or mesh size. | Report elapsed wall time and discretization error together; preserve configured rate limits; label that published result as non-transferable. |
| Pinheiro et al., *Path planning development for human-like virtual driver*, vision/path construction, five-factor sensitivity analysis, experimental comparison | Keep human-path fidelity separate from minimum-time performance. Do not fit five path factors to force the 71.685 s measured median, and do not treat this as speed-planning or containment proof. | Report path fidelity as its own dimension; no five-factor fit or speed/containment certification inferred from this paper. |

## Run manifest

Every fresh run contract includes the complete CLI arguments, resolved configuration, transitive vehicle/configuration byte hashes, track/data hashes, map file SHA-256 and source identity, plan file and array digests, source revision and dirty-source hashes, resolved controller values, full initial state/SOC, solver method/tolerances/step size, profile and line settings, package versions, thread settings, and watchdog budget. Missing historical evidence is encoded as `unknown`, never represented as a current-source attestation. Canonical JSON is sorted, compact UTF-8 and serialized with `allow_nan=False`; the SHA-256 is over the canonical contract before adding the hash field. The post-plan `inputs.json` contract binds the plan file SHA-256 and array digest; the immutable plan artifact records only the contract version and canonical contract hash, avoiding a circular plan-file self-hash.

## Compatibility and comparison identity

Compatibility is an explicit allowlist: geometry studies vary geometry spacing only; basis/control studies vary control spacing only; replay studies vary only the declared line/controller treatment. A paired geometry refinement with changed control spacing (or a control refinement with changed geometry spacing) is rejected. Output paths, timestamps and runtime observations are excluded. Numerical tolerances, vehicle/map/track identities, boundary type and maximum speed remain identity-bearing. Warm-start lineage is recorded separately, and compared runs must use matched warm-start policies. The comparison helper validates the dimension and emits a deterministic SHA-256 identity.

Strict completed-map semantics remain mandatory. This contract does not certify continuous feasibility, footprint containment between samples, or solver convergence beyond the evidence recorded by the existing mechanisms.

## Task 7 study-specific rulings and implementation corrections

For Task 7, profile rounds and profile budget must be reported separately from spatial-grid differences. A geometry pair is eligible only when both plans are independently validated and the profile-refinement criterion is met for both; otherwise report an inconclusive comparison even when numerical differences exceed project thresholds. Warm-start policy matching remains mandatory: a cold 2 m run versus a squared-speed-warm 1 m run is not a compatible spatial pair. Cold 0.5 m runs are sensitivity rows, not grid-convergence pairs against warm-start plans.

Plan file/array digests are per-run integrity bindings, verified against each immutable artifact and manifest, and are not cross-resolution compatibility fields. The pre-finalized study-contract hash embedded in a plan is reconstructed from the finalized manifest with pending plan digest placeholders; do not rewrite existing plans. Segment lap time is recomputed with the solver/validator's exact harmonic segment expression, `sum(2*ds/(v_i+v_{i+1}))`. Closed-path heading seam equivalence permits wrapped-equal headings or a full-turn unwrapped difference.

Project thresholds remain: profile candidate time relative difference <=0.001 and candidate speed max <=0.02 m/s; geometry lap time relative <=0.005, common-reference speed max <=0.1 m/s, and offset max <=0.05 m. These are study criteria, not paper-derived tolerances.

## Required analysis outputs

For each reported experiment, include the paper-derived check/evidence above, convergence/discretization error and wall time, executed settings, provenance status, and the limitations of sampled containment and historical records. Do not transfer another model's mesh, solver tolerances, steering limits, or claims.
