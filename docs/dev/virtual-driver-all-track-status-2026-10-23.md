# Virtual-driver all-track readiness — 2026-10-23

**Decision: HOLD — not ready for all-track use.** No configuration in the six-case free-line matrix is currently qualified. This is a cross-case engineering readiness record, not an MIS measurement report or an MIS 2024 report. A strict solver success, successful local checks, or passing tests do not substitute for independent dense feasibility, approved convergence evidence, event semantics, and the other applicable release gates.

**Scope and basis.** This status applies the six-configuration definition and U1/U2/U3 distinctions in the [research roadmap](virtual-driver-all-track-research-roadmap.md#L24-L45), including its release checklist [§8](virtual-driver-all-track-research-roadmap.md#L197-L210). It records the reviewed implementation and evidence available at repository HEAD `d8e4daf` (`d8e4daff489db49f9b455957fb7786a36b1d946b`), with relevant solver source at reviewed code commit `f2e8ea6`; it is not a whole-branch code-quality audit or completion certificate. Prior dated evidence remains available in the [execution-evidence history](all-track-execution-evidence-2026-10-23.md#L89-L117); its narrower fixed-path and coarse diagnostics must not be mistaken for current free-line qualification.

## Six-case decision

| Configuration | Current all-track qualification | Evidence interpretation |
|---|---|---|
| MIS closed | **NO — HOLD** | No qualifying strict + independent dense + live + mesh/basis/seed-converged free-line result. Historical bounded attempts and local fixed-path diagnostics are not qualification. |
| MIS cut-open | **NO — HOLD** | No qualifying free-line result under a reviewed cut/boundary contract. Historical fixed-path evidence does not establish this gate. |
| FSC closed | **NO — HOLD** | One recent strict five-free-coefficient solve and converged validator sampling, but independent dense feasibility fails on unchanged uncertainty bounds; no qualifying convergence study. |
| FSC cut-open | **NO — HOLD** | No qualifying free-line result under a reviewed cut/boundary contract. Historical fixed-path evidence is diagnostic only. |
| Skidpad | **NO — HOLD** | A historical coarse rolling run passed its recorded local dense/live/sampling checks, but finer mesh/basis evidence failed; it is non-converged and not strict acceptance. |
| Acceleration | **NO — HOLD** | Restricted straight standing-start solves are useful diagnostics, not a jointly optimized event-qualified result or approved convergence study. |

“NO” means no case currently satisfies the qualification bar; it does not mean physical infeasibility. Prior evidence does include local checks on narrower problems: for example, geometry-preserving cut fixed-path MIS/FSC runs and the coarse skidpad result described in the [dated evidence history](all-track-execution-evidence-2026-10-23.md#L109-L117). Those do not meet the free-line/all-gates bar.

## Load-bearing current results

**FSC closed, free line.** The public API run used the original source geometry and chart with five free coefficients, 155 nodes, 226 physical constraint sites, and a 150 m basis spacing. It used `energy_hs_v1` with `quadratic_energy_rate_over_metric_v1` and `cell_length_v1` energy-defect scaling. The strict solve succeeded in 68 iterations, with reported time `52.14983839252722 s`, maximum original constraint/bound violation `1.96957778975e-8`, and solve wall time about `22.389 s`. The diagnostic reserve was in memory only: acceleration reserve `min(0.075 m/s², 0.25 × (a_max − a_min))`, where the interval width is the signed chart acceleration range, plus speed-domain margin `0.05 m/s` and corridor margin `0.1 m`. It did not modify original source geometry, chart cache, or model, and it is a conservative modified problem—not an unrestricted optimum.

The unchanged default validator used its original finite-difference stencil, uncertainty formula, and tolerances; only the maximum refinement cap was raised from 7 to 10. It checked 631,128 samples in `101.447 s`; geometry, chart, corridor, and live original-model checks passed, and sampling converged. Dense feasibility nevertheless failed four derivative-uncertainty/upper-bound gates: approximately `2.20874e-6 (m/s)/m` and `2.99826e-5 m/s²` at their worst locations, against unchanged `1e-6` tolerances (the two corresponding upper-bound gates fail as well). Thus this artifact is not accepted or numerically qualified. This is validator sampling convergence for one artifact, not mesh, realized-basis, or seed convergence. Compact measurements and explicit scope are in the development-only [sampling comparison JSON](../../runs/no_tv_reliability/fsc_public_energy_scaling_20261023/sampling_convergence_comparison.json).

**Acceleration.** The reviewed public `physical_units_chain_v1` standing-straight formulation has strict finite-sample original-model passes at `N=32/48/64`, with an explicit `1e-6 m/s²` reserve. This is a restricted fixed-straight, nonnegative-acceleration transcription: it is not the original unrestricted problem, a jointly optimized curved line, or event qualification. At `N=64`, total rest-to-finish time is `6.51374722489 s`, versus timed-segment `6.11936803223 s` and independent maximum-capability ODE benchmark `6.1123040014 s`. The `7.064 ms` difference is not approved convergence evidence. Source distance is `75.3 m`, including a `0.3 m` launch/staging portion; do not add launch time or distance a second time to a full-distance profile. Historical implementation details and limits are described in the dated [execution-evidence history](all-track-execution-evidence-2026-10-23.md); its results are diagnostics, not convergence approval.

**Regression evidence.** The fresh round-40 focused regression run completed **390 tests successfully** in `93.227 s` (the log includes expected argparse output from negative overwrite tests and ends `OK`). This scoped run covered relevant energy, standing-start, replay, policy, geometry, validation, exchange, and runtime modules; it is not a full-repository pass or physical qualification. The earlier 63-unique focused result (10 new tests plus 53 prior tests) remains a separately dated result, not an additional 63 in the round-40 count. Historical unrelated vertical-suite failures/timeouts and Qt stall remain relevant; no blanket all-repository-green claim is made. See [round-40 log](../../.superpowers/sdd/2026-10-23-all-track-virtual-driver/round40-relevant-regression.log#L7-L10).

## Implemented components are not release qualification

Reviewed components include diagnostic seed support, coefficient scaling, energy-Hermite–Simpson row scaling, the restricted physical-units standing-start mode, public documentation, serialized provenance, and weighted-RMS replay diagnostics. These are scoped engineering components, not proof that all six tracks work, not a controller approval, and not U1/U2/U3 qualification. The relevant reviewed implementations include [energy spline OCP](../../src/simplified_vehicle_models/solvers/qss_spline_ocp.py#L1-L15) and [standing-straight OCP](../../src/simplified_vehicle_models/solvers/standing_straight_ocp.py#L1-L7). Reviewed implementation/test/documentation commits include `30e0c52`, `feaafe9`, `363d688`, `f2e8ea6`, `d2e3a4d`, `e1305ac`, and `d8e4daf`.

The seven source inputs and original capability caches were reported unchanged through the fresh round-40 provenance check; source-content identity matched the reviewed solver source. This preserves evidence provenance but does not itself make ignored local run outputs a durable release package. The machine-readable FSC run is development evidence in an ignored run directory, not a clean-environment reproducible release artifact.

## Release gates and next work

The roadmap’s eight-item release checklist remains unmet as a set. The following release work remains:

1. Run strict, independent dense, and live-model qualification for **each** of the six free-line configurations; retain failures and explicit boundary contracts.
2. Resolve the FSC uncertainty-limited derivative bounds through a methodologically justified, independent finite-difference/error analysis. Do not lower tolerances or relabel the current candidate.
3. Register and complete matched mesh, actually distinct realized-basis, and seed comparisons, plus independent reference and serialized-replay checks for all applicable cases.
4. Approve a source-hashed dimensional convergence/acceptance policy with consumer requirements, units, uncertainty treatment, and independent ownership; current policy remains **HOLD**.
5. Settle cut-open and standing-start event semantics, exact distance/time accounting, and correct transitions without double-counting.
6. Declare an end-to-end runtime budget and fail behavior, then reproduce in a clean environment with durable artifacts and manifests.
7. Demonstrate consumer equivalence and closed-loop tracking/safe-end behavior if a virtual driver/U3 is claimed; numerical path feasibility is not tracking.
8. Complete independent readiness review and assess relevant historical regression failures for release impact. This document is a readiness-gap record, not that whole-branch audit.

Until these gates are closed, retain **U1, U2, and U3 as NOT QUALIFIED**. Numerical feasibility would not establish empirical predictive validity against a real vehicle. No validator, default, model, cache, source input, or physical acceptance gate is relaxed by this status.
