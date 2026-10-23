# No-TV numerical convergence policy v1 (proposed)

**Status:** `proposed_pending_independent_review`  
**Policy identifier:** `no-tv-numerical-convergence-v1`  
**Claim type:** numerical-reference accuracy only

This is a versioned engineering accuracy proposal for comparing two numerical
trajectory representations. It is not a physical prediction validation, track
controller qualification, vehicle-readiness statement, or permission to relax
feasibility checks. Every numerical threshold and consumer-envelope bound below
is an **unapproved design assumption**, not an evidence-derived or approved
consumer requirement. No suitable approved all-track trajectory-accuracy
requirements were identified. Independent consumer evidence and review are
required before any threshold can be called a requirement. No reviewer or
external approval is claimed here.

## Unapproved envelope and illustrative allocation (not requirements)

The values in this section and the following comparison table are assumptions
for review only. A repository consumer-requirements audit found existing
runner-tracking and periodic-closure tolerances, but these address other scopes;
they do not establish general all-track trajectory accuracy, speed, curvature,
clearance, or lap-time requirements. No approved all-track consumer requirements
were found. Before a future approval decision, obtain consumer-owner evidence
for operating envelopes, spatial and temporal fidelity, geometry/clearance,
state/speed/curvature tolerances, and application-specific weighting; record
sensitivity and rationale for each allocation. Do not infer approval from
unrelated harness or closure tolerances.

For arithmetic illustration only, this proposed envelope is speed `v <= 30 m/s`, absolute
curvature `|k| <= 0.24 /m`, and wheelbase `L = 1.535 m` (the current vehicle
value). This defines the domain for this numerical accuracy policy; it is not a
vehicle performance cap. Values outside it are **not qualified** by this
version and must not be clipped into the envelope.

Allocate at most `0.025 m/s^2` of pointwise lateral-acceleration error to
numerical reference error. For maximum speed error `dv = 0.001 m/s`, the
speed contribution at the envelope limit is

`2 * 30 * 0.24 * 0.001 + 0.24 * 0.001^2 = 0.01440024 m/s^2`.

For maximum curvature error `dk = 1e-5 /m`, the curvature contribution is

`(30.001)^2 * 1e-5 = 0.00900060001 m/s^2`.

The combined bound is `0.02340084001 m/s^2`, leaving `0.00159915999 m/s^2`
reserved within the declared allocation. The corresponding feed-forward
steering error from curvature perturbation is bounded by `L * dk =
1.535e-5 rad`.

## Unapproved comparison assumptions (not pass criteria)

The following proposed numbers are stored only to enable review of an explicit
candidate allocation. They have not been accepted as consumer requirements and
must not be used to label candidates qualified or to alter harness status.

| Quantity | RMS assumption | Maximum / other assumption |
|---|---:|---:|
| XY reference position | 0.005 m | 0.02 m |
| Lateral offset | 0.005 m | 0.02 m |
| Path metric | 2e-5 | 1e-4 |
| Speed | 0.00025 m/s | 0.001 m/s |
| Signed curvature | 2.5e-6 /m | 1e-5 /m |
| Relative path length | — | 1e-4 |
| Lap time | — | absolute 0.01 s **and** relative 0.001 |
| Path length | — | absolute 0.05 m **and** relative 1e-4 |

The geometry maxima and RMS assumptions are separate proposed metrics; a small
RMS does not waive an observed sampled maximum. The `0.02 m` geometry value was
motivated by a hypothetical `2 m` reference-clearance context only. No evidence
establishes that clearance as a consumer requirement, and this number is not a
physical corridor allowance or waiver of any geometry/corridor constraint.

## Sampling uncertainty and comparison method

Both exact trajectory evaluators are sampled in vector batches on the same
source-station domain. The union of both solver meshes, reference geometry
stations, and spline control stations forms common initial intervals. Each
interval is subdivided dyadically; all metrics use the common station samples.
RMS uses trapezoidal integration weighted by the common mean path metric
`0.5 * (G_left + G_right)`, normalized by its integral. This avoids treating
unequal path metrics as equal path weights. Maxima are absolute pointwise
sampled differences. XY uses Euclidean position error; offset, metric, speed,
and curvature compare supplied evaluator values without reconstructing or
smoothing a path. Exact station-map endpoints provide path length, while
physical lap-time values are compared as reported rather than recomputed from
an OCP.

Adjacent-grid changes are empirical refinement diagnostics only; they are not
a formal error bound. In particular, stable nested grids cannot rule out an
unsampled narrow peak. The comparator always returns
`maximum_accuracy_certified: false`. Sampled RMS/max values, sampled envelope
checks, and refinement changes are diagnostic fields, not certificates.
`meets_accuracy_budget` remains `null` for a comparison with no known sampled
or integral violation; it is `false` if a sampled or exact integral check
already establishes a violation. It is never `true` merely because sampled
values or refinement changes look stable. Continuous maxima and continuous
envelope membership are not certified by this implementation.

All inputs, evaluator values, metrics, and exact path lengths must be finite;
source endpoints and periodicity must match, and the metric must remain
positive. Source equivalence is required: the same exact reference object may
be used directly, otherwise a canonical fingerprint of source station and
reference XY under a known coordinate-frame convention must match. Unknown
frame/provenance or same endpoints without matching source geometry fails
closed. Candidate trajectory XY must not be substituted for source-reference
XY when computing this identity. Sampled envelope checks are labeled as sampled
only. These comparisons do not replace feasibility checks: the existing
independent `DEFAULT_TOLERANCES` remain unchanged.

## Versioning, hash, and approval boundary

`NUMERICAL_POLICY_V1` provides immutable policy fields and deterministic
canonical JSON bytes. `policy_sha256` is SHA-256 of those canonical bytes; no
current date or runtime value changes the policy hash. Assessment results
include the identifier, hash, and proposed status. A numerical result field
`meets_accuracy_budget` is deliberately **not** an approval or registry
acceptance. There is no approved-policy registry integration in this task and
no harness green state or CLI threshold is authorized. Any future integration
requires independent policy review and a separate task. Later reviewers may
approve only the engineering derivation they have independently examined;
that would not establish physical validity or human/external approval.

An independent review should record numerical-analysis review, consumer
requirements review, and policy-ownership review roles, along with date,
reviewed policy hash, findings, and disposition. No signatures or reviewer
identities are fabricated in this proposal.
