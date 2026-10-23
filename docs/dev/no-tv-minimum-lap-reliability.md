# No-TV Minimum-Lap Reliability Matrix

## Purpose and scope

[`verify_no_tv_minimum_lap.py`](../../tools/diagnostics/verify_no_tv_minimum_lap.py)
is a bounded, reproducible **no torque-vectoring** evidence harness for the
existing spline minimum-time line solver. It neither installs a controller nor
changes controller globals, modes, vehicle physics, capability/source inputs,
or solver behavior. It is not an event-result, standing-start, or vehicle
acceptance runner.

It uses the existing `solve_spline_line`, authoritative spline trajectory,
Hermite--Simpson dense-state evaluator, and `validate_line`. It does not
reconstruct or smooth physics outside those APIs.

## Registered and diagnostic rungs

The preregistered mesh rungs are `5`, `2.5`, and `1.25 m`; the preregistered
nominal offset-basis spacings are `35` and `17.5 m`. The output labels those
rungs as `preregistered`. A non-registered mesh (including `.625 m`) or basis
spacing requires `--diagnostic-rungs` and is persisted as
`diagnostic_not_preregistered`; it must never be presented as preregistered
evidence.

The exact default off-collocation tuple remains the existing solver behavior:
`(1/8, 1/4, 3/4, 7/8)`. Omitting `--off-collocation-divisions` preserves it
exactly. An explicit `--off-collocation-divisions 32` selects every `i/32`
interior fraction except the midpoint. The harness rejects invalid counts,
non-finite/non-positive NLP tolerances, and dense refinement counts below two.
Defaults are `--nlp-tolerance 1e-6` and
`--dense-maximum-refinements 4`; the measured run below used `1e-8`, `32`, and
six refinements. No custom validation tolerance or tolerance-relaxation switch
is offered.

Each request persists its complete settings. JSON transport changes the
settings tuple to a list, and the spawned worker restores it to the immutable
`QssSettings` tuple before solving.  The optional energy-NLP initializer is
also canonicalized in the request: `seed_profile`, strictness, resolved profile
step, iteration limit, and wall limit are recorded, then the worker records the
actual seed fingerprint and whether it used the projection or explicit old
seed. A profile projection is not claimed feasible or globally optimal.

## Artifacts and qualification protections

Each attempt has an exclusive output directory and writes `request.json`,
`result.json`, and, after a solver return, `solution.npz`. The artifact records
`state_representation`, `time_quadrature`, and serialized `time_metadata`.
The separately written `trajectory_metadata.npz` is produced by the existing
runner serializer and records exact reference and station-map metadata. These
fields make energy-state and time-quadrature interpretation unambiguous without
changing runner output semantics.

The existing protections remain unchanged: spawned-process wall timeout is
authoritative, child evidence is retained separately; source SHA-256 values
are recorded before and after every attempt; and strict acceptance remains
fail-closed. The parent captures actual Git HEAD, tracked-dirty manifest,
SHA-256 hashes for the executed harness/solver/geometry/validation/profile
implementation, and Python/NumPy/CasADi runtime versions before spawn and
again after each attempt. `--base-commit` is optional and, if supplied, must
resolve to that exact HEAD; use `--baseline-label` for a historical comparison
label. Qualified mesh/basis comparisons additionally require solver success,
dense geometry/feasibility, live original-model feasibility, identical actual
initializer fingerprint, settings, source hashes, code/runtime provenance, and
exactly one changed axis.

## Optional matched fixed-path reference

`--matched-reference` is opt-in; its default is false, preserving the previous
attempt behavior and `null` (`None`) matched-baseline/replay checks.  After the
candidate artifact and exact trajectory metadata have been persisted, the
worker computes a `capability_profile` on the candidate's exact analytic
trajectory/path station map and a separate zero-offset analytic centerline
trajectory.  The boundary is the declared `closed_flying` or `open_rolling_10`
treatment; no standing-start phase, forward clipping, or synthetic event start
is introduced.

The declared capability-profile steps are `1.0`, `.5`, and `.25 m`.  The
persisted output records each actual profile's status, iterations, time, and
max-change.  Passing reference evidence requires both `.5` and `.25 m` profiles
to converge, their sampled-time change to be at most the existing `0.5%`
comparison limit, and the candidate `.25 m` profile time to agree with the
reported solution time to the existing `1%` replay limit.  The same convergence
requirement applies to the independent zero-offset baseline; the free candidate
must not be slower than it beyond the existing physical time tolerance.

This is a sampled **capability-table fixed-path surrogate**, not an empirical
or original-plant validation.  It records analytic station-map replay RMS speed
and path sampling RMS, but cannot make `physically_validated` true.  A profile
exception/failure is retained explicitly with false reference checks rather
than crashing the worker or erasing the candidate artifact.

At aggregation, strict `numerically_validated` status is possible only per
case when one actual same-condition source/settings/seed-qualified edge
establishes each mesh and distinct realized-grid basis axis, and both endpoints
of that exact edge carry passing matched baseline and replay evidence.  Edges
are never unioned or re-paired across configurations. Solver-returned rows also
need an actual success status in addition to dense/live checks; a failed status
cannot qualify even if a diagnostic payload says true. Nominal `35/17.5` labels
with the same realized control grid remain ineligible.

The default aggregate always evaluates all six preregistered cases and requires
coverage of every registered mesh (`5`, `2.5`, `1.25 m`) and nominal basis rung
(`35`, `17.5 m`) with actual passing reference/dense/live rows. Diagnostic
rungs (including `.625 m` and custom bases) remain recorded as diagnostics and
cannot establish global study coverage. Thus a bounded subset can report its
per-case evidence but cannot mark the full study accepted; missing, unresolved,
or candidate-only cases keep it incomplete.

Nominal basis spacing alone is not treated as an actual basis refinement.
Every result records its realized `control_count` and `control_station_m`; two
nominal spacings producing an identical realized grid remain candidate-only.

## Common-source convergence evidence

Each real worker artifact also carries dense samples evaluated from the exact
trajectory and dense state evaluator on a common source-station grid: source
station, mapped path station, north/east coordinates, metric, offset, speed,
curvature, and derived path length. Aggregation displays RMS XY, metric,
offset, speed, curvature, and relative length differences. `rms_path` remains
only as a backward-compatible alias for offset RMS; it is not an XY claim.

No universal geometry/state thresholds are invented by this harness. Without
an explicit `--convergence-criteria-json` object declaring finite, nonnegative
budgets for all six dimensional metrics, this evidence is `None` for
qualification and strict aggregation stays `incomplete`, even when the existing
0.5% time edge and 1% replay tests pass. Declared criteria are serialized with
every request and must pass at both endpoints; malformed, nonfinite, negative,
or incomplete criteria are refused.

## Parent measured evidence (2026-10-22)

The following parent measurements are recorded at
[`gl16_energy_dense_sites_runtime38`](../../runs/no_tv_reliability/gl16_energy_dense_sites_runtime38)
for the acceleration course. They are rolling-10 open-course diagnostics, not
standing-start event times:

| Item | Observed result |
|---|---|
| Settings | mesh `1.25` and diagnostic `.625 m`; nominal basis `35 m`; rolling start `10 m/s`; NLP tolerance `1e-8`; 32 explicit off-collocation divisions; old-state warm start; dense maximum refinements six |
| Solve status | strict success at both mesh rungs: 46 and 42 iterations; `4.508864642 s` and `4.508859374 s` |
| Dense validation | geometry, dense feasibility, and live original-model checks true for both; independent profile with `.25 m` step agreed at approximately `1.1e-6` / `7e-8` relative |
| Seed/nominal basis checks | centerline and perturbed seeds with nominal `35` and `17.5 m` returned strict successes near `4.508864651 s`, and independent dense checks passed |

This is achieved mesh/refinement and seed-repeat evidence for this narrow
rolling diagnostic. It does **not** establish all-track acceptance, an event
standing start, torque-vectoring behavior, or a general solver guarantee.

### Explicit remaining limitation: basis convergence is unavailable

On this approximately `75.3 m` course, the nominal `35 m` and `17.5 m`
settings both realize the same four control stations (`0`, `25.1`, `50.2`,
`75.3 m`). Therefore their agreement is not a basis-convergence result. The
actual diagnostic `12 m` basis creates six controls but hit the maximum
iteration cap (initial cap 100); it is not accepted as basis evidence. A real
basis refinement with distinct realized grids and qualified solves remains
required.

The acceleration/skidpad configurations remain `NOTstandingstart`: timed
launch-distance accounting has not been implemented. Do not use these values
as event times, proof of calibrated vehicle performance, or global physical
acceptance.

## Focused verification

```bash
PYTHONPATH=src python3 -m unittest tests.test_no_tv_minimum_lap -v
```

These mocked unit/harness tests cover default preservation, serialized tuple
restoration, invalid settings, diagnostic-rung labels, artifact metadata,
actual-grid basis qualification, exclusive output, timeout handling, source
hashes, and fail-closed aggregation. They do not run a large solve matrix.
