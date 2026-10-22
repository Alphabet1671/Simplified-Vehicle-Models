# No-TV Minimum-Lap Reliability Matrix

## Purpose and scope

[`verify_no_tv_minimum_lap.py`](../../tools/diagnostics/verify_no_tv_minimum_lap.py)
builds a bounded, reproducible evidence matrix for the spline minimum-time line
solver. It is a **no torque-vectoring** diagnostic harness: it does not install a
controller, invoke a controller global, select modes, alter vehicle physics, or
write cache/source inputs. It is not an event-result or vehicle-acceptance runner.

The harness uses the existing `solve_spline_line`, authoritative spline trajectory,
Hermite--Simpson dense state evaluator, and `validate_line`. It does not recreate
or smooth physics outside those APIs.

## Preregistered cases and start declarations

| Case | Geometry and topology | Start declaration | Interpretation |
|---|---|---|---|
| `mis_closed` | original closed MIS | `closed_flying` | full flying lap |
| `mis_cut_open` | same MIS samples and widths, in-memory full cut-open view | `open_rolling_10` | closure-control diagnostic |
| `fsc_closed` | original closed FSC | `closed_flying` | full flying lap |
| `fsc_cut_open` | same FSC samples and widths, in-memory full cut-open view | `open_rolling_10` | closure-control diagnostic |
| `skidpad` | original open course | `open_rolling_10` | `NOTstandingstart`, candidate only |
| `acceleration` | original open course | `open_rolling_10` | `NOTstandingstart`, candidate only |

A full cut-open view retains both terminal cross-sections and changes only
endpoint/periodicity treatment; it is not a cropped course. Thus topology effects
can be distinguished from changed geometry, samples, or width. The open skidpad
and acceleration models do **not** yet contain a launch phase with consistent
timed distance and time. Positive-speed solves for them are never silently called
standing starts, and no launch correction is added over a full-distance positive
start solution.

## Capability representation and provenance

The default cache is
[`signed_lp_fullrange_product_20261022.npz`](../../runs/no_tv_reliability/signed_lp_fullrange_product_20261022.npz).
The harness dispatches it explicitly as the versioned
`signed_transient_normalized_v1` representation. A legacy capability cache remains
an explicit `legacy_capability_surface` choice; no absent signed coverage falls
back to legacy automatically. Every attempt records the representation, full
formulation/surface paths, a capability/formulation provenance check, and SHA-256
hashes of the formulation, cache, track XML, and referenced track assets before
and after the matrix. Hash discovery recursively follows XML `data` and `source`
references relative to the declaring XML, rejects missing dependencies, and stops
safely at cyclic XML references.

The signed product cache remains under provenance review. Its presence is not a
claim of physical acceptance.

## Bounded execution and artifacts

The default sequence is independent mesh rungs `5, 2.5, 1.25 m` and independent
offset-basis spacings `35, 17.5 m`, each with deterministic `centerline` and
`low_amplitude_offset` coefficient seeds. The low-amplitude seed is a perturbation,
not presumed corridor/physics feasible: its coefficients and a control-site
corridor-domain check are recorded, while full feasibility remains a solver and dense
validation result. A smoke invocation can select a bounded subset, but only those
preregistered rungs:

```bash
PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m \
  tools.diagnostics.verify_no_tv_minimum_lap \
  --cases acceleration --mesh-spacings 5 --basis-spacings 35 \
  --seeds centerline --maximum-iterations 20 --wall-cap-s 120 \
  --output runs/no_tv_reliability/task5-accel-smoke
```

The output directory must not exist. Before spawning a worker, the harness writes
its complete `request.json`. Each attempt has its own exclusive directory,
`result.json`, and (only on a successful solver return, before dense validation)
`solution.npz`. Workers use Python's spawn context and only filesystem request/result
communication. The parent terminates and joins a worker that reaches the wall cap,
then writes the authoritative `timeout_result.json`; it retains any existing child
`result.json` as worker evidence. A worker error, missing result, non-success solver
status, or dense failure is persisted and is never converted into a `SplineSolution`
acceptance. JSON uses `null` for all non-finite diagnostics.

Recorded fields include actual settings, mesh and basis spacing, requested seed,
wall/iteration caps, objective, primal error, available dual error, iterations,
status/error, original-model dense report, source hashes, start treatment, and
artifact name. Failed solvers currently raise before exposing a raw primal/dual
iterate; their typed exception/status is still recorded rather than inventing an
accepted solution.

## Evidence interpretation

Completed artifact pairs appear as explicitly unqualified candidate diagnostics
(time difference and RMS path, speed, and curvature), including dense-failed results.
They are not convergence evidence. Qualified refinement comparisons require both
rows to have solver success, dense geometry and feasibility success, live original
model feasibility, identical seed/settings/source hashes/representation, and exactly
one changed axis: mesh at fixed basis or basis at fixed mesh. It does not claim
convergence for a missing mesh/basis rung. Strict aggregate status is intentionally
`incomplete` until **all** of these have independent evidence:

1. mesh refinement;
2. basis refinement;
3. matched same-condition baseline; and
4. exact-trajectory, matched-start point-mass replay.

`validate_line` provides dense geometry, state/control, capability, and original
point-mass equilibrium diagnostics, but a dense pass alone cannot satisfy the
other conditions.

## Measured limits as of 2026-10-22

Existing artifacts are candidate evidence, not green results:

- [`signed_accel_refinement_20261022.json`](../../runs/no_tv_reliability/signed_accel_refinement_20261022.json)
  records a rolling-10 signed-capability acceleration candidate at `4.508892776 s`
  (mesh 2.5 m, 35 m basis). It has `Solve_Succeeded`, but dense validation fails
  microconstraints/dynamics; it is not accepted.
- The bounded signed skidpad result at cap 300 converged near `21.5696 s`, but its
  live and dense checks fail. It remains candidate-only.
- No matched no-TV braking reference and no standing-start launch accounting are
  available. Refinement/replay evidence remains outstanding.

Do not use these values as event times, a calibrated vehicle result, proof of
physical feasibility, or a convergence claim.

## Focused verification

```bash
PYTHONPATH=src python3 -m unittest tests.test_no_tv_minimum_lap -v
```

The focused tests cover case construction, explicit starts, preregistered rungs,
finite JSON, exclusive output, worker timeout termination, missing-worker failure
persistence, input hashing, and fail-closed strict aggregation. They deliberately
do not run the full matrix.
