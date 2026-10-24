# No-TV Minimum-Lap Reliability Harness

## Purpose, scope, and present verdict

[`verify_no_tv_minimum_lap.py`](../../tools/diagnostics/verify_no_tv_minimum_lap.py)
is a bounded, reproducible **no torque-vectoring** evidence harness for the
existing spline minimum-time line solver. It does not install a controller or
change controller globals, vehicle physics, source inputs, capability archives,
or solver behavior merely by running. It is not an event-result,
standing-start, or vehicle-acceptance runner.

**Current verdict (reviewed at HEAD `c934b01`):** the harness and several
numerical safeguards are implemented and focused-reviewed, but the global
no-TV study is **incomplete**. A narrow rolling acceleration diagnostic has
passing dense/live evidence across two mesh rungs and a genuine four-versus-six
control basis comparison. It does not establish robust curved-track results,
all six registered cases, standing-start event timing, a global numerical
acceptance, or physical predictive validity. The detailed, dated measurement
ledger is [`no-tv-reliability-results-2026-10-22.md`](no-tv-reliability-results-2026-10-22.md).

The implementation uses the existing `solve_spline_line`, authoritative spline
trajectory, Hermite--Simpson dense-state evaluator, and `validate_line`; it
does not reconstruct or smooth physics outside those APIs.

## Registered cases, rungs, and defaults

The six registered cases are `mis_closed`, `mis_cut_open`, `fsc_closed`,
`fsc_cut_open`, `skidpad`, and `acceleration`. The preregistered mesh rungs are
`5`, `2.5`, and `1.25 m`; nominal offset-basis spacings are `35` and `17.5 m`.
A non-registered mesh (including `.625 m`) or basis needs
`--diagnostic-rungs` and is persisted as `diagnostic_not_preregistered`; it
cannot be presented as preregistered evidence.

The default off-collocation tuple remains the legacy solver behavior:
`(1/8, 1/4, 3/4, 7/8)`. Omitting `--off-collocation-divisions` preserves it.
An explicit `--off-collocation-divisions 32` selects every `i/32` interior
fraction except the midpoint. Defaults are `--nlp-tolerance 1e-6` and
`--dense-maximum-refinements 4`. The harness rejects invalid division counts,
non-finite/non-positive NLP tolerances, and dense refinement counts below two.
No custom validation tolerance or tolerance-relaxation switch is offered.

`QssSettings.control_representation` defaults to the legacy
`quadratic_acceleration_v1`; old artifacts that lack the field are explicitly
identified as legacy rather than silently reinterpreted. The reviewed opt-in
`quadratic_energy_rate_over_metric_v1` representation uses a
metric-consistent energy-rate control. At physical node/midpoint capability
sites it uses raw physical acceleration; at off-sites it evaluates the
quadratic interpolation of `F = 2 a G` and converts with `a = F / (2G)`.
This makes the Hermite--Simpson energy equation consistent with the independent
energy derivative (`W_s = F`) without changing the legacy default. Its old
acceleration Bernstein certificate is intentionally not claimed for the new
representation; the positive energy floor remains.

`QssSettings.speed_domain_margin_mps` also defaults to `0.0`, preserving the
legacy feasible set. A finite non-negative positive value requires
`domain_speed - v >= margin` at nodes, midpoints, and configured off-sites and
requires a compatible capability-domain floor before construction. It is an
explicit **conservative inner-domain profile**, not a relaxation, correction
of validation, or claim that the resulting time is the unrestricted zero-margin
physical optimum. The legacy per-node builder/solver rejects a nonzero margin.
The harness records both the selected representation and margin in request,
result, and solution metadata.

## Spline energy-defect scaling (opt-in)

`solve_spline_line(..., energy_defect_scaling="cell_length_v1")` selects a
solver-coordinate change for the Hermite--Simpson energy-defect rows only. The
public default remains `"legacy_reference_speed_v1"`; omitting the keyword
leaves the legacy NLP coordinates and behavior unchanged. The option is an
opt-in numerical-conditioning experiment, not a change to physical acceptance.

For an element of length `h`, the builder's original zero-equality residual is
`g = (W[i+1] - W[i] - h/6 * (F[i] + 4*F[mid] + F[i+1])) / 10`, where
`W = v**2` and `F = 2*a*G`. The selected solver row is `(10/h)*g`, i.e. the
same defect in `m^2/s^2` divided by that cell's length. The positive factor
preserves the zero set. Only recorded energy-defect indices are scaled: other
constraint rows—including periodic seam and open-start rows—retain their
original coordinates.

The public success gate evaluates the original, unscaled constraints and
bounds. Returned maximum violation and failure diagnostics therefore describe
the original residuals; failure metadata labels the scaling and its scope and
does not present it as a uniform physical error certificate. Between IPOPT
continuation stages, constraint multipliers are converted back to original
coordinates for public diagnostics, while warm-start arguments use IPOPT's
raw solver-coordinate `lam_g` (and its corresponding scaled decision state).

Typical use, in an already prepared study where `track`, `corridor`, `surface`,
`mesh`, `spline`, and `settings` have been defined:

```python
solution = solve_spline_line(
    track, corridor, surface, mesh, spline, settings,
    energy_defect_scaling="cell_length_v1",
)
```

This call fragment is not a standalone example or a qualification recipe; it
adds no validation gates, tolerance waivers, or evidence of global readiness.
The no-TV study's current readiness verdict remains **HOLD / incomplete**.

## Artifacts, provenance, and qualification protections

Each attempt receives an exclusive output directory and writes `request.json`,
`result.json`, and, after a solver return, `solution.npz`. It records
`state_representation`, `time_quadrature`, serialized `time_metadata`, control
representation, and speed-domain margin. The runner-generated
`trajectory_metadata.npz` records reference and station-map metadata.

The spawned-process wall timeout is authoritative and child evidence is kept
separately. Before spawn and after every attempt, the parent captures actual
Git HEAD, tracked-dirty manifest, hashes for the executed harness/solver/
geometry/validation/profile implementation, runtime versions, and source-input
hashes. `--base-commit` is optional only; if used, it must resolve to that
actual HEAD. For current reproductions, do **not** supply a stale
`--base-commit` override; use `--baseline-label` only for a historical
comparison label. The reviewed evidence-policy changes `c2aa8c0` and `8d8ed57`
keep initializer comparison stable by comparing requested strategy, resolved
options, actual outcome/fallback category, and implementation/runtime rather
than dimension-specific seed bytes or elapsed diagnostics. They also persist
actual resolved HEAD, code/runtime/input provenance, and the distinct baseline
label.

A mesh or basis edge can qualify only with solver success, dense geometry and
feasibility, live original-model feasibility, identical qualified initializer/
settings/provenance, and exactly one changed axis. Nominal basis labels are not
a basis refinement when they realize the same control grid. A solver-returned
row also needs an actual success status; a diagnostic payload cannot promote a
failed solve.

The optional `--matched-reference` default is false. If selected, it produces
candidate and zero-offset fixed-path capability profiles at `1.0`, `.5`, and
`.25 m`, retaining profile failures rather than erasing the candidate. Passing
reference evidence requires converged `.5`/`.25 m` profiles, their existing
`0.5%` sampled-time comparison, the existing `1%` replay agreement, and a
no-slower free candidate under the existing physical-time tolerance. This is a
sampled fixed-path surrogate, not original-plant or empirical validation.

## Convergence policy and honest strict status

Workers carry common source-station samples for RMS XY, metric, offset, speed,
curvature, and path-length comparisons. `rms_path` is retained only as a
backward-compatible alias for offset RMS; it is not an XY claim.

There is **no approved, source-hashed dimensional geometry/state convergence
policy** for this study. User-supplied
`--convergence-criteria-json` budgets remain diagnostic only: even finite
passing CLI budgets cannot promote strict numerical validation. Therefore the
geometry/state strict gate is `None` and the aggregate remains `incomplete`
until an approved policy and supporting real measurements exist. This
fail-closed rule, together with missing registered rungs/cases, is why the
implemented harness must not be described as a globally ready simulator.

## Runtime, source data, and reproducibility

The reviewed measurements used the explicit runtime:

```bash
CURRENTPYTHONPATH=src:/tmp/no-tv-casadi-runtime
PYTHONPATH="$CURRENTPYTHONPATH" python3 -c \
  "import casadi, numpy; print(casadi.__version__, numpy.__version__)"
# CasADi 3.8.1; NumPy 2.5.4
```

The immutable source targets and their SHA-256 values are also preserved in the
ignored local workspace record
[`.superpowers/sdd/2026-10-22-no-tv-lap-reliability/source-hashes.txt`](../../.superpowers/sdd/2026-10-22-no-tv-lap-reliability/source-hashes.txt):

| SHA-256 | Immutable target |
|---|---|
| `b9164ebbf2851b0e33b4a9a174c230eefa66e296b72e5b0fb9e10ef373071ca3` | `models/tracks/2024_michigan_endurance_closed_edged.xml` |
| `ff8dcdc715eaff07eeb1d7a9e0a0739072480150577622ec8854000a0a44dbd4` | `models/tracks/2024_michigan_endurance_closed.csv` |
| `276dce781d08db9d88dd77728b7389157a5cff351eb991055f39fde780846d84` | `models/tracks/2024_michigan_endurance_closed_width.csv` |
| `b77fb256d1a24c7281ab43e84c643b78a499a11fd894610e306db3078ef69beb` | `tracks_out/fsc_may_2026.xml` |
| `a3ba06448c663df79a09f0ae0ac1ab50adfc32ee07b7efbf4344c2f3569eba9b` | `tracks_out/fsc_may_2026.csv` |
| `7a02c6e81971473445f6c82f963d9b6d4efabdcc7c83fc891661367443d0be71` | `models/tracks/fsae_2026_skidpad.xml` |
| `97c2f3f868af1838444e2a0626e4308b67ff90828feed6a6ec35c4a2edb7d4fe` | `models/tracks/fsae_2026_acceleration.xml` |

The local `runs/` outputs and `.superpowers/` working notes are deliberately
ignored and are **not versioned deliverables**. They retain commands, result
JSON, NPZ states, and source hashes for reproduction but must not be mistaken
for committed curated evidence. Build a new exclusive signed chart rather than
overwriting an existing artifact:

```bash
CURRENTPYTHONPATH=src:/tmp/no-tv-casadi-runtime
PYTHONPATH="$CURRENTPYTHONPATH" python3 tools/model_data/build_signed_capability.py \
  --formulation models/formulations/iem24_murphy_real_planar_7dof.xml \
  --interval-fit linear_program \
  --output runs/no_tv_reliability/signed_lp_fullrange_product_rebuild.npz
```

A bounded acceleration smoke reproduction (with no stale `--base-commit`) is:

```bash
CURRENTPYTHONPATH=src:/tmp/no-tv-casadi-runtime
PYTHONPATH="$CURRENTPYTHONPATH" python3 -m tools.diagnostics.verify_no_tv_minimum_lap \
  --cases acceleration --mesh-spacings 1.25 --basis-spacings 35 \
  --wall-cap-s 120 --output runs/no_tv_reliability/acceleration_rebuild
```

## Focused verification

```bash
PYTHONPATH=src:/tmp/no-tv-casadi-runtime python3 -m unittest \
  tests.test_qss_spline_ocp -v
```

The frozen parent run passed **37 tests in 272.421 s**; see the ignored
[round-18 spline log](../../.superpowers/sdd/2026-10-22-no-tv-lap-reliability/round18-full-spline-tests.log).
The final focused integration selection passed **92 tests in 12.826 s**.
These tests exercise defaults, metadata, physical capability-site handling,
margin constraints, provenance, harness serialization, and fail-closed
aggregation. They do not substitute for the unresolved real-case matrix.
