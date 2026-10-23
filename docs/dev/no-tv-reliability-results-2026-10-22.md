# No-TV reliability results ledger — 2026-10-22

This is a curated developer status ledger for the no-TV reliability work at
reviewed HEAD `c934b01`. It records achieved behavior separately from actual
unqualified results. It is **not** an MIS/FSC event report, a standard report,
or a declaration that the implementation goal is complete.

## Reviewed implementation achieved

- The no-TV harness preserves legacy defaults while persisting explicit runtime,
  initializer, source/code/input provenance, and current Git identity.
- The opt-in `quadratic_energy_rate_over_metric_v1` control representation is
  reviewed clean (implementation `27b2c91`, critical physical-site correction
  `8fba1aa`, proof `f98199b`, final review `a54d02db`). Raw physical
  acceleration is used at nodes/midpoints; off-sites use interpolated `F=2aG`
  divided by `2G`; the independent Hermite--Simpson energy derivative is
  consistent. The legacy `quadratic_acceleration_v1` default, energy floor, and
  legacy artifact treatment remain explicit.
- The opt-in `speed_domain_margin_mps` control is reviewed clean (`7ae2b33`,
  metadata correction `c934b01`, final review `fc503145`). A positive margin
  enforces a conservative inner domain at all site families; zero preserves the
  legacy default. It is not an unrestricted physical optimum.
- The whole evidence-policy correction (`c2aa8c0`, `8d8ed57`) is reviewed
  clean: stable initializer policy and actual pre/post code/runtime/input
  provenance are persisted. CLI geometry/state budgets are **diagnostic only**;
  no approved policy can turn them into strict acceptance.

## Failure diagnostic contract

A strict spline solver rejection now raises `SplineSolveError`, a compatible
`RuntimeError` subclass. Its `diagnostics` dictionary reports return status,
per-stage and total iterations, objective, physical maximum constraint and
bound violations, plus scaled dual infeasibility when IPOPT supplied it. Raw
last-iterate decision, constraint, and available dual arrays are immutable
copies; this is failure evidence, not an accepted `SplineSolution`.

The no-TV worker records this dictionary as `failure_diagnostics` and, when
raw arrays exist, writes the exclusive `failed_iterate.npz` as
`failed_artifact`. It deliberately does not set `artifact`, invoke validation,
or feed that file to candidate/refinement comparison. The result remains
`accepted: false`, `solver_succeeded: false`, and `validation_status:
solver_failed`. Generic legacy `RuntimeError` behavior is unchanged.

## Actual measured cases

The following entries are local, ignored `runs/no_tv_reliability/` artifacts,
not versioned results. Settings listed as “LM” use limited-memory IPOPT. Dense
means independent dense validation; model means the live original-model check.
“PASS” below is a check result, not study acceptance.

| Artifact | Cap / hard wall | Settings | Solver status | Dense | Model / live | Refinement or qualification |
|---|---|---|---|---|---|---|
| `curved_cases_lm_dense_runtime38` | 200 / 120 s | mesh 5 m, basis 35 m, LM, 32 sites, `1e-8` | MIS closed/open: timeout; FSC closed: strict success, 104 iter, `50.674761 s`; FSC open: timeout; skidpad: strict success, 54 iter, `21.673194 s` | FSC and skidpad fail (corridor/dynamics/capability-domain defects) | FSC live PASS; skidpad live FAIL | No curved case qualifies; a timeout is not infeasibility proof. |
| `skidpad_rate_corrected_runtime38` | 100 / 120 s | mesh 1.25 m, basis 35 m, LM, 32 sites, `1e-7`, opt-in rate control, margin 0 | strict success, 55 iter, `21.6735328736 s` | FAIL only on signed capability-domain excess `+0.004086 m/s` near 15.17 m | geometry/live/dynamics/uncertainty PASS | Not qualified: finite sites miss the domain peak. |
| `skidpad_rate_margin_runtime38` | 100 / 120 s | prior case plus margin `0.001 m/s`, 32 sites + six peak fractions | strict success, 63 iter, `21.6756182599 s` | FAIL only on another domain peak `+0.0355711 m/s` near 129.6004 m | geometry/live/dynamics/uncertainty PASS | First peak was fixed, but another peak remains; no qualification. |
| `skidpad_rate_two_peaks_runtime38` | 100 / 120 s | corrected rate control, margin `0.001 m/s`, added second-peak fractions | hard 120 s timeout | no passing dense record | no passing live/dense record | Timeout; no qualification. |

The pre-fix stored new-mode skidpad states are retained only as negative
engineering evidence. A critical node/midpoint physical-acceleration converter
bug was corrected in `8fba1aa`; any pre-fix new-mode NLP site qualification is
**INVALID** and must not be reused or relabeled as evidence for the corrected
mode.

A parent-run margin sensitivity at `0.05 m/s` obtained a strict NLP success
(`43` iterations, `21.7669488 s`) and then independent geometry/live PASS, but
dense validation failed: maximum-acceleration excess `+0.0119038` near
129.894 m and minimum-acceleration excess `+0.0556584` near 15.1409 m.
The speed-domain check was then satisfied, but this conservative inner-domain
problem is still not densely feasible and is not the same as an unrestricted
original-problem optimum. It is therefore not a PASS and is intentionally
excluded from the table.

## Narrow acceleration evidence that did pass local checks

For the rolling-10 acceleration diagnostic, warm starts produced dense and live
PASS results at mesh `1.25 m` and diagnostic `.625 m`; independent fixed-path
reference agreement was about `1e-6` relative. With mesh `1.25 m`, a real LM
basis comparison used four controls at 35 m and six controls at 12 m, both
with dense/live PASS; times were `4.508864647621161 s` and
`4.508864609068931 s`, respectively. This is a meaningful local basis
comparison, unlike the nominal 35/17.5 m pair, which both realize four controls
on the short course.

It is only rolling warm-start evidence: it is not cold-start robustness,
all-case convergence, a global-minimum proof, or standing-start event timing.
Acceleration and skidpad remain explicitly `NOTstandingstart`, because a
positive speed floor is used and launch-distance/time accounting is not
implemented.

## What remains outstanding

1. Obtain dense/live-feasible curved-case candidates and independent mesh,
   distinct-realized-basis, seed, and matched-reference refinement evidence.
2. Resolve MIS/FSC cut/open timeouts and failures without changing source
   models, validator tolerances, or acceptance rules.
3. Define and approve a source-hashed geometry/state convergence policy, then
   collect real data under it. Until then, strict geometry/state convergence is
   `None` and the global study remains incomplete.
4. Implement standing-start launch distance/time semantics or continue to label
   acceleration/skidpad as unsupported for event-time use.
5. Finish the full six-case registered matrix and the final integration review.

## Test and environment status

Use exactly:

```bash
CURRENTPYTHONPATH=src:/tmp/no-tv-casadi-runtime
PYTHONPATH="$CURRENTPYTHONPATH" python3 -m unittest tests.test_qss_spline_ocp -v
```

The frozen spline suite passed **37 tests in 272.421 s**. The earlier focused
selection passed **92 tests in 12.826 s**; the later final integration selection
passed **128 tests in 15.124 s**. Broad non-GUI testing reported 101 of 106
modules passing; five outcomes were nonzero (closed-loop/energy 75-second
timeouts and stale vertical provenance/between-step failures). The motor
fixture issue was fixed in `cb1d344` and reviewed. A full Qt GUI run stalled and
was cancelled. Consequently, no all-repository-green claim is made.

The explicit environment used CasADi 3.8.1 and NumPy 2.5.4. Immutable input
source hashes are preserved in the local ignored artifact referenced by
[`no-tv-minimum-lap-reliability.md`](no-tv-minimum-lap-reliability.md); source
hash checks reported unchanged. Local run artifacts are deliberately ignored;
rebuild new exclusive artifacts using the commands in that developer document
instead of overwriting them.
