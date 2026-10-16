# Exploratory 7DoF acceptance-gate trials — original and canonical MIS lines

> **Superseded 2026-10-16:** generated with track 1.016 m and 55 % front mass. Values do not apply to the current vehicle.

> **Tools layout migration note.** `tools/` was reorganised into category subpackages (`data`, `tracks`, `model_data`, `run`, `gates`, `reports`, `diagnostics`). Commands and paths in this document use the current locations; commands recorded inside the saved `attempt_*` directories, and the frozen `tube_v2_protocol.md` / `tube_v2_protocol.freeze.txt`, use the pre-migration flat paths (e.g. `tools/check_iem24_forward_tube_v2.py`) and are intentionally not rewritten. See [docs/dev/tools-layout.md](../../../dev/tools-layout.md) for the old-to-new mapping. Recorded hashes, attempt directories and frozen artifacts are unchanged.

> **Provisional numerical acceptance record, not a simulated-vs-measured MIS report or a validated lap claim.** The existing [paired comparison](report.md) describes the cold-start traces and their measured-data comparison; this note records subsequent warm-start and refinement experiments. Neither run is a minimum-time optimum. **For the latest certified results, see the chronological verdict at the end of this record; earlier provisional verdicts are retained as historical results.**

## Protocol and provenance

Each baseline starts from the final physical state and pedal of the corresponding completed cold lap, resetting only path distance to zero. Both tracks use the same compiled IEM24 7DoF vehicle, live driver, 60 m/s cap, 0.175 s pedal travel, BDF, 0.5 m path plan, `rtol=1e-5`, and maximum integration step 0.02 s. Each independent refinement starts from **the same cold-lap endpoint**, at `rtol=5e-6` and maximum step 0.01 s. The runner verifies matching source hashes for warm starts; the refinement checker requires matching model/plan hashes and bit-identical initial trace and pedal rows. Each integration has its own 300 s parent wall-time cap. Original and canonical are **different track geometries**, not two vehicle formulations.

| Track | Cold source | Warm baseline | Independent refined attempt |
|---|---|---|---|
| Legacy original (1016.701 m) | [summary](original/attempt_1792091044461992000/summary.json) | [summary](original/closure_acceptance/attempt_1792092436305043000/summary.json) | [summary](original/refinement_acceptance/attempt_1792092584385545000/summary.json) |
| Canonical ten-lap median (1009.195 m) | [summary](new/attempt_1792091049300790000/summary.json) | [summary](new/closure_acceptance/attempt_1792092440876454000/summary.json) | [summary](new/refinement_acceptance/attempt_1792092589116351000/summary.json) |

## Acceptance results

The seam test compares ten states (lateral displacement, heading error, longitudinal and lateral velocities, yaw rate, four wheel speeds, steering) to the runner's per-state tolerances. Path distance is reset; SOC is an intentionally depleting state and is **not** part of this periodic test, nor is pedal included. The maximum seam ratio below is the maximum absolute state delta divided by its corresponding tolerance; a ratio ≤1 passes.

| Gate / quantity | Original baseline | Original refined | Canonical baseline | Canonical refined |
|---|---:|---:|---:|---:|
| Full lap / integration / path / ≤0.5 m spatial step / accepted-state physics / contact | PASS | PASS | PASS | PASS |
| Lap time (s; exploratory driven result) | 73.596485 | 73.596472 | 71.905397 | 71.905401 |
| Parent wall time (s; limit 300) | 117.261 | 164.700 | 116.064 | 162.366 |
| Periodic closure, maximum tolerance ratio | **0.02633× PASS** | **0.00972× PASS** | **0.02263× PASS** | **0.00232× PASS** |
| Every accepted interval's dense-output midpoint sampled and checked | PASS | PASS | PASS | PASS |
| Continuous between-step certification | **NOT ESTABLISHED** | **NOT ESTABLISHED** | **NOT ESTABLISHED** | **NOT ESTABLISHED** |
| Independent tighter-step refinement | paired with refined → PASS | **PASS** | paired with refined → PASS | **PASS** |
| Recorded `usable` | **false** | **false** | **false** | **false** |

The [original refinement comparison](original/refinement_acceptance/attempt_1792092584385545000/refinement_comparison.json) has maximum lap-time, body-speed and wheel-speed differences 0.00001286 s, 0.0002514 m/s and 0.01558 rad/s (limits 0.02 s, 0.03 m/s and 0.30 rad/s); its lateral displacement, heading, yaw-rate and steering checks also pass. The [canonical comparison](new/refinement_acceptance/attempt_1792092589116351000/refinement_comparison.json) gives 0.00000365 s, 0.0005565 m/s and 0.01281 rad/s, with the other four checks passing. Both comparisons sample 1017 common physical-distance positions. These numerical convergence checks do not establish measured-data validity or global optimality.

## Between-step gate and verdict

[`solve_forward`](../../../../src/simplified_vehicle_models/solvers/forward_driver.py) checks every accepted integration state and the dense-output midpoint of every accepted interval against the physical plant. This is substantial **sampled** evidence, but one interior sample cannot certify the entire continuous interval: violations between endpoints and the midpoint remain possible. Accordingly the [runner](../../../../tools/run/run_iem24_forward_driver.py) and [refinement checker](../../../../tools/gates/check_iem24_forward_refinement.py) deliberately leave `between_step_validation_complete=false`. The acceptance gate requires that field to be true. It would be incorrect to relabel midpoint sampling as continuous certification or force `usable=true` merely because the other gates now pass.

**Decision: not yet usable as an accepted validated 7DoF lap, on either track.** Periodic closure and the stated refinement tolerances now pass on both warm-started runs under the 300 s cap, but continuous between-step certification remains open. The cold-start laps in [the paired comparison](report.md) still fail their own seam and refinement gates; their existing `usable:false` verdict must not be replaced with the warm-lap result. Road-grip coefficients also remain an unvalidated prior, and the canonical line is derived from the measured stint, so numerical acceptance alone would not establish independent physical validation.

## Reproduce

Run from the repository root with the same hashed source files and track inputs as the saved attempts. For each track, use the cold source above as `--warm-start` twice, first with `--rtol 1e-5 --maximum-step 0.02`, then with `--rtol 5e-6 --maximum-step 0.01`; specify `--track` as either `models/tracks/2024_michigan_endurance_closed.xml` or `models/tracks/2024_michigan_endurance_multilap_closed.xml`, plus `--planner live --maximum-speed 60 --pedal-travel-time 0.175 --method BDF --spacing 0.5 --duration 300 --timeout-seconds 300 --output-root <new-output-root>`. Use `PYTHONPATH=src:. python3 tools/run/run_iem24_forward_driver.py` for both integrations. Compare each independently generated pair with `PYTHONPATH=src:. python3 tools/gates/check_iem24_forward_refinement.py <warm-baseline-attempt> <refined-attempt>`. Inspect both `summary.json` and `parent.json` for each attempt; do not substitute a successful process exit for the individual acceptance flags.

Regression checks during these trials: `PYTHONPATH=src:. python3 -m unittest tests.test_forward_driver tests.test_forward_driver_runner -q` (24 tests pass), and `PYTHONPATH=src python3 -m unittest discover -s tests -q` (619 tests pass; an existing ResourceWarning for an unclosed temporary `diag.jsonl` appeared).

## Interval-wide certification attempt (turn 2)

Code: `solvers/dense_domain.py` (exact degree-<=5 BDF dense polynomial per
accepted interval, Bernstein-hull enclosure of all 13 components),
`solvers/planar_dense_certificate.py` (plant state-domain inequalities over
those enclosures, plus parameter-dependent contact conditions). Tests:
`tests/test_dense_domain.py`, `tests/test_planar_dense_certificate.py`. The
runner replays no extra plant calls for this, so wall time is unchanged
(cold/warm about 116 s, refined about 148 s, cap 300 s).

New chain under the new source hashes (cold -> warm baseline rtol 1e-5 / step
0.02 -> refined rtol 5e-6 / step 0.01, same cold endpoint), under
`original/between_step_acceptance/` and `new/between_step_acceptance/`.

| Quantity | Original | Canonical |
|---|---|---|
| Warm lap time | 73.5965 s | 71.9054 s |
| Warm / refined max seam ratio | 0.0263 / 0.0097 | 0.0226 / 0.0023 |
| Refinement check | pass | pass |
| `dense_polynomial_domain_certified` | true (11964 intervals) | true (12029 intervals) |
| `contact_nonnegativity_conditions_hold` | true | true |
| Minimum sampled contact load | 42.4 N | 44.9 N |
| `between_step_validation_complete` | false | false |
| `usable` | false | false |

### What is and is not established

Established, for the reported BDF interpolant only: SOC in [0, 1], v_x > 0.25,
wheel speeds >= 0 and far below the rolling-moment pole, steering within its
bound, path denominator >= 0.5 (smallest margin 0.47) and path progress > 0.25
hold on every accepted interval, not only at sampled points.

Contact: the sufficient conditions are 2*M*h/t <= 1 and the front/rear static
and aero axle conditions (M is the largest of the four friction coefficients),
plus small `slip_eps` and `force_eps`. They hold for the IEM24 vehicle (lateral
ratio 0.937). The shorthand "2*mu*h/t < 1 alone" is false: counterexamples with
a large drag area, a short wheelbase, or a large `slip_eps` give negative loads
while that ratio stays below one. The argument is analytic and checked against
sampled states, not machine-proved.

Not established, so the between-step flag stays false:

* The enclosures cover the interpolant, not the true ODE solution. BDF interior
  error is not rigorously controlled.
* The implicit load fixed point cannot be proved contractive. Random states hit
  "load closure failed" in about 5 of 3000, so `load_converged` between nodes is
  sampled evidence only.
* The powertrain closure between nodes is an argument by construction, not a
  proof.
* Rounding is covered by outward padding, not directed-rounding arithmetic.

Verdict: still NOT flagged usable on either track. Periodicity and refinement
pass and the explicit domain is certified on the interpolant, but a full
between-step certificate is not achievable with the current model, and no gate
was weakened.

## Solver-fix pass (turn 6): load solve, continuous tube, interval load proof

Scope: only items a solver change can fix. Model-correlation items (road-grip
prior, canonical line from the measured stint, physical validation) and the
load-transfer-lag state are the user's call and are closed as out of scope here.
All results below are in `original/final_acceptance_run5` and
`new/final_acceptance_run5` (cold, warm, refined, finer, each run under the
300 s wall cap; 111-171 s measured). Pass criteria were set before any run.

| Item | Result |
|---|---|
| Load closure solver | Fixed. Newton with a Broyden update replaces the plain fixed point; 0 failures in 6000 random-state evaluations (was 52 and 9). Every lap evaluation used Newton, none fell back. Steps per call fell from 15.1 to 10.8 on lap states. |
| Powertrain closure | Resolved by argument plus audit: `tests/test_powertrain_closure_audit.py` checks the torque envelope, current-versus-power and the residual that `brentq` brackets on a grid. A grid audit, not a proof. |
| Contact loads | `tests/test_contact_condition_grid.py` found that raw per-wheel loads can be negative inside the acceleration box (down to -2871 N). The conditions secure only the axle totals; the model lifts such a wheel at zero load. The docstring said otherwise and has been corrected. |
| Midpoint replays | Now also record a scaled defect `|dy/dt - f| * h / (atol + rtol |y|)`. The maximum was 42 (refined) and 78.7 (cold). It is evidence about the interpolant, not a bound, and not a pass criterion. |
| Dense polynomials | Saved as `dense_polynomials.npz` per run; `tools/gates/check_iem24_forward_tube.py` compares three levels continuously (9 samples per interval). |
| Three-level tube, original track | Passed. All components shrink (observed ratios 0.15-0.76). Largest tube 6.3e-3 rad/s on the wheel speeds, 6.2e-5 on yaw rate. Margins survive widening by twice the tube. |
| Three-level tube, canonical track | **Not passed.** Yaw rate, steering and the two front wheel speeds do not shrink between the medium and finer level (ratios 1.05-1.14). The gap between medium and finer levels is about 3e-3 rad/s on wheel speed, so the tube is not confirmed. The widened domain margins would still hold. |
| Interval proof of the load closure | **Not achieved.** 20749 of 20753 intervals (original) and 20577 of 20589 (canonical) fail the Krawczyk test on dense-interval boxes. The measured point Jacobian has norm below 1 on the lap, so the cause is interval overestimation of 20-30 times (the dependency problem), not true non-contraction. Subdividing helps slowly: on one 300-node slice 93% of boxes fail at 64-fold subdivision and 51% at 256-fold (115 s for that slice alone), so it does not close at a tractable cost for a 20000-interval lap. |

Flags after these runs, both tracks: `between_step_validation_complete=false`,
`usable=false`. `tools/gates/certify_iem24_forward_between_step.py` raises the flag only
when the dense-domain, tube, contact, audit and load-closure certificates all pass;
the load-closure certificate fails on both tracks, and the tube also fails on the canonical track.
`check_iem24_forward_refinement.py` no longer forces the flag to false, and
`acceptance_gate` is unchanged.

Not covered: the interpolant bounds are not bounds on the true ODE solution;
the tube is empirical and assumes geometric convergence; rounding is padded
rather than directed; the road-grip coefficients are an unvalidated prior; the
canonical line comes from the measured stint; the laps are driven transients.
This is numerical acceptance, not physical validation.

Verdict: not usable on either track. No gate was weakened.

## Load-closure certificate, sharper enclosure (run6)

Option 1 of the last discussion: only the checker changed. `planar_7dof.py`,
the tire and vehicle XML, the lap trajectories and the lap results are
untouched. Everything was re-run from cold into `final_acceptance_run6/` so
the recorded source hashes match the code that certified (the certifier now
refuses to raise the flag if they do not). `final_acceptance_run5/` is kept as
the earlier failed record.

Pre-registered criterion: every interval of both final laps certifies, under
300 s per lap. Limits and `acceptance_gate` were not changed.

What changed in the interval code (`solvers/load_interval.py`, `dense_domain.py`):

- The lateral-transfer clamp loop computed `x - clamp(x)`, a cancellation that
  intervals cannot see. Where no clamp can be active over the whole box, the
  unclamped transfers are now used (exact, not a relaxation).
- The tire force direction times magnitude is smooth at zero slip, but its
  interval derivative divided by a near-zero quantity (widths up to 300). It
  now also carries an analytic bound from the homogeneity of that term and
  monotone enclosures of `(1 - e^-p)/p`. The two derivative enclosures are
  intersected, and the original is kept if rounding crosses them.
- Each guess box is re-centred by two Newton steps on the box's own mid state
  (Krawczyk is sound for any centre). Interpolated node guesses sat about 6e-5
  off the root, which exceeded the box radius.
- Each dense interval is bisected in time until every sub-span passes. A
  shorter span has a tighter Bernstein hull. The cover is exact.
- `_cos_range`/`_sin_range` widening was relative only, so `sin(0)` was
  excluded by ~1e-16; an absolute margin was added. A new test found this.

| | Original track | Canonical track |
|---|---|---|
| Load-closure certificate | **Certified**: 239,479 sub-spans, 0 failed, deepest bisection 8 | **Certified**: 223,572 sub-spans, 0 failed, deepest bisection 8 |
| Certifier wall time | 167 s | 225 s |
| Three-level tube | Passed (all components shrink) | **Not passed** (yaw rate 1.05, steering 1.14, w_fl 1.07, w_fr 1.09; heading 0.98) |
| Periodicity, refinement | Pass | Pass |
| `between_step_validation_complete` | **true** | false |
| `usable` | **true** | false |

The canonical tube result is unchanged from run5 and is a separate issue that
this pass did not address. It is not a solver-fix item I can close by
re-running: the finest level does not shrink relative to the medium level on
four components, so the empirical tube assumption fails there.

Caveats that still apply to the original track's `usable=true`:

- The certificate covers the solution branch inside each guess box, not that
  the solver's iterate lies on it.
- Interpolant bounds are not bounds on the true ODE solution. The tube is
  empirical, from three levels under assumed geometric convergence.
- Rounding is padded with outward widening, not directed rounding.
- Powertrain monotonicity is a grid audit plus argument.
- Per-wheel loads before the wheel-lift model can be negative.
- The road-grip coefficients are an unvalidated prior.
- This is numerical acceptance, not physical validation.

Historical run6 verdict: original track usable; canonical **finer** attempt not usable (tube). The prospective fourth-level outcome below does not retroactively change that attempt's certificate.

## Prospective canonical fourth refinement (run6 source snapshot)

After investigating the canonical four-component supremum spikes, a separately named, nonbinding [tube v2 protocol](tube_v2_protocol.md) was frozen (revision 2, SHA-256 `b0b499b2933048028f0c1b6196f26c5b01d725fb1ce172a65572454e486788f7`) **before** generating a held-out fourth level. The event analysis found pedal-zero torque-slope kinks near 5.71 and 28.41 s and a driver's piecewise-linear curvature-preview kink near 0.52 s; crossing the periodic plan seam near 71.82 s explains another local bump. This is a causal explanation of the forcing nonsmoothness, **not** a demonstrated solver restart or a proof that the errors are harmless. Increasing the dense sampling from 9 to 65 points per interval changed maxima by under 0.2%; even accepted-node-only comparisons fail on four components. The original v1 criterion has not been changed.

The canonical [fourth attempt](new/final_acceptance_run6/fourth/attempt_1792118214994730000/summary.json) uses `maximum_step=0.0025 s`, `rtol=1.25e-6`, the same cold endpoint, source hashes and other settings as the existing refinements. It completed in 269.4 s (the **per-run** 300 s cap). The unchanged [v1 tube certificate](new/final_acceptance_run6/fourth/attempt_1792118214994730000/tube_certificate.json) on the predeclared **refined → finer → fourth** triple passed: all 13 component sup differences strictly shrink (largest ratio 0.925 for distance), and the widened domain certifies. The [refinement comparison](new/final_acceptance_run6/fourth/attempt_1792118214994730000/refinement_comparison.json) passed. The [between-step certificate](new/final_acceptance_run6/fourth/attempt_1792118214994730000/between_step_certificate.json) certified all 223,416 load-closure sub-spans across 32,618 dense intervals, zero failures, deepest bisection 7, in 344.7 s of separate checker time. Every other certificate and source-hash check passed. After both checkers updated the [summary](new/final_acceptance_run6/fourth/attempt_1792118214994730000/summary.json), `between_step_validation_complete=true`, `refinement_validated=true`, and **`usable=true` under the original acceptance gate**. The checker has no 300 s run cap; its 344.7 s does not violate the integration-run cap.

The earlier canonical [finer attempt](new/final_acceptance_run6/finer/attempt_1792113956706252000/summary.json) retains `usable=false`, because its own warm → refined → finer [v1 tube certificate](new/final_acceptance_run6/finer/attempt_1792113956706252000/tube_certificate.json) failed on yaw rate, front wheel rates and steering. Do **not** relabel it based on the newer triple. The optional [v2 diagnostic](new/final_acceptance_run6/finer/attempt_1792113956706252000/tube_v2_certificate.json) is not an acceptance path: it found domain margin at twice its diagnostic scale and held-out `d_fg<=T`, but its out-of-window strict shrink failed for yaw/steering in the original triple and v_x, w_fr, w_rl in the held-out pair. No v2 pass is claimed. **Retrospective protocol disclosure:** the revision-2 [protocol](tube_v2_protocol.md) was hashed before g, but the final v2 script and tests were completed and hashed *after* g; their post-run fail-closed source, warm-start, array-consistency and margin checks, plus an explicitly exploratory aggregate 4b status, cannot be described as pre-registered implementation. The frozen text also has two editorial defects: its introductory run6 ratios refer specifically to the canonical/new track, not the original, and its date in the opening paragraph reflects revision 1 rather than revision 2. The [freeze log](tube_v2_protocol.freeze.txt) identifies both versions and timing. The v2 sampled-sup pair comparisons use each pair's finer grid, so sampling density differs; this is a diagnostic, not a continuous polynomial supremum or true-solution bound. The prospective evidence that determines usability is instead the unchanged **v1** triple, which passed. These findings are numerical acceptance of dense interpolants and empirical refinement only; they are not validation against measured vehicle behavior or rigorous error bounds on the true ODE trajectory.

**Git publication scope:** The versioned acceptance record includes source code, track/tire inputs, summaries, check certificates, and this chronology. The large run6 dense-polynomial `.npz` files and raw run6 traces are retained in the local attempt directories but deliberately not committed to ordinary Git. Therefore a fresh checkout can inspect the decisions and source hashes but **cannot independently replay the load-closure/tube checks from the saved run6 polynomials without obtaining or regenerating those raw artifacts**. The run summaries also contain original-machine absolute paths, so reproduction on another machine requires new attempts. The cold paired-report attempts and their traces/source snapshots *are* included so the paired comparison can be re-rendered (subject to the separately supplied measured MAT file).
