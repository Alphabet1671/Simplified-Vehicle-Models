# Task 9 — Historical optimized-line replay and yaw-approximation audit

## Scope and decision

Task 9 was implemented and the expressly permitted **historical exploratory** 40 m optimized-line plans were replayed against matched historical fixed 2 m plans. Task 7 remains spatially inconclusive, so no new geometry is selected, no Task 7/8-dependent final matrix was run, and no optimizer/geometry convergence or transient optimality claim is made. Both replay pairs completed full solver validation and completed a lap, but sampled wheel-contact checks fail and whole-footprint/between-step containment are not certified. The Task 9 reliability gate therefore does **not** establish a reliable pipeline.

The legacy OCP and strict map/mask/tolerance semantics were not changed. Existing dirty unrelated work was left untouched.

## Saved inputs, provenance, and matched comparisons

Map identity for all plans: `f8658cafbd35dc60cd195153ca4761db21d9a9d8b4819f1112a4f46ad7322419`. Track identities are preserved in original inputs. The exact named historical optimized plans are `runs/yaw_lap_suite/mis-line-40m-retained/plan.npz` and `runs/yaw_lap_suite/fsc-line-40m-grid2m/plan.npz`; their matched fixed comparators are the 2 m plans `runs/yaw_lap_suite/mis-fixed-2m/plan.npz` and `runs/yaw_lap_suite/fsc-fixed-2m/plan.npz` (not the 1 m historical baselines). Each historical plan was loaded and strictly revalidated by the replay runner against its map and track identity.

All four primary replay processes used `tools/run/run_with_deadline.py --seconds 1500`, duration 150 s, cooperative 1200 s, BDF, rtol `1e-6`, atol `1e-8`, max step `0.05`, and the model-aware controller. Their per-run recorded initial states are available in each run's `inputs.json`; the initial-state policy is identical in code (plant initialized from each bound plan's first speed), while actual numerical initial states differ by plan and are not claimed identical. Missing historical values remain unknown; no provenance was backfilled. The original historical plans predate closure contracts, so their absent historical source/package/thread provenance remains unknown/inconclusive.

| Track | Plan/controller | Planned lap (s) | Driven lap (s) | Speed RMS / max abs (m/s) | Lateral RMS / max abs (m) | Realized throttle saturation fraction | Solver validation | Center / footprint / between-step |
|---|---|---:|---:|---:|---:|---:|---|---|
| MIS | optimized line / model-aware | 60.58923 | 61.33835 | 0.31929 / 0.81770 | 0.06345 / 0.16293 | 0.18682 | complete | contained / unknown (wheel-contact fail) / false |
| MIS | fixed 2 m / model-aware | 61.47060 | 62.19915 | 0.30429 / 0.84442 | 0.06003 / 0.15482 | 0.15214 | complete | contained / unknown (wheel-contact fail) / false |
| FSC | optimized line / model-aware | 51.16844 | 51.83038 | 0.32368 / 0.91454 | 0.07510 / 0.18244 | 0.15239 | complete | contained / unknown (wheel-contact fail) / false |
| FSC | fixed 2 m / model-aware | 51.91682 | 52.57516 | 0.31188 / 0.68035 | 0.07294 / 0.17691 | 0.13800 | complete | contained / unknown (wheel-contact fail) / false |

Optimized minus fixed driven-time deltas are -0.86080 s for MIS (-1.38%) and -0.74478 s for FSC (-1.42%). This is a measured exploratory replay trade-off, not a universal improvement or convergence result. In both cases tracking speed/lateral RMS is slightly worse for optimized, while maximum speed error is smaller for MIS but larger for FSC. Requested and realized controls are distinct in the saved record; only realized saturation fractions are shown here. Brake and steering-rate realized saturation fractions are zero for these runs.

All centerline sampled checks are true. Four-wheel contact sample checks report negative clearance: MIS optimized -14.307 m at 1.729 s and fixed -14.192 m at 1.766 s; FSC optimized -2.411 m at 19.898 s and fixed -2.039 m at 20.086 s. These are conditional on assumed edge provenance. Whole-body envelope data are unavailable; footprint status remains `unknown`, and between-step containment is false. Therefore center containment must not be restated as vehicle footprint containment.

Full machine-readable comparison and provenance binding is in `runs/yaw_lap_suite/closure-v1/comparison-report.json`.

## Integration sensitivity repeats

The actual MIS optimized and matched fixed replay results were repeated with maximum step `0.025` under the same 1500/1200 limits. Both completed solver validation. Driven times and lateral metrics were bit-identical at printed precision to the 0.05 s runs: optimized 61.3383533 s and lateral RMS/max 0.0634499/0.162932 m; fixed 62.1991532 s and 0.0600300/0.154821 m. Changes are 0%, passing the requested ≤0.1% time, ≤0.005 m RMS lateral, and ≤0.01 m max lateral criteria for the MIS pair. Thus the MIS driven-time gain exceeds observed integration sensitivity, but remains an exploratory paired result and does not cure the containment issue.

FSC 0.025 s repeats were not run: Task 7/8 dependent/final convergence work is expressly not authorized, and the task’s conditional repeat wording requires an actual relevant completed replay that can be assessed; the MIS pair is the relevant repeat for the primary interpreted trade-off. FSC comparison remains exploratory and its smaller gain is not declared integration-insensitive.

## Yaw approximation diagnostic

The audit computes `beta=atan2(v_y,v_x)` from the recorded body-slip channel, course rate `r + beta_dot`, total speed `V=hypot(v_x,v_y)`, and `V*kappa`. Yaw-acceleration path demand is `a_t*kappa + V**2*kappa_prime`, where `a_t` is longitudinal tangential acceleration and `kappa_prime` uses physical station distance. The diagnostic trace is uniformly resampled in time at 0.01 s; curvature is interpolated on saved physical path distance, derivatives are centered, and residual summaries include raw RMS/P95/max and a periodic 11-sample boxcar filtered form. Filtered results are stored in JSON. The resampled distance wraps at the lap seam; the diagnostic uses periodic centered beta derivatives and distance-aware curvature derivative. These kinematic residuals do not establish force/moment inconsistency or map invalidity.

| Track/plan | `course_rate - Vκ` raw RMS / P95 / max (rad/s) | `yaw_acceleration - path demand` raw RMS / P95 / max (rad/s²) |
|---|---|---|
| MIS optimized | 0.10487 / 0.17147 / 3.91264 | 0.88470 / 1.86285 / 14.62305 |
| MIS fixed 2 m | 0.10001 / 0.17699 / 3.45234 | 0.88191 / 1.87835 / 15.20287 |
| FSC optimized | 0.09306 / 0.16834 / 1.05313 | 1.65895 / 2.17991 / 22.68459 |
| FSC fixed 2 m | 0.09324 / 0.17146 / 1.03431 | 1.63261 / 2.20671 / 22.51642 |

Analytic tests cover a constant-radius zero-sideslip case (zero residual to numerical precision) and a sinusoidal-beta case where including `beta_dot` recovers course-rate demand while body yaw rate differs. The diagnostic output explicitly labels itself not force-balance proof. The current persisted channel set has yaw acceleration, body-frame acceleration, tire forces, and tire aligning moments, but this report does not claim closure of total net yaw moment; force/moment consistency remains unknown and requires a separate signed frame-consistent sum.

The forward traces expose available channels but no interval/curvature-reversal attribution has been added to this closure artifact. We therefore do not mark specific limit/residual intervals as cause of the observed tracking difference. No global speed derating was applied.

## Tooling, tests, and preserved outcomes

Extended `yaw_replay_metrics.py` with periodic centered derivatives and `yaw_approximation_diagnostics`; added `tools/reports/compare_yaw_planning_replay.py` and tests for analytic cases, irregular-grid rejection, and missing-evidence handling. The comparison tool retains each plan’s recorded inputs/summary; missing replay provenance is `unknown` and prevents success classification.

Focused watchdog validation:

```text
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 180 -- python3 -m unittest tests.test_yaw_planning_replay tests.test_yaw_replay_binding tests.test_yaw_replay_metrics tests.test_yaw_lap_suite tests.test_yaw_lap_artifact tests.test_yaw_aware_record_path -q
Ran 24 tests in 14.337s — OK
```

Primary and repeat replay process exits were 0. No timed-out or failed replay was retried. No Task 7/8 final matrix or geometry selection was attempted. The measured optimized-line runs cannot validate transient optimality, and capability-map feasibility is not a transient optimality statement.
