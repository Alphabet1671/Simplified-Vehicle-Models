# Task 9 — Historical optimized-line replay and yaw-approximation audit

## Scope and decision

Task 9 was implemented and the expressly permitted **historical exploratory** 40 m optimized-line plans were replayed against matched historical fixed 2 m plans. Task 7 remains spatially inconclusive, so no new geometry is selected, no Task 7/8-dependent final matrix was run, and no optimizer/geometry convergence or transient optimality claim is made. Both replay pairs completed full solver validation and completed a lap, but sampled wheel-contact checks fail and whole-footprint/between-step containment are not certified. The Task 9 reliability gate therefore does **not** establish a reliable pipeline.

The legacy OCP and strict map/mask/tolerance semantics were not changed. Existing dirty unrelated work was left untouched.

## Saved inputs, provenance, and matched comparisons

Map identity for all plans: `f8658cafbd35dc60cd195153ca4761db21d9a9d8b4819f1112a4f46ad7322419`. The exact historical optimized plans are `runs/yaw_lap_suite/mis-line-40m-retained/plan.npz` and `runs/yaw_lap_suite/fsc-line-40m-grid2m/plan.npz`; matched fixed comparators are 2 m: `runs/yaw_lap_suite/mis-fixed-2m/plan.npz` and `runs/yaw_lap_suite/fsc-fixed-2m/plan.npz`. Each plan was loaded and strictly revalidated by replay against map/track identity. Replay used `run_with_deadline.py --seconds 1500`, duration 150 s, cooperative 1200 s, BDF, rtol `1e-6`, atol `1e-8`, max step `0.05`, and model-aware controller. The initial-state policy is the plant initialized from the bound plan’s first speed; resulting numerical initial states differ by plan. Historical plan-time package, thread/source provenance remains unavailable rather than inferred.

| Track | Plan/controller | Planned s | Driven s | Speed RMS/max abs m/s | Lateral RMS/max abs m | Realized throttle saturation fraction | Solver validation | Center / footprint / between-step |
|---|---|---:|---:|---:|---:|---:|---|---|
| MIS | optimized / model-aware | 60.58923 | 61.33835 | .31929/.81770 | .06345/.16293 | .18682 | complete | contained / unknown (wheel contact fail) / false |
| MIS | fixed 2 m / model-aware | 61.47060 | 62.19915 | .30429/.84442 | .06003/.15482 | .15214 | complete | contained / unknown (wheel contact fail) / false |
| FSC | optimized / model-aware | 51.16844 | 51.83038 | .32368/.91454 | .07510/.18244 | .15239 | complete | contained / unknown (wheel contact fail) / false |
| FSC | fixed 2 m / model-aware | 51.91682 | 52.57516 | .31188/.68035 | .07294/.17691 | .13800 | complete | contained / unknown (wheel contact fail) / false |

Optimized-minus-fixed driven-time deltas: MIS -0.86080 s (-1.38%); FSC -0.74478 s (-1.42%). This is an exploratory measured replay trade-off only. Optimized speed/lateral RMS errors are modestly higher in both pairs; maximum speed error is lower for MIS and higher for FSC. Requested and realized commands are distinct. Brake and steering-rate saturation are zero in replay summaries; full footprint clearance is not certified.

Wheel contact sample minimum clearance is MIS optimized -14.307 m at 1.729 s, MIS fixed -14.192 m at 1.766 s; FSC optimized -2.411 m at 19.898 s, FSC fixed -2.039 m at 20.086 s. Edge identity is nearest-left/right with assumed/conditional edge provenance. Whole-body envelope inputs are unavailable and between-step containment is false.

## Integration sensitivity

MIS optimized/fixed pair was repeated at max step `.025`; all completed and solver validated. Driven time and lateral RMS/max were unchanged at printed precision (optimized 61.3383533 s, .0634499/.162932 m; fixed 62.1991532 s, .0600300/.154821 m), within the 0.1% / 0.005 m / 0.01 m limits. FSC repeat was not run; its exploratory change is not claimed insensitive. No Task 7/8 final matrix was attempted.

## Yaw approximation: corrected method

The first review identified an invalid assumption in the original diagnostic: `acceleration-longitudinal` is body-axis acceleration, not tangential acceleration. The corrected tool computes scalar `a_t=(v_x*a_x+v_y*a_y)/V` from recorded body-frame velocity and both body accelerations, with `V=hypot(v_x,v_y)`. It also reports body yaw-rate residual `r-Vκ` separately from course-rate residual `r+beta_dot-Vκ`, with `beta=atan2(v_y,v_x)`. Curvature is mapped to monotonically **unwrapped** physical station; periodic seam neighbors are computed with a nonuniform three-point finite difference using the actual lap length and adjacent physical-distance spacings. The main trace is uniformly resampled in time at 0.01 s. Filter is periodic 11-sample boxcar (0.11 s). Residual stats are unweighted sample RMS, absolute P95 and absolute maximum over that resampled trace. Results are kinematic approximations; no residual by itself establishes map invalidity.

The analytic zero-sideslip constant-radius test gives zero residual. The sinusoidal-beta regression explicitly checks `beta_dot` and analytic yaw acceleration (`r=Vκ-beta_dot`, `r_dot=-beta_ddot`, constant V/κ, zero `a_t`), with second-order centered derivative tolerance. Nonuniform physical mesh regression checks periodic derivative. These tests avoid assuming body longitudinal acceleration equals tangential acceleration.

### Residual summaries (raw and filtered)

Format per cell: `RMS / abs-P95 / abs-max`; units for rate are rad/s, acceleration rad/s².

| Track/plan | Body `r−Vκ`, raw; filtered | Course `r+β̇−Vκ`, raw; filtered | Yaw acceleration minus `a_tκ+V²κ′`, raw; filtered |
|---|---|---|---|
| MIS optimized | .09988/.21087/.45948; .09650/.20372/.43882 | .10487/.17147/3.91264; .08027/.16649/.69104 | .88105/1.84072/14.84539; .76480/1.65433/3.66468 |
| MIS fixed 2 m | .10147/.22572/.48116; .09817/.22018/.46211 | .10001/.17699/3.45233; .07984/.17093/.61346 | .87781/1.86972/15.31929; .76360/1.64362/3.50228 |
| FSC optimized | .10424/.18768/.95675; .09368/.18069/.67428 | .09306/.16834/1.05313; .08090/.15439/.68327 | 1.65843/2.18970/22.68457; 1.19926/1.79666/13.52028 |
| FSC fixed 2 m | .10332/.19271/.95337; .09295/.17966/.67158 | .09324/.17146/1.03431; .08157/.16140/.68524 | 1.63143/2.20142/22.51639; 1.18719/1.84887/13.61977 |

Filtered values use the periodic 11-sample `.11 s` boxcar described above. Full evidence preserves the same fields per run in `runs/yaw_lap_suite/closure-v1/comparison-report.json`.

## Available interval and force/moment evidence; limits

Structured diagnostics now list (i) throttle/brake saturation intervals where the necessary realized channels and a meaningful bound are present, (ii) intervals with `|r+beta_dot−Vκ| > 0.05 rad/s` lasting at least 0.1 s, an explicitly exploratory marker (not an acceptance threshold), and (iii) curvature sign-reversal event times derived from the plan. The payload states that no pointwise reference-tracking error channel is stored in the run record, so temporal correlation with tracking-failure intervals cannot be computed; aggregate tracking metrics remain in the row. It explicitly lists steering-rate-limit attribution as unavailable because the maximum-rate bound is not recorded in those replay inputs/channels. It also states continuous envelope feasibility is not certified. Thus interval lists are descriptive, not claims that every saturation arose on a continuously feasible envelope segment.

Tire force X/Y and aligning-moment channels plus wheel body positions are present. The tool computes and reports the instantaneous signed tire-generated yaw-moment sum `Σ(x_i F_yi − y_i F_xi + M_zi)` and its RMS as an available partial consistency quantity. It does **not** call that dynamics closure: yaw inertia `I_z` is absent from run record channel/vehicle geometry payload, so the required comparison to `I_z*r_dot` and remaining applied moments cannot be made. Structured reasons enumerate this missing term. No force-balance or map-invalidity conclusion is made.

## Identity/status gates and artifacts

The comparison tool now binds case track to each plan’s recorded track, checks plan file and array digests from replay provenance, confirms replay plan path, map identity, track identity and source revision. Any missing/mismatched identity yields `comparison_status: inconclusive`; `exploratory_completion` remains separately true for an actually completed, solver-validated lap. It does not conflate completion with pairing success or containment. The corrected JSON has identity status `pass` for both pairs; historical unknown fields remain explicit.

The report JSON also provides detailed actuator/residual interval lists, curvature-reversal events, moment evidence, filter metadata and raw/filtered statistics. The ignored worktree report is this file; source/test/report paths were committed, generated replay outputs remain local evidence.

## Review correction evidence (round 1)

Review findings were addressed on top of commit `5301ae3` without new physical runs. Existing immutable run records were reused to regenerate the comparison JSON. Changes: project-body acceleration onto velocity for `a_t`; distinguish `r−Vκ` from `r+beta_dot−Vκ`; unwrap seam-crossing station and pass actual lap period; use nonuniform periodic three-point curvature differentiation; add filtered residual table for all cases; expose throttle/brake intervals, large sideslip-rate residual intervals, curvature reversal timestamps and explicit unavailable correlation/limit fields; compute the recorded tire yaw-moment sum while withholding full dynamics closure; enforce plan/map/track/revision/hash identities and separate exploratory completion from comparison status; extend analytic sinusoidal-beta, nonuniform periodic derivative and mismatch regressions. The previous `project_xy_to_reference` formatting was restored to avoid unrelated nonfunctional churn.

Focused validation command, enforced by the process watchdog:

```text
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 180 -- python3 -m unittest tests.test_yaw_planning_replay tests.test_yaw_replay_binding tests.test_yaw_replay_metrics tests.test_yaw_lap_suite tests.test_yaw_lap_artifact tests.test_yaw_aware_record_path -q
Ran 29 tests in 14.395s — OK
```

No physical replay was repeated and no timeout/failure was retried. Task 7 remains spatially inconclusive; no geometry/optimizer convergence, transient optimality, or reliable-pipeline claim follows.
