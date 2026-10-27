# Task 12 implementation report — controlled comparisons and attribution

## Outcome

Task 12 was implemented as a report-only evidence consumer. Its gate remains open because the required matched controlled treatments are not run/inconclusive; that is the evidence-supported outcome, not a claim of causal closure. No vehicle physics, map build, replay, or optimization was run by the report generator. The new standard-form report output is `docs/validation/iem24-murphy-yaw-closure/`; the historical `docs/validation/iem24-murphy-yaw-replay/report.md` was not edited.

The saved replay and measured cache reproduce the retained historical values: 64.80223171819523 s replay, 71.68501281738281 s median flying-lap duration, 6.88278109918758 s difference (9.60% of the measured median). Lap 0 remains in the lap table as a standing start and excluded; the ten flying laps are the statistic set. Median is computed from lap durations, not by integrating the pointwise-median speed trace. Reproduction consumed saved inputs and completed under the report command's process-level 300 s watchdog.

## Identity and causal-status finding

The record/plan pair used by the historical report can be structurally checked for record-declared completion, time monotonicity/finiteness, channel declarations/validity, and exact yaw-plan array/track/map identities. However, record provenance does not carry a current vehicle configuration-source hash or the plan solver/settings in the record manifest; those are explicitly unknown rather than reconstructed. The historical source/Task9 paired comparisons are identity-inconclusive, so no controlled causal attribution is supportable. The fresh report's ablation table reports controller, line, battery, tire-force scale, and alignment comparisons as not run/inconclusive; held-fixed identities and outcome metrics are not fabricated. Interactions remain unresolved and one-factor shifts are not summed.

## Code and tests

- Replaced the production assertion for one fixed lap duration with structural validation and an explicit `lap_completed=true` admission requirement.
- Added a completed second-duration synthetic fixture, incomplete-lap rejection, identity mismatch rejection, and request-versus-delivered-torque distinction test in `tests/test_yaw_mis_sensitivity.py`.
- Added a configurable reporting torque limit; reconstructed request is `realized throttle × limit` while delivered signed wheel torque remains a separately recorded channel.
- Kept six-channel metric outputs and the standard's three embedded figures. The new report contains an explicit not-run ablation table and provenance/measurement caveats.
- Added cumulative time versus reference-station utility and 50 m sector calculation with a final remainder. The mapping is explicitly described as physical-station-to-reference-station; `1/V` is not called `dt/ds_reference`.

Focused command (all commands guarded by the repository watchdog):

```bash
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 180 -- \
  python3 -m unittest tests.test_yaw_mis_report tests.test_yaw_mis_sensitivity \
  tests.test_mis_report_standard -v
```

Result: 17 tests passed. The standard conformance test verified all `docs/validation/**/report.md` reports, required sections, six metrics, embedded figure files, and link back to the standard.

## Remaining scope / limitations

No fresh controlled physics treatment was run. Identity-bound input pairs suitable for the legacy/final-controller, fixed/optimized-line, and grip/map/replan experiments were not present; battery OCV inference lacked the declared current/state model input; geometry/provenance evidence did not support alignment sensitivity as an attributable correction. The report states these conditions without relabeling unknown historical provenance as known. A complete attribution gate therefore remains open; Task 12's requested causal conclusion is explicitly inconclusive rather than claimed complete.

The generator retains duplicate reference-station samples by keeping the last corresponding physical time. For measured data, projected station is unwrapped independently within the selected flying lap before mapping onto the saved plan's reference axis. The regenerated output now contains 21 numeric 50 m sectors plus the final remainder, with largest positive and negative signed replay-minus-measured gaps rendered in the report; this is descriptive timing evidence, not causal attribution. Brake-onset/exit-speed and yaw/acceleration residual comparisons remain structured unavailable because the cache lacks measured brake-onset channels and historical alignment/geometry provenance does not support residual attribution. Operational traces keep requested torque, delivered wheel torque, bus power/limit, voltage/SOC, throttle/brake/steering values and saturation evidence separate; missing power-limit/saturation flags remain explicitly unavailable. The controlled causal treatments remain not-run/inconclusive.

Correction evidence: report prose is now parameterized for the selected replay duration and measured median rather than quoting a hardcoded historical gap. The sector table is computed using unwrapped measured station, preserving sampled lap elapsed timing; the final 16.7 m remainder is included. Focused tests were rerun after these corrections: 21 passed, including standard conformance.
