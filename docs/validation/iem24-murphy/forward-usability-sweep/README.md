# Forward steering sweep

Candidate gains for the live-planner steering controller (`--preview-seconds`, `--heading-gain`, `--lateral-gain`). Each candidate is one canonical cold run: live planner, `--maximum-speed 60.0`, `--pedal-travel-time 0.175`, BDF, rtol 1e-5, spacing 0.5 m, duration 300 s, step 0.02 s (7DoF) or 0.005 s (14DoF). Produced by `tools/run/sweep_forward_steering.py`; each directory has a `steering_sweep.json` with every candidate's recorded parameters, outcome and `canonical_settings_recorded`.

The results only choose controller gains. They are diagnostic: no acceptance threshold was changed, and a candidate is not evidence of usability until it passes the full chain (`acceptance_gate`).

## Directories

| Directory | Contents |
|---|---|
| `planar/` | The first, non-canonical attempt (preview 0.10 s). It is retained for provenance only. It ran with unbounded settings and ended `Timed_Out` at the 295 s solver budget. Its summary records `planner=conservative` although the wrapper asked for `live`, so it is not a tuning result. |
| `planar_canonical/` | Two rejected 7DoF candidates from the rewritten wrapper (preview 0.10 s). |
| `planar_canonical_b/` | The 7DoF sweep over preview 0.15/0.20, heading gain 1.5/2.5 and lateral gain 1.5/3.0 (8 candidates; the last in-flight one was interrupted and its partial directory removed). |
| `vertical_canonical/` | The 14DoF sweep, preview 0.20/0.25, heading gain 2.5/3.5, lateral gain 1.5/3.0. |

## What the 7DoF sweep showed

- A lateral gain of 3.0 with preview 0.10 or 0.15 s makes the integrator fail ("Required step size is less than spacing between numbers", status `Rejected`, about 50 s, in the S-bend near s = 775 m). With preview 0.20 s it completes.
- Max lateral error falls as preview and heading gain rise. With lateral gain 1.5: 0.334 m (0.15 s, heading 1.5), 0.269 m (0.15 s, 2.5), 0.256 m (0.20 s, 1.5), **0.181 m (0.20 s, heading 2.5)**. The gate limit is 0.25 m.
- Chosen 7DoF gains: preview 0.20 s, heading gain 2.5, lateral gain 1.5.

## Correction of an earlier explanation

An earlier version of this file said the first run was interrupted because a second lap started past the nominal lap distance. That was wrong. The first run used non-canonical settings and simply did not finish inside the runner's 295 s budget. Later canonical runs complete one lap in about 100 s with no second lap, so no "single-lap orchestration" was needed.
