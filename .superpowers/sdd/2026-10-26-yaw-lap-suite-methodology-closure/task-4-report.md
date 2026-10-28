# Task 4 — Fine-grid connectivity cost and diagnostics

Date: 2026-10-26. Scope is the coupled yaw-profile transition oracle, connectivity refinement diagnostics, strict batched graph edges, and reproducible tests. Task 5 forward/runner changes and Task 8 artifacts are not included in this report or intended commit.

## Result

The Task 4 implementation hardens interval-cache behavior and bounded pair processing, adds diagnostics for interval and graph work, and freezes a representative MIS 0.05 m transition window for strict scalar/batched equivalence. The selected narrow-window fixture has an actual connecting witness under the strict oracle. Masked-hole tests continue to require disconnected components; the algorithm does not bridge unsupported gaps.

**Unit/transition/coupled suite: PASS — 40 tests in 2.910 s**, protected by a 180 s process-group watchdog. **Physical-window regression suite: PASS — 2 tests in 3.296 s**, protected by a 600 s process-group watchdog. This proves the frozen 10 m-ish physical windows remain strictly valid under current changes; it does not establish whole-lap fine-grid resolution convergence or Task 7 spatial success.

## Implementation and diagnostics

- `interval_cache` is initialized before early paths and keyed with exact binary fixed-q and bounds representations plus path segment geometry, capability identity/content, yaw inertia, and settings. No q rounding is used. Results are only cached after successful completion; a deadline-truncated interval computation cannot be reused as a completed answer.
- Early typed diagnostic failures carry the required keys with neutral values. Normal and failed-after-work diagnostics report interval calls/cache hits, strict probes, pair count/peak batch size, frontier-by-round records, candidate cap hits, deferred requests, and stage timings including interval construction.
- Strict transition interval instrumentation checks the deadline through cell/root witness iteration; interruption diagnostics survive. Candidate-edge batches are flattened row-major, chunked at no more than 65,536 pairs, and preserve deterministic ordering including the final partial chunk. Unit coverage compares production batched feasibility and scalar strict evidence on random/exact-face sets, and compares winners under default vs smaller batching.
- The new physical fixture test uses the actual failing segment from the saved MIS raw-centerline 0.05 m window: segment 10, length 0.04999757312957953 m, endpoint curvature samples approximately -0.11853366/-0.11865297 1/m and rate approximately 0.00287788/-0.00239930 1/m². It records exact fixed endpoint q=36.693167236897324, map source identity `d6364141325f26541192852ad08c6f9f6668661a73527ae709799f1cb96bf30f`, and SHA-256 over capability axes/mask/moment arrays `6beca2ba9deb9c88fe47894c8a23fb57b761a5b19e11b4c2517e8cc4cd21fb49`. Its strict interval contains [36.768163596591705, 36.9931526756748], with strict-feasible lower, midpoint and upper evidence; 41 pairwise batched probes exactly match scalar strict decisions.
- Existing transition tests exercise narrow feasible regions, masked support slits, repeated/tangent roots, exact map faces, endpoint q/local-coordinate rounding, randomized strict-oracle agreement, and matrix/scalar equivalence.

## Physical-study evidence and limitations

The physical-window regression tests are frozen small windows; they do not compare full-lap speed/time winners. The historical MIS 1 m artifact is a profile-converged plan but not proof of spatial convergence. The Task 3 MIS 0.5 m sequence records first a strict polynomial-consistency failure, then—after local-coordinate polynomial construction—a separate 300 s frontier-refinement budget exhaustion without a validated path. These observations remain distinct; no identical 0.5 m run was repeated here.

Fresh MIS fixed-geometry physical runs were executed with identical track and map identities. The 10 m run completed in 4.55 s with 103 stations, lap time 52.2377944231 s, min/max speed 12.8721631/27.4624065 m/s, and `resolution_converged=true`; strict validation passed (minimum yaw margin 2.00001226). The 0.25 m run completed under a 900 s process watchdog in 599.60 s with 4,068 stations, lap time 63.4161785271 s, min/max speed 9.1230410/25.9371608 m/s, and `resolution_converged=false`; strict validation passed (max yaw moment violation 9.97e-10, recomputed time matched). Its diagnostics recorded 31,941 interval calls, 919 exact cache hits, 30,433,251 strict probes, 5,385,500 cells, 244,022,058 pairs, and peak batch 17,030. Timings: graph build 52.737 s, solve 9.233 s, interval work 510.607 s, refinement 535.311 s, validation .650 s, total 599.601 s. Thus the 10 m and .25 m measured times differ by +21.399% (11.1784 s); at shared coarse stations the maximum absolute speed difference is 11.9041 m/s. This is evidence of substantial resolution sensitivity, not convergence or same-winner/time equality. The fresh output directories are `runs/yaw_lap_suite/task4-mis-fixed-10m/` and `runs/yaw_lap_suite/task4-mis-fixed-0p25m-final-chunked/`; the latter uses the corrected bounded row-major chunking and shares array digest `c1bd127278e3fc3e32e1e2b62b9aaf9181976cdc4172fd43bea55ee201a2edcf` with the earlier 0.25 m result.

The 1 m artifact is historical (180 s/8 rounds, not a fresh Task 4 capture). No identical 0.5 m run was repeated. Earlier Task 3 0.5 m artifacts remain distinct: `mis-fixed-0p5m` ended with polynomial-vs-strict disagreement (segment 153, cell [18,25,286], query 12.8724009129, `interval_oracle_unresolved`); `mis-fixed-0p5m-local` was a separate 300 s wall deadline after local-coordinate polynomial construction, without a validated path.

Accordingly the **Task 4 Gate passes for the selected narrow-window fixture and scalar/batched decision agreement**. **Full physical fine-grid connectivity remains separately unresolved**: no valid 0.5 m lap is demonstrated, and the 0.25 m evidence is unresolved in resolution convergence. Task 7 spatial success remains open.

## Validation commands

- `PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 180 -- python3 -m unittest tests.test_yaw_profile_connectivity tests.test_yaw_profile_transition tests.test_coupled_yaw_profile -q` — 44 passed, 3.042 s.
- `PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 600 -- python3 -m unittest tests.test_coupled_yaw_physical_windows -q` — 2 passed, 3.321 s.
- Fresh 0.25 m physical profile: `PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 900 -- python3 tools/run/run_yaw_lap_suite.py --mode fixed --track models/tracks/mis_ev_endurance_2024_width.xml --yaw-map runs/yaw_map_build/iem24-vehicle-limit-final.npz --spacing 0.25 --control-spacing 40 --profile-rounds 16 --profile-seconds 900 --output-dir runs/yaw_lap_suite/task4-mis-fixed-0p25m-final-chunked` — completed with valid incumbent in 599.60 s, resolution convergence false.
- Fresh 10 m fixture: same command with `--spacing 10 --profile-seconds 300 --output-dir runs/yaw_lap_suite/task4-mis-fixed-10m` — completed in 4.55 s, valid incumbent and resolution convergence true.

Historical 1 m and 0.5 m diagnostic recaptures are not claimed. The prior artifacts and the above distinctions are preserved honestly rather than duplicating costly physical runs.

## Review correction evidence (2026-10-26)

- Deadline polling now also covers post-cell strict root/witness checks, cut-boundary queries, endpoint nextafter correction, fallback probes, endpoint bisection, and overlap checks. Expiration marks `deadline_stop`; the interval oracle discards partial results, causing the caller to raise its wall-deadline interruption before cache insertion. A fake-clock regression targets post-cell endpoint processing and verifies zero partial intervals.
- The narrow MIS fixture now freezes the exact strict component `[36.768163596591705, 36.9931526756748]`, its single connected component, the multi-support-cell union (`support_id=()`), and strict-feasible lower/mid/upper points. It also retains 41 pairwise candidate edge facts matching scalar strict decisions and nonzero masked-map behavior. This is a strict interval regression, not a full solver-frontier physical success claim.
- Saved artifacts do not contain a complete requested 1 m or 0.5 m diagnostic capture. In particular, the 1 m plan lacks first-cut/reachable/co-reachable/support-reason/component/stage fields. `mis-fixed-0p5m` preserves the original strict polynomial consistency failure (segment 153, cell `[18,25,286]`, discrepancy `0.00011132663689750188`). `mis-fixed-0p5m-local` and warm variants preserve separate budget failures and some refinement cut histories, but lack full frontier candidate sets, rejected support reasons, components and interval-vs-graph timing. No identical 0.5 m run was repeated.

| Requested saved diagnostic | Artifact evidence | Conclusion |
|---|---|---|
| MIS 1 m: first cut/reachable/co-reachable sets/support reasons/components/stage times | Historical saved plan omits required schema | **Inconclusive — missing historical diagnostic fields** |
| MIS 0.5 m: same fields | Original polynomial-failure artifact; separate local-polynomial 300 s budget artifacts have partial cut history only | **Inconclusive — missing historical diagnostic fields** |
| Post-chunk MIS 0.25 m (saved artifact only) | `runs/yaw_lap_suite/task4-mis-fixed-0p25m-final-chunked/summary.json`; strict-valid, `resolution_converged=false`, 599.601 s runtime, digest `c1bd127278e3fc3e32e1e2b62b9aaf9181976cdc4172fd43bea55ee201a2edcf`, interval 510.607 s, graph build 52.737 s, graph solve 9.233 s | Saved unresolved result, not a convergence/success claim |

The 0.25 m values above are from its saved post-chunk artifact and do not replace the distinct historical polynomial-failure and frontier-budget observations.

- Review regression: `PYTHONPATH=src python3 -m unittest tests.test_yaw_profile_connectivity -q` — 10 passed. Full suite/watchdog reruns are listed below.
