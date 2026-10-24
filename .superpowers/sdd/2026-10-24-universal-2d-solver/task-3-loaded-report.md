# Task 3 loaded-domain integration report

## Scope and status

Implemented a reviewed-scope loaded preflight in the no-TV diagnostic worker. Explicit `track_problem_contract` request members are strictly validated before filesystem admission, bound to actual loaded `ReferenceCurve`, corridor, capability chart, mesh and QSS start settings, and rejected before spline NLP setup when malformed, mismatched, or unsupported. Requests without the field remain explicitly labeled `legacy_diagnostic` and keep their historical route. This does **not** complete all Task 3 public transport: `build_attempt` / public request builders do not yet expose a user-facing contract option, so callers must supply the optional member through a structured worker request. Manifest identity wiring is present, but no newly generated run was used to round-trip a `track_problem` manifest document. No optimizer boundary changes, NPZ changes, production thresholds, source/cache changes, or solver/NLP run were made.

The actual QSS boundary limitation was honored: closed requests are rejected with `requested_boundary_not_enforced` because current closed solving only enforces periodic speed; open rolling below `max(surface.speed_floor, settings.minimum_speed)` is rejected with `start_speed_below_model_floor` rather than silently clamped. Open rolling above that floor is only admitted when the exact requested setting agrees, free finish is used, and loaded checks pass. Other pure-unsupported modes remain unsupported.

## Interfaces and persisted identity

- Added `preflight_loaded_problem(validation, *, track, reference, corridor, surface, settings, mesh, constraint_sites)` in `src/simplified_vehicle_models/solvers/loaded_track_problem.py`.
- Optional request field is `track_problem_contract` (existing pure schema `track_problem_v1`). Unknown/malformed contents fail the existing strict worker request gate. Absent member yields `legacy_diagnostic`.
- Worker result `track_problem` records status, normalized contract/hash, boundary enforcement status, corridor policy/source-station identity, reference fit diagnostics, sampled domain and explicit sampling limitations.
- The run manifest effective-request identity and `boundary_event_contract` include `track_problem`; the NPZ field set remains unchanged.
- Refinement comparison recomputes the canonical pure hash for both request contracts and requires both recorded hashes to match it. Legacy and explicit identities do not compare as equivalent.
- Runtime provenance inventory includes both new loaded/pure modules.

## Checks and limits

Loaded checks use the actual `ReferenceCurve` and loaded corridor grid; require exact source-grid and corridor-array agreement; check finite ordered corridor, topology, loaded source domain, mesh/sites, finite reference evaluations at source/request/query sites, and sampled reference curvature/chart domain. Signed charts are queried through physical `domain_speed(curvature)` and `speed_floor`; legacy charts use `speed_ceiling` on the curvature grid or their global speed axis.

These are sampled diagnostics, not continuous certificates. The curvature check explicitly covers sampled **reference** curvature only, not all offset paths. No unsampled cusp, continuous regularity, source-fit fidelity qualification, heading/fold/rate bound, vehicle-footprint containment, or physical feasibility claim is made. Figure-eight-like ordered routes are not blanket rejected. `CORRIDOR_POLICY` fallback is `legacy-abs-source-curvature-v1`; actual corridor provenance and station grid digest/range are recorded.

## Exact runtime and test outcomes

All commands used pinned Python 3.13.13 from the required runtime:
`PYTHONPATH=src runs/no_tv_reliability/runtime_install_20261023/venv/bin/python`

Focused RED: before implementation, `python -m unittest discover -s tests -p 'test_loaded_track_problem_preflight.py' -q` failed import with `ModuleNotFoundError` for missing `loaded_track_problem` (expected RED).

Per-module focused checks had these results (all exit 0):
- `... -m unittest discover -s tests -p 'test_track_problem_contract.py' -q` — 7 tests OK.
- `... -m unittest discover -s tests -p 'test_loaded_track_problem_preflight.py' -q` — 6 tests OK (real `ReferenceCurve`, signed chart API, open supported, below-floor and closed unenforced rejections, source grid mismatch, legacy tag).
- `... -m unittest discover -s tests -p 'test_run_manifest.py' -q` — 13 tests OK.
- `... -m unittest discover -s tests -p 'test_trajectory_artifact.py' -q` — 7 tests OK; exact NPZ schema regression.
- `... -m unittest discover -s tests -p 'test_minimum_time_run.py' -q` — 30 tests OK.
- `... -m unittest discover -s tests -p 'test_no_tv_minimum_lap.py' -q` — 48 tests OK, including early worker rejection, original schema-version request test, typed `not_run`, and canonical comparison-hash mismatch/forgery.
- `... -m unittest discover -s tests -p 'test_input_preflight.py' -q` — 3 tests OK.
- `git diff --check` — exit 0.

The exact combined covering command was:
`PYTHONPATH=src runs/no_tv_reliability/runtime_install_20261023/venv/bin/python -m unittest tests.test_run_manifest tests.test_input_preflight tests.test_no_tv_minimum_lap tests.test_minimum_time_run tests.test_trajectory_artifact tests.test_minimum_time_line_gate tests.test_tv_runner tests.test_track_problem_contract tests.test_loaded_track_problem_preflight -q`
It passed **192 tests** in 28.238 seconds, exit 0. Expected fixture/pipeline messages included; final result was `OK`. `git diff --check` passed. An earlier exploratory command used nonexistent glob `test_request_*.py`, reported 0 tests and exit 5; it was superseded by the exact command above. No full suite or expensive optimizer run was made.

## Commit and freeze

Commit: `7d0751aad3272ae2fec552d1abef38e72e117b01` (`Integrate loaded track problem preflight`). It contains only the five owned code/test paths and this report. Existing user/controller dirty docs, unrelated reports, root layout artifacts, and `x` remain untouched and unstaged.
