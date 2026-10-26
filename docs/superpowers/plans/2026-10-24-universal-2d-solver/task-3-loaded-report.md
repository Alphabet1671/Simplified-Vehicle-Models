# Task 3 loaded-domain integration report

## Scope and status

Implemented loaded preflight in the no-TV diagnostic worker. Explicit `track_problem_contract` request members are validated before filesystem admission, bound to loaded `ReferenceCurve`, corridor, capability chart, mesh, and QSS settings, and rejected before spline NLP setup when malformed, mismatched, or unsupported. Requests without the field remain explicitly labeled `legacy_diagnostic` and keep their historical route.

This is a partial Task 3 integration: `build_attempt` / public request builders do not expose a user-facing contract option; callers currently supply the optional member through a structured worker request. Manifest identity wiring is present, but no newly generated run round-tripped a `track_problem` manifest document. No optimizer boundary changes, NPZ changes, production thresholds, source/cache changes, or solver/NLP run were made.

Closed explicit contracts are rejected with `requested_boundary_not_enforced` because the current closed solver enforces only periodic speed. Open rolling below `max(surface.speed_floor, settings.minimum_speed)` is rejected with `start_speed_below_model_floor`, not silently clamped. Open rolling above the floor is admitted only when the requested start agrees exactly with QSS settings, finish is free, and loaded checks pass. Other pure-unsupported modes remain unsupported.

## Interfaces and identity

- `preflight_loaded_problem(validation, *, track, reference, corridor, surface, settings, mesh, constraint_sites)` in `src/simplified_vehicle_models/solvers/loaded_track_problem.py`.
- Optional request field: `track_problem_contract` with schema `track_problem_v1`. Unknown/malformed values fail strict worker request validation. Absence yields `legacy_diagnostic`.
- Worker result stores normalized contract/hash, boundary status, corridor policy/provenance/station identity, reference-fit diagnostics, sampled domain, and sampling limitations.
- Manifest effective-request identity and `boundary_event_contract` include `track_problem`; NPZ schema is unchanged.
- Refinement comparison recomputes canonical hashes from both request contracts and requires recorded hashes to match. Legacy and explicit identities do not compare as equivalent.
- Runtime provenance inventory includes the pure and loaded contract modules.

## Checks and limits

Checks use the actual loaded `ReferenceCurve` and corridor grid; require exact source-grid and corridor-array agreement; check finite ordered corridor, topology, loaded source domain, mesh/sites, finite reference evaluations at queried sites, and sampled reference curvature/chart domain. Signed charts use physical `domain_speed(curvature)` and `speed_floor`; legacy charts use `speed_ceiling` or the global speed axis.

These are sampled diagnostics, not continuous certificates. Curvature checks cover sampled **reference** curvature only, not all offset paths. No unsampled cusp, continuous regularity, source-fit fidelity qualification, heading/fold/rate bound, vehicle-footprint containment, or physical feasibility claim is made. Ordered figure-eight-like routes are not blanket rejected. Corridor fallback policy is `legacy-abs-source-curvature-v1`; actual provenance and station-grid digest/range are recorded.

## Runtime and tests

Pinned runtime prefix: `PYTHONPATH=src runs/no_tv_reliability/runtime_install_20261023/venv/bin/python` (Python 3.13.13).

Focused RED before implementation: `python -m unittest discover -s tests -p 'test_loaded_track_problem_preflight.py' -q` failed import with `ModuleNotFoundError` for the absent module, as expected.

Per-module focused checks (all exit 0):
- `... -m unittest discover -s tests -p 'test_track_problem_contract.py' -q` — 7 tests.
- `... -m unittest discover -s tests -p 'test_loaded_track_problem_preflight.py' -q` — 6 tests, including actual `ReferenceCurve` and signed-chart API.
- `... -m unittest discover -s tests -p 'test_run_manifest.py' -q` — 13 tests.
- `... -m unittest discover -s tests -p 'test_trajectory_artifact.py' -q` — 7 tests.
- `... -m unittest discover -s tests -p 'test_minimum_time_run.py' -q` — 30 tests.
- `... -m unittest discover -s tests -p 'test_no_tv_minimum_lap.py' -q` — 48 tests, including worker early rejection, original schema-version priority, typed `not_run`, and comparison hash mismatch/forgery.
- `... -m unittest discover -s tests -p 'test_input_preflight.py' -q` — 3 tests.
- `git diff --check` — exit 0.

Exact combined covering command:
`PYTHONPATH=src runs/no_tv_reliability/runtime_install_20261023/venv/bin/python -m unittest tests.test_run_manifest tests.test_input_preflight tests.test_no_tv_minimum_lap tests.test_minimum_time_run tests.test_trajectory_artifact tests.test_minimum_time_line_gate tests.test_tv_runner tests.test_track_problem_contract tests.test_loaded_track_problem_preflight -q`

Result: 192 tests passed in 28.238 seconds, exit 0. After the invalid-contract classification fix, focused command `PYTHONPATH=src runs/no_tv_reliability/runtime_install_20261023/venv/bin/python -m unittest tests.test_no_tv_minimum_lap tests.test_track_problem_contract tests.test_loaded_track_problem_preflight -q` passed 61 tests in 0.623 seconds, exit 0. The direct worker regression passed 1 test in 0.008 seconds, exit 0. No full suite or expensive optimizer run was made.

## Commits and freeze

Implementation commit: `e9646fc2cb3ec79433e24633273e9364cadb8f7b` (`Integrate loaded track problem preflight`). It accidentally tracked the ignored working report as a sixth path; that path is being removed from the index without deleting its working file. The curated copy of this report is under `docs/superpowers/plans/2026-10-24-universal-2d-solver/`.

Existing user/controller dirty docs, unrelated reports, root layout artifacts, and `x` remain untouched and unstaged. Follow-up index correction is recorded in a separate commit; no history rewrite is used.
