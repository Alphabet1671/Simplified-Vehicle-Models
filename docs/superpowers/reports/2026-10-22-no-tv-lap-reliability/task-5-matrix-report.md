# Task 5 bounded no-TV matrix harness report

**Date:** 2026-10-22  
**Branch:** `feature/minimum-time-line-2d-track`  
**Base requested:** `6286c5fbb02217fc21148fdc0be276728f13b99c`  
**Actual HEAD observed before implementation:** `6286c5fbb02217fc21148fdc0be276728f13b99c`

## Delivered scope

- [`verify_no_tv_minimum_lap.py`](../../../tools/diagnostics/verify_no_tv_minimum_lap.py)
- [`test_no_tv_minimum_lap.py`](../../../tests/test_no_tv_minimum_lap.py)
- [`no-tv-minimum-lap-reliability.md`](../../../docs/dev/no-tv-minimum-lap-reliability.md)

No signed-capability source/cache builder, QSS solver/settings/control tests, or
runner file was changed.

## Harness APIs

- `default_cases()` returns six preregistered no-TV cases.
- `make_track_view(case)` retains original geometry/samples/width and creates
  closed or full cut-open views in memory.
- `build_attempt(...)` serializes exact caps, start state, mesh/basis rung, seed,
  source paths, and no-TV flags.
- `load_surface_dispatch(path)` explicitly loads either legacy or signed capability
  representation; it never substitutes one for the other.
- `run_bounded_worker(request, output)` writes a request, starts a spawn-safe
  filesystem worker, terminates and joins on wall timeout, retains any child result,
  and writes a separate authoritative timeout result.
- `run_matrix(args)` saves after every attempt and recursively hashes sources/data
  dependencies before/after. Each attempt now owns a fresh pre-spawn source snapshot
  and a completion snapshot; any in-attempt mutation makes that attempt candidate-only.
- `aggregate(...)` separates unqualified candidate comparisons from qualified
  same-seed/settings/source, one-axis mesh/basis evidence, and leaves strict
  acceptance incomplete without mesh, basis, matched-baseline, and replay evidence.

## Matrix declaration

MIS and FSC each use original closed and same-full-geometry cut-open views. Closed
cases are `closed_flying`; cut-open, skidpad, and acceleration are `open_rolling_10`.
Skidpad and acceleration are explicitly `NOTstandingstart` candidate diagnostics.
The default preregistered rungs are mesh `5, 2.5, 1.25 m`, offset basis `35, 17.5 m`,
and deterministic centerline/low-amplitude offset seeds. The default iteration cap
is 200 and wall cap 300 s per attempt. The default matrix was not run.

## Executed verification

```bash
PYTHONPATH=src python3 -m py_compile tools/diagnostics/verify_no_tv_minimum_lap.py
PYTHONPATH=src python3 -m unittest tests.test_no_tv_minimum_lap -v
```

The focused test module verifies construction, starts, caps/rungs, finite JSON,
exclusive output, timeout termination/join while preserving a partial child result,
missing-worker result persistence, recursive XML source/data hashes and missing-
dependency rejection, qualified one-axis refinement separation, and fail-closed
strict status. It does not execute a full matrix or claim solver/event acceptance.

## Source-snapshot qualification fix evidence (review follow-up)

The harness now creates `request["source_hashes"]` immediately before every spawned
attempt and records `result["source_hashes_after"]` immediately after it completes.
A qualified comparison requires matching unchanged snapshots for both rows; attempted
solutions and candidate RMS diagnostics are still retained when that requirement
fails. The matrix-level initial/final source mismatch also disables every qualified
mesh/basis refinement comparison, including rows whose individual snapshots match.

```bash
PYTHONPATH=src python3 -m unittest tests.test_no_tv_minimum_lap -v
PYTHONPATH=src python3 -m py_compile tools/diagnostics/verify_no_tv_minimum_lap.py
```

**Result:** 14 focused tests passed. Regression coverage demonstrates source mutation
between attempts selects a fresh request snapshot and invalidates aggregate refinement;
mutation during an attempt rejects only qualified refinement while retaining candidate
diagnostics; identical unchanged sources retain qualified mesh and basis one-axis
comparisons. No full solver matrix was run.

## Current measured evidence and limits

The default signed cache is represented as `signed_transient_normalized_v1` and is
still marked under provenance review. Existing
[`signed_accel_refinement_20261022.json`](../../../runs/no_tv_reliability/signed_accel_refinement_20261022.json)
records rolling-10 acceleration time `4.508892776162197 s` at 2.5 m mesh/35 m basis,
with `Solve_Succeeded` but failed dense microconstraints/dynamics. It is candidate
only. The reported signed-cap-300 skidpad candidate near `21.5696 s` also has failed
live/dense checks and remains candidate only.

Strict aggregate is always `incomplete` unless independently supplied mesh and basis
refinement, matched baseline, and matched exact-trajectory replay exist. In addition,
there is currently **no approved curated, source-hashed geometry/state convergence
policy or supporting data**. The six `--convergence-criteria-json` budgets are
persisted as nonqualifying user diagnostics only: even arbitrarily large budgets
cannot set strict geometry/state convergence or make an edge/study accepted. The
established 0.5% reference-time and 1% replay limits are unchanged. The harness
does not provide a braking-feasible no-TV baseline or standing-start launch
accounting, so neither real event acceptance nor convergence from absent rungs is
claimed.

## Reproduction command (bounded smoke only)

```bash
PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m \
  tools.diagnostics.verify_no_tv_minimum_lap \
  --cases acceleration --mesh-spacings 5 --basis-spacings 35 \
  --seeds centerline --maximum-iterations 20 --wall-cap-s 120 \
  --output runs/no_tv_reliability/task5-accel-smoke
```
