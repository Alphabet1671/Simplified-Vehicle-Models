# Task 1B Canonical Manifest Implementation Report

**Status:** Manifest implementation is committed and tested, pending independent review. Implementation commit: `a4ae706932dc0b8ca73a9936e134f97ad7fdce62`. This report does not claim full Task 1B/input-preflight acceptance or physical qualification.

## Implementation

- Added `run_manifest_v1` construction and strict loading. It binds `trajectory_metadata.npz` (the loader-compatible trajectory, not the harness node-array `solution.npz`), input/evidence SHA-256 identities, runtime and implementation provenance, boundary-event contract, and explicitly unqualified policy.
- Worker-owned finalization emits manifests within the supervised worker. Failures are recorded as `manifest_unavailable`, keep acceptance false, and do not turn solver success into qualification.
- Input snapshots copy bytes and preserve source-tree relative layout. The dependency closure follows the repository XML model loader's reference graph plus track XML data references, allowing the copied track to load and the copied formulation to compile.
- Public ladder records and artifact indexes include rung manifest paths and hashes. `manifest_complete` indicates manifest presence only; `evidence_complete` remains false for the smoke.
- Provenance inventory includes the manifest implementation.
- The manifest is the authoritative inventory of its input/evidence closure; index consumers can discover member paths from the sidecar rather than requiring each member to be duplicated in the compact digest index.
- Strict loading requires exact top-level and trajectory field sets and qualification exactly `{approved: false, status: "unqualified"}`. Snapshot publication preflights path components and rejects symlinks before creating directories or copying files.

## Verification

Pinned runtime prefix:

```text
PYTHONPATH=src runs/no_tv_reliability/runtime_install_20261023/venv/bin/python
```

Focused final test command:

```text
PYTHONPATH=src runs/no_tv_reliability/runtime_install_20261023/venv/bin/python -m unittest tests.test_run_manifest tests.test_no_tv_minimum_lap tests.test_minimum_time_run tests.test_trajectory_artifact -q
```

Focused test history: one pre-fix attempt ran 93 tests and failed (2 failures, 1 error) due to deliberately incomplete worker fixtures and missing files in a conflict-document test. After correcting those fixtures/expectations, the final pre-commit command passed **93 tests in 26.267 seconds** (a preceding pass was 26.216 seconds). After commit, the expanded command below passed **120 tests in 26.743 seconds**; `git diff --check` passed.

```text
PYTHONPATH=src runs/no_tv_reliability/runtime_install_20261023/venv/bin/python -m unittest tests.test_run_manifest tests.test_no_tv_minimum_lap tests.test_minimum_time_run tests.test_trajectory_artifact tests.test_minimum_time_line_gate -q
```

Post-commit public smoke command:

```text
PYTHONPATH=src runs/no_tv_reliability/runtime_install_20261023/venv/bin/python tools/run/solve_minimum_time_line.py --track runs/no_tv_reliability/task1A2a2_fixture_postcommit/tiny_track.xml --formulation models/formulations/iem24_murphy_real_planar_7dof.xml --surface runs/no_tv_reliability/task1A2a2_fixture_postcommit/surface.npz --output runs/no_tv_reliability/task1B_public_manifest_postcommit --solver spline --spacings 20,10 --wall-cap-s 180 --half-width 2
```

The CLI exit code was 1 because this diagnostic case was unqualified/non-accepted, not because solving failed. Output: `runs/no_tv_reliability/task1B_public_manifest_postcommit/.supervision/`. The ladder result recorded both solves as `Solve_Succeeded`, two manifests present, `manifest_complete=true`, `evidence_complete=false`, and `accepted=false`. Each manifest and its trajectory reloaded with strict loaders; copied track loading and copied formulation compilation both succeeded. The artifact index included both rung manifests with `complete` status and SHA-256 values matching the files.

The smoke's numerical checks were not sufficient for acceptance (dense feasibility false); no acceptance or physical-validation claim is made. An earlier smoke exposed missing XML component dependencies in the source snapshot; the dependency closure was fixed, and the later snapshot compile/load verification succeeded. A later index check verified manifest indexing and hashes.

## Task 1B review follow-up

Independent review identified strict schema-label validation and pre-write symlink destination hardening as Important findings, plus a minor inventory recommendation. The implementation follow-up enforces exact top-level/trajectory/qualification fields and refuses symlinked snapshot path components before writes. The manifest is documented as the authoritative closure inventory; no extra parent-side hashing or unbounded index expansion was added.

Verification of the follow-up: `tests.test_run_manifest` passed 13 tests in 2.702 s. The expanded focused command passed 123 tests in 28.224 s before the fix commit and again in 28.059 s after commit; `git diff --check` passed. Post-fix public smoke ran with output directory `runs/no_tv_reliability/task1B_reviewfix_smoke/.supervision/`; the CLI exit was 1 for the unqualified diagnostic result. Both manifests and trajectories reloaded with strict loaders, qualification remained `{approved: false, status: "unqualified"}`, copied track loading and formulation compilation passed, and each indexed manifest SHA-256 matched. Ladder reported two manifests complete, `evidence_complete=false`, and `accepted=false`. Review closure remains pending independent confirmation.

## Boundaries and remaining work

- No clean-checkout/full raw FSC MAT and builder reproduction was run. Existing curated input-reproduction evidence was reused; the known missing raw builder input remains an explicit limitation.
- No all-track, U2/U3, physical, or full Task 1B acceptance is claimed.
- Manifest hashes establish byte identity, not signature/security trust or qualification.
- Evidence completeness remains false, and the policy remains unqualified.
