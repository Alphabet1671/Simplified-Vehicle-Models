# Yaw-Aware Simulation GUI and Legacy Lap Solver Retirement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate the simulation GUI with the yaw-aware fixed/line/replay solver for 7DoF/14DoF studies, preserve the point-mass forward/backward solver as a yaw-ignorant quick power-investigation mode that automatically adapts full models, archive only superseded non-point-mass lap workflows outside Git, and merge the feature branch into remote `main`.

**Architecture:** Merge the existing simulation GUI feature into `feature/yaw-aware-virtual-driver`, then adapt its study preparation/child-run contract to invoke one yaw-aware runner that always consumes a completed yaw-map artifact. GUI plans are yaw-aware saved artifacts and transient replay records, never point-mass `solve_lap` results. The prior point-mass forward/backward solver family is copied verbatim to `local-backups/legacy-lap-solvers-<UTC>/` and zipped before its tracked active modules, direct callers, examples, and tests are retired.

**Tech Stack:** Python 3, unittest, PyQt6 GUI, NumPy/SciPy, NPZ yaw-plan/run artifacts, Git/GitHub CLI, process-group deadline wrapper.

**Spec:** User direction in this session: wire the new yaw-aware solver into the simulation GUI; discard older lap solvers from active tracked use but retain an untracked backup folder and ZIP; push and merge to remote `main`.

## Global Constraints

- Work only in `.worktrees/yaw-aware-virtual-driver`; preserve pre-existing modified/untracked work and stage named migration paths only.
- Archive before removing any legacy tracked solver/caller/test. `local-backups/` and its ZIP must be Git-ignored and absent from commits.
- Preserve `forward_backward` point-mass solving as the yaw-ignorant quick power-investigation mode. It must automatically reduce compatible 7DoF/14DoF models through the existing point-mass view rather than requiring a separate point-mass XML input.
- Retire only superseded non-point-mass lap workflows. Do not remove transient planar/14DoF plants, yaw-aware solvers, point-mass power investigations, QSS/OCP/racing-line work, or independent capability-reference research merely because they optimize a speed/path.
- The GUI must require an existing completed yaw-map artifact. It must not rebuild or silently substitute a map, use `forward_backward.solve_lap`, or claim global/continuous/containment convergence.
- Every actual planner/replay/test process uses `tools/run/run_with_deadline.py`; planner/replay hard process deadline must not exceed 1,800 seconds.
- Preserve strict yaw map semantics: `interval_q_tolerance=1e-8`, `acceleration_tolerance=1e-7`, `yaw_moment_tolerance=1e-7`, masked support, and no arbitrary acceleration cap.
- Unsupported old public modes—point-mass `solve_lap`, energy/SOC/stint, and standing-start GUI requests—must fail clearly rather than dispatch archived code.
- Do not push/merge until migration tests, source-retirement search, review, clean committed migration state, and GitHub PR checks are complete.

---

### Task 1: Merge and characterize the simulation GUI

**Files:**
- Merge from: `feature/simulation-gui`
- Inspect/modify: `tools/sim_gui/{execute.py,run_case.py,settings.py,study.py,vehicle.py}`
- Inspect/modify: `tools/sim_gui/qt/{settings_tab.py,model_panel.py,track_tab.py}`
- Test: `tests/test_sim_gui_{core,run,qt}.py`

**Interfaces:**
- Existing GUI case payload: `StudyDefinition`, `CaseSpec`, `PreparedCase`.
- Existing GUI currently maps `RunSettings` into `tools.run.run_iem24_forward_driver` arguments.
- Produced migration baseline: GUI files are present on the yaw branch and all existing GUI tests either pass or have documented yaw-migration expectations.

- [ ] **Step 1: Fetch remote and merge the GUI feature locally**

```bash
git fetch origin
git merge --no-ff feature/simulation-gui -m "Merge simulation GUI for yaw-aware migration"
```

Resolve only semantic conflicts after inspecting both sides. Do not overwrite yaw-aware changes with stale GUI dependencies.

- [ ] **Step 2: Run GUI baseline tests under watchdog**

```bash
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 180 -- \
  python3 -m unittest tests.test_sim_gui_core tests.test_sim_gui_run -q
```

Record any expected migration failures before modifying the GUI.

- [ ] **Step 3: Commit the conflict-resolved GUI integration if it is not part of the merge commit**

```bash
git add tools/sim_gui tests/test_sim_gui_core.py tests/test_sim_gui_run.py
git commit -m "chore: integrate simulation GUI baseline"
```

### Task 2: Define point-mass and yaw-aware GUI settings and case contract

**Files:**
- Modify: `tools/sim_gui/settings.py`
- Modify: `tools/sim_gui/study.py`
- Modify: `tools/sim_gui/execute.py`
- Modify: `tools/sim_gui/qt/settings_tab.py`
- Modify: `docs/dev/simulation-gui.md`
- Test: `tests/test_sim_gui_core.py`, `tests/test_sim_gui_qt.py`

**Interfaces:**
- Replace the single old live-planner setting bundle with a serializable two-workflow contract:

```python
@dataclass(frozen=True, slots=True)
class RunSettings:
    solver_kind: Literal["point-mass", "yaw-aware"] = "yaw-aware"
    yaw_map: str = ""
    mode: Literal["fixed", "line", "replay"] = "fixed"
    plan: str | None = None
    spacing: float = 2.0
    control_spacing: float = 40.0
    profile_seconds: float = 900.0
    profile_rounds: int = 16
    line_seconds: float = 900.0
    line_evaluations: int = 24
    line_outer_passes: int = 3
    line_optimizer_iterations: int = 8
    duration: float = 150.0
    replay_seconds: float = 1200.0
    maximum_step: float = 0.05
    controller: Literal["preview", "model-aware"] = "model-aware"
    point_mass_duration: float = 300.0
    point_mass_spacing: float = 0.5
    point_mass_maximum_speed: float = 60.0
    point_mass_pedal_travel_time: float = 0.175
    point_mass_preview_seconds: float = 0.15
    point_mass_heading_gain: float = 1.5
    point_mass_lateral_gain: float = 1.5
    point_mass_rtol: float = 1e-5
```

- `solver_kind="point-mass"` remains supported for quick power investigations and automatically reduces `planar-7dof` and `14dof-simple` source models to a point-mass view; it deliberately ignores yaw-map capability.
- `solver_kind="yaw-aware"` requires a `planar-7dof` or `14dof-simple` source and automatically reduces a 14DoF source to the planar 7DoF solution form; it rejects source model `point-mass`.
- `mode="replay"` requires `plan`; `mode in {"fixed", "line"}` rejects it unless a separately named saved-plan workflow is deliberately supported.
- `yaw_map` must exist and be a completed artifact; validation must reject missing/invalid maps without rebuilding.

- [ ] **Step 1: Add failing settings serialization/validation tests**

Test missing yaw map, zero/nonfinite profile/line limits, replay without plan, unsupported standing boundary, and legacy fields (`planner`, `lateral_acceleration`, point-mass mode) rejected.

- [ ] **Step 2: Replace old `RunSettings` fields and argument mapping**

Remove `planner`, `lateral_acceleration`, `pedal_travel_time`, legacy `maximum_speed` live-plan inputs, and `quasi_static_attitude` from GUI state. Implement `yaw_runner_argv(settings, track)` that produces the explicit `run_yaw_lap_suite.py` CLI form and never imports `run_iem24_forward_driver`.

- [ ] **Step 3: Update Qt settings form**

Expose yaw-map browser/path, mode selector, profile spacing/control-spacing/budgets, replay plan browser when replay is selected, controller and integration settings. Hide/retire the old planner/lateral-cap/quasi-static fields. The UI must show that a completed yaw map is required and that output is sampled/discrete, not continuous certification.

- [ ] **Step 4: Run targeted tests**

```bash
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 180 -- \
  python3 -m unittest tests.test_sim_gui_core tests.test_sim_gui_qt -q
```

- [ ] **Step 5: Commit**

```bash
git add tools/sim_gui/settings.py tools/sim_gui/study.py tools/sim_gui/execute.py \
  tools/sim_gui/qt/settings_tab.py docs/dev/simulation-gui.md \
  tests/test_sim_gui_core.py tests/test_sim_gui_qt.py
git commit -m "feat: expose yaw-aware settings in simulation GUI"
```

### Task 3: Replace the GUI child runner with a yaw-aware runner

**Files:**
- Create: `tools/sim_gui/run_yaw_case.py`
- Modify: `tools/sim_gui/{execute.py,run_study.py}`
- Modify: `tools/run/run_yaw_lap_suite.py` only if it needs a narrow documented machine-readable case-input interface
- Test: `tests/test_sim_gui_run.py`, create `tests/test_sim_gui_yaw_runner.py`

**Interfaces:**
- `PreparedCase.argv` invokes `tools/sim_gui/run_yaw_case.py --case-dir <case> --child-attempt <attempt>`.
- The child reads immutable `case.json`, validates map/track/plan inputs, invokes the yaw study runner using its public argument parser, and writes `summary.json`, `inputs.json`, plan artifacts, and replay artifacts beneath the GUI attempt directory.
- GUI status mapping recognizes yaw runner statuses `planned`, `failed`, and replay completion; `completed` is reserved for an exit-zero child with required summary artifacts.

- [ ] **Step 1: Write failing tests for command construction and map binding**

Cover a fixed case, line case, replay case, missing map, map rebuild attempt, replay without plan, and child status propagation. Assert command contains the exact yaw-map path and does not contain `run_iem24_forward_driver` or old `--planner`.

- [ ] **Step 2: Implement `run_yaw_case.py`**

Load `case.json`; resolve paths relative to the repository/case; prohibit path escape outside declared map/plan/track inputs; build `run_yaw_lap_suite.parse_args` arguments; set `--output-dir` to the attempt directory; invoke `run()`; map `YawProfileSolveError`/validation errors to a structured failure summary, never to a legacy fallback.

- [ ] **Step 3: Switch `execute.child_command` and completion discovery**

`child_command()` must call `run_yaw_case.py`. `finish_case()` must read the yaw suite summary fields and retain plan/replay artifact relative paths. Ensure cancellation/timeouts still kill the complete child process group.

- [ ] **Step 4: Run runner tests**

```bash
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 180 -- \
  python3 -m unittest tests.test_sim_gui_run tests.test_sim_gui_yaw_runner -q
```

- [ ] **Step 5: Run one bounded fixed GUI headless smoke case**

Use a pre-existing completed yaw map and closed track, a fresh ignored `runs/gui-yaw-smoke-*` output, fixed mode, and a hard process deadline no greater than 1,800 seconds. Preserve a failure as evidence; do not retry automatically.

- [ ] **Step 6: Commit**

```bash
git add tools/sim_gui tools/run/run_yaw_lap_suite.py tests/test_sim_gui_run.py \
  tests/test_sim_gui_yaw_runner.py
git commit -m "feat: run yaw-aware plans from simulation GUI"
```

### Task 4: Archive and retire the superseded legacy transient GUI driver path

**Files to archive then remove from tracked active use:**
- `tools/run/run_iem24_forward_driver.py`, the former simulation-GUI child runner.
- `src/simplified_vehicle_models/solvers/forward_driver.py`, only if no retained non-GUI production entry point requires it; otherwise preserve the module and retire only the GUI-facing runner/documentation.
- Direct GUI tests/docs that exclusively assert the former live-driver GUI behavior.

**Files to preserve:** `forward_backward.py` and its point-mass power-investigation callers/tests; `capability_profile.py`, fixed-path energy, live-standing, profile-line, QSS/OCP/racing-line research; transient plants (`planar_forward.py`, `vertical_forward.py`); yaw-aware solver/map/validation modules; and historical documentation/reports.

**Archive format:**
- `local-backups/retired-gui-bridge-<UTC>/manifest.json`
- `local-backups/retired-gui-bridge-<UTC>/tracked-files/tools/sim_gui/run_case.py`
- `local-backups/retired-gui-bridge-<UTC>.zip`

**Interfaces:**
- Add `/local-backups/` to `.gitignore` before creating the archive.
- `manifest.json` contains UTC timestamp, branch/head SHA, original relative path, SHA-256, byte count, and archive reason for every copied file.
- The ZIP contains exactly the backup directory contents; every member’s bytes are verified against the staged file and manifest hash before publication.

- [x] **Step 1: Add the ignored archive root and archive manifest script/test**

`tools/maintenance/archive_retired_gui_bridge.py` archives only the retired `tools/sim_gui/run_case.py` bridge. It refuses existing timestamped outputs, hashes sources before copying, verifies copied and ZIP-member hashes, and prints its output paths.

- [x] **Step 2: Execute archive before retirement**

A watchdog-bounded run created and verified ignored local archive `local-backups/retired-gui-bridge-20261027T075649Z` plus its ZIP. Neither is staged.

- [x] **Step 3: Remove only the archived GUI bridge**

Delete `tools/sim_gui/run_case.py` after its verified local archive exists. Preserve `forward_backward.solve_lap` and its exports/callers/tests, `capability_profile`, fixed-path energy, live-standing, profile-line, transient forward driver/plants, and QSS/OCP workflows. Do not remove historical results merely because they mention older solver variants.

- [x] **Step 4: Make the active GUI path explicit**

`execute.child_command()` invokes only `tools/sim_gui/run_yaw_case.py`; tests assert it never invokes the former GUI bridge or full-model forward-driver runner as a point-mass substitute.

- [x] **Step 5: Add source-retirement tests**

Assert the archive manifest and ZIP payload hashes match the copied source, the active GUI runner uses the dedicated yaw/point-mass adapter, and no active caller depends on `tools/sim_gui/run_case.py`.

- [x] **Step 6: Run targeted retirement tests**

The archive/core/run/yaw suite passed 57 tests under the 180-second watchdog. The larger Qt-inclusive suite later timed out at 180 seconds and is recorded as inconclusive, without retry.

- [x] **Step 7: Commit only active source/doc/test retirement changes**

Commit `a18c53a` contains only the ignore rule, archive helper/test, and deleted bridge. `git status --ignored local-backups` confirms the archive and ZIP remain ignored.

### Task 5: Verify, review, publish, and merge

**Files:**
- Modify as needed: `docs/dev/simulation-gui.md`, `docs/dev/package-layout.md`, retirement report under `docs/superpowers/`
- Test: all new/migrated GUI and yaw tests

**Interfaces:**
- Branch must have no staged/unstaged migration files other than preserved unrelated work before publication.
- Remote PR targets `origin/main`; merge only after `gh` confirms PR/head/base and required checks/merge permissions.

- [ ] **Step 1: Run migration scope tests**

Run GUI/core/Qt/run/yaw-runner/retirement suites under a 180-second watchdog; run one GUI smoke test under its separately declared process deadline. Record pass/fail/inconclusive honestly.

- [ ] **Step 2: Run source and artifact audit**

```bash
git grep -nE 'solvers\.(forward_backward|capability_profile|fixed_path_energy_nlp|live_standing_profile|profile_line_seed)' -- ':!docs/validation/**' ':!docs/superpowers/**'
git status --short
git status --ignored --short local-backups
```

The first command must have no active imports. Confirm no archive/ZIP is staged and all unrelated pre-existing modifications remain unstaged.

- [ ] **Step 3: Create review package and run branch review**

Generate a full migration diff package from its pre-merge base through HEAD. Review for legacy imports, GUI map binding, watchdog behavior, backup manifest/ZIP integrity, false solver claims, and accidental changes to strict map semantics/transient plants.

- [ ] **Step 4: Commit final docs/review evidence**

```bash
git add docs/dev/simulation-gui.md docs/dev/package-layout.md docs/superpowers
git commit -m "docs: record yaw-aware GUI solver migration"
```

- [ ] **Step 5: Push, open PR, and merge with GitHub CLI**

```bash
git push -u origin feature/yaw-aware-virtual-driver
gh pr create --base main --head feature/yaw-aware-virtual-driver \
  --title "Wire yaw-aware solver into simulation GUI" \
  --body-file /tmp/yaw-aware-gui-pr.md
gh pr checks --watch
gh pr merge --merge --delete-branch
```

Before `gh pr merge`, verify the PR base is `main`, head is `feature/yaw-aware-virtual-driver`, current PR diff is the reviewed branch, required checks are passing or explicitly waived by repository policy, and no unrelated dirty files are included.

- [ ] **Step 6: Verify remote merge**

```bash
git fetch origin
git log -1 --oneline origin/main
gh pr view --json state,mergedAt,mergeCommit,url
```

Record merge commit, PR URL, checked test scope, full-suite status, and the non-Git local archive path/ZIP in the final delivery note.
