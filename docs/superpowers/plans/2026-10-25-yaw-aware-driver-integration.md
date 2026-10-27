# Yaw-Aware Driver Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the yaw-aware line planner runnable through the existing forward simulation and animation pipeline for an MIS diagnostic run, while honestly recording the FSC input-data blocker.

**Architecture:** Add a separate yaw-aware runner rather than modifying the established centreline runner. It compiles the existing vehicle, builds a physical yaw map and a `YawAwareLinePlan`, derives the optimized-path `PathCoordinates` for `RunRecordSink`, then uses the existing forward plant/`PreviewDriver`/`solve_forward` path. The existing animation exporter consumes the generated run record unchanged.

**Tech Stack:** Python, NumPy, SciPy, existing XML compiler, forward-driver plant, Matplotlib/ffmpeg animation exporter.

**Spec:** [virtual-driver proposal v2](../../dev/virtual-driver-proposal-v2-2026-10-25.md); [integration gap assessment](../../dev/yaw-aware-driver-integration-roadmap-2026-10-25.md).

## Global Constraints

- Preserve every `qss_*ocp.py` file and test unchanged.
- The new runner is diagnostic only: it must not claim minimum time, TV/FM U validation, measured-vehicle validity, full U3 qualification, footprint containment, between-step containment, periodicity, or refinement unless it actually records the separate evidence.
- MIS output is written under `runs/yaw_aware_driver/`; never overwrite an existing output directory.
- The runner must record the optimized offset path as the run-record reference geometry. The centreline must not be silently substituted.
- `physical_no_tv_v1` is the first runnable map source. Regen vectoring remains unavailable. An FSC run is blocked until an edged, signed FSC track source is supplied.

---

### Task 1: Document scope and FSC blocker

**Files:**
- Create: `docs/dev/yaw-aware-driver-integration-roadmap-2026-10-25.md`

**Interfaces:**
- Produces a dated diagnostic scope statement, runnable MIS command, artifact list, and FSC-data admission checklist.

- [ ] **Step 1: Write the document** with: current branch/commit identity; the diagnostic-only disclosures; the six integration stages; exact FSC required inputs (XML/CSV, signed XY, left/right widths, closure/seam); and the rule that absent FSC data produces a documented blocked outcome rather than an invented track.
- [ ] **Step 2: Verify internal links resolve** with a repository-relative link check.
- [ ] **Step 3: Commit** `docs/dev/yaw-aware-driver-integration-roadmap-2026-10-25.md`.

### Task 2: Convert a yaw-aware line to animation geometry

**Files:**
- Create: `src/simplified_vehicle_models/planning/yaw_aware_record_path.py`
- Test: `tests/test_yaw_aware_record_path.py`

**Interfaces:**
- Consumes `YawAwareLinePlan`.
- Produces `path_coordinates_from_yaw_line(plan) -> PathCoordinates` whose station is the plan’s physical path arclength, whose XY/heading/curvature are sampled from the exact `OffsetTrajectory`, and whose closure matches the plan.

- [ ] **Step 1: Write failing tests** using the synthetic circle plan:

```python
path = path_coordinates_from_yaw_line(plan)
np.testing.assert_allclose(path.distance, plan.speed_plan.distance)
np.testing.assert_allclose(path.curvature, plan.speed_plan.curvature)
assert path.is_closed
```

Also test an open plan retains nonperiodic endpoint geometry.

- [ ] **Step 2: Run** `PYTHONPATH=src python3 -m unittest tests.test_yaw_aware_record_path -v`; expect an import failure.
- [ ] **Step 3: Implement** by mapping each physical `SpeedPlan.distance` through `plan.trajectory.station_map.to_reference`, querying `trajectory.evaluate`, and returning immutable arrays.
- [ ] **Step 4: Run focused test and line/profile tests.**
- [ ] **Step 5: Commit** the module and tests.

### Task 3: Add diagnostic yaw-aware MIS runner

**Files:**
- Create: `tools/run/run_yaw_aware_forward_driver.py`
- Test: `tests/test_yaw_aware_forward_runner.py`

**Interfaces:**
- CLI arguments: `--track`, `--output-dir`, `--model {planar-7dof,vertical-14dof}`, `--map {no-tv,physical-tv}`, `--yaw-speed-points`, `--yaw-ax-points`, `--yaw-ay-points`, `--maximum-speed`, `--lateral-margin`, `--preview-seconds`, `--heading-gain`, `--lateral-gain`, `--speed-gain`, `--duration`, `--wall-seconds`, `--maximum-step`, `--rtol`, `--method`, `--record`, `--record-precision`, `--frame-rate-note`.
- Output directory: `plan.json`, `summary.json`, `trace.csv`, `pedal.csv` when available, `run_record.npz` when `--record`, and `line.csv`.
- `plan.json` contains map provenance, planning configuration, status, line shape metrics, candidate diagnostics, and explicit diagnostic-only disclosures.

- [ ] **Step 1: Write failing parser/output tests** using a temporary directory and a synthetic track/model fixture:

```python
with self.assertRaises(FileExistsError):
    prepare_output(existing_directory)
self.assertEqual(parse_args(["--map", "no-tv"]).map, "no-tv")
```

A test double path must assert that `RunRecordSink` receives optimized-path geometry, not `track.path_coordinates`.

- [ ] **Step 2: Run focused runner tests; confirm expected failure.**
- [ ] **Step 3: Implement** a standalone runner by reusing the public forward-driver APIs, not copying the old runner’s acceptance policy:
  1. Compile the model and obtain planar control parameters plus its yaw inertia.
  2. Load only signed XY tracks with edges.
  3. Build `YawCapabilityGrid` from validated declared grids and build the requested map.
  4. Use `corridor_from_track(..., policy=SIGNED_POLICY, reference=ReferenceCurve(track))` and `optimize_yaw_aware_line`.
  5. Use `path_coordinates_from_yaw_line(plan)` as the `RunRecordSink` path.
  6. Run existing `solve_forward`; use `trajectory_corridor_containment` for sampled center-point evidence.
  7. Emit a summary that cannot set `qualified`, `accepted`, or `U3_qualified` true.
- [ ] **Step 4: Run unit tests plus `tests.test_yaw_aware_*`.**
- [ ] **Step 5: Commit.**

### Task 4: Execute MIS diagnostic and animation

**Files:**
- Create: `runs/yaw_aware_driver/mis-no-tv-<timestamp>/` (gitignored artifacts)
- Create: `docs/dev/yaw-aware-mis-diagnostic-2026-10-25.md`

- [ ] **Step 1: Run a preflight** on `models/tracks/2024_michigan_endurance_closed_edged.xml` with the no-TV map and `--record`, using a conservative yaw grid and no more than the planar 300-second wall cap.
- [ ] **Step 2: If preflight completes, run the selected bounded MIS attempt** and preserve its command, output identity, plan/summary, and run record. If it fails, retain the failure directory and record the exact failing condition.
- [ ] **Step 3: Verify record sources and dry-run animation.**

```sh
python3 tools/anim/export_animation.py --record <run>/run_record.npz --dry-run
```

- [ ] **Step 4: Export an MP4 at 60 fps if ffmpeg is available; otherwise export a PNG sequence.**
- [ ] **Step 5: Write a diagnostic report** with exact command, planner/map identity, success/failure status, artifact paths, sampled containment, and all non-claims. Do not use MIS 2024 report format: this is not measured-vs-simulated comparison.
- [ ] **Step 6: Commit only documentation and durable small metadata; do not commit large traces/video unless repository policy explicitly permits them.**

### Task 5: FSC input admission

**Files:**
- Create: `docs/dev/yaw-aware-fsc-admission-2026-10-25.md`

- [ ] **Step 1: Search the repository for an FSC signed-edge XML/CSV source and record exact searched locations.**
- [ ] **Step 2: If an admissible source exists, run the runner preflight and then Task 4’s diagnostic/export steps using it.**
- [ ] **Step 3: If absent, write a BLOCKED admission report** listing the exact required source artifacts and declare no FSC animation generated. Never synthesize geometry from old OCP output.
- [ ] **Step 4: Commit the admission report.**

## Self-review

- Task 1 covers documentation and the FSC blocker.
- Task 2 ensures animation is geometrically truthful to the optimized path.
- Task 3 integrates planner, simulation and record generation without changing QSS OCPs.
- Task 4 generates actual MIS evidence and video/frames.
- Task 5 either runs FSC with authoritative data or preserves an honest block.
- The FMU harness is explicitly out of scope and remains shelved.
