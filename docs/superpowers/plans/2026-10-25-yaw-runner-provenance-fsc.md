# Yaw Runner Provenance and FSC Import Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make yaw-aware run records reproducible and failure-safe, then add a clearly synthetic 3 m wide FSC corridor generated from the original FSC one-dimensional centerline and execute a fine spatial-resolution diagnostic run.

**Architecture:** The runner writes a structured manifest mirroring the established forward-runner provenance while retaining the optimized line as record geometry. Failed no-sample simulations produce an explicit summary rather than a traceback. The committed FSC source is a copied original centerline plus a derived constant-width profile; its XML declares that its 3.0 m total corridor is synthetic/assumed rather than surveyed track-edge data.

**Tech Stack:** Python, NumPy, SciPy, XML/CSV track profiles, existing planar forward driver, NPZ run records, Matplotlib/ffmpeg.

**Spec:** [yaw-aware integration roadmap](../../dev/yaw-aware-driver-integration-roadmap-2026-10-25.md), [yaw-aware proposal v2](../../dev/virtual-driver-proposal-v2-2026-10-25.md).

## Global Constraints

- Do not modify any `qss_*ocp.py` source or QSS OCP test.
- The imported FSC corridor is a **synthetic assumed 3.0 m total width** (1.5 m per side), not measured/surveyed edges.
- Preserve source identity of `tracks_out/fsc_may_2026.xml` and CSV in committed copied source files.
- A planning-grid spacing of 0.01–0.05 m is mandatory for the requested FSC diagnostic; default runner spacing must not silently coarsen it.
- Results remain diagnostic: no minimum-time, TV/FM U, measured-vehicle, footprint, between-step, periodicity, refinement, or U3 claim.

---

### Task 1: Failure-safe and reproducible run records

**Files:**
- Modify: `tools/run/run_yaw_aware_forward_driver.py`
- Modify: `tests/test_yaw_aware_forward_runner.py`

**Interfaces:**
- `record_metadata(args, track, vehicle, capability, plan, result) -> dict` supplies producer, model, hashes/sources, source-track identity, optimized-line identity, map grid/identity, planner settings, and integrator/segment settings to `RunRecordSink.close`.
- A no-accepted-step record failure produces `summary.json` with `status: "failed"`, `failure_stage: "record"`, `lap_completed: false`, all qualification flags false, and the exception text; it does not emit a fake record.

- [ ] **Step 1:** Add mocks asserting `sink.close(record_metadata(...))` receives nonempty `model`, `sources`, `track`, `yaw_capability`, `planning`, and `integrator` values.
- [ ] **Step 2:** Add a mock sink whose `close` raises `RuntimeError("no accepted step was recorded")`; assert `run` writes failure summary and returns it.
- [ ] **Step 3:** Implement source hashes using the existing `source_hashes` helper and formulation/track files, and serialize map/plan identity through existing `_jsonable`.
- [ ] **Step 4:** Implement structured failure completion while retaining the new output directory for diagnosis.
- [ ] **Step 5:** Run focused tests and commit.

### Task 2: Commit original FSC centerline and synthetic-width track

**Files:**
- Create: `models/tracks/fsc_may_2026_centerline.csv`
- Create: `models/tracks/fsc_may_2026_centerline.xml`
- Create: `models/tracks/fsc_may_2026_synthetic_3m_width.csv`
- Create: `models/tracks/fsc_may_2026_synthetic_3m_edged.xml`
- Create: `docs/dev/fsc-synthetic-corridor-2026-10-25.md`
- Test: `tests/test_fsc_synthetic_track.py`

**Interfaces:**
- `fsc_may_2026_centerline.*` is a byte/content-provenance-preserving copy of the original 1D FSC source, with its XML data reference changed only to the co-located CSV.
- `fsc_may_2026_synthetic_3m_edged.xml` references the copied centerline CSV and a same-station width CSV of exact `half_width_left=half_width_right=1.5` m.

- [ ] **Step 1:** Test that both FSC tracks load, preserve closed topology and finite geometry, and that the edged track has `width_provenance == "assumed"` and exact 1.5 m widths.
- [ ] **Step 2:** Copy source CSV; create XML with visible provenance/disclosure; generate width rows matching source stations.
- [ ] **Step 3:** Write documentation stating 3.0 m is total width, not per-side width; record the original MAT hash/centerline provenance and non-admission as surveyed edges.
- [ ] **Step 4:** Run targeted track tests and commit.

### Task 3: Fine-resolution FSC diagnostic and animation

**Files:**
- Create (ignored): `runs/yaw_aware_driver/fsc-synthetic-3m-no-tv-<timestamp>/`
- Create: `docs/dev/yaw-aware-fsc-synthetic-diagnostic-2026-10-25.md`

- [ ] **Step 1:** Run runner preflight with the committed synthetic FSC edged XML, no-TV map, and `--planning-grid-spacing 0.05` m. Record the exact command and wall time.
- [ ] **Step 2:** If forward run completes, use `--record`, dry-run exporter, then write an MP4 at 30 fps/960×540; if it does not, preserve structured output and report exact failure without retrying the identical command.
- [ ] **Step 3:** Document plan status, completed state, times, sampled center-point evidence, source/provenance caveat, video path, and all nonclaims.
- [ ] **Step 4:** Commit documentation only; do not commit ignored run/video artifacts.

## Self-review

Task 1 addresses both reviewer-important defects. Task 2 obeys the user request without misrepresenting synthetic edges. Task 3 uses the requested fine 0.05 m track-space resolution, while preserving every diagnostic limitation and OCP isolation.
