# MIS Default Track, 7DoF Correlation and Road-Grip Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Adopt the ten-lap median MIS driven line as the repo's operational track, deliver a paired-track 7DoF correlation report, and implement defensible separately parameterized longitudinal/lateral UniTire grip scaling.

**Architecture:** Preserve the original track and previous result artifacts as historical evidence; copy the new driven line into a canonical model location and point active MIS consumers at it. Run bounded transient 7DoF solves with exact track and tire hashes, export both traces, and reuse the measured projection/report standard with honest acceptance-gate caveats. Scale the UniTire peak/sliding mu parameters, not slip stiffness, using a documented evidence-driven calibration; if asphalt grip cannot be identified from available evidence, retain values and clearly label a parameterized sensitivity study rather than inventing a coefficient.

**Tech Stack:** Python 3, NumPy, SciPy, Matplotlib, XML models, unittest, MIS report Markdown.

**Spec:** Direct user request of 2026-10-15; `docs/standards/mis-2024-report-standard.md`; `docs/validation/iem24-murphy/multilap_track_investigation.md`.

## Global Constraints

- Preserve `models/tracks/2024_michigan_endurance_closed.{xml,csv}` and all old runs unmodified.
- Use the 10 measured flying laps as median/reference, exclude standing lap from statistics but include it in the report table.
- Mark transient forward-driver 7DoF runs as non-optimal and not validated unless every acceptance gate passes.
- Do not infer unique road mu solely from a 120-grit belt label; disclose conditions and identifiability.
- Produce exactly ordered report sections `Configuration`, `Measured data`, `Trace comparison`, `Reading the comparison`, `Caveats`, `Reproduce` and required figures.

---

### Task 1: Canonical MIS Driven Line

**Files:** Create `models/tracks/2024_michigan_endurance_multilap_closed.{xml,csv}`; modify `tools/build_iem24_multilap_track.py` only if needed; test `tests/test_iem24_multilap_track.py`; document in `docs/validation/iem24-murphy/multilap_track_investigation.md`.

**Interfaces:** `load_track('models/tracks/2024_michigan_endurance_multilap_closed.xml')` returns a signed closed `TrackProfile` of length 1009.19501535 m.

- [ ] **Step 1: Write test:** assert `load_track` reports the expected length, closed XY geometry, and original track remains 1016.70064959 m.
- [ ] **Step 2: Run test** `PYTHONPATH=src:. python3 -m unittest tests.test_iem24_multilap_track -q` and verify failure before creating canonical files.
- [ ] **Step 3: Copy measured candidate CSV into `models/tracks/` without modifying old geometry; author metadata-rich canonical XML referring to that CSV.** Record SHA-256s in provenance documentation.
- [ ] **Step 4: Run the focused tests and preserve legacy regression tests.**

### Task 2: Parametric UniTire Grip and Evidence

**Files:** Modify `models/tires/iem24_murphy_real_unitire.xml` if justified, or create a road-grip variant and reference it from `models/vehicles/iem24_murphy_real_planar.xml`; create `docs/validation/iem24-murphy/road_grip_calibration.md`; test `tests/test_unitire_simple.py`.

**Interfaces:** Four parameters `mu_0_x`, `mu_s_x`, `mu_0_y`, `mu_s_y` in XML give direction-specific static/sliding grip; `evaluate_unitire` remains unchanged and reduces to the fitted model with multiplier 1.

- [ ] **Step 1: Write a failing test** asserting the selected road variant multiplies both peak and sliding mu of each direction by the documented ratios, and does not change slip stiffness or radius.
- [ ] **Step 2: Run the test** `PYTHONPATH=src:. python3 -m unittest tests.test_unitire_simple -q`, verify the targeted new assertion fails.
- [ ] **Step 3: Determine defensible coefficients** from measured longitudinal/lateral acceleration corrected for gravity, aero and load if possible; distinguish a lower bound from a fitted surface transfer, and never claim a 120-grit-to-road universal constant.
- [ ] **Step 4: Set XML coefficients or document that data are insufficient**; run force-curve regression and reference vector tests; explain changed combined-slip consequence.

### Task 3: Traceable Paired 7DoF Runs

**Files:** Modify `tools/run_iem24_forward_driver.py` and `tools/export_iem24_forward_correlation.py`; test `tests/test_forward_driver_runner.py`; create new outputs under `docs/validation/iem24-murphy/seven_dof_track_pair/`.

**Interfaces:** `run_iem24_forward_driver.py --track <signed-xml>` stores track XML and referenced CSV hashes in its summary/source snapshot; export reads the run's track setting and rejects mismatches.

- [ ] **Step 1: Write failing parser/provenance tests** for default canonical track, explicit old track, missing/unsigned track rejection, and warm-start hash mismatch.
- [ ] **Step 2: Run focused tests** to confirm failures.
- [ ] **Step 3: Implement runner setting/hash/snapshot and matching export track.** Preserve historical attempt compatibility with explicitly documented fallback.
- [ ] **Step 4: Run bounded isolated original-track and canonical-track solves** with identical solver/vehicle/grip parameters; save separate full attempt summaries and log acceptance status.
- [ ] **Step 5: Export accepted state traces** only on completed laps, audit closure, path deviation, mesh spacing and between-step validation.

### Task 4: MIS Correlation Report

**Files:** Create `tools/report_iem24_seven_dof_track_pair.py`; `docs/validation/iem24-murphy/seven_dof_track_pair/report.md` and `comparison.png`, `lap_spread.png`, `simulated_lap.png`, `three_overlay.png`; test `tests/test_mis_report_standard.py` and new reporter tests.

**Interfaces:** Compare independently projected measured median with original and canonical 7DoF traces on physical distance; do not phase-stretch unequal geometries.

- [ ] **Step 1: Write reporter test** asserting required six channels, three standard figure names, model status/track hashes and standing-lap table.
- [ ] **Step 2: Generate figures and report** from the two fresh exports; include voltage reconstruction and logged vs modeled RL/RR power accounting.
- [ ] **Step 3: Run `PYTHONPATH=src:. python3 -m unittest tests.test_mis_report_standard -q` and source-specific tests.**
- [ ] **Step 4: Inspect figure and numeric alignment**, disclose whether both laps completed, and avoid any unearned validation claim.
- [ ] **Step 5: Run full suite** `PYTHONPATH=src:. python3 -m unittest discover -s tests -q`, `git diff --check`, and deliver report and figure links.
