# IEM24 Correlation Panels Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate two single-image five-channel measured/point-mass/7DoF overlays, with distance and time axes.

**Architecture:** Extend the existing three-way comparison tool rather than alter the six-channel standardized report. Read measured motor power from the original MAT log; use the same physical centreline alignment and flying-lap range, and use each model's exported elapsed time to construct a time-grid comparison. Disclose that the available 7DoF trace is an unconverged historical iterate.

**Tech Stack:** Python, NumPy, SciPy, Matplotlib, unittest.

**Spec:** User request and `docs/standards/mis-2024-report-standard.md`.

## Global Constraints

- The standardized `report.md` retains its required sections, tables, and figures.
- Figures label models as `Driving Data`, `Point Mass`, `7DoF` and preserve the measured range band.
- Never present an unconverged 7DoF iterate as a valid model prediction.

---

### Task 1: Read and align motor-power traces

**Files:** Modify `tools/compare_iem24_three_way.py`; Test `tests/test_iem24_three_way.py`.

**Interfaces:** Consume exported model `distance_m`, `elapsed_time_s`, `velocity_mps`, `apps_fraction`; produce conditioned measured RL/RR power using original MAT channels and matched sampling.

- [ ] Write tests for missing MAT channel errors, unit conversion, and distance/time interpolation.
- [ ] Run `PYTHONPATH=src python3 -m unittest tests.test_iem24_three_way -v` and confirm initial failure.
- [ ] Implement the minimal extraction and alignment functions with explicit model-power approximation disclosure where needed.
- [ ] Run the targeted tests until passing.

### Task 2: Render and document both figures

**Files:** Modify `tools/compare_iem24_three_way.py`, `docs/validation/iem24-murphy/comparison_three_way_report.md`; Create `docs/validation/iem24-murphy/comparison_three_way_distance.png` and `comparison_three_way_time.png`.

**Interfaces:** Both figures plot velocity, ax, ay, RL motor power, RR motor power and share the chosen independent axis.

- [ ] Add a figure test asserting five panels, correctly named legend entries, and distinct distance/time axis labels.
- [ ] Run the targeted test and observe failure.
- [ ] Implement rendering and include both figures inline in the report, with measured range retained and solver-status caveat outside the legend.
- [ ] Run generator using the available measured MAT and archived 7DoF CSV, then inspect output images.
- [ ] Run report-standard tests, full suite, and `git diff --check`.
