# MIS Multilap Geometry Investigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Test whether the double-peaked modeled lateral acceleration comes from the current single-lap track, and construct a separately named representative geometry from consistent measured flying laps.

**Architecture:** Quantify curvature, point-mass ay, 7DoF ay and individual measured ay at common projected distances; reject alignment/filtering artifacts. Align each measured lap independently in north/east using its projected positions and fit an outlier-resistant mean line; smooth its periodic position and recompute signed curvature and arc length. Benchmark the candidate as a counterfactual with identical measured speed before any expensive solver reruns. Preserve the original track and simulation exports.

**Tech Stack:** Python 3, NumPy, SciPy, matplotlib, existing `tools/build_closed_track.py`, `tools/compare_iem24_murphy_stint.py` and `simplified_vehicle_models.tracks.profile`.

**Spec:** User request of October 15, 2026: investigate double-peaked model ay in relatively steady turns; assess smoothing similar normal measured laps into a new track.

## Global Constraints

- Do not replace `models/tracks/2024_michigan_endurance_closed.xml` or change existing model/figure outputs during diagnosis.
- Distinguish lateral acceleration (`ay`) from longitudinal braking (`ax`); do not call a point-mass kinematic comparison a converged new simulation.
- Project and compare individual laps before median folding; check geometry closure, winding, deviation and curvature stability.
- The 7DoF forward lap is driven transient, not an OCP optimum; a new geometry requires independent integration before attributing dynamic differences.

---

### Task 1: Locate and explain conflicting ay extrema

**Files:** Create `docs/validation/iem24-murphy/multilap_track_investigation.md`.

**Interfaces:** Consumes `measured_stint.csv`, signed track coordinates, `simulated_lap.csv` and the live 7DoF forward export; produces numerical distance windows, actual ay and curvature peak positions, per-lap residuals.

- [ ] Compute the old track's curvature and `v_measured² * curvature`, measured per-lap ay, point-mass and 7DoF ay on one 1 m distance grid.
- [ ] Verify the periodic projection's forward increments and compare individual-lap profiles to the median; record uncertainty from 20 Hz samples and any crossing ambiguity.
- [ ] Test whether identical temporal filtering changes the modeled double-peak signature. Describe the distinction from ax braking pulses.

### Task 2: Build a separate candidate from consistent laps

**Files:** Create `tools/build_iem24_multilap_track.py`, `tests/test_iem24_multilap_track.py`, `docs/validation/iem24-murphy/multilap_track_candidate.csv`, `docs/validation/iem24-murphy/multilap_track_candidate.xml`.

**Interfaces:** Input measured north/east position and ten flying-lap IDs, old track for distance-only registration; output a validated periodic signed XY geometry, selected lap IDs, per-lap offsets and diagnostics. Existing `write_csv`, `signed_geometry` and loader must accept the output unchanged.

- [ ] Write and fail tests for periodic alignment and line median on synthetic shifted circles plus rejection of outlier laps.
- [ ] Align each lap by robust constant translation, reject inconsistent line residuals with explicit threshold, correct per-lap seam drift, interpolate at common old-track phase, then periodically smooth and reparameterize arc length.
- [ ] Run tests, write a separately named XML/CSV and loader roundtrip; report closure gap, winding, minimum radius, geometric offset and cross-lap spread.

### Task 3: Diagnose mechanism and limitations

**Files:** Modify `docs/validation/iem24-murphy/multilap_track_investigation.md`.

**Interfaces:** Compare old and candidate curvature with measured speed held fixed and the per-lap measured ay at a common spatial grid; no claims about new point-mass or 7DoF runs unless those runs are explicitly executed with the candidate.

- [ ] Measure peak–valley–peak contrast on old/candidate kinematic ay and individual measured ay at specifically reported corners; record residual metrics away from corners.
- [ ] If the geometry corrects the kinematic double peak, propose a separate controlled model run; otherwise distinguish planner transient/braking changes and measurement bias.
- [ ] Re-run candidate loader/tests and relevant existing regression tests, retaining the original track unchanged.
