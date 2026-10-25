# Physical-Speed-Domain Yaw Driver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the yaw-aware runner’s arbitrary user-selected speed ceiling and derive its finite map domain from the compiled vehicle’s physical straight-line capability, then execute MIS and synthetic-FSC diagnostics without `--maximum-speed`.

**Architecture:** `--maximum-speed` becomes an optional explicit test limiter rather than a required default. The default yaw-map upper speed is computed through the existing point-mass straight-line capability envelope, expanded only until there is no feasible positive-drive steady-state cell. The map’s finite axis remains a declared numeric domain; curvature, acceleration, yaw authority, tire/drivetrain constraints, and the chosen map sampling continue to limit the speed profile.

**Tech Stack:** Python, NumPy, existing point-mass capability model, yaw-capability LP, planar 7DoF forward plant.

**Spec:** [yaw-aware proposal v2](../../dev/virtual-driver-proposal-v2-2026-10-25.md), [FSC synthetic diagnostic](../../dev/yaw-aware-fsc-synthetic-diagnostic-2026-10-25.md).

## Global Constraints

- Do not modify `qss_*ocp.py` or QSS OCP tests.
- “No cap” means no arbitrary CLI speed cap; it does not mean infinite speed or bypassed physical feasibility.
- The chosen map-domain upper speed must be stored in plan/record provenance along with its derivation.
- FSC remains a synthetic 3.0 m total-width corridor, not surveyed edges.
- Preserve 0.05 m physical profile station spacing for the FSC run.
- Both resulting runs remain diagnostic and must report containment independently.

---

### Task 1: Derive and expose physical map speed domain

**Files:**
- Modify: `tools/run/run_yaw_aware_forward_driver.py`
- Modify: `tests/test_yaw_aware_forward_runner.py`

**Interfaces:**
- `physical_map_speed_limit(point_mass_vehicle, *, search_limit_mps=...) -> float` returns a finite positive upper domain derived from steady straight-line positive-drive feasibility.
- `--maximum-speed` default is `None`; when supplied it is an explicit limiter. Plan/record metadata contains `speed_domain_source` equal to `"physical_straight_line_envelope"` or `"explicit_cli_limit"`.

- [ ] **Step 1:** Test omitted `--maximum-speed` parses as `None`; explicit value parses as positive.
- [ ] **Step 2:** Test the speed-domain helper with a deterministic fake/real point-mass fixture, requiring finite positive result and explicit limiter behavior.
- [ ] **Step 3:** Implement a finite expanding/bisecting search against existing physical capability functions; reject if no feasible positive-drive speed exists.
- [ ] **Step 4:** Build map and line settings from the derived value and record derivation/source.
- [ ] **Step 5:** Run focused runner/capability tests and commit.

### Task 2: No-explicit-cap MIS diagnostic

**Files:**
- Create (ignored): `runs/yaw_aware_driver/mis-physical-domain-no-tv-<timestamp>/`
- Create: `docs/dev/yaw-aware-mis-physical-domain-diagnostic-2026-10-25.md`

- [ ] **Step 1:** Run the runner with no `--maximum-speed`, recorded output, physical map domain provenance, and a bounded wall time.
- [ ] **Step 2:** If profile/planning fails, retain structured artifacts and document the exact physical-domain/map resolution limitation; do not substitute a hidden 7/8 m/s cap.
- [ ] **Step 3:** If it completes, export MP4 and report profile/driven speed maxima, times, containment, and nonclaims.
- [ ] **Step 4:** Commit documentation only.

### Task 3: No-explicit-cap FSC diagnostic

**Files:**
- Create (ignored): `runs/yaw_aware_driver/fsc-synthetic-3m-physical-domain-no-tv-<timestamp>/`
- Create: `docs/dev/yaw-aware-fsc-physical-domain-diagnostic-2026-10-25.md`

- [ ] **Step 1:** Run synthetic FSC with no `--maximum-speed`, `--planning-grid-spacing 0.05`, record output, and physical-domain provenance.
- [ ] **Step 2:** Export animation only for a completed record; otherwise document non-completion exactly.
- [ ] **Step 3:** Report map domain, actual profile/driven maxima, containment, synthetic-corridor caveat, and any high-speed profile limitation.
- [ ] **Step 4:** Commit documentation only.

## Self-review

Task 1 removes the arbitrary default safely without claiming a physically unbounded domain. Tasks 2–3 execute both requested tracks and preserve transparent diagnostics if the current sampled profile cannot support the derived map extent.
