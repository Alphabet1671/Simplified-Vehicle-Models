# Bounded yaw-aware lap suite

This suite is independent of the legacy OCP solver. Use a completed, identity-compatible vehicle-limit yaw map; planning never rebuilds the map or fills masked cells. Every run has a process-group watchdog of at most 1,800 seconds. The suite self-guards at 1,800 seconds; use a shorter outer deadline when appropriate.

## Plan once, replay without optimization

From the yaw-aware worktree:

```bash
export PYTHONPATH=src:.
python3 tools/run/run_with_deadline.py --seconds 240 -- \
  python3 tools/run/run_yaw_lap_suite.py \
  --mode fixed --track models/tracks/fsc_may_endurance_2026_width.xml \
  --yaw-map runs/yaw_map_build/iem24-vehicle-limit-final.npz \
  --spacing 1 --profile-seconds 180 --output-dir runs/yaw_lap_suite/example-fixed
```

Planning writes a compressed plan with speed, physical and reference stations, selected-line geometry, settings, map/track identities and array digest. The plan is loaded through the independent strict all-edge validator before it is declared usable. Existing plans are not overwritten.

```bash
python3 tools/run/run_with_deadline.py --seconds 950 -- \
  python3 tools/run/run_yaw_lap_suite.py \
  --mode replay --track models/tracks/fsc_may_endurance_2026_width.xml \
  --yaw-map runs/yaw_map_build/iem24-vehicle-limit-final.npz \
  --plan runs/yaw_lap_suite/example-fixed/plan.npz \
  --controller model-aware --replay-seconds 850 --duration 90 \
  --output-dir runs/yaw_lap_suite/example-replay
```

Replay verifies track/map identity and array integrity, then independently validates all profile edges. It does not optimize again. The legacy `preview` controller remains available for matched comparisons. `model-aware` explicitly uses assembled motor/storage drive-force and drag feedforward, inversion of the actual hydraulic brake curve, pedal dynamics, course-error feedback and matched-preview yaw damping. Static feedforward is not a claim of transient capability.

Optional model-aware gain overrides are recorded in inputs: `--preview-seconds`, `--heading-gain`, `--lateral-gain`, `--yaw-damping`. They do not modify the speed plan. No implicit lateral-acceleration cap or fallback is used.

## Bounded line search

```bash
python3 tools/run/run_with_deadline.py --seconds 1000 -- \
  python3 tools/run/run_yaw_lap_suite.py \
  --mode line --track models/tracks/fsc_may_endurance_2026_width.xml \
  --yaw-map runs/yaw_map_build/iem24-vehicle-limit-final.npz \
  --spacing 2 --control-spacing 40 --profile-seconds 60 \
  --line-evaluations 24 --line-seconds 900 \
  --output-dir runs/yaw_lap_suite/example-line
```

The search evaluates its seed family before optimizing corner windows from the incumbent. Exact coefficient evaluations (including failures) are cached. Profile calls and elapsed search time have global budgets. Dense-geometry-rejected QP seeds are deterministically contracted toward the centreline; every contraction must pass the unchanged geometry gate. The best validated objective sample survives an interrupted Powell call. Warm suggestions interpolate squared speed in reference station, then are strictly revalidated.

The cooperative search deadline is checked between geometry/profile operations, so a finishing operation may overshoot it. The outer process-group watchdog remains the hard kill limit. `budget_exceeded` can contain a useful independently validated plan; it never means optimizer convergence.

## Convergence studies

Vary geometry spacing while holding control spacing fixed, then separately vary control spacing with geometry fixed. Same-resolution repeats are not convergence evidence.

```bash
python3 tools/run/run_with_deadline.py --seconds 30 -- \
  python3 tools/run/compare_yaw_lap_study.py \
  runs/yaw_lap_suite/coarse runs/yaw_lap_suite/fine \
  --dimension geometry --output runs/yaw_lap_suite/grid-study.json
```

The comparator verifies plan array integrity and the run-summary bound digest before using samples. Historical reports lacking that binding are explicitly inconclusive, not silently trusted. It compares speed and selected-line displacement at common reference stations and lap time. Valid profile, demonstrated profile refinement convergence, optimizer convergence, no budget stop, identical map/track identity and a genuine single-dimension refinement are prerequisites. Failed runs remain in the report and force an inconclusive verdict. Comparison is not a new physical-feasibility validation.

For fixed-geometry refinement, `--warm-plan` accepts a validated coarser saved plan as squared-speed seed suggestions. This is not a feasibility shortcut.

## Evidence, failures, and non-certifications

- A successful plan is an independently validated **discrete** profile, not continuous feasibility or global optimality.
- Optimizer stopping, profile refinement, geometry-grid convergence, and control-basis convergence are separate statuses. Consult [the closure index](yaw-lap-suite-closure/index.md) for current evidence and each exact status cell.
- The closure-v1 map source identity is `f8658cafbd35dc60cd195153ca4761db21d9a9d8b4819f1112a4f46ad7322419`. MIS track identity is `69bce5525e17c12bf17655760e6aedd5e1a98af7bd2d23b1bb05f8fccafad66f` (reconstructed geometry); FSC identity is `9d23af92a8e943aad7a90da5581f73f2733ab3cbed3fbb454f8843bb5fc797eb` (synthetic 3 m edges). Fresh manifests carry plan file/array digests, contract hash, source/vehicle/track hashes where available, effective settings, and watchdog budget. Missing historical source identity stays `unknown`.
- Keep failures and budget stops in comparisons. In the closure-v1 matrix MIS 0.5 m warm and cold both failed strict validation; no plan digest or lap time exists. FSC 0.5→0.25 m strict-valid plans are still refinement-unproven and miss grid criteria; see curated manifests and raw ignored runs from the index.
- Successful replay requires integration and accepted/between-step solver checks to finish within budget, but those solver-domain checks are not a corridor-containment certificate. Task 9 historical replays completed, but optimized/fixed pair identity remains inconclusive because saved plan source provenance is absent.
- Sampled vehicle-centre containment is distinct from wheel-contact, whole-body footprint, and swept containment. Task 9 sampled wheel contacts failed; body dimensions are unavailable; no actual provider-run swept certificate is attached to those replay records.
- Requested command saturation metrics are labeled requested. The saved run record contains realized pedal-limited controls.
- Failures have a summary. Unsuccessful IVP results retain a bounded last solver state, explicitly marked unvalidated, and the latest rejected trial cause. A timeout inside integration cannot invent an accepted solver state.
- Preserve the parked Important `//` source-path-key limitation and cost in the closure index's final-deferred section; do not normalize historical contract keys silently.
