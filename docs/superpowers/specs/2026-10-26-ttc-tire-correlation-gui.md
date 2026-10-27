# TTC tire correlation GUI design

## Scope and decisions

Build a standalone PyQt6 tire correlation GUI, following the existing vehicle/simulation/track GUIs. This is a design, not an implemented fitter. Work is on local branch `feature/ttc-tire-correlation-gui`. Preserve unrelated working-tree edits. Do not change an existing vehicle's tire or overwrite the current fitted models as a side effect.

The model is a deliberate hybrid: **43075 R20 lateral / 43100 R20 longitudinal proxy**. The user confirms that 43075 is too small for the longitudinal rig. Show this label in the window, datasets, fit summaries, and exports. Run 72's combined-slip Fy also belongs to 43100, not 43075; give it a separate label and objective weight.

### Completed local preparation

- Added ignores for `/private/ttc/` and `/tire_fits_out/` in [`.gitignore`](../../../.gitignore).
- Copied runs 7, 8, 9 and 72 from the reference project's TTC folder into local `private/ttc/TTC/`, preserving source files and verifying SHA-256 equality.
- Copied the specifically requested `unitire_simple_fit.mat` into local `private/ttc/`.
- Recorded paths, byte counts and hashes in a **private, ignored** copy manifest. No raw samples, fitData arrays, plots, or private manifests belong in Git. Do not use `git add -f`.
- Run 73 is mentioned by upstream notes but is not present in the reference TTC directory. Do not substitute another tire/run silently.

Ignore rules prevent ordinary accidental staging, not all disclosure. Default all measured-data plots, sessions, caches, residuals, optimizer logs and reports to ignored `tire_fits_out/`; automated tests use synthetic data only. Do not infer redistribution rights for fitted coefficients: exports remain private by default, and publishing any coefficient-only artifact requires user review of the applicable TTC terms. No full license text was found/verified in this investigation.

## Reference evidence

The external source directory is `/Users/alphabet1671/Dev/FSAE-VD-Personal-Scripts/Yaw Dynamics/Tire Model`.

- `UnitireFit.asv`, lines 9–17, explicitly says Round 9 data is not shared, identifies **run 7 as transient**, runs 8/9 as cornering and speed, and runs 72/73 as 43100 longitudinal.
- `UnitireFit_Simple_Auto.m`, lines 19–27, selects run 8 and run 72, belt speed 40.193/3.6 m/s, and at most 180 samples per segment. Lines 50–91 implement force fitting followed by trail fitting. Lines 505–565 contain the segment tables. Lines 568–637 define channel aliases and transformations.
- The requested MAT file contains a fit, not a GUI. Use the adjacent auto-fit and manual-fit MATLAB scripts as behavioral references; import the requested MAT as an explicit selectable seed, not as an implicit replacement for the repository lab fit.
- [Existing validation](../../validation/unitire-simple.md#L21-L28) explicitly says the repository lab XML came from `Hoosier_R20_Combined_Simple.mat`, **not** `unitire_simple_fit.mat`. Display distinct “repository lab XML” and “requested MATLAB fit” seeds. Neither is the road-scaled Murphy seed.

### Transient investigation: what “when” can be established

Run **B2356run7** is the transient run according to the source script. MAT metadata identifies `Hoosier 43075 16x7.5-10 R20, 8 inch rim`, `Cornering`, and `FSAE TTC Round 9, tested at Calspan Tire Research Facility`. The channel `ET` is elapsed time, not a calendar timestamp: it spans **0.15–798.652 s**, with **58,762 rows**. No acquisition calendar date/time field was found. A MATLAB file creation stamp or filesystem modification time is not a test date. Obtaining the calendar date requires the original TTC test log/run workbook; show “acquisition date unavailable” until that evidence is supplied.

Available channels include ET (s), V (km/h), N (rpm), SA/IA (deg), RL/RE (cm), P (kPa), force (N), moment (N·m), and dimensionless SR/SL. Run 7's SL is zero; that is not evidence of usable longitudinal excitation. Input/output steps must be checked together with load, pressure, speed, inclination and time gaps. No existing run-7 event table was found. Run identification is confirmed by script attribution; event suitability and coefficient identifiability are separate decisions.

Further local MAT inspection established:

- Run 7 is nominally 100 Hz, but has **61 gaps longer than 0.02 s**, including gaps of 1.11, 4.12, 5.12, 15.12 and 16.62 s. Split before event extraction/integration.
- Candidate approximately 6-degree plateaus occur at elapsed **304.14–312.46**, **326.14–334.46**, **348.14–356.46 s**, repeated at **519.14–527.46**, **541.14–549.46**, **563.14–571.46 s**, and **734.13–742.45**, **756.13–764.45**, **778.13–786.45 s**. The first three have median raw FZ approximately -216/-664/-1105 N and near-zero inclination. These are candidate plateau bounds, not complete accepted fit events; include the preceding input transition and baseline, after checking continuity. The recorded low-speed value around 3.14 has descriptor kph; verify protocol/speed semantics rather than substituting the steady 40.193 kph assumption.
- Run 8 and run 9 metadata both identify 43075, 16x7.5-10 R20, 8-inch rim; run 72 identifies **43100, 18x6-10 R20, 7-inch rim**, Drive/Brake/Combined. The proxy differs in rim as well as tire size.
- The requested fit MAT contains `tire` and `fitData`, but **no `fitReport`**. FitData holds 1,080 lateral, 720 longitudinal, 1,440 combined and 1,080 Mz samples, reference tire and 11.1647 m/s belt speed. It contains no transient fit. Importing a seed must not require a report key or publish those sample arrays.

**Consequences:** build the separate transient tab around run 7 lateral response and Mz; enable lateral/trail auto-fit only after event quality checks. Keep `longitudinal_length_scale` manually adjustable but explicitly **unidentified / provisional** unless a longitudinal time series passes an equivalent excitation gate. Ordinary run-72 steady sweeps do not automatically qualify. Run 9 is a speed/cornering holdout only after matching zero camber, pressure and load—not assumed transient evidence.

## GUI layout and interaction

Use native PyQt6 widgets, platform palette and fonts, QMainWindow, QSplitter, QTabWidget, QSettings and existing success/warning/error color conventions. No new application-wide theme or web frontend. Add Matplotlib QtAgg plots as an optional tire-GUI dependency; style axes from the Qt palette and maintain the same load-color mapping across tabs.

```text
Tire correlation — 43075 lateral / 43100 longitudinal proxy
[Open XML] [Import MATLAB seed] [Data folder] [Save fit as…]
Dataset / conditions       | [Steady-state] [Transient] [Fit diagnostics]
Run / tire / axis         | plots: measurements, current fit, initial fit
Load / pressure / IA      | residuals; chosen events/segments; units
Segment/event selection   | parameter group / Fit / Cancel / Apply candidate
Warnings and provenance   | slider + precise numeric value + bounds + lock
                          | Undo / Redo / Reset group / Restore seed
Status: dirty / fitting / cancelled / provisional / validated locally
```

- Steady-state tab: Fy versus alpha; Fx versus kappa; Fx/Fz versus Fy/Fz combined slip; Mz versus alpha. Filter by load/pressure/camber, toggle normalized force axes, and compare measured / current / seed curves. Default zero inclination only because this model has no camber physics.
- Parameter controls: label, repository XML name, units, current numeric value, editable bounds, integer slider with double-precision spinbox, fit-lock checkbox. Linear mapping by default; optional log mapping only for strictly positive parameters. Fit bounds are not a license to accept nonphysical stiffness; validate the resulting model across the selected load domain.
- Sliders update predictions with a 75 ms single-shot debounce; spinboxes preserve full numeric precision. Undo/redo stores immutable parameter snapshots. A cancelled fit cannot mutate live values. A candidate is applied only explicitly. A candidate for an old data/parameter revision is marked stale and cannot overwrite newer edits.
- Transient tab: elapsed-time overview, selected-event browser, co-timed command/response channels, measured-versus-predicted Fy/Mz/Fx, residuals, estimated sigma_x/sigma_y against load, and the four transient parameter sliders. Display event quality/exclusion reasons and provenance per coefficient, not just one global “fitted” badge.
- Native window geometry and last directories use `QSettings("Simplified-Vehicle-Models", "tire-gui")`. Restore paths only if they still exist. Unsaved-close confirmation, status messages, accessible labels/tooltips and keyboard numeric editing follow existing GUIs.

## Repository model boundary

Use [the steady evaluator](../../../src/simplified_vehicle_models/tires/unitire_simple.py#L247) and [the transient evaluator](../../../src/simplified_vehicle_models/tires/unitire_transient.py#L192); do not port another force law into the GUI. Compilation uses the existing XML loader and tire compilers. Do not write XML on every slider movement.

Steady output: `type="tire-model"`, `subtype="unitire-simple"`, one scalar parameter per existing name and unit. Polynomial suffixes are descending powers of normalized Fz. Transient output: a **separate** `subtype="unitire-transient-simple"` XML containing `longitudinal_length_scale`, `lateral_length_scale`, `trail_pole_ratio`, `trail_lead_ratio`, all dimensionless. Pair through existing assembly roles `tire` and `tire-transient`; never nest transient coefficients into the steady XML. No existing formulation is modified automatically.

Manual advanced controls may expose additional steady coefficients, but unidentifiable carcass stiffness, radius and regularization terms are locked by default; zero camber/overturning limitations remain visible. In particular, keep k_cx/k_cy fixed during transient scale fitting, otherwise only their ratios with the scales are identifiable. Do not import road-grip 2/3 scaling into a lab correlation by default.

Save-as produces a validated steady XML, separate transient XML if requested, and a private session/report. Include per-axis provenance and the steady-parameter fingerprint used by the transient fit. Changing steady coefficients invalidates previous transient fit status and requires replay/refit. Validate both models through the real compilers before finalizing; reject traversal, symlink escapes, accidental source overwrite and existing destination collision. Write paired artifacts through a temporary output directory and rename only after all checks succeed.

## Data and fit contracts

### TTC ingestion and reproducibility

Read MAT v5 files with SciPy. Preserve source sample order and raw indices; retain ET gaps and never concatenate separated events as continuous data. Missing units/channels, mismatched vector lengths, nonfinite required channels and nonmonotonic ET produce actionable errors or explicitly excluded segments. Do not silently interpolate across a time gap. Read channel metadata rather than guessing percent slip from magnitude. Support a user-confirmed unit override only with recorded provenance.

Runtime SI: alpha = -deg2rad(SA), positive normal-load magnitude = -FZ for the verified compression-negative TTC convention, speed = V/3.6, wheel speed = abs(N)*2*pi/60; FY, FX and MZ retain their measured signs. Do not negate MZ merely because an upstream comment describes a sign conversion: the actual auto script leaves MZ unchanged. Confirm signs against synthetic tests and measured small-slip trends. SL is the compatibility script's preferred longitudinal-slip channel; SR is not silently substituted when both exist.

**Compatibility profile:** copy exact source segment selection and MATLAB one-based inclusive indices; preserve nominal load, constant belt speed, inferred hub speed, channel signs and 180-point sampling. Use MATLAB-compatible half-up rounding, not NumPy bankers' rounding. Record original run identity, segment IDs, selected samples, exclusions and mode in private reports. Fresh measured-channel mode can use actual FZ/V/N after unit validation but must not claim MATLAB numeric parity.

### Steady automatic fit

1. Fit the 15 force parameters: k_x and k_y polynomial triplets, e_1, mu_0_x, mu_s_x, h_x, v_m_x, mu_0_y, mu_s_y, h_y, v_m_y.
2. Hold forces fixed; fit d_x_0, d_e, d_1 and d_2 pairs to Mz.
3. Compatibility objective is the **sum of group RMS values**, not one unweighted stacked least-squares residual: RMS(Fy/Fz residual) + RMS(Fx/Fz residual) + 0.75 RMS(combined Fx/Fz residual) + 0.75 RMS(combined Fy/Fz residual). Trail scale is max(Fz*max(r_1,0.001),1).
4. Preserve logistic bounded Nelder–Mead, iteration/evaluation limits and tolerance settings as a named MATLAB-compatible mode. Matching objective/transform is required; identical optimizer trajectories across MATLAB/SciPy are not promised.
5. Respect fit locks, reject empty active groups, report nonconvergence/bound saturation, and always show dimensional RMSE and selected-sample counts. Validation uses held-out whole segments/runs, not randomly interleaved adjacent points. Default compatibility training matches upstream; holdouts are explicit and do not secretly change its training sample set.

### Transient automatic fit

Use contiguous, unsmoothed time traces. Detect **candidate** SA changes, then require user-reviewed event boundaries and pre-event baseline. Split at gaps, load/IA/pressure/speed changes. Plot rather than conceal offset, overshoot or saturation. A supplied instrument delay is fixed and recorded; do not freely fit delay and relaxation simultaneously without an independent delay measurement. Initial states come from a stable pre-roll via `steady_rolling_state`; integrate the existing state derivatives with `solve_ivp`, piecewise-linear measured inputs and actual ET, stopping/restarting at discontinuities and gaps. Verify convergence by reducing maximum integration step.

Fit lateral_length_scale from Fy; then trail_pole_ratio/trail_lead_ratio from Mz with lateral scale fixed, finally optionally jointly refine those three if sensitivity is adequate. Fit longitudinal_length_scale from Fx only with independently approved excited longitudinal events. Fix steady parameters throughout. Use load-normalized force and load-times-radius-normalized moment residuals, equal weight per event, and whole-event holdouts. Broad initial working bounds: length scales [0.05,10], pole ratio [0.05,20], lead ratio [0,1]; these are engineering search bounds, not TTC-derived coefficient confidence intervals. Report sensitivity rank/condition, bound hits, event count and measured bandwidth. If dynamics are not time-resolved or sensitivity is deficient, leave affected coefficients provisional; never force a “successful” four-parameter fit.

## Acceptance criteria

1. Runs 8 and 72 reproduce the upstream sample selections, transformations and compatibility objectives; the requested MAT seed imports without dropping supported coefficients.
2. Steady and transient automatic fitting and manual sliders operate through the same repository evaluators and snapshot model.
3. Synthetic known-coefficient traces support recovery tests; held-out local TTC diagnostics report errors without an invented accuracy threshold or unearned validation claim.
4. Run 7 is labeled with the correct source evidence and elapsed-time range; calendar acquisition date remains explicitly unknown; unavailable longitudinal dynamics cannot be presented as fitted.
5. The app opens/saves repository-compatible separate XMLs, stays responsive during fitting, handles cancellation/stale results, and matches sibling GUI conventions.
6. No licensed samples, source MATs, screenshots containing measurements or derived fitData enter tracked fixtures or default shareable outputs.
7. Core tests run without Qt, GUI tests run offscreen with the optional extra, and existing tire/runtime tests stay unchanged and passing.
