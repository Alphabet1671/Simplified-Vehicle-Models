# IEM24 Murphy vertical 14DoF-simple — parameter provenance, reference numbers and Task 9 gate results

Scope: the `14dof-simple` rung of the fidelity ladder (`models/formulations/iem24_murphy_real_14dof_simple.xml`).
This document is **not** an MIS 2024 report and not a validation report: it records where every vertical,
suspension, damper and transient-tire parameter comes from, states the reference numbers the rung is
tested against, and reports the Task 9 acceptance gates with their measured evidence. The lap evidence
lives in [`docs/validation/iem24-murphy/14dof_simple_trials/`](iem24-murphy/14dof_simple_trials/); the
machine-readable gate table is
[`attempt_1792288406878879000/gates.json`](iem24-murphy/14dof_simple_trials/attempt_1792288406878879000/gates.json),
checked by [`tests/test_vertical_14dof_evidence.py`](../../tests/test_vertical_14dof_evidence.py) and produced by
[`tools/gates/check_14dof_simple_gates.py`](../../tools/gates/check_14dof_simple_gates.py).

> **Superseded evidence (2026-10-17).** The suspension was refactored into kinematics, spring, ARB and damper components behind one `suspension-forces` capability (see `docs/superpowers/plans/2026-10-17-suspension-layouts.md`). The gate tool pins the sha256 of the source files, so the acceptance evidence was regenerated once from the refactored sources. The earlier attempts `14dof_simple_trials/attempt_1792190392697144000`, `periodicity/attempt_1792190828124776000`, `refinement/attempt_1792190567221803000` and the planar reference `seven_dof_track_pair/original/attempt_1792187768241214000` are **superseded**. They stay in the repository unchanged as history; their source hashes no longer match the working tree. The regenerated lap reproduces the old numbers (14DoF 66.56339750 s, 20 075 steps, 74 486 evaluations, minimum contact load 97.887 N; planar 66.53234595 s), so the refactor moved no result. Only the wall times differ (cold lap child wall 140.1 s against 138.0 s).

> **A transient tire model fitting tool is needed.** Every coefficient of the transient tire layer
> (`longitudinal_length_scale`, `lateral_length_scale`, `trail_pole_ratio`, `trail_lead_ratio`), the
> low-load contact window, the longitudinal relaxation law and the load-coupled carcass response are
> **PROPOSED**, unfitted placeholders. They are motivated by Ma et al. (DOI
> [10.3390/machines10070527](https://doi.org/10.3390/machines10070527)), whose coefficients are
> tire-specific, and the paper text was not independently re-verified. No measured transient tire data
> exists for this Hoosier. A fitting tool is required for the longitudinal relaxation length, the lateral
> carcass response, the trail pole and lead, and the contact transitions. **This plan does not build that
> tool**; until it exists, no measured-vehicle accuracy claim may be based on the transient layer.

> **Status of the lap evidence (read before quoting any number).** All three Task 9 attempts complete a
> lap and pass the seven plan gates of this document, but every attempt has `usable = false` under the
> wider 7DoF `acceptance_gate`: the driven lap misses the 0.25 m path-tracking cap (0.383 m), it is a
> cold, non-periodic transient at the acceptance attempt, and `between_step_validation_complete` is never
> certified. The vertical gates below establish numerical evidence about this model; they do not make the
> lap a validated, measured-data-backed result. The same is true of the corrected 7DoF reference lap it is
> compared with (0.366 m path error, `usable = false`).

## 1. Parameter table and sources

Source column key: **DSS cell** = read from `IEM_24_DSS.xlsx` by fixed cell address (sha256
`e436c2f61d46f23614515e34cbce381e45a5e508b9840012c88511f59afac5d8`); **user** = supplied by the user
decision list of the plan (2026-10-16); **PROPOSED** = chosen by the project without measured data.

### 1.1 Shared assembly parameters used by both rungs

| Parameter | Value | Source |
|---|---|---|
| `total_mass` | 287 kg (209 kg car + 78 kg driver) | existing assembly |
| `wheelbase` | 1.535 m | existing assembly |
| `track_width` | 1.240 m | user (D6); 1.016 m was an error |
| `cg_height` | 0.300 m | existing assembly |
| `front_mass_fraction` | 0.45 | user (D6); 0.55 was an error |
| `yaw_radius_of_gyration` | 0.679366 m | existing 7DoF measurement |
| `wheel_inertia` | 0.2064512 kg·m² | existing assembly (10 kg, 16 in, 8 in solid cylinder) |
| `tire_vertical_damping` | 400 N·s/m, all four tires | **user estimate** (D5) |

### 1.2 Vertical 14DoF-only parameters

| Parameter | Value | Source |
|---|---|---|
| `unsprung_mass_front`, `unsprung_mass_rear` | 13 kg per corner (52 kg total) | **PROPOSED** — not weighed |
| `roll_radius_of_gyration` | 0.193 m | **user estimate** (D5) |
| `pitch_radius_of_gyration` | 0.386 m | **user-reported measurement**, unvalidated (D5) |
| sprung mass | 235 kg | derived (287 − 52) |
| sprung CG height | 0.3211 m | derived, with the unsprung CG at `r_1` = 0.2045 m |
| sprung front fraction | (287·0.45 − 26)/235 = 0.4389 | derived |
| roll inertia `Ixx` | 8.754 kg·m² | derived (`m_s · 0.193²`) |
| pitch inertia `Iyy` | 35.01 kg·m² | derived (`m_s · 0.386²`) |

The sprung yaw inertia is taken from the 7DoF measurement minus the estimated four unsprung yaw
contributions (about 81.5 kg·m²). It is **not** consistent with `Ixx + Iyy` (43.8 kg·m²); no sprung yaw
radius is measured, and no radius was invented to hide the inconsistency.

### 1.3 Suspension, damper and geometry (generated LUTs)

| Item | Value | Source |
|---|---|---|
| Wheel rate, front / rear | 450 / 350 lbf/in = 78 807.08 / 61 294.39 N/m | **user-selected** (D4); DSS rows 18–19 intentionally overridden |
| Rear ARB wheel rate in pure roll | 120 lbf/in = 21 015.22 N/m | **user-selected** (D4) |
| Front ARB wheel rate | 0 | D4 |
| Damper motion ratio F / R | 1.05 / 1.10 | DSS C23 / F23 (first `<ratio>:1 heave` value) |
| Damper table | Multimatic DSSV VC01, 12 clicks × 13 velocity knots per half, ±0.5 m/s | source workbook, fixed click 8/+300 mm/s correction 671 → 761 N (D2) |
| Damper clicks (baseline) | compression 5, rebound 7 | **user-selected** (D1) |
| RC height static F / R | 41.275 / 57.15 mm | DSS C30 / F30 |
| RC height at 1 g F / R | 45.5 / 65.3 mm | DSS D31 / G31 |
| RC lateral at 1 g F / R | −6.0 / −8.0 mm | DSS E31 / H31 |
| Anti (brake, drive) F | (0.542, 0.320) | DSS C29 (first brake, second drive, D9) |
| Anti (brake, drive) R | (0.574, 0.550) | DSS F29 |
| Camber gain F / R | 25.1 / 45.3 deg/m | DSS C24 / F24 (stored for `14dof-complete`, not used in forces) |
| Jounce travel | ±31.8 mm | DSS D17/E17/G17/H17 — hard acceptance limit |
| Spring preload | sprung static corner load at jounce 0: 505.778 N F, 646.503 N R | derived from the corrected geometry and PROPOSED unsprung mass |
| Bump/droop stops | **PROPOSED**: 10× wheel rate from 29.8 mm, C1 smoothstep blend over 27.8–29.8 mm, both directions | project choice; DSS travel stays the limit |
| Inertance F / R | 0 kg | **PROPOSED** (D5) |
| Tire vertical stiffness at static load | 80.4 N/mm F, 80.7 N/mm R | from the UniTire loaded-radius fit (D7), not a new parameter |
| Tire vertical law low-load ramp | `delta1` = 0.5 mm, `F1` = 39.70 N, `delta0` = −0.4996 mm; 9.92 N ghost load at zero deflection | numerical device, accepted and disclosed |

### 1.4 Steering: static toe and jounce steer (D13)

* **Static toe is 0° at both axles** (DSS cells C26 and F26 give 0). `static_toe_front` and
  `static_toe_rear` are both 0 rad in the suspension XML.
* **The jounce-steer curves are derived from the suspension hardpoints** (from real car data) and are small:
  front -0.205° to +0.005°, rear -0.103° to +0.026° over -31.8 to +31.8 mm, with an exact 0° at 0 mm.
  Each axle has 13 knots, and the NPZ metadata and XML comments record the source as *"from real car data"*.
  The derivation reproduces the tabulated camber gains (25.1 and 45.3 deg/m), which cross-checks the points. The
  generator still writes a zero curve when it is given no knots, and it enforces strictly rising jounce, finite
  values, an explicit 0 mm / 0° reference and a knot at each travel limit.
* At rest `corner_toe_angles` returns four zeros; in motion the 14DoF wheel angles equal the planar
  Ackermann/driver angles plus the derived bump steer, at most about 0.2°. A synthetic +0.5° front toe-in curve at +20 mm is tested to change
  only the FL wheel angle, its contact velocity, its slip angle and the rotation of its force
  (`tests/test_vertical_14dof.py`).

### 1.5 Suspension layout extensions (Task 6)

The reference acceptance evidence continues to use the independent-corner layout;
its numbers and state count remain unchanged. The following layouts are now
available for setup studies:

| Assembly subtype | State contribution | Status |
|---|---:|---|
| `independent-corner` | 0 | reference, numbers pinned |
| `axle-roll-heave` | 0 | linear modal approximation |
| `four-mode` | 0 | linear modal approximation |
| `hydraulic` | one gas fraction per accumulator node | **PROPOSED** circuit sizing example |

The modal examples use wheel-referred mode rates and mode damper velocities with
no additional motion ratio. Consequently their per-axle ride frequencies and
corner damping ratios are diagonal approximations, not a nonlinear full-car mode
fit. The four-mode roll frequency saturates with rate because the tires remain in
series (9.90 to 10.41 Hz over the documented rate sweep). The axle example's
front ride frequency is 4.283 Hz; its rear ride frequency is 3.74 Hz versus 3.59
Hz for the independent layout because the diagonal includes the rear ARB. Its
axle heave frequency is 4.072 Hz versus 3.989 Hz for the independent layout.

The hydraulic example is `models/suspensions/iem24_roll_decoupled_hydraulic.xml`.
Its circuit graph is pinned in `models/data/iem24_roll_decoupled_circuit.npz` and
loaded from `models/hydraulics/iem24_roll_decoupled_circuit.xml`. Cylinder areas,
precharge, gas volume, polytropic exponent and orifice coefficients are all
**PROPOSED**; they are not measured hydraulic data. The circuit uses algebraic
orifice flow and one integrated gas-volume fraction per accumulator. A state outside
its accumulator range is a named network-state gate failure, rather than silent
extrapolation. The 60 m smoke run completed with zero out-of-range evaluations,
zero lifted-wheel evaluations and a 786.4 N minimum contact load; wall time was
4.6 s on the development machine, compared with 3.8 s for the corresponding
independent-corner smoke run.

### 1.6 Transient tire layer — equations and compiled coefficients

File `models/tires/iem24_murphy_real_unitire_transient.xml` (`tire-model/unitire-transient-simple`,
capability `transient-combined-slip`), module
[`src/simplified_vehicle_models/tires/unitire_transient.py`](../../src/simplified_vehicle_models/tires/unitire_transient.py).
The steady force law `evaluate_unitire` and its MATLAB fixture are **unchanged**; the transient layer is a
separate three-state-per-corner component layered on top of it.

**Paper:** Ma et al., *The Unsteady-State Response of Tires to Slip Angle and Vertical Load Variations*,
Machines 2022, 10(7), 527, **DOI 10.3390/machines10070527**. The plan cites equations 52–55, 60–61, 64 and
69–70. **The paper text could not be fetched when the layer was written** (publisher returned HTTP 403);
the equation numbers and the statements about the paper below come from the project plan and were not
re-verified. The equations are this project's reduced forms motivated by those equations, not quotations.

| State (per wheel) | Symbol | Unit |
|---|---|---|
| delayed longitudinal slip | `kappa_eff` | 1 |
| lateral carcass deformation | `lateral_deformation` | m |
| trail angle | `trail_angle` | rad |

Twelve tire states for four wheels, separate from the 14 mechanical DoFs (`3 × 4 = 12`; the plant is
therefore 38 states, or 39 with the optional pedal state).

| Law | Equation | Status |
|---|---|---|
| Lateral (reduced form motivated by eq. 61) | `sigma_y(Fz) = lateral_length_scale · k_y(Fz)/k_cy`; `alpha_eff = y/sigma_y(Fz)`; `dy/dt = max(\|Vx\|, 0.1) · (alpha_cmd − alpha_eff)` | paper-based structure, unfitted coefficient |
| Trail (approximation motivated by eq. 64, 69–70) | `d alpha_t/dt = max(\|Vx\|, 0.1)·(alpha_eff − alpha_t)/sigma_trail`, `sigma_trail = sigma_y/trail_pole_ratio`; `t_eff = t(alpha_t) + trail_lead_ratio·(t(alpha_eff) − t(alpha_t))`; `Mz = −(Fy·(t_eff + Fx/k_cx) − Fx·Fy/k_cy)` | approximation; lead correction is zero at steady state |
| Longitudinal | `sigma_x(Fz) = longitudinal_length_scale · k_x(Fz)/k_cx`; `d kappa_eff/dt = max(\|Vx\|, 0.1)·(kappa_cmd − kappa_eff)/sigma_x` | **PROPOSED extension, not in the paper** |
| Contact window | `w(Fz) = 3s² − 2s³`, `s = clip((Fz − 0.5)/1.5, 0, 1)`: 0 at `Fz ≤ 0.5 N`, 1 at `Fz ≥ 2 N`. Multiplies Fx, Fy, Mz and the overturning moment; blends each state rate to `−state/0.01 s` below the window | project choice |
| Forces | Fx, Fy and the rolling-resistance moment from `evaluate_unitire(steady, alpha_eff, kappa_eff, Fz, \|omega\|)`; the separate vertical contact force is never windowed or invented here | unchanged steady law |

Compiled coefficients (all four are **PROPOSED**, the XML comments say so):

| Parameter | Value | Basis, and what it is not |
|---|---|---|
| `longitudinal_length_scale` | 1.0 | unit scale on `k_x(Fz)/k_cx`; an engineering guess, no longitudinal data. σ_x ≈ 0.1195 m at 660 N |
| `lateral_length_scale` | 1.0 | unit scale on the steady `k_y/k_cy` relaxation length, not fitted to transient data. σ_y ≈ 0.1533 m at 660 N |
| `trail_pole_ratio` (m) | 1.0 | neutral value: the trail lags the effective angle by one lateral relaxation length; the published m is tire-specific |
| `trail_lead_ratio` (n) | 0.25 | modest lead chosen without data to approximate the zero of eq. 64; the published n is tire-specific |

Reduced-model limits (carried from the Task 4A report): the paper uses a secant-stiffness inversion and
this layer uses the steady `k_y/k_cy` length, so the carcass response does not slow near force saturation;
there is no `dFz/dt` term and no nested root; the longitudinal law, contact window, 0.01 s relaxation and
all four coefficients are unfitted; the trail pole/lead are first order; below 0.1 m/s all three rates are
frozen at the 0.1 m/s rate. The caller's 1 m/s slip-computation floor is separate from the tire's internal
`vroll_min = 0.1 m/s`.

## 2. Reference numbers the rung is tested against

From the plan's reference table (D4–D7) and the Task 6 acceptance gates
([task-6-acceptance-report.md](../superpowers/sdd/2026-10-16-vertical-14dof-model/task-6-acceptance-report.md)):

| Quantity | Front | Rear |
|---|---|---|
| Sprung mass per corner | 51.6 kg | 65.9 kg |
| Tire vertical stiffness at static load | 80.4 N/mm | 80.7 N/mm |
| Ride frequency (spring in series with tire) | 4.42 Hz | 3.66 Hz |
| Wheel-hop frequency (undamped, body-fixed) | 17.61 Hz | 16.63 Hz |
| Damping ratio, bump c5 / rebound c7 at 25 mm/s damper velocity | 0.49 / 0.78 | 0.43 / 0.68 |
| Roll stiffness at the wheels | 1057 N·m/deg | 1104 N·m/deg |
| Roll gradient (rigid tire, static RC, reference oracle) | 0.289 deg/g (oracle reproduces 0.2905–0.2923; closed form 0.28968) | |
| Steady-state front LLTD at 1 g (rigid tire, static RC) | 0.473 (oracle 0.477–0.481; closed form 0.47399) | |
| Production roll gradient / front LLTD (compliant tire, measured RC map) | 0.66–0.67 deg/g / 0.486–0.489 | |

Measured full-vehicle undamped modes (Task 6 acceptance gates, mass-weighted participation):

| Frequency | Label | Against the plan |
|---|---|---|
| 3.83148 Hz | heave | **inside** the 3.4–4.7 Hz plan band |
| 7.29886 Hz | pitch | **OUTSIDE** the plan band — see disclosure D1 |
| 9.89547 Hz | roll | un-gated (not named by the plan) |
| 17.17610, 17.71280, 19.46531, 23.96987 Hz | hop set | full-vehicle, body-coupled; see the body-fixed set below |
| 17.613839 (F), 16.630917 (R symmetric), 17.819518 (R warp) Hz | body-fixed hop set | matches the plan's 17.61 / 16.63 Hz references within 3 % |

## 3. Task 9 acceptance gates — measured

Command (base lap; the paired runs use the same command with `--warm-start` and their own
`--output-root`):

```bash
PYTHONPATH=src:. python3 tools/run/run_iem24_forward_driver.py --model 14dof-simple \
  --track models/tracks/2024_michigan_endurance_closed.xml --planner live --maximum-speed 60.0 \
  --pedal-travel-time 0.175 --maximum-step 0.005 --rtol 1e-05 --method BDF --spacing 0.5 \
  --duration 300.0 --timeout-seconds 1800.0 \
  --output-root docs/validation/iem24-murphy/14dof_simple_trials
PYTHONPATH=src:. python3 tools/gates/check_14dof_simple_gates.py \
  --attempt      docs/validation/iem24-murphy/14dof_simple_trials/attempt_1792288406878879000 \
  --periodicity  docs/validation/iem24-murphy/14dof_simple_trials/periodicity/attempt_1792288811970287000 \
  --refinement   docs/validation/iem24-murphy/14dof_simple_trials/refinement/attempt_1792288547293860000 \
  --planar-reference docs/validation/iem24-murphy/seven_dof_track_pair/original/attempt_1792288979150821000
```

The accepted 1800 s guard was not needed: the cold lap used 138.0 s of child wall time (solver 125.8 s of
a 1764 s budget). No 300 s threshold is applied to this rung (D12); the 7DoF 300 s limit was not touched.

| Plan gate | Runner key(s) | Verdict | Measured evidence |
|---|---|---|---|
| `completed` | `lap_completed`, `status`, elapsed ≤ requested | **PASS** | lap **66.56339750 s** over 1016.70064959 m; `status = Completed`; 20 075 accepted steps, 74 486 solver evaluations, 0 rejected domain trials; solver 128.679 s of 1764 s budget; child elapsed **140.113 s ≤ 1800 s**; peak child RSS 1 651 589 120 B (1.65 GB) |
| `contact_valid` | `contact_valid` | **PASS** | minimum tire load **97.887 N ≥ 0**; lifted-wheel evaluations **0**; `out_of_range_evaluations` 0; travel valid |
| `travel_valid` | `jounce_travel_valid` | **PASS** | accepted jounce (nodes + per-interval dense bounds) **[−5.8023, +7.1369] mm** inside ±31.8 mm; trace-node extreme ±7.13688 mm; **bump-stop entries 0** (onset 27.8 mm); peak wheel force 1083.95 N (rear at max compression), minimum wheel force 48.52 N |
| `table_domain_valid` | `out_of_range_evaluations == 0` | **PASS** | `out_of_range_evaluations` **0** over every plant evaluation including rejected trials (damper, spring, bump-stop, jounce-steer); max damper velocity **0.09253 m/s** of a 0.5 m/s table (81.5 % headroom); max \|roll\| **1.1925°** < 5° table limit, so the roll-centre clamp never bound; jounce-steer baseline toe 0 |
| `refinement_validated` | paired warm baseline vs refinement | **PASS** | warm baseline 66.58945362 s vs refinement 66.58945439 s → **Δ 7.65e-7 s ≤ 5 ms**; max \|Δv\| at 1017 matched s samples **9.36e-05 m/s ≤ 0.05**; worst matched tire state `kappa_eff_rl` ratio **0.0367 ≤ 1** (largest tire-state Δ 7.34e-05 against a 2e-3 tolerance); same initial state and source hashes; refined settings `maximum_step` 0.0025 s, `rtol` 1e-6 |
| `periodicity_valid` | `periodicity_valid` of the warm attempt | **PASS** | warm-started lap 66.58945362 s, closure ratio **0.01098 ≤ 1** (worst state `n`: 5.49e-06 m against its 5e-4 m tolerance); all 36 periodic states inside tolerance; tolerances are the plant's own vector (0.5 mm length, 0.005 rad angle/rate, 0.01 m/s velocity, tire states 0.002 / 0.2 mm / 0.002 rad) |
| `cross_fidelity_reported` | vertical vs corrected 7DoF traces | **PASS** (report item) | 14DoF **66.56339750 s** vs corrected 7DoF **66.53234595 s** → **+0.03105 s**; on the **tracked 200-sample grid** (the CSV beside the gate table) speed RMSE 0.032862 m/s (0.22 % of the 15.11 m/s mean) and max \|Δv\| **0.207638 m/s at s = 326.98 m**; on the **finer 2001-sample grid** RMSE 0.033866 m/s and max \|Δv\| 0.257727 m/s at s = 452.43 m; both grids are recorded in `gates.json`; see §4 |

The seven plan gate names and their booleans are also collected in `gates.json` as `plan_gates`, because
four of those names (`completed`, `travel_valid`, `table_domain_valid`, `cross_fidelity_reported`) are not
`summary.json` keys: the runner records `lap_completed`, `jounce_travel_valid` and
`out_of_range_evaluations` instead, and `refinement_validated` needs a second integration. `gates.json`
also records the gate tool's path and sha256 (checked against the working file by the evidence test), the
sha256/size/rows/columns of the four traces the trace-derived evidence came from, and the
recorded-vs-working source hashes of the cold attempt and of the planar reference.

Supporting lap measurements (cold acceptance attempt): mean speed 15.110 m/s, min/max 8.850/22.702 m/s;
heave 8.65–16.38 mm; roll **−1.088° to +1.193°**; pitch −0.650° to +0.229°; jounce rate ≤ 0.0972 m/s;
wheel speeds 35.92–122.34 rad/s; steering peak 14.73°; `kappa_eff` ∈ [−0.557, +0.190]; max lateral
carcass deformation 39.5 mm; max trail angle 0.1576 rad; final SOC 0.80538; `dense_defect_scaled_max`
88.8; `maximum_motor_speed_ratio` 0.704; `speed_regulated_evaluations` 0.

Paired attempts: the periodicity baseline used 146.30 s of child wall (solver 134.52 s, 19 949 steps,
1.65 GB peak RSS); the refinement used 260.69 s (solver 247.79 s, 38 744 steps, 140 635 evaluations,
**3.08 GB peak RSS** — the largest single-run memory of the three).

## 4. Cross-fidelity comparison with the corrected 7DoF lap

Reference: `docs/validation/iem24-murphy/seven_dof_track_pair/original/attempt_1792288979150821000`
(Task 8 Step 5, corrected 1.240 m track / 45 % front geometry), the same track and driver settings.

| Quantity | 14DoF-simple | 7DoF corrected |
|---|---|---|
| Lap time | 66.56339750 s | 66.53234595 s |
| Accepted steps / evaluations | 20 075 / 74 486 | 11 586 / 36 460 |
| Solver wall / child wall | 128.68 s / 140.11 s | 89.50 s / 100.11 s |
| Peak child RSS | 1.65 GB | 0.399 GB |
| Minimum contact load | 97.887 N | 131.377 N |
| Maximum lateral path error | 0.38302 m (**fails 0.25 m**) | 0.36569 m (**fails 0.25 m**) |
| `path_tracking_valid` / `usable` | false / false | false / false |
| `periodicity_valid` (cold) | false | false |
| `refinement_validated` | **true** (warm pair) | false (no pair) |
| `between_step_validation_complete` | false | false |

Speed trace on **two explicitly labelled matched-distance grids**, both recorded in `gates.json`:

| Grid | Samples | Speed RMSE | max \|Δv\| | max at | max \|Δv\| after s = 508 m |
|---|---|---|---|---|---|
| Tracked CSV `cross_fidelity_speed.csv` | 200 | 0.032862 m/s | **0.207638 m/s** | **s = 326.98 m** | 0.014226 m/s |
| Finer in-memory grid | 2001 | 0.033866 m/s | **0.257727 m/s** | **s = 452.43 m** | 0.015673 m/s |

Mean signed difference −0.0017 m/s. The difference is concentrated in the first half of the lap (on the
finer grid, max \|Δv\| 0.2577 m/s over s = 254–508 m) and stays below 0.016 m/s after s = 508 m **on both
grids** — the tail values above are the ones to quote, at 200 samples or at 2001.

Every number in this section comes with the grid it was measured on: quoting the 0.2076 m/s extreme
against s = 452 m, or the 0.2577 m/s extreme against s = 327 m, would mix two sampling densities.

The two laps were also integrated with **different maximum steps** (14DoF 0.005 s, 7DoF 0.02 s) and
different `rtol` (1e-5 for both), so part of the residual difference in the first half — where the two
trajectories have not yet converged onto the same line — is integration resolution, not model structure.
The comparison therefore says "the two rungs produce a lap 31 ms apart with a speed trace within about
0.03 m/s RMSE", not "the LLTD and transient effects are 0.03 m/s".

The corrected 7DoF reference was produced by the **Task 8 runner before its final review-fix round**: its
recorded runner sha256 `4a51d3d5…` matches no commit, while all 13 of its other recorded sources match the
working tree. `gates.json.planar_reference_source_hash_agreement` records this explicitly (one mismatch of
14 sources, the runner) and no gate fails on it, because the review fixes were behaviour-preserving for
the planar path — the planar regression fixture test (`tests/test_forward_driver_runner.py`, recorded from
the pre-change runner) passes unchanged. Re-running the reference with the current runner would be a new
attempt directory with new timings (about 100 s of child wall time, plus the exporter), and that is
deliberately not done here: the comparison uses the committed attempt.

Interpretation, and what it does **not** show:

* The plan's expected effects are the 0.473 rigid-tire front LLTD and the measured roll-centre geometry
  against the planar model's fixed 0.42 (more front lateral load transfer, earlier front saturation) plus
  the tire and damper transients the planar quasi-steady tire has no equivalent of. Both act on lateral
  grip, so a speed-trace difference is expected even though the two rungs share the steady UniTire
  coefficients, aero, powertrain and driver.
* This comparison cannot isolate those effects. Both laps are **driven transients with different lateral
  path errors** (0.383 m vs 0.366 m) and non-periodic cold starts, so the 0.031 s lap-time difference and
  the 0.033 m/s speed RMSE mix model structure with a different driven line and a different start. The
  first-half concentration is consistent with that (the two laps track the path differently before they
  settle). Nothing here is tuned, and no separation of the LLTD and transient effects is claimed.

## 5. PROPOSED values (complete list)

1. `unsprung_mass_front` = `unsprung_mass_rear` = **13 kg** per corner — not weighed.
2. `inertance_front` = `inertance_rear` = **0 kg**.
3. Bump and droop stop onset **27.8–29.8 mm** with a **10× wheel-rate** rise, C1 smoothstep blend, both
   directions.
4. Jounce-steer curves: **zero at both axles** (baseline placeholder, no measured data).
5. Transient tire: `longitudinal_length_scale` **1.0**, `lateral_length_scale` **1.0**,
   `trail_pole_ratio` **1.0**, `trail_lead_ratio` **0.25**.
6. Transient tire contact window **0.5–2 N** (cubic smoothstep), low-load state relaxation **0.01 s**,
   rate speed floor **0.1 m/s**.
7. Longitudinal relaxation law — a project extension, explicitly **not** from the cited paper.
8. Tire vertical law low-load ramp (`delta0` = −0.4996 mm, `delta1` = 0.5 mm) and its **9.92 N ghost load**
   at zero deflection — a numerical device, not physics.
9. Roll-centre map above the 1 g DSS point: height and lateral held constant; queries clipped at the
   committed 5° table maximum.
10. Anti-geometry scaling `(h_cg/L)/phi_ref` (the implementer's recorded choice in
    `task-6-roll-report.md`); the total tire load transfer is unaffected by this choice.
11. `tire_vertical_damping` 400 N·s/m and `roll_radius_of_gyration` 0.193 m are **user estimates**, not
    measurements.

## 6. Disclosures and open issues

* **D1 — pitch mode outside the plan band.** The full-vehicle pitch mode is **7.2989 Hz**, outside the
  plan's 3.4–4.7 Hz ride band. The ledger ruling attributes this to the D5 radii of gyration
  (pitch 0.386 m, roll 0.193 m): with those inertias the geometry-implied body inertias are 3.9×/10× larger
  than the quarter-car reference assumes, so 7.30 Hz is what the parameters imply. The roll mode
  (9.8955 Hz) is also un-gated. The radii remain **unvalidated**; no spring, ARB, tire or inertia value was
  tuned, and the plan's pitch band is **not** claimed to pass. The Task 6 tests assert the deviation is
  outside the band so it cannot silently become a pass.
* **D2 — the reference lap is not an accepted lap, and its runner is not the final one.** The corrected
  7DoF reference is `path_tracking_valid = false` (0.366 m > 0.25 m) and `periodicity_valid = false`, hence
  `usable = false`. The 14DoF acceptance lap also fails the 0.25 m path-tracking cap (0.383 m). All
  cross-fidelity numbers are therefore between two **non-accepted driven transients**. Nothing was tuned to
  pass a gate. In addition, the reference's recorded runner sha256 `4a51d3d5…` matches no commit: it was
  produced by the Task 8 runner before that task's final review fixes, while all 13 of its other recorded
  sources match the working tree. `gates.json.planar_reference_source_hash_agreement` records the one
  mismatch; no gate fails on it, because the fixes were behaviour-preserving for the planar path (the
  pre-change planar fixture regression passes unchanged). Re-running the reference would cost about 100 s of
  child wall time plus the exporter and would create a new attempt directory; that is deliberately not done.
* **D3 — aero-drag-couple planar-reduction gap.** The literal aero-enabled planar-reduction comparison
  fails at 1e-2 (yaw acceleration 22 % at 5 m/s, 111 % at 30 m/s) because the planar quasi-static load rule
  books `m·a·h_cg/L` and omits the aero drag couple `D·(h_s − h_cg)/L` that the vertical model carries
  (measured front-axle gap matches `D·h_s/L` to 1e-14 relative). The vertical model matches the rigid-body
  moment balance; the planar files were not changed (global constraint). Task 6's reduction gate therefore
  uses the aero-zeroed comparison, and nothing in Task 9 compares drag-on planar and vertical models at
  1e-2.
* **D4 — gravity constant mismatch.** `vertical_14dof` / `vertical_equilibrium` use **9.80665 m/s²** while
  `core.conventions.GRAVITY` (used by the planar quasi-static load law) is **9.81**. The offset is a
  constant 0.4327 N on the front axle (3.4e-4 relative) and about 1 N in the total load. It does not move
  any vertical gate.
* **D5 — inertance tolerance inconsistency.** The plan's parenthetical "unchanged to < 0.2 % (e.g. 3/5 kg)"
  is internally inconsistent: measured at b = 3/5 kg the heave mode shifts **−0.855 %** (the hub is
  10–15 % heavier; the mode has 4.7 % of its energy in jounce). The gate uses b = 0.3/0.5 kg for the 0.2 %
  assertion and separately asserts the 3/5 kg shift is between 0.2 % and 5 %. Inertance is **0** in the
  compiled vehicle, so the mass-matrix path is exercised only in tests.
* **D6 — tire stiffening ×1e4 is not representable.** The plan's "very stiff tires" clause in the reduced
  oracle cannot be implemented in a test without changing the evaluator's hard-coded 0.5 mm ramp knot
  (a ×1e4 loaded-radius scale moves the static deflection below `delta0`, where the elastic load is
  exactly zero and no equilibrium exists). The reference oracle uses tires stiffened ×1/300 and
  stiff springs, which is documented, not a weakened tolerance.
* **D7 — production roll gradient is unexplained.** The compliant-tire/measured-RC roll gradient
  (0.66–0.67 deg/g) is 16 % above the naive spring‖tire series estimate (0.579 deg/g). No roll-moment
  budget explains the gap, so the 0.66–0.67 deg/g value is reported as an open issue and is **not** an
  acceptance value.
* **D8 — inertia tensor inconsistency.** The sprung yaw inertia (about 81.5 kg·m²) is not consistent with
  `Ixx + Iyy` = 43.8 kg·m² because no sprung yaw radius is measured. The planar reduction test is not
  forced to reconcile them.
* **D9 — transient tire layer is provisional.** See the box at the top: all four coefficients, the contact
  window and the longitudinal law are unfitted; the paper text was not re-verified (publisher 403). A
  **transient tire model fitting tool is needed** and is not built by this plan.
* **D10 — the lap is not between-step certified.** `between_step_validation_complete` is false on every
  attempt (the runner never certifies it), so `usable` stays false even where periodicity and refinement
  pass. `dense_defect_scaled_max` was 88.8 on the cold lap — a diagnostic of the interpolant, not a bound.
* **D11 — memory.** Peak child RSS grows with the accepted-step count (1.65 GB cold, 3.08 GB refined at
  half the step). `peak_memory_bytes` is a process high-water mark including numpy/SciPy/BLAS and the
  stored dense polynomials, not the solver's own allocation. A longer 14DoF lap at a small step needs
  memory headroom.
* **D12 — `contact_valid` is a weak gate, and its load half is vacuous by construction.** The evaluator
  clamps the tire normal load, `loads = max(elastic + damping, 0)` (`vertical_14dof.py`), and marks
  `lifted = loads <= 0`, so a *negative* normal load cannot be represented in the first place: the
  `minimum_contact_load >= 0` condition can never fail, and a clamped wheel shows up as a lifted-wheel
  evaluation instead. The gate's real content is the table-domain counter, the travel bound and the
  accepted-trajectory replay, and it cannot localize which corner is at fault. On this lap no wheel lifted
  (0 lifted-wheel evaluations, minimum load 97.887 N), so the clamp never bound.
* **D13 — `out_of_range_evaluations` also counts solver trials**, so the table-domain gate is stricter
  than an accepted-state-only count; the counter does not separate damper-velocity exits from spline
  domain exits (the exception message and the evaluator's own counter do). The roll-centre part of that
  gate is a **proxy**: the evaluator silently clips the roll-centre query at the committed 5° table
  maximum (`roll_query = clip(|roll|, 0, 5°)`) and publishes no counter, so the gate can only evidence that
  the clip was never active (max \|roll\| 1.1925° here), not that no RC query left the table. A run with
  \|roll\| above 5° would need an explicit out-of-range counter for the RC table.
* **D14 — the 0.25 m path-tracking failure is a driver/plan property of both rungs**, not a vertical-model
  effect: the corrected car is faster than the plan's speed profile assumed, and the unmodified preview
  driver overshoots. No driver or plan parameter was tuned to make the gate pass.
* **D15 — the refinement criteria bound the warm-start pair, not the cold lap's global error.** The
  refinement and the periodicity baseline both start from the cold lap's final state, exactly as the 7DoF
  protocol does, so `refinement_validated` measures the step and tolerance sensitivity of the solution
  branch through that state. It does **not** bound the cold acceptance lap's own global discretization
  error, and it says nothing about the first lap's start transient. The cold lap is the one whose gates and
  measurements this document reports; its own resolution evidence is the accepted/dense-bound travel check,
  the `dense_defect_scaled_max` diagnostic (88.8, not a bound) and the un-certified
  `between_step_validation_complete` flag.
* **D16 — the cross-fidelity laps used different integration settings.** The 14DoF acceptance lap ran BDF
  with `maximum_step` 0.005 s and `rtol` 1e-5; the corrected 7DoF reference ran BDF with `maximum_step`
  0.02 s and `rtol` 1e-5. Together with the different driven line and start (D2), that difference is part
  of the 0.031 s lap-time gap and the first-half speed-trace difference, so the comparison is a
  like-configuration-driving comparison, not a step-converged model-to-model comparison.
* **D17 — the trace-based gate evidence needs the gitignored traces.** `trace.csv`, `pedal.csv`,
  `dense_polynomials.npz` and `run.log` are not committed, so a fresh checkout has `summary.json`,
  `parent.json`, `gates.json` and `cross_fidelity_speed.csv` only. `gates.json` records the sha256 of all
  four traces; the evidence tests re-derive the refinement, periodicity and cross-fidelity numbers from the
  traces when they are present and match their pinned digest, and skip with a message naming what to
  regenerate when they are absent. A trace that is present but does not match its pinned digest fails the
  provenance test (tampering or a stale pin), so a re-run set must always be re-pinned by re-running the
  gate tool.

## 7. Reproduce

```bash
# From the repository root, with the corrected vehicle and the source files recorded in the attempt.
# 1. Cold acceptance lap (about 2.5 minutes of wall time).
PYTHONPATH=src:. python3 tools/run/run_iem24_forward_driver.py --model 14dof-simple \
  --track models/tracks/2024_michigan_endurance_closed.xml --planner live --maximum-speed 60.0 \
  --pedal-travel-time 0.175 --maximum-step 0.005 --rtol 1e-05 --method BDF --spacing 0.5 \
  --duration 300.0 --timeout-seconds 1800.0 \
  --output-root docs/validation/iem24-murphy/14dof_simple_trials
BASE=$(ls -dt docs/validation/iem24-murphy/14dof_simple_trials/attempt_* | head -1)
# 2. Refinement (same warm start, tighter step and rtol) and 3. periodicity baseline (same warm start,
#    cold-lap settings), about 4.5 and 2.5 minutes.
PYTHONPATH=src:. python3 tools/run/run_iem24_forward_driver.py --model 14dof-simple \
  --track models/tracks/2024_michigan_endurance_closed.xml --planner live --maximum-speed 60.0 \
  --pedal-travel-time 0.175 --maximum-step 0.0025 --rtol 1e-6 --method BDF --spacing 0.5 \
  --duration 300.0 --timeout-seconds 1800.0 --warm-start "$BASE" \
  --output-root docs/validation/iem24-murphy/14dof_simple_trials/refinement
PYTHONPATH=src:. python3 tools/run/run_iem24_forward_driver.py --model 14dof-simple \
  --track models/tracks/2024_michigan_endurance_closed.xml --planner live --maximum-speed 60.0 \
  --pedal-travel-time 0.175 --maximum-step 0.005 --rtol 1e-05 --method BDF --spacing 0.5 \
  --duration 300.0 --timeout-seconds 1800.0 --warm-start "$BASE" \
  --output-root docs/validation/iem24-murphy/14dof_simple_trials/periodicity
# 4. Gate table (writes gates.json into the cold attempt).
PYTHONPATH=src:. python3 tools/gates/check_14dof_simple_gates.py --attempt "$BASE" \
  --periodicity  $(ls -dt docs/validation/iem24-murphy/14dof_simple_trials/periodicity/attempt_* | head -1) \
  --refinement   $(ls -dt docs/validation/iem24-murphy/14dof_simple_trials/refinement/attempt_* | head -1) \
  --planar-reference docs/validation/iem24-murphy/seven_dof_track_pair/original/attempt_1792288979150821000
# 5. Evidence test.
PYTHONPATH=src:. python3 -m unittest tests.test_vertical_14dof_evidence -q
```

Attempt directories are timestamped with nanoseconds: a fresh reproduction creates new directories and new
timings. `trace.csv`, `pedal.csv`, `dense_polynomials.npz`, `run.log` and `source_snapshot/` are
gitignored per-attempt artifacts, so a fresh checkout carries `summary.json`, `parent.json`, `gates.json`
and `cross_fidelity_speed.csv` but must regenerate the traces to re-run the comparisons.

**Step 4 is not optional after a lap, and step 4 must be repeated whenever a new attempt appears in
`14dof_simple_trials/`.** The evidence test selects the newest `attempt_*` that carries a `gates.json`; if
the newest attempt has none, the test fails with a message naming the command above (never a bare
`FileNotFoundError`, and never a silent pass on an older table). Running the gate tool rewrites
`gates.json` (and the `cross_fidelity_speed.csv` beside it) and re-pins the four trace sha256 values, which
is what makes the trace-pinned evidence tests able to re-derive the refinement, periodicity and
cross-fidelity numbers; when a trace is missing or no longer matches its pin, those tests skip with a
message naming what to regenerate, and a present-but-changed trace fails the provenance test.

Task 9 full record: [task-9-report.md](../superpowers/sdd/2026-10-16-vertical-14dof-model/task-9-report.md).
