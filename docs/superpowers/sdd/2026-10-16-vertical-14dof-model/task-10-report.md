# Task 10 report — damper, anti-roll and inertance setup sweep tool

*(The plan's file and function names — `tools/run/sweep_vertical_setup.py`,
`sweep_vertical_setup` — are kept. A **spring-rate override is not available**:
the wheel-rate curve is a spline inside the sha256-pinned corner NPZ, not an XML
scalar. See Open issue 1.)*

## Status

Implemented and verified with focused tests only (the parent runs the full suite and
commits). Nothing committed, merged, or pushed. Files:

| File | Change |
|---|---|
| `tools/run/sweep_vertical_setup.py` | NEW — `sweep_vertical_setup`, `SweepResult`, `apply_overrides`, `valid_override_names`, `step_response`, `write_csv`, `write_json`, CLI |
| `tests/test_vertical_setup_sweep.py` | NEW — 33 tests |

Final hashes (after the review fix round): tool
`fc4409df713b3ab4a9d1a6adde9e18ae8d7d5baadc3b24c93496002acba22c62`, test
`e5d52879c37c13952895275f38cd02945b99f56ec92939e06bcf64da8f63b3bf`.

No other tracked file was touched. `docs/superpowers/plans/2026-10-16-vertical-14dof-model.md`
stays untracked, as required. No lap was run, no `docs/validation/**` file or attempt
directory was touched, no MIS report generator was run, and no library code under
`src/` was changed (so the tool is NumPy-only and SciPy-free).

## What was built

### 1. Override mechanics: `dataclasses.replace` on the compiled vehicle

`sweep_vertical_setup(formulation_xml, overrides, study)` compiles the formulation
**once** and then, per setup, rebuilds only what the override names name:

* a click rebuilds the `DamperSetting` of the affected `CompiledDamper` (the
  `DamperTable` — and therefore the sha256-pinned NPZ LUT — is shared, never
  rewritten), and `DamperSetting.validate` re-checks range and integrality;
* `arb_wheel_rate_front/rear`, `inertance_front/rear`, `static_toe_front/rear`
  `replace` the matching field of the compiled `AxleSuspension`.

This is the robust choice given Tasks 2-3: a click *indexes* into the
damper-force table (it is not an XML-text edit), so recompiling a modified XML
would add nothing except a re-read of the same pinned NPZ, while `replace` keeps
the table identity and lets the sweep run 12 setups in 0.25 s.

`valid_override_names(vehicle)` derives the accepted names from what the
formulation actually exposes:

| Name family | Source | Baseline value |
|---|---|---|
| `click_compression`, `click_rebound` | valve names of both compiled damper tables (bare name applies to every damper that has the valve) | 5 / 7 |
| `front.click_<valve>`, `rear.click_<valve>` | one damper only | 5 / 7 |
| `click_<valve>` for a 4-valve table | e.g. `click_lsc`, `click_hsc`, `click_lsr`, `click_hsr` | per synthetic XML |
| `arb_wheel_rate_front`, `arb_wheel_rate_rear` | compiled `AxleSuspension.arb_wheel_rate` | 0.0 / 21015.22 N/m |
| `inertance_front`, `inertance_rear` | compiled `AxleSuspension.inertance` | 0.0 kg |
| `static_toe_front`, `static_toe_rear` | compiled `AxleSuspension.static_toe` | 0.0 rad |

An unknown name raises `ValueError` listing the sorted valid names; a click of
12, −1, 6.5 or NaN raises `ValueError`; a negative ARB rate or inertance raises.
Spline-valued parameters (the spring/bump-stop wheel-rate curve, the roll-centre
map, the motion-ratio curve, jounce steer) are **not** overridable, because they
live in the sha256-pinned corner NPZ, not in an XML scalar — see Open issue 1.

### 2. Results derived from the model (Task 6 linearization + Task 7 step)

Every setup is evaluated at the speed-`--speed` static equilibrium
(`solve_static_equilibrium`, the Task 6 solver):

* **Mass** — the evaluator's own `vertical_mass_block` (7x7), which already
  carries the ruled hub-to-chassis `b/MR²` inerter term.
* **Stiffness** — central-difference Jacobian of the evaluator's
  `generalized_forces` (7x7), the same construction the Task 6 acceptance gates
  validate against the closed form.
* **Damping** — tire vertical damping between hub and ground (tire-stiffness
  structure) plus `slope / MR²` from each compiled damper at zero velocity
  (`slope` is the table's own derivative).
* **Modes** — heave/roll/pitch frequency and damping ratio from the eigenvalues
  of the 14x14 first-order form `[[0, I], [-M⁻¹K, -M⁻¹C]]`; each name is the
  distinct eigenmode with the largest mass-weighted participation in that
  coordinate, assigned greedily by descending participation. **Convention:** the
  reported `*_frequency_hz` is `|λ|/2π`, the undamped-equivalent natural frequency
  of the damped pole (baseline heave 3.9893 Hz), **not** `Im(λ)/2π` (3.9196 Hz);
  the `*_damped_frequency_hz` fields (3.91956 / 10.02833 / 7.72418 Hz for
  heave/roll/pitch) report `Im(λ)/2π` and are 0 for an overdamped mode. The
  damping matrix at zero velocity uses each damper table's **compression-branch**
  slope for both directions (the compiled table is kinked at `v = 0`); see the
  branch-convention ruling below for what that changes.
* **Per-axle ride frequency** — the lower mode of the exact two-DoF body-fixed
  quarter car `[[k_w, -k_w], [-k_w, k_w + k_t]]` on the corner masses, with the
  tangent spring rate and the tangent tire rate at the static deflection.
* **Corner damping ratio** — the secant figure of merit at the plan's 25 mm/s
  reference damper velocity: `zeta = (|F(v_ref)|/v_ref) / MR² / (2 m_s ω_ride)`.

**`step-response`**: a 2 000 N downward step on the front axle's sprung-body
points, applied as the generalized force `[F, 0, -F*x_front, 0, 0, 0, 0]` (the
Task 7 input). The response is the front body-point displacement
`heave + y_front*roll - x_front*pitch`, solved in **closed form with NumPy only**
(`z(t) = V diag((exp(λt)-1)/λ) V⁻¹ B F` on the 14x14 form) — no integrator and no
SciPy. Metrics: first peak, time to first peak, overshoot fraction above the
steady value, steady state, and settling time, defined explicitly as *the first
sample after which `|y - y_steady|` stays inside `0.02·|y_steady|` for every
remaining sample of the 1.2 s window* (the band and the definition are recorded in
the JSON).

**`frequency-response`**: displacement per generalized force
`H_ii(ω) = eᵢᵀ (K - ω²M + iωC)⁻¹ eᵢ` for heave, roll, and pitch, on a fixed grid
of 0.5-30 Hz with 3 001 points (9.83 mHz resolution), reported as the
maximum-magnitude grid point and its frequency. Units: m/N (heave),
rad/(N·m) (roll, pitch). The observable choice (displacement, not body
acceleration) is documented in the JSON and in Ruling 5.

### 3. CLI, CSV, and JSON

```
PYTHONPATH=src:. python3 tools/run/sweep_vertical_setup.py \
    --study step-response \
    --grid '{"click_compression": [0,1,2,3,4,5,6,7,8,9,10,11], "click_rebound": [7]}' \
    --output-csv /tmp/t10_click_sweep.csv --output-json /tmp/t10_click_sweep.json
```

`--formulation` (default `models/formulations/iem24_murphy_real_14dof_simple.xml`,
the Task 8/9 runner formulation), `--overrides-json` (a JSON list, or a path to a
JSON file) or `--grid` (a JSON object of a value or a list per name; the
cartesian product), mutually exclusive, `--study` (required, two choices),
`--speed`, `--step-force-n`, `--step-duration-s`, `--step-samples`,
`--settling-band`, `--frequency-{min,max}-hz`, `--frequency-samples`,
`--output-csv`, `--output-json`. `main` returns 2 and prints the message on an
invalid override or spec; argparse rejects a bad study or both sources.

The CSV has one row per setup: the union of override names as columns, then
`study`, then every metric (`""` for the study that is not selected). The JSON
summary holds `tool`, `tool_version` (`vertical-setup-sweep-v1`), `formulation`,
`study`, `study_definition` (observable, input, band, solver, grid, resolution,
`modal_frequency_convention`, `damping_branch_convention`), `source_hashes`,
`overrides`, and `results`. `source_hashes` is built from the XML graph
(formulation → assembly → every component XML → every `<data>` table they declare,
resolved absolutely, so a tmp synthetic formulation hashes the tmp files it
compiled) plus the **eleven** library sources the numbers depend on — including the
tool file itself, `compiler/compile.py` (the formulation compile the sweep calls)
and `tires/unitire_transient.py` (the transient tire state the static equilibrium
uses) — 25 hashed files in total for the baseline formulation.

## Commands and counts

| Command | Result |
|---|---|
| `PYTHONPATH=src:. python3 -m unittest tests.test_vertical_setup_sweep -q` | **33 tests OK in 1.1 s** |
| Step 2 evidence: the same command with the tool file absent | **1 error (ImportError), FAILED** — the module did not exist before Step 3 |
| `... tests.test_vertical_setup_sweep tests.test_repository_layout tests.test_mis_report_standard tests.test_damper_component tests.test_damper_table -q` | 79 tests OK in 1.7 s |
| `... tests.test_vertical_forward tests.test_vertical_14dof tests.test_vertical_equilibrium -q` | 82 tests OK in 65.8 s |
| `... tests.test_vertical_14dof_compile tests.test_suspension_corner tests.test_tire_vertical -q` | 67 tests OK in 0.5 s |
| The whole required set in one run (the five commands above, minus the Step 2 probe) | **231 tests OK in 65.8 s** |
| After the review fix round: `tests.test_vertical_setup_sweep tests.test_repository_layout tests.test_mis_report_standard -q` | **52 tests OK in 1.85 s** |
| After the review fix round: the whole required set again | **231 tests OK in 67.0 s**, exit 0 (`/tmp/t10_final_batch3.log`) |
| `PYTHONPATH=src:. python3 tools/run/sweep_vertical_setup.py --study step-response --grid '{"click_compression": [0..11], "click_rebound": [7]}' --output-csv … --output-json …` | 12 setups, exit 0, `elapsed: 0.24 s`, CSV 4 626 B, JSON 21 401 B |
| `git diff --check` | clean |

The full suite was intentionally not run (the parent's job, ~5 min), and no lap
was run.

### Runtime

| Sweep | Wall time |
|---|---|
| 12-setup `step-response` (`sweep_vertical_setup` only) | **0.25 s** (0.25/0.26/0.33 s over three warm runs) |
| 12-setup `frequency-response` | **0.245 s** (0.245/0.245/0.247 s) |
| 12-setup `step-response` including CSV + JSON | **0.25 s** (CSV 5 367 B, JSON 24 419 B) |
| 12-setup `step-response` via the CLI (process start included) | **0.24 s** internal `elapsed`, 0.37 s wall |
| 4-valve synthetic formulation: compile + 2-setup sweep | 0.014 s + 0.047 s |

The cost is dominated by `solve_static_equilibrium` (7 DoF Newton) plus 14
evaluator calls per setup; the step solution and the 3 001-point frequency grid are
vectorized and negligible. A lap is never run.

## Measured numbers

### Click sweep: compression 0-11 at rebound 7, baseline formulation, speed 0

`--study step-response`, `--grid '{"click_compression": [0..11], "click_rebound": [7]}'`
(the table is the tool's own CSV/JSON output; `zeta` = corner damping ratio at the
25 mm/s reference velocity, exact ride frequencies are 4.2830 / 3.5895 Hz for every
row).

| click | ζ corner F | ζ corner R | ζ F rebound | ζ R rebound | heave f (Hz) | ζ heave | roll f (Hz) | pitch f (Hz) | step peak (mm) | t_peak (s) | overshoot | settling (s) | steady (mm) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 0.3137 | 0.2668 | 0.8104 | 0.6892 | 4.0020 | 0.1902 | 10.2087 | 7.9164 | 32.3891 | 0.0797 | 0.2887 | 0.5434 | 25.1333 |
| 1 | 0.3268 | 0.2779 | 0.8104 | 0.6892 | 3.9968 | 0.1886 | 10.2049 | 7.9057 | 32.4536 | 0.0797 | 0.2912 | 0.5446 | 25.1344 |
| 2 | 0.3529 | 0.3002 | 0.8104 | 0.6892 | 3.9868 | 0.1853 | 10.1972 | 7.8838 | 32.5842 | 0.0796 | 0.2963 | 0.5471 | 25.1370 |
| 3 | 0.3921 | 0.3335 | 0.8104 | 0.6892 | 4.0033 | 0.1906 | 10.2096 | 7.9191 | 32.3730 | 0.0797 | 0.2881 | 0.5431 | 25.1330 |
| 4 | 0.4313 | 0.3669 | 0.8104 | 0.6892 | 3.9880 | 0.1857 | 10.1982 | 7.8866 | 32.5678 | 0.0796 | 0.2956 | 0.5468 | 25.1367 |
| 5 | 0.5097 | 0.4336 | 0.8104 | 0.6892 | 3.9893 | 0.1861 | 10.1992 | 7.8893 | 32.5514 | 0.0796 | 0.2950 | 0.5465 | 25.1363 |
| 6 | 0.6143 | 0.5225 | 0.8104 | 0.6892 | 3.9807 | 0.1832 | 10.1922 | 7.8697 | 32.6667 | 0.0796 | 0.2995 | 0.5486 | 25.1388 |
| 7 | 0.7711 | 0.6559 | 0.8104 | 0.6892 | 4.0151 | 0.1942 | 10.2176 | 7.9425 | 32.2297 | 0.0797 | 0.2825 | 0.5402 | 25.1307 |
| 8 | 0.9672 | 0.8226 | 0.8104 | 0.6892 | 4.0728 | 0.2089 | 10.2491 | 8.0355 | 31.6203 | 0.0797 | 0.2586 | 0.5265 | 25.1241 |
| 9 | 1.2547 | 1.0672 | 0.8104 | 0.6892 | 4.2698 | 0.2383 | 10.3086 | 8.2151 | 30.1330 | 0.0795 | 0.1995 | 0.3767 | 25.1208 |
| 10 | 1.6338 | 1.3896 | 0.8104 | 0.6892 | 4.8630 | 0.2224 | 10.3874 | 8.4435 | 26.6176 | 0.0769 | 0.0594 | 0.3394 | 25.1254 |
| 11 | 2.0912 | 1.7787 | 0.8104 | 0.6892 | 5.1081 | 0.1726 | 10.4166 | 8.5226 | 25.8708 | 0.3038 | 0.0300 | 0.4186 | 25.1173 |

**Requirement (a): the corner damping ratio increases monotonically with the
compression click**, strictly, for both axles — the 12 values above are strictly
increasing (front 0.3137 → 2.0912, rear 0.2668 → 1.7787) and the test asserts it
strictly. The rebound ratio only depends on `click_rebound`, so it is constant at
7 (0.8104 / 0.6892).

**Requirement (b): the baseline (click 5 / rebound 7) matches the plan's D7
reference** 0.49 / 0.78 front and 0.43 / 0.68 rear within 10 %: measured
0.5097 / 0.8104 (+4.0 % / +3.9 %) and 0.4336 / 0.6892 (+0.8 % / +1.4 %). D7 quotes
the series-spring ride frequency (4.42 / 3.66 Hz), this tool the exact two-DoF
quarter-car ride mode (4.2830 / 3.5895 Hz); that difference is the documented
reason for the few-percent offset (Ruling 3).

**Finding: the step peak is not monotone in the click.** The first-peak drop first
wanders by under 1.4 % over clicks 0-7 (32.23-32.67 mm, non-monotone: 32.39,
32.45, 32.58, 32.37, 32.57, 32.55, 32.67, 32.23), then falls strictly from click 7
on (32.67 → 25.87 mm, −21 % versus click 0) with the overshoot collapsing from
0.295 to 0.030. The physical reason: the step's first peak is heave/pitch
dominated, and the bump damper acts on the jounce (hop) coordinate, so it barely
touches that first peak until the low-speed slope is stiff enough to lock the hub
towards the body — which also raises the coupled heave mode from 3.99 to 5.11 Hz
(above the plan's 3.4-4.7 Hz band, a reported consequence of the damper, not of a
spring change). The test gates the net effect (peak and overshoot at click 11
versus click 0, and monotonicity from click 7 on) and pins the non-monotone
low-click band, rather than forcing a false monotone claim.

### Baseline modal, ride, and damping-ratio metrics (click 5 / rebound 7)

| Quantity | Value |
|---|---|
| Front / rear ride frequency (exact two-DoF quarter car) | 4.2830 / 3.5895 Hz |
| Heave mode, `\|λ\|/2π` / `Im(λ)/2π` | 3.9893 / 3.9196 Hz, ζ = 0.18614 |
| Roll mode, `\|λ\|/2π` / `Im(λ)/2π` | 10.1992 / 10.0283 Hz, ζ = 0.18226 |
| Pitch mode, `\|λ\|/2π` / `Im(λ)/2π` | 7.8893 / 7.7242 Hz, ζ = 0.20354 |
| Pole identity checked by the test | ζ² + (f_damped/f_natural)² = 1 to 1e-12 |
| Corner damping ratio, front bump / rebound | 0.5097 / 0.8104 |
| Corner damping ratio, rear bump / rebound | 0.4336 / 0.6892 |
| Step peak / time / overshoot / settling / steady | 32.551 mm / 0.0796 s / 0.2950 / 0.5465 s / 25.136 mm |

The Task 6 acceptance gates pin the **undamped** modes (heave 3.83, pitch 7.30,
roll 9.90 Hz); the table above includes the damping matrix, which is why each is a
few percent higher. The pitch mode is still outside the plan's 3.4-4.7 Hz band,
consistent with the ledger's D5 inertia ruling.

### Frequency-response baseline (click 5 / rebound 7, speed 0)

| Channel | Peak magnitude | Unit | Peak frequency (Hz) | Damped modal frequency (Hz) | Peak vs modal |
|---|---|---|---|---|---|
| heave | 1.85842e-05 | m/N | 3.8433 | 3.9893 | −3.7 % |
| roll | 3.12134e-05 | rad/(N·m) | 9.7138 | 10.1992 | −4.8 % |
| pitch | 2.16250e-05 | rad/(N·m) | 7.4522 | 7.8893 | −5.5 % |

Each peak sits below its damped modal frequency, as a displacement (as opposed to
acceleration) transfer must; the test gates the two within 10 % and pins the three
grid peaks.

### ARB override: roll moves, heave and pitch do not

`--study frequency-response`, `arb_wheel_rate_rear` scaled from 0 to 10x the
compiled 21 015.22 N/m:

| ARB rear | roll peak f (Hz) | roll peak magnitude | heave peak f (Hz) | heave peak magnitude | pitch peak f (Hz) |
|---|---|---|---|---|---|
| 0 | 9.7040 | 3.08593e-05 | 3.8433 | 1.85842e-05 | 7.4522 |
| 1x (baseline) | 9.7138 | 3.12134e-05 | 3.8433 | 1.85842e-05 | 7.4522 |
| 3x | 9.7630 | 3.13857e-05 | 3.8433 | 1.85842e-05 | 7.4522 |
| 5x | 9.7925 | 3.12014e-05 | 3.8433 | 1.85842e-05 | 7.4522 |
| 10x | 9.8613 | 3.04683e-05 | 3.8433 | 1.85842e-05 | 7.4522 |

The roll peak frequency is strictly increasing (+1.62 % from 0 to 10x) while the
heave frequency, heave magnitude, and pitch frequency are **bit-identical** to
nine decimal places: the ARB is a pure warpshaft element in this model. The roll
peak *magnitude* is not monotone (3.086, 3.121, 3.139, 3.120, 3.047 e-05), because
a stiffer ARB moves the resonance across a damping-dominated response; the test
gates the frequency monotonicity and the heave/pitch invariance, not the
magnitude.

### Inertance and static-toe overrides

`inertance_front = 3.0 kg` (the Task 6 inertance gate value) adds 3/1.05² = 2.72 kg
to the front jounce diagonal: heave frequency 3.98928 → 3.98328 Hz (−0.151 %),
heave damping ratio 0.18614 → 0.18659 (+0.242 %), pitch frequency −0.121 %, while
the ride frequency and the rear corner damping ratio move by exactly 0. A
`static_toe_front = 0.01 rad` override is accepted and changes **no** reported
metric at the speed-0 static state (zero slip, so no tangential force exists to be
rotated) — both behaviours are gated.

### Four-valve synthetic damper (tmp formulation)

A tmp directory holds a synthetic `damper-table-v1` NPZ with valves `lsc`/`hsc`
(compression) and `lsr`/`hsr` (rebound), built exactly like
`tests/test_damper_table.py` builds its four-valve table, plus two damper XMLs
(`table-front`, `table-rear`, each declaring all four `click_<valve>`
parameters), a copy of the real IEM24 assembly with its damper roles pointed at
the synthetic XMLs and every other reference rewritten to an absolute path, and a
formulation XML over that assembly.

* `valid_override_names` for the synthetic vehicle contains
  `click_lsc`, `click_hsc`, `click_lsr`, `click_hsr` and their `front.`/`rear.`
  forms, and **not** `click_compression` — the names come from the compiled table,
  so the real vehicle rejects `click_lsc` and the synthetic one rejects
  `click_compression`.
* The test applies all four click overrides at once (values land in the compiled
  `DamperSetting`) and runs a 2-setup sweep: soft (all clicks 0) ζ_F = 0.1704,
  ζ_R = 0.1450, peak 37.150 mm; stiff (3/2/3/2) ζ_F = 0.6788, ζ_R = 0.5774, peak
  32.060 mm. The high-band compression valve alone (`click_hsc` 0 → 2) raises the
  front corner damping ratio, so no valve is ignored.

### Cross-validation against the Task 7 step oracle

| Quantity | Tool (pure NumPy, speed 0, no inerter) | Task 7 report (linear 7-coordinate oracle, 15 m/s, 0.5 kg inerter) | Difference |
|---|---|---|---|
| first peak | 32.551 mm | 32.504 mm | **+0.15 %** (gate ±15 %) |
| time to first peak | 0.0796 s | 0.0797 s | −0.13 % |
| overshoot fraction | 0.2950 | 0.2947 | +0.09 % |
| steady state | 25.136 mm | 25.105 mm | +0.12 % |

`tests/test_vertical_setup_sweep.py::StepOracleTests` applies the plan's ±15 %
tolerance to the recorded Task 7 numbers, and then **recomputes** the Task 7
linear oracle from its own helpers (`tests.test_vertical_forward`'s
`closed_form_{mass,stiffness,damping}`, `with_inertance`, `VerticalForwardPlant`,
SciPy `DOP853`) and checks agreement to 1 % on the peak and steady state and 5 % on
the time and overshoot. Tolerances were not weakened.

## Mutation probes (all restored byte-exact; final tool sha256 `fc4409df…22c62`)

The first 14 probes were run against the pre-review tool; M12, M15, M16 and M17
plus M1, M4b, M13 and M14 were re-run against the review-fixed tool and all still
fail their gates.

| # | Mutation | Caught by |
|---|---|---|
| M1 | corner damping ratio drops `/ MR²` | hand-calculation + pinned-values tests (3 failures) |
| M2 | corner damping ratio uses the tangent slope, not the 25 mm/s secant force | hand-calculation + pinned values (4) |
| M3 | roll and pitch mode names swapped | peak-vs-modal + modal band tests (3) |
| M4b | the rear-axle `replace` is skipped, so `arb_wheel_rate_rear` never lands | ARB monotonicity test (2) |
| M5 | step pitch moment sign flipped | Task 7 oracle cross-check (1) |
| M6b | unknown override names accepted silently (both guards removed) | unknown-name test (2) |
| M7 | clicks validated with `allow_fractional=True` | fractional-click test (1) |
| M8 | missing study columns written as `0.0` instead of empty | CSV round-trip test (1) |
| M9 | frequency response reported as a body-acceleration transfer (×ω²) | peak/definition tests (2) |
| M10 | ride frequency uses the series-spring estimate instead of the two-DoF mode | pinned ride frequencies (2) |
| M11 | settling band used as an absolute tolerance | settling-definition test (2) |
| M12 | heave damping ratio offset by +0.1 | baseline modal pin (1) |
| M13 | frequency response read across channels (`H[0,1]`) | peak pins (4) |
| M14 | step response measured at the rear body point | Task 7 oracle + metric tests (3) |
| M15 | `*_damped_frequency_hz` reports `\|λ\|/2π` instead of `Im(λ)/2π` | damped-frequency pin + pole identity (1) |
| M16 | the hashed source list drops `compiler/compile.py` | JSON source-hash test (1) |
| M17 | damping built from the mean of the two branch slopes | baseline modal pin + peak-vs-modal test (2) |

(The first M4/M6 attempts were mis-targeted — they blocked only the front axle /
only the name-listing guard, both of which are behaviour-preserving for the tests —
and are re-run as M4b/M6b.)

## Rulings

Ruling: overrides are applied with `dataclasses.replace` on the compiled vehicle
(a click rebuilds the `DamperSetting`, an ARB/inertance/toe override replaces the
compiled axle scalar) rather than by rewriting an XML in memory/tmp and
recompiling — because a click *indexes* the damper-force table NPZ pinned by
sha256, so a recompile would re-read the same table and add nothing, while
`replace` keeps the table identity, runs 12 setups in 0.25 s, and lets
`DamperSetting.validate` keep the range/integrality check in one place — cost if
wrong: an override that needs an XML-level change (a valve count or a table swap)
is not expressible through this tool and needs `dataclasses.replace` on the table
itself or a recompile path.

Ruling: the accepted override names are exactly the compiled damper valve clicks
plus the six compiled suspension scalars, and the **spring wheel-rate curve is not
overridable** (it is a spline inside the sha256-pinned corner NPZ, not an XML
scalar; the suspension XML exposes only ARB, inertance and static toe per Task 4)
— so the tool executes the plan's required click/ARB behaviours and the plan's
"spring setup" wording is met only for the compiled spring-adjacent scalars — cost
if wrong: a user who asks for a spring-rate sweep must regenerate
`models/data/iem24_murphy_real_corners.npz` with `tools/model_data/generate_suspension_lut.py`
(which requires private suspension source data) or the suspension model must gain a spring scale
parameter in a later task; neither is in Task 10's file list.

Ruling: the per-axle ride frequency is the lower mode of the **exact two-DoF**
body-fixed quarter car built from the tangent spring rate, the tangent tire rate,
and the corner masses, which gives 4.2830 / 3.5895 Hz, while the plan's D7
reference table quotes the series-spring estimate `sqrt(k_w k_t/(k_w+k_t)/m_s)`
(4.42 / 3.66 Hz) that neglects the hub mass — cost if wrong: the tool's ride
frequency is about 3 % below the plan's reference numbers, so a direct comparison
with D7 must carry that allowance (the test uses 10 % on the damping ratio and
pins the tool's own values).

Ruling: the corner damping ratio is the **25 mm/s secant** figure of merit
`(|F(v_ref)|/v_ref)/MR² / (2 m_s ω_ride)` rather than the damper's tangent slope or
the linearized modal damping — because it is the only definition that reproduces
the plan's D7 numbers (0.49 / 0.78 front, 0.43 / 0.68 rear) within a few percent
while a damper table alone cannot give a modal ratio — cost if wrong: it is not
the modal ζ of the heave/pitch/roll modes (those are reported separately from the
eigenvalues); a reader who treats the corner ratio as the body-mode damping
overstates the low-frequency damping, which the click-sweep table shows explicitly
(ζ_corner rises 6.7x while ζ_heave stays 0.17-0.24).

Ruling: the frequency-response observable is **displacement per generalized force**
`H_ii(ω)`, not body acceleration — because the displacement peak is a
property of `(K - ω²M + iωC)⁻¹` with units that need no extra ω² factor and it sits
just below the damped modal frequency, which makes it directly checkable against
the modal metrics — cost if wrong: a reviewer who wants acceleration peaks must
multiply by ω², which shifts the peaks above the modal frequency and changes the
reported peak frequencies; the definition is recorded in every JSON summary.

Ruling: the frequency-response peak is the maximum-magnitude point of the fixed
0.5-30 Hz, 3 001-point grid with no local refinement — cost if wrong: peak
frequencies are quantized to 9.83 mHz, so a study that needs sub-mHz peak
resolution must pass a finer `--frequency-samples` or refine externally.

Ruling: the step-response study uses the **linearized** seven-coordinate subsystem
in closed form (NumPy only, no integrator, no SciPy) and is cross-validated
against the Task 7 linear oracle instead of driving the 38-state nonlinear plant —
because the sweep must stay fast (12 setups in 0.25 s) and the Task 7 report
already measured the nonlinear plant within −1.46 % of its own linear oracle —
cost if wrong: a setup's absolute step peak can differ from a nonlinear plant run
by a couple of percent (the Task 7 nonlinear measurement is 32.03 mm versus the
linear 32.50 mm), so an acceptance decision at the percent level needs a plant run
through the runner.

Ruling: the linearized damping matrix uses each compiled damper's
**compression-branch** slope at zero velocity for both directions, and the report
discloses that this is one of two defensible conventions because the compiled
table is kinked at `v = 0` (click 5: 2126.7 N·s/m compression, 2346.7 rebound) —
the alternative (averaging the two branch slopes, a cycle-average of the kink) was
measured and moves the click-11 step peak and time-to-peak from 25.871 mm /
0.3038 s to **26.434 mm / 0.0767 s**, matching the independent review's 26.44 mm /
0.0767 s; the click-0/5/8/9 numbers stay within ±3 % and peaks move by under 3 %
(click 0 32.389 → 32.206 mm, click 5 32.551 → 32.285, click 8 31.620 → 31.821,
click 9 30.133 → 31.020, click 10 26.618 → 28.696) — cost if wrong: the click-10/11
first peak and time-to-peak are convention-sensitive (a 0.6 mm peak and a 4x
time-to-peak difference at click 11), so a sweep that ends in the very stiff clicks
must be re-read with the branch convention in mind; the JSON's
`damping_branch_convention` field states it for every run.

Ruling: the mode names come from a greedy distinct assignment of the largest
mass-weighted participation over **all** kept eigenvalues, rather than over a
fixed seven. The kept set is one eigenvalue per conjugate pair plus every real
(overdamped) eigenvalue, i.e. **7 + r/2 modes for r real eigenvalues** (r is always
even, because the 14 real eigenvalues split into conjugate pairs and reals): r = 4
gives 9 at the baseline (clicks 0-7), r = 6 gives 10 at click 8, and r = 8 gives 11
at clicks 9-11 — measured from the tool's own eigenvalue spectrum. This was
discovered during implementation: the first version iterated
`range(7)` over an unordered `eig` result and silently dropped the 10.20 Hz roll
mode, reporting roll = 67.34 Hz — cost if wrong: for a setup whose damping moves a
roll-heavy mode between a complex pair and two real eigenvalues, the "roll" label
can follow a different eigenmode, so the named frequency should be read as a guide
and the full eigenvalue list (not exposed by the tool) consulted for a
pathological setup.

Ruling: the ARB gate is the roll **peak frequency** monotonicity plus exact
heave/pitch invariance, not a large roll shift, because the 400 N·s/m tire
vertical damping dominates the damped roll response: the roll peak moves only
1.62 % from ARB 0 to 10x (9.7040 → 9.8613 Hz) while the undamped roll mode moves
9.79 → 10.12 Hz and the rear warp hop 17.18 → 23.97 Hz (Task 6 gates) — cost if
wrong: the ARB probe is a weak sensitivity test, but a wrong ARB sign or a missing
override still fails it (both reverse the monotone roll trend and the mutation
probe M4b confirms the missing-override case).

## Deviations from the plan

1. **`SweepResult` field set is defined here.** The plan fixes only the interface
   and the required metrics; the field names (`front_ride_frequency_hz`,
   `heave_damping_ratio`, `step_settling_time_s`, `roll_peak_magnitude`, …) and the
   CSV/JSON layouts are this task's choice.
2. **`sweep_vertical_setup` writes the CSV/JSON only when paths are given**; the
   CLI passes them. The plan's sentence "It writes a CSV and a JSON summary" is
   satisfied through the CLI and through the optional `output_csv`/`output_json`
   arguments; a pure-computation call writes nothing.
3. **The tool is SciPy-free.** The plan allows SciPy in `tools/`; the closed-form
   state-space step made it unnecessary, so the tool's numbers need NumPy only.
   Only the *test* that recomputes the Task 7 oracle imports SciPy, and it skips
   with a clear message if SciPy is absent.
4. **Extra tests beyond Step 1's three**: the independent hand calculation, the
   plan-D7 comparison, the modal pin, the Task 7 oracle recomputation, the settling
   definition, the ARB/inertance/toe overrides, determinism, the CSV/JSON
   round-trip, the CLI (grid, error paths), and the four-valve synthetic
   formulation. Step 1 names only the click monotonicity, the unknown name, and
   the four-valve case.
5. **The synthetic four-valve test compiles a tmp formulation** (assembly + two
   damper XMLs + NPZ) rather than hand-building a `CompiledDamper`, so the
   override-name derivation is exercised through the real compiler path. The tmp
   files are cleaned up by `TemporaryDirectory`.
6. **Review fix round (independent Task 10 review: approved with fixes).** Seven
   items folded in, all inside the three task files: (i) the kept-set arithmetic in
   Ruling 8 corrected to `7 + r/2` with the measured r per click range; (ii) the
   `|λ|/2π` frequency convention stated in the module docstring, `SweepResult`, the
   JSON `modal_frequency_convention`, and this report, with three new
   `*_damped_frequency_hz` fields reporting `Im(λ)/2π` (existing field values
   unchanged, baseline 3.91956 / 10.02833 / 7.72418 Hz, and the test now checks
   `ζ² + (f_damped/f_natural)² = 1`); (iii) the kinked-damper branch convention
   disclosed in the docstring, the JSON `damping_branch_convention`, and the new
   ruling, with the measured cycle-average sensitivity (26.434 mm / 0.0767 s
   versus 25.871 mm / 0.3038 s at click 11); (iv) `compiler/compile.py` and
   `tires/unitire_transient.py` added to the hashed sources (23 → 25 files, and the
   JSON test now asserts both are present); (v) the unreachable
   `axle_name not in ("front", "rear")` guard in `apply_overrides` **removed** (the
   catalogue check already rejects everything else; every catalogued non-click name
   carries the `_front`/`_rear` suffix, so the guard could not fire) — behaviour
   unchanged, confirmed by the unchanged test results; (vi) the 12-setup runtime
   assertion tightened from 60 s to **5 s** (measured 0.25 s); (vii) the tool title,
   docstring, CLI description and this report no longer advertise a spring-rate
   capability, while the plan's file and function names are unchanged.
7. **No `docs/` file other than this report was added.** The click-sweep CSV/JSON
   used for the numbers above were written to `/tmp`, not into the repository, so
   no evidence directory or provenance file changed.

## Open issues

1. **No spring-rate override.** The plan's task title names springs, but the
   compiled 14DoF model exposes no spring scalar: the wheel-rate curve ships inside
   `models/data/iem24_murphy_real_corners.npz` (sha256 `04bb06c9…f0f0`). A spring
   sweep needs either a regeneration of that NPZ (private suspension source data required) or a new
   `spring_rate_scale_front/rear` parameter in the suspension XML and component —
   a Task 4/5-scope change, deliberately not made here.
2. **The step study is linear.** Absolute first-peak values differ from a
   nonlinear plant run by roughly 1.5 % (Task 7 measurement) and more for a stiff
   setting; the tool never runs the plant, and laps remain the runner's job.
3. **Roll-mode identification can switch eigenmodes** under very stiff damping
   (the greedy assignment always returns three distinct modes, but "roll" can move
   between the two roll-heavy modes of the 9-11 eigenvalue set — `7 + r/2` for r
   real eigenvalues, r = 4…8 over the click sweep). The reported frequency should be
   read with the mode name as a guide.
4. **Peak frequencies are grid-quantized** (9.83 mHz at the default grid) and are
   the grid maximum, not a refined local maximum.
5. **The linearization is at speed 0 by default.** `--speed` is implemented and
   records `speed_m_s`, but only speed 0 is tested: at speed > 0
   `solve_static_equilibrium` carries no longitudinal force (the ledger's Task 6/7
   ruling), so the operating point is not exactly stationary and the damping
   matrix here ignores the speed-dependent (gyroscopic/tangential) terms. A
   speed sweep is not validated.
6. **`static_toe_*` overrides are inert at the tested operating point** (zero slip),
   and `inertance_*` is exercised only through the front axle at 3 kg; a rear-axle
   or large-inertance sweep is untested beyond the compiled-scalar path.
7. **The ARB probe is weak** (1.6 % roll-peak shift over a 10x ARB change) because
   the tire damping dominates; see Ruling 9.
8. **Runtime is measured on this machine only** (0.25 s for 12 setups); no memory
   measurement was taken, and the plan asks for none for this tool.

