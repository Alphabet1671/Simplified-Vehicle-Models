# Tube diagnostic v2 — prospective protocol (NON-BINDING)

Status: **pre-registered before any fourth refinement level (g) is run.** Written before 2026-10-16T02:34:16Z (UTC; see the freeze record). The exact freeze time and the SHA-256 of this file are recorded in `tube_v2_protocol.freeze.txt` (written immediately after this file; see "Freeze record"). Anything not stated here is not part of the protocol, and later edits invalidate the freeze.

## 0. Standing and scope

* **Diagnostic only.** Nothing here is an acceptance gate. It must not set, raise, or lower `usable`, `refinement_validated`, `between_step_validation_complete`, or any other field of any `summary.json`. It reads those files and writes only its own new file `tube_v2_certificate.json` next to the finest level. It never imports or modifies `tools/check_iem24_forward_tube.py` (v1) or `acceptance_gate`.
* **Reason it exists.** v1 caps the observed ratio `r = d_mf/d_cm` at 0.9. Where `r >= 1` (true for several components in run 6, e.g. yaw_rate 1.05, steering 1.14, w_fr 1.09), its tube `d_mf*(1+r/(1-r))` is capped to `10*d_mf`, which is *not* an error bound, because the data show no convergence there. v2 therefore does not assume geometric convergence.
* **User-selected alternative metric**, used only because the event diagnosis could not resolve the v1 failure. No claim of acceptance is made without a reviewed standard.
* Frozen sources: `tools/check_iem24_forward_tube.py` sha256 `57b92964f3cc13057a230f7c72e866e2df6a7742d6c4931e27aaec511ee200ab`; `tools/run_iem24_forward_driver.py` sha256 `6520a4f224f2dce9f0ad371155ab87d5954ae8aa084c27758ffc1021e402a46f`; `planar_dense_certificate.py` sha256 `0b9c7f5309f367c1674f776d176d109bff20cc2c264496bf858682a604a7f5bc`; `dense_domain.py` sha256 `5a671c710fad8ea85df1aabdb4cd8061b70a30b94e5072b95781cd879ba71d75` (checked at freeze time). The v2 files are `tools/check_iem24_forward_tube_v2.py` and `tests/test_forward_tube_v2.py` (new; their hashes are added to the freeze record when written).

## 1. Levels and the held-out fourth level

Per track (original, canonical), levels ordered by decreasing `maximum_step` and `rtol`:

| Level | Role | Settings |
|---|---|---|
| c (coarse) | tube input | the existing level of lowest resolution in the v1 triple |
| m (medium) | tube input | existing, `rtol=5e-6`, `maximum_step=0.01` |
| f (fine) | tube input; the run being qualified | existing, `rtol=2.5e-6`, `maximum_step=0.005` |
| g (held-out) | **prospective test only** | **new:** `rtol=1.25e-6`, `maximum_step=0.0025`, same warm start (the same cold endpoint), planner, track, speed, pedal time, BDF, spacing, as f |

The c, m, f paths are exactly those named in the existing `tube_certificate.json` of that track (`levels`). Level g is generated only after this protocol is frozen. Its settings above are fixed now and may not change after seeing any g data. If g fails to complete (e.g. the 300 s wall cap), the outcome is **"g not completed"**, recorded as such; settings are not loosened and the criteria are not re-tuned. Level g is never used to compute T (§3).

All levels must have `lap_completed` true, a `dense_polynomials.npz`, identical `source_hashes` (the whole dict), identical `track`, `planner`, `spacing`, `maximum_speed`, `lateral_acceleration`, `pedal_travel_time`, `method`, and strictly decreasing `maximum_step` and `rtol` (same as v1). Any mismatch, missing field, or non-finite value raises an error and **no certificate is written** (fail closed).

## 2. Difference measure

For levels a, b the componentwise difference is `d_ab[k] = max_t |a_k(t) - b_k(t)|` over the shared time span `[0, min(T_a, T_b)]`, evaluated on the dense polynomials (v1's `evaluate`, degree <= 5 in the scaled variable `s = (t - t_end)/step`) at 9 equispaced points per interval of the finer level plus the end time (v1's `SAMPLES_PER_INTERVAL = 9`).

* This is a **sampled sup** and is labeled `"sampled_sup"` in the output. It is a lower bound on the true sup of the piecewise-polynomial difference.
* An exact piecewise-polynomial sup over the union of knots is *optional* (the difference polynomial on each sub-interval has degree <= 5, so its extrema are roots of a degree-4 derivative). If implemented, it is reported as a separate field `exact_sup_union_knots`, never replaces the sampled value, and is not used in any pass criterion unless it is finite and computed for every component and every interval. Sampled sup is the registered quantity.
* Reported per component and per pair (c,m), (m,f), (c,f), and (f,g) when g exists: `sup`, `rms` (root-mean-square over the sample points, unweighted by interval length), `p99` (99th percentile of `|diff|` over the same samples), and `argmax_time_s`.
* Time is not a compared state; the state components are the 13 plant states in the order `s, n, heading, v_x, v_y, yaw_rate, w_fl, w_fr, w_rl, w_rr, soc, steering, pedal`.

## 3. Convergence-agnostic tube T (three levels c, m, f only)

```
T[k] = 10 * max(d_cm[k], d_mf[k])      componentwise, no ratio, no cap, no extrapolation
```

* **T is a numerical diagnostic scale, NOT a true ODE error bound.** No assumption of geometric or any other convergence is made. It is reported as `tube_T_diagnostic_scale`; the words "error bound" are never used for it.
* If `d_cm[k] == d_mf[k] == 0` for a component, `T[k] = 0` and the component is flagged `degenerate_zero`.
* Any non-finite `d` or `T` -> fail closed (no certificate).

## 4. Two separate tests (always reported separately and labeled)

**4a. Numerical domain-margin test (not a convergence test).** Recompute `certify_planar_dense_domain(vehicle, plan, bounds_widened, True)` on the fine run's dense bounds widened by `2T` (`lo - 2T`, `hi + 2T`, per component); also report the unwidened call. Passes iff both return `certified: true` with finite margins. This says only that the plant's domain inequalities still hold on the Bernstein enclosure after inflating by `2T`. It says nothing about whether the runs converged. Output field: `domain_margin_test_widened_2T`.

**4b. Empirical convergence test (separate, never merged into 4a).** Per component, with `d_cm`, `d_mf` (and `d_fg` if g exists):
* `shrink_cm_mf`: `d_mf < d_cm` strictly.
* `shrink_mf_fg`: `d_fg < d_mf` strictly (only with g).
* `observed_ratio_mf_over_cm` and `observed_ratio_fg_over_mf` are reported raw, **uncapped**.
* Prospective containment (g only): `d_fg[k] <= T[k]` for all k. T is computed from c, m, f alone, so this is a held-out prediction.

The headline verdict reports 4a and 4b as independent booleans; there is no combined "passed" field. A top-level field `acceptance_claim` is always the string `"none; diagnostic only; requires reviewed standard"`.

## 5. Fixed event windows (registered before g)

Purpose: separate non-smooth switching regions from smooth regions, to report where the sup differences live.

* Event times are computed from the **coarse level c only** (never from any difference between levels, and never from m, f, or g), as the crossing times, using linear interpolation between the 9-per-interval sample points of c's dense polynomial, of:
  * pedal component crossing `0` (the throttle/brake switch in `realized_command`, `max(p,0)` vs `max(-p,0)`; added in revision 2),
  * pedal component crossing `+1` or `-1` (the point where `realized_command` clips),
  * any wheel-speed component (`w_fl, w_fr, w_rl, w_rr`) crossing `0`,
  * steering component crossing `+/- vehicle.steering_maximum_angle`.
  A sample with `|x - threshold| <= 1e-9` counts as a crossing.
* Each event time `t_e` defines a window `[t_e - 3*h_c, t_e + 3*h_c]` with `h_c` = coarse `maximum_step`. Overlapping windows are merged. Windows are saved in the certificate (`event_windows_s`) with the event list and the rule above. Windows are determined once, before comparison, and are never adjusted after seeing any difference.
* **Unmodeled non-smoothness (declared, revision 2).** The driver's curvature preview (`plan.sample(distance + speed*0.15)`, piecewise-linear in the 0.5 m plan table) and the plan's feedforward-acceleration kinks have knot/seam crossings that are **not** windowed. Windowing every 0.5 m knot with ±3*h_c would cover essentially the whole lap above ~8 m/s, and no principled non-arbitrary threshold has been fixed. They are therefore deliberately left as *unexplained out-of-window* sources. Consequence, stated before g: the out-of-window strict-shrink test in 4b **may fail because of them, and a failure is reported as such, not explained away or re-windowed.**
* **Fail closed.** If, for a pair, there are zero out-of-window samples, the out-of-window statistics and shrink test are reported as `null` / not established, never as pass.
* If no events are found, `event_windows_s` is empty, all samples are "out-of-window", and the output records `"events_found": 0`. This does not prove smoothness.
* Per component, report `d_*` separately for **in-window** and **out-of-window** samples (sup, RMS, p99, argmax) for each pair.
* Out-of-window convergence diagnostic: strict shrink `d_mf^out < d_cm^out` (and `d_fg^out < d_mf^out` with g). In-window values are reported but are not required to shrink.
* The domain-margin test (4a) uses the global `T` (full sup, not window-restricted), so in-window differences, at 10x their size, are included in the safety test.
* The event-rule definition is part of the freeze; the windows themselves are not a fit to any run-6 finding. If the rule is believed wrong, it is replaced by a **v3 protocol** and the v2 results stay on record.

## 6. Anticipated outcomes (stated before g, to prevent post-hoc reading)

* Observed v1 run-6 ratios `>= 1` (yaw_rate, steering, w_fr, ...) are **not** evidence of convergence; v2 reports them raw.
* Result categories for the fourth run, each reported as is: (i) g not completed; (ii) all of 4b hold; (iii) out-of-window shrinks but some in-window component does not; (iv) out-of-window does not shrink; (v) `d_fg > T` for some component. Any of (iii)-(v) leaves the tube diagnostic unestablished. None changes `usable`.
* No threshold (the factor 10, the 2T widening, the 3*h_c half-width, the 9-sample density, the strictness of shrink) may be changed after g exists. A different choice requires a new, separately-versioned protocol.

## 7. Reproduce

```bash
# after levels c, m, f (and optionally g) exist
PYTHONPATH=src:. python3 tools/check_iem24_forward_tube_v2.py <c_attempt> <m_attempt> <f_attempt> [<g_attempt>]
# writes <f_attempt>/tube_v2_certificate.json; exit 0 only if the file was written; the exit code does
# NOT encode pass/fail of 4a or 4b. Tests:
PYTHONPATH=src:. python3 -m unittest tests.test_forward_tube_v2 -q
```

Level g, per track (identical to f except the two numbers and a new output root; use that track's f `--warm-start` source and `--track`):

```bash
PYTHONPATH=src:. python3 tools/run_iem24_forward_driver.py --warm-start <same cold attempt as f> \
  --rtol 1.25e-6 --maximum-step 0.0025 --planner live --maximum-speed 60 --pedal-travel-time 0.175 \
  --method BDF --spacing 0.5 --duration 300 --timeout-seconds 300 --track <track xml> --output-root <new root>
```

## 8. Freeze record and revision history

`tube_v2_protocol.freeze.txt` (same directory) carries this file's SHA-256 and UTC timestamp, taken after saving and before any g run. No git commit is made. The freeze file is append-only.

* Revision 1 (SUPERSEDED before any g run): omitted the pedal-zero event. Its SHA-256 `cf45fb1b6bfcfe997a10bdedce48e777b0847965c3ab6d8d33a0d7317c8ee4ef` (frozen 2026-10-16T02:34:16Z) stays in the freeze file, marked SUPERSEDED.
* Revision 2 (current): adds the pedal-zero crossing to the event list, declares preview-knot / feedforward-kink non-smoothness as unmodeled (§5), and adds the zero-out-of-window fail-closed rule. No other threshold changed. Windows come from the coarse level only; no g data was used.
