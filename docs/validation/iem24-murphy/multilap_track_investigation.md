# MIS 2024 lateral-acceleration double-peak investigation

> **Tools layout migration note.** `tools/` was reorganised into category subpackages (`data`, `tracks`, `model_data`, `run`, `gates`, `reports`, `diagnostics`). Commands and paths in this document use the current locations; See [docs/dev/tools-layout.md](../../dev/tools-layout.md) for the old-to-new mapping. Recorded hashes, attempt directories and frozen artifacts are unchanged.

*Historical (see the 2026-10-15 status update at the end, which supersedes the "not a replacement track" and "experimental" wording below):* this was a **diagnostic study**, not an MIS standardized report, a new point-mass solve, or a new 7DoF solve. The [candidate geometry](multilap_track_candidate.xml) is a representative *driven line*, since adopted as `models/tracks/2024_michigan_endurance_multilap_closed.xml`. The existing `models/tracks/2024_michigan_endurance_closed.xml` also comes from one integrated driven lap—not from a surveyed centerline.

## Observed mechanism at the clearest same-sign doublet

On the present track, signed curvature has two positive peaks at about **252.0 and 268.5 m** (0.058 and 0.059 m⁻¹), separated by a dip near 260 m (0.042 m⁻¹, about 27% lower). The point-mass solver's speed/ay responds almost directly to these local curvature maxima; the independently integrated 7DoF driver follows the same path and inherits their shape. On a common 1 m grid, the original path curvature multiplied by the *measured median speed squared* (not a solver run) has two lateral-acceleration maxima, **15.68 and 15.28 m/s²** at 252 and 268 m. The old point-mass ay is 10.64 and 10.67 m/s², separated by a dip to 7.94; the plotted live-limit 7DoF ay is 9.44 and 10.65, separated by a dip to 8.06. Its two maxima survive qualitatively even when velocity and tires differ. In contrast, the measured median ay runs about 12.44 at 252, 13.31 at 260 and 11.55 at 268 m (raw interpolated values). The ten individually smoothed measured laps show one broad/shifted maximum rather than a consistent 252/268 m doublet; measured v·yaw confirms broadly similar turn evolution.

The cause of *this ay doublet* is therefore primarily **the curvature definition of the current single-lap driven path**, not two independent vehicles independently choosing the same transient turn input. This does not imply the entire curve or every corner is wrong: real chicanes with yaw reversals exist, and some ay peaks match one-to-one. The point-mass model is quasi-static, so it cannot independently generate a steering transient.

Do not confuse this with the distinct **ax braking pulses**: measured driving brakes once through some chicanes whereas both models brake to successive local speed limits. Equal 1 Hz filtering does not remove those modeled ax pulses; the speed drop over 190–260 m is roughly 47% greater in point mass (specific kinetic-energy drop 113.5 vs 77.4 m²/s²). Some measured laps have secondary ax troughs, so the median is not universal. The plot applies a 1 Hz zero-phase filter to measured acceleration but displays unfiltered model acceleration, making model sharpness look larger; future overlays should filter both consistently for *visual* comparison without changing source data. Measured ay also has an approximately −0.7 m/s² offset against v·yaw, plausibly IMU bias/banking.

## Why the existing path can introduce artificial doublets

`tools/tracks/build_closed_track.py` integrates `VNorth/VEast` over fixed 628.61–698.94 s, closes the trajectory by linear displacement correction, twice smooths XY with a 2 m sigma Gaussian, resamples and finite-differences twice for curvature. The actual closure-detected flying lap is 629.44–699.43 s. The legacy window has a **3.45 m closure gap and ~0.46 rad endpoint tangent mismatch**, which are absorbed into a tight-corner seam around 1005 m. The measured velocity data's independent fixes are about 5–6 Hz despite its 100 Hz timestamp array; 2 m smoothing applied twice means effective ≈2.83 m sigma, about a navigation fix's travel. Small second-derivative ripples are plausible. Piecewise-linear curvature interpolation then feeds the speed planner; two local |κ| peaks produce two local speed limits. This hypothesis does not label every same-sign pair as noise. In particular the seam and rapid alternating turn sequences require separate checks against gyro, individual ay and cross-lap position.

The distance-only nearest-node projection uses 0.5 m nodes. Ten flying laps have no backward projected steps; the typical nearest-path residual is about 1 m and the fitted translation is ~(+9.97,+3.43) m. Misprojection by sub-metre distances cannot create a 16 m-separated doublet. The measured folded median can hide differing apex timing, which is why individual laps were checked. The measured driven paths have p95 original-line residuals roughly 1.6–3.1 m, so a surveyed physical centerline cannot be inferred from these traces.

## Experimental ten-lap candidate

`tools/tracks/build_iem24_multilap_track.py` independently translates each of the ten flying laps, projects its samples to the existing distance phase, converts to signed lateral offsets, periodically interpolates/smooths at 3 m, takes the median of consistent laps, offsets the original XY, then smooths XY at 3 m and resamples as an independent closed path. All ten flying laps pass an explicit robust RMS-consistency rule (0.39–1.15 m from their median). The p95 across-lap line spread is **1.16 m**, and the maximum median offset from the original is **1.39 m**. The [diagnostics](multilap_track_candidate_diagnostics.json) record translations and selection. The [candidate XML](multilap_track_candidate.xml) and [CSV](multilap_track_candidate.csv) are separate and load with the existing profile loader; the original track is untouched.

| Geometry | Original one-lap path | Experimental ten-lap median |
|---|---:|---:|
| Closed length | 1016.70 m | 1009.20 m |
| Minimum radius | 6.18 m | 5.76 m |
| Winding | −6.285 rad | −6.286 rad |
| Curvature-derived ay near 252–268 m, using measured median speed | Double: peaks 15.68, 15.28 m/s² | Single broad peak about 14.77 m/s² near 260 m |

**Important limitation:** A 7.5056 m (0.74%) shrinkage is substantial: direct same-distance lap-time or exact peak-position comparison is invalid without alignment. XY smoothing inherently shortens curved paths and affects minimum radius, so this result supports the *local line-shape hypothesis*, not the candidate as the circuit's ground truth (it was later adopted only as a canonical *driven line*; see the status update). It also uses the old path to define progress; position INS drift and shared measured laps make fitting-and-testing on the same ten laps optimistic. A held-out lap comparison, independent navigation/survey measurement, varied smoothing and a length-preserving periodic fit remain needed to claim surveyed fidelity; they were not done before adoption. Winding near 2π checks only topology, not geometric fidelity.

No model was rerun on the candidate: the 15.68/15.28 → 14.77 result is a **kinematic geometry counterfactual**, not a validated new point-mass or 7DoF prediction. Controlled reruns should take the candidate via an explicit track argument into *separate* output folders and report model settings, candidate provenance, lap lengths and separately filtered/unfiltered traces. *Superseded:* at the time of this study the forward-driver runner hard-coded the existing track and its default was not changed; the plan's later tasks add an explicit track argument. No changes were made to the established three-way figures or report.

## Status update 2026-10-15: canonical adoption of the candidate geometry

The candidate line is now available as a **separate canonical model**, `models/tracks/2024_michigan_endurance_multilap_closed.xml`, with its CSV `models/tracks/2024_michigan_endurance_multilap_closed.csv`. The CSV is a byte copy of `multilap_track_candidate.csv`. The legacy `2024_michigan_endurance_closed.{xml,csv}` is unchanged (1016.70064959 m) and stays available for the existing reports. Adoption only provides the file; **no solver default was changed and no model was rerun**, so the limitations above (7.5 m shorter, driven line not surveyed centerline, same-lap fitting optimism, no held-out check) still apply. Runs on the two tracks must be compared by physical position, not equal distance.

`load_track` returns a signed closed profile: 2034 nodes (last repeats first), length **1009.19501535 m**, closure gap 0 m, net turn −6.285968 rad (heading change −6.2778 rad, so a 0.0054 rad end-tangent mismatch at the seam), minimum radius 5.7648 m.

| File | SHA-256 |
|---|---|
| `models/tracks/2024_michigan_endurance_multilap_closed.csv` (= `multilap_track_candidate.csv`) | `2db5a981f12d0c055afa3b167a9e6efa07cd09b1f6e7b2c51afea44f00e73d77` |
| `docs/validation/iem24-murphy/multilap_track_candidate_diagnostics.json` | `c75baa6ae92ea192c4b1522097d4cdd6e5701d33f81455e987f60283651e67a4` |
| `models/tracks/2024_michigan_endurance_closed.csv` (legacy, unchanged) | `ff8dcdc715eaff07eeb1d7a9e0a0739072480150577622ec8854000a0a44dbd4` |
| `models/tracks/2024_michigan_endurance_closed.xml` (legacy, unchanged) | `2880418d148b763e1360d25eb4b7a8a4144bb16e6e56017facf7f36bddc47180` |

Source telemetry `0615CompEndurance.mat` sha256 `091e4745a5ff6ccf7d48729154033bb555edd028f0e3e2ed2f3f270e14f6e5b7`. Tests: `PYTHONPATH=src:. python3 -m unittest tests.test_iem24_multilap_track -q`.

Provenance of the build: `PYTHONPATH=src:. python3 tools/tracks/build_iem24_multilap_track.py --output docs/validation/iem24-murphy/multilap_track_candidate`, from inputs `docs/validation/iem24-murphy/measured_stint.csv` (sha256 `795d37a904302f712bab1fa1809dbd838c4cb9c0db3e60c9215561487d6c3d2b`) and the legacy track XML above. Re-running it to a scratch directory on 2026-10-15 reproduced the canonical CSV byte for byte. The canonical track winds counterclockwise viewed from above (net left turn, negative signed heading change).
