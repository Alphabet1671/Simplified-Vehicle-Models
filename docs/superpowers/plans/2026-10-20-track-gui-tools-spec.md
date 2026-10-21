# Track GUI tools specification

## Requirements

Extend the existing PyQt6 track GUI with selectable MoTeC MAT x/north and y/east channels, direct multi-lap import and robust mean-line generation, closed-to-open section export, direction reversal, and an existing-track viewer. The user approved a feature branch, independent review, fixes, local main merge and commits. Do not push or publish.

Channel modes are velocity (integrated), local position in metres, and GPS latitude/longitude in degrees. Respect each struct's Time and Value fields and synchronize channels without extrapolation. Display units and support explicit scale conversion where units are ambiguous. Preserve old single-lap defaults and legacy synthetic fixtures with xtime.Value but no per-channel Time. Do not silently pick empty channels or assume vehicle-frame velocities are north/east.

Multi-lap import must not require a reference track. Detect complete loops by spatial return plus heading and minimum travel/time guards; allow a selected time window and configurable detection limits. A constant lap counter or unreliable lap trigger must not defeat spatial detection. Align laps by translation and cyclic phase while retaining the north/east orientation. Clip large-deviation samples using a robust centre and threshold, then compute the arithmetic mean of retained samples (not merely the median). Reject incomplete, grossly different, stopped or pit laps. Report accepted/rejected laps, sample clipping, boundaries and residuals. Show overlap lines against the result. Limit resources and explain refusal when data cannot support a useful track.

Existing tracks: load repository XML formats. Signed XY tracks show map, start/direction and curvature. Legacy unsigned distance/radius tracks have no recoverable map: show their profile and an explicit unsigned-geometry notice; reversal must still work without inventing a turn direction. Reverse distance order, heading (+pi) and signed curvature (-); closed reversal retains the start position. Double reversal restores geometry modulo heading's 2pi ambiguity.

Closed sectioning: editable cut distances, N equal parts, and map clicks snapped to nodes. Mark cuts on both plots. Export separate numbered open XML/CSV pairs with validated geometry, retaining the measured line within a documented small tolerance. Include start/finish wrap behavior. Do not weaken src loader checks. Guard all targets before batch writes; refusals must not leave a partial section set. Existing models and generated files retain overwrite protections.

## Real-file acceptance

Use /Users/alphabet1671/Desktop/0615CompEndurance.mat (MIS June 2024) and /Users/alphabet1671/Dev/Simplified-Vehicle-Models/IEM24 Files/FSC-2026-May-Endurance.mat. Do not commit either MAT file. Record SHA256, chosen channels, mode, window, detection settings, accepted laps and result diagnostics. Assert meaningful geometry and load/save/reverse/split round trips on both. Compare overlaps visually with generated diagnostic figures, without claiming manual desktop testing.

MIS has VNorth/VEast in m/s; lap is constant and laptrig is unreliable. FSC has Latitude_MTI/Longitude_MTI in degrees, with invalid GPS around 237–276 s; other position channels are often empty. FSC velY_MTI is approximately north and velX_MTI approximately east, unlike MIS. Initial exploration suggests MIS loops near 1 km and FSC loops near 293 m; these are sanity observations, not hard-coded circuit assumptions.

## Constraints

Keep Qt imports in qt/, app entrypoints and Qt tests. Do not modify src/, tools/sim_gui/, tools/anim/, or pyproject.toml. Preserve canonical model files. Keep new top-level tool basenames unique. Documentation uses ASD-STE100 style. Store curated plans/reports under docs/superpowers/; scratch may use ignored .superpowers/. Use unittest with PYTHONPATH=src:. and offscreen Qt. Full regression command is PYTHONPATH=src python3 -m unittest discover -s tests -q. Compare named failures with the existing main baseline; no 14dof-simple test runner. The GUI smoke hook is TRACK_GUI_QUIT_AFTER_MS.
