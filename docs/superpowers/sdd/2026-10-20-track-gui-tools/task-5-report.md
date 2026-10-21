# Task 5: real-data acceptance and documentation

## Status

The explicit-path validator, opt-in private-fixture tests, source-anchored export mutation tests, developer guide, and desktop manual-check updates are complete. Both actual sources passed the strengthened acceptance run at `c9576a5` plus the source-anchoring test/validator commit `afbf2cc`; production behavior is unchanged, so the acceptance results apply to the final branch working tree at `afbf2cc`. The run followed the core section-reconstruction fix that added strict-loader-aware refinement without weakening loader tolerances. The original FSC finding (section 3, row 154, 0.00204 rad heading/integrated-curvature discrepancy) was fixed in core before this run.

## Validator and commands

`tools/track_gui/validate_track_imports.py` takes explicit `--mis`, `--fsc`, and required `--output` paths. It records source SHA256, selected channels and mode, all settings, lap boundaries, rejected laps, clipping/support and residual summaries, geometry diagnostics, and measured round-trip errors. A requested path that does not exist fails. There is no implicit private-data lookup and no leaked temporary build directory.

Run both fixtures from the repository root:

```bash
PYTHONPATH=src:. MPLCONFIGDIR=/tmp/mplcfg python3 tools/track_gui/validate_track_imports.py \
  --mis /Users/alphabet1671/Desktop/0615CompEndurance.mat \
  --fsc "/Users/alphabet1671/Dev/Simplified-Vehicle-Models/IEM24 Files/FSC-2026-May-Endurance.mat" \
  --output /tmp/track-gui-acceptance
```

Focused tests passed:

```bash
PYTHONPATH=src:. MPLCONFIGDIR=/tmp/mplcfg QT_QPA_PLATFORM=offscreen python3 -m unittest \
  tests.test_track_gui_channels tests.test_track_gui_laps tests.test_track_gui_ops \
  tests.test_track_gui_closed tests.test_track_gui_docs tests.test_track_gui_real_data -q
```

Result: 46 tests, OK. The docs plus real-data modules separately passed 12 tests. Private fixture classes skip with a clear path hint when the private file is absent; they also skip when SciPy or matplotlib is unavailable.

## Review round 1 corrections

- G1: closed save/load now compares node count, distance, north/east, curvature and heading against `from_closed_geometry(...)`. CSV tolerances are 1e-7 m and 1e-9 1/m. Base headings and the writer's final-step seam heading are checked independently; the converter's approximate seam heading is not treated as physical data.
- G2: reversal is checked against an independently computed physical mapping (distance, every XY row, heading offset by π modulo 2π, and negated curvature with seam handling). The saved reversal, double reversal versus the original, and the first open section versus `split_closed` are all checked field by field.
- G3: every saved section is compared with its `split_closed` source. Independent periodic cubic-spline points at the requested cuts and the wrap replace a pure parameter-span assumption. Stored distance must equal the reconstructed arc length, while arc length versus parameter span is measured and bounded separately at max(0.02 m, length/10,000) (0.013 m maximum on MIS, 0.000915 m on FSC).
- G4: double reversal no longer uses a loose XY allowance. Base rows are tight; the terminal heading representation is measured and bounded at 5e-4 rad.
- G5: private tests assert numeric errors. New no-private mutation tests use canonical closed data and prove rejection of +10 m translation, row rotation, +0.1 rad heading offset, +0.5 m shifted cut, wrong reverse curvature sign, and an open start-row shift.
- G6: all temporary output uses a context-managed temporary directory.

## MIS run

Source SHA256: `091e4745a5ff6ccf7d48729154033bb555edd028f0e3e2ed2f3f270e14f6e5b7`. Channels VNorth/VEast in velocity mode; window 550–1050 s; return radius 25 m; minimum travel 150 m; duration 15–120 s; spacing 0.5 m; smoothing 1 m; deviation limit 8 m. Five laps accepted, zero candidate/quality rejections; boundaries 550.00–623.96, 623.96–693.98, 693.98–764.87, 764.87–836.84 and 836.84–907.39 s. 21 of 10,240 aligned samples clipped. Mean length 1021.8715 m, winding error 0.003809 rad, minimum radius 4.767 m, minimum self-distance 14.311 m.

Closed save/load measured 2045 nodes; distance error 4.82e-9 m, XY 5.00e-10 m, heading 5.00e-12 rad, curvature 4.93e-13 1/m. Reverse mapping and saved reverse errors are 0; closed start moved 0 m and the first reversed segment dot product is negative. Double reverse XY error is 0, curvature error 0, base heading error 0, terminal heading error 2.09e-5 rad. Three 340.634 m sections have zero comparison and endpoint errors, zero wrap error, 0.000512 m stored-distance/arc error, and maximum source correction 0.00632 m against the 0.02 m bound. Open save/reverse/double-reverse errors are zero apart from 6.3e-13 m distance in the double reverse.

## FSC run

Source SHA256: `e160110c53e41c8a3419edafc30d84f6bf3bf442016ec7480c11c3d425e6f82b`. Channels Latitude_MTI/Longitude_MTI in GPS mode; window 720.9–1103.6 s; return radius 10 m; minimum travel 150 m; duration 18–35 s; spacing 0.1 m; smoothing 0.5 m; deviation limit 8 m. Fourteen loops accepted, no quality-rejected laps; 133 heading-mismatch candidate attempts rejected. Mean length 288.4339 m, winding error 0.000795 rad, minimum radius 1.369 m, minimum self-distance 0.0032 m (a close-approach metric, not proof of a crossing). 3,438 of 41,384 aligned point samples clipped.

Closed save/load measured 2885 nodes; distance error 5.00e-10 m, XY 5.00e-11 m, heading 4.99e-12 rad, curvature 4.99e-13 1/m. Reverse mapping and saved reverse errors are 0; closed start moved 0 m. Double reverse XY and curvature errors are 0, base heading error 1.78e-15 rad, terminal heading error 4.18e-5 rad. Three sections have zero comparison, endpoint and wrap errors, 0.000568 m stored-distance/arc error, and maximum source correction 0.000846 m against the 0.02 m bound. Open save/reverse/double-reverse errors are zero apart from sub-picometre distance rounding.

## Documentation and limitations

The guide now includes channel units and axes, empty FSC channel names, the invalid FSC GPS interval near 237.25–276.46 s, multi-lap limits, viewer/reversal behavior, exact add/edit/remove/equal-parts cut controls and map-node snapping, the 0.02 m reconstruction bound, the 100,000-node cap, refusal behavior and the explicit validator command. The manual checklist covers signed and unsigned desktop viewing, reverse/save/load, cuts by field and map click, wrap, and safety checks. Existing literal phrases required by `tests.test_track_gui_docs` are preserved.

This work did not run the full repository suite. Figures were generated but not visually inspected because the current image-read capability is unavailable. Native desktop checks remain unperformed.
