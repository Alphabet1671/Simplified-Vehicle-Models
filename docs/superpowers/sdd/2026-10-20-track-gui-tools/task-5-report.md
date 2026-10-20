# Task 5: real-data acceptance and documentation

## Status

The explicit-path validator, opt-in private-fixture tests, developer guide, and desktop manual-check updates are in place. The actual MIS and FSC fixtures passed after a core section-reconstruction fix added strict-loader-aware refinement without weakening loader tolerances. Both paths include closed save/load, closed reverse save/load, double reverse, three strict open sections, per-section measured positional correction, and open-section save/reverse/load. The original FSC finding (section 3, row 154, 0.00204 rad heading/integrated-curvature discrepancy) was fixed in the core implementation before the final run.

## Validator and commands

`tools/track_gui/validate_track_imports.py` takes explicit `--mis`, `--fsc`, and required `--output` paths. It records source SHA256, selected channels and mode, all selected settings, detected lap boundaries, rejected laps, clipping/support and residual summaries, geometry diagnostics, and round-trip results. It writes overlap PNG figures. Missing requested paths fail; there is no implicit private-data lookup. Generated figures and source MAT files were not visually inspected or modified. No desktop/manual checks are claimed.

Run both fixtures from repository root:

```bash
PYTHONPATH=src:. MPLCONFIGDIR=/tmp/mplcfg python3 tools/track_gui/validate_track_imports.py \
  --mis /Users/alphabet1671/Desktop/0615CompEndurance.mat \
  --fsc "/Users/alphabet1671/Dev/Simplified-Vehicle-Models/IEM24 Files/FSC-2026-May-Endurance.mat" \
  --output /tmp/track-gui-acceptance
```

Focused tests passed: `PYTHONPATH=src:. MPLCONFIGDIR=/tmp/mplcfg QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_track_gui_channels tests.test_track_gui_laps tests.test_track_gui_ops tests.test_track_gui_closed tests.test_track_gui_docs tests.test_track_gui_real_data -q` — 38 tests, OK. The separate docs+real-data command also passed (6 tests). Private fixture test classes skip with a clear path hint when their configured/default file is absent in CI, and skip when SciPy or matplotlib is unavailable.

## MIS run

Source SHA256: `091e4745a5ff6ccf7d48729154033bb555edd028f0e3e2ed2f3f270e14f6e5b7`. Channels VNorth/VEast in velocity mode; window 550–1050 s; return radius 25 m; minimum travel 150 m; duration 15–120 s; spacing 0.5 m; smoothing 1 m; deviation limit 8 m. Five laps accepted, zero candidate/quality rejections, boundaries 550.00–623.96, 623.96–693.98, 693.98–764.87, 764.87–836.84 and 836.84–907.39 s. 21 of 10,240 aligned samples clipped. The JSON records full per-lap details, support and residual distribution. Closed save/load, reverse save/load, start retention/curvature sign, double-reverse XY, and three strict saved open sections passed. Curated JSON and figure are in this directory.

## FSC run

Source SHA256: `e160110c53e41c8a3419edafc30d84f6bf3bf442016ec7480c11c3d425e6f82b`. Channels Latitude_MTI/Longitude_MTI in GPS mode; window 720.9–1103.6 s; return radius 10 m; minimum travel 150 m; duration 18–35 s; spacing 0.1 m; smoothing 0.5 m; deviation limit 8 m. Fourteen loops accepted, no quality-rejected laps; 133 boundary/detection attempts rejected for heading mismatch. Final mean length 288.4339 m, winding error 0.000795 rad, minimum radius 1.369 m, minimum self-distance 0.0032 m (a close-approach metric, not proof of a crossing). The run clipped 3,438 of 41,384 aligned point samples. Closed and open round trips and three sections passed strict loader validation. Section corrections are in acceptance.json and each is bounded by the recorded 0.02 m source-position limit. The first run exposed section 3 row 154 heading/integrated-curvature error of 0.00204 rad; the implementation was refined to satisfy strict loader validation, and the complete command was rerun successfully without changing loader tolerance.

## Documentation and limitations

The developer guide describes channel units and axes, known empty FSC channel names, multi-lap limits, viewer/reversal/cuts and the explicit validator. The manual checklist covers desktop map, unsigned geometry notice, reverse/save/load, cuts, wrap, and safety checks. Existing literal phrases required by `tests.test_track_gui_docs` are preserved. This work did not run the full repository suite. Figures are generated but not visually inspected because the current image-read capability is unavailable. Desktop checks remain unperformed.
