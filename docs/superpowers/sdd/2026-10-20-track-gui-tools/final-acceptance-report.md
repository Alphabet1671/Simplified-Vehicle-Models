# Final review and acceptance — track GUI tools

This report records the independent whole-branch review, its one blocking finding and fix, the per-task approvals, the exact regression comparison, and the limits of the evidence. The feature branch was merged into local `main` as commit `eca4300`; branch head was `b0a98a4` from base `f634d21`. Nothing was pushed.

## Result

The feature branch `feature/track-gui-tools` implements the five requested capabilities:

1. Selectable MAT x/north and y/east channels in velocity, local-position metre, and GPS modes, with units, scales, explicit unknown-unit opt-in, clocks synchronized over their union, and no extrapolation.
2. Reference-free multi-lap detection, translation and cyclic-phase alignment, robust deviation clipping, and the arithmetic mean of retained samples, with overlap diagnostics.
3. Closed-to-open sectioning with editable cut distances, N equal parts, map clicks snapped to nodes, markers on both plots, and transactional numbered XML/CSV exports.
4. Reversal of signed open tracks, closed tracks, and legacy unsigned radius profiles. Closed reversal keeps the start position; double reversal restores the geometry modulo the heading turn.
5. A viewer tab for existing open, closed, and unsigned tracks, with map, curvature, reversal, cuts, and guarded save.

The source loader tolerances are unchanged. Protected paths under `src/`, `tools/sim_gui/`, `tools/anim/`, `pyproject.toml`, and `models/` are unchanged from `f634d21`.

## Independent reviews

Every task received an independent specification and quality review. The final whole-branch review covered `f634d21..afbf2cc` (21 commits) and found one blocking defect: when `VNorth`/`VEast` are absent, both channel selectors could default to the same channel, and a same-channel velocity build could save a degenerate closed track that the loader accepted.

Fix `b0a98a4` rejects identical north/east channels as the first statement of `mat_channels.load_coordinates`, before mode/time validation, file access, or SciPy, for velocity, position and GPS modes. The GUI refuses the same selection before starting a worker. A SciPy-free test covers all three modes with a nonexistent path, and a Qt test covers the absent-default GUI case. A scoped re-review verified the ordering with a targeted `_metadata` probe and approved the commit (specification PASS, quality APPROVED, 7 files, +34/−3). No other behavior changed.

All remaining findings are Minor and deferred with reasons:

- Cross-filesystem export stages in the system temporary directory; a destination on another filesystem fails safely without a partial set. Removable-drive export is not supported and is now documented.
- A closed `TrackData` built outside the library is not required to repeat its seam row; all library producers do. This is a latent public-API invariant only.
- The 12,000-node single-lap build cap and the 100,000-node per-section cap are documented.
- Full multi-lap diagnostics are stored in the XML comment and GUI report; this is truthful but verbose.
- Finished discovery threads accumulate; a finished close pointer is not cleared. No stale save can be enabled.
- The viewer receives signed and unsigned fixtures in memory in the Qt tests; successful GUI `load_path` of each stored subtype remains a follow-up, as recorded in the Task 4 report.
- One source file uses compressed formatting. Readability only.
- The Task 5 report calls `afbf2cc` the final working tree; the guard `b0a98a4` came after it, and the actual acceptance tests ran again successfully because the acceptance channel pairs are distinct.

## Actual fixture acceptance

The private MAT files remain untracked. The validator ran with explicit paths on both files and regenerated `acceptance.json` and the two overlap PNGs.

| Fixture | SHA256 | Channels | Accepted laps | Averaged length | Clipped aligned samples | Maximum section correction |
|---|---|---|---:|---:|---:|---:|
| MIS June 2024, `0615CompEndurance.mat` | `091e4745a5ff6ccf7d48729154033bb555edd028f0e3e2ed2f3f270e14f6e5b7` | VNorth/VEast, velocity, 550–1050 s | 5 | 1021.8715 m | 21 / 10,240 | 0.00632 m |
| FSC May 2026, `FSC-2026-May-Endurance.mat` | `e160110c53e41c8a3419edafc30d84f6bf3bf442016ec7480c11c3d425e6f82b` | Latitude_MTI/Longitude_MTI, GPS, 720.9–1103.6 s | 14 | 288.4339 m | 3,438 / 41,384 | 0.000846 m |

Both runs pass source-anchored closed save/load, reversal, double reversal, three-section reconstruction with independent periodic-spline cut endpoints, and open-section save/reverse/load. Measured closed export errors are below 5e-9 m and 1e-12 1/m. Reconstructed section arc length differs from the requested cut span by at most 0.0128 m on MIS and 0.000915 m on FSC, bounded explicitly. FSC accepts no quality-rejected complete lap; 133 detector attempts fail the heading-consistency test and are reported separately from rejected laps.

## Tests

Baseline was the unchanged `main` at `f634d21`: **1,817 tests, 20 failures, 9 errors, 2 skipped**, with 29 normalized failure/error entries. The merged `main` at `eca4300` ran **1,872 tests, 20 failures, 9 errors, 2 skipped**; the normalized comparison gives exactly the same 29 entries, zero added and zero removed, so the feature adds 55 tests and no new failure. The merged checkout has the ignored pinned trace CSVs, so the seven environment-only errors seen in the feature worktree do not occur here.

- Final `main` track-GUI family: **115 tests, OK** in 6.431 s.
- Before the guard, final feature branch: **157 focused tests, OK** (113 track-GUI including private fixtures and mutations, plus 44 track/layout tests).
- After the guard: **64 tests, OK** in the channel/docs/Qt scoped run and **68 tests, OK** in the core/actual-fixture scoped run.
- Actual MIS and FSC acceptance: **2 tests, OK** after the guard.
- Offscreen startup smoke: exit 0.

## Limits

No manual desktop session was performed. The generated overlap figures were checked for existence, format and dimensions but not visually inspected, because the current model cannot accept images. FSC's 0.0032 m close-approach metric is not proof of a crossing; it is a reason for a human to inspect `fsc-overlap.png`. Windows and Linux are not tested.

Start the GUI with `PYTHONPATH=src:. python3 tools/track_gui/app.py`. The user guide is `docs/dev/track-gui.md` and the manual checks are `docs/dev/gui-manual-checks.md`.
