# FSC signed-edge source admission — 2026-10-25

**Decision: BLOCKED.** The current worktree contains no admissible authoritative FSC track source: no FSC XML/CSV pair providing signed XY geometry together with left and right edges/widths and closure semantics. No FSC preflight, forward run, or FSC animation was generated. Do not infer or reconstruct a physical FSC track from OCP outputs.

## Search scope and result

The inventory was performed from the worktree root on `feature/yaw-aware-virtual-driver`. It covered tracked and ignored worktree files, including `models/`, `docs/`, `runs/`, `workspace/` (the directory does not exist), `tracks_out/` (does not exist), and `.superpowers/`. Searches included XML/CSV source candidates, FSC/FSAE-named paths, MAT/NPZ candidates, and OCP-related files/references. No FSC-named XML/CSV, authoritative FSC source pair, private MAT fixture, or FSC run output was found in this worktree. Several non-FSC or derived candidates are listed below because their names or provenance could otherwise appear relevant.

## Candidate inventory and admission decisions

| Exact worktree path / referenced path | What it is | Admission decision |
|---|---|---|
| `models/tracks/fsae_2026_acceleration.xml` + `models/tracks/fsae_2026_acceleration.csv` | Open signed-XY acceleration-course geometry generated from the 2026 Formula SAE rules; XML identifies the prescribed minimum course width but has no width-role data. CSV contains centerline geometry. | **Reject for FSC.** It is a synthetic rule-course model, not the FSC endurance source; a rule minimum width is not independently supplied left/right edge data. Its open topology does not establish FSC seam/closure semantics. |
| `models/tracks/fsae_2026_skidpad.xml` + `models/tracks/fsae_2026_skidpad.csv` | Open signed-XY skidpad construction generated from rules; XML describes modeling choices and nominal driving-path width but does not reference independent left/right edge profiles. | **Reject for FSC.** It is a constructed test course, not authoritative FSC endurance geometry; scalar nominal width is not the required edge/corridor source. |
| `models/tracks/2024_michigan_endurance_closed_edged.xml`, `models/tracks/2024_michigan_endurance_closed.csv`, `models/tracks/2024_michigan_endurance_closed_width.csv` | Edged MIS endurance track; XML explicitly calls its 2.25 m half-width an assumption and cites `0615CompEndurance.mat` as geometry source. | **Reject for FSC.** Wrong event/course (MIS), and its assumed width is not FSC surveyed edge data. |
| `models/tracks/2024_michigan_endurance_closed.xml` + `models/tracks/2024_michigan_endurance_closed.csv`; likewise `models/tracks/2024_michigan_endurance_multilap_closed.xml` + matching CSV, and `models/tracks/2024_michigan_endurance_v11.xml` + matching CSV | MIS source-track variants. | **Reject for FSC.** They are Michigan endurance sources, not FSC; unedged variants also do not supply left/right edges. |
| `docs/validation/iem24-murphy/seven_dof_track_pair/original/attempt_1792091044461992000/source_snapshot/2024_michigan_endurance_closed.xml` and matching `2024_michigan_endurance_closed.csv`; `docs/validation/iem24-murphy/seven_dof_track_pair/new/attempt_1792091049300790000/source_snapshot/2024_michigan_endurance_multilap_closed.xml` and matching CSV | Archived source snapshots used for MIS validation runs. | **Reject for FSC.** These are MIS snapshots, not FSC sources. OCP/forward-run snapshots do not become FSC source geometry by reuse. |
| `docs/validation/iem24-murphy/multilap_track_candidate.xml` + `docs/validation/iem24-murphy/multilap_track_candidate.csv` | Archived MIS multilap candidate. | **Reject for FSC.** Wrong track/event and no evidence of FSC provenance. |
| `tracks_out/fsc_may_2026.xml` | Referenced by `tools/diagnostics/verify_no_tv_minimum_lap.py` as a legacy FSC OCP matrix input. `tracks_out/` is absent from the worktree; no corresponding CSV is present. | **Not available / reject.** A path reference is not an available source artifact. Even if restored, admission would require auditing its provenance, signed XY data, paired edge data, and closure/seam declaration; an old OCP input alone is not authority. |
| `/Users/alphabet1671/Dev/Simplified-Vehicle-Models/IEM24 Files/FSC-2026-May-Endurance.mat` | Private telemetry MAT path referenced in `tools/track_gui/validate_track_imports.py` and track-GUI docs. It is outside this worktree and was not found among its tracked or ignored files. | **Not admissible as-is.** It is a telemetry fixture, not an admitted signed-edge XML/CSV track. A validated, provenance-preserving import may be proposed separately, but it must produce signed XY and independent edge data; no private file was read or used in this investigation. |
| `docs/superpowers/sdd/2026-10-20-track-gui-tools/fsc-overlap.png` and `docs/superpowers/sdd/2026-10-20-track-gui-tools/acceptance.json` | Prior import-validation illustration/summary artifacts. | **Reject as geometry input.** A figure or validation report is not the source coordinate and edge data. |
| Historical FSC QSS/OCP runs, summaries, traces, profiles, or cached solver inputs under `docs/`, `runs/`, `workspace/`, `.superpowers/`, or `tracks_out/` | Search found OCP code and historical documentation/references but no FSC source XML/CSV pair in the worktree. `runs/` and `tracks_out/` are absent; no `workspace/` directory exists. | **Reject as source geometry.** Solver outputs and caches are derived evidence; they cannot supply or authorize a fabricated/retrofitted FSC path or corridor. |

The repository's committed `fsae_2026_*` files are signed XY course examples, but the word “FSAE” in their names does not identify an FSC endurance telemetry source. The available signed-edge pair is MIS and explicitly uses an assumed corridor. Neither closes the FSC admission gap.

## Required artifacts to reopen admission

Provide all of the following, with provenance sufficient to distinguish measured/authoritative track data from an OCP approximation:

1. An FSC source XML declaring a signed `closed-xy-profile` or `open-xy-profile`, with documented coordinate frame, direction, units, and closure/cut meaning.
2. Its referenced CSV containing ordered, finite signed XY path data (and any required curvature/station fields), with the original source identity/hash documented.
3. Independently supplied left and right edge coordinates or left/right width profiles in that same source direction and station convention; do not substitute a single assumed total/minimum width.
4. A documented closure/seam convention, including duplicate terminal-point policy and how the edges meet or terminate at the seam/cut.
5. Import/geometry validation that preserves source provenance and verifies alignment of path, edges, and stationing. Only after this admission should the yaw-aware runner preflight be attempted.

Until those inputs are present and admitted, FSC status remains **BLOCKED**. No FSC path, edge corridor, OCP-derived replacement geometry, simulation run, or animation was created by this investigation.
