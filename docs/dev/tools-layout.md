# Tools Layout

`tools/` holds runnable studies, report generators, and data import/export scripts. The scripts were once a flat directory. They now sit in category subpackages. The move changed no numerical result.

Run a tool from the repository root by its file path, for example `PYTHONPATH=src:. python3 tools/run/run_iem24_forward_driver.py`. Tools import each other as `tools.<category>.<module>`, which is why `.` is on `PYTHONPATH` for those that do.

## Categories

| Category | Purpose |
|---|---|
| `tools/data/` | Import and condition measured data: telemetry, vehicle spec sheets |
| `tools/tracks/` | Build closed track geometry; `open_course.py` builds open courses from straight, arc, and clothoid sections, and `build_fsae_dynamic_tracks.py` makes the FSAE acceleration and skidpad tracks. See [fsae-dynamic-tracks.md](fsae-dynamic-tracks.md) |
| `tools/model_data/` | Generate or export model data: motor map, tire, legacy powertrain; `generate_component_sweep.py` writes one component file per value for a GUI sweep |
| `tools/sim_gui/` | Run simulations from a GUI: component swaps, sweeps, run records. See [simulation-gui.md](simulation-gui.md) |
| `tools/anim_gui/` | Export a run record as a video or PNG frames from a GUI. See [animation-gui.md](animation-gui.md) |
| `tools/track_gui/` | Design open courses or build closed tracks from a GUI. See [track-gui.md](track-gui.md) |
| `tools/gui_common/` | Code that the three GUIs share: safe file writes, a cancelable child process |
| `tools/vehicle_gui/` | Edit vehicle parameters and swap components from a GUI. See [vehicle-gui.md](vehicle-gui.md) |
| `tools/run/` | Run the forward driver and the paired point-mass solver |
| `tools/gates/` | Acceptance, refinement, tube, and between-step checks |
| `tools/reports/` | Comparison tools, report writers, plots, correlation export |
| `tools/diagnostics/` | One-off diagnosis of speed caps, trace commands, accelerations |
| `tools/anim/` | Export a run record as a 120 fps video: frame source, panels, ffmpeg encoder, CLI. See [animation-export.md](animation-export.md) |

## Old to new paths

| Old path | New path |
|---|---|
| `tools/iem24_telemetry.py` | `tools/data/iem24_telemetry.py` |
| `tools/extract_iem24_telemetry.py` | `tools/data/extract_iem24_telemetry.py` |
| `tools/import_vehicle_spec_sheet.py` | `tools/data/import_vehicle_spec_sheet.py` |
| `tools/build_closed_track.py` | `tools/tracks/build_closed_track.py` |
| `tools/build_iem24_multilap_track.py` | `tools/tracks/build_iem24_multilap_track.py` |
| `tools/generate_motor_lut.py` | `tools/model_data/generate_motor_lut.py` |
| `tools/identify_motor_parameters.py` | `tools/model_data/identify_motor_parameters.py` |
| `tools/export_unitire_simple_tire.py` | `tools/model_data/export_unitire_simple_tire.py` |
| `tools/generate_unitire_reference.m` | `tools/model_data/generate_unitire_reference.m` |
| `tools/export_legacy_powertrain_data.py` | `tools/model_data/export_legacy_powertrain_data.py` |
| `tools/run_iem24_forward_driver.py` | `tools/run/run_iem24_forward_driver.py` |
| `tools/run_iem24_point_mass_track_pair.py` | `tools/run/run_iem24_point_mass_track_pair.py` |
| `tools/check_iem24_forward_refinement.py` | `tools/gates/check_iem24_forward_refinement.py` |
| `tools/check_iem24_forward_tube.py` | `tools/gates/check_iem24_forward_tube.py` |
| `tools/check_iem24_forward_tube_v2.py` | `tools/gates/check_iem24_forward_tube_v2.py` |
| `tools/certify_iem24_forward_between_step.py` | `tools/gates/certify_iem24_forward_between_step.py` |
| `tools/compare_iem24_murphy_stint.py` | `tools/reports/compare_iem24_murphy_stint.py` |
| `tools/compare_iem24_three_way.py` | `tools/reports/compare_iem24_three_way.py` |
| `tools/compare_planar_point_mass.py` | `tools/reports/compare_planar_point_mass.py` |
| `tools/report_iem24_point_mass_track_pair.py` | `tools/reports/report_iem24_point_mass_track_pair.py` |
| `tools/report_iem24_seven_dof_track_pair.py` | `tools/reports/report_iem24_seven_dof_track_pair.py` |
| `tools/plot_iem24_endurance_correlation.py` | `tools/reports/plot_iem24_endurance_correlation.py` |
| `tools/plot_iem24_forward_driver_comparison.py` | `tools/reports/plot_iem24_forward_driver_comparison.py` |
| `tools/plot_point_mass_planar_comparison.py` | `tools/reports/plot_point_mass_planar_comparison.py` |
| `tools/export_iem24_forward_correlation.py` | `tools/reports/export_iem24_forward_correlation.py` |
| `tools/diagnose_speed_cap.py` | `tools/diagnostics/diagnose_speed_cap.py` |
| `tools/analyze_trace_commands.py` | `tools/diagnostics/analyze_trace_commands.py` |
| `tools/recompute_forward_driver_accelerations.py` | `tools/diagnostics/recompute_forward_driver_accelerations.py` |

## Scripts that are not in the tree

These names appear in older documents but no script with that name exists in the current tree. They were not part of the move and are not mapped.

| Name | Where it appears |
|---|---|
| `tools/run_iem24_murphy_lap.py` | Reproduce blocks of `docs/validation/iem24-murphy/report.md` and `model_run.md`, and the reference table of the MIS report standard |
| `tools/solve_planar_mesh.py` | `docs/validation/planar-ocp.md` |
| `tools/make_forward_driver_report.py` | Archived attempt report under `docs/superpowers/sdd/` (labelled "proposed") |
| `tools/run_iem24_murphy_planar_7dof.py`, `tools/run_iem24_mesh_sensitivity.py` | Planned in `docs/superpowers/plans/` only |

## Historical commands

Some documents record the exact command that produced an artifact. These keep the pre-migration flat path on purpose, because rewriting them would break the record or a frozen hash:

- Anything under an `attempt_*` directory (`command.txt`, `parent.json`, `source_snapshot/`, reports). The runner hashed the sources at the flat path.
- `docs/validation/iem24-murphy/seven_dof_track_pair/tube_v2_protocol.md` and its `tube_v2_protocol.freeze.txt`. The protocol hash is frozen and names `tools/check_iem24_forward_tube.py`, `tools/check_iem24_forward_tube_v2.py` and `tools/run_iem24_forward_driver.py`.
- `docs/superpowers/plans/`, `docs/superpowers/specs/` and `docs/superpowers/sdd/`.

To re-run a historical command, replace the flat path with the new path from the table. The script bytes are unchanged apart from path depth and imports, except that `tools/gates/check_iem24_forward_tube.py` is preserved byte for byte, so its SHA-256 `57b92964f3cc13057a230f7c72e866e2df6a7742d6c4931e27aaec511ee200ab` still holds at its new location.

User-facing validation documents under `docs/validation/` use the new paths and carry a migration note where they recorded commands from before the move. Reports whose numbers come from earlier runs keep their recorded SHA-256 values.
