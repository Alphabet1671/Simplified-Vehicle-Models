# iem24-murphy

Artifacts for the 2024 Michigan endurance stint of the IEM24 Murphy car, one subdirectory per study.

## Files

| File | Contents |
|---|---|
| `comparison.csv` | CSV table. |
| `comparison.png` | Figure. |
| `comparison_summary.json` | JSON record. |
| `comparison_three_way_distance.png` | Figure. |
| `comparison_three_way_summary.json` | JSON record. |
| `comparison_three_way_time.png` | Figure. |
| `lap_spread.png` | Figure. |
| `measured_stint.csv` | CSV table. |
| `multilap_track_candidate.csv` | CSV table. |
| `multilap_track_candidate.xml` | `iem24-multilap-driven-line-candidate` model, type `track-model`, subtype `closed-xy-profile`. |
| `multilap_track_candidate_diagnostics.json` | JSON record. |
| `simulated_lap.csv` | CSV table. |
| `simulated_lap.png` | Figure. |
| `simulated_lap_summary.json` | JSON record. |
| `telemetry_extraction.json` | JSON record. |

## Subdirectories

| Directory | Purpose |
|---|---|
| `14dof_simple_trials/` | Forward-driver attempts of the vertical 14-DoF model on the endurance lap. |
| `animation-export/` | Figures from the animation export of an endurance lap. |
| `contact_gate_trials/` | Forward-driver attempts run for the tire contact-load gate. |
| `current_7dof_live_trials/` | Forward-driver attempts of the planar 7-DoF model with live runs. |
| `current_7dof_trials/` | Forward-driver attempts of the planar 7-DoF model. |
| `forward-usability-fixes/` | Attempts that check the forward-driver usability fixes, grouped by model. |
| `forward-usability-sweep/` | Attempts of the forward-driver usability sweep, grouped by model setup. |
| `forward_driver_trials/` | Early forward-driver attempts. |
| `four_way_latest_model/` | Four-way comparison of the measured stint with the point-mass, 7-DoF and 14-DoF models. |
| `mesh_sensitivity/` | Mesh-spacing sensitivity study of the endurance-lap optimal-control problem, one subdirectory per spacing. |
| `midpoint_gate_trials/` | Forward-driver attempts run for the midpoint gate. |
| `modal_vs_corner_arb/` | Comparison of the modal roll/heave suspension with independent corners plus an anti-roll bar. |
| `point_mass_track_pair/` | Point-mass solves on two signed tracks, one subdirectory per track. |
| `refinement_gate_trials/` | Forward-driver attempts run for the refinement gate. |
| `seven_dof_track_pair/` | 7-DoF forward-driver laps on two tracks, one subdirectory per track, with the comparison outputs. |
| `task2_forward_trial/` | Forward-driver attempt of an early trial. |
| `three_way_recheck_2026-10-15/` | Re-run of the three-way comparison of 2026-10-15 (historical, not validation evidence). |
| `warm_start_trials/` | Forward-driver attempts that test warm starts. |
