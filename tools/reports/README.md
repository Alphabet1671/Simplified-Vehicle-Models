# reports

Report generators and plot scripts.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Package marker. |
| `compare_iem24_four_way.py` | Four-way MIS 2024 comparison: measured stint vs point mass, 7DoF and 14DoF. |
| `compare_iem24_modal_vs_corner.py` | Compare the IEM24 modal roll/heave setup with the independent-corner + rear ARB. |
| `compare_iem24_murphy_stint.py` | Compare one simulated MIS lap with the measured IEM24 Murphy stint. |
| `compare_iem24_three_way.py` | Overlay unchanged measured IEM24 laps with point-mass and planar 7-DOF runs. |
| `compare_planar_point_mass.py` | Compare the supported real-IEM24 point-mass solver on the signed MIS path. |
| `export_forward_attempt_channels.py` | Export the six MIS 2024 comparison channels from *any* forward-driver attempt. |
| `export_iem24_forward_correlation.py` | Export a freshly integrated 7DoF forward-driver lap for correlation plots. |
| `plot_iem24_endurance_correlation.py` | Draw five-channel IEM24 measured/point-mass/7DoF distance and time overlays. |
| `plot_iem24_forward_driver_comparison.py` | Generate comparison figures for IEM24 Murphy forward driver trials. |
| `plot_point_mass_planar_comparison.py` | Retired compatibility entrypoint for the historical point-mass/planar plot. |
| `report_iem24_point_mass_track_pair.py` | Standard MIS 2024 correlation report for paired point-mass track experiments. |
| `report_iem24_seven_dof_track_pair.py` | MIS 2024 correlation report for two fresh 7DoF forward-driver laps on two tracks. |
