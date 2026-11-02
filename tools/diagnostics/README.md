# diagnostics

Diagnostic and profiling scripts.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Package marker. |
| `audit_speed_derivative_resolution.py` | Analytic evidence study for the independent speed derivative heuristic. |
| `compare_qss_track_boundaries.py` | Bounded, non-mutating closed/cut-open QSS comparisons. |
| `diagnose_speed_cap.py` | Diagnose why forward driver speed is capped below 8 m/s. |
| `generate_minimum_time_cli_fixture.py` | Create a fresh self-contained synthetic fixture for public spline CLI smoke runs. |
| `no_tv_provenance.py` | Capture immutable, actual runtime provenance for no-TV matrix attempts. |
| `no_tv_runtime.py` | Report the Python environment actually executing the no-TV tools. |
| `profile_models.py` | Profile one lap of the point-mass, planar 7DoF or vertical 14DoF model; forward profiles use the current sampled braking-envelope policy. |
| `verify_no_tv_minimum_lap.py` | Bounded, reproducible no-TV minimum-lap experiment matrix. |
