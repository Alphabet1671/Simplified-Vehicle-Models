# track_gui

Track GUI.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Track GUI: a Qt-free core and a thin PyQt6 layer (tools/track_gui/qt). |
| `app.py` | Start the track GUI: ``PYTHONPATH=src:. python3 tools/track_gui/app.py [design.json]``. |
| `closed.py` | Build a closed track from MAT telemetry, for the track GUI. |
| `design.py` | An editable open-course design: a section list and the build settings. |
| `export.py` | Write and check track files for the track GUI. |
| `lap_average.py` | Reference-free spatial loop detection, registration and robust mean geometry. |
| `mat_channels.py` | Qt-free MAT telemetry channel discovery and synchronized coordinate loading. |
| `track_ops.py` | Geometry-only track operations and safe exports for the track GUI. |
| `validate_track_imports.py` | Run real MAT acceptance checks for closed-track telemetry imports. |

## Subdirectories

| Directory | Purpose |
|---|---|
| `qt/` | Qt layer of the track GUI. |
