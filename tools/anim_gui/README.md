# anim_gui

Animation GUI.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Animation GUI: a Qt-free core and a thin PyQt6 layer (tools/anim_gui/qt). |
| `animation_gui_app.py` | Start the animation GUI: ``PYTHONPATH=src:. python3 tools/anim_gui/animation_gui_app.py [run_record.npz]``. |
| `animation_gui_settings.py` | Export settings and the command line they make. |
| `inspect_plan.py` | Read a record and describe what an export would do. |
| `progress.py` | Read the exporter's output lines. |
| `records.py` | Find run records: one file, a folder, or the cases of a ``study.json``. |

## Subdirectories

| Directory | Purpose |
|---|---|
| `qt/` | Qt layer of the animation GUI. |
