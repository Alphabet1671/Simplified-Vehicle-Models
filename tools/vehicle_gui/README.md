# vehicle_gui

Vehicle GUI.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Vehicle GUI: a Qt-free core and a thin PyQt6 layer (tools/vehicle_gui/qt). |
| `app.py` | Start the vehicle GUI: ``PYTHONPATH=src:. python3 tools/vehicle_gui/app.py [model.xml]``. |
| `model.py` | An editable vehicle: parameter edits and component swaps on top of a configuration. |
| `save.py` | Turn a ``VehicleEdit`` into files, and check them with the real compiler. |
| `xmltext.py` | Edit model XML as text while preserving comments, spacing and line order. |

## Subdirectories

| Directory | Purpose |
|---|---|
| `qt/` | Qt layer of the vehicle GUI. |
