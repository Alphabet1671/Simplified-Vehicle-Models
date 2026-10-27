# maneuver_gui

Simulation-only maneuver execution and its Qt widgets. The maneuver tool writes simulation results; video rendering belongs exclusively to the animator GUI.

## Launch from the repository venv

```bash
.venv/bin/python tools/maneuver_gui/app.py models/vehicles/iem24_murphy_real_14dof.xml
```

After `python -m pip install -e '.[gui,forward]'` in the activated venv, `maneuver-gui` opens the shared simulation window with the Maneuvers tab selected. Direct-script launches also work from another working directory when given absolute script/model paths; no `PYTHONPATH` setup is needed.

Choose a vehicle, configure the maneuver and run it. A completed attempt publishes `run_record.npz` in the same `run-record-v1` format as lap simulations. Open that file (or its study/folder) in the animator GUI to inspect or render it. The nested `result/` directory retains hash-checked simulation evidence; its raw arrays are not the public simulation-result handoff. Failed or cancelled attempts do not advertise a completed RunRecord.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Headless, simulation-only maneuver execution and artifact helpers. |
| `app.py` | Launch the shared simulation GUI directly in Maneuvers mode. |
| `artifacts.py` | Safe atomic artifacts for headless maneuver executions. |
| `runner.py` | Headless subprocess boundary for simulation-only maneuver definitions. |

## Subdirectories

| Directory | Purpose |
|---|---|
| `examples/` | Example maneuver definitions. |
| `qt/` | Qt layer of the maneuver tools. |
