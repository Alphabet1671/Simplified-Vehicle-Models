# Simplified Vehicle Models

Simplified vehicle models for Formula SAE lap simulation: a Python library, an XML model graph, runnable tools, and a test suite.

## Install and run

**Simulation and numerical acceptance are not measured-vehicle validation.** The retired planar optimal-control solver is not a supported interface.

From the repository root, create and activate a local virtual environment, then install the GUI and forward-driver extras:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[gui,forward]'
```

Start the simulation GUI directly with the venv:

```bash
.venv/bin/python tools/sim_gui/app.py
```

After the editable install, `maneuver-gui` opens its Maneuvers mode. For tire correlation, install `python -m pip install -e '.[tire-gui]'` and run `tire-gui`. Neither requires `PYTHONPATH`. Maneuver simulations write canonical simulation result files; use the animator GUI separately for visualization and video export.

Run the tests:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -q
```

IEM's 2024 EV race car "Murphy" supplied the modeled vehicle and Matt Northrop drove the measured endurance laps used for correlation.

## Files

| File | Contents |
|---|---|
| `.gitignore` | File. |
| `AGENTS.md` | Markdown file. |
| `LICENSE` | File. |
| `pyproject.toml` | TOML file. |

## Subdirectories

| Directory | Purpose |
|---|---|
| `docs/` | Data kept with the project that is not code: model-data provenance, validation artifacts, and documentation examples. |
| `examples/` | Runnable example scripts. |
| `models/` | XML model graph: one subdirectory per model type. |
| `requirements/` | Pinned dependency lists. |
| `src/` | Importable source packages. |
| `tests/` | The unit and regression suite, run with `unittest`. |
| `tools/` | Runnable programs: studies, report generators, data import/export and GUIs. Programs only, no documentation. |
