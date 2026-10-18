# Task 1 report

## Built

- Added the Qt-free `tools.sim_gui` package.
- Added repository-root child process environment setup.
- Added frozen `ModelEntry` and XML catalogue scanning.
- Matched model roots against `simplified_vehicle_models.compiler.registry.component_descriptor`.
- Added explicit track profile handling because the registry does not register track models.
- Added candidate matching, signed-track filtering, and skipped-file reporting.
- Added the `gui = ["PyQt6>=6.5"]` optional extra.
- Ignored `/runs/` and `/models/sweeps/`.

## TDD evidence

RED command:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. python3 -m unittest tests.test_sim_gui_core -v
```

Result: failed during import with `ModuleNotFoundError: No module named 'tools.sim_gui'`.

GREEN command:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. python3 -m unittest tests.test_sim_gui_core tests.test_default_track -v
```

Result: `Ran 11 tests ... OK`.

## Files changed

- `.gitignore`
- `pyproject.toml`
- `tools/sim_gui/__init__.py`
- `tools/sim_gui/catalogue.py`
- `tests/test_sim_gui_core.py`

## Concerns

The brief says every XML is matched against the compiler registry. The real registry has no track descriptors, while the authoritative tests require signed track entries. Track roots are therefore accepted as `kind="track"` outside the registry. The real lookup function is `component_descriptor`, as named in the brief's implementation example.
