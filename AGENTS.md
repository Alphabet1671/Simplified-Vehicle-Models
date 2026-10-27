# Repository instructions

Documentation is local: every tracked directory has a `README.md` that says what the directory holds and what each subdirectory is for. It does not describe the contents of subdirectories. Update the `README.md` of a directory when you add, remove or rename files or subdirectories in it.

There is no central documentation folder. Agent working directories, plans, specs, task reports and the former central documentation are not tracked. They live under the ignored `.superpowers/` (archives in `.superpowers/archive/`).

## Where things live

| Path | Contents |
|---|---|
| `src/simplified_vehicle_models/` | The library: XML compiler, models, solvers |
| `models/` | XML model graph: vehicles, tires, aero, powertrains, tracks |
| `tools/` | Runnable studies, report generators, and data import/export |
| `docs/` | Generated validation artifacts, model-data provenance and documentation examples |
| `tests/` | The unit and regression suite |

## Commands

```bash
# Full test suite.
PYTHONPATH=src python3 -m unittest discover -s tests -q

# Compile a formulation and inspect the resolved model.
PYTHONPATH=src python3 -c "from simplified_vehicle_models.compiler.compile import compile_formulation; print(compile_formulation('models/formulations/murphy_point_mass.xml').model)"
```

Some tools need CasADi (planar optimal control) and the bundled wheels under
`/tmp/casadi_pkgs`; the tools state their own `PYTHONPATH` in their docstrings.
