# Simplified Vehicle Models

Python models for point-mass laps and driven planar 7-DoF transients. **Simulation and numerical acceptance are not measured-vehicle validation.** The retired planar optimal-control solver is not a supported interface.

- [Project guide](docs/project-master-plan.md)
- [Model-data provenance](docs/model-data/provenance.md)
- [Validation reports and generated artifacts](docs/validation/)
- [Tools and reproducibility commands](docs/dev/tools-layout.md)
- [Forward driver solver (7DoF and 14DoF)](docs/dev/forward-driver.md)
- [Computational cost by model](docs/dev/computational-cost.md)
- [Simulation GUI](docs/dev/simulation-gui.md)
- [Open-loop maneuver GUI and commissioning limits](docs/dev/maneuver-gui.md)
- [MIS report standard](docs/standards/mis-2024-report-standard.md)

## Install and run

From the repository root, create and activate a local virtual environment, then install the GUI and forward-driver extras:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[gui,forward]'
```

Start the simulation GUI from the repository root:

```bash
PYTHONPATH=src:. python tools/sim_gui/app.py
```

To open a vehicle model on startup, pass its XML path, for example:

```bash
PYTHONPATH=src:. python tools/sim_gui/app.py models/vehicles/iem24_murphy_real_planar.xml
```

The GUI runs forward-driver studies; it supports `planar-7dof` and `14dof-simple` models (not point-mass). See [Simulation GUI](docs/dev/simulation-gui.md) for model selection, studies, outputs, and limitations. The virtual environment is local to this checkout and is ignored by Git.

Run the point-mass example with `PYTHONPATH=src python examples/run_mis_point_mass.py`. Run the bounded 7-DoF forward driver with `PYTHONPATH=src:. python -m tools.run.run_iem24_forward_driver --help`. Every generated report defaults to a path under `docs/validation/`; `tools/` contains programs, not documentation.

The [IEM24 acceptance chronology](docs/validation/iem24-murphy/seven_dof_track_pair/acceptance_gate_trials.md) distinguishes the original lap, the failed canonical finer attempt, and the later held-out canonical fourth refinement. Raw run6 polynomials and traces are not committed, so certificates cannot be replayed from a fresh checkout without regenerating them.

IEM’s 2024 EV race car “Murphy” supplied the modeled vehicle and Matt Northrop drove the measured endurance laps used for correlation.
