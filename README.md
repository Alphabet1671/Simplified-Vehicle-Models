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

### Simulation-only open-loop maneuvers

Run a checked-in open-loop maneuver definition headlessly with the simulation-only runner. This example writes an immutable attempt/result under `runs/open-loop-step/`:

```bash
PYTHONPATH=src:. python -m tools.maneuver_gui.runner \
  --definition tools/maneuver_gui/examples/iem24_14dof/step.json \
  --output-root runs/open-loop-step
```

Export a completed 14DoF maneuver artifact to an animator-compatible MP4 and its adjacent RunRecord. The output path must not already exist:

```bash
PYTHONPATH=src:. python -m tools.maneuver_gui.export_animation \
  --artifact runs/open-loop-step/<attempt>/result \
  --output runs/open-loop-step.mp4
```

The runner accepts simulation definitions only; it has no vehicle transport, CAN, analog I/O, or hardware arming path. See the [open-loop maneuver guide](docs/dev/maneuver-gui.md) for definitions, CSV inputs, native study/case/attempt outputs, queue workflow, animator interoperability, and limitations.

Run the point-mass example with `PYTHONPATH=src python examples/run_mis_point_mass.py`. Run the bounded 7-DoF forward driver with `PYTHONPATH=src:. python -m tools.run.run_iem24_forward_driver --help`. Every generated report defaults to a path under `docs/validation/`; `tools/` contains programs, not documentation.

The [IEM24 acceptance chronology](docs/validation/iem24-murphy/seven_dof_track_pair/acceptance_gate_trials.md) distinguishes the original lap, the failed canonical finer attempt, and the later held-out canonical fourth refinement. Raw run6 polynomials and traces are not committed, so certificates cannot be replayed from a fresh checkout without regenerating them.

IEM’s 2024 EV race car “Murphy” supplied the modeled vehicle and Matt Northrop drove the measured endurance laps used for correlation.
