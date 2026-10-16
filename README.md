# Simplified Vehicle Models

Python models for point-mass laps and driven planar 7-DoF transients. **Simulation and numerical acceptance are not measured-vehicle validation.** The retired planar optimal-control solver is not a supported interface.

- [Project guide](docs/project-master-plan.md)
- [Model-data provenance](docs/model-data/provenance.md)
- [Validation reports and generated artifacts](docs/validation/)
- [Tools and reproducibility commands](docs/dev/tools-layout.md)
- [MIS report standard](docs/standards/mis-2024-report-standard.md)

Install from the repository root with `python -m pip install -e '.[forward]'`. Run the point-mass example with `PYTHONPATH=src python examples/run_mis_point_mass.py`. Run the bounded 7-DoF forward driver with `PYTHONPATH=src:. python -m tools.run.run_iem24_forward_driver --help`. Every generated report defaults to a path under `docs/validation/`; `tools/` contains programs, not documentation.

The [IEM24 acceptance chronology](docs/validation/iem24-murphy/seven_dof_track_pair/acceptance_gate_trials.md) distinguishes the original lap, the failed canonical finer attempt, and the later held-out canonical fourth refinement. Raw run6 polynomials and traces are not committed, so certificates cannot be replayed from a fresh checkout without regenerating them.

IEM’s 2024 EV race car “Murphy” supplied the modeled vehicle and Matt Northrop drove the measured endurance laps used for correlation.
