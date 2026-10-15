# Simplified Vehicle Models

A Python toolkit for compiling vehicle models from XML and simulating laps on prescribed tracks. The currently supported lap solvers are:

- **Point-mass spatial lap simulation**: forward/backward speed-profile solver with an optional energy/SOC-coupled calculation.
- **Planar 7-DoF transient simulation**: time-domain forward driver with body motion, yaw, four dynamic wheel speeds, tire/load coupling, and battery states. Uses simplified Unitire tire model, correlated with TTC test data.

The former planar optimal-control (OCP) solver and its point-mass seed APIs are retired and are not part of the supported interface. Do not interpret archived OCP lap times or solver iterates as current validation results.

## Requirements

- Python 3.12 or newer
- NumPy 2.0 or newer
- SciPy 1.11 or newer for the transient 7-DoF forward driver
- Matplotlib for tools that generate plots

Install the package and optional forward-solver dependency from the repository root:

```bash
python -m pip install -e '.[forward]'
```

For point-mass-only use, install without the optional extra if desired:

```bash
python -m pip install -e .
```

## Run a point-mass lap

A compact example runs the Murphy point-mass formulation on the Michigan endurance track and prints flying-lap and standing-start results:

```bash
PYTHONPATH=src python examples/run_mis_point_mass.py
```

Include energy/SOC traces (and optionally progress messages) with:

```bash
PYTHONPATH=src python examples/run_mis_point_mass.py --energy --progress
```

For the real-parameter IEM24 Murphy point-mass model on the signed closed MIS centreline, run:

```bash
PYTHONPATH=src python tools/run_iem24_murphy_lap.py
```

This writes a simulated trace and summary under `docs/validation/iem24-murphy/` by default. Choose a different output directory or track with `--output-dir` and `--track`. The solver output is a model simulation, not by itself evidence of agreement with measured telemetry; see the IEM24 report for assumptions and comparison methodology.

## Run the planar 7-DoF transient driver

The 7-DoF plant is available through a bounded forward-driver prototype. It simulates a driven lap with dynamic wheel speeds; it does not solve for a minimum-time racing line or periodic hot lap.

```bash
PYTHONPATH=src python tools/run_iem24_forward_driver.py
```

The parent runner creates a unique attempt directory beneath `docs/validation/iem24-murphy/forward_driver_trials/`, applies a wall-time cap (default 300 seconds), and saves run logs and a machine-readable summary. Defaults include a conservative speed planner, 0.5 m path spacing, Radau integration, and an 8 m/s maximum speed. For example, select the live point-mass planner and BDF integration:

```bash
PYTHONPATH=src python tools/run_iem24_forward_driver.py \
  --planner live --method BDF --maximum-speed 60 \
  --spacing 0.5 --timeout-seconds 300
```

Options and their defaults are shown by:

```bash
PYTHONPATH=src python tools/run_iem24_forward_driver.py --help
```

A process may complete a lap while the result remains unusable: the acceptance gate also requires periodicity, path tracking, resolution, physical checks between integration steps, refinement evidence, and contact validity. The current prototype does not establish all of these gates on its recorded full-lap runs; summaries may correctly report `usable: false`. Do not compare such iterates as valid lap times or call them validation evidence.

## Models and data

- `models/formulations/`: selectable model formulations, including point-mass and planar 7-DoF models.
- `models/vehicles/`, `models/powertrains/`, `models/tires/`, and related directories: XML component graphs and parameter sets.
- `models/tracks/`: track geometry and distance/radius profiles. The real-IEM24 comparisons use signed closed-path geometry so turn direction is retained.
- `src/simplified_vehicle_models/`: XML compiler, vehicle physics, parameter types, and solvers.
- `tools/`: runnable simulations, telemetry processing, comparison, and plotting utilities.

Shoutout to IEM’s 2024 EV race car “Murphy” for providing means to validate the solver, and Matt Northrop for driving the endurance laps used for correlation.
