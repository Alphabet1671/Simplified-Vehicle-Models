# simplified_vehicle_models

The library: XML compiler, models, solvers.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Package marker. |
| `gui_launchers.py` | Console-script adapters for GUI tools that live in the source checkout. |

## Subdirectories

| Directory | Purpose |
|---|---|
| `aero/` | Aerodynamic component code. |
| `compiler/` | The XML model compiler: registry, capabilities, reduction adapters. |
| `controls/` | Driver-input conversion and controller contracts. |
| `core/` | Shared conventions, types and numerical helpers. |
| `drivetrain/` | Drivetrain component code. |
| `evolution/` | Fixed-rate co-simulation and torque-vectoring controller contracts. |
| `formulations/` | Vehicle formulations and capability sampling. |
| `hydraulics/` | Hydraulic circuit elements and solver. |
| `parameters/` | Parameter containers for the vehicle, tire, aero and drivetrain. |
| `powertrain/` | Electric powertrain components. |
| `results/` | Run-record and trace writers. |
| `robot/` | Fake-only robot execution contract. |
| `solvers/` | Lap, line, forward-driver and maneuver solvers, with their certificates and evidence helpers. |
| `suspension/` | Suspension components and the force-network seam. |
| `tires/` | Tire component code. |
| `tracks/` | Track geometry code. |
| `xml/` | XML loading and validation. |
