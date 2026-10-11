# Package Layout

This page records the package tree of `src/simplified_vehicle_models/`. The tree follows Section 7 of the powertrain master plan. Task P0A moved the flat modules into the tree. The move changed no behavior.

## The Tree

```text
src/simplified_vehicle_models/
├── __init__.py               # The public export list
├── core/
│   ├── conventions.py        # GRAVITY
│   └── types.py              # FloatArray
├── formulations/
│   └── point_mass.py         # Point-mass physics and live limits
├── parameters/
│   ├── vehicle.py            # PointMassVehicleParameters
│   ├── tire.py               # TireEnvelopeParameters
│   ├── aero.py               # AeroParameters
│   └── drivetrain.py         # DrivetrainParameters
├── results/
│   └── traces.py             # LapMode and LapResult
├── solvers/
│   └── forward_backward.py   # solve_lap and its helpers
├── tracks/
│   ├── types.py              # TrackProfile
│   └── profile.py            # load_track and its helpers
└── xml/
    ├── loader.py             # The loader and the compile functions
    └── model.py              # Model and ModelReference
```

Each package has an empty `__init__.py`. Only the top-level `__init__.py` holds code. Import a name from the module that defines it. Do not import from a package `__init__.py`.

## The Moves

The table lists the source of every module in the tree.

| Module | Moved from |
|---|---|
| `core/conventions.py` | The `GRAVITY` constant in `point_mass.py` |
| `core/types.py` | The `FloatArray` alias in `point_mass.py`, `tracks.py`, and `lap_solver.py` |
| `formulations/point_mass.py` | `point_mass.py`, moved whole |
| `parameters/vehicle.py` | The `PointMassVehicleParameters` class in `parameters.py` |
| `parameters/tire.py` | The `TireEnvelopeParameters` class in `parameters.py` |
| `parameters/aero.py` | The `AeroParameters` class in `parameters.py` |
| `parameters/drivetrain.py` | The `DrivetrainParameters` class in `parameters.py` |
| `results/traces.py` | The `LapMode` and `LapResult` classes in `lap_solver.py` |
| `solvers/forward_backward.py` | `solve_lap` and its helpers in `lap_solver.py` |
| `tracks/types.py` | The `TrackProfile` class in `tracks.py` |
| `tracks/profile.py` | `load_track` and its helpers in `tracks.py` |
| `xml/loader.py` | The loader and the compile functions in `xml_loader.py` |
| `xml/model.py` | The `_Reference` and `_Model` dataclasses in `xml_loader.py` |

## The One Rename

`xml/model.py` holds the two dataclasses. Two modules share them now. The task renamed the classes for that reason.

| Old name | New name |
|---|---|
| `_Reference` | `ModelReference` |
| `_Model` | `Model` |

No other name changed.

## Import Rules

- Import a shared type from `core/`. For example, use `from simplified_vehicle_models.core.types import FloatArray`.
- Import a vehicle parameter class from its own module in `parameters/`.
- Keep the public export list in the top-level `__init__.py`. A caller can still write `from simplified_vehicle_models import load_track`.
- Do not import a private helper across a package boundary.
