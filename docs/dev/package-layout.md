# Package Layout

This page records the package tree of `src/simplified_vehicle_models/`. The tree follows Section 7 of the powertrain master plan. Task P0A moved the flat modules into the tree. Task P0B added the compiler package and the component packages. Neither task changed a numerical result.

## The Tree

```text
src/simplified_vehicle_models/
├── __init__.py               # The public export list
├── aero/
│   └── fixed_aero.py         # Fixed-aero compile function and capability
├── compiler/
│   ├── capabilities.py       # Capability names and Protocols
│   ├── registry.py           # (type, subtype) to component descriptor
│   ├── adapters.py           # Reduction adapters and chain resolution
│   ├── manifest.py           # Loaded, reduced, and unused records
│   └── compile.py            # compile_formulation and its helpers
├── core/
│   ├── conventions.py        # GRAVITY
│   └── types.py              # FloatArray
├── drivetrain/
│   └── lumped.py             # Lumped drivetrain compile function and capability
├── formulations/
│   └── point_mass.py         # Point-mass physics, live limits, and its compiler
├── parameters/
│   ├── vehicle.py            # PointMassVehicleParameters
│   ├── tire.py               # TireEnvelopeParameters
│   ├── aero.py               # AeroParameters
│   └── drivetrain.py         # DrivetrainParameters
├── results/
│   └── traces.py             # LapMode and LapResult
├── solvers/
│   └── forward_backward.py   # solve_lap and its helpers
├── suspension/
│   └── lookup_table.py       # Fixture suspension compile function and capability
├── tires/
│   ├── friction_ellipse.py   # Friction-ellipse compile function and capability
│   └── transient_slip.py     # Fixture transient tire compile function
├── tracks/
│   ├── types.py              # TrackProfile
│   └── profile.py            # load_track and its helpers
└── xml/
    ├── loader.py             # Parse, resolve a reference, and validate a model
    ├── model.py              # Model and ModelReference
    └── schema.py             # Parameter schema and range checks
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

Task P0B moved one more group. The per-component compile functions left `xml/loader.py`:

| Module | Moved from |
|---|---|
| `tires/friction_ellipse.py` | `_compile_tire` in `xml/loader.py` |
| `aero/fixed_aero.py` | `_compile_aero` in `xml/loader.py` |
| `drivetrain/lumped.py` | `_compile_drivetrain` in `xml/loader.py` |
| `formulations/point_mass.py` | `_compile_vehicle` in `xml/loader.py` |
| `xml/schema.py` | The parameter checks in `xml/loader.py` |

## The One Rename

`xml/model.py` holds the two dataclasses. Two modules share them now. The task renamed the classes for that reason.

| Old name | New name |
|---|---|
| `_Reference` | `ModelReference` |
| `_Model` | `Model` |

No other name changed.

## The Compiler

The compile path runs from a formulation to a compiled model.

1. `xml/loader.py` parses one file, resolves one reference, and validates the model type against its registry descriptor.
2. `compiler/compile.py` collects the assembly components and resolves each requested capability.
3. `compiler/adapters.py` selects a direct provider or the shortest adapter chain. The choice is deterministic.
4. The formulation compiler builds the compiled model. Only the components that satisfy a request are compiled.
5. `compiler/manifest.py` records the loaded components, the capability decisions, the unused components, and the unconsumed parameters. The compiled model holds its manifest.

A component module declares its capability and registers itself. `compiler/registry.py` imports every component module one time.

## The Parameter Rule

A component owns a closed schema. An unknown parameter in a component is an error.

An assembly owns an open schema. The compiler records each unknown assembly parameter as unconsumed. The compiler does not raise. An assembly may also carry a component that the formulation does not request. The compiler records that component as unused and never compiles it.

A required parameter that is absent is always an error.

## Import Rules

- Import a shared type from `core/`. For example, use `from simplified_vehicle_models.core.types import FloatArray`.
- Import a vehicle parameter class from its own module in `parameters/`.
- Import a capability name from `compiler/capabilities.py`. Do not write a capability name as a literal.
- Keep the public export list in the top-level `__init__.py`. A caller can still write `from simplified_vehicle_models import load_track`.
- Do not import a private helper across a package boundary.
