# model_data

Generators and exporters for the files in `models/`.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Package marker. |
| `build_capability_surface.py` | Build and cache a point-mass quasi-steady capability surface. |
| `build_signed_capability.py` | Build an exclusive opt-in signed transient capability archive. |
| `derive_bump_steer.py` | Derive bump-steer (toe versus jounce) curves from suspension hardpoints. |
| `derive_steering_lift.py` | Derive the front steering-lift coefficients from steering-axis hardpoints. |
| `export_legacy_powertrain_data.py` | Export the legacy powertrain data into ``models/data``. |
| `export_unitire_simple_tire.py` | Export the fitted simplified UniTire parameters to one XML model. |
| `generate_component_sweep.py` | Generate deterministic component-file sweeps for simulation studies. |
| `generate_damper_lut.py` | Deterministically generate a damper force table NPZ and damper model XMLs. |
| `generate_hydraulic_circuit.py` | Deterministically generate a hydraulic circuit NPZ and its component XML. |
| `generate_motor_lut.py` | Deterministically generate the canonical motor operating map. |
| `generate_suspension_lut.py` | Deterministically generate the corner suspension table NPZ and its model XML. |
| `generate_unitire_reference.m` | MATLAB script. |
| `identify_motor_parameters.py` | Identify the AMK DD5 machine parameters from the exported map. |
| `import_real_mode_dampers.py` | Import measured heave and roll mode damper tables into pinned NPZ models. |
| `xml_references.py` | Utilities for moving model XML files while preserving references. |
