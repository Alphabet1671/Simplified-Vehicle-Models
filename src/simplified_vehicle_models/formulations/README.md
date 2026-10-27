# formulations

Vehicle formulations and capability sampling.

## Files

| File | Contents |
|---|---|
| `__init__.py` | Package marker. |
| `capability_backend.py` | Capability backends used by the line OCP; controller behavior is supplied. |
| `controller_capability.py` | Controller-aware capability sampling; the policy is an oracle, not a plant. |
| `planar_7dof.py` | Compile the planar 7-DOF vehicle and evaluate its quasi-static loads. |
| `point_mass.py` | Python module `point_mass`. |
| `point_mass_capability.py` | Sample the quasi-steady capability of the point-mass vehicle onto a grid. |
| `signed_capability.py` | Opt-in, physically signed transient capability charts. |
| `vertical_14dof.py` | Compile the vertical 14-DOF (``14dof-simple``) vehicle parameters. |
