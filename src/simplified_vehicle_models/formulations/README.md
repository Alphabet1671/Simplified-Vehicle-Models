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
| `yaw_map_parallel.py` | Bounded spawn-process yaw-cell evaluation and ordered batches. |
| `yaw_map_physical.py` | Cached direct-cell physical yaw-map evaluation and batches. |
| `yaw_map_domain.py` | Resolution-limited signed yaw-domain discovery. |
| `yaw_map_workflow.py` | Physical yaw-map build, independent audits, and bounded refinement. |
| `yaw_map_artifact.py` | Integrity-checked physical yaw-map artifacts and source identity. |
| `signed_capability.py` | Opt-in, physically signed transient capability charts. |
| `vertical_14dof.py` | Compile the vertical 14-DOF (``14dof-simple``) vehicle parameters. |
