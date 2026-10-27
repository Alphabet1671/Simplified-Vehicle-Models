# Simulation GUI

The simulation GUI prepares vehicle investigations using either the supported point-mass quick power-investigation workflow or the yaw-aware transient workflow. Solver selection and source model are separate: a selected full model can be adapted to the representation used by the chosen solver.

## Solver kinds and resolution

| Solver kind | Supported selected source | Resolved solution model | Yaw map |
|---|---|---|---|
| `point-mass` | `point-mass`, `planar-7dof`, `14dof-simple` | `point-mass` | Ignored; not required |
| `yaw-aware` | `planar-7dof`, `14dof-simple` | `planar-7dof` | Existing completed artifact required |

The mapping is exposed by `tools.sim_gui.settings.solution_model_for(settings, selected_model)`. It is a resolution contract only: it does not compile or physically adapt a model. Task 3's runner consumes it to perform the actual model adaptation. A direct point-mass source remains valid for the point-mass workflow. Yaw-aware rejects a point-mass source.

The point-mass workflow is a yaw-ignorant quick power investigation. Its point-mass-prefixed settings are serialized independently; yaw map, mode, and plan fields are not applicable. Until Task 3 refactors child dispatch, prepared point-mass cases continue to map to the current forward runner's supported full-model forward command. This Task 2 contract does not claim full-model point-mass compilation is implemented; `solution_model_for` is the explicit mapping that Task 3 must consume.

Yaw-aware supports `fixed`, `line`, and `replay`; replay requires a saved plan. It requires an existing completed, identity-checked vehicle-limit yaw-map artifact and never builds or substitutes one. Profile/line/replay budgets, profile/control spacing, controller, and maximum integration step are serialized in the study definition. Output and replay validation are sampled/discrete evidence, not continuous feasibility or containment certification.

Settings JSON rejects unknown fields, including legacy `planner`, unprefixed point-mass settings such as `lateral_acceleration`, and unsupported standing-start requests; it does not silently alias old fields.

## Running

Install PyQt6 and launch from the repository root:

```bash
PYTHONPATH=src:. python3 tools/sim_gui/app.py
```

Choose the solver kind first, then provide fields relevant to that workflow. The yaw-map file picker accepts completed `.npz` artifacts; the replay-plan picker is visible only for replay mode.
