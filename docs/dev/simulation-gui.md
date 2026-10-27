# Simulation GUI

The simulation GUI prepares isolated vehicle-model cases, runs single cases or parameter studies, and records results for later review. It does not edit model parameters. Solver selection is separate from selected source model.

## Install

Install the GUI extra from the repository root:

```bash
python -m pip install -e '.[gui]'
```

Alternatively, install PyQt6 directly:

```bash
python -m pip install "PyQt6>=6.5"
```

## Start and load a model

Start from the repository root, optionally opening a model immediately:

```bash
PYTHONPATH=src:. python3 tools/sim_gui/app.py
PYTHONPATH=src:. python3 tools/sim_gui/app.py models/vehicles/iem24_murphy_real_planar.xml
```

Choose **Open Vehicle Model…** to select a vehicle formulation or assembly XML. Vehicle and track dropdowns list files opened in the current session. Track selections stay synchronized between the Track and Single Run tabs; the Track tab previews the centerline. Component candidates are discovered under `models/`, filtered for slot compatibility, and shown with read-only parameter values.

## Solver kinds and source resolution

| Solver kind | Supported selected source | Resolved solution model | Yaw map |
|---|---|---|---|
| `point-mass` | `point-mass`, `planar-7dof`, `14dof-simple` | `point-mass` | Ignored; not required |
| `yaw-aware` | `planar-7dof`, `14dof-simple` | `planar-7dof` | Existing completed artifact required |

The mapping is exposed by `tools.sim_gui.settings.solution_model_for(settings, selected_model)`. It is a resolution contract only: it does not compile or physically adapt a model. Point-mass remains a yaw-ignorant quick power-investigation workflow, and a direct point-mass source is accepted. Its settings and model mapping are selectable and serializable, but physical execution is preflight-blocked with `point-mass adaptation unavailable until yaw GUI runner migration` until Task 3 implements adaptation. No point-mass case is sent to the legacy full-model runner.

Yaw-aware fixed/line/replay supports only selected 7DoF and 14DoF models; 14DoF resolves explicitly to planar 7DoF. It requires an existing completed, identity-checked vehicle-limit yaw-map artifact and never builds or substitutes one. Replay requires a saved plan. Output and replay validation are sampled/discrete evidence, not continuous feasibility or containment certification.

RunSettings JSON rejects unknown legacy fields and validates types, enums, numeric finiteness, and container shapes during deserialization. Point-mass-prefixed settings are separate from yaw-aware map/mode/plan settings.

## Swap a component

1. Open a vehicle model and select a slot in the **Model** panel.
2. Select a compatible candidate or choose **Browse…** for an XML file.
3. Review the read-only parameter table; choose **Reset** to restore the baseline.

A candidate must match the slot capability, or model type/subtype when the slot has no capability. Replacing a component refreshes nested slots; incompatible candidates are rejected.

## Parameter sweeps

The GUI does not edit parameter values. Create sweep members using `tools.model_data.generate_component_sweep`; use a temporary folder while checking a command:

```bash
PYTHONPATH=src:. python3 -m tools.model_data.generate_component_sweep \
    --base models/suspensions/iem24_murphy_real_suspension.xml \
    --set parameter:arb_wheel_rate_rear --offsets -100 0 100 \
    --name rch-example --output-dir /tmp/rch-example
```

The generator writes `sweep.json` and generated XML (or NPZ data for data sweeps), refuses an existing output folder unless `--force` is specified, and defaults to `models/sweeps/<name>`. To add a sweep, open the model, use **Study → Add sweep folder…**, choose the sweep dimension and study mode (**One-at-a-time** or **Full factorial**), then build cases. Sweep members must fit exactly one vehicle slot; use one sweep dimension per slot.

## Single runs, studies, and output

For a single run, choose a vehicle and track in **Single Run**, select an **Export folder**, then choose **Go** or press `F5`. Study cases are built from the **Study** tab and executed through the Queue; case outputs are isolated under the study output directory. The default export/output root is `runs/`. Logs, case metadata, attempt summaries, and run records are persisted there.

Headless studies can be run from a saved definition:

```bash
PYTHONPATH=src:. python3 -m tools.sim_gui.run_study \
    --definition path/to/definition.study.json --output-root runs --parallel 1
```

Inspect options with `PYTHONPATH=src:. python3 -m tools.sim_gui.run_study --help`. The GUI currently has no warm-start chain, multi-segment run mode, or post-processing graphs/reports/animations. Export an animation from a run record with `python3 tools/anim/export_animation.py --record <path>`; see [Animation export](animation-export.md).
