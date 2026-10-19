# Simulation GUI

The simulation GUI builds and runs forward-driver studies. It selects registered vehicle components. It writes one isolated model for each case. It does not edit model parameters.

## Install

Install the GUI extra from the repository root:

```bash
python -m pip install -e '.[gui]'
```

You can install PyQt6 instead:

```bash
python -m pip install "PyQt6>=6.5"
```

## Start the GUI

Start the application from the repository root:

```bash
PYTHONPATH=src:. python3 tools/sim_gui/app.py
```

You can open a model when you start the application:

```bash
PYTHONPATH=src:. python3 tools/sim_gui/app.py models/vehicles/iem24_murphy_real_planar.xml
```

## Load a vehicle model

1. Start the GUI.
2. Select **Open Vehicle Model…**.
3. Select a vehicle formulation or vehicle assembly XML file.
4. Select a track in the **Model** panel.

The GUI scans registered XML files below `models/`. It lists only components that fit the selected slot. It shows the selected component parameters as read-only values.

The forward runner supports `planar-7dof` and `14dof-simple`. The point-mass model has no forward runner in this GUI.

## Swap a component

1. Open a vehicle model.
2. Select a slot in the **Model** panel.
3. Select a candidate from the slot list, or select **Browse…** and choose an XML file.
4. Check the parameter table.
5. Select **Reset** to restore the baseline component.

A candidate must have the same capability as the slot. If the slot has no capability, the candidate must have the same model type and subtype. A component change also refreshes its nested component slots. The GUI rejects an incompatible candidate.

## Make a parameter sweep

The GUI does not edit parameters. Use `generate_component_sweep.py` to create one component file for each value.

The generator accepts a leaf component XML file. It accepts one target in the form `parameter:<name>` or `data:<role>:<key>`. It accepts one of `--values`, `--offsets`, or `--scales`. A sweep has 1 to 100 numbers. A name must contain 1 to 40 ASCII letters, numbers, `_`, or `-` characters.

Use a temporary output folder while you check a command:

```bash
PYTHONPATH=src:. python3 -m tools.model_data.generate_component_sweep \
    --base models/suspensions/iem24_murphy_real_suspension.xml \
    --set parameter:arb_wheel_rate_rear \
    --offsets -100 0 100 \
    --name rch-example \
    --output-dir /tmp/rch-example
```

This example uses `arb_wheel_rate_rear`, a real numeric parameter in the real suspension component. RCH is an example of the same procedure. Replace the target with the real parameter that you need to study. The generator writes `sweep.json` and generated XML files. It writes generated NPZ files for a data target. It refuses an existing output folder unless you add `--force`. The default output folder is `models/sweeps/<name>`.

Add the sweep to a study:

1. Open the vehicle model.
2. Open the **Study** tab.
3. Select **Add sweep folder…**.
4. Select the folder that contains `sweep.json`.
5. Select the sweep dimension.
6. Select **One-at-a-time** or **Full factorial**.
7. Select **Build Cases**.

The sweep members must fit exactly one vehicle slot. Use one sweep dimension per slot. Do not add two sweep targets to one component.

## Run a study

The GUI supports two modes:

- **One-at-a-time** runs one baseline case and then one case for each changed member.
- **Full factorial** runs every combination of dimension members.

The GUI warns when a study has more than 50 cases. It refuses a study above 500 cases. The study name must contain 1 to 40 ASCII letters, numbers, `_`, or `-` characters.

Select **Run** or press `F5`. Select **Stop** to cancel pending cases and terminate active cases. The GUI validates each generated formulation before it starts the case. It writes the case status as `pending`, `invalid`, `running`, `completed`, `rejected`, `timed_out`, `cancelled`, or `failed`.

A planar run has a wall limit of 300 s. The GUI passes the timeout to the runner. The runner rejects a planar timeout above 300 s. The `14dof-simple` model can use a longer user-selected timeout.

## Output layout

The default output root is `runs/`. Each study uses a directory named `<study-name>_<YYYYMMDD-HHMMSS>`. The GUI creates these paths:

```text
runs/<study>/
├── definition.study.json
├── study.json
└── cases/
    └── case_000/
        ├── case.json
        ├── model/
        │   ├── formulation.xml
        │   ├── vehicle.xml
        │   └── nested component XML files when needed
        └── attempt_<timestamp>/
            ├── parent.json
            ├── run.log
            ├── summary.json
            ├── run_record.npz       # when the child writes a record
            └── source_snapshot/     # files used by the child
```

The study index updates each case with its `directory`, `status`, `error`, `attempt`, `run_record`, and `summary` fields when those fields are available. The child runner writes `summary.json` and may write `run_record.npz`.

### `study.json` schema

`StudyIndex` writes an example payload with these top-level keys:

```json
{
  "schema": "sim-study-v1",
  "name": "example",
  "created_utc": "2026-10-18T00:00:00+00:00",
  "repository_commit": "<git commit or null>",
  "definition": "definition.study.json",
  "cases": [
    {
      "id": "case_000",
      "label": "baseline",
      "model": "planar-7dof",
      "track": "models/tracks/2024_michigan_endurance_multilap_closed.xml",
      "choices": {},
      "sweep_points": {},
      "status": "pending"
    }
  ]
}
```

This JSON is an example. `created_utc` and `repository_commit` come from the current run. `cases` contains the expanded `CaseSpec` keys and the fields that the GUI adds while it prepares and runs cases. `definition.study.json` stores the definition. Its keys are `name`, `source`, `track`, `model`, `choices`, `settings`, `mode`, and `dimensions`. Each dimension stores `key`, `values`, `labels`, `sweep`, and `points`.

## Headless runner

Run a saved definition without Qt:

```bash
PYTHONPATH=src:. python3 -m tools.sim_gui.run_study \
    --definition /path/to/definition.study.json \
    --output-root runs \
    --parallel 1
```

`--definition` is required. `--output-root` defaults to `runs`. `--parallel` defaults to `1` and must be at least `1`. The command exits with status 0 only when at least one case is valid and every valid case is `completed`. It exits with status 1 otherwise.

Inspect the available flags with:

```bash
PYTHONPATH=src:. python3 -m tools.sim_gui.run_study --help
```

## Limits

- Use the generator for parameter changes. The GUI does not edit parameters.
- Add one sweep dimension per slot.
- The GUI has no warm-start chain.
- The GUI has no multi-segment run mode.
- The GUI has no point-mass runner.
- Planar runs have a 300 s wall limit.

## Hand-off

The post-processing GUI reads `study.json` and each `run_record.npz`. The current GUI does not provide post-processing graphs, reports, or animations. Export an animation from a record with:

```bash
python3 tools/anim/export_animation.py --record <path>
```

See [Animation export](animation-export.md) for video options.
