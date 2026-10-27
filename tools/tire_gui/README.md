# tire_gui

Tire correlation GUI.

## Launch from the repository venv

```bash
.venv/bin/python tools/tire_gui/app.py models/tires/hoosier-43075-r20-simple.xml
```

Install or refresh the activated environment with `python -m pip install -e '.[tire-gui]'`; then `tire-gui` is also available directly in the venv. Both entrypoints work without `PYTHONPATH`. From another working directory, use an absolute script path or the installed command; model and data paths supplied by the user resolve normally.

Licensed TTC inputs belong in ignored `private/ttc/`; fit/session exports default to ignored `tire_fits_out/`. Automatic results are candidates, not validation, and require explicit application. The model deliberately combines 43075 R20 lateral data with a 43100 R20 longitudinal proxy. Keep unidentified transient coefficients provisional and review TTC licensing before sharing derived outputs.

## Opening transient tire XML

The **Open XML** action accepts both `unitire-simple` and `unitire-transient-simple` models. A transient XML contains only relaxation/trail coefficients, so it needs a steady force-law companion:

- If a steady model is already open, opening a transient XML attaches its coefficients to the current steady parameters, preserving manual steady edits.
- If no model is open, the GUI asks you to select the steady XML. Conventional matching sibling filenames are suggested, never silently chosen.
- For an explicit pair at startup, use:

```bash
tire-gui models/tires/iem24_murphy_real_unitire_transient.xml \
  --steady models/tires/iem24_murphy_real_unitire.xml
```

The Transient tab is selected automatically. Imported XML coefficients remain labeled provisional unless validated fit evidence is restored through a session; opening a file is not a fitting or validation claim.

## Files

| File | Contents |
|---|---|
| `__init__.py` | TTC tire correlation data and GUI support. |
| `app.py` | Launch the native TTC tire correlation GUI. |
| `contracts.py` | Python module `contracts`. |
| `data.py` | Python module `data`. |
| `events.py` | Candidate detection and physical screening for transient tire events. |
| `fit.py` | Two-stage MATLAB-compatible steady tire fitting. |
| `model.py` | Immutable repository-compiled tire model snapshots and evaluator boundary. |
| `parameters.py` | Public canonical mapping for flattened UniTire XML parameters. |
| `save.py` | Validated private export bundles for the tire correlation model. |
| `segments.py` | Python module `segments`. |
| `transient.py` | Time-domain replay and gated relaxation parameter identification. |

## Subdirectories

| Directory | Purpose |
|---|---|
| `qt/` | Qt layer of the tire correlation GUI. |
