# tire_gui

Tire correlation GUI.

## Launch from the repository venv

```bash
.venv/bin/python tools/tire_gui/app.py models/tires/hoosier-43075-r20-simple.xml
```

Install or refresh the activated environment with `python -m pip install -e '.[tire-gui]'`; then `tire-gui` is also available directly in the venv. Both entrypoints work without `PYTHONPATH`. From another working directory, use an absolute script path or the installed command; model and data paths supplied by the user resolve normally.

Licensed TTC inputs belong in ignored `private/ttc/`; fit/session exports default to ignored `tire_fits_out/`. Automatic results are candidates, not validation, and require explicit application. The model deliberately combines 43075 R20 lateral data with a 43100 R20 longitudinal proxy. Keep unidentified transient coefficients provisional and review TTC licensing before sharing derived outputs.

## Controls and curves

The left sidebar separates **Coefficients** from **Data & fitting**. Coefficients has the full sidebar height, with only the Mu, Stiffness factors, Shape factors, and Misc parameter tabs (each scrolls independently). Run, condition, pressure, speed-source, segment and event controls live on Data & fitting; they no longer extend the coefficient page. Steady plots use a 2×2 grid. The transient panel shows only **lateral force Fy**, comparing isolated zero-IA step-steer events (steady SA before and after a single outward transition). Continuous sine/triangular sweep legs and longitudinal sweep segments are not included. Nonzero inclination is excluded across the entire event, with ±0.25° allowed only as nominal-zero sensor tolerance. Compatible step conditions overlay the different loads. The condition-page selector exposes every group without allocating an excessively tall canvas. Equivalent input ramps overlay the different loads; dotted SA traces show actuator alignment. Force and model traces use the same measured-input midpoint time shift, never a shift fitted to the force response. Longitudinal excitation also shows slip (%) beside SA.

Transient model curves are asynchronous time-domain integrations of the original sampled SA, slip, load, wheel speed and resolved belt speed, including measured prehistory. They are not steady-state sweeps or idealized replacement ramps. Display does not imply approval for fitting; physical eligibility and explicit event approval still apply.

Belt speed uses the selected run's measured encoder V in SI units, not the near-zero wheel N channel. The speed-source control also supports wheel/effective-radius/slip reconstruction when those signals are usable, and an explicit verified m/s override. Original encoder values remain available when switching sources; source and warnings are retained in run metadata. Auto recovery is conservative: stationary samples in a moving measured trace are preserved, and unusable wheel signals never justify fabricating a belt speed. Run 7 includes genuinely near-stationary phases; runs 8/9 contain moving steer sweeps, with run 9 supplying multiple belt-speed conditions. Isolated step events retain their measured speed, including near-stationary phases; no rolling speed is invented to make them look like moving-belt tests. Clock resets and acquisition gaps split independent windows; they are never interpolated across. Detection uses a small command deadband to reject encoder noise. Simulation initializes from the pre-ramp measured condition, and the isolated event includes the measured settled hold and post-step response. Replay remains a comparison, not proof that relaxation coefficients have been identified. Wheel recovery assumes kappa = (R omega − V)/V and requires a physically valid effective rolling radius, not an assumed loaded radius or the other tire's radius.

Each seed, current-fit, and pending-candidate curve is a separate model sweep for one compatibility segment and nominal load, with the secondary slip held fixed. Combined-slip curves use each model's predicted Fy/Fz and Fx/Fz in slip-ratio order. Measured samples and residuals are points at the original experimental inputs; fitting still uses those original inputs, not the display grids. The nominal-load selector filters both fitting and displayed conditions.

Transient comparisons show **measured (solid)** and **current model (dashed)** only, colored by Fz, with one representative dotted SA command per plot. A pressure-set selector defaults to the approximately 83 kPa set so different pressure tests do not create duplicate-looking plots; all pressure sets remain available. The widened sidebar gives sliders a full-width row (minimum 480 px). The longitudinal X/Fx panel is removed; the lateral response uses the original SA command.

## Opening transient tire XML

The **Open XML** action accepts both `unitire-simple` and self-contained `unitire-transient-simple` models. A transient XML includes the full steady force law and four relaxation/trail coefficients. Opening it always replaces both steady and transient values in the current GUI model; the GUI does not look for a sibling or prompt for a companion. The transient coefficients remain provisional unless fit evidence is present.

New session exports contain exactly one XML plus `session.json`: a complete transient XML when transient values are included, or a steady-only XML otherwise. The loader also recognizes exact v1 paired bundles and migrates them in memory; new writes use v2.

The Transient tab is selected automatically. Imported XML coefficients remain labeled provisional unless validated fit evidence is restored through a session; opening a file is not a fitting or validation claim.

## Files

| File | Contents |
|---|---|
| `__init__.py` | TTC tire correlation data and GUI support. |
| `app.py` | Launch the native TTC tire correlation GUI. |
| `contracts.py` | Python module `contracts`. |
| `data.py` | Descriptor-driven TTC channel normalization, including effective radius. |
| `belt_speed.py` | Provenance-preserving encoder, wheel/slip and verified-override speed resolution. |
| `transient_plotting.py` | Isolated zero-IA step-steer selection, general ramp helpers, condition grouping and actuator-based time alignment. |
| `events.py` | Candidate detection and physical screening for transient tire events. |
| `fit.py` | Two-stage MATLAB-compatible steady tire fitting. |
| `model.py` | Immutable repository-compiled tire model snapshots and evaluator boundary. |
| `parameters.py` | Public canonical mapping for flattened UniTire XML parameters. |
| `plotting.py` | Independent fixed-condition model sweeps for display, separate from fitting samples. |
| `save.py` | Validated private export bundles for the tire correlation model. |
| `segments.py` | Python module `segments`. |
| `transient.py` | Time-domain replay and gated relaxation parameter identification. |

## Subdirectories

| Directory | Purpose |
|---|---|
| `qt/` | Qt layer of the tire correlation GUI. |
