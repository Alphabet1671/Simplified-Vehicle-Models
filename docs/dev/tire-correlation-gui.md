# TTC tire correlation GUI

The native TTC tire GUI is an **interactive correlation and inspection tool**, not an automatic validation or approval system. It evaluates the repository's existing UniTire simple and transient implementations; it does not change their physics. Fits, bounds, and local residuals are candidate evidence only. Review the selected data and diagnostics before applying any candidate.

## Install and launch

From the repository root, create a local environment and install the optional tire-GUI dependencies (PyQt6, SciPy, and Matplotlib):

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[tire-gui]'
PYTHONPATH=src:. python tools/tire_gui/app.py
```

To open an XML model on launch:

```bash
PYTHONPATH=src:. python tools/tire_gui/app.py models/tires/hoosier-43075-r20-simple.xml
```

Use **Open XML** to select another steady or transient tire XML. A transient XML contains only relaxation/trail coefficients: opening it attaches to the current steady parameters (preserving steady edits), or prompts you to choose a steady companion if none is open. Conventional sibling names are suggested, never silently paired. The Transient tab is selected automatically and imported coefficients stay provisional. To provide an explicit pair at startup:

```bash
python tools/tire_gui/app.py models/tires/iem24_murphy_real_unitire_transient.xml \
  --steady models/tires/iem24_murphy_real_unitire.xml
```

Direct script launches now bootstrap the checkout paths; `PYTHONPATH` is not required. Use **Import MATLAB seed** to explicitly import a supported tire parameter structure; it does not import a fit report or transient coefficients. The MAT seed import requires SciPy. The GUI can be tested without a display with `QT_QPA_PLATFORM=offscreen`.

## Local data and privacy

The default local dataset location for this checkout is `private/ttc/TTC/`; the requested MATLAB seed is `private/ttc/unitire_simple_fit.mat`. These are local inputs and are ignored by Git, not repository fixtures. Keep raw MAT files, measurements, plots, fit sessions, residuals, and optimizer logs local. The GUI's suggested output folder is `tire_fits_out/`, also ignored. Verify any alternate output location before saving or sharing. A saved bundle contains model XML and session metadata (including source path references), not measurement arrays. It may still contain fitted coefficient values: do not publish it without reviewing applicable TTC terms and export rights. No licensing permission is implied by the tool.

The initial seed options are deliberately distinct:

- **Repository lab XML** is the checked-in lab tire model.
- **Requested MATLAB fit** is an explicitly imported seed from `unitire_simple_fit.mat`; the file contains `tire` and `fitData`, but no `fitReport`, and it supplies no transient fit.
- Neither is the road-scaled Murphy seed. Do not import road-grip scaling into a lab fit by default.

The current proxy is **43075 R20 lateral / 43100 R20 longitudinal**. Run 72 is a **43100 R20, 7-inch-rim** dataset (including its combined-slip Fy); it is not a 43075 result. The lateral and longitudinal proxy mismatch must remain visible when interpreting plots and exported model use.

## Dataset interpretation and sample selection

The compatibility profile represents the source MATLAB selection: run 8 lateral and run 72 longitudinal/combined-slip, their explicit one-based inclusive segment indices, MATLAB-style half-up point selection (up to 180 points per segment), prescribed signs, nominal loads, and 40.193 km/h belt speed. This preserves the selection and objective contract, not identical MATLAB/SciPy optimizer trajectories. Fresh measured-channel use is a separate interpretation and must not be described as compatibility parity. Channel descriptors and units are used explicitly; missing/ambiguous units should be resolved rather than inferred from value magnitude. The TTC compression-negative FZ convention is converted to positive load, SA is negated into model alpha, V is converted to SI, and SL is preferred over SR when both occur.

Use whole segments/runs as holdouts. A held-out segment must be excluded from training, including combined-slip groups; changing holdouts, events, locks, or bounds invalidates the current fit generation. A lower residual on training data is not held-out validation. Review sample counts and dimensional RMSE alongside normalized objective values and per-axis residuals.

## Fit and manual-control workflow

Steady automatic fitting has two ordered stages:

1. Fit the 15 force coefficients against lateral Fy/Fz, longitudinal Fx/Fz, and the two combined-slip normalized force groups. The compatibility objective is the sum of the four group RMS values, with combined groups weighted 0.75.
2. Keep forces fixed and fit the trail/Mz coefficients. Its scale is load times tire radius with the specified lower floor.

The status panel reports convergence, dimensional RMSE, sample counts, bound saturation, and warnings. A finite best candidate may be retained when an optimizer stage does not converge, but it is not thereby accepted or automatically applied. There is intentionally no universal “good fit” threshold.

Parameter controls show the XML parameter, units, numeric value, bounds, fit lock, and slider. A slider edit updates the model prediction; the spinbox allows precise entry. Fit locks exclude parameters from an automatic fit. Bounds constrain search and are not confidence intervals or permission to accept nonphysical values. Undo/redo records edits; reset restores the selected group/seed state. Applying an optimizer candidate is explicit. A cancelled fit cannot update the live values; candidates from stale data or parameter revisions are refused. Editing steady coefficients invalidates transient-fit status because the transient response depends on the steady model.

Transient replay uses contiguous, unsmoothed traces and the actual elapsed-time samples; gaps are not stitched or interpolated across. Candidate steering changes are **not approvals**. Review each event's start/stop and pre-event baseline, command and response timing, load, speed, pressure, inclination, time gaps, and exclusion reasons; explicitly approve only suitable events. Run-7 attribution as transient is supported by source-script evidence, but candidate windows are not pre-approved events. The source metadata identifies FSAE TTC Round 9; ET is elapsed time, not calendar time. The acquisition calendar date is unavailable unless supported by the original run workbook/test log. Filesystem dates are not test dates.

Transient fitting is gated on excited, time-resolved events and adequate sensitivity. The workflow fits lateral length scale from Fy, then trail pole/lead parameters from Mz when the lateral fit is usable, and can fit longitudinal length scale from Fx only with independently approved longitudinal excitation. Reported rank, conditioning, time-resolution, bound hits, and holdouts must be reviewed. Unidentified parameters remain provisional; in particular, ordinary run-72 steady sweeps do not identify a longitudinal relaxation scale. Do not invent a run 73 or force-fit an unavailable response. The GUI does not add camber physics: zero inclination is the supported steady fit condition, and unsupported camber/overturning effects remain limitations.

## Saving model outputs

Use **Save fit as…** to create a separate bundle in the chosen output directory. It writes a compiler-validated `unitire-simple` XML and, when requested, a separate `unitire-transient-simple` XML, plus a session manifest with parameter provenance/status, steady fingerprint, selected IDs, locks/bounds, event windows and approvals, and private source path references. The steady and transient models are separate assembly roles (`tire` and `tire-transient`); do not nest transient parameters in the steady XML. Existing model files are not overwritten automatically; collisions and unsafe paths are refused. Reopen a bundle/session to inspect the saved state. Missing referenced source files must be treated as unavailable inputs, not reconstructed data.

Before export or redistribution, review the TTC data license and the rights covering any derived coefficients. The GUI does not establish export permission. Keep default exports and sessions private unless that review explicitly permits sharing.

## Developer checks

Run GUI-related tests in headless mode:

```bash
QT_QPA_PLATFORM=offscreen PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_tire_gui*.py' -v
```

Core model/runtime tests do not need Qt; GUI integration requires the `tire-gui` extra. Synthetic tests do not read local TTC inputs. GUI presence should be reported as a skip when its optional dependencies are not installed, not as a successful GUI run.
