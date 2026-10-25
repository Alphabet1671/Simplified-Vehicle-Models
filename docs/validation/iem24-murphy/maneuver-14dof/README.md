# IEM24 Murphy 14DoF maneuver simulations

This directory contains six **simulation-only** open-loop IEM24 Murphy 14DoF maneuver results. No CAN, analog I/O, vendor API, actuator transport, hardware backend, or physical commissioning was enabled.

Each case was run through the compiled `models/formulations/iem24_murphy_real_14dof_simple.xml` vertical model, published as a hash-validated completed artifact, converted from accepted evidence to a RunRecord, and rendered as an MP4. The display uses straight-reference world-pose evidence; it is not a simulated track path-following result.

## Provenance index

| Case | Completed duration | Artifact attempt / manifest SHA-256 | RunRecord / SHA-256 | MP4 / SHA-256 |
|---|---:|---|---|---|
| Step, 1.0 g requested at 60 km/h | 3.0 s | `step/95592495-de61-49e7-a3f8-64dbff489896/result` / `7fa771ccbc832d7e0c9a500e42f4e997b9c061b71a69b3fe901eb29982129c7c` | `step/step.runrec` / `fe00c1b9d28b4a87d2218ed037ce4ace20581d2b72fdee807f391ecee42e03af` | `step/step.mp4` / `a4087536ad9c71cde44da6700696d3f86f98413fa7a15fc1e4272e76d857cbfe` |
| Ramp, 1.0 g requested at 60 km/h | 3.0 s | `ramp/c47a7e0-8ebc-403e-b2c6-c0a808742ce5/result` / `af1b8195beac8b2fa974854e597b57d3ade8dc55b9187e01518324486673ae52` | `ramp/ramp.runrec` / `ffcf2af22ccb09c72bcc3599847fd511a3fbb1dac222939d8b1c7dad6d159030` | `ramp/ramp.mp4` / `0d2ceabfc23e810d77b36d00f3265c744d84ef554cd8462fb98c45f9b6e88780` |
| Sine, 1.0 g requested at 60 km/h | 6.0 s | `sine/439a578a-5687-4bf3-a861-9417f0289d33/result` / `c904530f1dfa96f70c487fbdb1653d5a1d564a2c6fabd2e03f25296db722cb1b` | `sine/sine.runrec` / `b509e8d97b024546068ce1a695f04719554fcaf96d18893f47816b4acdb20833` | `sine/sine.mp4` / `35ec69afe5bd65c1367513ebb06b5244285060ef6b0ababf344a82382a3c572a` |
| Locked-SWA acceleration, 50% throttle | 5.0 s | `acceleration/7066f5f5-106d-4b52-8d63-3e1c8fdf89db/result` / `918525a4da87fd8be0d6b16cd160e04acca7bd2667c4e4f3b92764c5207d8d7c` | `acceleration/acceleration.runrec` / `44e8780b8ac3b73b1141d9d992109bf3b8ba62d3b2461613ebe6816d354ec644` | `acceleration/acceleration.mp4` / `dbc8dc9cbfac21115975a9bb4f1a051f5f7f3903e0d8931d6392a48e771c5080` |
| Locked-SWA braking, 0.5 brake travel | 1.2 s | `braking/42b87554-92b2-40d7-af3f-3756a017cab6/result` / `3f5b6d889a133827acc674e0b65e18bb06307f9b465d1feafc691e7943200278` | `braking/braking.runrec` / `fa94c06dd56395baba22e5eed710e2e2564be0667e2f8e080b9851f1682277b1` | `braking/braking.mp4` / `0790e436bee62fa4c64c4e6b13cdc24b895f61a04af4654c1a0c24be121737e2` |
| CSV steering sweep, 10% throttle | 10.0 s | `frequency-sweep/8e580421-41e9-4070-abf9-83a765cfd6f9/result` / `fe9ad073ca9e7ec8ade31ab45bf4617a8a14b1d7308ac558949316822d811923` | `frequency-sweep/frequency-sweep.runrec` / `ec3443e5d247e3a6a5271777f9b2e4dbf99e0369c420d0b8e32fda1d301e1293` | `frequency-sweep/frequency-sweep.mp4` / `c3d6f3dd4d4835790317512341b776b5dd74da2166608e134e407c42bbd13071` |

All accepted state arrays have 38 columns and monotone sample time. The artifact loader verified their manifests and payload hashes before animation. Accelerator and brake never overlap.

## Input fidelity

The immutable source inputs are in `tools/maneuver_gui/examples/iem24_14dof/`. Their SHA-256 values are: step `17847c049f109b99e1aff62c1fb090cd2411984ed09259db3ebfe966e44bf877`; ramp `43ae0717b72cf27605d1f2dc968aee850d65aff237153d4d52994992293f0dd8`; sine `34465d140dd3cea74d3397a9e0448224d2538af262b7a82ab813c09ff47e9080`; acceleration `b68241c70d0191db3fb86a5ac51085ccb29d8dc6252e6e072d188dad22e9e4c9`; braking `3065e3c8c062ed96f4c1f9c8c4bf9c1aa501e9eb38e78b6edcff09f29cb5debe`; and CSV definition `6497ffdd9494a0b020ae3415ac37cc0560aaf990d9b2bbe264281890f101c5b8`.

- Step, ramp, and sine request 16.6666666667 m/s cruise and 9.80665 m/s² peak ay. Before each final maneuver, simulation-only calibration trials selected and froze SWA **0.07491760253906252 rad**; the retained accepted trial response was 9.610520355712351 m/s² (residual -0.19612964428764812 m/s², within the calibrator tolerance). The definitions constrain calibration to 0.12 rad because higher scan values reached an ambiguous response branch.
- The step and ramp runs are 3.0 s envelope-bounded response windows. Their original longer profiles reached a model error after the usable response; no partial trajectory was rendered. The sine completed its requested 6.0 s integral-cycle profile.
- Acceleration holds normalized accelerator travel at 0.5 for 5.0 s.
- Braking holds normalized brake travel at 0.5 and accelerator at zero. The user-provisional `60 kgf → 0.5 travel` mapping remains provisional. Its duration was bounded to 1.2 s because the original 5.0 s request crossed the vertical model's low-speed validity boundary at 1.39 s.
- The 10 s CSV uses 0.1 accelerator and zero brake on every source row, with 0.1–10 Hz logarithmic steering frequency evolution at 5 s/dec. Source SHA-256: `0a78a5a9214809dfdf065be7f7eef412cf26598b7855582be0644473fe534301`.

## Limitations and safety disclosure

These videos are neither measured-vehicle validation nor commissioning approval. The peak-ay calibration is a frozen result of simulation trials only; it is not physical actuator calibration. The display-only acceleration track metadata does not make the straight-reference vertical simulation a path-following simulation. Physical actuator output remains disabled.
