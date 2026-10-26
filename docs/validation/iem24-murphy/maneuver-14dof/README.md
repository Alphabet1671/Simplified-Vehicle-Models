# IEM24 Murphy 14DoF maneuver simulations

This directory contains six **simulation-only** open-loop IEM24 Murphy 14DoF maneuver results. No CAN, analog I/O, vendor API, actuator transport, hardware backend, or physical commissioning was enabled.

Each case was run through the compiled `models/formulations/iem24_murphy_real_14dof_simple.xml` vertical model, published as a hash-validated completed artifact, converted from accepted evidence to a RunRecord, and rendered as an MP4. Accepted evidence uses the animator's canonical channel convention: body accelerations, dynamic roll/pitch/heave, wheel pose/steer/loaded radius, tire Fx/Fy/Mz/Fz, slip, wheel torque, and powertrain channels are recorded from the same vertical evaluation used by the integrator. Consequently every MP4 exposes GG, track, rear, right, and top/load views. The display uses straight-reference world-pose evidence; it is not a simulated track path-following result.

## Provenance index

| Case | Completed duration | Artifact attempt / manifest SHA-256 | RunRecord / SHA-256 | MP4 / SHA-256 |
|---|---:|---|---|---|
| Step, 1.0 g requested at 60 km/h | 3.0 s | `step/983ad114-143b-419b-8836-9103abc27d19/result` / `b9b2edc72aad36152649605e04bf8e76785d132391c57b35ed8522d8a14b96ce` | `step/step.runrec` / `cd222221925c854a6f1f2cc10af7ad80e5384dea30089ae450a5a84002c049ec` | `step/step.mp4` / `47300a02ad8709065313cf6bfa915e2e1b2dccb31a1533d04f95a324f0463a92` |
| Ramp, 1.0 g requested at 60 km/h | 3.0 s | `ramp/66b2fda8-4229-4a13-88d2-dab1f86f1d06/result` / `3b056ac2028baba6ca594298980e70efff38694eed352c8fe37b64bbd8a82e58` | `ramp/ramp.runrec` / `6fa59a1fbae5d5919bdf4d68d85a6f281bada65ff979dbd32d260fb45a8379dc` | `ramp/ramp.mp4` / `03677baba2ec219f0aa34629d74c0aed7fdab4d9ed47b9637ab6bb8660a5359b` |
| Sine, 1.0 g requested at 60 km/h | 6.0 s | `sine/843c7274-15f1-4030-bc5b-b42ea0a12afb/result` / `cc796701fc91ddb7d8a97e771746e9e3dca595041023ad8218b03b6fa28a3293` | `sine/sine.runrec` / `16ad5cbb52051d071daaba1d0822606acd11c1ed58cff70b5d6221570d1374a2` | `sine/sine.mp4` / `871895d6640775c1b4c50da0dd4ebefc2d719e84b60a4895b488719171b3d910` |
| Locked-SWA acceleration, 50% throttle | 5.0 s | `acceleration/29bbda4b-a8fa-4d5f-888b-4c79ae8cd0e0/result` / `9e48612c4f96a7344f017f53ff9361112e4c8848519dec50280bea47e4b55e34` | `acceleration/acceleration.runrec` / `7c812c199a274af77a2f49a08c56398019c6f5c10e79d26d3e4c6a055b237607` | `acceleration/acceleration.mp4` / `045f47dac964220a137f2d9c07df339710c37d22b8d51a31626911814f83c8a9` |
| Locked-SWA braking, 0.5 brake travel | 1.2 s | `braking/e5427eee-34ef-43e9-b675-1941f7834f22/result` / `d3d4897f841ce14b2b6db9637cd9887edb7e6758adaa2281e6d27f5793f39d97` | `braking/braking.runrec` / `a143fb4a182859094012123cf95444dd189e3a03126af9208ebadcb2f9c3863d` | `braking/braking.mp4` / `a416ea72d528ef72be684121239212a979cdf05658fd259fc04d4832fc53a435` |
| CSV steering sweep, 10% throttle | 10.0 s | `frequency-sweep/a8afa586-bf2e-4ace-8d54-dcb4f78c3ce2/result` / `8825afa1a404497f5eb4cff9a82fe41129bbd157ec41b43daad9ecc874785305` | `frequency-sweep/frequency-sweep.runrec` / `35af7323e833b803c56e8f7bb16a00522a95bc017af0c3cb2c48d8233f96e32a` | `frequency-sweep/frequency-sweep.mp4` / `ead95f30c18736db68908684e55703d4b6c7ffab5ed05d5427229aa8e0fdf73b` |

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
