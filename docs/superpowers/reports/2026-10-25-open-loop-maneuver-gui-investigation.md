# Open-loop maneuver GUI investigation report

Date: 2026-10-25
Branch: `feature/open-loop-maneuver-gui`
Worktree: `.worktrees/open-loop-maneuver-gui`
Baseline commit: `eb37c44`

## Deliverables

- [Investigation and proposed design](../specs/2026-10-25-open-loop-maneuver-gui.md): maneuver/control matrix, direct robot channel contract, CSV semantics, peak-ay interpretation, Cartesian simulation architecture, GUI layout, physical commissioning gates.
- [Implementation plan](../plans/2026-10-25-open-loop-maneuver-gui.md): eleven test/implementation/review units, proposed file ownership and interfaces, commands, acceptance mapping and execution handoff.

No application/library implementation was changed. No physical robot was connected, configured, armed or commanded. No GUI/browser visual test was performed.

## Principal findings

1. The target is the repository's existing PyQt6 simulation application. A Maneuvers tab can share model configuration and process lifecycle without requiring a track.
2. Existing track-following runner is not an open-loop input engine; wrapping it unchanged would introduce hidden feedback.
3. Existing plant command is throttle fraction, brake force and road-wheel steering rate. Required robot command is SWA plus two calibrated pedal channels. Explicit geometry and travel/force conversion are essential.
4. Peak ay needs a stated meaning. Proposed default is pre-run exact-waveform amplitude calibration followed by a frozen SWA history, not live ay tracking.
5. Frenet/path progress constraints are inappropriate for unrestricted limit maneuvers. A Cartesian planar adapter reusing existing physics is the proposed first backend.
6. Physical operation needs its own backend, watchdog, telemetry and approved stop behavior. A fake backend and software state machine do not qualify an actual robot for vehicle testing.

## Baseline checks actually performed

Environment: Python 3.13.13; NumPy, SciPy and PyQt6 import specifications found. Dependency version floors are from the repository packaging metadata, not inferred from this availability check.

| Check | Outcome |
|---|---|
| `PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_sim_gui_core.py' -q` | PASS: 36 tests, 0.221 s |
| `PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_planar_forward.py' -q` | PASS: 9 tests, 0.141 s |
| `QT_QPA_PLATFORM=offscreen PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_sim_gui_qt.py' -q` | INCOMPLETE: did not complete after over ten minutes; job explicitly cancelled with SIGTERM. Output included font-alias and propagateSizeHints platform warnings. These do not establish the cause of the hang. |
| Full repository suite | Not run for this documentation-only investigation |
| Proposed maneuver/robot tests | Not run: planned modules do not yet exist |

The initial Qt command had a planar test chained after it; the chain did not reach that test before cancellation. The planar baseline above was then run independently. No hanging job was intentionally left running. A future executor should investigate the Qt test timeout with bounded per-test logging before treating the GUI baseline as green.

## Task 6 fix round — calibration fingerprint and skidpad evidence

- Changed maneuver fingerprint normalization to exclude UI metadata and recognized output-location keys only; a semantic `model.output` payload remains part of the hash.
- Added direct fingerprint tests proving UI/output-path changes preserve the digest while changed model output torque changes it.
- Skidpad limit statuses now require a configured positive target speed and at least three recent speed samples, all meeting the 98% target threshold. Missing/short evidence yields `not_established` plus a target-speed-evidence intervention.
- Persistent path error requires a trailing three-sample run. A physical bracket is reported only when a valid sample precedes the persistent invalid run; first-sample-invalid cases have no fabricated bracket. Unmet target speed remains an intervention and prevents classification.
- Regression coverage includes missing target speed, short speed evidence, target speed unmet, first-sample persistent failure, and the ordinary clean-sweep/persistent-bracket cases.
- Verification: calibration tests 5 PASS; analysis tests 8 PASS; all maneuver tests 62 PASS. Evidence-only analysis unchanged; no plant evaluation introduced.

## Review and open decisions

Plan self-review mapped every requested maneuver and channel mode to delivery tasks; scanned for placeholder markers; checked SWA/pedal and PID ownership naming; corrected the schema field count. Proposed decisions awaiting user confirmation are peak-ay calibration versus tracking, canonical CSV channel units, skidpad radius-sweep default, and simulation-first/hardware-gated release scope.

Physical commissioning additionally requires robot identity/API, command and feedback modes/calibration, independent safety and stopping architecture, sensor timing/accuracy, and approved operating thresholds. These unknowns are explicitly gated rather than assigned invented defaults.

## Task 5 implementation report — low-speed segment-start stop fix

- Changed `simulate_maneuver` to check the current longitudinal speed against its computed stop floor before invoking the integrator. For a valid state strictly above the plant's actual minimum and at/below the floor, it records accepted rate, observation, command, and optional accepted outputs at the current state, adds a `zero_speed` event with threshold and actual speed, and returns `stopped` / `zero_speed` without calling `solve_ivp`.
- A state at or below the plant's actual minimum is deliberately excluded from this early-stop path, preserving model-domain / `model_limit` semantics.
- Added regression coverage using a domain-rejecting SpeedPlant at 0.1005 m/s (minimum 0.1 m/s; floor 0.101 m/s), asserting stop outcome/reason, event speeds and accepted evidence.
- Verification: maneuver simulation 9 tests PASS; all maneuver tests 49 PASS; planar tests 77 PASS; forward tests 130 PASS. `git diff --check` and commit status are recorded in the Task 5 handoff.
