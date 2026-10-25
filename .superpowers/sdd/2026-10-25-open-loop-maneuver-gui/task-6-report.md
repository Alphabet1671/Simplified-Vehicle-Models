# Task 6 report — calibration and evidence-only analysis

## Delivered

- Added `calibration.py` with an offline `calibrate_peak_ay` scan/refinement process. It performs eight uniformly spaced positive-magnitude trial amplitudes including bounds, then only refines a feasible monotonic bracket. The result freezes signed SWA amplitude, accepted peak `ay`, residual, full-definition fingerprint, and auditable trials. It explicitly returns `matched`, `unreachable`, `ambiguous`, or `model_limit`; it installs no live feedback.
- Added `analysis.py` with `analyze_maneuver`, which consumes only `SimulationResult` evidence and definition data. It reports execution, input-fidelity, objective, metrics, and interventions without importing/evaluating a vehicle plant. Invalid, short, or nonuniform evidence is masked instead of used for unsafe divisions/integrals.
- Circular-skidpad classification only brackets a physical limit after persistent path/radial error with adequate target-speed evidence and no saturation/model failure. Locked-SWA output is marked research-only.

## Verification

- `test_maneuver_calibration.py`: 4 passed, including canonical maneuver-v1 `steering.amplitude_mps2` magnitude extraction and `steering.direction` signed trial/final SWA selection.
- `test_maneuver_analysis.py`: 5 passed, including clean radius-controlled sweep `limit_not_reached` versus transient invalid evidence.
- `test_maneuver_*.py`: 56 passed.
- `test_planar_forward.py`: 9 passed.
- `git diff --check`: passed.

## Fix round — review findings

- Fingerprinting now omits UI and recognized `output_path`/`output_directory` location metadata, but preserves semantic `output` fields at any depth. Regression asserts location/UI changes keep the digest stable while changing `model.output.torque_nm` changes it.
- Skidpad `bracketed_physical_limit` and `limit_not_reached` require a configured positive target speed and at least three final speed samples all at or above 98% of target. Missing or inadequate evidence yields `not_established` and an intervention; unmet target speed also blocks both classifications.
- Persistent invalidity requires a final run of three bad path-error samples. A bracket requires a valid sample before that run; first-sample invalid evidence reports `not_established` with both bracket speeds unset.
- Added tests for metadata-vs-semantic fingerprint behavior, missing/short/unmet speed evidence, and first-sample persistent invalidity.
- Fix-round verification: calibration 5 passed; analysis 8 passed; all maneuver tests 62 passed; `test_planar_forward.py` 9 passed; diff check passed. No plant evaluation added.
