# Task 6 report — calibration and evidence-only analysis

## Delivered

- Added `calibration.py` with an offline `calibrate_peak_ay` scan/refinement process. It performs eight uniformly spaced positive-magnitude trial amplitudes including bounds, then only refines a feasible monotonic bracket. The result freezes signed SWA amplitude, accepted peak `ay`, residual, full-definition fingerprint, and auditable trials. It explicitly returns `matched`, `unreachable`, `ambiguous`, or `model_limit`; it installs no live feedback.
- Added `analysis.py` with `analyze_maneuver`, which consumes only `SimulationResult` evidence and definition data. It reports execution, input-fidelity, objective, metrics, and interventions without importing/evaluating a vehicle plant. Invalid, short, or nonuniform evidence is masked instead of used for unsafe divisions/integrals.
- Circular-skidpad classification only brackets a physical limit after persistent path/radial error with adequate target-speed evidence and no saturation/model failure. Locked-SWA output is marked research-only.

## Verification

- `test_maneuver_calibration.py`: 4 passed, including canonical maneuver-v1 `steering.amplitude_mps2` magnitude extraction and `steering.direction` signed trial/final SWA selection.
- `test_maneuver_analysis.py`: 4 passed.
- `test_maneuver_*.py`: 56 passed.
- `test_planar_forward.py`: 9 passed.
- `git diff --check`: passed.
