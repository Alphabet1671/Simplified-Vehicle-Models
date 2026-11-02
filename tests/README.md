# tests

The unit and regression suite, run with `unittest`.

## Files

| File | Contents |
|---|---|
| `test_catalunya_ttc_formulation.py` | Exact TTC artifact preservation, steady/transient parameter partition, and planar 7-DoF assembly selection. |
| `test_track_gui_lap_closure.py` | Synthetic stadium coverage for bounded periodic seam smoothing and skip/refusal conditions. |
| `test_forward_validation_cost.py` | Accepted-state and dense-midpoint validation coverage and timing-accounting regressions for the shared forward solver. |
| `test_forward_driver.py` | Continuous forward-driver dynamics, directional pedal slew, and speed planning. |
| `test_braking_envelope.py` | Analytic sampled braking-cap propagation, closed seams, sampling and oracle-query boundaries. |
| `test_envelope_driver.py` | Pure actual-speed/pedal-aware policy arithmetic and forward-solver diagnostics hook. |
| `test_forward_driver_runner.py` | Runner CLI, warm-start, provenance, and synthetic integration regression coverage. |
| `test_default_track.py` | Canonical default-track coverage for the runner, comparison tools, and plots. |
| `test_driver_maps.py` | Immutable pedal map interpolation, validation, shaping, strict JSON configuration, and stable digest.
| `test_pedal_actuator_calibration.py` | Runtime physical pedal calibration binding, forward evaluator parity, and bounds.
| `test_sim_gui_forward.py` | Current GUI forward-driver settings, immutable identity, child dispatch, RunRecord-backed completion gate, and workflow-specific Qt visibility. |
| `test_sim_gui_case.py` | GUI case adapter dispatch, point-mass boundaries, and retired workflow rejection. |

Test modules named `test_<topic>.py` each cover one topic and run under `unittest`. `test_gui_launchers.py` exercises direct venv launch from unrelated working directories. Maneuver result tests verify canonical RunRecord handoff to animator readers; video encoding is not a maneuver responsibility.

## Subdirectories

| Directory | Purpose |
|---|---|
| `fixtures/` | Data files that tests read. |
