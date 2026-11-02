# tests

The unit and regression suite, run with `unittest`.

## Files

| File | Contents |
|---|---|
| `test_catalunya_ttc_formulation.py` | Exact TTC artifact preservation, steady/transient parameter partition, and planar 7-DoF assembly selection. |
| `test_forward_driver.py` | Continuous forward-driver dynamics, directional pedal slew, and speed planning. |
| `test_braking_envelope.py` | Analytic sampled braking-cap propagation, closed seams, sampling and oracle-query boundaries. |
| `test_envelope_driver.py` | Pure actual-speed/pedal-aware policy arithmetic and forward-solver diagnostics hook. |
| `test_forward_command_mapper.py` | Forward command mapping, pedal realization, and integration callback contract. |
| `test_forward_driver_runner.py` | Runner CLI, warm-start, provenance, and synthetic integration regression coverage. |
| `test_driver_maps.py` | Immutable pedal map interpolation, validation, shaping, strict JSON configuration, and stable digest.
| `test_pedal_actuator_calibration.py` | Runtime physical pedal calibration binding, forward evaluator parity, and bounds.
| `test_continuous_driver_throttle_comparison.py` | Matched throttle-runner plan slicing, duration-weighted metrics, manifest/output contracts and arm isolation. |
| `test_linear_driver_input_study.py` | Dedicated fixed-plan linear driver demand/slew candidate matrix, provenance, and accepted-trace metric contracts. |
| `test_sim_gui_forward.py` | Current GUI forward-driver settings, immutable identity, child dispatch, RunRecord-backed completion gate, and workflow-specific Qt visibility. |
| `test_sim_gui_case.py` | GUI case adapter dispatch, point-mass boundaries, and retired workflow rejection. |

Test modules named `test_<topic>.py` each cover one topic and run under `unittest`. `test_gui_launchers.py` exercises direct venv launch from unrelated working directories. Maneuver result tests verify canonical RunRecord handoff to animator readers; video encoding is not a maneuver responsibility.

## Subdirectories

| Directory | Purpose |
|---|---|
| `fixtures/` | Data files that tests read. |
