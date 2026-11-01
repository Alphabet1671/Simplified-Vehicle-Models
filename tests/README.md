# tests

The unit and regression suite, run with `unittest`.

## Files

| File | Contents |
|---|---|
| `test_catalunya_ttc_formulation.py` | Exact TTC artifact preservation, steady/transient parameter partition, and planar 7-DoF assembly selection. |
| `test_continuous_driver_throttle_comparison.py` | Matched throttle-runner plan slicing, duration-weighted metrics, manifest/output contracts and arm isolation. |

Test modules named `test_<topic>.py` each cover one topic and run under `unittest`. `test_gui_launchers.py` exercises direct venv launch from unrelated working directories. Maneuver result tests verify canonical RunRecord handoff to animator readers; video encoding is not a maneuver responsibility.

## Subdirectories

| Directory | Purpose |
|---|---|
| `fixtures/` | Data files that tests read. |
