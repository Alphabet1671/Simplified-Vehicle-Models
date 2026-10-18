# Task 11 report (round 2)

## Tests

Ran:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. python3 -m unittest tests.test_component_sweep tests.test_sim_gui_core -v
```

Final result: **47 tests, OK**. Also ran `PYTHONPATH=src:. python3 tools/model_data/generate_component_sweep.py --help` successfully.

## Failures and fixes

The first run found three issues in the newly written tests: the CSV-target test used an assembly base and therefore received the earlier leaf-base error; the compiled suspension exposes `roll_centre_height` rather than `rc_height`; and the reproducibility test omitted the required data unit (and incorrectly assumed two same-second timestamps differ). These were test-reading errors and were corrected.

The generator defect found while reviewing the binding label rules was that scale labels could append a unit; `_format_label` now emits scale labels without a unit for every scale, as required (`rc_height_front x1.1`).

## Files

- `tests/test_component_sweep.py` (new generator coverage)
- `tests/test_sim_gui_core.py` (generated-sweep dimension/expansion test)
- `tools/model_data/generate_component_sweep.py` (scale-label fix)
