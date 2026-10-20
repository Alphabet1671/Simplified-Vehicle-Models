# Task 3 report: vehicle edit model

## Implementation

Implemented `tools/vehicle_gui/model.py` with the immutable Qt-free `VehicleEdit` and `ParameterRow` interfaces from the brief. The model uses the reviewed `xmltext` layer for parameter discovery/formatting, imports `VehicleConfiguration` and `generated_by` read-only, accepts only `vehicle-formulation` subtypes `planar-7dof` and `14dof-simple`, and rejects bare assemblies. Parameter edits and component swaps return new values; descendant edits are dropped when a parent slot is swapped. Added the verbatim focused tests in `tests/test_vehicle_gui_model.py`.

## Focused test

Command:

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_vehicle_gui_model -v
```

Output: `Ran 6 tests in 0.019s` / `OK`.

## Self-review

- Confirmed `changed`, `ParameterRow.value_text`, and `ParameterRow.numeric` are properties.
- Confirmed assembly key is `ASSEMBLY == ""` and appears first.
- Confirmed formulation-only subtype gate and no assembly fallback.
- Confirmed generated files refuse direct parameter edits.
- Confirmed `with_choice` delegates to the existing configuration API without broadening candidate scope.
- `git diff --check` passed.

## Commit

Commit hash: `177e82c` (amended after recording this report).

## Concerns

The required `frozen=True` dataclass interface still exposes the brief-mandated `parameters` dictionaries; callers should treat nested dictionaries as read-only. No functional concern found in the focused tests.
