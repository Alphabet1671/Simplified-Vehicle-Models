# Task 5 report

## Files

- `tools/vehicle_gui/qt/__init__.py` — empty Qt package initializer.
- `tools/vehicle_gui/qt/parameter_table.py` — `ParameterTableModel` with four columns, numeric-only value editing, finite-number validation, edit/refusal signals, generated-slot locking, edited-row highlighting, and lock tooltips.
- `tests/test_vehicle_gui_table.py` — focused coverage for numeric versus non-numeric value editability, direct rejection of locked/non-numeric edits, invalid/non-finite refusal signals, valid edit signals, and row-value preservation.

## Tests

Command:

```bash
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_vehicle_gui_table -v
```

Output:

```text
test_invalid_and_nonfinite_values_are_refused (tests.test_vehicle_gui_table.TableModelTests.test_invalid_and_nonfinite_values_are_refused) ... ok
test_locked_table_rejects_direct_edit (tests.test_vehicle_gui_table.TableModelTests.test_locked_table_rejects_direct_edit) ... ok
test_non_numeric_rows_are_not_editable (tests.test_vehicle_gui_table.TableModelTests.test_non_numeric_rows_are_not_editable) ... ok
test_valid_value_emits_edited_without_mutating_rows (tests.test_vehicle_gui_table.TableModelTests.test_valid_value_emits_edited_without_mutating_rows) ... ok

----------------------------------------------------------------------
Ran 4 tests in 0.018s

OK
```

## Self-review

Implementation follows the Task 5 interface and exact behavior: only the Value column is editable, non-numeric and locked rows are read-only both through Qt flags and direct `setData` calls, invalid or non-finite input emits `refused` without changing rows, and valid input emits `edited` without mutating the model row. Added safe bounds handling for headers and indexes.

## Commit

Initial commit: `408eca9` — `feat: add the parameter table model for the vehicle GUI`

Fix commit: pending (adds direct-edit guards, robustness checks, and expanded tests).
