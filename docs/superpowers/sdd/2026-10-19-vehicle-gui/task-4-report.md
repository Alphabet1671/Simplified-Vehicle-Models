# Task 4 report: safe compile-check and transactional save

Implemented `tools/vehicle_gui/save.py` and `tests/test_vehicle_gui_save.py`.

## Behavior

- `render_files` renders faithful XML copies for edited/swapped files and ancestors.
- `check_files` writes and compiles all rendered files in a temporary scratch directory.
- `save_vehicle` validates before touching the destination, then commits the complete set through sibling staging and backups.
- Normal in-process commit failures attempt to remove moved outputs and restore every backup.
- Rollback actions are independent; if rollback is incomplete, the original commit error remains raised and carries a recovery-path note while staging/backup artifacts are retained.
- No abrupt-termination or power-loss recovery guarantee is made.
- `save_component` validates with the XML loader before writing.
- `models/` remains protected unless explicitly allowed.

## Verification

Focused command:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_vehicle_gui_save -v
```

Result: `Ran 20 tests in 0.559s ... OK` (20 vehicle/component tests, including all supplied fidelity, compiler, protection, component, and transaction cases).

Existing Task 2/3 regression command:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_vehicle_gui_xmltext tests.test_vehicle_gui_model -v
```

Result: `Ran 17 tests ... OK`.

The supplied 20 tests are present and passing, including render fidelity, nested chains, swaps, generated-copy behavior, compiler rejection, manifest reporting, overwrite/name/models protections, stale outputs, unchanged copies, all three save-component cases, staging-write failure, later commit failure with byte-for-byte restoration, and rollback-restore failure/recovery-path cases. No files under `models/` were modified.
