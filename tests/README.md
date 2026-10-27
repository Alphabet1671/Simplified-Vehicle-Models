# tests

The unit and regression suite, run with `unittest`.

## Files

Test modules named `test_<topic>.py` each cover one topic and run under `unittest`. `test_gui_launchers.py` exercises direct venv launch from unrelated working directories. Maneuver result tests verify canonical RunRecord handoff to animator readers; video encoding is not a maneuver responsibility.

## Subdirectories

| Directory | Purpose |
|---|---|
| `fixtures/` | Data files that tests read. |
