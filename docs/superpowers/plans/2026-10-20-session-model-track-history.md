# Session Model and Track History Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Start vehicle-model and track selectors empty and show only files opened in the current GUI session.

**Architecture:** Remove repository scans and automatic default-track selection from SingleRunTab and TrackTab. Keep their existing resolved-path deduplication and MainWindow synchronization. Explicitly open tracks in tests that previously depended on implicit defaults.

**Tech Stack:** Python, PyQt6, unittest.

**Spec:** Direct user request: remove all pre-populated model/track options and add only models opened during this session.

## Global Constraints

- Do not change component-swap catalogues or formulation-type options.
- Do not persist dropdown history across launches.
- Invalid browsed files must not enter history.
- Preserve model/track synchronization between tabs.

---

### Task 1: Session-only selectors

**Files:**
- Modify: `tools/sim_gui/qt/single_run_tab.py`
- Modify: `tools/sim_gui/qt/track_tab.py`
- Test: `tests/test_sim_gui_qt.py`

**Interfaces:**
- Consumes: existing `set_model(path)`, `set_track(path)`, browse handlers and resolved-path deduplication.
- Produces: empty initial selectors, session-only model/track choices.

- [x] Add assertions for empty selectors and no selected paths on construction:

```python
self.assertEqual(self.tab.model_combo.count(), 0)
self.assertEqual(self.tab.track_combo.count(), 0)
self.assertIsNone(self.tab.model_path())
self.assertIsNone(self.tab.track_path())
```

- [x] Replace the default-track test with empty-history assertions and add repeated-selection deduplication tests; verify a second widget has no prior history.
- [x] Run `QT_QPA_PLATFORM=offscreen PYTHONPATH=src:. .venv/bin/python -m unittest discover -s tests -p test_sim_gui_qt.py -q` and verify the new expectations fail before changing production code.
- [x] Remove the initial population loops and their unused catalogue/runner imports. Keep all existing selection handlers.
- [x] Update preview, single-run, study and synchronization tests to explicitly open their required tracks/models instead of relying on pre-population.
- [ ] Run the focused Qt tests again and investigate any failures. Review the diff for unrelated changes. Nine focused selector/guard/sync tests pass; full Qt suite pending.

No commit is required unless requested by the user.
