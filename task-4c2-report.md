# Task 4c2 report — explicit cut-open reference view

Implemented the scoped full-course cut view and explicit source-fit metadata in the five authorized files. `TrackProfile.source_reference_interpolation_periodic` is appended as an optional typed field, retaining `None` defaults and legacy open-refit behavior. `ReferenceCurve` honors explicit metadata and rejects conflicting overrides. `cut_open_track` validates a closed signed-geometry full course and returns readonly snapshots with unchanged samples, an open query/boundary domain, and a periodic source interpolation. The QSS reference normalization path propagates this metadata when normalizing nested topology.

## Validation

- Red checkpoint: `PYTHONPATH=src python3 -m unittest tests.test_cut_open_reference_view -q` failed as expected before implementation (missing boundary view and metadata field).
- Green targeted checkpoint: pinned runtime Python `/Users/alphabet1671/Dev/Simplified-Vehicle-Models/runs/no_tv_reliability/runtime_install_20261023/venv/bin/python -m unittest tests.test_cut_open_reference_view -q` passed.
- Final related geometry/QSS/artifact regressions: pinned runtime Python `-m unittest tests.test_cut_open_reference_view tests.test_line_trajectory tests.test_trajectory_artifact tests.test_qss_constraint_stations tests.test_qss_track_boundaries tests.test_track_width -q` passed: 52 tests.

The new tests exercise the actual FSC closed source against the cut-view direct reference fit at 1001 stations, compare positions and first through third derivatives, metric, metric derivative, curvature and curvature derivative, check strict open-domain queries, immutable snapshots and source immutability, default legacy behavior, typed/conflicting metadata, and nested-topology normalization. The suite does not run a full OCP/NLP/ODE solve. No source XML/CSV, model, cache, validator, tolerance, TV, harness, or existing artifact files were changed. Existing harness/tool view migration and wider physical reruns remain outside this task.
