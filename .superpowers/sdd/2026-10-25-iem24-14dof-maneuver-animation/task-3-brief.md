### Task 3: Export accepted maneuver evidence to RunRecord

**Files:**
- Create: `src/simplified_vehicle_models/results/maneuver_run_record.py`
- Test: `tests/test_maneuver_run_record.py`

**Interfaces:**
- Consumes: completed `SimulationResult`, `VerticalManeuverPlant.state_names`, accepted output channels, original definition and artifact manifest.
- Produces: `write_maneuver_run_record(path, definition, result, *, model_kind, source_manifest) -> Path`, containing `run-record-v1` accepted samples and provenance note `maneuver-artifact-sha256`.

- [ ] **Step 1: Write failing evidence-only conversion test**

Construct a small synthetic accepted 38-state result and channels. Convert it, load through `load_run_record`, and assert `time`, `state`, and controls exactly match supplied evidence. Patch plant/evaluator identifiers so any reevaluation would fail; exporter must not call physics.

- [ ] **Step 2: Verify failure**

Run: `PYTHONPATH=src:. python3 -m unittest tests.test_maneuver_run_record -v`

Expected: import failure.

- [ ] **Step 3: Implement RunRecorder conversion**

Use `RunRecorder` with vertical state names, units, canonical controls (`swa-rad`, `accelerator-travel`, `brake-travel`), and only channels from the standard catalogue that adapter accepted outputs supply. For every accepted sample, append stored state/rate/control/channel values. Include source definition and maneuver artifact hash in the manifest sources/notes; do not calculate any force or pose from state during conversion.

- [ ] **Step 4: Add animator compatibility test**

Load the output through `tools.anim.resample.FrameSource`; assert frame source time endpoints equal accepted evidence endpoints and position/speed channels are present. No ffmpeg invocation in this test.

- [ ] **Step 5: Run focused export tests**

Run: `PYTHONPATH=src:. python3 -m unittest tests.test_maneuver_run_record -v`

Expected: PASS.

- [x] **Step 6: Commit**

Completed across initial implementation and repair commits; round 2 repair recorded in task-3-report.md.

