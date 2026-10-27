# Task 6 — Bind optimized replay geometry and report realized controls

## Implementation

- Added `yaw_replay_metrics.py` with strict one-dimensional, matching-shape, finite-value validation and strictly increasing time checks. RMS uses the specified explicit trapezoidal integral; boolean saturation fractions use trapezoidal integration over elapsed time.
- Replay consumes the supplied saved `YawPath`, `SpeedPlan`, and geometry arrays directly to construct `PathCoordinates` and `PlanarForwardPlant`; it does not call either the line optimizer or fixed-profile builder. Replay corridor evaluation reconstructs driven XY from selected path geometry and lateral displacement, then performs bounded, station-local forward-monotone projection to the reference curve. This replaces the selected-offset-plus-lateral-error approximation and accounts for differing selected/reference normals.
- Summary contains maximum absolute speed error, distinct requested and realized throttle/brake/steering-rate saturation fractions, plan-array/map/controller digests, and solver-validation completion. Realized metrics come from accepted command channels in the saved run record; requested values are separately sampled from `driver.command` and never stand in for realized channels.
- Saved artifacts validate increasing physical stations and preserve supplied nonzero-offset geometry arrays without forcing them to match the reference curve. Closed optimized-line curvature and curvature-rate seam checks use the path's own saved arrays, not centerline values.

## Gate tests

- Replay binding stub uses nonzero-offset XY and physical stations distinct from reference stations. It patches line optimization and fixed-profile generation to raise, then checks the supplied speed and geometry arrays reach the plant and record sink exactly.
- Metrics tests assert the irregular-time trapezoidal RMS formula, rejection of unequal/nonfinite/non-increasing inputs, and boolean fraction integration.
- Mapping/projection tests cover nonuniform physical/reference station mapping, roundtrip and periodic start/end wrapping, signed left/right displacement, a 30-degree selected/reference normal difference, and station-local hairpin branch selection rather than unrestricted global nearest point.
- Record-path and artifact tests check optimized path curvature and curvature-rate at the periodic seam and persistence of nonzero-offset periodic saved geometry.

## Validation

Required focused suite run under watchdog:

```text
PYTHONPATH=src:. python3 -m unittest tests.test_yaw_replay_binding tests.test_yaw_replay_metrics tests.test_yaw_lap_suite tests.test_yaw_lap_artifact tests.test_yaw_aware_record_path -q
Ran 18 tests in 15.396s
OK
```

No physical replay was run; replay tests use a stub.

## Commit/staging note

The worktree is shared with Task 4. The suite runner and artifact files include unrelated pre-existing Task 4 modifications. Exclude those from the Task 6 commit; stage only Task 6 hunks in those files plus Task 6-only paths/tests/report.
